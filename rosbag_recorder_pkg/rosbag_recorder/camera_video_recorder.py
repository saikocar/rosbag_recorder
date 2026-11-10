"""カメラの JPEG(sensor_msgs/CompressedImage)を受けて、GPU(NVENC)の H.265 で 1 分ごとの mkv に書く(2026-11-10)。

bag に JPEG のまま入れる(5 台で毎秒 約 17 MB)代わりに、動画にして軽くする。後で同じトピック名・同じ時刻の bag に戻す道具は
dw-notes `sensing/tools/camera_video/video_to_bag.py`。動画にすると画質は少し落ちる(元の JPEG とバイト列までは同じに戻らない)。

- rosbag の recorder(rosbag_recorder_video2ch)が、config.yaml の camera_record_mode が video / both のときに子として起こす。
  同じ config.yaml を読む。単独でも `camera_video_recorder <config.yaml> --ros-args -r __node:=camera_video_recorder` で動く。
- 置き場: <video_dir>/<YYMMDD>/<カメラ>_<最初のコマの時刻 YYMMDDHHMMSS>.mkv と、同じ名前の .stamps.tsv
  (コマごとに 番号・header.stamp[ns]・受けた時刻[ns]・frame_id。最初の行はトピック名)。書いている間は .part が付く。
  分の切り目は header.stamp(撮像の時刻)で切る。mkv の中の時刻は、そのファイルの最初のコマからの差(ms 単位)。正しい時刻は .stamps.tsv。
- 記録用のディスクが付いていない・目印のファイルが無い・空きが少ないときは書かない(ログに 1 回出す)。
- bag の記録先に bag が残っていない時間帯の動画は、grace 秒たってから消す(Ouster の pcap と同じ。bag が 1 本も無いときは消さない)。
- 分の切り目では、前のファイルを閉じきってから次を開く(符号化器を同時に開く数を台数までに抑える)。
- 書く手(GStreamer)が詰まって、まだ書いていない量が上限を超えたら、そのコマを捨てて数える(1 分ごとにログ)。
- SIGINT / SIGTERM で、書いているファイルを閉じてから抜ける。親(recorder)がいなくなったら同じように閉じて抜ける。
"""
import datetime
import os
import re
import shutil
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
import yaml

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst  # noqa: E402

try:
    from .version_info import get_version as version_string
except Exception:  # noqa: BLE001
    def version_string():
        return 'unknown'


def cam_name(topic):
    # /sensing/camera/roscube/front_wide/image_rect/compressed → front_wide
    parts = [p for p in topic.split('/') if p]
    return parts[3] if len(parts) > 3 else re.sub(r'[^A-Za-z0-9]+', '_', topic).strip('_')


class Writer:
    """1 台・1 ファイル分の GStreamer の流れ。"""

    def __init__(self, node, cam, topic, first_stamp_ns, out_dir, fmt):
        self.node = node
        self.cam = cam
        name = datetime.datetime.fromtimestamp(first_stamp_ns / 1e9).strftime('%y%m%d%H%M%S')
        self.final = os.path.join(out_dir, f'{cam}_{name}.mkv')
        n = 1
        while os.path.exists(self.final) or os.path.exists(self.final + '.part'):
            self.final = os.path.join(out_dir, f'{cam}_{name}_{n}.mkv')
            n += 1
        self.part = self.final + '.part'
        self.tsv_final = self.final[:-4] + '.stamps.tsv'
        self.tsv_part = self.tsv_final + '.part'
        self.t0 = first_stamp_ns
        self.count = 0
        self.closing_since = None
        c = node.cfg
        enc = {'h265': 'nvh265enc', 'h264': 'nvh264enc'}[c['codec']]
        parse = {'h265': 'h265parse', 'h264': 'h264parse'}[c['codec']]
        desc = (f'appsrc name=src is-live=false format=time do-timestamp=false block=false max-bytes={c["queue_mb"] << 20} '
                f'caps=image/jpeg,framerate=0/1 ! jpegdec ! videoconvert ! video/x-raw,format=I420 '
                f'! {enc} rc-mode=constqp qp-const={c["qp"]} gop-size={c["gop"]} preset=hq '
                f'! {parse} ! matroskamux ! filesink location="{self.part}" sync=false async=false')
        self.pipe = Gst.parse_launch(desc)
        self.src = self.pipe.get_by_name('src')
        self.bus = self.pipe.get_bus()
        self.tsv = open(self.tsv_part, 'w')
        self.tsv.write(f'# topic\t{topic}\tformat\t{fmt}\tcodec\t{c["codec"]}\tqp\t{c["qp"]}\tversion\t{version_string()}\n')
        self.tsv.write('index\tstamp_ns\trecv_ns\tframe_id\n')
        self.pipe.set_state(Gst.State.PLAYING)

    def queued_bytes(self):
        return self.src.get_property('current-level-bytes')

    def push(self, msg, stamp_ns, recv_ns):
        buf = Gst.Buffer.new_wrapped(bytes(msg.data))
        buf.pts = max(0, stamp_ns - self.t0)
        ret = self.src.emit('push-buffer', buf)
        if ret != Gst.FlowReturn.OK:
            return False
        self.tsv.write(f'{self.count}\t{stamp_ns}\t{recv_ns}\t{msg.header.frame_id}\n')
        self.count += 1
        return True

    def close(self):
        if self.closing_since is None:
            self.closing_since = time.time()
            self.src.emit('end-of-stream')
            self.tsv.close()

    def poll(self, timeout_s=0.0):
        """閉じ終わったら True。"""
        m = self.bus.timed_pop_filtered(int(timeout_s * Gst.SECOND), Gst.MessageType.EOS | Gst.MessageType.ERROR)
        if m is not None and m.type == Gst.MessageType.ERROR:
            err, dbg = m.parse_error()
            self.node.get_logger().error(f'{self.cam}: GStreamer error {err.message} ({dbg}) file={self.part}')
            self.finish(ok=False)
            return True
        if m is not None and m.type == Gst.MessageType.EOS:
            self.finish(ok=True)
            return True
        if self.closing_since is not None and time.time() - self.closing_since > 30:
            self.node.get_logger().error(f'{self.cam}: 30 s で閉じきらない。.part のまま残す: {self.part}')
            self.finish(ok=False)
            return True
        return False

    def finish(self, ok):
        self.pipe.set_state(Gst.State.NULL)
        if not self.tsv.closed:
            self.tsv.close()
        if ok:
            os.replace(self.part, self.final)
            os.replace(self.tsv_part, self.tsv_final)


class CameraVideoRecorder(Node):
    def __init__(self, config_path):
        super().__init__('camera_video_recorder')
        with open(config_path) as f:
            conf = yaml.safe_load(f)

        def text(key, default):  # 空欄(YAML の null)は '' として読む(str(None) = 'None' にしない)
            v = conf.get(key, default)
            return '' if v is None else str(v)
        self.cfg = {
            'mode': text('camera_record_mode', 'bag').strip(),
            'topics': list(conf.get('camera_image_topics') or []),
            'dir': text('video_dir', '/media/sit/ROSBAG/video_save/'),
            'segment': int(conf.get('video_segment_sec', 60)),
            'codec': text('video_codec', 'h265'),
            'qp': int(conf.get('video_qp', 20)),
            'gop': int(conf.get('video_gop', 18)),
            'queue_mb': int(conf.get('video_queue_mb', 256)),
            'mount': text('video_mount', '/media/sit/ROSBAG'),
            'marker': text('video_marker', 'DISK_ID.txt'),
            'min_free_gb': float(conf.get('video_min_free_gb', 30)),
            'bag_root': text('bag_output_dir', ''),
            'grace': float(conf.get('video_prune_grace_sec', 1800)),
            'sub_depth': int(conf.get('video_sub_depth', 90)),
        }
        self.get_logger().info(f'version {version_string()} config {config_path} {self.cfg}')
        if self.cfg['mode'] not in ('video', 'both'):
            self.get_logger().warn(f'camera_record_mode={self.cfg["mode"]}: 動画は録らない')
        if self.cfg['codec'] not in ('h265', 'h264'):
            raise SystemExit(f'video_codec は h265 / h264: {self.cfg["codec"]}')
        Gst.init(None)
        self.parent = os.getppid()
        self.cur = {}        # cam -> Writer
        self.closing = []    # 閉じている途中の Writer
        self.dropped = {}    # cam -> 捨てたコマの数(通算)
        self.frames = {}     # cam -> この 1 分に書いたコマの数
        self.skip_reason = None
        self.disk_checked = 0
        self.disk_why = None
        self.bag_end_cache = {}
        self.last_prune = 0
        self.stopping = False
        # 受け取りの待ち行列。分の切り目で 5 台の前のファイルを順に閉じきる間(実機で約 1.5 秒)、受け取りの処理が止まり、絵はここに溜まる。
        # 30 枚(18 Hz で約 1.66 秒)では余裕が 0.1 秒しかなく、起動の直後(最初の切り目)には溢れて 2〜3 秒分を落としていた(2026-11-10 nuvo)。
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=self.cfg['sub_depth'], reliability=ReliabilityPolicy.RELIABLE)
        if self.cfg['mode'] in ('video', 'both'):
            for t in self.cfg['topics']:
                cam = cam_name(t)
                self.dropped[cam] = 0
                self.frames[cam] = 0
                self.create_subscription(CompressedImage, t, lambda m, t=t, cam=cam: self.on_image(m, t, cam), qos)
        self.create_timer(1.0, self.tick)
        self.create_timer(60.0, self.report)

    # ---- 記録先の点検(10 秒ごとに見直す) ----
    def disk_ok(self):
        if time.time() - self.disk_checked < 10:
            return self.disk_why
        self.disk_checked = time.time()
        c = self.cfg
        why = None
        if c['mount'] and not os.path.ismount(c['mount']):
            why = f'{c["mount"]} がマウントされていない'
        elif c['mount'] and c['marker'] and not os.path.exists(os.path.join(c['mount'], c['marker'])):
            why = f'{c["mount"]}/{c["marker"]} が無い(記録用のディスクではない)'
        else:
            try:
                where = c['mount'] or c['dir']
                while where and not os.path.exists(where):   # まだ作っていないフォルダなら、ある所まで親をたどる
                    where = os.path.dirname(where.rstrip('/'))
                free = shutil.disk_usage(where or '/').free / 1e9
                if free < c['min_free_gb']:
                    why = f'空き {free:.0f} GB < {c["min_free_gb"]} GB'
            except OSError as e:
                why = f'空きが読めない: {e}'
        if why != self.skip_reason:
            if why:
                self.get_logger().warn(f'動画を書かない: {why}')
                for cam in list(self.cur):
                    self.close_cam(cam)
            elif self.skip_reason:
                self.get_logger().info('記録先が戻った。動画を書く')
            self.skip_reason = why
        self.disk_why = why
        return why

    def on_image(self, msg, topic, cam):
        if self.stopping:
            return
        if 'jpeg' not in msg.format.lower() and 'jpg' not in msg.format.lower():
            self.dropped[cam] += 1
            return
        if self.disk_ok():
            return
        recv = time.time_ns()
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        seg = self.cfg['segment'] * 10**9
        w = self.cur.get(cam)
        if w is not None and stamp // seg != w.t0 // seg:
            # 前のファイルを閉じきってから次を開く(最大 3 秒)。閉じきる前に開くと、切り目で符号化器が台数の 2 倍開き、
            # 家庭向けの GPU では同時に開ける上限(8)を超えて「Failed to open encoder」になった(2026-11-10 garage の試験)
            self.cur.pop(cam, None)
            w.close()
            t_end = time.time() + 3.0
            while not w.poll(0.1):
                if time.time() > t_end:
                    self.closing.append(w)
                    break
            w = None
        if w is None:
            d = os.path.join(self.cfg['dir'], datetime.datetime.fromtimestamp(stamp / 1e9).strftime('%y%m%d'))
            try:
                os.makedirs(d, exist_ok=True)
                w = self.cur[cam] = Writer(self, cam, topic, stamp, d, msg.format)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{cam}: 書き始められない: {e}')
                self.dropped[cam] += 1
                return
        if w.queued_bytes() > (self.cfg['queue_mb'] << 20) * 0.9 or not w.push(msg, stamp, recv):
            self.dropped[cam] += 1
            return
        self.frames[cam] += 1

    def close_cam(self, cam):
        w = self.cur.pop(cam, None)
        if w is not None:
            w.close()
            self.closing.append(w)

    def tick(self):
        self.closing = [w for w in self.closing if not w.poll(0)]
        for cam, w in list(self.cur.items()):
            if w.poll(0):            # 書いている途中のエラー
                self.cur.pop(cam, None)
        if os.getppid() != self.parent:
            self.get_logger().warn('親(recorder)がいなくなった。閉じて抜ける')
            self.stop_all()
            raise SystemExit(0)
        if time.time() - self.last_prune > 60:
            self.last_prune = time.time()
            try:
                self.prune()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'prune failed: {e}')

    def report(self):
        if self.cfg['mode'] not in ('video', 'both'):
            return
        s = '、'.join(f'{c} {self.frames[c]} 枚(捨てた通算 {self.dropped[c]})' for c in self.frames)
        q = max((w.queued_bytes() for w in self.cur.values()), default=0) / 1e6
        self.get_logger().info(f'この 1 分: {s}。待ち行列 {q:.0f} MB' + (f'。書いていない: {self.skip_reason}' if self.skip_reason else ''))
        for c in self.frames:
            self.frames[c] = 0

    def stop_all(self):
        self.stopping = True
        for cam in list(self.cur):
            self.close_cam(cam)
        t_end = time.time() + 35
        while self.closing and time.time() < t_end:
            self.closing = [w for w in self.closing if not w.poll(0.2)]

    # ---- bag が残っていない時間帯の動画を消す(pcap_recorder.py と同じ考え) ----
    def bag_intervals(self):
        root = self.cfg['bag_root']
        out = []
        for base in (root, os.path.join(root, '.spool')):
            try:
                names = os.listdir(base)
            except OSError:
                continue
            for n in names:
                if not re.fullmatch(r'\d{12}', n):
                    continue
                p = os.path.join(base, n)
                try:
                    st = datetime.datetime.strptime(n, '%y%m%d%H%M%S').timestamp()
                except ValueError:
                    continue
                if base != root:
                    out.append((st, time.time() + 3600))
                    continue
                end = self.bag_end_cache.get(p)
                if end is None:
                    end = st
                    for dp, _, fs in os.walk(p):
                        for f in fs:
                            try:
                                end = max(end, os.stat(os.path.join(dp, f)).st_mtime)
                            except OSError:
                                pass
                    if time.time() - end > 600:
                        self.bag_end_cache[p] = end
                out.append((st, end))
        return out

    def prune(self):
        c = self.cfg
        if not c['bag_root'] or not os.path.isdir(c['bag_root']) or not os.path.isdir(c['dir']):
            return
        iv = self.bag_intervals()
        if not iv:
            return
        now = time.time()
        for day in sorted(os.listdir(c['dir'])):
            d = os.path.join(c['dir'], day)
            if not os.path.isdir(d):
                continue
            for f in sorted(os.listdir(d)):
                m = re.fullmatch(r'(.+)_(\d{12})(_\d+)?\.mkv', f)
                if not m:
                    continue
                st = datetime.datetime.strptime(m.group(2), '%y%m%d%H%M%S').timestamp()
                en = st + c['segment']
                if now - en < c['grace']:
                    continue
                if any(bs - 10 <= en and st <= be + 10 for bs, be in iv):
                    continue
                for p in (os.path.join(d, f), os.path.join(d, f[:-4] + '.stamps.tsv')):
                    try:
                        sz = os.path.getsize(p)
                        os.remove(p)
                        with open(os.path.join(c['dir'], 'pruned.tsv'), 'a') as g:
                            g.write(f'{time.strftime("%F %T")}\t{day}/{os.path.basename(p)}\t{sz}\tbag なし\n')
                    except OSError:
                        pass


def main(args=None):
    rclpy.init(args=args)
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    config_path = argv[1] if len(argv) > 1 else 'config.yaml'
    node = CameraVideoRecorder(config_path)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
