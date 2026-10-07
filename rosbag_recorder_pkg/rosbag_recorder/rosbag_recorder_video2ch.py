import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import yaml
import subprocess
import os
import re
import glob
from datetime import datetime
import signal
import shutil
import sys

from autoware_auto_system_msgs.msg import AutowareState
from rviz_2d_overlay_msgs.msg import OverlayText
from nav_msgs.msg import Odometry
from rosbag2_interfaces.msg import WriteSplitEvent

from .video_recorder import VideoRecorder,RawVideoSource


ON_LANELET_ID = 87984
OFF_LANELET_ID = 134130
# in_segment のまま分割窓が続いた場合の上限 (30分相当)。超過時は failsafe で強制離脱。
SEGMENT_FAILSAFE_LIMIT = 30


class TimedRosbagRecorder(Node):
    """常駐 ros2 bag record + ネイティブ分割方式のレコーダー。

    旧実装 (rosbag_recorder_video2ch_legacy.py) は interval_sec ごとに
    ros2 bag record を kill→再起動しており、毎分の全トピック再discovery・
    再subscribeとキャッシュflushで /perception 系の出力が瞬断し、
    component_state_monitor の 1Hz 割れ→MRM の原因になっていた (2026-08-09調査)。

    本実装では record プロセスを1本だけ常駐させ、rosbag2 ネイティブの
    --max-bag-duration 分割で約60秒ごとのファイルを作らせる。
    /events/write_split を受けて完成ファイルを従来の毎分ディレクトリ構成
    (<bag_output_dir>/<yymmddHHMMSS>/<mmddHHMMSS>/<mmddHHMMSS>_0.mcap)
    へ移動する。マーク無しバッグの破棄(1周期遅延)・memo.txt・動画録画の
    ロジックは旧実装と同じ。in_segment 中は「1本の長いバッグ」ではなく
    連続した毎分バッグ群としてすべて保存される。
    """

    def __init__(self, config_path):
        super().__init__('timed_rosbag_recorder')
        self.load_config(config_path)

        self.recording = False
        self.should_record = False
        self.prev_should_record = False
        self.prev_control_state = AutowareState.INITIALIZING
        self.in_segment = False
        self.segment_window_count = 0
        self.bag_process = None
        self.spool_dir = None
        self.window_start = None
        self.shutting_down = False
        self.start_fail_count = 0
        self.current_bag_path = None  # 窓の確定処理中のみ実ディレクトリを指す
        # 消去なしモード(2026-09-23 追加、rosbag_recorder.py と同じ): 合図(自動運転・MRM・メモ)が無い bag も消さずに全部残す。
        #   config.yaml の keep_all_bags: true、または keep_all_flag_file(既定 /tmp/rosbag_recorder_keep_all)が存在する間だけ有効。
        #   フラグファイルは窓ごとに見るので、走行中に touch / rm で切り替えられる。
        self.keep_all_bags = bool(self.config.get('keep_all_bags', False))
        self.keep_all_flag_file = os.path.expanduser(
            str(self.config.get('keep_all_flag_file', '/tmp/rosbag_recorder_keep_all')))
        self.get_logger().info(
            f'keep_all_bags={self.keep_all_bags} keep_all_flag_file={self.keep_all_flag_file}')
        self.prev_bag_path = None
        self.current_video_path = None
        self.prev_video_path = None
        self.memo_phrase = ""
        self.position = None
        # config.yamlのvideo_enabled: falseでffmpegによる動画録画を無効化できる(既定: 有効)
        self.video_enabled = bool(self.config.get('video_enabled', True))
        self.video1 = VideoRecorder()
        self.video_source1=RawVideoSource('/dev/v4l/by-id/usb-MACROSILICON_C7_USB3.0_Video_19323064-video-index0', 'v4l2', 'mjpeg', (1920, 1080), 30),
        self.video2 = VideoRecorder()
        self.video_source2=RawVideoSource('/dev/v4l/by-id/usb-MACROSILICON_C7_USB3.0_Video_41475953-video-index0', 'v4l2', 'mjpeg', (1920, 1080), 30),
        #self.video3 = VideoRecorder()
        #self.video_source3=RawVideoSource('/dev/v4l/by-id/usb-MACROSILICON_C7_USB3.0_Video_45720291-video-index0', 'v4l2', 'mjpeg', (1920, 1080), 30),

        self.control_sub = self.create_subscription(
            AutowareState, self.config['control_topic'], self.control_callback, 10)
        self.memo_sub = self.create_subscription(
            String, self.config['memo_topic'], self.memo_callback, 10)
        # 会話・警告音声の文字起こし(ava1 の audio_transcriber、1 発話 1 行)。窓の締めで dialog.txt に書く(2026-10-07)
        self.dialog_lines = []
        self.transcript_sub = self.create_subscription(
            String, self.config.get('transcript_topic', '/audio/transcript'), self.transcript_callback, 100)
        self.lanelet_info_sub = self.create_subscription(
            OverlayText,
            self.config.get('lanelet_info_topic', '/map/lanelet_param/current_lanelet_info_text'),
            self.lanelet_info_callback, 10)
        self.subscription = self.create_subscription(
            Odometry,'/localization/kinematic_state',self.kinematic_state_callback,10)
        # 常駐recorderの分割完了イベント
        self.split_sub = self.create_subscription(
            WriteSplitEvent, '/events/write_split', self.split_callback, 10)

        self.start_recorder()
        # 常駐プロセスの死活監視 (旧実装の毎分再起動に代わる保険)
        self.watchdog_timer = self.create_timer(5.0, self.watchdog)
        self.get_logger().info(f'ON_LANELET_ID: {ON_LANELET_ID} ,OFF_LANELET_ID: {OFF_LANELET_ID}')

    def load_config(self, path):
        with open(path, 'r') as f:
            self.config = yaml.safe_load(f)

    def kinematic_state_callback(self, msg: Odometry):
        # 位置と姿勢をログに出力
        self.position = msg.pose.pose.position

    def control_callback(self, msg):
        if self.prev_control_state == AutowareState.DRIVING and not msg.state == AutowareState.DRIVING:
            self.memo_concat('AutoDrive disengage')
            self.should_record = True
        self.prev_control_state = msg.state

    def memo_concat(self,msg: str):
        # 位置未受信(起動直後など)でもメモでクラッシュしないようにする
        if self.position is not None:
            pos = f"{self.position.x},{self.position.y},{self.position.z}"
        else:
            pos = "nan,nan,nan"
        self.memo_phrase+=f"[{datetime.now()}],{pos},{msg}\n"
    def memo_treat(self):
        if self.current_bag_path:
            memo_path = os.path.join(self.current_bag_path, 'memo.txt')
            with open(memo_path, 'a') as f:
                f.write(f"{self.memo_phrase}")
    def previous_memo_treat(self):
        if self.prev_bag_path:
            memo_path = os.path.join(self.prev_bag_path, 'memo.txt')
            with open(memo_path, 'a') as f:
                f.write(f"[{datetime.now()}] Memo entry queued for next bag file.\n")

    def _record_memo(self, text: str):
        """memo_callback と同じ流れ（記録フラグON＋メモ追記＋前バッグへの通知）。"""
        self.should_record = True
        self.memo_concat(text)
        self.previous_memo_treat()

    def lanelet_info_callback(self, msg: OverlayText):
        match = re.search(r'Current ID\s*:\s*(\d+)', msg.text)
        if not match:
            return
        lanelet_id = int(match.group(1))
        self.get_logger().debug(f'{lanelet_id}')
        if lanelet_id == ON_LANELET_ID and not self.in_segment:
            self.in_segment = True
            self.segment_window_count = 0
            self._record_memo('enter segment')
            self.get_logger().info(f'in_segment -> True (lanelet {lanelet_id})')
        elif lanelet_id == OFF_LANELET_ID and self.in_segment:
            self.in_segment = False
            self._record_memo('leave segment')
            self.get_logger().info(f'in_segment -> False (lanelet {lanelet_id})')

    def memo_callback(self, msg: String):
        self._record_memo(msg.data)

    def transcript_callback(self, msg: String):
        # 会話は記録の合図にしない(should_record は触らない)。合図の無い bag が消えるときは dialog.txt も一緒に消える。
        # ava1 側の控え(~/audio_transcriber/log/YYYY-MM-DD.txt)は残る
        self.dialog_lines.append(msg.data.rstrip('\n') + '\n')

    def dialog_treat(self, bag_dir):
        """窓の間に届いた文字起こしの行を <bag_dir>/dialog.txt に書く(memo.txt と同じく窓の締めで)。
        行頭の時刻は発話の始まりなので、窓の境目をまたいだ発話は次の窓の dialog.txt に入ることがある。"""
        if not self.dialog_lines:
            return
        lines, self.dialog_lines = self.dialog_lines, []
        try:
            with open(os.path.join(bag_dir, 'dialog.txt'), 'a') as f:
                f.writelines(lines)
        except OSError as e:
            self.get_logger().warn(f'dialog.txt write failed ({bag_dir}): {e}')

    # ---- 常駐recorder管理 -------------------------------------------------

    def start_recorder(self):
        now = datetime.now()
        spool_root = os.path.join(self.config['bag_output_dir'], '.spool')
        try:
            os.makedirs(spool_root, exist_ok=True)
        except OSError as e:
            # 外付けディスク未接続など。クラッシュせず watchdog で5秒ごとに再試行する
            self.start_fail_count += 1
            if self.start_fail_count == 1 or self.start_fail_count % 12 == 0:
                self.get_logger().error(
                    f'bag output dir not available ({e}); retrying every 5s')
            self.spool_dir = None
            self.bag_process = None
            self.recording = False
            return
        if self.start_fail_count:
            self.get_logger().info(
                f'bag output dir became available after {self.start_fail_count} retries')
            self.start_fail_count = 0
        self.bag_start_time = now  # journal退避用にbag開始時刻を保持
        self.window_start = now
        self.spool_dir = os.path.join(spool_root, now.strftime('%y%m%d%H%M%S'))
        cmd = [
            'ros2', 'bag', 'record',
            '-o', self.spool_dir,
            '--storage', 'mcap',  # sqlite3より書き込み負荷が軽い (2026-07-17)
            '--max-cache-size', '268435456',  # 256MiB: フラッシュ頻度を下げてCPUバースト緩和
            '-d', str(self.config['interval_sec']),  # ネイティブ分割 (プロセスは常駐)
        ] + self.config['record_topics']
        self.get_logger().info(f'Starting resident rosbag: {" ".join(cmd)}')
        self.bag_process = subprocess.Popen(cmd)
        self.recording = True
        self.start_video_window(now)

    def watchdog(self):
        if self.shutting_down:
            return
        if self.bag_process is None:
            # 起動失敗中(出力先未接続など)のリトライ
            self.start_recorder()
            return
        rc = self.bag_process.poll()
        if rc is None:
            return
        self.get_logger().error(
            f'rosbag process died unexpectedly (exit={rc}); '
            f'leftover spool: {self.spool_dir}; restarting')
        self.bag_process = None
        if self.video_enabled:
            self.video1.stop()
            self.video2.stop()
        self.recording = False
        self.start_recorder()

    def start_video_window(self, t):
        if not self.video_enabled:
            self.current_video_path = None
            return
        dir_date = t.strftime('%y%m%d%H%M%S')
        dir_time = t.strftime('%m%d%H%M%S')
        full_dir_video = os.path.join(self.config['bag_output_dir'], "video", dir_date, dir_time)
        os.makedirs(full_dir_video, exist_ok=True)
        video_file1 = os.path.join(full_dir_video, 'screen_capture1.mp4')
        self.video1.start(self.video_source1, video_file1)
        video_file2 = os.path.join(full_dir_video, 'screen_capture2.mp4')
        self.video2.start(self.video_source2, video_file2)
        #video_file3 = os.path.join(full_dir_video, 'screen_capture3.mp4')
        #self.video3.start(self.video_source3, video_file3)
        self.current_video_path = full_dir_video

    # ---- 分割窓の確定 -----------------------------------------------------

    def split_callback(self, msg: WriteSplitEvent):
        if self.shutting_down:
            return
        # 自分の常駐recorder以外の分割イベントは無視
        if self.spool_dir and not msg.closed_file.startswith(self.spool_dir):
            return
        self.finalize_window(msg.closed_file, final=False)

    def finalize_window(self, closed_file, final):
        """完成した分割ファイル1本を従来の毎分レイアウトへ確定する。

        旧実装の rotate_bag/stop_bag の窓締め処理に相当。final=True は
        shutdown経路で、旧実装同様 discard 判定を通さず常に保存する。
        戻り値は確定先ディレクトリ。
        """
        win_start = self.window_start or datetime.now()
        now = datetime.now()
        self.window_start = now

        # in_segment 中の窓はすべて保存対象 (旧: 1本の長いマーク付きバッグ相当)
        if self.in_segment:
            self.should_record = True
            self.segment_window_count += 1
            if self.segment_window_count >= SEGMENT_FAILSAFE_LIMIT:
                self.get_logger().warn(
                    f'in_segment continued for {self.segment_window_count} windows; forcing leave segment')
                self.in_segment = False
                self.segment_window_count = 0
                self._record_memo('leave segment(failsafe)')

        dir_date = win_start.strftime('%y%m%d%H%M%S')
        dir_time = win_start.strftime('%m%d%H%M%S')
        dest = os.path.join(self.config['bag_output_dir'], dir_date, dir_time)
        os.makedirs(dest, exist_ok=True)
        if closed_file and os.path.isfile(closed_file):
            shutil.move(closed_file, os.path.join(dest, f'{dir_time}_0.mcap'))
        else:
            self.get_logger().warn(f'closed bag file not found: {closed_file}')

        # 消去なしモード: この窓を合図の有無によらず残す(memo にも残す。下の memo_treat で書かれる)
        if self._keep_all_active() and not self.should_record:
            self.should_record = True
            self.memo_concat('keep_all mode (bag preserved without trigger)')

        # memo は旧実装の stop_bag と同じく窓の締めで書く
        self.current_bag_path = dest
        self.memo_treat()
        self.memo_phrase = ""
        self.current_bag_path = None
        self.dialog_treat(dest)

        # 動画は窓ごとに停止し、(最終窓以外は)次の窓を開始する
        current_video_path = self.current_video_path
        if self.video_enabled:
            self.video1.stop()
            self.video2.stop()
            #self.video3.stop()

        if final:
            # 旧 shutdown 経路と同じく最終バッグは discard 判定を通さず常に残す
            self.current_video_path = None
            self.get_logger().info(f'Final bag preserved: {dest}')
            self._reindex_async(dest)
            return dest

        # マーク無しバッグの破棄 (旧 rotate_bag と同一の1周期遅延ロジック:
        # 未マークの窓は「次の窓も未マーク」だったときに初めて消える)
        if not self.should_record:
            self.get_logger().info(f'Unmarked bag (kept as pre-context for now): {dest}')
            if self.prev_bag_path != None and not self.prev_should_record:
                self.get_logger().info(f'Discarding unmarked bag: {self.prev_bag_path}')
                shutil.rmtree(os.path.dirname(self.prev_bag_path), ignore_errors=True)
                #videoの消去(video_enabled=False時はprev_video_pathがNone)
                if self.prev_video_path:
                    shutil.rmtree(os.path.dirname(self.prev_video_path), ignore_errors=True)
        else:
            self.get_logger().info(f'Preserved bag: {dest}')
            self._reindex_async(dest)
            if self.prev_bag_path != None and not self.prev_should_record:
                # 未マークだが直前窓(pre-context)として生き残る分にも metadata を付ける
                self._reindex_async(self.prev_bag_path)

        # 判定フラグをリセット（次の窓用）
        self.prev_should_record = self.should_record
        self.should_record = False
        self.prev_bag_path = dest
        self.prev_video_path = current_video_path

        self.start_video_window(now)
        return dest

    def _keep_all_active(self) -> bool:
        """消去なしモードが有効か(config の keep_all_bags、またはフラグファイルの存在)。"""
        return self.keep_all_bags or os.path.exists(self.keep_all_flag_file)

    def _reindex_async(self, bag_dir):
        """従来レイアウト互換の metadata.yaml を低優先度で生成 (ros2 bag play <dir> 用)"""
        try:
            subprocess.Popen(
                ['nice', '-n', '19', 'ionice', '-c', '3',
                 'ros2', 'bag', 'reindex', '-s', 'mcap', bag_dir],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().warn(f'reindex spawn failed: {e}')

    def stop_bag(self):
        if self.shutting_down:
            return
        self.shutting_down = True
        if self.bag_process:
            self.get_logger().info('Stopping rosbag...')
            self.bag_process.send_signal(signal.SIGINT)
            self.bag_process.wait()
            self.bag_process = None

        # 一度も録画を開始できていなければ確定処理は不要
        if self.spool_dir is None:
            self.recording = False
            return

        # SIGINTで閉じられた最終ファイル(と未処理の分割ファイル)を従来レイアウトへ移す
        last_file = None
        extras = []
        if self.spool_dir and os.path.isdir(self.spool_dir):
            files = sorted(glob.glob(os.path.join(self.spool_dir, '*.mcap')),
                           key=os.path.getmtime)
            if files:
                last_file = files[-1]
                extras = files[:-1]
        dest = self.finalize_window(last_file, final=True)
        for f in extras:
            # 未処理の分割イベント分。データを失わないよう最終窓のディレクトリへ退避
            self.get_logger().warn(f'moving unprocessed split file into final bag dir: {f}')
            shutil.move(f, os.path.join(dest, os.path.basename(f)))
        # spoolを掃除 (metadata.yaml はspool相対の内容なので破棄)
        if self.spool_dir and os.path.isdir(self.spool_dir):
            shutil.rmtree(self.spool_dir, ignore_errors=True)
        self.recording = False

    def _save_journal_async(self, bag_path):
        """bag記録区間の4台ぶんジャーナルを退避する(非同期・記録ループを妨げない)"""
        start_time = getattr(self, 'bag_start_time', None)
        if start_time is None:
            return
        try:
            label = os.path.basename(os.path.dirname(bag_path))  # 例: 260714161431
            subprocess.Popen(
                ['/home/sit/journal_save/save_journal_window.sh',
                 start_time.strftime('%Y-%m-%d %H:%M:%S'), label],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'journal save started for bag: {label}')
        except Exception as e:
            self.get_logger().warn(f'journal save spawn failed: {e}')

    def shutdown(self):
        self.stop_bag()


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    else:
        config_path = 'config.yaml'
    recorder = TimedRosbagRecorder(config_path)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Shutting down...')
        recorder.stop_bag();
    finally:
        recorder.shutdown()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
