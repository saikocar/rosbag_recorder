import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import yaml
import subprocess
import os
from datetime import datetime
import signal
import shutil
import sys
import threading

from autoware_auto_system_msgs.msg import AutowareState
from autoware_adapi_v1_msgs.msg import MrmState
from tier4_system_msgs.msg import HazardStatus
from vehicle_std_msgs.msg import Uint8 as VehicleUint8

from .video_recorder import VideoRecorder,RawVideoSource
from .version_info import get_version

# in_segment中に rotate_bag をスキップし続ける上限。超過時は failsafe で強制離脱。
ROTATE_SKIP_FAILSAFE_LIMIT = 30

class TimedRosbagRecorder(Node):
    def __init__(self, config_path):
        super().__init__('timed_rosbag_recorder')
        self.version = get_version()
        self.get_logger().info(f'version {self.version}')
        self.load_config(config_path)

        self.recording = False
        self.should_record = False
        self.prev_should_record = False
        self.prev_control_state = AutowareState.INITIALIZING
        self.prev_mrm_state = MrmState.NORMAL
        self.last_hazard_status = None
        self.in_segment = False
        self.rotate_skip_count = 0
        self.prev_autonomy_level = 0
        self.bag_process = None
        self.current_bag_path = None
        self.prev_bag_path = None
        self.current_video_path = None
        self.prev_video_path = None
        self.memo_phrase = ""
        # config.yamlのvideo_enabled: falseでffmpegによる動画録画を無効化できる(既定: 有効)
        self.video_enabled = bool(self.config.get('video_enabled', True))
        # 消去なしモード(2026-09-23 追加): 合図(自動運転・MRM・メモ)が無い bag も消さずに全部残す。
        #   config.yaml の keep_all_bags: true、または keep_all_flag_file(既定 /tmp/rosbag_recorder_keep_all)が存在する間だけ有効。
        #   手動走行の全区間記録(事故対策の段階 3)など、合図の無い走行を丸ごと残したいときに使う。
        #   フラグファイルは周期ごとに見るので、走行中に touch / rm で切り替えられる。
        self.keep_all_bags = bool(self.config.get('keep_all_bags', False))
        self.keep_all_flag_file = os.path.expanduser(
            str(self.config.get('keep_all_flag_file', '/tmp/rosbag_recorder_keep_all')))
        self.get_logger().info(
            f'keep_all_bags={self.keep_all_bags} keep_all_flag_file={self.keep_all_flag_file}')
        self.video = VideoRecorder()
        self.video_source=RawVideoSource('/dev/v4l/by-id/usb-MACROSILICON_C7_USB3.0_Video_41475953-video-index0', 'v4l2', 'mjpeg', (1920, 1080), 30),

        self.control_sub = self.create_subscription(
            AutowareState, self.config['control_topic'], self.control_callback, 10)
        self.mrm_sub = self.create_subscription(
            MrmState, self.config.get('mrm_topic', '/system/fail_safe/mrm_state'),
            self.mrm_callback, 10)
        self.hazard_sub = self.create_subscription(
            HazardStatus, self.config.get('hazard_topic', '/system/emergency/hazard_status'),
            self.hazard_callback, 10)
        self.memo_sub = self.create_subscription(
            String, self.config['memo_topic'], self.memo_callback, 10)
        self.autonomy_level_sub = self.create_subscription(
            VehicleUint8,
            self.config.get('autonomy_level_topic', '/system/operational_design_domain/autonomy_level'),
            self.autonomy_level_callback, 10)
        self.rotate_bag()
        self.timer = self.create_timer(self.config['interval_sec'], self.rotate_bag)

    def load_config(self, path):
        with open(path, 'r') as f:
            self.config = yaml.safe_load(f)

    def control_callback(self, msg):
        if self.prev_control_state == AutowareState.DRIVING and not msg.state == AutowareState.DRIVING:
            self.memo_concat('AutoDrive disengage')
            self.should_record = True
        self.prev_control_state = msg.state

    def hazard_callback(self, msg):
        """Cache latest HazardStatus for use when MRM triggers."""
        self.last_hazard_status = msg

    def mrm_callback(self, msg):
        if msg.state != self.prev_mrm_state:
            state_names = {
                MrmState.NORMAL: 'NORMAL',
                MrmState.MRM_OPERATING: 'MRM_OPERATING',
                MrmState.MRM_SUCCEEDED: 'MRM_SUCCEEDED',
                MrmState.MRM_FAILED: 'MRM_FAILED',
            }
            behavior_names = {
                MrmState.NONE: 'NONE',
                MrmState.COMFORTABLE_STOP: 'COMFORTABLE_STOP',
                MrmState.EMERGENCY_STOP: 'EMERGENCY_STOP',
                MrmState.PULL_OVER: 'PULL_OVER',
            }
            state_str = state_names.get(msg.state, str(msg.state))
            behavior_str = behavior_names.get(msg.behavior, str(msg.behavior))

            if msg.state != MrmState.NORMAL:
                # Build MRM memo with HazardStatus SPF diagnostics
                memo_lines = [f'MRM {state_str} behavior={behavior_str}']
                if self.last_hazard_status:
                    level_names = {0: 'NF', 1: 'SF', 2: 'LF', 3: 'SPF'}
                    hs = self.last_hazard_status
                    memo_lines.append(f'  hazard_level={level_names.get(hs.level, str(hs.level))} emergency={hs.emergency}')
                    if hs.diagnostics_spf:
                        memo_lines.append('  SPF diagnostics:')
                        for diag in hs.diagnostics_spf:
                            memo_lines.append(f'    [{diag.name}] {diag.message}')
                    if hs.diagnostics_lf:
                        memo_lines.append('  LF diagnostics:')
                        for diag in hs.diagnostics_lf[:5]:  # limit to 5
                            memo_lines.append(f'    [{diag.name}] {diag.message}')

                self.memo_concat('\n'.join(memo_lines))
                self.should_record = True
                self.previous_memo_treat()
                # Save system logs in background to avoid blocking ROS callbacks
                threading.Thread(target=self._save_system_logs, daemon=True).start()
                self.get_logger().warn(f'MRM detected: {state_str} behavior={behavior_str}')

        self.prev_mrm_state = msg.state

    def _save_system_logs(self):
        """Save dmesg and journalctl logs when MRM occurs (local + remote hosts)."""
        if not self.current_bag_path:
            return
        try:
            log_dir = os.path.join(self.current_bag_path, 'system_logs')
            os.makedirs(log_dir, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')

            # Local host logs
            self._save_host_logs(log_dir, ts, 'local')

            # Remote host logs (roscube, sub PCs, etc.)
            remote_hosts = self.config.get('remote_log_hosts', [])
            for host in remote_hosts:
                self._save_host_logs(log_dir, ts, host)

            self.get_logger().info(f'System logs saved to {log_dir}')
        except Exception as e:
            self.get_logger().error(f'Failed to save system logs: {e}')

    def _save_host_logs(self, log_dir, ts, host):
        """Save dmesg and journalctl for a given host ('local' or ssh hostname)."""
        try:
            prefix = host if host != 'local' else 'local'

            if host == 'local':
                dmesg_cmd = ['dmesg', '--color=always', '--time-format=iso', '-T', '--since=-300']
                journal_cmd = ['journalctl', '-b', '0', '--since=-5min', '--no-pager', '-o', 'short-iso']
            else:
                dmesg_cmd = ['ssh', '-o', 'ConnectTimeout=3', host,
                             'dmesg --color=always --time-format=iso -T --since=-300']
                journal_cmd = ['ssh', '-o', 'ConnectTimeout=3', host,
                               'journalctl -b 0 --since=-5min --no-pager -o short-iso']

            dmesg_path = os.path.join(log_dir, f'dmesg_{prefix}_{ts}.txt')
            with open(dmesg_path, 'w') as f:
                subprocess.run(dmesg_cmd, stdout=f, stderr=subprocess.DEVNULL, timeout=10)

            journal_path = os.path.join(log_dir, f'journalctl_{prefix}_{ts}.txt')
            with open(journal_path, 'w') as f:
                subprocess.run(journal_cmd, stdout=f, stderr=subprocess.DEVNULL, timeout=10)

        except Exception as e:
            self.get_logger().warn(f'Failed to get logs from {host}: {e}')

    def memo_concat(self,msg: str):
        self.memo_phrase+=f"[{datetime.now()}] {msg}\n"
    def memo_treat(self):
        if self.current_bag_path:
            memo_path = os.path.join(self.current_bag_path, 'memo.txt')
            with open(memo_path, 'a') as f:
                f.write(f"[recorder version {self.version}]\n{self.memo_phrase}")
            #self.get_logger().info(f'Memo saved: {msg}')
        #else:
        #    self.get_logger().warn('Memo received but no current bag directory exists.')    
    def previous_memo_treat(self):
        if self.prev_bag_path:
            memo_path = os.path.join(self.prev_bag_path, 'memo.txt')
            with open(memo_path, 'a') as f:
                f.write(f"[{datetime.now()}] Memo entry queued for next bag file.\n")
            #self.get_logger().info(f'Memo saved: {msg}')
        #else:
        #    self.get_logger().warn('Memo received but no current bag directory exists.')    
    
    def _record_memo(self, text: str):
        """memo_callback と同じ流れ（記録フラグON＋メモ追記＋前バッグへの通知）。"""
        self.should_record = True
        self.memo_concat(text)
        self.previous_memo_treat()

    def autonomy_level_callback(self, msg: VehicleUint8):
        level = msg.data
        if level == 4 and not self.in_segment:
            self.in_segment = True
            self.rotate_skip_count = 0
            self._record_memo('enter segment (lv4)')
            self.get_logger().info(f'in_segment -> True (autonomy_level {level})')
        elif level != 4 and self.in_segment:
            self.in_segment = False
            self._record_memo(f'leave segment (lv4->lv{level})')
            self.get_logger().info(f'in_segment -> False (autonomy_level {level})')
        self.prev_autonomy_level = level

    def memo_callback(self, msg: String):
        self._record_memo(msg.data)

    def _keep_all_active(self) -> bool:
        """消去なしモードが有効か(config の keep_all_bags、またはフラグファイルの存在)。"""
        return self.keep_all_bags or os.path.exists(self.keep_all_flag_file)

    def rotate_bag(self):
        # 消去なしモード: この周期の bag を合図の有無によらず残す(memo にも残す)
        if self.recording and self._keep_all_active() and not self.should_record:
            self.should_record = True
            self.memo_concat('keep_all mode (bag preserved without trigger)')
        # in_segment中は1本の長いバッグとして残したいのでローテーションをスキップ
        if self.in_segment:
            self.rotate_skip_count += 1
            if self.rotate_skip_count >= ROTATE_SKIP_FAILSAFE_LIMIT:
                self.get_logger().warn(
                    f'rotate_bag skipped {self.rotate_skip_count} times; forcing leave segment')
                self.in_segment = False
                self.rotate_skip_count = 0
                self._record_memo('leave segment(failsafe)')
                # fall through してローテーション実行
            else:
                self.get_logger().info(
                    f'rotate_bag skipped (in_segment), count={self.rotate_skip_count}')
                return

        # 一度止める（前回の周期分）
        if self.recording:
            #stop_bagでcurrentのpathがNoneにされる前に保持する
            current_bag_path = self.current_bag_path
            current_video_path = self.current_video_path
            self.stop_bag()
            if not self.should_record and current_bag_path != None:
                self.get_logger().info(f'Discarding unmarked bag: {self.current_bag_path}')
                if self.prev_bag_path != None and not self.prev_should_record:
                    shutil.rmtree(os.path.dirname(self.prev_bag_path), ignore_errors=True)
                    #videoの消去(video_enabled=False時はprev_video_pathがNone)
                    if self.prev_video_path:
                        shutil.rmtree(os.path.dirname(self.prev_video_path), ignore_errors=True)
            else:
                self.get_logger().info(f'Preserved bag: {self.current_bag_path}')

            # 判定フラグをリセット（次の周期用）
            self.prev_should_record = self.should_record
            self.should_record = False
            self.prev_bag_path=current_bag_path
            self.prev_video_path=current_video_path

        # 新しい録画を開始
        self.start_bag()

    def start_bag(self):
        now = datetime.now()
        self.bag_start_time = now  # journal退避用にbag開始時刻を保持
        dir_date = now.strftime('%y%m%d%H%M%S')
        dir_time = now.strftime('%m%d%H%M%S')
        full_dir = os.path.join(self.config['bag_output_dir'], dir_date, dir_time)
        print(full_dir)
        cmd = [
            'ros2', 'bag', 'record',
            '-o', full_dir, '--no-discovery',
            '--storage', 'mcap',  # sqlite3より書き込み負荷が軽い (2026-07-17)
            '--max-cache-size', '268435456',  # 256MiB: フラッシュ頻度を下げてCPUバースト緩和
        ] + self.config['record_topics']
        print(cmd)

        self.get_logger().info(f'Starting rosbag: {" ".join(cmd)}')
        self.bag_process = subprocess.Popen(cmd)

        # ffmpeg コマンド
        if self.video_enabled:
            full_dir_video = os.path.join(self.config['bag_output_dir'],"video", dir_date, dir_time)
            # 混ぜたいけどvideo.startにrosbag recordでのディレクトリ作成が間に合わない
            os.makedirs(full_dir_video, exist_ok=True)
            video_file = os.path.join(full_dir_video, 'screen_capture.mp4')
            print(video_file)
            self.video.start(self.video_source, video_file)
            self.current_video_path = full_dir_video
        else:
            self.current_video_path = None

        self.current_bag_path = full_dir
        self.recording = True

    def stop_bag(self):
        if self.bag_process:
            self.get_logger().info('Stopping rosbag...')
            self.bag_process.send_signal(signal.SIGINT)
            self.bag_process.wait()
            self.bag_process = None
        self.memo_treat()
        self.memo_phrase = ""
        # 動画停止
        if self.video_enabled:
            self.video.stop()

        # 保存対象(マーク付き)のbagは、その区間のジャーナルもjournal_saveへ退避 (2026-07-14)
        # この時点ではshould_recordが未リセットのため、rotate/Ctrl+C/shutdownの全経路で
        # 「保存されるbagのときだけ」発火する
        if self.should_record and self.current_bag_path:
            self._save_journal_async(self.current_bag_path)

        self.recording = False
        self.current_bag_path = None
        self.current_video_path = None

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
