import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import yaml
import subprocess
import os
import re
from datetime import datetime
import signal
import shutil
import sys

from autoware_auto_system_msgs.msg import AutowareState
from rviz_2d_overlay_msgs.msg import OverlayText
from nav_msgs.msg import Odometry

from .video_recorder import VideoRecorder,RawVideoSource


ON_LANELET_ID = 87984
OFF_LANELET_ID = 134130
# in_segment中に rotate_bag をスキップし続ける上限。超過時は failsafe で強制離脱。
ROTATE_SKIP_FAILSAFE_LIMIT = 30

class TimedRosbagRecorder(Node):
    def __init__(self, config_path):
        super().__init__('timed_rosbag_recorder')
        self.load_config(config_path)

        self.recording = False
        self.should_record = False
        self.prev_should_record = False
        self.prev_control_state = AutowareState.INITIALIZING
        self.in_segment = False
        self.rotate_skip_count = 0
        self.bag_process = None
        self.current_bag_path = None
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
        self.lanelet_info_sub = self.create_subscription(
            OverlayText,
            self.config.get('lanelet_info_topic', '/map/lanelet_param/current_lanelet_info_text'),
            self.lanelet_info_callback, 10)
        self.subscription = self.create_subscription(
            Odometry,'/localization/kinematic_state',self.kinematic_state_callback,10)        
        
        self.rotate_bag()
        self.timer = self.create_timer(self.config['interval_sec'], self.rotate_bag)
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
        self.memo_phrase+=f"[{datetime.now()}],{self.position.x},{self.position.y},{self.position.z},{msg}\n"
    def memo_treat(self):
        if self.current_bag_path:
            memo_path = os.path.join(self.current_bag_path, 'memo.txt')
            with open(memo_path, 'a') as f:
                f.write(f"{self.memo_phrase}")
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

    def lanelet_info_callback(self, msg: OverlayText):
        match = re.search(r'Current ID\s*:\s*(\d+)', msg.text)
        if not match:
            return
        lanelet_id = int(match.group(1))
        self.get_logger().info(f'{lanelet_id}')
        if lanelet_id == ON_LANELET_ID and not self.in_segment:
            self.in_segment = True
            self.rotate_skip_count = 0
            self._record_memo('enter segment')
            self.get_logger().info(f'in_segment -> True (lanelet {lanelet_id})')
        elif lanelet_id == OFF_LANELET_ID and self.in_segment:
            self.in_segment = False
            self._record_memo('leave segment')
            self.get_logger().info(f'in_segment -> False (lanelet {lanelet_id})')

    def memo_callback(self, msg: String):
        self._record_memo(msg.data)

    def rotate_bag(self):
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
            '-o', full_dir, #'--no-discovery',
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
            video_file1 = os.path.join(full_dir_video, 'screen_capture1.mp4')
            self.video1.start(self.video_source1, video_file1)
            video_file2 = os.path.join(full_dir_video, 'screen_capture2.mp4')
            self.video2.start(self.video_source2, video_file2)
            #video_file3 = os.path.join(full_dir_video, 'screen_capture3.mp4')
            #self.video3.start(self.video_source3, video_file3)
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
            self.video1.stop()
            self.video2.stop()
            #self.video3.stop()

        # 保存対象(マーク付き)のbagは、その区間のジャーナルもjournal_saveへ退避 (2026-07-14)
        # この時点ではshould_recordが未リセットのため、rotate/Ctrl+C/shutdownの全経路で
        # 「保存されるbagのときだけ」発火する
        #if self.should_record and self.current_bag_path:
        #    self._save_journal_async(self.current_bag_path)

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
