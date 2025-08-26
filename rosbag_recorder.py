import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import yaml
import subprocess
import os
from datetime import datetime
import signal
import shutil

from autoware_auto_system_msgs.msg import AutowareState

from video_recorder import VideoRecorder,RawVideoSource

class TimedRosbagRecorder(Node):
    def __init__(self, config_path):
        super().__init__('timed_rosbag_recorder')
        self.load_config(config_path)

        self.recording = False
        self.should_record = False
        self.prev_should_record = False
        self.prev_control_state = AutowareState.INITIALIZING
        self.bag_process = None
        self.current_bag_path = None
        self.prev_bag_path = None
        self.memo_phrase = ""
        self.video = VideoRecorder()
        self.video_source=RawVideoSource('/dev/video0', 'v4l2', 'mjpeg', (1920, 1080), 30),

        self.control_sub = self.create_subscription(
            AutowareState, self.config['control_topic'], self.control_callback, 10)
        self.memo_sub = self.create_subscription(
            String, self.config['memo_topic'], self.memo_callback, 10)
        self.rotate_bag()
        self.timer = self.create_timer(self.config['interval_sec'], self.rotate_bag)

    def load_config(self, path):
        with open(path, 'r') as f:
            self.config = yaml.safe_load(f)

    def control_callback(self, msg):
        if self.prev_control_state == AutowareState.DRIVING and not msg.state == AutowareState.DRIVING:
            self.memo_concat('AutoDrive cancel')
            self.should_record = True
        self.prev_control_state = msg.state

    def memo_concat(self,msg: str):
        self.memo_phrase+=f"[{datetime.now()}] {msg}\n"
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
    
    def memo_callback(self, msg: String):
        self.should_record = True
        self.memo_concat(msg.data)
        self.previous_memo_treat()

    def rotate_bag(self):
        # 一度止める（前回の周期分）
        if self.recording:
            #stop_bagでcurrentのpathがNoneにされる前に保持する
            current_bag_path = self.current_bag_path
            self.stop_bag()
            if not self.should_record and current_bag_path != None:
                self.get_logger().info(f'Discarding unmarked bag: {self.current_bag_path}')
                if self.prev_bag_path != None and not self.prev_should_record:
                    shutil.rmtree(os.path.dirname(self.prev_bag_path), ignore_errors=True)
            else:
                self.get_logger().info(f'Preserved bag: {self.current_bag_path}')

            # 判定フラグをリセット（次の周期用）
            self.prev_should_record = self.should_record
            self.should_record = False
            self.prev_bag_path=current_bag_path

        # 新しい録画を開始
        self.start_bag()

    def start_bag(self):
        now = datetime.now()
        dir_date = now.strftime('%y%m%d%H%M%S')
        dir_time = now.strftime('%m%d%H%M%S')
        full_dir = os.path.join(self.config['bag_output_dir'], dir_date, dir_time)
        print(full_dir)
        cmd = [
            'ros2', 'bag', 'record',
            '-o', full_dir
        ] + self.config['record_topics']
        print(cmd)

        self.get_logger().info(f'Starting rosbag: {" ".join(cmd)}')
        self.bag_process = subprocess.Popen(cmd)

        # ffmpeg コマンド
        full_dir_video = os.path.join(self.config['bag_output_dir'],"video", dir_date, dir_time)
        # 混ぜたいけどvideo.startにrosbag recordでのディレクトリ作成が間に合わない
        os.makedirs(full_dir_video, exist_ok=True)
        video_file = os.path.join(full_dir_video, 'screen_capture.mp4')        
        print(video_file)
        self.video.start(self.video_source, video_file)

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
        self.video.stop();

        self.recording = False
        self.current_bag_path = None


    def shutdown(self):
        self.stop_bag()


def main(args=None):
    rclpy.init(args=args)
    recorder = TimedRosbagRecorder(config_path='config.yaml')
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
