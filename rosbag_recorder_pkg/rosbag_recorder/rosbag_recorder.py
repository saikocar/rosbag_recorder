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

from .video_recorder import VideoRecorder,RawVideoSource

class TimedRosbagRecorder(Node):
    def __init__(self, config_path):
        super().__init__('timed_rosbag_recorder')
        self.load_config(config_path)

        self.recording = False
        self.should_record = False
        self.prev_should_record = False
        self.prev_control_state = AutowareState.INITIALIZING
        self.prev_mrm_state = MrmState.NORMAL
        self.last_hazard_status = None
        self.bag_process = None
        self.current_bag_path = None
        self.prev_bag_path = None
        self.current_video_path = None
        self.prev_video_path = None
        self.memo_phrase = ""
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
            current_video_path = self.current_video_path
            self.stop_bag()
            if not self.should_record and current_bag_path != None:
                self.get_logger().info(f'Discarding unmarked bag: {self.current_bag_path}')
                if self.prev_bag_path != None and not self.prev_should_record:
                    shutil.rmtree(os.path.dirname(self.prev_bag_path), ignore_errors=True)
                    #videoの消去
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
        dir_date = now.strftime('%y%m%d%H%M%S')
        dir_time = now.strftime('%m%d%H%M%S')
        full_dir = os.path.join(self.config['bag_output_dir'], dir_date, dir_time)
        print(full_dir)
        cmd = [
            'ros2', 'bag', 'record',
            '-o', full_dir, '--no-discovery'
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
        self.current_video_path = full_dir_video
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
        self.video.stop()

        self.recording = False
        self.current_bag_path = None
        self.current_video_path = None


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
