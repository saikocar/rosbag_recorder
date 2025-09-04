from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('rosbag_recorder'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='rosbag_recorder',
            executable='rosbag_recorder',
            name='rosbag_recorder',
            output='screen',
            arguments=[config_path]  # ← 引数で渡す
        )
    ])

