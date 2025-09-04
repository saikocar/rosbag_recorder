from setuptools import find_packages, setup

package_name = 'rosbag_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
        ('share/' + package_name + '/launch', ['launch/rosbag_recorder.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rosbag_recorder_video2ch.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 rosbag recorder with video integration',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'rosbag_recorder = rosbag_recorder.rosbag_recorder:main',
            'rosbag_recorder_video2ch = rosbag_recorder.rosbag_recorder_video2ch:main',
        ],
    },
)
