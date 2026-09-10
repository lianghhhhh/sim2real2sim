# my_launch.launch.py
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # collect data & control car 
        Node(
            package='calibrate_env_pkg',
            executable='calibrate_env_node',
            output='screen',
            on_exit=Shutdown(reason='測試腳本跑完'),
        ),
    ])
