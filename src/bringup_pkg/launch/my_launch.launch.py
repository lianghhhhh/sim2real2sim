# my_launch.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 呼叫第一個 package 的 node
        Node(
            package='car_inference', 
            executable='car_inference_node'
        ),
        # 呼叫第二個 package 的 node
        Node(
            package='calibrate_env_pkg', 
            executable='calibrate_env_node'
        )
    ])