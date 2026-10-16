from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="wildbot_calibration",
            executable="run_all_calibrations",
            name="run_all_calibrations",
            output="screen",
            emulate_tty=True,
        ),
    ])
