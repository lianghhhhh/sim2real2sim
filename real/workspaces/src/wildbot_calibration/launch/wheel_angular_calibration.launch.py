from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="wildbot_calibration",
            executable="wheel_angular_calibration",
            name="wheel_angular_calibration",
            output="screen",
            emulate_tty=True,
        ),
    ])
