from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="wildbot_calibration",
            executable="imu_axis_sanity",
            name="imu_axis_sanity",
            output="screen",
            emulate_tty=True,
        ),
    ])
