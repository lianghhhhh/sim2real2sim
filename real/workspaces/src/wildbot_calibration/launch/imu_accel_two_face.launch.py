from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="wildbot_calibration",
            executable="imu_accel_two_face",
            name="imu_accel_two_face",
            output="screen",
            emulate_tty=True,
        ),
    ])
