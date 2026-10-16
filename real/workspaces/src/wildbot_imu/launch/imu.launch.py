from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    frame_id = LaunchConfiguration("frame_id")

    imu_node = Node(
        package="wildbot_imu",
        executable="wildbot_imu_node",
        name="imu",
        output="screen",
        parameters=[
            {"port": port},
            {"baud": ParameterValue(baud, value_type=int)},
            {"frame_id": frame_id},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud", default_value="9600"),
            DeclareLaunchArgument("frame_id", default_value="imu_link"),
            imu_node,
        ]
    )
