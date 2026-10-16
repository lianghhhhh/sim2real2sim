from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    rvizconfig = LaunchConfiguration("rvizconfig")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kros_car_description"), "urdf", "kros_car.xacro"]
        ),
        description="Absolute path to robot xacro file",
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Use joint_state_publisher_gui if true",
    )

    rviz_enable_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz if true",
    )

    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kros_car_description"), "rviz", "kros_car.rviz"]
        ),
        description="Absolute path to RViz config file",
    )

    robot_description = Command([FindExecutable(name="xacro"), " ", model])

    return LaunchDescription(
        [
            model_arg,
            gui_arg,
            rviz_enable_arg,
            rviz_arg,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(gui),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
                condition=UnlessCondition(gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rvizconfig],
                condition=IfCondition(rviz),
            ),
        ]
    )
