import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import yaml

# User-override directory. Users (or Docker volumes) can place config files here
# to override the package defaults without modifying the installed package.
USER_CONFIG_DIR = "/configs"


# The two wheel controllers command the same joints, so only the one named by
# the `wheel_controller` launch argument starts active. Switch at runtime with
# the /controller_manager/switch_controller service.
WHEEL_CONTROLLERS = ("base_controller", "wheel_velocity_controller")


HARDWARE_ARGUMENT_DEFAULTS = {
    "wheel_port": "/dev/usb_wheel",
    "wheel_baudrate": "460800",
    "wheel_motor_ids": "1,2,3,4",
    "wheel_motor_directions": "1,1,-1,-1",
    "max_rpm": "100",
    "acceleration_act": "1",
    "wheel_command_rate_hz": "50",
    "wheel_heartbeat_ms": "500",
    "wheel_feedback_timeout_ms": "200",
    "wheel_torque_constant": "0.75",
    "wheel_require_feedback": "true",
    "wheel_joints": "front_left_joint,rear_left_joint,front_right_joint,rear_right_joint",
}


def _get_packaged_config_path(filename):
    try:
        package_share_dir = get_package_share_directory("kros_car")
    except PackageNotFoundError:
        return None

    config_path = os.path.join(package_share_dir, "config", filename)
    if os.path.isfile(config_path):
        return config_path
    return None


def _read_hardware_config(path):
    with open(path) as f:
        config = yaml.safe_load(f) or {}

    if not isinstance(config, dict):
        raise RuntimeError(f"Hardware config must be a YAML mapping: {path}")

    return config


def _load_hardware_defaults():
    """Return launch defaults with precedence: args > /configs > hardcoded."""
    defaults = dict(HARDWARE_ARGUMENT_DEFAULTS)

    user_path = os.path.join(USER_CONFIG_DIR, "hardware.yaml")
    if os.path.isfile(user_path):
        for key, value in _read_hardware_config(user_path).items():
            if key in defaults:
                defaults[key] = str(value)
    return defaults


def _launch_setup(context, *args, **kwargs):
    enable_wheel = LaunchConfiguration("enable_wheel").perform(context)
    wheel_controller = LaunchConfiguration("wheel_controller").perform(context)
    if wheel_controller not in WHEEL_CONTROLLERS:
        raise RuntimeError(
            f"wheel_controller must be one of {WHEEL_CONTROLLERS}, got '{wheel_controller}'"
        )
    model = LaunchConfiguration("model").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    xacro_mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in HARDWARE_ARGUMENT_DEFAULTS
    }

    robot_description_xml = xacro.process_file(model, mappings=xacro_mappings).toxml()

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_xml}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                {"robot_description": robot_description_xml},
                controllers_file,
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "120",
            ],
        ),
        *[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                condition=IfCondition(enable_wheel),
                arguments=[
                    name,
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "120",
                ]
                + ([] if name == wheel_controller else ["--inactive"]),
            )
            for name in WHEEL_CONTROLLERS
        ],
    ]

    return actions


def _resolve_default_config(filename):
    """Return config path with precedence: launch arg > /configs > packaged share."""
    user_path = os.path.join(USER_CONFIG_DIR, filename)
    if os.path.isfile(user_path):
        return user_path

    packaged_path = _get_packaged_config_path(filename)
    if packaged_path:
        return packaged_path

    return PathJoinSubstitution([FindPackageShare("kros_car"), "config", filename])


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument("enable_wheel", default_value="true"),
        DeclareLaunchArgument(
            "wheel_controller",
            default_value="base_controller",
            description="Wheel controller that starts active: base_controller (twist on "
            "/base_controller/cmd_vel) or wheel_velocity_controller (wheel speeds on "
            "/joint_command)",
        ),
        DeclareLaunchArgument(
            "model",
            default_value=PathJoinSubstitution(
                [FindPackageShare("kros_car_description"), "urdf", "kros_car.xacro"]
            ),
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=_resolve_default_config("controllers.yaml"),
        ),
    ]
    hardware_defaults = _load_hardware_defaults()
    launch_arguments.extend(
        DeclareLaunchArgument(name, default_value=default_value)
        for name, default_value in hardware_defaults.items()
    )
    launch_arguments.append(OpaqueFunction(function=_launch_setup))
    return LaunchDescription(launch_arguments)
