"""LiDAR + IMU 定位。

    ros2 launch car_localization localization.launch.py
    ros2 launch car_localization localization.launch.py evaluate:=true
    ros2 launch car_localization localization.launch.py yaw_source:=gyro
    ros2 launch car_localization localization.launch.py map_path:=/workspaces/src/car_localization/maps/room.npz

evaluate:=true 會同時開一個節點, 拿 Isaac 的 /odom 當尺, 即時報告誤差幾公分。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('car_localization')
    params = os.path.join(pkg, 'config', 'localization.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='空字串 = 用 package 內附的 maps/car_usd.npz'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation',
                              description='imu_orientation (模擬) 或 gyro (真車路線)'),
        DeclareLaunchArgument('evaluate', default_value='false',
                              description='true = 同時跟 Isaac 的 /odom 比對誤差'),
        DeclareLaunchArgument('publish_debug_cloud', default_value='false',
                              description='true = 把配準後的點雲發到 rviz / Foxglove 看'),
        # 定位模式沒有別人在用 /scan, 預設就發出去 —— Foxglove / rviz 要看雷射靠它。
        DeclareLaunchArgument('publish_scan', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='true = 一起開遙控的速度控制層, 可以邊定位邊開車'),
        # 感測器掛載高度 (從 car.usd 量出來的, 一般不用改)
        DeclareLaunchArgument('lidar_z', default_value='0.20'),
        DeclareLaunchArgument('imu_z', default_value='0.075'),
        DeclareLaunchArgument('lidar_frame', default_value='sim_lidar'),
        DeclareLaunchArgument('imu_frame', default_value='sim_imu'),
        DeclareLaunchArgument('csv', default_value=''),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 這兩個 static TF 是給 rviz 用的 —— 定位節點本身讀的是 lidar_translation 參數,
    # 不查 TF (查 TF 會在 Isaac 還沒 Play 的時候卡住)。兩邊的值要一致。
    static_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_tf_lidar',
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('lidar_z'),
                   '--frame-id', 'base_link',
                   '--child-frame-id', LaunchConfiguration('lidar_frame')],
        parameters=[{'use_sim_time': use_sim_time}])

    static_imu = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_tf_imu',
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('imu_z'),
                   '--frame-id', 'base_link',
                   '--child-frame-id', LaunchConfiguration('imu_frame')],
        parameters=[{'use_sim_time': use_sim_time}])

    localizer = Node(
        package='car_localization', executable='car_localizer', name='car_localizer',
        output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'map_path': LaunchConfiguration('map_path'),
            'yaw_source': LaunchConfiguration('yaw_source'),
            'publish_debug_cloud': LaunchConfiguration('publish_debug_cloud'),
            'publish_scan': LaunchConfiguration('publish_scan'),
        }])

    teleop = Node(
        package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('teleop')),
        parameters=[{'use_sim_time': use_sim_time}])

    evaluator = Node(
        package='car_localization', executable='localization_eval',
        name='localization_eval', output='screen',
        condition=IfCondition(LaunchConfiguration('evaluate')),
        parameters=[{'use_sim_time': use_sim_time,
                     'csv': LaunchConfiguration('csv')}])

    return LaunchDescription(
        args + [static_lidar, static_imu, localizer, evaluator, teleop])
