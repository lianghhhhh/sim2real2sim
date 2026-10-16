#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 定義所有可以外部傳入的參數
    param_names = [
        'device_model', 'frame_id', 'scan_topic', 'port_name', 'baudrate',
        'angle_min', 'angle_max', 'range_min', 'range_max',
        'clockwise', 'motor_speed'
    ]

    # 宣告所有參數並給定預設值
    param_declarations = [
        DeclareLaunchArgument('device_model', default_value='MS200', description='Lidar device model'),
        DeclareLaunchArgument('frame_id', default_value='laser', description='Frame ID for LiDAR'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='Scan topic name'),
        DeclareLaunchArgument('port_name', default_value='/dev/oradar', description='LiDAR port'),
        DeclareLaunchArgument('baudrate', default_value='230400', description='Baudrate for serial communication'),
        DeclareLaunchArgument('angle_min', default_value='0.0', description='Minimum angle in degrees'),
        DeclareLaunchArgument('angle_max', default_value='360.0', description='Maximum angle in degrees'),
        DeclareLaunchArgument('range_min', default_value='0.05', description='Minimum range in meters'),
        DeclareLaunchArgument('range_max', default_value='20.0', description='Maximum range in meters'),
        DeclareLaunchArgument('clockwise', default_value='false', description='Scan direction'),
        DeclareLaunchArgument('motor_speed', default_value='10', description='Motor speed of LiDAR')
    ]

    # 建立 LiDAR 節點，並讓參數從外部傳入
    ordlidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='MS200',
        output='screen',
        parameters=[{param: LaunchConfiguration(param) for param in param_names}]
    )

    return LaunchDescription(param_declarations + [ordlidar_node])
