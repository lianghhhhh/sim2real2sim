"""方法二 —— 階段二: 只用 LiDAR 對既有地圖定位 (Oradar MS200, 2D 單線雷射)。

    ros2 launch car_loc_lidar lidar_loc.launch.py
    ros2 launch car_loc_lidar lidar_loc.launch.py evaluate:=true
    ros2 launch car_loc_lidar lidar_loc.launch.py map_path:=/workspaces/src/car_loc_lidar/maps/room.yaml

地圖還沒建的話先跑 mapping.launch.py (見那份的說明)。

evaluate:=true 會多開一個節點, 拿 Isaac 的 ground truth /odom 當尺。
**先看它報的「常數偏移」** —— slam_toolbox 的地圖原點是車子按 Play 那一刻的
位置, 跟 /odom 差一個固定平移是正常的, 那不是定位在漂。

實體車 —— 因為模擬跟實體用的是同一顆 MS200 規格 (同樣的 LaserScan、同樣的
量程與轉速), 只要關掉模擬時鐘就好:

    ros2 launch car_loc_lidar lidar_loc.launch.py use_sim_time:=false

換回 3D 雷射 (PointCloud2):

    ros2 launch car_loc_lidar lidar_loc.launch.py input_type:=pointcloud
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('car_loc_lidar')
    params = os.path.join(pkg, 'config', 'lidar_loc.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='空 = 用 package 內附的 maps/room.yaml'),
        DeclareLaunchArgument('input_type', default_value='scan',
                              description='scan (Oradar MS200 的 LaserScan) '
                                          '或 pointcloud (3D RTX LiDAR)'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/point_cloud'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame',
                              description='要跟 car.usd 裡 helper 的 frameId 一致'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='三條路線同時跑時只能留一條發 map->base_link'),
        DeclareLaunchArgument('publish_debug_cloud', default_value='false',
                              description='true = 把配準後的點雲發出去疊在地圖上看'),
        DeclareLaunchArgument('lidar_z', default_value='0.20'),
        DeclareLaunchArgument('evaluate', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('csv', default_value=''),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    static_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('lidar_z'),
                   '--frame-id', 'base_link',
                   '--child-frame-id', LaunchConfiguration('lidar_frame')],
        parameters=[{'use_sim_time': use_sim_time}])

    localizer = Node(
        package='car_loc_lidar', executable='lidar_localizer',
        name='lidar_localizer', output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'map_path': LaunchConfiguration('map_path'),
            'input_type': LaunchConfiguration('input_type'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'publish_debug_cloud': LaunchConfiguration('publish_debug_cloud'),
        }])

    evaluator = Node(
        package='car_loc_lidar', executable='lidar_loc_eval',
        name='lidar_loc_eval', output='screen',
        condition=IfCondition(LaunchConfiguration('evaluate')),
        parameters=[{'use_sim_time': use_sim_time,
                     'csv': LaunchConfiguration('csv')}])

    teleop = Node(
        package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('teleop')),
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription(args + [static_lidar, localizer, evaluator, teleop])
