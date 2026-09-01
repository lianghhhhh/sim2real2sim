"""建圖: 慢慢開一圈把房間掃完, 存成地圖檔。

    ros2 launch car_navigation build_map.launch.py map_save_path:=maps/room.npz

開的時候請注意:
  - 慢慢開, 盡量不要原地打轉。建圖期間的位姿誤差會直接變成地圖上的鬼牆。
  - 要把柱子後面、角落都繞到, 不然那些地方沒有地圖, 之後定位開進去就會飄。
  - 開完 Ctrl-C, 地圖會自動存檔; 也可以中途用
        ros2 service call /save_map std_srvs/srv/Trigger

存完一定要親眼看過再拿去定位:
    python3 -m car_navigation.gridmap show maps/room.npz

牆應該是細線。同一面牆出現兩條平行線 = 建圖時位姿漂了, 重建。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_save_path', default_value='maps/room.npz'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/point_cloud'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('lidar_frame', default_value='sim_lidar'),
        DeclareLaunchArgument('imu_frame', default_value='sim_imu'),
        DeclareLaunchArgument('lidar_z', default_value='0.20'),
        DeclareLaunchArgument('imu_z', default_value='0.075'),
        DeclareLaunchArgument('z_min', default_value='0.10'),
        DeclareLaunchArgument('z_max', default_value='0.95'),
        # 建圖期間無條件累積多久。比定位模式長很多 —— 這段時間是拿來把房間看完的。
        DeclareLaunchArgument('map_build_sec', default_value='8.0'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    static_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_tf_lidar',
        arguments=['--x', '0.0', '--y', '0.0', '--z', LaunchConfiguration('lidar_z'),
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', LaunchConfiguration('lidar_frame')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    static_imu = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_tf_imu',
        arguments=['--x', '0.0', '--y', '0.0', '--z', LaunchConfiguration('imu_z'),
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', LaunchConfiguration('imu_frame')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    mapper = Node(
        package='car_navigation', executable='lidar_imu_odometry', name='map_builder',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'z_min': LaunchConfiguration('z_min'),
            'z_max': LaunchConfiguration('z_max'),
            'map_mode': True,
            'map_build_sec': LaunchConfiguration('map_build_sec'),
            'map_save_path': LaunchConfiguration('map_save_path'),
            'publish_tf': True,          # 建圖時沒有 EKF, 由它自己發 TF
            'status_period': 3.0,        # 建圖時多印一點, 好隨時看地圖長多大
        }],
    )
    return LaunchDescription(args + [static_lidar, static_imu, mapper])
