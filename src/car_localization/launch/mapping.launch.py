"""內建的建圖模式 (hector 式: 掃描對著一直長大的地圖配準)。

先確認你要的是這個:

| 想做的事 | 用哪個 |
| --- | --- |
| car.usd 這個模擬場景 | 都不要用, 跑 `./scripts/make_map_from_usd.py` (零建圖誤差) |
| 實體環境 / 未知場景, 要一張長期用的地圖 | `slam.launch.py` (slam_toolbox, 有回環偵測) |
| 小房間, 只想快速拿一張堪用的圖, 不想裝別的東西 | 這個 |

這個模式沒有回環偵測。走遠再繞回來時, 累積誤差沒有任何機制可以分攤, 地圖會在
接縫處錯開 —— 而「地圖歪了」跟「定位歪了」在結果上長得一模一樣, 很難查。
房間夠小、四面牆隨時都在視野裡的時候它才可靠。

用法:
    ros2 launch car_localization mapping.launch.py \
        map_save_path:=/workspaces/src/car_localization/maps/room.npz

Ctrl-C 會自動存檔並印出俯視圖; 中途也可以
    ros2 service call /car_mapper/save_map std_srvs/srv/Trigger

存完一定要親眼看過再拿去定位:
    python3 -m car_localization.gridmap show maps/room.npz
牆要是細線。同一面牆出現兩條平行線 = 建圖時位姿漂了, 重來。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('car_localization')
    params = os.path.join(pkg, 'config', 'localization.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_save_path',
                              default_value='/workspaces/src/car_localization/maps/room.npz'),
        DeclareLaunchArgument('map_resolution', default_value='0.05'),
        # 地圖會自己長大, 這個只是起始大小
        DeclareLaunchArgument('map_bounds', default_value='[-5.0, -5.0, 5.0, 5.0]'),
        DeclareLaunchArgument('initial_pose', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('input_type', default_value='pointcloud'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation'),
        DeclareLaunchArgument('lidar_z', default_value='0.20'),
        DeclareLaunchArgument('imu_z', default_value='0.075'),
        DeclareLaunchArgument('max_points', default_value='3000'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    mapper = Node(
        package='car_localization', executable='car_localizer', name='car_mapper',
        output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'mode': 'mapping',
            'map_path': '',
            'global_init': False,
            'input_type': LaunchConfiguration('input_type'),
            'map_save_path': LaunchConfiguration('map_save_path'),
            'map_bounds': LaunchConfiguration('map_bounds'),
            'map_resolution': LaunchConfiguration('map_resolution'),
            'initial_pose': LaunchConfiguration('initial_pose'),
            'yaw_source': LaunchConfiguration('yaw_source'),
            'max_points': LaunchConfiguration('max_points'),
            'status_period': 3.0,
        }])

    static_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_tf_lidar',
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('lidar_z'),
                   '--frame-id', 'base_link', '--child-frame-id', 'sim_lidar'],
        parameters=[{'use_sim_time': use_sim_time}])
    static_imu = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_tf_imu',
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('imu_z'),
                   '--frame-id', 'base_link', '--child-frame-id', 'sim_imu'],
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription(args + [static_lidar, static_imu, mapper])
