"""用 slam_toolbox 建地圖 —— 這是「要搬到實體環境」的那條路。

為什麼是 slam_toolbox 而不是本 package 內建的 mapping 模式:
    內建的 mapping 是 hector 式的 (掃描對著一直長大的地圖配準)。在一個四面牆
    隨時都看得到的小房間裡它很好用, 但它**沒有回環偵測**: 走遠再繞回來的時候,
    累積的那點誤差沒有任何機制去分攤掉, 地圖會在接縫處錯開。
    slam_toolbox 有位姿圖 + 回環偵測 + 全域最佳化, 而且這正是 wildbot 實體車
    已經驗證過的東西 (docker-compose_slam_oradarlidar.yml)。地圖是要長期使用的
    資產, 值得用那一套。

它缺的那一塊由這裡補上:
    slam_toolbox 需要有人發 odom -> base_link。實體車那是輪速計 + EKF 給的;
    這台模擬車沒有輪速計, 所以用 car_localizer 的 odometry 模式 —— 只對最近
    幾個關鍵幀組成的滾動子圖配準, 產生局部準確、連續不跳的里程計。
    順便它也把運動補償後的 LaserScan 發出來給 slam_toolbox 吃, 這比
    pointcloud_to_laserscan 好: 車子邊轉邊掃的那一圈, 沒補償的版本是歪的。

    /lidar/point_cloud ─┐
    /imu ───────────────┴─> car_localizer (odometry) ─┬─> TF odom->base_link
                                                      └─> /scan (去畸變)
                                                                   │
                                                       slam_toolbox ┴─> TF map->odom + /map

用法:
    ros2 launch car_localization slam.launch.py

    # 慢慢開一圈, 把角落跟柱子後面都繞到, 而且要繞回起點 (回環才有東西可以閉)
    # 建完存檔:
    ros2 run nav2_map_server map_saver_cli -f /workspaces/src/car_localization/maps/room

    # 之後就用這張地圖定位 (直接吃 nav2 的 .yaml):
    ros2 launch car_localization localization.launch.py \
        map_path:=/workspaces/src/car_localization/maps/room.yaml

實體車上換三個參數就好:
    use_sim_time:=false input_type:=scan imu_topic:=/imu/data yaw_source:=gyro
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('car_localization')
    loc_params = os.path.join(pkg, 'config', 'localization.yaml')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('input_type', default_value='pointcloud',
                              description='pointcloud (Isaac) 或 scan (實體 2D 雷射)'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/point_cloud'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='input_type:=scan 時的輸入 topic'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation',
                              description='實體車請用 gyro'),
        DeclareLaunchArgument('lidar_z', default_value='0.20'),
        DeclareLaunchArgument('imu_z', default_value='0.075'),
        # 建圖時點多一點比較划算 (定位是每幀都要跑, 建圖只做一次)
        DeclareLaunchArgument('max_points', default_value='3000'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params),
        # 建圖一定要手動把房間開一圈, 所以順手把遙控的速度控制層一起開。
        # 鍵盤遙控要另開 terminal 跑 (它需要真的 TTY):
        #     ros2 run car_teleop teleop_key
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('max_linear', default_value='0.6'),
        DeclareLaunchArgument('max_angular', default_value='1.2'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_cloud = PythonExpression(["'", LaunchConfiguration('input_type'), "' != 'scan'"])

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

    common = {
        'use_sim_time': use_sim_time,
        'mode': 'odometry',
        'map_path': '',
        'input_type': LaunchConfiguration('input_type'),
        'cloud_topic': LaunchConfiguration('cloud_topic'),
        'imu_topic': LaunchConfiguration('imu_topic'),
        'yaw_source': LaunchConfiguration('yaw_source'),
        'max_points': LaunchConfiguration('max_points'),
        'publish_tf': True,
        'publish_scan': True,
        'status_period': 3.0,
    }
    # 點雲輸入: 自己發去畸變的 /scan 給 slam_toolbox
    odom_from_cloud = Node(
        package='car_localization', executable='car_localizer', name='lidar_odometry',
        output='screen', condition=IfCondition(is_cloud),
        parameters=[loc_params, dict(common, scan_out_topic='/scan')])
    # 2D 雷射輸入: 原本的 /scan 直接給 slam_toolbox, 這裡就不要再發一份
    odom_from_scan = Node(
        package='car_localization', executable='car_localizer', name='lidar_odometry',
        output='screen', condition=UnlessCondition(is_cloud),
        parameters=[loc_params, dict(common, publish_scan=False,
                                     scan_topic=LaunchConfiguration('scan_topic'))])

    slam = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file'),
                    {'use_sim_time': use_sim_time}])

    teleop = Node(
        package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('teleop')),
        parameters=[{'use_sim_time': use_sim_time,
                     'imu_topic': LaunchConfiguration('imu_topic'),
                     'max_linear': LaunchConfiguration('max_linear'),
                     'max_angular': LaunchConfiguration('max_angular')}])

    return LaunchDescription(
        args + [static_lidar, static_imu, odom_from_cloud, odom_from_scan,
                slam, teleop])
