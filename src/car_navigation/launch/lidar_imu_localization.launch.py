"""
只用 LiDAR + IMU 估車子位置。

  Isaac Sim ──/lidar/point_cloud──┐
                                  ├─> lidar_imu_odometry (Python, ICP) ──/odom_lidar──┐
              ──/imu──────────────┘                                                   ├─> ekf_node ──/odometry/filtered
                                  └───────────────────────────────────────────────────┘         └─> TF odom->base_link

用法:
  ros2 launch car_navigation lidar_imu_localization.launch.py
  ros2 launch car_navigation lidar_imu_localization.launch.py cloud_topic:=/lidar/point_cloud lidar_frame:=sim_lidar
  ros2 launch car_navigation lidar_imu_localization.launch.py use_ekf:=false      # 只跑掃描比對, 由它自己發 TF
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from typing import List


def generate_launch_description():
    pkg_dir = get_package_share_directory('car_navigation')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='true=接 robot_localization 融合 IMU; false=只跑 ICP 里程計'),
        DeclareLaunchArgument('input_type', default_value='pointcloud',
                              description='pointcloud 或 scan'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/point_cloud'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        # 這兩個要跟 Isaac 發出來的 header.frame_id 一模一樣, 不然外參查不到
        DeclareLaunchArgument('lidar_frame', default_value='sim_lidar'),
        DeclareLaunchArgument('imu_frame', default_value='sim_imu'),
        # 這是感測器在車上的「實際掛載高度」, 一定要跟 USD 裡量到的一致。
        # 寫錯的話 z 濾波會整個錯位, 地面點全部留下來, ICP 會靜默失效。
        # car.usd 實測: LiDAR / IMU 都掛在 /World/small_car/Cube, 世界高度 0.075 m。
        # 應等於 USD 裡的實際掛載高度 (目前的 car.usd 是 0.200)。
        # 填錯不會再讓濾波失效 (節點會自己從點雲量地面), 但輸出位姿的座標原點
        # 會差這個高度, 而且節點會在 log 裡告訴你正確值。
        DeclareLaunchArgument('lidar_z', default_value='0.20'),
        DeclareLaunchArgument('imu_z', default_value='0.075'),
        # z 濾波區間 (base_link 座標, base_link 在地面上)。下界要高過地面,
        # 上界要低於可用結構的頂端。注意 car.usd 的牆是 2 m 高但中心在 z=0,
        # 所以地面以上其實只有 1.0 m -> 設 0.95。
        DeclareLaunchArgument('z_min', default_value='0.10'),
        DeclareLaunchArgument('z_max', default_value='0.95'),
        DeclareLaunchArgument('range_max', default_value='30.0'),
        # 事先建好並確認過的地圖。強烈建議用這個, 而不是每次啟動即時建圖 ——
        # 即時建出來的地圖沒人看過, 好壞憑運氣, 出事也分不清是地圖爛還是定位爛。
        #   ros2 launch car_navigation build_map.launch.py map_save_path:=maps/room.npz
        #   python3 -m car_navigation.gridmap show maps/room.npz
        #   ros2 launch car_navigation lidar_imu_localization.launch.py map_path:=maps/room.npz
        DeclareLaunchArgument('map_path', default_value=''),
        # 載入地圖時, 車子在地圖座標系裡的起始位姿 "[x, y, yaw_deg]"。填錯的話
        # 第一次配準就會對到錯的地方。
        DeclareLaunchArgument('initial_pose', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('map_mode', default_value='true'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf = LaunchConfiguration('use_ekf')

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

    scan_matcher = Node(
        package='car_navigation', executable='lidar_imu_odometry', name='lidar_imu_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_type': LaunchConfiguration('input_type'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'odom_topic': '/odom_lidar',
            'z_min': LaunchConfiguration('z_min'),
            'z_max': LaunchConfiguration('z_max'),
            'range_max': LaunchConfiguration('range_max'),
            'map_mode': LaunchConfiguration('map_mode'),
            'map_path': LaunchConfiguration('map_path'),
            'initial_pose': ParameterValue(LaunchConfiguration('initial_pose'),
                                           value_type=List[float]),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            # 有 EKF 時 TF 由 EKF 發, 沒有的話這個節點自己發, 兩者不可同時發
            'publish_tf': PythonExpression(["not ('", use_ekf, "' in ('true','True','1'))"]),
        }],
    )

    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[os.path.join(pkg_dir, 'config', 'ekf_lidar_imu.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

    # 印出目前估到的位置, 方便肉眼確認
    pose_echo = Node(
        package='car_navigation', executable='car_pose_subscriber', name='car_pose_subscriber',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'odom_topic': PythonExpression(
                         ["'/odometry/filtered' if '", use_ekf,
                          "' in ('true','True','1') else '/odom_lidar'"])}],
    )

    return LaunchDescription(args + [static_lidar, static_imu, scan_matcher, ekf, pose_echo])
