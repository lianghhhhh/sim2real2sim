import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('car_navigation')

    return LaunchDescription([
        # 1. 靜態 TF: base_link -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sim_lidar',
            arguments=['0.0', '0.0', '0.5', '0.0', '0.0', '0.0', 'base_link', 'sim_lidar']
        ),

        # 2. 靜態 TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sim_imu',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'sim_imu']
        ),

        # 3. Pointcloud to LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[('cloud_in', '/lidar/point_cloud'), ('scan', '/scan')],
            parameters=[{
                'use_sim_time': True,
                'target_frame': 'base_link',
                'min_height': -0.2,
                'max_height': 1.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'range_min': 0.05,
                'range_max': 60.0
            }]
        ),

        # 3b. QoS Bridge: /scan (BEST_EFFORT) -> /scan_reliable (RELIABLE)
        # pointcloud_to_laserscan 用 BEST_EFFORT 發布 /scan，
        # 但 rf2o_laser_odometry 訂閱端要求 RELIABLE，兩者 QoS 不相容時
        # DDS 不會報錯，只是訊息完全傳不過去。這裡加一個轉發節點解決。
        Node(
            package='car_navigation',
            executable='scan_qos_bridge',
            name='scan_qos_bridge',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/scan',
                'output_topic': '/scan_reliable'
            }]
        ),

        # 4. RF2O Laser Odometry
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            parameters=[{
                'use_sim_time': True,
                'laser_scan_topic': '/scan_reliable',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom'
            }]
        ),

        # 5. Robot Localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'ekf.yaml')]
        ),

        # 6. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')]
        )
    ])