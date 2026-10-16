from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ms200_scan_launch = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch', 'ms200_scan.launch.py'
    )

    scan_filters_yaml = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        'config', 'scan_filters.yaml'
    )

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ms200_scan_launch),
        launch_arguments={
            'frame_id': 'laser_frame',
            'angle_min': '0.1',
            'angle_max': '350.9',
            'range_max': '12.0',
            'motor_speed': '14',
            'scan_topic': '/scan_tmp',
        }.items()
    )

    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[scan_filters_yaml],
        remappings=[
            ('scan', '/scan_tmp'),
            ('scan_filtered', '/scan'),
        ],
    )

    # Disabled for now -- re-enable if scan-matcher odom is needed again.
    # scan_matcher_node = Node(
    #     package='ros2_laser_scan_matcher',
    #     executable='laser_scan_matcher',
    #     name='scan_matcher',
    #     output='screen',
    #     parameters=[
    #         {'base_frame': 'base_link'},
    #         {'publish_tf': True},
    #         {'publish_odom': 'odom'}
    #     ]
    # )

    return LaunchDescription([
        lidar_driver,
        scan_filter_node,
    ])
