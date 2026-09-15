"""方法五: 結合所有感測器的融合定位。

    # 一鍵: 相機 + LiDAR + 融合一起開 (融合發 map->base_link)
    ros2 launch car_loc_fusion fusion_loc.launch.py
    ros2 launch car_loc_fusion fusion_loc.launch.py evaluate:=true

    # A/B: 少一個來源會怎樣 (節點照跑, 只是那一路沒有輸入)
    ros2 launch car_loc_fusion fusion_loc.launch.py camera:=false
    ros2 launch car_loc_fusion fusion_loc.launch.py lidar:=false
    # A/B: 關掉延遲補償 (應該會變差 v x 0.08 s, 見 README [B])
    ros2 launch car_loc_fusion fusion_loc.launch.py evaluate:=true rewind:=false

    # 真車 (6 軸 IMU, 沒有絕對姿態)
    ros2 launch car_loc_fusion fusion_loc.launch.py use_sim_time:=false \
        imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro \
        sigma_acc:=0.35

**跑之前要有的東西**

1. **LiDAR 那條要先有地圖** —— 沒有的話 lidar_localizer 會自己退出, 融合就只
   剩相機 + 遞推 (還是跑得動, 但相機被擋住時只能靠遞推)。先建圖:
       ros2 launch car_loc_lidar mapping.launch.py
2. **相機那條要有 YOLO 模型** (見 car_loc_camera 的 README)。
3. **開始之前先讓車子停著一秒** —— 開機靜止校正要在那裡把陀螺零偏量掉。

**TF 只能有一個人發 map->base_link。** 這個 launch 讓**融合**發, 相機與 LiDAR
兩條強制關掉 (`publish_tf:=false`) —— 它們的輸出照樣發在自己的 topic 上, 融合
訂閱那個。要讓別人發就改 `tf_source`。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fus_params = os.path.join(
        get_package_share_directory('car_loc_fusion'), 'config', 'fusion_loc.yaml')
    cam_params = os.path.join(
        get_package_share_directory('car_loc_camera'), 'config', 'camera_loc.yaml')
    lid_params = os.path.join(
        get_package_share_directory('car_loc_lidar'), 'config', 'lidar_loc.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='要不要順便開相機那條 (它是絕對量測來源之一)'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('gravity_mode', default_value='orientation',
                              description='orientation (Isaac/9軸) | complementary (真車6軸)'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation',
                              description='imu_orientation | gyro (真車6軸)'),
        DeclareLaunchArgument('sigma_acc', default_value='0.05',
                              description='complementary 模式要 0.35'),
        DeclareLaunchArgument('forward_deg', default_value='-90.0',
                              description='car.usd 的車頭是 -Y; REP-103 的車給 0'),
        DeclareLaunchArgument('wheel_scale', default_value='0.93'),
        DeclareLaunchArgument('rewind', default_value='true',
                              description='延遲補償 (倒帶重放)。false 只拿來做 A/B'),
        DeclareLaunchArgument('tf_source', default_value='fusion',
                              description='誰發 map->base_link: fusion | lidar | camera | none'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='空 = car_loc_lidar/maps/room.yaml'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
        DeclareLaunchArgument('lidar_z', default_value='0.20'),
        DeclareLaunchArgument('evaluate', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('csv', default_value=''),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    def owns_tf(who):
        return PythonExpression(
            ["'", LaunchConfiguration('tf_source'), "' == '", who, "'"])

    camera = Node(
        package='car_loc_camera', executable='camera_localizer',
        name='camera_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('camera')),
        parameters=[cam_params, {
            'use_sim_time': use_sim_time,
            'publish_tf': owns_tf('camera'),
        }])

    static_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_lidar',
        condition=IfCondition(LaunchConfiguration('lidar')),
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('lidar_z'),
                   '--frame-id', 'base_link',
                   '--child-frame-id', LaunchConfiguration('lidar_frame')],
        parameters=[{'use_sim_time': use_sim_time}])

    lidar = Node(
        package='car_loc_lidar', executable='lidar_localizer',
        name='lidar_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('lidar')),
        parameters=[lid_params, {
            'use_sim_time': use_sim_time,
            'map_path': LaunchConfiguration('map_path'),
            'publish_tf': owns_tf('lidar'),
        }])

    fusion = Node(
        package='car_loc_fusion', executable='fusion_localizer',
        name='fusion_localizer', output='screen',
        parameters=[fus_params, {
            'use_sim_time': use_sim_time,
            'imu_topic': LaunchConfiguration('imu_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'gravity_mode': LaunchConfiguration('gravity_mode'),
            'yaw_source': LaunchConfiguration('yaw_source'),
            'sigma_acc': ParameterValue(
                LaunchConfiguration('sigma_acc'), value_type=float),
            'forward_deg': ParameterValue(
                LaunchConfiguration('forward_deg'), value_type=float),
            'wheel_scale': ParameterValue(
                LaunchConfiguration('wheel_scale'), value_type=float),
            'rewind': ParameterValue(
                LaunchConfiguration('rewind'), value_type=bool),
            'camera_enabled': ParameterValue(
                LaunchConfiguration('camera'), value_type=bool),
            'lidar_enabled': ParameterValue(
                LaunchConfiguration('lidar'), value_type=bool),
            'publish_tf': owns_tf('fusion'),
        }])

    evaluator = Node(
        package='car_loc_fusion', executable='fusion_loc_eval',
        name='fusion_loc_eval', output='screen',
        condition=IfCondition(LaunchConfiguration('evaluate')),
        parameters=[{'use_sim_time': use_sim_time,
                     'csv': LaunchConfiguration('csv')}])

    teleop = Node(
        package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('teleop')),
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription(
        args + [camera, static_lidar, lidar, fusion, evaluator, teleop])
