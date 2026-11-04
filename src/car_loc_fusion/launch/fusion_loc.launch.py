"""方法五: 結合所有感測器的融合定位 (多相機 + LiDAR + IMU + 輪速)。

    # 一鍵: YOLO 偵測 + LiDAR + 融合一起開 (融合發 map->base_link)
    ros2 launch car_loc_fusion fusion_loc.launch.py
    ros2 launch car_loc_fusion fusion_loc.launch.py evaluate:=true

    # A/B: 少一個來源會怎樣
    ros2 launch car_loc_fusion fusion_loc.launch.py multicam:=false
    ros2 launch car_loc_fusion fusion_loc.launch.py lidar:=false
    # A/B: 關掉延遲補償 (應該會變差 v x 延遲, 見 README [B])
    ros2 launch car_loc_fusion fusion_loc.launch.py evaluate:=true rewind:=false

    # 框由別處發 (每台 Pi 自己跑 YOLO, 或 multicam_loc.launch.py 已經開了偵測)
    ros2 launch car_loc_fusion fusion_loc.launch.py detector:=false

    # 真車 (6 軸 IMU, 沒有絕對姿態)
    ros2 launch car_loc_fusion fusion_loc.launch.py use_sim_time:=false \\
        imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro \\
        sigma_acc:=0.35

**跑之前要有的東西**

1. **stage 要有 8 台環境相機** (`kros_car_new.usd`)。沒有的話融合照跑, 只剩 LiDAR + 遞推。
2. **LiDAR 那條要先有地圖** —— 沒有的話 lidar_localizer 會自己退出, 融合就只
   剩多相機 + 遞推 (相機被擋住時只能靠遞推)。先建圖:
       ros2 launch car_loc_lidar mapping.launch.py
3. **開始之前先讓車子停著一秒** —— 開機靜止校正要在那裡把陀螺零偏量掉。

**節點本體在 car_loc_multicam** (`multicam_fusion`, FusionLocalizer 的子類別), 這裡用
`fusion_localizer` 的名字開它。car_loc_multicam 依賴 car_loc_fusion, 所以 package.xml
不能反過來宣告 (colcon 會判循環依賴), 這是這個 launch 唯一沒寫進 package.xml 的依賴。

**TF 只能有一個人發 map->base_link。** 這個 launch 讓**融合**發, LiDAR 那條強制關掉
(`publish_tf:=false`) —— 它的輸出照樣發在自己的 topic 上, 融合訂閱那個。
要讓別人發就改 `tf_source`。

**跟方法六同時開** (要比 `fus_` 跟 `mc_`): 8 台相機只要一個偵測節點, 另一邊給
`detector:=false`, 而且兩邊只能一個發 TF。
"""
import os

import yaml
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
    mc_params = os.path.join(
        get_package_share_directory('car_loc_multicam'), 'config', 'multicam_loc.yaml')
    lid_params = os.path.join(
        get_package_share_directory('car_loc_lidar'), 'config', 'lidar_loc.yaml')
    # 框的量測雜訊 / 綁組 / 挑框沿用方法六那一組, 兩條路線不分岔。
    # 節點名字不一樣, 所以要自己拆開 (param file 是照節點名字對的)。
    with open(mc_params, encoding='utf-8') as f:
        multicam_base = yaml.safe_load(f)['multicam_fusion']['ros__parameters']
    with open(fus_params, encoding='utf-8') as f:
        fusion_base = yaml.safe_load(f)['fusion_localizer']['ros__parameters']

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('multicam', default_value='true',
                              description='吃 8 台相機的車框 (絕對量測來源之一)'),
        DeclareLaunchArgument('detector', default_value='true',
                              description='要不要開 YOLO 偵測節點 (false = 框由別處發)'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='偵測節點發 /env_cam_i/car_bbox_debug 疊圖'),
        DeclareLaunchArgument('max_rate', default_value='15.0',
                              description='每台相機最多跑幾 Hz 的 YOLO'),
        DeclareLaunchArgument('cameras_file', default_value=''),
        DeclareLaunchArgument('car_model_file', default_value=''),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('gravity_mode', default_value='orientation',
                              description='orientation (Isaac/9軸) | complementary (真車6軸)'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation',
                              description='imu_orientation | gyro (真車6軸)'),
        DeclareLaunchArgument('sigma_acc', default_value='0.05',
                              description='complementary 模式要 0.35'),
        DeclareLaunchArgument('forward_deg', default_value='0.0',
                              description='kros_car 照 REP-103 是 0; 舊的 car.usd 車頭是 -Y 要給 -90'),
        DeclareLaunchArgument('wheel_scale', default_value='1.0'),
        DeclareLaunchArgument('rewind', default_value='true',
                              description='延遲補償 (倒帶重放)。false 只拿來做 A/B'),
        DeclareLaunchArgument('tf_source', default_value='fusion',
                              description='誰發 map->base_link: fusion | lidar | none'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='空 = car_loc_lidar/maps/room.yaml'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
        DeclareLaunchArgument('lidar_x', default_value='0.045'),
        DeclareLaunchArgument('lidar_z', default_value='0.236'),
        DeclareLaunchArgument('evaluate', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('csv', default_value=''),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    def owns_tf(who):
        return PythonExpression(
            ["'", LaunchConfiguration('tf_source'), "' == '", who, "'"])

    # ROI 跟著融合的輸出走, 所以 odom_topic 要換掉 (multicam_loc.yaml 寫的是方法六的 topic)
    detector = Node(
        package='car_loc_multicam', executable='multicam_detector',
        name='multicam_detector', output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('multicam'), "' == 'true' and '",
             LaunchConfiguration('detector'), "' == 'true'"])),
        parameters=[mc_params, {
            'use_sim_time': use_sim_time,
            'odom_topic': '/fusion_loc/odom',
            'cameras_file': LaunchConfiguration('cameras_file'),
            'car_model_file': LaunchConfiguration('car_model_file'),
            'publish_debug': ParameterValue(LaunchConfiguration('debug'), value_type=bool),
            'max_rate': ParameterValue(LaunchConfiguration('max_rate'), value_type=float),
        }])

    static_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_lidar',
        condition=IfCondition(LaunchConfiguration('lidar')),
        arguments=['--x', LaunchConfiguration('lidar_x'), '--y', '0',
                   '--z', LaunchConfiguration('lidar_z'),
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

    # 順序: 方法六的設定 -> fusion_loc.yaml -> 這裡的 launch 參數。後面蓋前面, 所以 topic
    # 名字以 fusion_loc.yaml 為準, launch 給的 (lidar:=false、rewind:=false ...) 最優先。
    # **參數一律先在這裡合併成一個 dict 再交給節點**, 不要寫成 [dict, yaml 檔, 覆寫 dict]。
    # ROS 2 套參數時是照「節點名第一次出現的順序」: 第一個是 dict (/**) 的話, 後面 yaml 檔
    # 裡指名 fusion_localizer: 的那一段會蓋掉**所有** /** 的覆寫, 包括排在最後的那個 ——
    # 2026-11-03 實測 tf_source:=none 時融合照發 TF, rewind:=false / gravity_mode 之類的
    # launch 參數也都沒生效。
    fusion = Node(
        package='car_loc_multicam', executable='multicam_fusion',
        name='fusion_localizer', output='screen',
        parameters=[{
            **multicam_base, **fusion_base,
            'use_sim_time': use_sim_time,
            'cameras_file': LaunchConfiguration('cameras_file'),
            'car_model_file': LaunchConfiguration('car_model_file'),
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
            'multicam_enabled': ParameterValue(
                LaunchConfiguration('multicam'), value_type=bool),
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
        args + [detector, static_lidar, lidar, fusion, evaluator, teleop])
