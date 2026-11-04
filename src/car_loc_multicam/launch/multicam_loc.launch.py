"""方法六: 8 台斜拍環境相機 + IMU + 輪速。

    ros2 launch car_loc_multicam multicam_loc.launch.py
    ros2 launch car_loc_multicam multicam_loc.launch.py evaluate:=true      # 跟 GT /odom 比
    ros2 launch car_loc_multicam multicam_loc.launch.py debug:=true         # 每台相機的偵測疊圖

    # 偵測在別的地方跑 (例如每台 Pi 自己發 /env_cam_i/car_bbox), 這裡只開融合
    ros2 launch car_loc_multicam multicam_loc.launch.py detector:=false

**跑之前要有的東西**

1. Isaac 要發 8 台相機的影像 (`/env_cam_0/rgb` ...)。kros_car_new.usd 已經有了;
   沒有發布的 stage 才需要在 Script Editor 跑 scripts/publish_env_cameras.py。
2. `/imu` 與 `/joint_states` (跟方法五一樣)。
3. 開始之前讓車子停一秒 (靜止校正)。

遞推那一半的參數讀 car_loc_fusion 的 fusion_loc.yaml, 再用本 package 的
multicam_loc.yaml 蓋上去 —— 兩條路線的遞推永遠是同一組參數。

**TF 只能有一個人發 map->base_link。** 跟方法五 (fusion_loc.launch.py) 同時開的話,
其中一個要 `publish_tf:=false`。

**用 bringup_pkg 的 collect_all / friction_test 時不要再開這個** —— 它們已經包含方法六
(`multicam_only`, 預設開), 而這裡預設會再開一個偵測節點 (兩個 YOLO 搶同樣 8 台影像,
2026-11-03 那兩輪 Isaac 的相機與 LiDAR 一起斷)。偵測節點發現有兩個時會一直印紅字。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('car_loc_multicam')
    mc_params = os.path.join(share, 'config', 'multicam_loc.yaml')
    fus_yaml = os.path.join(get_package_share_directory('car_loc_fusion'),
                            'config', 'fusion_loc.yaml')
    with open(fus_yaml, encoding='utf-8') as f:
        fusion_base = yaml.safe_load(f)['fusion_localizer']['ros__parameters']

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('detector', default_value='true',
                              description='要不要開 YOLO 偵測節點 (false = 框由別處發)'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='偵測節點發 /env_cam_i/car_bbox_debug 疊圖'),
        DeclareLaunchArgument('evaluate', default_value='false'),
        DeclareLaunchArgument('csv', default_value=''),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='false',
                              description='也吃 /lidar_loc/odom (LiDAR 那條要另外開)'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('gravity_mode', default_value='orientation',
                              description='orientation (Isaac/9軸) | complementary (真車6軸)'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation',
                              description='imu_orientation | gyro (真車6軸)'),
        DeclareLaunchArgument('sigma_acc', default_value='0.05',
                              description='complementary 模式要 0.35'),
        DeclareLaunchArgument('cameras_file', default_value=''),
        DeclareLaunchArgument('car_model_file', default_value=''),
        DeclareLaunchArgument('max_rate', default_value='15.0',
                              description='每台相機最多跑幾 Hz 的 YOLO'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    detector = Node(
        package='car_loc_multicam', executable='multicam_detector',
        name='multicam_detector', output='screen',
        condition=IfCondition(LaunchConfiguration('detector')),
        parameters=[mc_params, {
            'use_sim_time': use_sim_time,
            'cameras_file': LaunchConfiguration('cameras_file'),
            'car_model_file': LaunchConfiguration('car_model_file'),
            'publish_debug': ParameterValue(LaunchConfiguration('debug'), value_type=bool),
            'max_rate': ParameterValue(LaunchConfiguration('max_rate'), value_type=float),
        }])

    # **參數一律先在這裡合併成一個 dict 再交給節點**, 不要寫成 [dict, yaml 檔, 覆寫 dict]。
    # ROS 2 套參數時是照「節點名第一次出現的順序」: 第一個是 dict (/**) 的話, 後面 yaml 檔
    # 裡指名 multicam_fusion: 的那一段會蓋掉**所有** /** 的覆寫, 包括排在最後的那個 ——
    # 2026-11-03 實測 tf_source:=none 時融合照發 TF, rewind:=false / gravity_mode 之類的
    # launch 參數也都沒生效。
    with open(mc_params, encoding='utf-8') as f:
        multicam_base = yaml.safe_load(f)['multicam_fusion']['ros__parameters']
    fusion = Node(
        package='car_loc_multicam', executable='multicam_fusion',
        name='multicam_fusion', output='screen',
        parameters=[{
            **fusion_base, **multicam_base,
            'use_sim_time': use_sim_time,
            'cameras_file': LaunchConfiguration('cameras_file'),
            'car_model_file': LaunchConfiguration('car_model_file'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'gravity_mode': LaunchConfiguration('gravity_mode'),
            'yaw_source': LaunchConfiguration('yaw_source'),
            'sigma_acc': ParameterValue(LaunchConfiguration('sigma_acc'), value_type=float),
            'lidar_enabled': ParameterValue(LaunchConfiguration('lidar'), value_type=bool),
            'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
        }])

    # 評估直接用方法五的評估節點, 只是換一個 topic
    evaluator = Node(
        package='car_loc_fusion', executable='fusion_loc_eval',
        name='multicam_loc_eval', output='screen',
        condition=IfCondition(LaunchConfiguration('evaluate')),
        parameters=[{'use_sim_time': use_sim_time,
                     'estimate_topic': '/multicam_loc/odom',
                     'csv': LaunchConfiguration('csv')}])

    return LaunchDescription(args + [detector, fusion, evaluator])
