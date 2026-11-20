"""感測器定位 (方法五: 多相機 + LiDAR + IMU + 輪速) + path_tracker + MPC, 一個指令開完。

    # 模擬: Isaac 開 kros_car_mpc.usd 按 Play (不用在 Stage 選車和路線, 路線由 CSV 給)
    ros2 launch sensor_mpc_pkg sensor_mpc.launch.py

    # 定位已經在別的 terminal 跑 (例如 collect_all.launch.py) 時只開 tracker + MPC
    ros2 launch sensor_mpc_pkg sensor_mpc.launch.py localization:=false

    # A/B: 位姿改吃 Isaac 的 GT (其餘一樣)
    ros2 launch sensor_mpc_pkg sensor_mpc.launch.py localization:=false odom_topic:=/odom

    # 實體車 (topic、IMU 模式、地圖、路徑檔都不一樣)
    ros2 launch sensor_mpc_pkg sensor_mpc.launch.py use_sim_time:=false multicam:=false \\
        wheel_feedback_topic:=/wheel_feedback imu_topic:=/imu/data \\
        gravity_mode:=complementary yaw_source:=gyro fusion_sigma_acc:=0.35 \\
        map_path:=/s2r/src/car_loc_lidar/maps/lab.yaml \\
        path_file:=/s2r/src/sensor_mpc_pkg/config/path_lab.csv gt_topic:='' \\
        output_csv:=/s2r/car_run_data/real_mpc.csv

模擬會一起開 actuator_model (kros_car*.usd 吃 /joint_command_isaac, 不開車子不動);
use_sim_time:=false 時一律不開。kros_car_mpc_sensor.usd 有房間 (牆 + 圓柱), LiDAR 預設開;
kros_car_mpc.usd 的 /World/Room 是關的, **沒有牆就沒有 /scan**, 用它要 lidar:=false path_file:=...path_kros_car_mpc.csv。
"""
import os
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('sensor_mpc_pkg')
    default_path = os.path.join(share, 'config', 'path_kros_car_mpc_sensor.csv')
    # 參數先在這裡合併成一個 dict 再交給節點 (不要寫成 [yaml 檔, 覆寫 dict]):
    # 見 bringup_pkg/collect_all.launch.py 裡的說明, 順序錯了 launch 參數會沒生效。
    with open(os.path.join(share, 'config', 'sensor_mpc.yaml'), encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    tracker_base = cfg['path_tracker']['ros__parameters']
    # sensor_mpc_node 這個 process 裡有兩個節點, 兩段參數合在一起給 (/**), 各自只收有宣告的
    mpc_base = {**cfg['sensor_state_node']['ros__parameters'],
                **cfg['sensor_mpc_node']['ros__parameters']}
    lc = LaunchConfiguration

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true',
                              description='開融合定位 (collect_all.launch.py 的 fusion 那一組)'),
        DeclareLaunchArgument('odom_topic', default_value='/fusion_loc/odom',
                              description='位姿來源; A/B 用 /odom (Isaac GT)'),
        DeclareLaunchArgument('path_file', default_value=default_path,
                              description='路徑 CSV。預設是 kros_car_mpc_sensor.usd 房間用的 '
                                          '(scripts/make_path_kros_car_mpc_sensor.py); '
                                          'kros_car_mpc.usd 的舊路線是 path_kros_car_mpc.csv'),
        DeclareLaunchArgument('path_topic', default_value='',
                              description='nav_msgs/Path 來源 (收到就整條換掉); 給了要同時 path_file:=\'\'。'
                                          'nav_mpc_pkg 用 /nav_mpc/path'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('direction', default_value='auto'),
        DeclareLaunchArgument('mpc_config', default_value=EnvironmentVariable(
            'MPC_CONFIG_PATH', default_value='/workspaces/model/mpc_config.json')),
        DeclareLaunchArgument('output_csv', default_value='',
                              description='空 = <工作區>/car_run_data/sensor_mpc_{time}.csv; {time} 換成啟動時間 '
                                          '(台灣, 例 20261119_153012); 同名檔已存在就自動在檔名後加時間, 不覆寫'),
        DeclareLaunchArgument('gt_topic', default_value='/odom',
                              description='模擬裡對答案用的 GT; 實體車給空字串'),
        DeclareLaunchArgument('velocity_source', default_value='twist'),
        DeclareLaunchArgument('vy_source', default_value='zero'),
        DeclareLaunchArgument('abs_age_max', default_value='1.0'),
        DeclareLaunchArgument('pose_sigma_max', default_value='0.3'),
        # 位姿延遲補償 (MPC 這邊外插; 見 sensor_state.py 檔頭)。先看 CSV 的 pose_age 再決定。
        DeclareLaunchArgument('pose_lead_mode', default_value='none', description='none | stamp | fixed'),
        DeclareLaunchArgument('pose_lead_extra', default_value='0.0', description='s'),
        # 圍籬 (map 座標, m)。預設是 kros_car_mpc_sensor.usd 的房間 (牆內側 ±4.93 x ±3.39) 往內縮 0.4 m,
        # 加上圓柱 (2.10, -2.04) r 0.5 外擴 0.4 m。**實體場地一定要改**; 不想要就 geofence_x_min:=0 geofence_x_max:=0。
        DeclareLaunchArgument('geofence_x_min', default_value='-4.5'),
        DeclareLaunchArgument('geofence_x_max', default_value='4.5'),
        DeclareLaunchArgument('geofence_y_min', default_value='-3.0'),
        DeclareLaunchArgument('geofence_y_max', default_value='3.0'),
        DeclareLaunchArgument('geofence_circles', default_value='[2.10, -2.04, 0.9]',
                              description='[x, y, r, ...] 禁入的圓; 沒有就 "[0.0]"'),
        DeclareLaunchArgument('geofence_horizon', default_value='0.5'),
        # offset-free MPC: 估計模型與車子的固定差 (見 sensor_mpc_pkg/disturbance.py)。false = 舊的行為
        DeclareLaunchArgument('disturbance', default_value='true'),
        # 上層停車訊號 (std_msgs/Bool, true = 停; nav_mpc_pkg 用 /nav_mpc/hold)。空 = 不訂閱
        DeclareLaunchArgument('hold_topic', default_value=''),
        DeclareLaunchArgument('spin_topic', default_value='',
                              description='原地旋轉輪速 (std_msgs/Float32, 只在 hold 時有效; nav_mpc 用 /nav_mpc/spin)'),
        DeclareLaunchArgument('hold_timeout', default_value='0.0',
                              description='s; > 0: hold_topic 這麼久沒訊息也停車'),
        DeclareLaunchArgument('disturbance_tau', default_value='1.0', description='s'),
        # ---- 定位那一組 (跟 friction_test.launch.py 同名) ----
        DeclareLaunchArgument('imu_topic', default_value='/imu', description='實體車 /imu/data'),
        DeclareLaunchArgument('wheel_feedback_topic', default_value='/joint_states',
                              description='輪速回授; 實體車 /wheel_feedback'),
        DeclareLaunchArgument('multicam', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='LiDAR 進融合 (要有地圖: maps/room.yaml 跟 kros_car_mpc_sensor.usd 的房間同尺寸)。'
                                          'kros_car_mpc.usd 沒有牆 -> 沒 /scan, 要給 false'),
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('gravity_mode', default_value='orientation'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation'),
        DeclareLaunchArgument('fusion_sigma_acc', default_value='0.05'),
        DeclareLaunchArgument('wheel_scale', default_value='1.0'),
        DeclareLaunchArgument('actuator', default_value='true',
                              description='模擬時開 actuator_model (use_sim_time:=false 一律不開)'),
        DeclareLaunchArgument('actuator_delay', default_value='0.05'),
        # 融合那邊的常數延遲補償 (加在量測時戳上, 倒帶重放會補回去; 比 MPC 外插正確)。
        # 2026-11-17: 8 台相機那條位置落後 GT ~5 cm, 量出來大部分是固定偏移 (車型), 延遲只有 ~0.05 s;
        # 不要用 bbox_extra_delay 0.25 去補 (只在 0.25 m/s 剛好對)。LiDAR 單獨沒有落後。
        DeclareLaunchArgument('bbox_extra_delay', default_value='0.0'),
        DeclareLaunchArgument('lidar_extra_delay', default_value='0.0'),
    ]
    use_sim_time = ParameterValue(lc('use_sim_time'), value_type=bool)

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bringup_pkg'), 'launch', 'collect_all.launch.py')),
        condition=IfCondition(lc('localization')),
        launch_arguments={
            'use_sim_time': lc('use_sim_time'),
            'camera': 'false', 'imu': 'false', 'wheel': 'false',
            'multicam': lc('multicam'), 'lidar': lc('lidar'),
            'multicam_only': 'false', 'fusion': 'true',
            'teleop': 'false', 'collector': 'false',
            'tf_source': 'fusion',
            'map_path': lc('map_path'),
            'imu_topic': lc('imu_topic'),
            'joint_states_topic': lc('wheel_feedback_topic'),
            'gravity_mode': lc('gravity_mode'),
            'yaw_source': lc('yaw_source'),
            'fusion_sigma_acc': lc('fusion_sigma_acc'),
            'wheel_scale': lc('wheel_scale'),
            'bbox_extra_delay': lc('bbox_extra_delay'),
            'lidar_extra_delay': lc('lidar_extra_delay'),
        }.items())

    actuator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('car_teleop'), 'launch', 'actuator_model.launch.py')),
        condition=IfCondition(PythonExpression(
            ["'", lc('actuator'), "' == 'true' and '", lc('use_sim_time'), "' == 'true'"])),
        launch_arguments={'use_sim_time': lc('use_sim_time'),
                          'delay': lc('actuator_delay')}.items())

    tracker = Node(
        package='sensor_mpc_pkg', executable='path_tracker', name='path_tracker',
        output='screen',
        parameters=[{
            **tracker_base,
            'use_sim_time': use_sim_time,
            'path_file': ParameterValue(lc('path_file'), value_type=str),
            'path_topic': ParameterValue(lc('path_topic'), value_type=str),
            'odom_topic': lc('odom_topic'),
            'loop': ParameterValue(lc('loop'), value_type=bool),
            'direction': lc('direction'),
        }])

    mpc = Node(
        package='sensor_mpc_pkg', executable='sensor_mpc_node', output='screen',
        parameters=[{
            **mpc_base,
            'use_sim_time': use_sim_time,
            'odom_topic': lc('odom_topic'),
            'joint_states_topic': lc('wheel_feedback_topic'),
            'gt_topic': lc('gt_topic'),
            'velocity_source': lc('velocity_source'),
            'vy_source': lc('vy_source'),
            'abs_age_max': ParameterValue(lc('abs_age_max'), value_type=float),
            'pose_sigma_max': ParameterValue(lc('pose_sigma_max'), value_type=float),
            'pose_lead_mode': lc('pose_lead_mode'),
            'pose_lead_extra': ParameterValue(lc('pose_lead_extra'), value_type=float),
            'geofence_x_min': ParameterValue(lc('geofence_x_min'), value_type=float),
            'geofence_x_max': ParameterValue(lc('geofence_x_max'), value_type=float),
            'geofence_y_min': ParameterValue(lc('geofence_y_min'), value_type=float),
            'geofence_y_max': ParameterValue(lc('geofence_y_max'), value_type=float),
            'geofence_circles': ParameterValue(lc('geofence_circles'), value_type=List[float]),
            'geofence_horizon': ParameterValue(lc('geofence_horizon'), value_type=float),
            'disturbance_enabled': ParameterValue(lc('disturbance'), value_type=bool),
            'disturbance_tau': ParameterValue(lc('disturbance_tau'), value_type=float),
            'hold_topic': ParameterValue(lc('hold_topic'), value_type=str),
            'hold_timeout': ParameterValue(lc('hold_timeout'), value_type=float),
            'spin_topic': ParameterValue(lc('spin_topic'), value_type=str),
            'mpc_config': lc('mpc_config'),
            'output_csv': ParameterValue(lc('output_csv'), value_type=str),
        }])

    return LaunchDescription(args + [localization, actuator, tracker, mpc])
