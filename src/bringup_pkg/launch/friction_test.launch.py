"""摩擦力測試腳本 + 感測器定位, 一個指令跑完 (跑完自動結束整個 launch)。

    # 接近真車的流程 (預設): 用融合定位 + IMU 控制, 不碰 GT
    ros2 launch bringup_pkg friction_test.launch.py csv_filename:=gravel.csv

    # 舊流程: 用 Isaac GT 控制, 不開定位節點
    ros2 launch bringup_pkg friction_test.launch.py state_source:=gt csv_filename:=gravel.csv

輸出兩個檔: <csv_filename> (50 Hz, 含各定位線、GT、輪子扭矩) 與 <csv 檔名>_imu.csv
(/imu 全速率, 每列帶著最新的輪速指令 / 輪速 / 輪子扭矩)。分析:

    ./scripts/estimate_friction.py ref.csv target.csv              # 自動用 gyro
    ./scripts/estimate_friction.py ref.csv target.csv --compare-gt  # 模擬器裡對答案

sensor 模式開的定位節點: 多相機偵測 + lidar + fusion (fusion 吃前兩者), 以及方法六
(`mc_`, 跟 fusion 共用同一個偵測節點; `multicam_only:=false` 關掉)。**不要再另外開
multicam_loc.launch.py** —— 那會多一個偵測節點 (2026-11-03 兩輪就是這樣)。camera (單一
天花板相機) 與 imu / wheel 兩條航位推算線預設不開 —— 控制跟分析都不用它們, 開了只是多吃
CPU / GPU; 想一起比較定位品質就 camera:=true imu:=true wheel:=true (記得給 imu_initial_pose)。

stage 沒有 8 台環境相機 (目前只有 kros_car_new.usd 有) 時 fusion 只剩 LiDAR + 遞推,
給 multicam:=false 可以省下偵測節點的 GPU。

**sensor 模式在 GT 裡仍然記錄 /odom** (collect_data_node 一律訂閱), 那只是拿來對答案,
控制完全不看。真車上沒有 /odom, 那幾欄就是 NaN。

實體車 (topic、IMU 模式、地圖與圍籬都不一樣, 輪速上限要跟模擬那一輪相同;
完整步驟見 real/REAL_CAR_TELEOP_LIDAR_GUIDE.md):

    ros2 launch bringup_pkg friction_test.launch.py use_sim_time:=false multicam:=false \
        wheel_feedback_topic:=/wheel_feedback imu_topic:=/imu/data \
        gravity_mode:=complementary yaw_source:=gyro fusion_sigma_acc:=0.35 wheel_scale:=1.015 \
        map_path:=/s2r/src/car_loc_lidar/maps/lab.yaml \
        room_x_min:=... room_x_max:=... room_y_min:=... room_y_max:=... \
        output_dir:=/s2r/car_run_data csv_filename:=real.csv

輪速有兩個設定, **要比較的兩個環境兩個都必須一樣**:
  * `spin_wheel_speed` (預設 6 rad/s): B1 / B2 自旋與 B3 直行量測用的輪速。馬達出得了的
    扭矩隨轉速下降, 轉太快會飽和 —— 分析報「馬達飽和」就把它調低。
  * `max_wheel_speed` (預設 10 rad/s = 實體車目前的 max_rpm 100): B4 起步的階躍與
    歸位的車速。
送出去的輪速一律量化成整數 rpm (實體車的馬達只收整數 rpm)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('state_source', default_value='sensor',
                              description='sensor (融合定位 + IMU) | gt (Isaac /odom)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value='/workspaces/car_run_data'),
        DeclareLaunchArgument('csv_filename', default_value='sim_data.csv'),
        DeclareLaunchArgument('blocks', default_value='[1,2,3,4]'),
        DeclareLaunchArgument('max_wheel_speed', default_value='10.0',
                              description='輪速指令上限 (rad/s); sim 與 real 要一樣'),
        DeclareLaunchArgument('spin_wheel_speed', default_value='6.0',
                              description='自旋 / 直行量測用的輪速 (rad/s); sim 與 real 要一樣。'
                                          '分析報馬達飽和就調低'),
        DeclareLaunchArgument('spin_max_w', default_value='0.0',
                              description='車體轉速安全上限 (rad/s); 0 = 依輪速上限自動算'),
        DeclareLaunchArgument('imu_topic', default_value='/imu',
                              description='實體車是 /imu/data'),
        DeclareLaunchArgument('wheel_feedback_topic', default_value='joint_states',
                              description='輪速 / 扭矩回授; 實體車是 /wheel_feedback'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='單一天花板相機, fusion 不吃它'),
        DeclareLaunchArgument('multicam', default_value='true'),
        DeclareLaunchArgument('multicam_only', default_value='true',
                              description='同時記 mc_ (方法六, 不吃 LiDAR), 共用偵測節點'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('imu', default_value='false'),
        DeclareLaunchArgument('wheel', default_value='false'),
        DeclareLaunchArgument('imu_initial_pose', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='LiDAR 定位的地圖; 空 = car_loc_lidar/maps/room.yaml (模擬房間)'),
        # 定位節點的 IMU 模式 (見 collect_all.launch.py)。實體車:
        #   gravity_mode:=complementary yaw_source:=gyro fusion_sigma_acc:=0.35
        DeclareLaunchArgument('gravity_mode', default_value='orientation'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation'),
        DeclareLaunchArgument('fusion_sigma_acc', default_value='0.05'),
        DeclareLaunchArgument('wheel_scale', default_value='1.0',
                              description='定位用的有效輪半徑比例 (實體輪徑 / 0.05035)'),
        # 圍籬: 車子到得了的內牆範圍 (map 座標, m)。預設是模擬房間 —— 實體場地一定要改。
        DeclareLaunchArgument('room_x_min', default_value='-4.58'),
        DeclareLaunchArgument('room_x_max', default_value='4.58'),
        DeclareLaunchArgument('room_y_min', default_value='-2.58'),
        DeclareLaunchArgument('room_y_max', default_value='2.58'),
    ]
    sensor = PythonExpression(["'", lc('state_source'), "' == 'sensor'"])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bringup_pkg'), 'launch', 'collect_all.launch.py')),
        condition=IfCondition(sensor),
        launch_arguments={
            'use_sim_time': lc('use_sim_time'),
            'camera': lc('camera'), 'multicam': lc('multicam'), 'lidar': lc('lidar'),
            'multicam_only': lc('multicam_only'),
            'imu': lc('imu'), 'wheel': lc('wheel'), 'fusion': 'true',
            'teleop': 'false', 'collector': 'false',
            'tf_source': 'fusion',
            'imu_initial_pose': lc('imu_initial_pose'),
            'map_path': lc('map_path'),
            'imu_topic': lc('imu_topic'),
            'joint_states_topic': lc('wheel_feedback_topic'),
            'gravity_mode': lc('gravity_mode'),
            'yaw_source': lc('yaw_source'),
            'fusion_sigma_acc': lc('fusion_sigma_acc'),
            'wheel_scale': lc('wheel_scale'),
        }.items())

    test = Node(
        package='calibrate_env_pkg', executable='calibrate_env_node',
        output='screen',
        on_exit=Shutdown(reason='測試腳本跑完'),
        parameters=[{
            'use_sim_time': ParameterValue(lc('use_sim_time'), value_type=bool),
            'state_source': lc('state_source'),
            'output_dir': lc('output_dir'),
            'csv_filename': lc('csv_filename'),
            'blocks': ParameterValue(lc('blocks'), value_type=None),
            'max_wheel_speed': ParameterValue(lc('max_wheel_speed'), value_type=float),
            'spin_wheel_speed': ParameterValue(lc('spin_wheel_speed'), value_type=float),
            'spin_max_w': ParameterValue(lc('spin_max_w'), value_type=float),
            'imu_topic': lc('imu_topic'),
            'wheel_feedback_topic': lc('wheel_feedback_topic'),
            'room_x_min': ParameterValue(lc('room_x_min'), value_type=float),
            'room_x_max': ParameterValue(lc('room_x_max'), value_type=float),
            'room_y_min': ParameterValue(lc('room_y_min'), value_type=float),
            'room_y_max': ParameterValue(lc('room_y_max'), value_type=float),
        }])

    return LaunchDescription(args + [localization, test])
