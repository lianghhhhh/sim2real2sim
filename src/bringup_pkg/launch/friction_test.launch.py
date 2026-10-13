"""摩擦力測試腳本 + 感測器定位, 一個指令跑完 (跑完自動結束整個 launch)。

    # 接近真車的流程 (預設): 用融合定位 + IMU 控制, 不碰 GT
    ros2 launch bringup_pkg friction_test.launch.py csv_filename:=gravel.csv

    # 舊流程: 用 Isaac GT 控制, 不開定位節點
    ros2 launch bringup_pkg friction_test.launch.py state_source:=gt csv_filename:=gravel.csv

輸出兩個檔: <csv_filename> (50 Hz, 含各定位線、GT、輪子扭矩) 與 <csv 檔名>_imu.csv
(/imu 全速率, 每列帶著最新的輪速指令 / 輪速 / 輪子扭矩)。分析:

    ./scripts/estimate_friction.py ref.csv target.csv              # 自動用 gyro
    ./scripts/estimate_friction.py ref.csv target.csv --compare-gt  # 模擬器裡對答案

sensor 模式開的定位節點: camera + lidar + fusion (fusion 吃前兩者)。imu / wheel
兩條航位推算線預設不開 —— 控制跟分析都不用它們, 開了只是多吃 CPU; 想一起比較
定位品質就 imu:=true wheel:=true (記得給 imu_initial_pose)。

**sensor 模式在 GT 裡仍然記錄 /odom** (collect_data_node 一律訂閱), 那只是拿來對答案,
控制完全不看。真車上沒有 /odom, 那幾欄就是 NaN。

實體車 (topic 名稱不一樣, 輪速上限也要跟模擬那一輪相同):

    ros2 launch bringup_pkg friction_test.launch.py use_sim_time:=false \
        wheel_feedback_topic:=/wheel_feedback imu_topic:=/imu/data csv_filename:=real.csv

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
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('imu', default_value='false'),
        DeclareLaunchArgument('wheel', default_value='false'),
        DeclareLaunchArgument('imu_initial_pose', default_value='[0.0, 0.0, 0.0]'),
    ]
    sensor = PythonExpression(["'", lc('state_source'), "' == 'sensor'"])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bringup_pkg'), 'launch', 'collect_all.launch.py')),
        condition=IfCondition(sensor),
        launch_arguments={
            'use_sim_time': lc('use_sim_time'),
            'camera': lc('camera'), 'lidar': lc('lidar'),
            'imu': lc('imu'), 'wheel': lc('wheel'), 'fusion': 'true',
            'teleop': 'false', 'collector': 'false',
            'tf_source': 'fusion',
            'imu_initial_pose': lc('imu_initial_pose'),
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
        }])

    return LaunchDescription(args + [localization, test])
