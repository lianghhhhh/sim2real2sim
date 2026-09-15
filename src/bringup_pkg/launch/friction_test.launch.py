"""摩擦力測試腳本 + 感測器定位, 一個指令跑完 (跑完自動結束整個 launch)。

    # 接近真車的流程 (預設): 用融合定位 + IMU 控制, 不碰 GT
    ros2 launch bringup_pkg friction_test.launch.py csv_filename:=gravel.csv

    # 舊流程: 用 Isaac GT 控制, 不開定位節點
    ros2 launch bringup_pkg friction_test.launch.py state_source:=gt csv_filename:=gravel.csv

輸出兩個檔: <csv_filename> (20 Hz, 含各定位線與 GT) 與 <csv 檔名>_imu.csv
(/imu 全速率)。分析:

    ./scripts/estimate_friction.py ref.csv target.csv              # 自動用 gyro
    ./scripts/estimate_friction.py ref.csv target.csv --compare-gt  # 模擬器裡對答案

sensor 模式開的定位節點: camera + lidar + fusion (fusion 吃前兩者)。imu / wheel
兩條航位推算線預設不開 —— 控制跟分析都不用它們, 開了只是多吃 CPU; 想一起比較
定位品質就 imu:=true wheel:=true (記得給 imu_initial_pose)。

**sensor 模式在 GT 裡仍然記錄 /odom** (collect_data_node 一律訂閱), 那只是拿來對答案,
控制完全不看。真車上沒有 /odom, 那幾欄就是 NaN。
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
        DeclareLaunchArgument('spin_max_w', default_value='0.0',
                              description='0 = 自動 (sensor 7.5, gt 12)'),
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
            'spin_max_w': ParameterValue(lc('spin_max_w'), value_type=float),
        }])

    return LaunchDescription(args + [localization, test])
