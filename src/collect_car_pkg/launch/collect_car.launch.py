"""收動態模型的訓練資料 (只給 Isaac 模擬用): 激發駕駛 + 記錄 + 致動器模型, 一個指令跑完。

    ros2 launch collect_car_pkg collect_car.launch.py csv_filename:=dyn_mu05.csv
    ros2 launch collect_car_pkg collect_car.launch.py mode:=hold_ramp seed:=1

一直跑到 Ctrl-C (駕駛節點結束前補送輪速 0)。
給沒有牆的開放場地用 (駕駛節點沒有撞牆保護)。

開三個節點:
  * collect_car_pkg/control_car_node  隨機輪速指令 -> /joint_command (見那支的檔頭);
                                      它結束 (Ctrl-C) 整個 launch 跟著關
  * calibrate_env_pkg/collect_data_node  記錄到 <output_dir>/<csv_filename> (50 Hz,
                                      指令、輪速、輪子扭矩、GT、IMU、phase / scenario_name)
                                      與 <csv 檔名>_imu.csv (IMU 全速率)
  * car_teleop/actuator_model         /joint_command -> /joint_command_isaac, 加上實體車的
                                      延遲 / 加速斜坡 (actuator:=false 不開)

actuator_model 只有在 USD 的指令 topic 是 joint_command_isaac 時才有作用 (kros_car.usd 是;
kros_car_05_sim.usd / kros_car_10_real.usd 還是 joint_command, 那時 Isaac 直接吃原始指令,
等於理想輪子)。USD 是 joint_command_isaac 卻沒開 actuator_model 的話車子完全不動。

記下來的 velocity_command_* 是 actuator_model **之前**的指令 = 實體車會收到的指令,
動態模型的輸入要用這個。

換環境比較時 seed、mode、ou_smooth_tau、max_wheel_speed 都要一樣, 指令序列才相同
(跑的時間長短不同沒關係, 短的那份就是長的那份的開頭)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value='/workspaces/car_run_data'),
        DeclareLaunchArgument('csv_filename', default_value='dyn_data.csv'),
        DeclareLaunchArgument('mode', default_value='ou', description='ou | hold_ramp'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('ou_smooth_tau', default_value='0.25',
                              description='OU 兩層低通的時間常數 (s); 0 = 不濾'),
        DeclareLaunchArgument('max_wheel_speed', default_value='10.0'),
        DeclareLaunchArgument('rate', default_value='20.0', description='送指令的頻率 (Hz)'),
        DeclareLaunchArgument('actuator', default_value='true',
                              description='開 actuator_model (實體車的延遲 / 加速斜坡)'),
        DeclareLaunchArgument('actuator_delay', default_value='0.05'),
    ]

    def f(name):
        return ParameterValue(lc(name), value_type=float)

    sim_time = ParameterValue(lc('use_sim_time'), value_type=bool)

    driver = Node(
        package='collect_car_pkg', executable='control_car_node', output='screen',
        on_exit=Shutdown(reason='激發駕駛結束'),
        parameters=[{
            'use_sim_time': sim_time,
            'mode': lc('mode'),
            'seed': ParameterValue(lc('seed'), value_type=int),
            'ou_smooth_tau': f('ou_smooth_tau'),
            'max_wheel_speed': f('max_wheel_speed'),
            'rate': f('rate'),
        }])

    collector = Node(
        package='calibrate_env_pkg', executable='collect_data_node', output='screen',
        parameters=[{
            'use_sim_time': sim_time,
            'output_dir': lc('output_dir'),
            'csv_filename': lc('csv_filename'),
        }])

    actuator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('car_teleop'), 'launch', 'actuator_model.launch.py')),
        condition=IfCondition(lc('actuator')),
        launch_arguments={
            'use_sim_time': lc('use_sim_time'),
            'delay': lc('actuator_delay'),
        }.items())

    return LaunchDescription(args + [collector, actuator, driver])
