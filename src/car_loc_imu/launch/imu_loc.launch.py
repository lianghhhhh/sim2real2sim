"""方法三: 只用 IMU 定位。

    ros2 launch car_loc_imu imu_loc.launch.py
    ros2 launch car_loc_imu imu_loc.launch.py evaluate:=true

    # 真車 (6 軸 IMU, 沒有絕對姿態)。sigma_acc 一定要跟著調大 ——
    # 傾角是估出來的, 那個估計誤差就是持續存在的假加速度。
    ros2 launch car_loc_imu imu_loc.launch.py use_sim_time:=false \
        imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro \
        sigma_acc:=0.35

    # A/B: 關掉抗漂移機制看差多少
    ros2 launch car_loc_imu imu_loc.launch.py evaluate:=true \
        enable_zupt:=false enable_nhc:=false

    # 用擬合出來的零偏模型參數 (imu_fit_noise 的輸出)
    ros2 launch car_loc_imu imu_loc.launch.py noise_fit:=$PWD/imu_noise_fit.yaml

**開始之前先讓車子停著幾秒** —— 開機靜止校正要在那幾秒裡把零偏量掉。
車子一開始就在動的話漂移會明顯大很多。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('car_loc_imu')
    params = os.path.join(pkg, 'config', 'imu_loc.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('noise_fit',
                              default_value=os.path.join(pkg, 'config',
                                                         'imu_noise_fit.yaml'),
                              description='零偏模型的參數檔 (imu_fit_noise 的輸出)'),
        DeclareLaunchArgument('gravity_mode', default_value='orientation',
                              description='orientation (Isaac/9軸) | complementary (真車6軸) | none'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation',
                              description='imu_orientation | gyro (真車6軸)'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='三條路線同時跑時只能留一條發 map->base_link'),
        DeclareLaunchArgument('sigma_acc', default_value='0.03',
                              description='加速度過程雜訊底線; '
                                          'orientation 模式 0.03, '
                                          'complementary (6 軸) 要 0.35'),
        DeclareLaunchArgument('forward_deg', default_value='-90.0',
                              description='base_link +X 量到車頭的角度; '
                                          'car.usd 車頭是 -Y 所以 -90, '
                                          'REP-103 的車給 0'),
        DeclareLaunchArgument('enable_zupt', default_value='true'),
        DeclareLaunchArgument('enable_zaru', default_value='true'),
        DeclareLaunchArgument('enable_nhc', default_value='true'),
        DeclareLaunchArgument('evaluate', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('csv', default_value=''),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    localizer = Node(
        package='car_loc_imu', executable='imu_localizer', name='imu_localizer',
        output='screen',
        parameters=[params, LaunchConfiguration('noise_fit'), {
            'use_sim_time': use_sim_time,
            'imu_topic': LaunchConfiguration('imu_topic'),
            'gravity_mode': LaunchConfiguration('gravity_mode'),
            'yaw_source': LaunchConfiguration('yaw_source'),
            'forward_deg': ParameterValue(
                LaunchConfiguration('forward_deg'), value_type=float),
            'sigma_acc': ParameterValue(
                LaunchConfiguration('sigma_acc'), value_type=float),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'enable_zupt': LaunchConfiguration('enable_zupt'),
            'enable_zaru': LaunchConfiguration('enable_zaru'),
            'enable_nhc': LaunchConfiguration('enable_nhc'),
        }])

    evaluator = Node(
        package='car_loc_imu', executable='imu_loc_eval', name='imu_loc_eval',
        output='screen', condition=IfCondition(LaunchConfiguration('evaluate')),
        parameters=[{'use_sim_time': use_sim_time,
                     'csv': LaunchConfiguration('csv')}])

    teleop = Node(
        package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('teleop')),
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription(args + [localizer, evaluator, teleop])
