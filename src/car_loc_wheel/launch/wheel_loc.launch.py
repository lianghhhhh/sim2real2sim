"""IMU + 四輪輪速的航位推算。

    ros2 launch car_loc_wheel wheel_loc.launch.py
    ros2 launch car_loc_wheel wheel_loc.launch.py evaluate:=true

    # 真車 (6 軸 IMU, 沒有絕對姿態)。sigma_acc 要跟著調大 —— 傾角是估出來的,
    # 那個估計誤差就是持續存在的假加速度。
    ros2 launch car_loc_wheel wheel_loc.launch.py use_sim_time:=false \
        imu_topic:=/imu/data joint_states_topic:=/joint_states \
        gravity_mode:=complementary yaw_source:=gyro sigma_acc:=0.35

    # A/B: 關掉輪速看退化成純 IMU 差多少
    ros2 launch car_loc_wheel wheel_loc.launch.py evaluate:=true enable_wheel:=false

**開始之前先讓車子停著一秒** —— 開機靜止校正要在那裡把陀螺零偏量掉。
陀螺零偏沒校正的話 yaw 會持續漂, 而 yaw 誤差 × 行走距離就是位置誤差。
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
    pkg = get_package_share_directory('car_loc_wheel')
    params = os.path.join(pkg, 'config', 'wheel_loc.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('gravity_mode', default_value='orientation',
                              description='orientation (Isaac/9軸) | complementary (真車6軸) | none'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation',
                              description='imu_orientation | gyro (真車6軸)'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='多條路線同時跑時只能留一條發 map->base_link'),
        DeclareLaunchArgument('sigma_acc', default_value='0.05',
                              description='加速度過程雜訊底線; '
                                          'orientation 模式 0.05, '
                                          'complementary (6 軸) 要 0.35'),
        DeclareLaunchArgument('wheel_scale', default_value='0.93',
                              description='有效輪半徑 / 幾何輪半徑。跟牽引狀態有關: '
                                          '有扭矩時 0.93, 滑行時 1.008 (幾何值)。'
                                          '要自己校就錄一段正常開的直線再用 '
                                          'replay_bag.py --measure'),
        DeclareLaunchArgument('forward_deg', default_value='-90.0',
                              description='base_link +X 量到車頭的角度; '
                                          'car.usd 車頭是 -Y 所以 -90, '
                                          'REP-103 的車給 0'),
        DeclareLaunchArgument('enable_wheel', default_value='true',
                              description='false = 退化成純 IMU (A/B 用)'),
        DeclareLaunchArgument('enable_slip_gate', default_value='true'),
        DeclareLaunchArgument('enable_zupt', default_value='true'),
        DeclareLaunchArgument('evaluate', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('csv', default_value=''),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    localizer = Node(
        package='car_loc_wheel', executable='wheel_localizer', name='wheel_localizer',
        output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'imu_topic': LaunchConfiguration('imu_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'gravity_mode': LaunchConfiguration('gravity_mode'),
            'yaw_source': LaunchConfiguration('yaw_source'),
            'forward_deg': ParameterValue(
                LaunchConfiguration('forward_deg'), value_type=float),
            'sigma_acc': ParameterValue(
                LaunchConfiguration('sigma_acc'), value_type=float),
            'wheel_scale': ParameterValue(
                LaunchConfiguration('wheel_scale'), value_type=float),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'enable_wheel': LaunchConfiguration('enable_wheel'),
            'enable_slip_gate': LaunchConfiguration('enable_slip_gate'),
            'enable_zupt': LaunchConfiguration('enable_zupt'),
        }])

    evaluator = Node(
        package='car_loc_wheel', executable='wheel_loc_eval', name='wheel_loc_eval',
        output='screen', condition=IfCondition(LaunchConfiguration('evaluate')),
        parameters=[{'use_sim_time': use_sim_time,
                     'csv': LaunchConfiguration('csv')}])

    teleop = Node(
        package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('teleop')),
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription(args + [localizer, evaluator, teleop])
