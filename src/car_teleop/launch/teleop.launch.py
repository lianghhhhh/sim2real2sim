"""手動開車用的速度控制層。

    ros2 launch car_teleop teleop.launch.py

然後**另開一個 terminal** 跑鍵盤遙控 (它需要一個真的 TTY, 放進 launch 裡收不到按鍵):

    docker exec -it <container> bash -lc 'r && ros2 run car_teleop teleop_key'

或者不用鍵盤, 從 Foxglove 的 Teleop 面板往 /cmd_vel 發也可以 (見
car_viz 的 viz.launch.py)。

為什麼要有這一層: 車子 (kros_car.usd / 實體 KROS car) 收的是四輪的輪速
(/joint_command 的 velocity, rad/s)。這個節點把 /cmd_vel 的「我要幾 m/s、幾 rad/s」
換算成左右輪速, 預設是固定換算 (同一個指令永遠是同一組輪速); 給 nav2 用時加
yaw_trim_gain:=1.0, 轉速會用 /imu 的 gyro 修正。指令逾時會自動歸零。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        # 輪速上限 10 rad/s (實體車 max_rpm 100) 對應 0.50 m/s, 再高會被輪速上限縮掉。
        DeclareLaunchArgument('max_linear', default_value='0.5'),
        DeclareLaunchArgument('max_angular', default_value='1.2'),
        # 0 = 固定輪速 (鍵盤開車用); 1.0 = 用 gyro 修正轉速 (給 nav2 這類程式用)
        DeclareLaunchArgument('yaw_trim_gain', default_value='0.0'),
    ]
    return LaunchDescription(args + [
        Node(package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
             output='screen',
             parameters=[{
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                 'imu_topic': LaunchConfiguration('imu_topic'),
                 'max_linear': LaunchConfiguration('max_linear'),
                 'max_angular': LaunchConfiguration('max_angular'),
                 'yaw_trim_gain': ParameterValue(LaunchConfiguration('yaw_trim_gain'),
                                                 value_type=float),
             }]),
    ])
