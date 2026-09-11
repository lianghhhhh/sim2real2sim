"""手動開車用的速度控制層。

    ros2 launch car_teleop teleop.launch.py

然後**另開一個 terminal** 跑鍵盤遙控 (它需要一個真的 TTY, 放進 launch 裡收不到按鍵):

    docker exec -it <container> bash -lc 'r && ros2 run car_teleop teleop_key'

或者不用鍵盤, 從 Foxglove 的 Teleop 面板往 /cmd_vel 發也可以 (見
car_viz 的 viz.launch.py)。

為什麼要有這一層: car.usd 的車子是扭矩控制, 而且幾乎沒有滾動阻力 —— 固定 effort
會一路加速, 鬆開也不會停。這個節點把 /cmd_vel 的「我要幾 m/s」用 PI 翻譯成扭矩,
回授吃 /joint_states 的輪速與 /imu 的角速度 (都是真車上也有的感測器)。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        # 建圖用的預設值刻意很慢。這台車 effort 開到 4 就有 3.6 m/s,
        # 在 10x6 的房間裡兩秒撞牆, 掃描比對也跟不上。
        DeclareLaunchArgument('max_linear', default_value='0.6'),
        DeclareLaunchArgument('max_angular', default_value='1.2'),
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
             }]),
    ])
