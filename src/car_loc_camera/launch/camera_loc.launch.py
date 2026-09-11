"""方法一: 只用天花板相機 + YOLO 定位。

    ros2 launch car_loc_camera camera_loc.launch.py
    ros2 launch car_loc_camera camera_loc.launch.py evaluate:=true
    ros2 launch car_loc_camera camera_loc.launch.py teleop:=true      # 邊開邊看

evaluate:=true 會多開一個節點, 拿 Isaac 的 ground truth /odom 當尺,
每 5 秒印一行誤差, Ctrl-C 印總結。

輸出有兩個 topic, 用途不一樣, 別混用:
  /camera_loc/odom      影像時刻的狀態 + 影像時刻的時戳。依時戳內插的融合節點
                        (eskf / graph) 與 camera_loc_eval 要的是這個。
  /camera_loc/odom_now  外推到現在的狀態 + 現在的時戳。TF 與 nav 要的是這個
                        (預設 TF 就走這條, 見 config 的 tf_predict)。
濾波器與輸出的調校值 (accel_sigma / predict_rate / yaw_offset_deg ...) 一律在
config/camera_loc.yaml, 不從 launch 傳 —— 這裡只留 topic / frame / 開關。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('car_loc_camera')
    params = os.path.join(pkg, 'config', 'camera_loc.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='/rgb'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='空 = 自己找 (見 detector.find_model)'),
        DeclareLaunchArgument('calibration_path', default_value='',
                              description='空 = 用 package 內附的 config/camera_ground.yaml'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='三條路線同時跑時只能留一條發 map->base_link'),
        DeclareLaunchArgument('publish_annotated', default_value='true'),
        DeclareLaunchArgument('evaluate', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='true = 一起開遙控的速度控制層'),
        DeclareLaunchArgument('csv', default_value=''),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    localizer = Node(
        package='car_loc_camera', executable='camera_localizer',
        name='camera_localizer', output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'image_topic': LaunchConfiguration('image_topic'),
            'model_path': LaunchConfiguration('model_path'),
            'calibration_path': LaunchConfiguration('calibration_path'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'publish_annotated': LaunchConfiguration('publish_annotated'),
        }])

    evaluator = Node(
        package='car_loc_camera', executable='camera_loc_eval',
        name='camera_loc_eval', output='screen',
        condition=IfCondition(LaunchConfiguration('evaluate')),
        parameters=[{'use_sim_time': use_sim_time,
                     'csv': LaunchConfiguration('csv')}])

    # 遙控只是拿來開車, 跟定位無關 —— 鍵盤那一端要另開 terminal:
    #     ros2 run car_teleop teleop_key
    teleop = Node(
        package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('teleop')),
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription(args + [localizer, evaluator, teleop])
