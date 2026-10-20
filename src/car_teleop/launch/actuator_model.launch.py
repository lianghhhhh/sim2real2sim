"""模擬端的輪子致動器模型 (只給 Isaac 用, 實體車不要開)。

    ros2 launch car_teleop actuator_model.launch.py

把 /joint_command 加上實體車 DDSM115 的延遲、加速斜坡、停車衰減與逾時, 轉發到
/joint_command_isaac。USD 的 ActionGraph_odom/ros2_subscribe_joint_state 的
topicName 要改成 joint_command_isaac 才會生效。細節見 car_teleop/actuator_model.py。

A/B 對照: enabled:=false 會直接轉發 (只保留逾時)。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('input_topic', default_value='/joint_command'),
        DeclareLaunchArgument('output_topic', default_value='/joint_command_isaac'),
        DeclareLaunchArgument('delay', default_value='0.035'),
        DeclareLaunchArgument('accel', default_value='30.0'),
        DeclareLaunchArgument('stop_tau', default_value='0.08'),
        DeclareLaunchArgument('smooth_tau', default_value='0.04'),
        DeclareLaunchArgument('enabled', default_value='true'),
    ]

    def f(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    return LaunchDescription(args + [
        Node(package='car_teleop', executable='actuator_model', name='actuator_model',
             output='screen',
             parameters=[{
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                value_type=bool),
                 'input_topic': LaunchConfiguration('input_topic'),
                 'output_topic': LaunchConfiguration('output_topic'),
                 'delay': f('delay'),
                 'accel': f('accel'),
                 'stop_tau': f('stop_tau'),
                 'smooth_tau': f('smooth_tau'),
                 'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
             }]),
    ])
