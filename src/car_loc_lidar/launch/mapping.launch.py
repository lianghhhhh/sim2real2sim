"""方法二 —— 階段一: 手動開車建地圖 (Oradar MS200, 2D 單線雷射)。

    ros2 launch car_loc_lidar mapping.launch.py

裡面同時起了三個東西:

    /scan (MS200) ─> lidar_odometry (純雷射) ─┬─> TF odom->base_link
                                              └─> /scan_deskewed (運動補償過)
                                                          │
                                              slam_toolbox ┴─> /map + TF map->odom
    cmd_vel_bridge  ── 遙控的速度控制層 (鍵盤那一端要另開 terminal)

**`/scan` 是感測器自己的 topic, 不會被蓋掉** —— 模擬 (Isaac 的 laser_scan) 跟
實體車 (MS200 驅動) 發的都是它。去畸變過的版本另外發 `/scan_deskewed` 給
slam_toolbox 吃, 因為 MS200 一圈是 100 ms, 邊轉邊掃出來的那一圈沒補償是歪的。

鍵盤遙控 (`-it` 是必要的, 鍵盤需要真的 TTY):

    docker exec -it ros2_node bash -lc 'r && ros2 run car_teleop teleop_key'

開的時候三件事:

* **慢慢開。** MS200 只有 10 Hz、一圈 450 點, 比之前的雷射稀疏又慢, 開太快
  掃描比對更容易跟不上。
* **柱子後面、每個角落都要繞到**, 沒繞到的地方地圖上就是空的。
* **要繞回起點**, 回環偵測才有東西可以閉。
* MS200 只看得到 12 m —— 大場地要多繞幾趟, 不能站在中間轉一圈就算數。

建完存檔:

    ros2 run nav2_map_server map_saver_cli -f /workspaces/src/car_loc_lidar/maps/room

存完 colcon build 一次 (地圖會跟著 package 裝進 share), 然後:

    ros2 launch car_loc_lidar lidar_loc.launch.py

換回 3D 雷射 (PointCloud2):

    ros2 launch car_loc_lidar mapping.launch.py input_type:=pointcloud
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('car_loc_lidar')
    odom_params = os.path.join(pkg, 'config', 'lidar_odom.yaml')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('input_type', default_value='scan',
                              description='scan (Oradar MS200 的 LaserScan) '
                                          '或 pointcloud (3D RTX LiDAR)'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='感測器發的原始 LaserScan'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/point_cloud'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame',
                              description='要跟 car.usd 裡 helper 的 frameId 一致'),
        DeclareLaunchArgument('lidar_z', default_value='0.20'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='建圖一定要手動開一圈, 預設就把速度控制層開起來'),
        DeclareLaunchArgument('max_linear', default_value='0.6'),
        DeclareLaunchArgument('max_angular', default_value='1.2'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 給 rviz / Foxglove 看的; 里程計節點自己讀 lidar_translation 參數, 不查 TF
    # (查 TF 會在 Isaac 還沒按 Play 的時候卡住)。兩邊的值要一致。
    static_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('lidar_z'),
                   '--frame-id', 'base_link',
                   '--child-frame-id', LaunchConfiguration('lidar_frame')],
        parameters=[{'use_sim_time': use_sim_time}])

    odom = Node(
        package='car_loc_lidar', executable='lidar_odometry', name='lidar_odometry',
        output='screen',
        parameters=[odom_params, {
            'use_sim_time': use_sim_time,
            'input_type': LaunchConfiguration('input_type'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
        }])

    slam = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[LaunchConfiguration('slam_params_file'),
                    {'use_sim_time': use_sim_time}])

    teleop = Node(
        package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('teleop')),
        parameters=[{'use_sim_time': use_sim_time,
                     'max_linear': LaunchConfiguration('max_linear'),
                     'max_angular': LaunchConfiguration('max_angular')}])

    return LaunchDescription(args + [static_lidar, odom, slam, teleop])
