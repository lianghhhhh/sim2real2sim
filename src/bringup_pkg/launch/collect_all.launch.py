"""五種定位同時跑, 跟 ground truth 一起記進同一個 CSV。

    ros2 launch bringup_pkg collect_all.launch.py \
        imu_initial_pose:="[2.0, -0.3, 0.0]" csv_filename:=all_loc.csv

然後另開一個 terminal 用鍵盤開車:

    ros2 run car_teleop teleop_key

CSV 會長在 output_dir/csv_filename, 欄位說明看 collect_data_node 的 docstring。

| 開哪些 | 記到哪幾欄 |
| --- | --- |
| `car_loc_camera` | `cam_*` |
| `car_loc_lidar` | `lid_*` |
| `car_loc_imu` | `imu_*` |
| `car_loc_wheel` | `whl_*` |
| `car_loc_fusion` | `fus_*` (它吃 cam_ 與 lid_ 的輸出當絕對量測) |

(舊名是 `three_way_collect.launch.py` —— 那時候只有三條。)

五件會咬到的事
--------------
1. **TF 只能有一個人發 map -> base_link。** 五條都預設 publish_tf:=true, 同時開
   就會五個人搶著發同一條邊, TF tree 變成誰後發誰贏 —— RViz 看起來像車子在抽搐,
   而且**五邊的估計都沒錯**, 錯的是 TF。這個 launch 用 `tf_source` 決定誰發
   (預設 fusion), 其他強制關掉。純粹要蒐 CSV 的話設 `tf_source:=none` 最乾淨。

2. **imu / wheel 那兩條要給起點。** 它們是航位推算, 從 `imu_initial_pose` 開始
   積分, 沒給就是從 (0,0) 起跑 —— 車子不是生在原點的話整段差一個常數平移,
   看起來像超大飄移。按 Play 之前先看一下:

       ros2 topic echo /odom --once --field pose.pose.position

   **融合那條不用給** —— 它拿第一則絕對量測當起點。這件事本身就是一個結果:
   同一份資料裡 `imu_`/`whl_` 要人喂起點, `fus_` 不用。

3. **按 Play 之後先讓車子停幾秒再開。** imu / wheel / fusion 三條都要在那幾秒
   裡量陀螺零偏。一啟動就衝出去的話漂移會大很多, 而且事後看 CSV 分不出來 ——
   那不是方法的問題, 是資料的問題。

4. **LiDAR 那條要先有地圖。** 還沒建的話先跑
   `ros2 launch car_loc_lidar mapping.launch.py`。地圖不存在時 lidar_localizer
   會自己退出, 其他照跑, CSV 的 lid_* 整欄 NaN —— 而 `fus_` 會少一個來源
   (它照樣跑, 只剩相機 + 遞推)。

5. **fusion 吃的是 camera 與 lidar 的 topic。** 把 `camera:=false` 關掉的話,
   `fus_` 也跟著少一個來源。要比「融合 vs 單一來源」就五條全開。

只比其中幾條就把別的關掉, 例如不想跑 YOLO (吃 GPU):

    ros2 launch bringup_pkg collect_all.launch.py camera:=false
"""
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cam_params = os.path.join(
        get_package_share_directory('car_loc_camera'), 'config', 'camera_loc.yaml')
    lid_params = os.path.join(
        get_package_share_directory('car_loc_lidar'), 'config', 'lidar_loc.yaml')
    imu_params = os.path.join(
        get_package_share_directory('car_loc_imu'), 'config', 'imu_loc.yaml')
    whl_params = os.path.join(
        get_package_share_directory('car_loc_wheel'), 'config', 'wheel_loc.yaml')
    fus_params = os.path.join(
        get_package_share_directory('car_loc_fusion'), 'config', 'fusion_loc.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('imu', default_value='true'),
        DeclareLaunchArgument('wheel', default_value='true'),
        DeclareLaunchArgument('fusion', default_value='true',
                              description='方法五 —— 吃 camera 與 lidar 的輸出, '
                                          '那兩條關掉的話它就少一個來源'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='cmd_vel -> joint_command 的速度控制層'),
        DeclareLaunchArgument('tf_source', default_value='fusion',
                              description='誰來發 map->base_link: '
                                          'fusion | lidar | camera | imu | wheel | none'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='空 = car_loc_lidar/maps/room.yaml'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
        DeclareLaunchArgument('lidar_z', default_value='0.20'),
        DeclareLaunchArgument('imu_initial_pose', default_value='[0.0, 0.0, 0.0]',
                              description='x, y, yaw(度)。imu_ 與 whl_ 兩條的起點, '
                                          '要對到車子的實際出生點。fusion 不需要'),
        DeclareLaunchArgument('collector', default_value='true',
                              description='開 collect_data_node。friction_test.launch.py '
                                          '會關掉 (calibrate_env_node 自己帶一個)'),
        DeclareLaunchArgument('output_dir', default_value='/workspaces/car_run_data'),
        DeclareLaunchArgument('csv_filename', default_value='all_loc.csv'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')
    initial_pose = ParameterValue(
        LaunchConfiguration('imu_initial_pose'), value_type=List[float])

    def owns_tf(who):
        # 字串比較要在 launch 展開之後才做得到, 所以包成 PythonExpression。
        return PythonExpression(["'", LaunchConfiguration('tf_source'), "' == '", who, "'"])

    camera = Node(
        package='car_loc_camera', executable='camera_localizer',
        name='camera_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('camera')),
        parameters=[cam_params, {
            'use_sim_time': use_sim_time,
            'publish_tf': owns_tf('camera'),
        }])

    # 雷射掛在 base_link 上方 0.2 m。只有 LiDAR 需要這條 static TF, 而它跟
    # tf_source 無關 —— 那是 base_link -> laser_frame, 不是 map 那條。
    static_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_lidar',
        condition=IfCondition(LaunchConfiguration('lidar')),
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('lidar_z'),
                   '--frame-id', 'base_link',
                   '--child-frame-id', LaunchConfiguration('lidar_frame')],
        parameters=[{'use_sim_time': use_sim_time}])

    lidar = Node(
        package='car_loc_lidar', executable='lidar_localizer',
        name='lidar_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('lidar')),
        parameters=[lid_params, {
            'use_sim_time': use_sim_time,
            'map_path': LaunchConfiguration('map_path'),
            'publish_tf': owns_tf('lidar'),
        }])

    imu = Node(
        package='car_loc_imu', executable='imu_localizer',
        name='imu_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('imu')),
        parameters=[imu_params, {
            'use_sim_time': use_sim_time,
            'publish_tf': owns_tf('imu'),
            'initial_pose': initial_pose,
        }])

    # imu_ 與 whl_ 要**同時開**才有意義 —— 這一組是拿來直接回答「加了輪速值多少」
    # 的, 兩條吃同一份 /imu, 差別只在 whl_ 多吃了 /joint_states。
    wheel = Node(
        package='car_loc_wheel', executable='wheel_localizer',
        name='wheel_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('wheel')),
        parameters=[whl_params, {
            'use_sim_time': use_sim_time,
            'publish_tf': owns_tf('wheel'),
            'initial_pose': initial_pose,
        }])

    # 方法五。它訂閱 /camera_loc/odom 與 /lidar_loc/odom, 所以要放在同一份 launch
    # 裡才拿得到 —— 但它**不需要** initial_pose (第一則絕對量測就是起點)。
    fusion = Node(
        package='car_loc_fusion', executable='fusion_localizer',
        name='fusion_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('fusion')),
        parameters=[fus_params, {
            'use_sim_time': use_sim_time,
            'publish_tf': owns_tf('fusion'),
            'camera_enabled': ParameterValue(
                LaunchConfiguration('camera'), value_type=bool),
            'lidar_enabled': ParameterValue(
                LaunchConfiguration('lidar'), value_type=bool),
        }])

    # calibrate_env_node 會連 ControlCarNode 一起開 (車子自己跑摩擦力腳本, 跑完
    # 整個 process 結束)。這裡要的是「一直記到 Ctrl-C」, 所以用只有蒐集的那個。
    collector = Node(
        package='calibrate_env_pkg', executable='collect_data_node',
        name='collect_data_node', output='screen',
        condition=IfCondition(LaunchConfiguration('collector')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_dir': LaunchConfiguration('output_dir'),
            'csv_filename': LaunchConfiguration('csv_filename'),
        }])

    # 鍵盤那一端要另開 terminal: ros2 run car_teleop teleop_key
    teleop = Node(
        package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('teleop')),
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription(
        args + [camera, static_lidar, lidar, imu, wheel, fusion, collector, teleop])
