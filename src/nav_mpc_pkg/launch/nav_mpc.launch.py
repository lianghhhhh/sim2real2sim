"""點目標 -> Nav2 規劃 -> MPC 循跡, 前方有障礙物就停車重新規劃。一個指令開完:

    # 模擬: Isaac 開 kros_car_mpc_sensor.usd (有房間 = 有 /scan) 按 Play
    ros2 launch nav_mpc_pkg nav_mpc.launch.py

    # 跟實體車一樣的定位組合 (沒有環境相機)
    ros2 launch nav_mpc_pkg nav_mpc.launch.py multicam:=false

    # 定位已經在別的 terminal 跑
    ros2 launch nav_mpc_pkg nav_mpc.launch.py localization:=false

開的東西:
  * sensor_mpc_pkg/sensor_mpc.launch.py: 融合定位 (localization:=true 時) + actuator_model (模擬) +
    path_tracker (路線改從 /nav_mpc/path 來, 不封閉, 順著走) + sensor_mpc_node (吃 /nav_mpc/hold)
  * Nav2: nav_map_server (地圖 -> /nav_mpc/map) + planner_server (全域 costmap + Smac Hybrid-A*) +
    lifecycle_manager
  * nav_mpc_manager: 收目標、叫規劃器、看前方的掃描、發停車訊號

sensor_mpc.launch.py 的參數 (multicam、lidar、imu_topic、wheel_feedback_topic、gt_topic、geofence_* ...)
直接在這行指令給就會傳下去 (launch 參數對 include 進來的檔案也有效)。
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nav2_and_manager(context):
    share = get_package_share_directory('nav_mpc_pkg')
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    use_sim_time = lc('use_sim_time').lower() in ('true', '1')

    nav_map = lc('nav_map') or lc('map_path') or os.path.join(
        get_package_share_directory('car_loc_lidar'), 'maps', 'room.yaml')

    # ---- Nav2 參數: 填地圖 / use_sim_time / Humble 的 plugin 名稱, 寫成暫存檔 ----
    with open(os.path.join(share, 'config', 'nav2_params.yaml'), encoding='utf-8') as f:
        nav2 = yaml.safe_load(f)
    nav2['nav_map_server']['ros__parameters']['yaml_filename'] = nav_map
    planner = nav2['planner_server']['ros__parameters']
    if os.environ.get('ROS_DISTRO', '') == 'humble':
        for pid in planner['planner_plugins']:
            planner[pid]['plugin'] = planner[pid]['plugin'].replace('::', '/')
    if lc('minimum_turning_radius'):
        planner['SmacHybrid']['minimum_turning_radius'] = float(lc('minimum_turning_radius'))
    gc = nav2['global_costmap']['global_costmap']['ros__parameters']
    gc['obstacle_layer']['scan']['topic'] = lc('scan_topic')
    if lc('robot_radius'):
        gc['robot_radius'] = float(lc('robot_radius'))
    for node in ('nav_map_server', 'planner_server', 'lifecycle_manager_nav_mpc'):
        nav2[node]['ros__parameters']['use_sim_time'] = use_sim_time
    gc['use_sim_time'] = use_sim_time
    tmp = tempfile.NamedTemporaryFile('w', prefix='nav_mpc_nav2_', suffix='.yaml', delete=False)
    yaml.safe_dump(nav2, tmp, allow_unicode=True, sort_keys=False)
    tmp.close()

    with open(os.path.join(share, 'config', 'nav_mpc.yaml'), encoding='utf-8') as f:
        mgr = yaml.safe_load(f)['nav_mpc_manager']['ros__parameters']
    mgr.update({
        'use_sim_time': use_sim_time,
        'odom_topic': lc('odom_topic'),
        'scan_topic': lc('scan_topic'),
        'planner_id': lc('planner_id'),
        'fallback_planner_id': lc('fallback_planner_id'),
        'obstacle_check': lc('obstacle_check').lower() in ('true', '1'),
        'check_distance': float(lc('check_distance')),
        'corridor_half_width': float(lc('corridor_half_width')),
        'events_csv': lc('events_csv'),
        'align_heading': lc('align_heading').lower() in ('true', '1'),
        'align_tolerance_deg': float(lc('align_tolerance_deg')),
    })
    hyb = nav2['planner_server']['ros__parameters'].get(lc('planner_id'), {})
    mgr['planner_downsample'] = int(hyb.get('downsampling_factor', 1)) if hyb.get('downsample_costmap') else 1

    return [
        LogInfo(msg=f'[nav_mpc] 規劃地圖 {nav_map}, Nav2 參數 {tmp.name}'),
        Node(package='nav2_map_server', executable='map_server', name='nav_map_server',
             output='screen', parameters=[tmp.name]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[tmp.name]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_nav_mpc', output='screen', parameters=[tmp.name]),
        Node(package='nav_mpc_pkg', executable='nav_mpc_manager', name='nav_mpc_manager',
             output='screen', parameters=[mgr]),
    ]


def generate_launch_description():
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mpc', default_value='true',
                              description='開 sensor_mpc.launch.py (定位 + tracker + MPC)。false = 只開 Nav2 + 管理節點'),
        DeclareLaunchArgument('odom_topic', default_value='/fusion_loc/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='LiDAR 定位的地圖; 空 = car_loc_lidar/maps/room.yaml'),
        DeclareLaunchArgument('nav_map', default_value='',
                              description='規劃用的地圖; 空 = 跟 map_path 一樣'),
        DeclareLaunchArgument('planner_id', default_value='SmacHybrid', description='SmacHybrid | Grid'),
        DeclareLaunchArgument('fallback_planner_id', default_value=''),
        DeclareLaunchArgument('minimum_turning_radius', default_value='',
                              description='Hybrid-A* 的最小轉彎半徑 (m); 空 = nav2_params.yaml (0.5)'),
        DeclareLaunchArgument('robot_radius', default_value='',
                              description='costmap 的車子半徑 (m); 空 = nav2_params.yaml (0.45)'),
        DeclareLaunchArgument('obstacle_check', default_value='true'),
        DeclareLaunchArgument('align_heading', default_value='true',
                              description='到終點後原地旋轉對準目標方向'),
        DeclareLaunchArgument('align_tolerance_deg', default_value='5.0'),
        DeclareLaunchArgument('check_distance', default_value='1.5'),
        DeclareLaunchArgument('corridor_half_width', default_value='0.25'),
        DeclareLaunchArgument('events_csv', default_value='/workspaces/car_run_data/nav_mpc_events.csv',
                              description='已存在就自動加 _<時間>, 不覆寫; 也可以寫 {time}'),
        DeclareLaunchArgument('output_csv', default_value='/workspaces/car_run_data/nav_mpc_tracking.csv',
                              description='sensor_mpc_node 的逐週期 CSV'),
        DeclareLaunchArgument('hold_timeout', default_value='0.5',
                              description='s; /nav_mpc/hold 這麼久沒訊息 MPC 就停 (管理節點掛了)'),
    ]

    mpc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sensor_mpc_pkg'), 'launch', 'sensor_mpc.launch.py')),
        condition=IfCondition(lc('mpc')),
        launch_arguments={
            'use_sim_time': lc('use_sim_time'),
            'odom_topic': lc('odom_topic'),
            'map_path': lc('map_path'),
            'path_file': '',
            'path_topic': '/nav_mpc/path',
            'loop': 'false',
            'direction': 'forward',
            'hold_topic': '/nav_mpc/hold',
            'spin_topic': '/nav_mpc/spin',
            'hold_timeout': lc('hold_timeout'),
            'output_csv': lc('output_csv'),
        }.items())

    return LaunchDescription(args + [mpc, OpaqueFunction(function=_nav2_and_manager)])
