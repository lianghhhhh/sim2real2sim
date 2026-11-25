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
| `car_loc_fusion` | `fus_*` (絕對量測吃 8 台相機的車框與 lid_ 的輸出) |
| `car_loc_multicam` | `mc_*` (方法六: 同一份車框, 不吃 LiDAR) |

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
   (它照樣跑, 只剩多相機 + 遞推)。

5. **fusion 吃的是 8 台相機的車框與 lidar 的 topic。** `lidar:=false` 或 `multicam:=false`
   的話 `fus_` 跟著少一個來源; `camera:=false` 不影響它 —— 單一天花板相機現在只是
   獨立的 `cam_` 那一條。stage 沒有環境相機 (只有 kros_car_new.usd 有) 的話 `fus_` 只剩
   LiDAR + 遞推, 這時給 `multicam:=false` 可以省下偵測節點的 GPU。

6. **8 台相機只開一個偵測節點, `fus_` 與 `mc_` 共用。** 跟 LiDAR 一樣: `lid_` 那一個
   lidar_localizer 同時給 CSV 記、給 fusion 吃。`mc_` (方法六) 也在這裡開
   (`multicam_only`, 預設開), **不要再另外開 `multicam_loc.launch.py`** —— 它預設會再開
   一個偵測節點。2026-11-03 兩輪就是這樣: 兩個 YOLO 各訂閱 8 台 RELIABLE 影像, 82~89% 的
   影像被偵測兩次, 而且只有那兩輪 Isaac 的相機與 LiDAR 一起斷 (51 s / 130 s)。

只比其中幾條就把別的關掉, 例如不想跑天花板相機那一條:

    ros2 launch bringup_pkg collect_all.launch.py camera:=false

`mc_` 不想記 (省一點 CPU) 就 `multicam_only:=false`。
"""
import os
from typing import List

import yaml
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
    mc_params = os.path.join(
        get_package_share_directory('car_loc_multicam'), 'config', 'multicam_loc.yaml')
    with open(mc_params, encoding='utf-8') as f:
        multicam_base = yaml.safe_load(f)['multicam_fusion']['ros__parameters']
    with open(fus_params, encoding='utf-8') as f:
        fusion_base = yaml.safe_load(f)['fusion_localizer']['ros__parameters']

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='單一天花板相機 (cam_ 那一條), fusion 不吃它'),
        DeclareLaunchArgument('multicam', default_value='true',
                              description='YOLO 偵測節點 + fusion 吃 8 台相機的車框'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('imu', default_value='true'),
        DeclareLaunchArgument('wheel', default_value='true'),
        DeclareLaunchArgument('fusion', default_value='true',
                              description='方法五 —— 吃多相機的車框與 lidar 的輸出, '
                                          '那兩條關掉的話它就少一個來源'),
        DeclareLaunchArgument('multicam_only', default_value='true',
                              description='方法六 (mc_) —— 吃同一個偵測節點的框, 不吃 LiDAR。'
                                          '不要另外開 multicam_loc.launch.py (會多一個偵測節點)'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='cmd_vel -> joint_command 的速度控制層'),
        DeclareLaunchArgument('tf_source', default_value='fusion',
                              description='誰來發 map->base_link: '
                                          'fusion | multicam | lidar | camera | imu | wheel | none'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='空 = car_loc_lidar/maps/room.yaml'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
        # laser_frame 相對 base_link 的位置 (kros_car.usd); 舊的 car.usd 是 x 0 / z 0.20
        DeclareLaunchArgument('lidar_x', default_value='0.045'),
        DeclareLaunchArgument('lidar_z', default_value='0.236'),
        DeclareLaunchArgument('imu_initial_pose', default_value='[0.0, 0.0, 0.0]',
                              description='x, y, yaw(度)。imu_ 與 whl_ 兩條的起點, '
                                          '要對到車子的實際出生點。fusion 不需要'),
        DeclareLaunchArgument('collector', default_value='true',
                              description='開 collect_data_node。friction_test.launch.py '
                                          '會關掉 (calibrate_env_node 自己帶一個)'),
        DeclareLaunchArgument('output_dir', default_value='/workspaces/car_run_data'),
        DeclareLaunchArgument('csv_filename', default_value='all_loc.csv'),
        # ---- 感測器 topic 與 IMU 模式 (imu_ / whl_ / fus_ 三條與 collector 共用) ----
        # 預設值跟 imu_loc.yaml / wheel_loc.yaml / fusion_loc.yaml 一樣 (模擬)。
        # 實體車: imu_topic:=/imu/data joint_states_topic:=/wheel_feedback
        #         gravity_mode:=complementary yaw_source:=gyro fusion_sigma_acc:=0.35
        DeclareLaunchArgument('imu_topic', default_value='/imu',
                              description='實體車是 /imu/data'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states',
                              description='輪速 / 扭矩回授; 實體車用 /wheel_feedback'),
        DeclareLaunchArgument('gravity_mode', default_value='orientation',
                              description='實體車 6 軸 IMU 用 complementary'),
        DeclareLaunchArgument('yaw_source', default_value='imu_orientation',
                              description='實體車用 gyro'),
        DeclareLaunchArgument('fusion_sigma_acc', default_value='0.05',
                              description='fus_ 的加速度過程雜訊; complementary 模式要 0.35'),
        # 2026-11-23 實體車 lab.yaml: LiDAR 正常追蹤時回報的 sigma 中位數 2.3~3.4 mm (p90 最高 4.4 mm),
        # 比模擬的 ~1.5 mm 大, 預設 2.5 mm 會擋掉 36~100% 的量測 (融合等不到起點 -> 沒有 map TF)。
        # 追丟時是 14.5 mm (模擬) / 70 mm (實體車全域定位中)。
        # 2026-11-24 實體車給 0.006 不夠: 配準殘差 4.9 cm (sigma 跟殘差成正比) 時 sigma ~6.6 mm, 融合擋掉 68%
        # (sigma擋 1313 / 用 625), 車子停著的位置一直超標 -> MPC「絕對量測斷了」停 103 s。改 0.012 後 0 s。
        # 實體車給 0.012: 仍遠低於追丟的 70 mm (另外還有 lidar_lock_* 的鎖死偵測與融合的卡方閘門)。
        DeclareLaunchArgument('fusion_lidar_sigma_max', default_value='0.0025',
                              description='fus_ 擋 LiDAR 量測的 sigma 上限 (m); 0 = 不擋'),
        DeclareLaunchArgument('wheel_scale', default_value='1.0',
                              description='whl_ / fus_ 的有效輪半徑比例 (實際輪徑 / 節點的 wheel_radius 0.051356)'),
        # 絕對量測的常數延遲 (s), 加回量測的時戳上 (見 car_loc_fusion/sources.py 「時鐘偏移」那段:
        # 時戳跟 /clock 不同基準時, 偏移鎖定會把真正的延遲一起吃掉, 只能這樣手動補)。
        # 2026-11-17 kros_car_mpc_sensor.usd 上 8 台相機的融合位置落後 GT 約 5 cm, 但大部分是固定偏移
        # (−5.25 cm − 0.045 s × v), 不是延遲 —— 車型 car_model.yaml 的問題, 補延遲只在等速時剛好對。
        DeclareLaunchArgument('bbox_extra_delay', default_value='0.0',
                              description='8 台相機車框的常數延遲 (s), fus_ 與 mc_ 都吃'),
        DeclareLaunchArgument('lidar_extra_delay', default_value='0.0',
                              description='/lidar_loc/odom 的常數延遲 (s)'),
        # 2026-11-19 實體車 lab.yaml: 正確位姿的穿牆比例就有 4.7~10.9% (地圖的牆是稀疏的雜物點,
        # 射線從縫隙穿過去), 預設 3% 會把每個全域定位的解都擋掉、追蹤中也一直判成鎖死。
        # 實體車給 0.0 關掉 (lidar_localizer 退回只看殘差 / inlier)。
        DeclareLaunchArgument('lidar_lock_see_through', default_value='0.03',
                              description='lidar_localizer 的 lock_see_through; 0 = 不檢查穿牆射線'),
        # 2026-11-23 實體車 lab.yaml: 正確位姿的殘差 2.7~5 cm (11/19 是 0.5~2 cm, 現場跟建圖時不一樣),
        # 預設 2.5 cm 在 2 分鐘內誤判鎖死 75 次 -> 每次整張地圖重新全域定位 + 10 幀 sigma 5 cm
        # -> 融合擋掉 -> MPC「絕對量測斷了」停了 47 s。實體車給 lidar_lock_residual:=0.06
        # lidar_lock_inlier:=0.80: 那 75 次只剩開機頭 2 秒 3 次 (殘差 7.2~8.0 cm, 位置真的偏了 0.6 m)。
        # 模擬裡真的鎖到 180 度是 inlier 52~77%, 0.80 仍抓得到。
        DeclareLaunchArgument('lidar_lock_residual', default_value='0.025',
                              description='lidar_localizer 的 lock_residual (m): 殘差連續超過就判鎖死'),
        DeclareLaunchArgument('lidar_lock_inlier', default_value='0.85',
                              description='lidar_localizer 的 lock_inlier: inlier 連續低於就判鎖死'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')
    initial_pose = ParameterValue(
        LaunchConfiguration('imu_initial_pose'), value_type=List[float])
    imu_mode = {
        'imu_topic': LaunchConfiguration('imu_topic'),
        'gravity_mode': LaunchConfiguration('gravity_mode'),
        'yaw_source': LaunchConfiguration('yaw_source'),
    }
    wheel_input = {
        'joint_states_topic': LaunchConfiguration('joint_states_topic'),
        'wheel_scale': ParameterValue(LaunchConfiguration('wheel_scale'), value_type=float),
    }

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
        arguments=['--x', LaunchConfiguration('lidar_x'), '--y', '0',
                   '--z', LaunchConfiguration('lidar_z'),
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
            'lock_see_through': ParameterValue(
                LaunchConfiguration('lidar_lock_see_through'), value_type=float),
            'lock_residual': ParameterValue(
                LaunchConfiguration('lidar_lock_residual'), value_type=float),
            'lock_inlier': ParameterValue(
                LaunchConfiguration('lidar_lock_inlier'), value_type=float),
        }])

    imu = Node(
        package='car_loc_imu', executable='imu_localizer',
        name='imu_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('imu')),
        parameters=[imu_params, {
            'use_sim_time': use_sim_time,
            'publish_tf': owns_tf('imu'),
            'initial_pose': initial_pose,
            **imu_mode,
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
            **imu_mode, **wheel_input,
        }])

    # 8 台相機的偵測, **只有這一個**: fusion (fus_) 與方法六 (mc_) 共用同一份框。
    # ROI 跟著融合的輸出走; 沒開 fusion 時跟方法六的。
    fusion_on = ["'", LaunchConfiguration('fusion'), "' == 'true'"]
    detector = Node(
        package='car_loc_multicam', executable='multicam_detector',
        name='multicam_detector', output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('multicam'), "' == 'true' and ('",
             LaunchConfiguration('fusion'), "' == 'true' or '",
             LaunchConfiguration('multicam_only'), "' == 'true')"])),
        parameters=[mc_params, {
            'use_sim_time': use_sim_time,
            'odom_topic': PythonExpression(
                ["'/fusion_loc/odom' if "] + fusion_on + [" else '/multicam_loc/odom'"]),
        }])

    # **參數一律先在這裡合併成一個 dict 再交給節點**, 不要寫成 [dict, yaml 檔, 覆寫 dict]。
    # ROS 2 套參數時是照「節點名第一次出現的順序」: 第一個是 dict (/**) 的話, 後面 yaml 檔
    # 裡指名 fusion_localizer: 的那一段會蓋掉**所有** /** 的覆寫, 包括排在最後的那個 ——
    # 2026-11-03 實測 tf_source:=none 時融合照發 TF, rewind:=false / gravity_mode 之類的
    # launch 參數也都沒生效。
    # 方法五。它訂閱 /env_cam_i/car_bbox 與 /lidar_loc/odom, 所以要放在同一份 launch
    # 裡才拿得到 —— 但它**不需要** initial_pose (第一組車框就是起點)。
    # 節點本體在 car_loc_multicam, 參數的疊法見 fusion_loc.launch.py。
    fusion = Node(
        package='car_loc_multicam', executable='multicam_fusion',
        name='fusion_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('fusion')),
        parameters=[{
            **multicam_base, **fusion_base,
            'use_sim_time': use_sim_time,
            'publish_tf': owns_tf('fusion'),
            **imu_mode, **wheel_input,
            'sigma_acc': ParameterValue(
                LaunchConfiguration('fusion_sigma_acc'), value_type=float),
            'multicam_enabled': ParameterValue(
                LaunchConfiguration('multicam'), value_type=bool),
            'lidar_enabled': ParameterValue(
                LaunchConfiguration('lidar'), value_type=bool),
            'lidar_sigma_max': ParameterValue(
                LaunchConfiguration('fusion_lidar_sigma_max'), value_type=float),
            'bbox_extra_delay': ParameterValue(
                LaunchConfiguration('bbox_extra_delay'), value_type=float),
            'lidar_extra_delay': ParameterValue(
                LaunchConfiguration('lidar_extra_delay'), value_type=float),
        }])

    # 方法六 (mc_): 同一個節點類別, 只差不吃 LiDAR —— 拿 mc_ 跟 fus_ 比就是「LiDAR 值多少」。
    # 參數疊法跟 multicam_loc.launch.py 一樣 (fusion_loc.yaml -> multicam_loc.yaml -> 這裡)。
    multicam_only = Node(
        package='car_loc_multicam', executable='multicam_fusion',
        name='multicam_fusion', output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('multicam'), "' == 'true' and '",
             LaunchConfiguration('multicam_only'), "' == 'true'"])),
        parameters=[{
            **fusion_base, **multicam_base,
            'use_sim_time': use_sim_time,
            'publish_tf': owns_tf('multicam'),
            **imu_mode, **wheel_input,
            'sigma_acc': ParameterValue(
                LaunchConfiguration('fusion_sigma_acc'), value_type=float),
            'lidar_enabled': False,
            'publish_markers': False,
            'bbox_extra_delay': ParameterValue(
                LaunchConfiguration('bbox_extra_delay'), value_type=float),
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
            'imu_topic': LaunchConfiguration('imu_topic'),
            'wheel_feedback_topic': LaunchConfiguration('joint_states_topic'),
        }])

    # 鍵盤那一端要另開 terminal: ros2 run car_teleop teleop_key
    teleop = Node(
        package='car_teleop', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('teleop')),
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription(
        args + [camera, static_lidar, lidar, imu, wheel, detector, fusion, multicam_only,
                collector, teleop])
