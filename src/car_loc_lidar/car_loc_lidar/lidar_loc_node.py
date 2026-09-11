#!/usr/bin/env python3
"""方法二: 只用 LiDAR 對既有地圖定位。

    /scan (Oradar MS200) ──> 運動補償 ──┐
                                        ├─> scan-to-map 3 自由度配準
    地圖 (手動開車用 slam_toolbox 建的) ─┘             │
                                                               v
                        /lidar_loc/odom + /lidar_loc/pose + TF map->base_link

**這個節點不訂閱 /imu, 不訂閱任何相機 topic。** 三條定位路線要能互相當對照,
就不能偷用彼此的資料。

沒有 IMU 之後改變的三件事
-------------------------
1. **yaw 要自己解。** 配準從 2 自由度變成 3 自由度。yaw 誤差會透過「距離 x 角度」
   放大成位置誤差 —— 10 m 外的牆差 0.5 度就是 8.7 cm。這是方法二真正的代價。
2. **預測只剩等速模型。** 有 IMU 的時候下一幀的朝向是量出來的; 這裡是猜的。
   急轉彎時預測會落後, 所以 LM 的單步上限與 Huber 門檻都要留餘裕。
3. **運動補償用自己估的速度。** 一圈掃描 50 ms, 這台車能轉到 20 rad/s, 那 50 ms
   裡會轉超過 50 度 —— 不補償配準必錯。速度是從上一段配準結果差分出來的,
   所以配準爛 -> 速度爛 -> 補償爛 -> 配準更爛。斷這個迴圈的是
   `ConstVelMotion.coast/decay`: 配準失敗的那幾幀**不更新速度**。

地圖從哪裡來
------------
    ros2 launch car_loc_lidar mapping.launch.py      # 手動開一圈
    ros2 run nav2_map_server map_saver_cli -f .../maps/room
    ros2 launch car_loc_lidar lidar_loc.launch.py map_path:=.../maps/room.yaml
"""
from __future__ import annotations

import os

# BLAS 執行緒數一定要在 import numpy 之前設。這裡的矩陣都很小 (720x3), 但呼叫
# 得很密; OpenBLAS 預設開滿 nproc 個執行緒而且呼叫之間是**忙等**的, 會把同機的
# 其他節點餓死。鎖成單執行緒又快又不擾民。
for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_v, '1')

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_srvs.srv import Trigger

from .frontend import ScanFrontend
from .gridmap import GridMap
from .matcher import ScanMatcher, rot2
from .motion import ConstVelMotion
from .scan import to_laserscan

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class LidarLocalizer(Node):

    def __init__(self):
        super().__init__('lidar_localizer')
        p = self.declare_parameter

        # --- topics / frames -------------------------------------------------
        # 'scan' = 2D 雷射的 LaserScan (Oradar MS200, 模擬與實體車都是這條);
        # 'pointcloud' = 3D RTX LiDAR 的 PointCloud2 (換回多線雷射時用)
        p('input_type', 'scan')
        p('cloud_topic', '/lidar/point_cloud')
        p('scan_topic', '/scan')
        p('odom_topic', '/lidar_loc/odom')
        p('pose_topic', '/lidar_loc/pose')
        p('map_frame', 'map')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('publish_tf', True)
        # 'direct'      : 直接發 map -> base_link (這條路沒有別的里程計來源, 預設)
        # 'map_to_odom' : 只發 map -> odom, 假設別人在發 odom -> base_link (nav2 慣例)
        p('tf_mode', 'direct')

        # --- 外參 (從 car.usd 量出來的, 不要憑感覺改) -------------------------
        # /World/small_car/Cube/oradar_ms200 的世界高度 = 0.200 m, 旋轉是單位矩陣。
        # scripts/setup_oradar_lidar.py 每次都會把實際量到的值印出來, 兩邊要一致。
        # 放低於 0.15 會被輪子擋掉下半視野 (上一顆雷射踩過這個坑)。
        p('lidar_translation', [0.0, 0.0, 0.20])
        p('lidar_rpy_deg', [0.0, 0.0, 0.0])

        # --- 地圖 -------------------------------------------------------------
        # 空 = 用 package 內附的 maps/room.yaml (建完圖 map_saver_cli 存進去的)
        p('map_path', '')

        # --- 點雲過濾 ----------------------------------------------------------
        # MS200 的量程是 0.03~12 m。range_max 要**比感測器上限大一點**, 不然
        # 11.99 m 的牆被雜訊推到 12.01 就整個被丟掉; 真的沒打到東西的射線是
        # inf/0, 本來就會被 finite 與 range_min 兩個條件擋掉。
        # range_min 一定要 > 0 —— 無效回波的座標是 0, 靠它濾。
        p('range_min', 0.05)
        p('range_max', 12.5)
        # z 是**感測器座標**的高度 —— 沒有 IMU 就不知道車身傾角, 只能假設水平。
        # 這個假設在平地成立; 斜坡或急煞俯仰時高度帶會跟著歪。
        p('z_min', 0.15)
        p('z_max', 0.90)
        # MS200 一圈 450 點 (4500 Hz / 10 Hz), 所以這個上限實際上不會觸發。
        # 留著是為了換更密的雷射時不用改程式。
        p('max_points', 1500)

        # --- 初始化 -----------------------------------------------------------
        p('global_init', True)      # true = 自己在整張地圖找, 不用填初始位姿
        p('initial_pose', [0.0, 0.0, 0.0])        # x, y, yaw(度)
        p('global_search_step', 0.30)
        p('global_search_clearance', 0.25)
        p('global_yaw_bins', 72)    # 5 度一格; 只有雷射的時候 yaw 也要搜

        # --- 配準 --------------------------------------------------------------
        p('huber', 0.10)
        p('max_iter', 40)
        p('inlier_dist', 0.30)
        p('min_inlier_ratio', 0.50)
        p('max_residual', 0.25)
        p('max_failures', 8)        # 連續失敗這麼多次就重新全域定位
        # 鎖死偵測 —— 鎖到 180 度對稱解時配準不會失敗, 上面那條永遠不會觸發。
        # 車子幾乎沒在轉的時候, 連續 lock_scans 幀殘差 > lock_residual 或
        # inlier < lock_inlier 就當作鎖死, 整張地圖重新全域定位。見 lidar_loc.yaml。
        p('lock_residual', 0.025)
        p('lock_inlier', 0.85)
        p('lock_omega', 0.5)
        p('lock_scans', 15)
        # 配準失敗時先在預測位姿附近掃一圈角度再試一次, 才算真的失敗。
        # 沒有 IMU 的時候「下一幀朝哪」是外推的, 車子原地自旋到 20 rad/s 時
        # 一幀就轉過 57 度, 角加速度一變預測就差幾十度 —— 那已經超出 refine 的
        # 收斂半徑。離線實測 (test/test_matcher.py, 20 rad/s 自旋 200 幀):
        #   不重試: 位置 RMS 114.60 cm, yaw RMS 91.80°, 122 幀失敗
        #   有重試: 位置 RMS   3.53 cm, yaw RMS  0.05°,   0 幀失敗 (只重試了 1 次)
        p('sweep_on_fail', True)
        p('sweep_yaw_span', 0.7)    # rad; 實際會跟 |omega|*period 取大的
        # 但要有上限。MS200 是 10 Hz, 一圈 100 ms —— 車子用 20 rad/s 自旋的話
        # |omega|*period*1.5 會算出 3.0 rad (172 度), 那已經接近「整圈都搜」,
        # 而長方形房間對 180 度幾乎對稱, 掃出來的解會鎖到反方向。
        # 超過這個範圍就不要硬掃, 報失敗去做全域定位比較安全。
        p('sweep_yaw_max', 1.2)
        p('sweep_xy_span', 0.3)     # m

        # --- 運動模型 -----------------------------------------------------------
        p('v_max', 4.0)
        p('omega_max', 25.0)
        p('twist_alpha', 0.5)       # 速度低通; 1 = 完全信最新的差分

        # --- 運動補償 -----------------------------------------------------------
        p('deskew', True)
        p('scan_period', 0.0)       # 0 = 從連續兩則訊息的時間差自己量
        p('scan_stamp', 'end')      # 'end' | 'mid' | 'start'
        # 索引方向 vs 時間方向。Isaac 的 laser_scan 是依**方位角遞增**排序的,
        # 而 MS200 是 CW 旋轉 -> 索引跟發射順序相反, 所以預設 'reverse'。
        # 見 scan.py 的說明。
        p('time_order', 'reverse')  # 'forward' | 'reverse'
        # 時戳對應到一圈的頭/中/尾, 再乘上索引方向, 一共 6 種組合, 規格書通常
        # 都沒寫。開這個就在車子轉得夠快的那幾幀上把 6 種都試一遍用殘差投票 ——
        # 猜錯的症狀很賊: 直線走完全正常, 一轉彎殘差就變兩倍。
        # !! 預設**關掉**: 投票只有角速度下限沒有上限, 跟摩擦力腳本 (自旋到
        #    12 rad/s) 一起跑時兩次都投錯 (2026-09-08, 2026-09-11)。見 lidar_loc.yaml。
        p('auto_scan_stamp', False)
        p('stamp_calib_scans', 15)
        p('stamp_calib_omega', 1.0)   # 角速度要大於這個 (rad/s) 才拿來校正
        p('stamp_calib_giveup', 600)

        # --- 輸出 --------------------------------------------------------------
        p('publish_scan', True)
        p('scan_out_topic', '/lidar_loc/scan')
        # MS200 一圈 450 點 (0.8 度)。bins 開太多只是多出一堆 inf。
        p('scan_out_bins', 450)
        p('publish_map', True)
        p('map_topic', '/map')
        p('publish_debug_cloud', False)
        p('debug_cloud_topic', '/lidar_loc/scan_matched')
        p('status_period', 2.0)

        g = self.get_parameter
        self.map_frame = g('map_frame').value
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value
        self.do_tf = bool(g('publish_tf').value)
        self.tf_mode = g('tf_mode').value
        self.min_inlier = float(g('min_inlier_ratio').value)
        self.max_residual = float(g('max_residual').value)
        self.max_failures = int(g('max_failures').value)
        self.lock_res = float(g('lock_residual').value)
        self.lock_inlier = float(g('lock_inlier').value)
        self.lock_omega = float(g('lock_omega').value)
        self.lock_scans = int(g('lock_scans').value)
        self.sweep_on_fail = bool(g('sweep_on_fail').value)
        self.sweep_yaw = float(g('sweep_yaw_span').value)
        self.sweep_yaw_max = float(g('sweep_yaw_max').value)
        self.sweep_xy = float(g('sweep_xy_span').value)
        self.v_max = float(g('v_max').value)
        self.global_step = float(g('global_search_step').value)
        self.global_clear = float(g('global_search_clearance').value)
        self.global_bins = int(g('global_yaw_bins').value)

        # --- 地圖 --------------------------------------------------------------
        path = g('map_path').value
        if not path:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('car_loc_lidar')
            for name in ('room.yaml', 'room.npz'):
                cand = os.path.join(share, 'maps', name)
                if os.path.exists(cand):
                    path = cand
                    break
        if not path or not os.path.exists(path):
            raise FileNotFoundError(
                '找不到地圖。先手動開車建一張:\n'
                '    ros2 launch car_loc_lidar mapping.launch.py\n'
                '    ros2 run nav2_map_server map_saver_cli -f '
                '/workspaces/src/car_loc_lidar/maps/room\n'
                '然後 colcon build, 或直接給 -p map_path:=/path/to/room.yaml')
        self.map = GridMap.load(path)
        self.get_logger().info(f'地圖 {path}: {self.map}')

        self.matcher = ScanMatcher(self.map,
                                   huber=float(g('huber').value),
                                   max_iter=int(g('max_iter').value),
                                   inlier_dist=float(g('inlier_dist').value))

        # --- 前端 --------------------------------------------------------------
        self.input_type = g('input_type').value
        self.front = ScanFrontend(
            input_type=self.input_type,
            translation=list(g('lidar_translation').value),
            rpy_deg=list(g('lidar_rpy_deg').value),
            range_min=float(g('range_min').value),
            range_max=float(g('range_max').value),
            z_min=float(g('z_min').value), z_max=float(g('z_max').value),
            max_points=int(g('max_points').value),
            scan_period=float(g('scan_period').value),
            scan_stamp=g('scan_stamp').value,
            do_deskew=bool(g('deskew').value),
            time_order=g('time_order').value)

        self.motion = ConstVelMotion(alpha=float(g('twist_alpha').value),
                                     v_max=self.v_max,
                                     omega_max=float(g('omega_max').value))

        # --- 狀態 --------------------------------------------------------------
        self.located = False
        self.failures = 0
        self.lock_count = 0          # 連續幾幀「沒在轉、殘差卻偏高」
        self.full_search = False     # 下一次全域定位不限半徑 (鎖死偵測觸發時)
        self.n_scan = 0
        self.n_ok = 0
        self.n_sweep = 0
        self.last_ok_t = None
        self.last_result = None
        self.match_ms = 0.0
        self.relocalize_req = bool(g('global_init').value)
        if not self.relocalize_req:
            ip = list(g('initial_pose').value)
            self.motion.set_pose(ip[0], ip[1], math.radians(ip[2]))
            self.located = True
            self.get_logger().info(
                f'初始位姿 x={ip[0]:.2f} y={ip[1]:.2f} yaw={ip[2]:.1f}°')

        # --- 時戳校正 ------------------------------------------------------------
        self.auto_stamp = bool(g('auto_scan_stamp').value)
        self.stamp_need = int(g('stamp_calib_scans').value)
        self.stamp_omega = float(g('stamp_calib_omega').value)
        self.stamp_giveup = int(g('stamp_calib_giveup').value)
        self.stamp_votes = {(o, k): 0.0
                            for o in ('forward', 'reverse')
                            for k in ('start', 'mid', 'end')}
        self.stamp_n = 0

        # --- ROS 介面 ------------------------------------------------------------
        self.pub_odom = self.create_publisher(Odometry, g('odom_topic').value, 10)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, g('pose_topic').value, 10)
        self.pub_scan = (self.create_publisher(LaserScan, g('scan_out_topic').value, 5)
                         if bool(g('publish_scan').value) else None)
        self.scan_bins = int(g('scan_out_bins').value)
        self.pub_dbg = (self.create_publisher(PointCloud2,
                                              g('debug_cloud_topic').value, 2)
                        if bool(g('publish_debug_cloud').value) else None)
        self.tf = tf2_ros.TransformBroadcaster(self) if self.do_tf else None

        if bool(g('publish_map').value):
            self.pub_map = self.create_publisher(
                OccupancyGrid, g('map_topic').value, LATCHED)
            self.create_timer(1.0, self._publish_map_once)

        if self.input_type == 'scan':
            self.create_subscription(LaserScan, g('scan_topic').value,
                                     self.on_scan, SENSOR_QOS)
            src = g('scan_topic').value
        else:
            self.create_subscription(PointCloud2, g('cloud_topic').value,
                                     self.on_scan, SENSOR_QOS)
            src = g('cloud_topic').value
        self.create_service(Trigger, '~/relocalize', self.on_relocalize)
        self.create_timer(float(g('status_period').value), self.status)
        self.get_logger().info(f'等 {src} ... (只用 LiDAR, 不吃 IMU / 相機)')

    # ------------------------------------------------------------------
    def _publish_map_once(self):
        seed = self.motion.pose[:2] if self.located else None
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.info.resolution = self.map.resolution
        m.info.height, m.info.width = self.map.shape
        m.info.origin.position.x = float(self.map.origin[0])
        m.info.origin.position.y = float(self.map.origin[1])
        m.info.origin.orientation.w = 1.0
        m.data = self.map.occupancy_data(seed).ravel().tolist()
        self.pub_map.publish(m)

    def on_relocalize(self, req, res):
        self.relocalize_req = True
        self.located = False
        res.success = True
        res.message = '下一幀會重新做一次全域定位'
        self.get_logger().warn(res.message)
        return res

    # ------------------------------------------------------------------
    def on_scan(self, msg):
        t = stamp_sec(msg.header.stamp)
        self.n_scan += 1

        f = self.front.process(msg, t, self.motion.vx, self.motion.vy,
                               self.motion.omega)
        if f.n_kept < 20:
            if self.n_scan % 20 == 1:
                self.get_logger().warn(
                    f'這一幀只剩 {f.n_kept} 個點 (原始 {f.n_raw})。'
                    f'高度帶 z[{self.front.z_min:.2f},{self.front.z_max:.2f}] '
                    f'跟 lidar_translation 對得上嗎?')
            return

        t0 = time.perf_counter()
        if not self.located:
            self._global(f)
        else:
            self._track(f, msg)
        self.match_ms = 0.9 * self.match_ms + 0.1 * (time.perf_counter() - t0) * 1e3

        if self.located:
            self.publish(t, f, msg)

    # ------------------------------------------------------------------
    def _global(self, f):
        """全域定位: 整張地圖找一次 (位置 x 角度)。"""
        center, radius = None, 0.0
        # 鎖死偵測觸發的時候**不能**設限: 鎖在 180 度對稱解時, 「上一個可信位置」
        # 本身就在房間對面, 在它附近找永遠找不到真值 (離線實測: 限 1 m 找回來的
        # 解離真值 2~9 m; 整張地圖找則是 0.5~1 cm)。
        if self.last_ok_t is not None and not self.full_search:
            # 追丟了要救回來的時候一定要設限: 車子最快 v_max, 追丟了 dt 秒,
            # 就不可能離開上一個可信位置 v_max*dt 以外。不設限的話, 格局重複的
            # 場地很容易在別的房間找到一個殘差同樣漂亮的解, 估計瞬間跳好幾公尺。
            dt = max(f.t - self.last_ok_t, 0.0)
            center = self.motion.pose[:2]
            radius = max(1.0, self.v_max * dt)
        t0 = time.perf_counter()
        r = self.matcher.global_localize(
            f.xy, step=self.global_step, clearance=self.global_clear,
            yaw_bins=self.global_bins, center=center, radius=radius)
        el = time.perf_counter() - t0

        if r.inlier_ratio >= self.min_inlier and r.residual <= self.max_residual:
            self.motion.set_pose(r.t[0], r.t[1], r.theta, f.t)
            self.located = True
            self.failures = 0
            self.lock_count = 0
            self.full_search = False
            self.last_ok_t = f.t
            self.last_result = r
            self.get_logger().info(
                f'全域定位成功 ({el:.2f} s): x={r.t[0]:+.3f} y={r.t[1]:+.3f} '
                f'yaw={math.degrees(r.theta):+.1f}° '
                f'殘差 {r.residual * 100:.2f} cm, inlier {r.inlier_ratio:.0%}'
                + (f' (限定在上一個位置 {radius:.1f} m 內)' if radius else ''))
        else:
            self.get_logger().warn(
                f'全域定位失敗 ({el:.2f} s): 殘差 {r.residual * 100:.1f} cm, '
                f'inlier {r.inlier_ratio:.0%}。車子在地圖涵蓋的範圍裡嗎?')

    # ------------------------------------------------------------------
    def _track(self, f, msg):
        px, py, pth = self.motion.predict(f.t)

        if self._calibrating():
            r = self._calibrate_stamp(f, msg, px, py, pth)
        else:
            r = self.matcher.refine(f.xy, [px, py], pth)

        if not self._good(r) and self.sweep_on_fail:
            # 掃一圈角度再試一次才算真的失敗。需要掃多大 = 上一段角速度 x
            # 一圈的時間, 也就是這一幀的預測最多可能錯多少。
            need = max(self.sweep_yaw,
                       abs(self.motion.omega_trusted) * self.front.period * 1.5)
            if need <= self.sweep_yaw_max:
                r = self.matcher.refine_sweep(f.xy, [px, py], pth, yaw_span=need,
                                              xy_span=self.sweep_xy)
                self.n_sweep += 1
            # need 超過上限就**不要掃**。用一個蓋不住真值的範圍去掃, 找到的是
            # 「範圍邊緣某個看起來還行的解」, 而長方形房間對 180 度幾乎對稱 ——
            # 離線實測 (test/test_matcher.py, MS200 10 Hz, 12 rad/s 自旋) 硬掃
            # 會鎖在 180 度反方向, 殘差還很漂亮, 再也回不來。
            # 這種時候直接算失敗, 讓 max_failures 觸發全域重定位才救得回來。

        if self._good(r):
            self.motion.update(f.t, r.t[0], r.t[1], r.theta)
            self.failures = 0
            self.n_ok += 1
            self.last_ok_t = f.t
            self.last_result = r
            self._check_lock(r)
            return

        self.failures += 1
        # 失敗的時候照等速推過去, 但**不更新速度** —— 拿一個由爛配準差分出來的
        # 速度去做下一幀的運動補償, 只會讓下一幀配得更爛。
        self.motion.coast(f.t)
        self.motion.decay(0.7)
        if self.failures == 1 or self.failures % 5 == 0:
            self.get_logger().warn(
                f'配準失敗 x{self.failures}: 殘差 {r.residual * 100:.1f} cm, '
                f'inlier {r.inlier_ratio:.0%}')
        if self.failures >= self.max_failures:
            self.get_logger().warn(
                f'連續失敗 {self.failures} 次, 重新做全域定位')
            self.located = False
            self.failures = 0

    def _good(self, r) -> bool:
        return bool(r.inlier_ratio >= self.min_inlier
                    and r.residual <= self.max_residual
                    and np.isfinite(r.residual))

    def _check_lock(self, r):
        """配準「成功」但其實鎖在 180 度對稱解 —— _good 擋不掉, 只能在這裡抓。

        轉彎時殘差本來就會變大 (補償誤差), 只在幾乎沒在轉的時候判斷;
        轉彎的幀不累加也不歸零, 讓「自旋完停下來」那幾秒接得上。
        """
        if abs(self.motion.omega) >= self.lock_omega:
            return
        if r.residual > self.lock_res or r.inlier_ratio < self.lock_inlier:
            self.lock_count += 1
        else:
            self.lock_count = 0
            return
        if self.lock_count >= self.lock_scans:
            self.get_logger().warn(
                f'疑似鎖在對稱解: 連續 {self.lock_count} 幀殘差 '
                f'{r.residual * 100:.1f} cm / inlier {r.inlier_ratio:.0%} '
                f'(正常 < {self.lock_res * 100:.1f} cm / > {self.lock_inlier:.0%}), '
                f'整張地圖重新全域定位')
            self.located = False
            self.full_search = True
            self.lock_count = 0

    # ------------------------------------------------------------------ 時戳校正
    def _calibrating(self) -> bool:
        return (self.auto_stamp and self.stamp_n < self.stamp_need
                and self.n_scan < self.stamp_giveup)

    def _calibrate_stamp(self, f, msg, px, py, pth):
        """車子轉得夠快的時候, 把 6 種組合都試一遍, 用配準殘差投票。

        6 種 = 時戳對應一圈的 (頭/中/尾) x 索引方向 (順/逆)。轉得慢的時候
        全部幾乎一樣 (補償量本來就小), 分不出來也沒關係 —— 分不出來就代表
        選錯也沒差。
        """
        if abs(self.motion.omega) < self.stamp_omega:
            return self.matcher.refine(f.xy, [px, py], pth)

        cur = (self.front.time_order, self.front.scan_stamp)
        best = None
        for key in self.stamp_votes:
            order, stamp = key
            g = self.front.process(msg, f.t, self.motion.vx, self.motion.vy,
                                   self.motion.omega, stamp_at=stamp,
                                   time_order=order)
            r = self.matcher.refine(g.xy, [px, py], pth)
            self.stamp_votes[key] += (r.residual if np.isfinite(r.residual)
                                      else 1.0)
            if key == cur:
                best = r
        self.stamp_n += 1
        if self.stamp_n >= self.stamp_need:
            win = min(self.stamp_votes, key=self.stamp_votes.get)
            avg = sorted((v / self.stamp_n * 100, k)
                         for k, v in self.stamp_votes.items())
            self.get_logger().info(
                f'掃描時序校正完成 ({self.stamp_n} 幀, 殘差由小到大): '
                + ', '.join(f'{o}/{s} {v:.2f} cm' for v, (o, s) in avg)
                + f' -> 用 {win[0]}/{win[1]}')
            self.front.time_order, self.front.scan_stamp = win
        return best if best is not None else self.matcher.refine(f.xy, [px, py], pth)

    # ------------------------------------------------------------------
    def publish(self, t, f, msg):
        x, y, th = self.motion.pose
        qz, qw = math.sin(th * 0.5), math.cos(th * 0.5)
        stamp = msg.header.stamp

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.map_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        cov = np.zeros((6, 6))
        r = self.last_result
        if r is not None and r.cov is not None:
            cov[0, 0], cov[0, 1] = r.cov[0, 0], r.cov[0, 1]
            cov[1, 0], cov[1, 1] = r.cov[1, 0], r.cov[1, 1]
            cov[5, 5] = r.cov[2, 2]
        else:
            cov[0, 0] = cov[1, 1] = 0.25
            cov[5, 5] = 0.1
        cov[2, 2] = cov[3, 3] = cov[4, 4] = 1e-6
        od.pose.covariance = cov.ravel().tolist()
        od.twist.twist.linear.x = float(self.motion.vx)
        od.twist.twist.linear.y = float(self.motion.vy)
        od.twist.twist.angular.z = float(self.motion.omega)
        self.pub_odom.publish(od)

        pc = PoseWithCovarianceStamped()
        pc.header = od.header
        pc.pose.pose = od.pose.pose
        pc.pose.covariance = od.pose.covariance
        self.pub_pose.publish(pc)

        if self.tf is not None:
            tfm = TransformStamped()
            tfm.header.stamp = stamp
            tfm.header.frame_id = self.map_frame
            tfm.child_frame_id = (self.base_frame if self.tf_mode == 'direct'
                                  else self.odom_frame)
            tfm.transform.translation.x = float(x)
            tfm.transform.translation.y = float(y)
            tfm.transform.rotation.z = qz
            tfm.transform.rotation.w = qw
            self.tf.sendTransform(tfm)

        if self.pub_scan is not None:
            self.pub_scan.publish(to_laserscan(
                LaserScan, f.xy, stamp, self.base_frame, bins=self.scan_bins,
                range_min=self.front.range_min, range_max=self.front.range_max))

        if self.pub_dbg is not None:
            self.pub_dbg.publish(self._debug_cloud(f.xy, stamp, x, y, th))

    def _debug_cloud(self, xy, stamp, x, y, th):
        """配準後的點雲 (地圖座標) —— 疊在地圖上看貼不貼, 比看數字快。"""
        from sensor_msgs.msg import PointField
        w = xy @ rot2(th).T + np.array([x, y])
        pts = np.zeros((w.shape[0], 3), dtype=np.float32)
        pts[:, :2] = w
        m = PointCloud2()
        m.header.stamp = stamp
        m.header.frame_id = self.map_frame
        m.height = 1
        m.width = pts.shape[0]
        m.fields = [PointField(name=n, offset=4 * i, datatype=7, count=1)
                    for i, n in enumerate(('x', 'y', 'z'))]
        m.is_bigendian = False
        m.point_step = 12
        m.row_step = 12 * pts.shape[0]
        m.data = pts.tobytes()
        m.is_dense = True
        return m

    # ------------------------------------------------------------------
    def status(self):
        if self.n_scan == 0:
            self.get_logger().warn('還沒有收到掃描')
            return
        x, y, th = self.motion.pose
        r = self.last_result
        res = f'{r.residual * 100:.2f} cm' if r else 'n/a'
        inl = f'{r.inlier_ratio:.0%}' if r else 'n/a'
        state = '定位中' if self.located else '**追丟, 正在重新全域定位**'
        self.get_logger().info(
            f'{self.n_scan} 幀 (成功 {100.0 * self.n_ok / max(self.n_scan, 1):.0f}%, '
            f'掃角度重試 {self.n_sweep}), '
            f'配準 {self.match_ms:.1f} ms | x={x:+.3f} y={y:+.3f} '
            f'yaw={math.degrees(th):+.1f}° v={math.hypot(self.motion.vx, self.motion.vy):.2f} '
            f'w={self.motion.omega:+.2f} | 殘差 {res}, inlier {inl} | {state}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
