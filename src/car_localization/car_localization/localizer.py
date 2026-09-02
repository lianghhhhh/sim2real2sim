#!/usr/bin/env python3
"""LiDAR + IMU 定位節點。

架構 (為什麼長這樣, 詳見 README):

    /imu  ──> ImuTrack ──┬─> 每個雷射點發射瞬間的車體姿態 (運動補償用)
                         └─> yaw (Isaac 的 IMU orientation 是精確值)
    /lidar/point_cloud ──> 濾地面/天花板 ──> 轉到世界軸向 ──┐
                                                            ├─> scan-to-map 配準
    地圖 (直接從 car.usd 幾何算出來, 沒有建圖誤差) ─────────┘        │
                                                                     v
                                              /localization/odom + TF map->base_link

三個關鍵決定:

1. 地圖不是用 SLAM 建的, 是直接從 USD 幾何切出來的。建圖誤差因此是 0,
   定位精度的上限只剩雷射雜訊。
2. yaw 直接吃 IMU 的 orientation, 掃描比對只解 x/y 兩個自由度。兩個自由度的
   最小平方在一個四面牆都看得到的房間裡是超定到不能再超定的問題。
3. 一整圈掃描是 50 ms 累積出來的, 這台車原地可以轉到 20 rad/s 以上, 那 50 ms
   裡車子會轉超過 50 度。所以每個點都各自用它發射瞬間的姿態去轉 (deskew),
   不是整圈共用一個姿態。不做這件事的話, 快轉時掃描圖形會被抹開, 配準必錯。
"""
from __future__ import annotations

import math
import os
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from std_srvs.srv import Trigger

from .gridmap import GridMap, voxel_downsample
from .imu_track import ImuTrack, matrix_to_quat, quat_to_matrix, quat_to_yaw, wrap_pi
from .matcher import ScanMatcher, rot2
from .pointcloud import (laserscan_to_xyz, pointcloud2_to_xyz,
                         stride_subsample, valid_mask)

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]], dtype=np.float64)


def rotz3(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class Localizer(Node):

    def __init__(self):
        super().__init__('car_localizer')
        p = self.declare_parameter

        # --- topics / frames -------------------------------------------------
        # 'pointcloud' = Isaac 的 RTX LiDAR; 'scan' = 一般 2D 雷射 (實體車)
        p('input_type', 'pointcloud')
        p('cloud_topic', '/lidar/point_cloud')
        p('scan_topic', '/scan')
        p('imu_topic', '/imu')
        p('odom_topic', '/localization/odom')
        p('pose_topic', '/localization/pose')
        p('map_frame', 'map')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('publish_tf', True)
        # 'direct'      : 直接發 map -> base_link (這個場景沒有別的里程計來源, 預設)
        # 'map_to_odom' : 只發 map -> odom, 假設別人在發 odom -> base_link (nav2 標準做法)
        p('tf_mode', 'direct')

        # --- 外參 (從 car.usd 量出來的, 不要憑感覺改) --------------------------
        # /World/small_car/Cube/World/multiScan136 的世界高度 = 0.200 m,
        # 旋轉是單位矩陣 (SICK 資產裡 sensor 相對於資產原點也是單位矩陣)。
        p('lidar_translation', [0.0, 0.0, 0.20])
        p('lidar_rpy_deg', [0.0, 0.0, 0.0])

        # --- 地圖 -------------------------------------------------------------
        p('map_path', '')                 # 空 = 用 package 內附的 car_usd.npz;
                                          # 也吃 nav2/slam_toolbox 的 .yaml
        # localize : 對既有地圖定位 (預設)
        # mapping  : 一邊配準一邊把地圖長出來 (hector 式, 沒有回環偵測)
        # odometry : 只對「最近幾個關鍵幀組成的滾動子圖」配準, 發 odom -> base_link。
        #            這是拿來餵 slam_toolbox / nav2 的那一層 —— 實體車有輪速計可以
        #            當這一層, 這台模擬車沒有, 就用雷射自己生一個。
        p('mode', 'localize')
        p('map_bounds', [-8.0, -6.0, 8.0, 6.0])   # mapping 用
        p('map_resolution', 0.05)                 # mapping 用
        p('map_save_path', '')
        p('map_insert_min_inlier', 0.60)
        # 殘差比這個大的幀不准寫進地圖。開頭幾幀速度還沒估出來, 運動補償的平移項
        # 是 0, 掃描會被抹開幾公分 —— 那幾幀寫進去就是一道鬼牆, 而且之後所有幀都
        # 會對齊到那道鬼牆上。
        p('map_insert_max_residual', 0.05)
        # --- odometry 模式的滾動子圖 ---
        p('keyframe_dist', 0.5)           # 走多遠加一個關鍵幀 (m)
        p('keyframe_angle', 0.35)         # 轉多少加一個關鍵幀 (rad)
        p('submap_keyframes', 15)         # 子圖保留最近幾個關鍵幀
        p('submap_radius', 15.0)          # 子圖只留車子附近這個半徑內的點 (m)
        p('submap_voxel', 0.05)           # 子圖的點先降採樣到這個間距

        # --- 點雲過濾 (z 是「世界座標」的高度, 地面在 0) -----------------------
        # 用世界 z 而不是感測器 z: 車子加速/煞車會俯仰, IMU 已經精確告訴我們
        # 傾斜多少, 拿它把點轉正之後再濾, 地面才切得乾淨。
        p('range_min', 0.30)
        p('range_max', 40.0)
        p('z_min', 0.15)                  # 高過地面, 濾掉地面回波
        p('z_max', 0.90)                  # 低於牆頂 (1.00), 避開掠過牆頂的回波
        p('max_points', 1500)

        # --- 初始化 -----------------------------------------------------------
        p('global_init', True)            # true = 自己在整張地圖找, 不用填初始位姿
        p('initial_pose', [0.0, 0.0, 0.0])          # x, y, yaw(度)
        p('global_search_step', 0.20)
        p('global_search_clearance', 0.25)

        # --- 配準 -------------------------------------------------------------
        p('huber', 0.10)
        p('max_iter', 30)
        p('inlier_dist', 0.30)
        p('min_inlier_ratio', 0.50)
        p('max_residual', 0.25)
        p('max_failures', 10)             # 連續失敗這麼多次就重新全域定位

        # --- yaw 來源 ---------------------------------------------------------
        # 'imu_orientation': 吃 IMU 的絕對姿態 (Isaac 裡是精確值) -> 只解 x/y
        # 'gyro'           : 陀螺儀積分 + 由掃描比對修正 -> 解 x/y/yaw (真車路線)
        p('yaw_source', 'imu_orientation')

        # --- 運動補償 ---------------------------------------------------------
        p('deskew', True)
        p('scan_period', 0.0)             # 0 = 從連續兩則訊息的時間差自己量
        p('scan_stamp', 'end')            # 'end' | 'mid' | 'start'
        p('auto_scan_stamp', True)        # 開頭趁車子在轉的時候自己驗哪個才對
        p('stamp_calib_scans', 20)
        p('stamp_calib_gyro', 2.0)    # 角速度要大於這個 (rad/s) 才拿來校正
        p('stamp_calib_giveup', 600)  # 等這麼多幀還沒轉夠快就放棄, 維持預設
        p('deskew_bins', 64)          # 時間分桶數的上限
        p('deskew_bin_angle', 0.02)   # 每個桶最多容許車子轉這麼多 rad (~1.15 度)

        # --- 輸出 -------------------------------------------------------------
        p('extrapolate', True)            # 兩次掃描之間用等速外推, 讓輸出跟 IMU 同頻
        p('status_period', 2.0)
        # 把運動補償後的掃描以 LaserScan 發出去, 給 slam_toolbox / nav2 這類
        # 只吃 LaserScan 的東西用。比 pointcloud_to_laserscan 好的地方是它已經
        # 去過畸變了 —— 邊轉邊掃出來的那一圈, 未補償的版本是歪的。
        p('publish_scan', False)
        p('scan_out_topic', '/scan')
        p('scan_out_bins', 720)
        p('publish_debug_cloud', False)
        p('debug_cloud_topic', '/localization/scan_matched')
        # 把地圖以 OccupancyGrid 發出去 (latched), rviz / Foxglove 直接看得到。
        # odometry 模式不發 —— 那個模式的地圖是滾動子圖, 而且 /map 是 slam_toolbox 的。
        p('publish_map', True)
        p('map_topic', '/map')

        g = self.get_parameter
        self.map_frame = g('map_frame').value
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value
        self.publish_tf = bool(g('publish_tf').value)
        self.tf_mode = g('tf_mode').value
        self.mode = g('mode').value
        self.yaw_source = g('yaw_source').value
        self.lock_yaw = (self.yaw_source == 'imu_orientation')
        self.range_min = float(g('range_min').value)
        self.range_max = float(g('range_max').value)
        self.z_min = float(g('z_min').value)
        self.z_max = float(g('z_max').value)
        self.max_points = int(g('max_points').value)
        self.deskew = bool(g('deskew').value)
        self.deskew_bins = max(1, int(g('deskew_bins').value))
        self.deskew_bin_angle = max(1e-3, float(g('deskew_bin_angle').value))
        self.scan_stamp = g('scan_stamp').value
        self.auto_stamp = bool(g('auto_scan_stamp').value) and self.deskew
        self.stamp_calib_scans = int(g('stamp_calib_scans').value)
        self.stamp_calib_gyro = float(g('stamp_calib_gyro').value)
        self.stamp_calib_giveup = int(g('stamp_calib_giveup').value)
        self.extrapolate = bool(g('extrapolate').value)
        self.min_inlier_ratio = float(g('min_inlier_ratio').value)
        self.max_residual = float(g('max_residual').value)
        self.max_failures = int(g('max_failures').value)
        self.map_insert_min_inlier = float(g('map_insert_min_inlier').value)
        self.map_insert_max_residual = float(g('map_insert_max_residual').value)
        self.input_type = g('input_type').value
        self.map_resolution = float(g('map_resolution').value)
        self.keyframe_dist = float(g('keyframe_dist').value)
        self.keyframe_angle = float(g('keyframe_angle').value)
        self.submap_radius = float(g('submap_radius').value)
        self.submap_voxel = float(g('submap_voxel').value)
        self.keyframes = deque(maxlen=max(2, int(g('submap_keyframes').value)))
        self.kf_pose = None
        # localize 模式的位姿是 map 座標; odometry 模式發的是 odom -> base_link
        self.pose_frame = (self.odom_frame if self.mode == 'odometry'
                           else self.map_frame)

        t = np.asarray(g('lidar_translation').value, dtype=np.float64)
        rpy = np.deg2rad(np.asarray(g('lidar_rpy_deg').value, dtype=np.float64))
        self.R_bl = rpy_to_matrix(*rpy)
        self.t_bl = t

        # --- 地圖 -------------------------------------------------------------
        self.gmap = self._load_map()
        self.matcher = ScanMatcher(self.gmap,
                                   huber=float(g('huber').value),
                                   max_iter=int(g('max_iter').value),
                                   inlier_dist=float(g('inlier_dist').value))

        # --- 狀態 -------------------------------------------------------------
        self.imu = ImuTrack()
        self.pos = np.zeros(2)            # 車子在 map 座標的位置
        self.vel = np.zeros(2)
        self.yaw_corr = 0.0               # 疊在 IMU 姿態上的 yaw 修正 (gyro 模式才會動)
        self.yaw_int = 0.0                # gyro 模式的積分 yaw
        self._yaw_hist_t = []
        self._yaw_hist_y = []
        self.initialized = False
        self.last_scan_t = None
        self.last_result = None
        self.fail_count = 0
        self.scan_count = 0
        self.scan_period = float(g('scan_period').value)
        self._scan_dt = []
        self._stamp_scores = {'end': [], 'mid': [], 'start': []}
        self._stamp_locked = not self.auto_stamp
        self._stamp_wait = 0
        self._t_pub = 0.0
        self._proc_ms = 0.0
        # 各 topic 的「訊息時戳 - 牆上時間」。Isaac 各 publisher 用的時間源不一定
        # 同一個 (IMU 走 sensorTime, LiDAR 走 simulationTime), 反覆 Stop/Play 之後
        # 可能差開幾百秒。差開的話運動補償會查到完全不相干的姿態, 而且不會報錯,
        # 只會安靜地變不準 —— 所以這裡主動量、主動吵。
        self._clock_off = {'imu': [], 'cloud': []}
        self._warned_clock = False

        init = np.asarray(g('initial_pose').value, dtype=np.float64)
        self.global_init = bool(g('global_init').value)
        if not self.global_init:
            self.pos = init[:2].copy()
            self.yaw_corr = 0.0
            self.yaw_int = math.radians(float(init[2]))
            self.initialized = True
        if self.mode in ('mapping', 'odometry'):
            self.global_init = False
            self.pos = init[:2].copy()
            self.yaw_int = math.radians(float(init[2]))
            self.initialized = True
            self.kf_pose = np.array([init[0], init[1], math.radians(float(init[2]))])

        # --- ROS 介面 ---------------------------------------------------------
        self.pub_odom = self.create_publisher(Odometry, g('odom_topic').value, 10)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, g('pose_topic').value, 10)
        self.pub_scan = None
        if bool(g('publish_scan').value):
            out_topic = g('scan_out_topic').value
            if self.input_type == 'scan' and out_topic == g('scan_topic').value:
                self.get_logger().warn(
                    f'publish_scan 的輸出 topic 跟輸入是同一個 ({out_topic}), '
                    '會自己餵自己 -> 關掉輸出。要用的話請指定不同的 scan_out_topic。')
            else:
                self.scan_bins = int(g('scan_out_bins').value)
                self.pub_scan = self.create_publisher(
                    LaserScan, out_topic, SENSOR_QOS)
        self.pub_dbg = None
        if bool(g('publish_debug_cloud').value):
            self.pub_dbg = self.create_publisher(
                PointCloud2, g('debug_cloud_topic').value, 1)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_mapviz = self.create_publisher(
            PointCloud2, '/localization/map_cloud', latched)
        self.pub_map = None
        if bool(g('publish_map').value) and self.mode != 'odometry':
            self.pub_map = self.create_publisher(
                OccupancyGrid, g('map_topic').value, latched)
        self._map_stamp = -1
        self.tf_bc = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Imu, g('imu_topic').value, self.on_imu, SENSOR_QOS)
        if self.input_type == 'scan':
            self.create_subscription(LaserScan, g('scan_topic').value,
                                     self.on_scan, SENSOR_QOS)
        else:
            self.create_subscription(PointCloud2, g('cloud_topic').value,
                                     self.on_cloud, SENSOR_QOS)
        self.create_service(Trigger, '~/relocalize', self.on_relocalize)
        self.create_service(Trigger, '~/save_map', self.on_save_map)
        self.create_timer(float(g('status_period').value), self.on_status)
        self.create_timer(2.0, self._publish_map_cloud)
        self.create_timer(1.0, self._publish_occupancy)

        self.get_logger().info(
            f'模式={self.mode}  yaw來源={self.yaw_source}  '
            f'運動補償={"開" if self.deskew else "關"}\n'
            f'  地圖: {self.gmap}\n'
            f'  LiDAR 外參: t={self.t_bl.tolist()} rpy(deg)={np.rad2deg(rpy).tolist()}\n'
            f'  初始化: {"全域搜尋" if self.global_init else f"指定 {init.tolist()}"}')

    # ================================================================== 地圖
    def _load_map(self) -> GridMap:
        path = self.get_parameter('map_path').value
        if self.mode == 'odometry':
            self.get_logger().info('里程計模式: 子圖由掃描自己長出來, 不載入地圖')
            return GridMap.empty([-2.0, -2.0, 2.0, 2.0],
                                 float(self.get_parameter('map_resolution').value),
                                 meta={'source': 'submap'})
        if self.mode == 'mapping' and not path:
            b = [float(v) for v in self.get_parameter('map_bounds').value]
            res = float(self.get_parameter('map_resolution').value)
            self.get_logger().info(f'建圖模式: 空白地圖 {b} @ {res} m')
            return GridMap.empty(b, res, meta={'source': 'mapping'})
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(get_package_share_directory('car_localization'),
                                    'maps', 'car_usd.npz')
            except Exception:
                path = ''
        path = os.path.expanduser(path)
        if not path or not os.path.exists(path):
            raise SystemExit(
                f'找不到地圖檔: {path!r}\n'
                '請先在 host 上產生一次 (不用開 Isaac):\n'
                '    ./scripts/make_map_from_usd.py\n'
                '然後在容器裡重新 colcon build。')
        g = GridMap.load(path)
        self.get_logger().info(f'載入地圖 {path}')
        return g

    # ================================================================== IMU
    def on_imu(self, msg: Imu):
        t = stamp_sec(msg.header.stamp)
        q = np.array([msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                         msg.angular_velocity.z])
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        last_t = self.imu.latest_time
        if math.isfinite(last_t) and t < last_t - 1.0:
            # 時間往回跳 = Isaac 被 Stop 之後重新 Play 了。舊資料全部作廢。
            self.get_logger().warn(f'IMU 時戳往回跳 {last_t - t:.1f} s '
                                   '(模擬重新開始?), 重置定位狀態')
            self.imu = ImuTrack()
            self._yaw_hist_t.clear()
            self._yaw_hist_y.clear()
            self._scan_dt.clear()
            self.last_scan_t = None
            self._t_pub = 0.0
            self.vel[:] = 0.0
            if self.global_init:
                self.initialized = False
            last_t = float('-inf')
        self._note_clock('imu', t)
        self.imu.add(t, q, gyro, acc)

        if self.yaw_source != 'imu_orientation' and math.isfinite(last_t):
            dt = t - last_t
            if 0.0 < dt < 0.5:
                self.yaw_int = wrap_pi(self.yaw_int +
                                       (gyro[2] - self.imu.gyro_bias[2]) * dt)
        self._yaw_hist_t.append(t)
        self._yaw_hist_y.append(self.yaw_int)
        if len(self._yaw_hist_t) > 4000:
            del self._yaw_hist_t[:2000]
            del self._yaw_hist_y[:2000]

        if self.initialized:
            self._publish(t)

    def _note_clock(self, key: str, stamp: float):
        buf = self._clock_off[key]
        buf.append(stamp - time.monotonic())
        if len(buf) > 200:
            del buf[:100]

    def _check_clocks(self, t_ref: float) -> bool:
        """雷射的時戳有沒有落在 IMU 緩衝的時間範圍裡。沒有的話兩邊時間源對不上,
        繼續算下去只會得到一個看起來很正常但其實是錯的位姿。"""
        lo, hi = self.imu.earliest_time, self.imu.latest_time
        if lo - 1.0 <= t_ref <= hi + 1.0:
            return True
        off = float('nan')
        if len(self._clock_off['imu']) > 10 and len(self._clock_off['cloud']) > 10:
            off = (float(np.median(self._clock_off['cloud']))
                   - float(np.median(self._clock_off['imu'])))
        self.get_logger().error(
            f'LiDAR 與 IMU 的時間源對不上: 掃描時戳 {t_ref:.3f}, '
            f'IMU 緩衝 [{lo:.3f}, {hi:.3f}], 估計常數偏移 {off:+.3f} s。\n'
            '  這通常是 Isaac 裡某些節點的 resetOnStop / resetSimulationTimeOnStop '
            '沒開, 反覆 Stop/Play 之後時鐘就分家了。\n'
            '  修法: 在 host 上跑 ./scripts/fix_car_usd_lidar.py, 然後重新載入場景。',
            throttle_duration_sec=10.0)
        return False

    def _yaw_int_at(self, t: float) -> float:
        if not self._yaw_hist_t:
            return self.yaw_int
        ts = self._yaw_hist_t
        i = min(max(np.searchsorted(ts, t), 1), len(ts) - 1)
        t0, t1 = ts[i - 1], ts[i]
        y0, y1 = self._yaw_hist_y[i - 1], self._yaw_hist_y[i]
        k = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        return y0 + k * wrap_pi(y1 - y0)

    def _rot_wb(self, t: float) -> np.ndarray:
        """車體 -> map 的旋轉矩陣。"""
        if self.yaw_source == 'imu_orientation':
            R = quat_to_matrix(self.imu.quat_at(t))
        else:
            R = rotz3(self._yaw_int_at(t))
        if abs(self.yaw_corr) > 1e-12:
            R = rotz3(self.yaw_corr) @ R
        return R

    def _yaw_at(self, t: float) -> float:
        if self.yaw_source == 'imu_orientation':
            return wrap_pi(quat_to_yaw(self.imu.quat_at(t)) + self.yaw_corr)
        return wrap_pi(self._yaw_int_at(t) + self.yaw_corr)

    # ================================================================== 雷射
    def on_cloud(self, msg: PointCloud2):
        xyz = pointcloud2_to_xyz(msg)
        self._process(xyz, stamp_sec(msg.header.stamp), msg.header.stamp,
                      use_z_filter=True)

    def on_scan(self, msg: LaserScan):
        # 2D 雷射本來就只有一個水平切片, 沒有「濾掉地面」這件事可做, 所以關掉
        # 高度過濾。實體車 (wildbot 的 oradar) 走的是這條路。
        xyz = laserscan_to_xyz(msg)
        rmin = max(self.range_min, float(msg.range_min))
        rmax = min(self.range_max, float(msg.range_max))
        self._process(xyz, stamp_sec(msg.header.stamp), msg.header.stamp,
                      use_z_filter=False, range_min=rmin, range_max=rmax)

    def _process(self, xyz, t_ref, stamp, use_z_filter=True,
                 range_min=None, range_max=None):
        self._note_clock('cloud', t_ref)
        if not self.imu.ready:
            return
        if not self._check_clocks(t_ref):
            return
        if self.last_scan_t is not None:
            dt = t_ref - self.last_scan_t
            if 0.0 < dt < 1.0:
                self._scan_dt.append(dt)
                if len(self._scan_dt) > 50:
                    self._scan_dt.pop(0)
        t0 = time.perf_counter()

        if xyz.shape[0] < 20:
            return
        n_raw = xyz.shape[0]
        ok = valid_mask(xyz,
                        self.range_min if range_min is None else range_min,
                        self.range_max if range_max is None else range_max)
        idx = np.nonzero(ok)[0]
        if idx.size < 50:
            self.get_logger().warn(f'有效點只有 {idx.size} 個 (共 {n_raw}), 跳過這幀',
                                   throttle_duration_sec=5.0)
            return
        sel = idx[stride_subsample(idx.size, self.max_points)]
        # 點的「索引比例」就是它在這一圈裡的發射時刻比例
        frac = sel.astype(np.float64) / max(n_raw - 1, 1)
        pts_l = xyz[sel]

        period = self._scan_period()
        pred = self._predict(t_ref)

        if self.auto_stamp and not self._stamp_locked and self.gmap.n_occupied > 0:
            self._calibrate_stamp(pts_l, frac, t_ref, period, pred, use_z_filter)

        base_xy = self._build_base_points(pts_l, frac, t_ref, period,
                                          self.scan_stamp, use_z_filter)
        if base_xy.shape[0] < 50:
            self.get_logger().warn(
                f'高度帶 [{self.z_min}, {self.z_max}] 裡只剩 {base_xy.shape[0]} 點。'
                '檢查 lidar_translation 或 z_min/z_max。', throttle_duration_sec=5.0)
            return

        if self.pub_scan is not None:
            self._publish_scan_out(base_xy, t_ref, stamp, period)

        # 自己建圖的模式, 第一幀沒有東西可以對 —— 直接把它當成起點的地圖。
        # (少了這一步會連續配準失敗, 然後掉進「對空地圖做全域定位」的死迴圈。)
        if self.mode in ('mapping', 'odometry') and self.gmap.n_occupied == 0:
            world = base_xy + self.pos
            if self.mode == 'odometry':
                self._add_keyframe(world, t_ref)
            else:
                self.gmap.ensure_bounds(world, margin=2.0)
                self.gmap.insert(world)
            self.last_scan_t = t_ref
            self.get_logger().info(
                f'{self.mode} 起點: x={self.pos[0]:+.3f} y={self.pos[1]:+.3f} '
                f'yaw={math.degrees(self._yaw_at(t_ref)):+.2f} deg '
                f'({self.gmap.n_occupied} 格)')
            return

        if not self.initialized:
            self._do_global_init(base_xy)
            self.last_scan_t = t_ref
            return

        res = self.matcher.refine(base_xy, pred, 0.0, lock_yaw=self.lock_yaw)
        good = (res.inlier_ratio >= self.min_inlier_ratio
                and res.residual <= self.max_residual)

        if good:
            world = base_xy @ rot2(res.delta).T + res.t
            self._accept(res.t, res.delta, res.inlier_ratio, res.residual, t_ref, world)
            self.fail_count = 0
        else:
            self.fail_count += 1
            self.get_logger().warn(
                f'配準不合格 (inlier {res.inlier_ratio:.2f}, '
                f'殘差 {res.residual * 100:.1f} cm), 這幀用推估的; '
                f'連續 {self.fail_count} 次', throttle_duration_sec=2.0)
            self.pos = pred
            if self.fail_count >= self.max_failures:
                if self.mode == 'odometry':
                    # 里程計沒有「全域」可以回去, 只能把子圖重開一個
                    self.get_logger().error('連續失敗太多次 -> 子圖重開')
                    self.keyframes.clear()
                else:
                    self.get_logger().error('連續失敗太多次 -> 重新全域定位')
                    self.initialized = False
                self.fail_count = 0

        self.last_scan_t = t_ref
        self.last_result = res
        self.scan_count += 1
        self._proc_ms = (time.perf_counter() - t0) * 1e3
        if self.pub_dbg is not None and good:
            self._publish_debug(base_xy, res, stamp)

    # ------------------------------------------------------------------
    def _scan_period(self) -> float:
        fixed = float(self.get_parameter('scan_period').value)
        if fixed > 0:
            return fixed
        if len(self._scan_dt) >= 5:
            return float(np.median(self._scan_dt))
        return 0.05          # SICK multiScan136 的 scanRateBaseHz = 20

    def _window(self, t_ref: float, period: float, mode: str):
        if not self.deskew:
            return t_ref, t_ref
        if mode == 'start':
            return t_ref, t_ref + period
        if mode == 'mid':
            return t_ref - period * 0.5, t_ref + period * 0.5
        return t_ref - period, t_ref          # 'end'

    def _build_base_points(self, pts_l, frac, t_ref, period, stamp_mode,
                           use_z_filter=True):
        """把雷射點轉成「以車體原點為中心、世界軸向」的 2D 點集。

        world_xy_i = pos + base_xy_i    <- 配準要解的就只剩 pos
        """
        t_lo, t_hi = self._window(t_ref, period, stamp_mode)
        t_pt = t_lo + frac * (t_hi - t_lo)

        pts_b = pts_l @ self.R_bl.T + self.t_bl          # lidar -> base_link

        # 分桶數跟著轉速走: 停著時 1 個桶就夠, 原地打轉時每個桶最多讓車子轉
        # deskew_bin_angle。固定 16 桶在 20 rad/s 下每桶要轉 3.6 度, 那個殘留的
        # 抹除量實測會讓誤差從 0.4 cm 變成 1.4 cm。
        if not self.deskew:
            nb = 1
        else:
            wz = abs(float(self.imu.gyro_at(t_ref)[2]))
            need = int(math.ceil(wz * max(t_hi - t_lo, 1e-6) / self.deskew_bin_angle))
            nb = int(min(max(need, 1), self.deskew_bins, pts_b.shape[0]))
        rot = np.empty_like(pts_b)
        if nb <= 1:
            rot = pts_b @ self._rot_wb(t_ref).T
        else:
            edges = np.linspace(0.0, 1.0, nb + 1)
            b = np.clip((frac * nb).astype(int), 0, nb - 1)
            for k in range(nb):
                m = b == k
                if not m.any():
                    continue
                tc = t_lo + 0.5 * (edges[k] + edges[k + 1]) * (t_hi - t_lo)
                rot[m] = pts_b[m] @ self._rot_wb(tc).T

        if use_z_filter:
            keep = (rot[:, 2] >= self.z_min) & (rot[:, 2] <= self.z_max)
        else:
            keep = np.ones(rot.shape[0], dtype=bool)
        base = rot[keep, :2]
        if self.deskew and self.extrapolate:
            # 掃描期間車子自己也在平移, 把每個點各自的平移補回去
            base = base + self.vel[None, :] * (t_pt[keep] - t_ref)[:, None]
        return base

    def _predict(self, t_ref: float) -> np.ndarray:
        if self.last_scan_t is None:
            return self.pos.copy()
        dt = t_ref - self.last_scan_t
        if not (0.0 < dt < 1.0):
            return self.pos.copy()
        if self.imu.is_still(t_ref):
            return self.pos.copy()
        return self.pos + self.vel * dt

    def _accept(self, new_pos, total_delta, inlier_ratio, residual, t_ref, world_pts):
        new = np.asarray(new_pos, dtype=np.float64).reshape(2)
        if self.last_scan_t is not None:
            dt = t_ref - self.last_scan_t
            if 0.0 < dt < 1.0:
                v = (new - self.pos) / dt
                # 速度只拿來做外推與運動補償, 平滑一點比較不會抖
                self.vel = 0.5 * self.vel + 0.5 * v
        self.pos = new
        if not self.lock_yaw:
            self.yaw_corr = wrap_pi(self.yaw_corr + total_delta)
        if (inlier_ratio < self.map_insert_min_inlier
                or residual > self.map_insert_max_residual):
            return
        if self.mode == 'mapping':
            # 地圖不夠大就自己長大, 所以不必事先知道房間多大
            self.gmap.ensure_bounds(world_pts, margin=2.0)
            self.gmap.insert(world_pts)
        elif self.mode == 'odometry':
            moved = float(np.linalg.norm(new - self.kf_pose[:2]))
            turned = abs(wrap_pi(self._yaw_at(t_ref) - self.kf_pose[2]))
            if moved > self.keyframe_dist or turned > self.keyframe_angle:
                self._add_keyframe(world_pts, t_ref)

    def _add_keyframe(self, world_pts, t_ref=None):
        """把這一幀併進滾動子圖。

        為什麼是「滾動」而不是一直累積: 一直累積就變成 hector SLAM, 誤差雖然比
        逐幀里程計小, 但沒有回環偵測, 走遠了地圖一樣會扭曲, 而且扭曲的地圖會反過來
        把位姿拉歪。odom -> base_link 這一層要的是「局部準、連續、不跳」, 全域一致
        那件事交給 slam_toolbox 的位姿圖去做, 兩邊各司其職。
        """
        self.keyframes.append(np.asarray(world_pts, dtype=np.float64))
        yaw = self._yaw_at(t_ref if t_ref is not None else self.imu.latest_time)
        self.kf_pose = np.array([self.pos[0], self.pos[1], yaw])

        pts = np.concatenate(list(self.keyframes), axis=0)
        if self.submap_radius > 0:
            d = np.linalg.norm(pts - self.pos, axis=1)
            pts = pts[d <= self.submap_radius]
        pts = voxel_downsample(pts, self.submap_voxel)
        if pts.shape[0] < 50:
            return
        # 這裡刻意用格點 EDT 而不是表面點的精確距離: 子圖每隔幾秒就要重建一次,
        # 精確版在大場景要幾百 ms。格點的半格量化偏差在這裡無害 —— 格點原點永遠
        # 對齊同一個全域格網, 所以重建前後偏差一致, 不會讓位姿跳動。
        self.gmap = GridMap.from_points(pts, resolution=self.map_resolution,
                                        margin=1.0, meta={'source': 'submap'})
        self.matcher.map = self.gmap

    def _do_global_init(self, base_xy):
        yaw_search = not self.lock_yaw
        self.get_logger().info(
            f'全域定位中 ({"搜尋 x/y/yaw" if yaw_search else "yaw 由 IMU 給定, 只搜 x/y"})...')
        t0 = time.perf_counter()
        res = self.matcher.global_localize(
            base_xy,
            step=float(self.get_parameter('global_search_step').value),
            clearance=float(self.get_parameter('global_search_clearance').value),
            yaw_search=yaw_search, lock_yaw=self.lock_yaw)
        ms = (time.perf_counter() - t0) * 1e3
        if res.inlier_ratio < self.min_inlier_ratio or res.residual > self.max_residual:
            self.get_logger().warn(
                f'全域定位還不夠好 (inlier {res.inlier_ratio:.2f}, '
                f'殘差 {res.residual * 100:.1f} cm, {ms:.0f} ms), 下一幀再試')
            return
        self.pos = res.t.copy()
        self.vel[:] = 0.0
        if not self.lock_yaw:
            self.yaw_corr = wrap_pi(self.yaw_corr + res.delta)
        self.initialized = True
        self.last_result = res
        self.get_logger().info(
            f'定位成功: x={self.pos[0]:+.3f} y={self.pos[1]:+.3f} '
            f'yaw={math.degrees(self._yaw_at(self.imu.latest_time)):+.2f} deg  '
            f'(殘差 {res.residual * 100:.2f} cm, inlier {res.inlier_ratio:.2f}, {ms:.0f} ms)')

    # ------------------------------------------------------------------ 時戳校正
    def _calibrate_stamp(self, pts_l, frac, t_ref, period, pred, use_z_filter=True):
        """Isaac 沒有說一整圈掃描的時戳是掃描的開始還是結束。猜錯的話運動補償會補到
        錯的方向, 快轉時比不補償還糟。與其猜, 不如趁車子在**快速旋轉**的時候把三種
        假設各跑一次配準, 看誰的殘差小。

        兩個門檻是有意義的, 不是隨便訂的:
          * 只在 |gyro_z| 夠大的時候取樣。轉得慢的時候三種假設本來就差不多,
            拿那種資料來選等於擲骰子 —— 實測 0.5 rad/s 的資料會讓三者殘差
            完全打平 (2.65 / 2.65 / 2.65 cm), 然後選到錯的那個。
          * 贏的那個要明顯贏過預設值才換。差距在雜訊等級以內就維持預設。
        """
        if not self.initialized:
            return
        if abs(self.imu.gyro_at(t_ref)[2]) < self.stamp_calib_gyro:
            self._stamp_wait += 1
            if self._stamp_wait > self.stamp_calib_giveup:
                self._stamp_locked = True
                self.get_logger().info(
                    f'車子一直沒有轉到 {self.stamp_calib_gyro:.1f} rad/s 以上, '
                    f'無法校正掃描時戳, 維持預設 scan_stamp="{self.scan_stamp}"。'
                    '(想校正的話讓車子原地快轉幾秒)')
            return
        for mode in self._stamp_scores:
            base = self._build_base_points(pts_l, frac, t_ref, period, mode,
                                           use_z_filter)
            if base.shape[0] < 50:
                return
            r = self.matcher.refine(base, pred, 0.0, lock_yaw=self.lock_yaw)
            self._stamp_scores[mode].append(
                r.residual if math.isfinite(r.residual) else 9.9)
        if len(self._stamp_scores[self.scan_stamp]) < self.stamp_calib_scans:
            return

        avg = {k: float(np.mean(v)) for k, v in self._stamp_scores.items()}
        cur = self.scan_stamp
        best = min(avg, key=avg.get)
        self._stamp_locked = True
        detail = ', '.join(f'{k}={v * 100:.2f} cm' for k, v in avg.items())
        if best != cur and avg[best] < avg[cur] * 0.8 - 0.003:
            self.scan_stamp = best
            self.get_logger().info(
                f'掃描時戳校正完成 ({detail}) -> 改用 scan_stamp="{best}"')
        else:
            self.get_logger().info(
                f'掃描時戳校正完成 ({detail}) -> 差距不夠明顯, '
                f'維持 scan_stamp="{cur}"')

    # ================================================================== 輸出
    def _publish(self, t: float):
        if t <= self._t_pub:
            return
        self._t_pub = t
        pos = self.pos.copy()
        if self.extrapolate and self.last_scan_t is not None:
            dt = t - self.last_scan_t
            if 0.0 <= dt < 0.5:
                pos = pos + self.vel * dt

        if self.yaw_source == 'imu_orientation':
            R = quat_to_matrix(self.imu.quat_at(t))
            if abs(self.yaw_corr) > 1e-12:
                R = rotz3(self.yaw_corr) @ R
            quat = matrix_to_quat(R)
        else:
            yaw = self._yaw_at(t)
            quat = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
            R = rotz3(yaw)

        stamp = rclpy.time.Time(seconds=t).to_msg()

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.pose_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(pos[0])
        od.pose.pose.position.y = float(pos[1])
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation.x = float(quat[0])
        od.pose.pose.orientation.y = float(quat[1])
        od.pose.pose.orientation.z = float(quat[2])
        od.pose.pose.orientation.w = float(quat[3])
        # twist 用車體座標 (ROS 規定 child_frame_id 那個座標系)
        v_body = R[:2, :2].T @ self.vel
        od.twist.twist.linear.x = float(v_body[0])
        od.twist.twist.linear.y = float(v_body[1])
        gz = self.imu.gyro_at(t)
        od.twist.twist.angular.z = float(gz[2])
        cov = self._pose_cov()
        od.pose.covariance[0] = cov[0]
        od.pose.covariance[7] = cov[1]
        od.pose.covariance[35] = cov[2]
        self.pub_odom.publish(od)

        pc = PoseWithCovarianceStamped()
        pc.header = od.header
        pc.pose.pose = od.pose.pose
        pc.pose.covariance = od.pose.covariance
        self.pub_pose.publish(pc)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.pose_frame
            tf.child_frame_id = (self.base_frame
                                 if self.tf_mode == 'direct' or self.mode == 'odometry'
                                 else self.odom_frame)
            tf.transform.translation.x = float(pos[0])
            tf.transform.translation.y = float(pos[1])
            tf.transform.rotation.x = float(quat[0])
            tf.transform.rotation.y = float(quat[1])
            tf.transform.rotation.z = float(quat[2])
            tf.transform.rotation.w = float(quat[3])
            self.tf_bc.sendTransform(tf)

    def _pose_cov(self):
        r = self.last_result
        if r is None or r.cov is None:
            return [0.25, 0.25, 0.10]
        base = [max(float(r.cov[0, 0]), 1e-6), max(float(r.cov[1, 1]), 1e-6),
                max(float(r.cov[2, 2]), 1e-6)]
        # 外推越久越不可信
        if self.last_scan_t is not None:
            dt = max(0.0, self._t_pub - self.last_scan_t)
            base[0] += (0.5 * dt) ** 2
            base[1] += (0.5 * dt) ** 2
        return base

    def _publish_scan_out(self, base_xy, t_ref, stamp, period):
        """把 base_xy (世界軸向、以車體為中心) 轉回 base_link, 打成 LaserScan。

        base_xy 每個點都已經用它自己發射瞬間的姿態轉過, 也補過掃描期間的平移,
        所以這裡轉回車體座標得到的就是「t_ref 這一瞬間的一圈掃描」—— 去過畸變的。
        """
        yaw = self._yaw_at(t_ref)
        pts = base_xy @ rot2(-yaw).T
        n = self.scan_bins
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        rng = np.hypot(pts[:, 0], pts[:, 1])
        inc = 2 * math.pi / n
        b = np.clip(((ang + math.pi) / inc).astype(int), 0, n - 1)
        out = np.full(n, np.inf)
        np.minimum.at(out, b, rng)          # 同一格取最近的一點

        m = LaserScan()
        m.header.stamp = stamp
        m.header.frame_id = self.base_frame
        m.angle_min = -math.pi
        m.angle_max = math.pi - inc
        m.angle_increment = inc
        m.time_increment = 0.0              # 已經補償過了, 對外就是同一瞬間
        m.scan_time = float(period)
        m.range_min = float(self.range_min)
        m.range_max = float(self.range_max)
        m.ranges = out.astype(np.float32).tolist()
        self.pub_scan.publish(m)

    def _publish_debug(self, base_xy, res, stamp):
        pts = base_xy @ rot2(res.delta).T + res.t
        xyz = np.concatenate([pts, np.zeros((pts.shape[0], 1))], axis=1)
        self.pub_dbg.publish(self._make_cloud(xyz, stamp))

    def _publish_map_cloud(self):
        if self.pub_mapviz.get_subscription_count() == 0:
            return
        pts = self.gmap.occupied_points()
        xyz = np.concatenate([pts, np.zeros((pts.shape[0], 1))], axis=1)
        self.pub_mapviz.publish(self._make_cloud(xyz, self.get_clock().now().to_msg()))

    def _publish_occupancy(self):
        """地圖沒變就不重發 (latched, 新的訂閱者還是收得到最後一則)。"""
        if self.pub_map is None:
            return
        key = self.gmap.n_occupied + 1000003 * self.gmap.occ.size
        if key == self._map_stamp:
            return
        self._map_stamp = key
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.pose_frame
        m.info.resolution = float(self.gmap.resolution)
        m.info.width = int(self.gmap.occ.shape[1])
        m.info.height = int(self.gmap.occ.shape[0])
        m.info.origin.position.x = float(self.gmap.origin[0])
        m.info.origin.position.y = float(self.gmap.origin[1])
        m.info.origin.orientation.w = 1.0
        seed = self.pos if self.initialized else None
        m.data = self.gmap.occupancy_data(seed).ravel().tolist()
        self.pub_map.publish(m)

    def _make_cloud(self, xyz, stamp) -> PointCloud2:
        m = PointCloud2()
        m.header.stamp = stamp
        m.header.frame_id = self.pose_frame
        m.height = 1
        m.width = int(xyz.shape[0])
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(('x', 'y', 'z'))]
        m.is_bigendian = False
        m.point_step = 12
        m.row_step = 12 * m.width
        m.is_dense = True
        m.data = np.asarray(xyz, dtype=np.float32).tobytes()
        return m

    # ================================================================== 其他
    def on_status(self):
        if not self.initialized:
            self.get_logger().info('等待第一次成功定位...')
            return
        r = self.last_result
        yaw = math.degrees(self._yaw_at(self.imu.latest_time))
        extra = ''
        if r is not None:
            extra = (f'  殘差 {r.residual * 100:5.2f} cm  inlier {r.inlier_ratio:.2f}  '
                     f'{r.n_points} 點  {r.iterations} 次迭代  {self._proc_ms:.1f} ms')
            if not self.lock_yaw:
                extra += (f'  yaw修正量 {math.degrees(r.delta):+.3f} deg'
                          f'  累積 {math.degrees(self.yaw_corr):+.2f} deg')
        if self.mode == 'odometry':
            extra += f'  子圖 {len(self.keyframes)} 幀/{self.gmap.n_occupied} 格'
        elif self.mode == 'mapping':
            extra += f'  地圖 {self.gmap.n_occupied} 格 {self.gmap.shape}'
        self.get_logger().info(
            f'x={self.pos[0]:+7.3f}  y={self.pos[1]:+7.3f}  yaw={yaw:+7.2f} deg  '
            f'v={np.linalg.norm(self.vel):5.2f} m/s{extra}')

    def on_relocalize(self, req, resp):
        self.initialized = False
        self.fail_count = 0
        self.vel[:] = 0.0
        resp.success = True
        resp.message = '下一幀會重新做全域定位'
        return resp

    def on_save_map(self, req, resp):
        path = self.get_parameter('map_save_path').value
        if self.mode == 'odometry':
            resp.success = False
            resp.message = ('里程計模式的子圖是滾動的, 不是完整地圖, 存了也沒用。'
                            '要建圖請用 mode:=mapping 或 slam_toolbox。')
            return resp
        if not path:
            resp.success = False
            resp.message = '沒有設定 map_save_path'
            return resp
        p = self.gmap.save(path)
        self.gmap.save_nav2(p[:-4] if p.endswith('.npz') else p)
        resp.success = True
        resp.message = f'已存到 {p}'
        self.get_logger().info(resp.message)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = Localizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        path = node.get_parameter('map_save_path').value
        if path and node.mode == 'mapping':
            p = node.gmap.save(path)
            node.gmap.save_nav2(p[:-4])
            print(f'\n地圖已存到 {p}')
            print(node.gmap.ascii_view())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
