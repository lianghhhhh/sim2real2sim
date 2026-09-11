#!/usr/bin/env python3
"""純雷射里程計 —— 建圖時餵給 slam_toolbox 的那一層。

slam_toolbox 需要有人發 `odom -> base_link`。實體車那是輪速計 + EKF 給的; 這台
模擬車沒有輪速計, 而方法二又不准用 IMU, 所以這一層只能由雷射自己生出來:

    掃描 ──> 對「最近幾個關鍵幀組成的滾動子圖」配準 ──┬─> TF odom -> base_link
                                                      └─> /scan (已去畸變)
                                                                    │
                                                        slam_toolbox ┴─> /map + map->odom

為什麼是滾動子圖, 不是「對上一幀配準」
--------------------------------------
掃描對掃描 (scan-to-scan) 每一幀都只跟一幀比, 誤差是純隨機遊走, 走幾公尺就明顯
歪掉。對最近 N 個關鍵幀疊出來的子圖配準等於一次跟好幾百幀的資訊比對, 局部精度
高很多, 而且輸出連續不跳 —— 這正是 slam_toolbox 想要的里程計性質。

為什麼不直接讓這一層當定位輸出
------------------------------
它沒有回環偵測。走遠再繞回來時累積的誤差沒有任何機制分攤掉, 地圖會在接縫處
錯開。回環與全域最佳化交給 slam_toolbox, 這一層只負責「局部準、不跳」。

為什麼發自己的 /scan 而不是用 pointcloud_to_laserscan
------------------------------------------------------
因為這裡發的是**運動補償過**的一圈。車子邊轉邊掃出來的那一圈, 沒補償的版本是
歪的, 直接拿去建圖會建出一道糊掉的牆, 而且之後所有幀都會對齊到那道糊牆上。
"""
from __future__ import annotations

import os

for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_v, '1')

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_srvs.srv import Trigger

from .frontend import ScanFrontend
from .gridmap import GridMap, voxel_downsample
from .matcher import ScanMatcher, rot2
from .motion import ConstVelMotion
from .scan import to_laserscan

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class ScanOdometry(Node):

    def __init__(self):
        super().__init__('lidar_odometry')
        p = self.declare_parameter

        # 'scan' = Oradar MS200 的 LaserScan (模擬與實體車都是這條)
        p('input_type', 'scan')
        p('cloud_topic', '/lidar/point_cloud')
        p('scan_topic', '/scan')
        p('odom_topic', '/lidar_odom/odom')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('publish_tf', True)

        p('lidar_translation', [0.0, 0.0, 0.20])
        p('lidar_rpy_deg', [0.0, 0.0, 0.0])

        # MS200: 0.03~12 m。range_max 要比感測器上限大一點, 見 lidar_loc_node。
        p('range_min', 0.05)
        p('range_max', 12.5)
        p('z_min', 0.15)
        p('z_max', 0.90)
        # 2D 雷射一圈就 450 點, 這個上限不會觸發。換多線雷射時建圖可以多留一點
        # (建圖只做一次, 定位是每幀都要跑)。
        p('max_points', 3000)

        # --- 滾動子圖 ------------------------------------------------------------
        p('keyframe_dist', 0.4)      # 走多遠加一個關鍵幀 (m)
        p('keyframe_angle', 0.30)    # 轉多少加一個關鍵幀 (rad)
        p('submap_keyframes', 15)    # 子圖保留最近幾個關鍵幀
        p('submap_radius', 15.0)     # 子圖只留車子附近這個半徑內的點 (m)
        p('submap_voxel', 0.05)      # 子圖的點先降採樣到這個間距
        p('submap_resolution', 0.05)
        # 殘差比這個大的幀不准寫進子圖。開頭幾幀速度還沒估出來, 運動補償的平移項
        # 是 0, 掃描會被抹開幾公分 —— 那幾幀寫進去就是一道鬼牆, 而且之後所有幀
        # 都會對齊到那道鬼牆上。
        p('keyframe_max_residual', 0.06)
        p('keyframe_min_inlier', 0.60)

        p('huber', 0.10)
        p('max_iter', 40)
        p('inlier_dist', 0.30)
        p('min_inlier_ratio', 0.45)
        p('max_residual', 0.30)
        # 配準失敗時先掃一圈角度再試一次 —— 見 lidar_loc_node 的說明
        p('sweep_on_fail', True)
        p('sweep_yaw_span', 0.7)
        p('sweep_yaw_max', 1.2)     # 上限, 見 lidar_loc_node 的說明
        p('sweep_xy_span', 0.3)

        p('v_max', 4.0)
        p('omega_max', 25.0)
        p('twist_alpha', 0.5)

        p('deskew', True)
        p('scan_period', 0.0)
        p('scan_stamp', 'end')
        # 見 scan.py: Isaac 的 laser_scan 依方位角遞增排序, MS200 是 CW 旋轉,
        # 所以索引跟發射順序相反, 要 'reverse'。
        # !! **不要照 lidar_loc_node 的 auto_scan_stamp 投票結果改這裡** ——
        #    2026-09-08 照它改成 forward/mid, 地圖直接建爛 (見 lidar_odom.yaml)。
        #    唯一的驗收是重建地圖再量佔據範圍。
        p('time_order', 'reverse')

        p('publish_scan', True)
        # **不要發回 /scan** —— 那是感測器自己的 topic (實體車也是), 蓋掉會變成
        # 自己吃自己。slam_toolbox 改吃這個去畸變過的版本。
        p('scan_out_topic', '/scan_deskewed')
        p('scan_out_bins', 450)     # MS200 一圈 450 點 (0.8 度)
        p('status_period', 3.0)

        g = self.get_parameter
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value
        self.do_tf = bool(g('publish_tf').value)
        self.kf_dist = float(g('keyframe_dist').value)
        self.kf_angle = float(g('keyframe_angle').value)
        self.kf_max = int(g('submap_keyframes').value)
        self.submap_radius = float(g('submap_radius').value)
        self.submap_voxel = float(g('submap_voxel').value)
        self.submap_res = float(g('submap_resolution').value)
        self.kf_max_res = float(g('keyframe_max_residual').value)
        self.kf_min_inlier = float(g('keyframe_min_inlier').value)
        self.min_inlier = float(g('min_inlier_ratio').value)
        self.max_residual = float(g('max_residual').value)
        self.sweep_on_fail = bool(g('sweep_on_fail').value)
        self.sweep_yaw = float(g('sweep_yaw_span').value)
        self.sweep_yaw_max = float(g('sweep_yaw_max').value)
        self.sweep_xy = float(g('sweep_xy_span').value)

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
                                     v_max=float(g('v_max').value),
                                     omega_max=float(g('omega_max').value))
        self._match_cfg = dict(huber=float(g('huber').value),
                               max_iter=int(g('max_iter').value),
                               inlier_dist=float(g('inlier_dist').value))

        self.keyframes = []          # [(pose, world_xy), ...]
        self.submap = None
        self.matcher = None
        self.n_scan = 0
        self.n_ok = 0
        self.n_sweep = 0
        self.match_ms = 0.0
        self.last_result = None
        self.last_kf_pose = None

        self.pub_odom = self.create_publisher(Odometry, g('odom_topic').value, 10)
        self.pub_scan = (self.create_publisher(LaserScan, g('scan_out_topic').value, 5)
                         if bool(g('publish_scan').value) else None)
        self.scan_bins = int(g('scan_out_bins').value)
        self.tf = tf2_ros.TransformBroadcaster(self) if self.do_tf else None

        if self.input_type == 'scan':
            self.create_subscription(LaserScan, g('scan_topic').value,
                                     self.on_scan, SENSOR_QOS)
            src = g('scan_topic').value
        else:
            self.create_subscription(PointCloud2, g('cloud_topic').value,
                                     self.on_scan, SENSOR_QOS)
            src = g('cloud_topic').value
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_timer(float(g('status_period').value), self.status)
        self.get_logger().info(f'等 {src} ... (純雷射里程計, 不吃 IMU)')

    # ------------------------------------------------------------------
    def on_reset(self, req, res):
        self.keyframes.clear()
        self.submap = None
        self.matcher = None
        self.motion.set_pose(0.0, 0.0, 0.0)
        self.last_kf_pose = None
        res.success = True
        res.message = '里程計已歸零, 子圖已清空'
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
                    f'這一幀只剩 {f.n_kept} 個點 (原始 {f.n_raw})')
            return

        t0 = time.perf_counter()
        if self.matcher is None:
            # 第一幀直接定義原點。里程計的原點本來就是隨便訂的 ——
            # 全域座標是 slam_toolbox 的 map -> odom 負責的。
            self.motion.set_pose(0.0, 0.0, 0.0, t)
            self._add_keyframe(f)
            self.n_ok += 1
        else:
            px, py, pth = self.motion.predict(t)
            r = self.matcher.refine(f.xy, [px, py], pth)
            if not self._good(r) and self.sweep_on_fail:
                # 見 lidar_loc_node: 需要的範圍超過上限就不要硬掃, 那會鎖到
                # 180 度反方向而且回不來。
                need = max(self.sweep_yaw,
                           abs(self.motion.omega_trusted) * self.front.period * 1.5)
                if need <= self.sweep_yaw_max:
                    r = self.matcher.refine_sweep(f.xy, [px, py], pth,
                                                  yaw_span=need,
                                                  xy_span=self.sweep_xy)
                    self.n_sweep += 1
            if self._good(r):
                self.motion.update(t, r.t[0], r.t[1], r.theta)
                self.last_result = r
                self.n_ok += 1
                if self._need_keyframe(r):
                    self._add_keyframe(f)
            else:
                # 失敗就純推, 不更新速度 (爛速度 -> 爛補償 -> 更爛的配準)
                self.motion.coast(t)
                self.motion.decay(0.7)
                if self.n_scan % 10 == 1:
                    self.get_logger().warn(
                        f'配準失敗: 殘差 {r.residual * 100:.1f} cm, '
                        f'inlier {r.inlier_ratio:.0%} —— 開慢一點')
        self.match_ms = 0.9 * self.match_ms + 0.1 * (time.perf_counter() - t0) * 1e3
        self.publish(msg, f)

    def _good(self, r) -> bool:
        return bool(r.inlier_ratio >= self.min_inlier
                    and r.residual <= self.max_residual
                    and np.isfinite(r.residual))

    # ------------------------------------------------------------------
    def _need_keyframe(self, r) -> bool:
        if r.residual > self.kf_max_res or r.inlier_ratio < self.kf_min_inlier:
            return False
        if self.last_kf_pose is None:
            return True
        x, y, th = self.motion.pose
        dx = math.hypot(x - self.last_kf_pose[0], y - self.last_kf_pose[1])
        dth = abs(math.atan2(math.sin(th - self.last_kf_pose[2]),
                             math.cos(th - self.last_kf_pose[2])))
        return dx >= self.kf_dist or dth >= self.kf_angle

    def _add_keyframe(self, f):
        x, y, th = self.motion.pose
        world = f.xy @ rot2(th).T + np.array([x, y])
        self.keyframes.append((np.array([x, y, th]), world))
        if len(self.keyframes) > self.kf_max:
            del self.keyframes[0]
        self.last_kf_pose = np.array([x, y, th])
        self._rebuild_submap()

    def _rebuild_submap(self):
        """把最近幾個關鍵幀疊成一張小地圖。

        只在**加關鍵幀時**重建 (不是每幀), 而且先裁半徑再降採樣 —— 重建的成本
        幾乎全在距離場的 EDT 上, 而 EDT 的成本跟格子數成正比。
        """
        x, y, _ = self.motion.pose
        pts = np.vstack([w for _, w in self.keyframes])
        d2 = (pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2
        pts = pts[d2 <= self.submap_radius ** 2]
        if pts.shape[0] < 20:
            return
        pts = voxel_downsample(pts, self.submap_voxel)
        self.submap = GridMap.from_points(pts, self.submap_res, margin=1.0,
                                          meta={'type': 'submap'})
        self.matcher = ScanMatcher(self.submap, **self._match_cfg)

    # ------------------------------------------------------------------
    def publish(self, msg, f):
        x, y, th = self.motion.pose
        qz, qw = math.sin(th * 0.5), math.cos(th * 0.5)
        stamp = msg.header.stamp

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        cov = np.zeros((6, 6))
        r = self.last_result
        if r is not None and r.cov is not None:
            cov[0, 0], cov[1, 1], cov[5, 5] = r.cov[0, 0], r.cov[1, 1], r.cov[2, 2]
        else:
            cov[0, 0] = cov[1, 1] = 0.05
            cov[5, 5] = 0.02
        cov[2, 2] = cov[3, 3] = cov[4, 4] = 1e-6
        od.pose.covariance = cov.ravel().tolist()
        od.twist.twist.linear.x = float(self.motion.vx)
        od.twist.twist.linear.y = float(self.motion.vy)
        od.twist.twist.angular.z = float(self.motion.omega)
        self.pub_odom.publish(od)

        if self.tf is not None:
            tfm = TransformStamped()
            tfm.header.stamp = stamp
            tfm.header.frame_id = self.odom_frame
            tfm.child_frame_id = self.base_frame
            tfm.transform.translation.x = float(x)
            tfm.transform.translation.y = float(y)
            tfm.transform.rotation.z = qz
            tfm.transform.rotation.w = qw
            self.tf.sendTransform(tfm)

        if self.pub_scan is not None:
            self.pub_scan.publish(to_laserscan(
                LaserScan, f.xy, stamp, self.base_frame, bins=self.scan_bins,
                range_min=self.front.range_min, range_max=self.front.range_max))

    # ------------------------------------------------------------------
    def status(self):
        if self.n_scan == 0:
            self.get_logger().warn('還沒有收到掃描')
            return
        x, y, th = self.motion.pose
        r = self.last_result
        n = self.submap.n_occupied if self.submap is not None else 0
        self.get_logger().info(
            f'{self.n_scan} 幀 (成功 {100.0 * self.n_ok / max(self.n_scan, 1):.0f}%), '
            f'配準 {self.match_ms:.1f} ms | x={x:+.3f} y={y:+.3f} '
            f'yaw={math.degrees(th):+.1f}° | 關鍵幀 {len(self.keyframes)}, '
            f'子圖 {n} 格 | 殘差 '
            f'{f"{r.residual * 100:.2f} cm" if r else "n/a"}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanOdometry()
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
