#!/usr/bin/env python3
"""方法六: 8 台斜拍的環境相機 + IMU + 輪速 融合定位 (方法 B + C)。

**方法五 (全感測器融合) 跑的也是這個節點**: `fusion_loc.launch.py` 用節點名字
`fusion_localizer` 開它, 打開 `lidar_enabled`, 輸出 /fusion_loc/odom。兩條路線只差 LiDAR。

    /imu, /joint_states ──> 遞推 (car_loc_fusion 原封不動) ──┐
                                                             ├─> MultiCamEkf ─> /multicam_loc/odom
    /env_cam_0..7/car_bbox ─> 挑框 ─> 同時刻綁成一組 ─────────┘      (倒帶重放)    /multicam_loc/pose
                                                                                  TF map->base_link
                                                                                  /multicam_loc/markers

跟 car_loc_fusion (方法五) 的關係
---------------------------------
**這個節點就是 car_loc_fusion 的 FusionLocalizer, 多了一種絕對量測。** IMU / 輪速 /
靜止校正 / 時鐘偵測 / 發布全部是繼承來的, 一行都沒有抄 —— 那些東西在
car_loc_fusion 修過的每一個坑 (時鐘差 625 秒、倒帶重放要重現決定、IMU 槓桿臂在
Isaac 裡不要扣 ...) 這裡自動都有。

新加的只有:
  1. 訂閱每台相機的車框 (`/env_cam_i/car_bbox`, 格式見 bbox_msg.py)
  2. 挑框 (car_model.associate) + 把時戳相同的綁成一組 (`group_tol`)
  3. 起點: 第一組框用方法 B 反推 (car_model.initial_pose), yaw 用 IMU 的
  4. 之後每一組都當成 BboxMeas 交給 MultiCamEkf (方法 C, 量測模型 = 視線)

`/camera_loc/odom` (方法一的單一天花板相機) 與 `/lidar_loc/odom` 那兩個來源也還在,
方法六預設關掉 (`camera_enabled` / `lidar_enabled`), 這樣這一條才能單獨拿來比。

有 LiDAR 時多三條規則, 都是為了同一件事 —— 起點錯了要救得回來:
  * 起點優先用多相機 (`_may_seed`)。LiDAR 在長方形房間可能鎖到對稱解, 而相機 + IMU yaw 不會
  * 挑框太久都沒中就不再用預測挑 (`_assoc_lost`)。挑框是用濾波器的預測, 預測錯得離譜時
    每台都挑不到, 框進不了 EKF -> 閘門計時不會開始 -> 逃生口永遠不會觸發
  * 恢復模式下兩台以上相機同時看到車, 就重跑方法 B 直接換位置 (`reloc_min_cameras`)。
    強制接受是「走過去」, 而車子停著時 P 不會長, 每一步都走不動 (3 m 走 10 秒還差 2.4 m)
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Point, PolygonStamped
from visualization_msgs.msg import Marker, MarkerArray

from car_loc_fusion.fusion_loc_node import QOS, FusionLocalizer, stamp_sec
from car_loc_fusion.sources import AbsSource
from car_loc_wheel.wheel_ins import quat_to_yaw

from . import bbox_msg
from .camera_model import load_cameras, share_file
from .car_model import CarModel, associate, initial_pose
from .multicam_ekf import BboxMeas, BboxObs, MultiCamEkf


class _Group:
    __slots__ = ('t', 'wall0', 'obs')

    def __init__(self, t, wall0):
        self.t = t
        self.wall0 = wall0
        self.obs = {}           # 相機名字 -> BboxObs


class MultiCamLocalizer(FusionLocalizer):

    def __init__(self):
        super().__init__()        # 會呼叫 self._make_ekf() —— 那裡順便讀本節點的參數
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        # 每台相機一個收件政策 (時鐘偏移偵測 / 限流 / 斷線判斷), 直接用 car_loc_fusion 的
        for cam in (self.cams if self.multicam_enabled else []):
            self.src[cam.name] = AbsSource(
                cam.name, sigma_floor=0.0,
                min_dt=float(g('bbox_min_dt')), timeout=float(g('bbox_timeout')),
                clock_max_offset=float(g('bbox_clock_max_offset')),
                extra_delay=float(g('bbox_extra_delay')))
            self.create_subscription(PolygonStamped, cam.detection_topic,
                                     lambda m, c=cam: self.on_bbox(m, c), QOS)
        self.cam_by_name = {c.name: c for c in self.cams}
        self.pending = []
        self.last_imu_yaw = None
        self.n_assoc_miss = 0
        self.init_tries = 0
        self.t_assoc = None             # 上一次有框被 EKF 採用的時刻
        self.pub_mk = (self.create_publisher(MarkerArray, g('marker_topic'), 5)
                       if g('publish_markers') else None)
        self.create_timer(0.01, self.flush)
        self.get_logger().info(
            f'方法六 (多相機斜拍 + 3D 車型 + 視線量測) —— {len(self.cams)} 台相機: '
            + ', '.join(c.detection_topic for c in self.cams)
            + f' | 車型 {len(self.car.points)} 點, base_z {self.car.base_z:.3f} m'
            + f" | 輸出 {g('odom_topic')}")

    # ------------------------------------------------------------------ 設定
    def _load_multicam(self):
        p = self.declare_parameter
        p('multicam_enabled', True)     # false = 不訂閱車框 (拿來 A/B: 只有 LiDAR + 遞推)
        p('cameras_file', '')
        p('car_model_file', '')
        # --- 量測雜訊 (px): sqrt(sigma_px^2 + (size_frac * 框的邊長)^2) * r_inflate ---
        p('bbox_sigma_px', 3.0)
        p('bbox_size_frac', 0.03)
        p('bbox_r_inflate', 1.5)
        p('bbox_border_px', 4.0)        # 貼著畫面邊緣這麼近的邊不用 (框被裁掉了)
        p('bbox_update_yaw', True)      # 讓框也修 yaw (H 的 yaw 那一欄)
        # --- 收件政策 (每台相機各一份, 見 car_loc_fusion/sources.py) ---
        p('bbox_min_dt', 0.0)
        p('bbox_timeout', 0.5)
        p('bbox_clock_max_offset', 1.0)
        p('bbox_extra_delay', 0.0)
        # --- 綁組: 時戳差這麼多以內的框算同一時刻; 等這麼久 (牆上時鐘 s) 才送出 ---
        p('group_tol', 0.002)
        p('group_wait', 0.03)
        # --- 挑框 / 起點 ---
        p('assoc_gate', 1.5)            # 中心距離 / 預測框邊長 超過這個就不是車
        p('init_max_rms', 15.0)         # 起點的反推殘差 (px) 超過就不採用, 等下一組
        p('init_min_cameras', 1)
        # 開機後這麼久 (s) 內起點只用多相機, 過了還沒有才讓 LiDAR 給 (YOLO 暖機要 ~30 s)
        p('init_lidar_wait', 5.0)
        # 恢復模式下這麼多台相機同時看到車才重跑方法 B (一台的話可能是假框, 走強制接受)
        p('reloc_min_cameras', 2)
        p('publish_markers', True)
        p('marker_topic', '/multicam_loc/markers')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cams = load_cameras(g('cameras_file') or share_file('env_cameras.yaml'))
        self.car = CarModel.from_yaml(g('car_model_file') or share_file('car_model.yaml'))
        self.bbox_opts = dict(sigma_px=float(g('bbox_sigma_px')),
                              size_frac=float(g('bbox_size_frac')),
                              border_px=float(g('bbox_border_px')),
                              r_inflate=float(g('bbox_r_inflate')),
                              update_yaw=bool(g('bbox_update_yaw')))
        self.group_tol = float(g('group_tol'))
        self.group_wait = float(g('group_wait'))
        self.assoc_gate = float(g('assoc_gate'))
        self.init_max_rms = float(g('init_max_rms'))
        self.init_min_cameras = int(g('init_min_cameras'))
        self.init_lidar_wait = float(g('init_lidar_wait'))
        self.reloc_min_cameras = int(g('reloc_min_cameras'))
        self.n_reloc = 0
        self.multicam_enabled = bool(g('multicam_enabled'))
        self.t_started = None
        self._mc_ready = True

    def _make_ekf(self):
        if not getattr(self, '_mc_ready', False):
            self._load_multicam()
        return MultiCamEkf.adopt(super()._make_ekf(), self.car, **self.bbox_opts)

    def _reset_state(self):
        super()._reset_state()
        self.pending = []
        self.last_imu_yaw = None
        self.t_assoc = None
        self.t_started = None
        self.init_tries = 0

    def _start(self, t, msg):
        self.t_started = t
        super()._start(t, msg)

    def _may_seed(self, which):
        if which != 'lidar' or not self.multicam_enabled:
            return True
        return self.last_t - self.t_started >= self.init_lidar_wait

    def _assoc_lost(self, t) -> bool:
        """連續 abs_reject_time 秒沒有任何框被採用 -> 預測本身大概錯了, 不要再用它挑框。"""
        if self.t_assoc is None:
            self.t_assoc = t
        return t - self.t_assoc > self.ekf.abs_reject_time

    def on_imu(self, msg):
        super().on_imu(msg)
        if self.yaw_source == 'imu_orientation':
            q = msg.orientation
            self.last_imu_yaw = quat_to_yaw((q.x, q.y, q.z, q.w))

    # ------------------------------------------------------------------ 車框
    def on_bbox(self, msg: PolygonStamped, cam):
        s = self.src[cam.name]
        t_raw = stamp_sec(msg.header.stamp)
        if t_raw <= 0.0:
            return
        # 先換到濾波器的時鐘 (跟 FusionLocalizer.on_abs 同一個理由與同一套做法)
        ref_t = self.ekf.ins.t if self.ekf.ins.t is not None else self.last_t
        t = s.to_filter_clock(t_raw, ref_t)
        if s.clock_flagged and cam.name not in self._clock_warned:
            self._clock_warned.add(cam.name)
            self.get_logger().error(
                f'{cam.name} 的時戳跟 /imu 差 {s.clock_offset:+.2f} 秒 —— 時鐘基準不一樣, '
                '已經自動扣掉, 但延遲補償等於沒有 (見 car_loc_fusion/sources.py)。')
        # **時鐘偏移鎖定之前的框一律不用。** 那時 t 還是原始時戳: 相機的時鐘偏了幾千秒的話,
        # 它就是一則「未來的量測」, 會留在 abs_log 裡, 每次倒帶都用開機時的框重新套用一次,
        # 把估計一路拉回起點 (2026-10-30 那一輪: 前 66 秒最大 34 cm, 見 test [7])。
        if not s.clock_locked:
            return
        boxes = bbox_msg.decode(msg)
        now = self.get_clock().now().nanoseconds * 1e-9
        if not boxes:
            s.last_recv = t           # 相機活著, 只是沒看到車
            return
        ok, _ = s.accept(t, 0.0, now)
        if not ok:
            return

        ekf = self.ekf
        state = None
        if ekf.initialized and not ekf.recovering and not self._assoc_lost(t):
            state = (float(ekf.pos[0]), float(ekf.pos[1]), float(ekf.yaw))
        i = associate(self.car, cam, boxes, state, gate=self.assoc_gate)
        if i is None:
            self.n_assoc_miss += 1
            return
        ob = BboxObs(cam, boxes[i][0], boxes[i][1])

        for grp in self.pending:
            if abs(grp.t - t) <= self.group_tol:
                grp.obs[cam.name] = ob
                break
        else:
            grp = _Group(t, time.monotonic())
            grp.obs[cam.name] = ob
            self.pending.append(grp)

    def flush(self):
        """把等夠久的組送進濾波器 (照時戳順序)。"""
        if not self.pending:
            return
        wall = time.monotonic()
        ready = [g for g in self.pending
                 if wall - g.wall0 >= self.group_wait or len(g.obs) == len(self.cams)]
        if not ready:
            return
        self.pending = [g for g in self.pending if g not in ready]
        for grp in sorted(ready, key=lambda g: g.t):
            self._process(grp)

    def _process(self, grp: _Group):
        ekf = self.ekf
        obs = list(grp.obs.values())
        if not ekf.initialized:
            if self.started:
                self._try_init(grp, obs)
            return
        if (ekf.recovering and len(obs) >= self.reloc_min_cameras
                and self._try_init(grp, obs)):
            self.n_reloc += 1
            return
        m = BboxMeas(grp.t, obs)
        ekf.absolute(m)
        for ob in obs:
            s = self.src[ob.cam.name]
            if ob.accepted:
                s.mark_used(grp.t)
                self.t_assoc = grp.t
            elif ob.accepted is False:
                s.counts['gate'] += 1
        if self.pub_mk is not None:
            self._markers(grp, obs)

    def _try_init(self, grp, obs) -> bool:
        """第一組框 -> 起點 (方法 B)。yaw 用 IMU 的絕對姿態, 沒有就用濾波器的, 再沒有就掃一圈。"""
        if len(obs) < self.init_min_cameras:
            return False
        self.init_tries += 1
        yaw = self.last_imu_yaw
        if yaw is None and self.ekf.initialized:
            yaw = float(self.ekf.yaw)
        r = initial_pose(self.car, [(o.cam, o.box) for o in obs], yaw,
                         max_rms=self.init_max_rms,
                         sigma_px=self.bbox_opts['sigma_px'],
                         size_frac=self.bbox_opts['size_frac'],
                         border_px=self.bbox_opts['border_px'])
        if r is None:
            if self.init_tries in (1, 20) or self.init_tries % 100 == 0:
                self.get_logger().warn(
                    f'第 {self.init_tries} 次嘗試找起點失敗 ({len(obs)} 台相機有框, '
                    f'反推殘差 > {self.init_max_rms:.0f} px 或解不出來) —— 框可能認錯, '
                    '或 env_cameras.yaml / car_model.yaml 跟場景對不上')
            return False
        self.ekf.set_pose(r['x'], r['y'], r['yaw'], t=grp.t,
                          sigma_p=r['sigma_p'], sigma_yaw=r['sigma_yaw'])
        self.ekf.last_abs_t = grp.t
        self.t_assoc = grp.t
        for o in obs:
            self.src[o.cam.name].mark_used(grp.t)
        self.get_logger().info(
            f"起點 ({r['x']:+.3f}, {r['y']:+.3f}) yaw {math.degrees(r['yaw']):+.1f}° "
            f"—— {len(r['used'])} 台相機 ({', '.join(r['used'])}) 反推, "
            f"殘差 {r['rms']:.1f} px"
            + ('' if yaw is not None
               else ' (沒有 IMU 絕對 yaw, 是掃出來的: 前後 180 度可能顛倒)'))
        return True

    # ------------------------------------------------------------------ 視覺化
    def _markers(self, grp, obs):
        """每台相機一條視線 (綠 = 採用, 紅 = 被擋) + 那台相機自己反推的位置 (方法 B)。"""
        ekf = self.ekf
        x, y, yaw = float(ekf.pos[0]), float(ekf.pos[1]), float(ekf.yaw)
        zc = self.car.base_z + float(self.car.center_body[2])
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for k, ob in enumerate(obs):
            ln = Marker()
            ln.header.frame_id = self.map_frame
            ln.header.stamp = stamp
            ln.ns = 'line_of_sight'
            ln.id = int(ob.cam.name.split('_')[-1]) if ob.cam.name[-1].isdigit() else k
            ln.type = Marker.LINE_LIST
            ln.scale.x = 0.01
            ok = bool(ob.accepted)
            ln.color.r, ln.color.g, ln.color.a = (0.1, 0.9, 0.8) if ok else (0.9, 0.1, 0.8)
            ln.lifetime.nanosec = 300_000_000
            c = ob.cam.center
            ln.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])),
                         Point(x=x, y=y, z=zc)]
            arr.markers.append(ln)
            r = self.car.solve_position([(ob.cam, ob.box)], yaw, x0=(x, y), iters=5,
                                        sigma_px=self.bbox_opts['sigma_px'],
                                        size_frac=self.bbox_opts['size_frac'],
                                        border_px=self.bbox_opts['border_px'])
            if r is not None:
                sp = Marker()
                sp.header = ln.header
                sp.ns = 'per_camera_B'
                sp.id = ln.id
                sp.type = Marker.SPHERE
                sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = r['x'], r['y'], zc
                sp.pose.orientation.w = 1.0
                sp.scale.x = sp.scale.y = sp.scale.z = 0.04
                sp.color.r, sp.color.g, sp.color.b, sp.color.a = 1.0, 0.8, 0.1, 0.9
                sp.lifetime.nanosec = 300_000_000
                arr.markers.append(sp)
        self.pub_mk.publish(arr)

    # ------------------------------------------------------------------
    def status(self):
        super().status()
        if self.n_imu == 0 or not self.multicam_enabled:
            return
        ekf = self.ekf
        if not ekf.initialized:
            self.get_logger().warn(
                f'多相機: 還沒有起點 (嘗試 {self.init_tries} 次) —— '
                + ('還在做靜止校正' if not self.started else
                   '在等第一組車框; 沒有的話先看偵測節點的偵測率'))
            return
        self.get_logger().info(f'多相機: {ekf.bbox_report()} 挑框沒中 {self.n_assoc_miss} '
                               f'重定位 {self.n_reloc}')


def main(args=None):
    """`ros2 run` 直接跑也要能用: 節點名字改成 multicam_fusion, 並且帶上本 package 的
    預設參數檔 —— 不然會用 car_loc_fusion 的名字與預設值 (發到 /fusion_loc/odom,
    跟方法五撞在一起)。launch 有給的話以 launch 為準。"""
    argv = list(sys.argv if args is None else args)
    extra = []
    if not any(a.startswith('__node:=') or a.startswith('__name:=') for a in argv):
        extra += ['-r', '__node:=multicam_fusion']
    if '--params-file' not in argv:
        extra += ['--params-file', share_file('multicam_loc.yaml')]
    if extra:
        argv = argv[:1] + ['--ros-args'] + extra + argv[1:]
    rclpy.init(args=argv)
    node = MultiCamLocalizer()
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
