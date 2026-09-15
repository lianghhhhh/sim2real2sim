#!/usr/bin/env python3
"""方法五: 把現有的所有感測器結合起來定位。

    /imu          ─┐
    /joint_states ─┴─> 遞推 (60~200 Hz, 連續、不會斷)  ──┐
                                                          ├─> 7 維 EKF ─> /fusion_loc/odom
    /camera_loc/odom (相機+YOLO, 30 Hz, 延遲 80 ms) ──┐   │              /fusion_loc/pose
    /lidar_loc/odom  (LiDAR 對地圖, 10 Hz)          ─┴───┘              TF map->base_link
                     絕對量測 (不漂, 但會斷)

這個節點**訂閱另外兩條定位線的輸出**, 不自己跑 YOLO 也不自己做掃描配準。
所以跑之前那兩條要先開起來 (launch 檔會一起開)。

為什麼是「訂閱它們的輸出」而不是「自己吃原始感測器」
----------------------------------------------------
鬆耦合 (loosely coupled)。緊耦合 (把雷射點雲與 YOLO 像素直接放進同一個濾波器)
理論上更準, 但它有兩個在這個 repo 裡很貴的代價:

1. **沒辦法單獨評估任何一種感測器了。** 四條獨立的路線是這個 repo 的資產,
   緊耦合會把它們變成一個沒辦法拆開的黑盒。
2. **一個感測器的失效模式會直接傳染。** LiDAR 鎖到 180 度的時候, 鬆耦合看到的
   是「一則位置差 10 公尺的 pose」—— 一道閘門就擋掉了; 緊耦合看到的是幾百個
   點的殘差同時變小 (它真的收斂到那個對稱解了), 沒有東西擋得住。

代價是每一條路的濾波器已經平滑過一次, 量測不再獨立 —— 那個問題在 `sources.py`
處理 (R 放大)。

**這個節點做對的三件事** (每一件都有量過, 見 README):
  1. 絕對量測的時戳是**過去**的 -> 倒帶重放 (`fusion_ekf.py`)
  2. 來源自己會知道自己追丟了 -> 用它回報的 sigma 擋 (`sources.py`)
  3. 濾波器自己也會錯 -> 逃生口: 連續擋太久就進入恢復模式
"""
from __future__ import annotations

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from car_loc_wheel.wheel_ins import (G, WHEELS, TiltTracker, WheelReader,
                                     WheelStillDetector, quat_to_matrix,
                                     quat_to_yaw)

from .fusion_ekf import AbsMeas, FusionEkf, Step
from .sources import AbsSource

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=50)


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class FusionLocalizer(Node):

    def __init__(self):
        super().__init__('fusion_localizer')
        p = self.declare_parameter

        # --- 輸入 -----------------------------------------------------------
        p('imu_topic', '/imu')
        p('joint_states_topic', '/joint_states')
        p('camera_topic', '/camera_loc/odom')
        p('lidar_topic', '/lidar_loc/odom')
        p('odom_topic', '/fusion_loc/odom')
        p('pose_topic', '/fusion_loc/pose')
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('publish_tf', True)

        # --- 車體 (跟 car_loc_wheel 同一組, 值也要一樣) ----------------------
        p('wheel_names', list(WHEELS))
        p('wheel_radius', 0.075)
        p('wheel_scale', 0.93)
        p('forward_deg', -90.0)          # car.usd 的車頭是 -Y, 不是 REP-103 的 +X

        p('gravity_mode', 'orientation')   # orientation | complementary | none
        p('yaw_source', 'imu_orientation')  # imu_orientation | gyro
        p('imu_yaw_sigma', 0.02)
        p('tilt_alpha', 0.02)

        # 起點。**融合不需要它** —— 第一則絕對量測就是起點 (這是相對於方法三/四
        # 的一個實質差別)。只有在 wait_for_absolute:=false 時才會用到。
        p('initial_pose', [0.0, 0.0, 0.0])
        p('initial_yaw_from_imu', True)
        p('wait_for_absolute', True)

        p('calib_time', 1.0)
        p('calib_timeout', 10.0)
        p('still_wheel', 0.5)
        p('still_gyro', 0.05)
        p('still_window', 0.15)

        # --- 遞推濾波器 (值跟 car_loc_wheel 一致, 理由見那邊的 README) -------
        p('sigma_gyro', 0.02)
        p('sigma_gyro_scale', 0.01)
        p('sigma_acc', 0.05)
        p('sigma_acc_omega', 0.15)
        p('sigma_cross', 0.02)
        p('sigma_bg', 1.0e-4)
        p('sigma_ba', 1.0e-3)
        p('sigma_k', 0.0)
        p('wheel_sigma', 0.07)
        p('wheel_sigma_slip', 10.0)
        p('slip_spread', 2.0)
        p('slip_spread_max', 8.0)
        p('wheel_reject_time', 2.0)
        p('zupt_sigma', 0.01)
        p('zaru_sigma', 0.002)
        p('anchor_sigma', 0.02)
        p('anchor_after', 0.5)
        p('v_max', 4.0)
        p('max_pos_correction', 0.5)     # 高頻偽量測用的上限 (WheelIns 內部)

        # --- 融合層 ---------------------------------------------------------
        # 倒帶重放的時間視窗。要蓋得住**最慢的那一條**的延遲 + 傳輸抖動:
        # 相機是 79.5 ms (曝光+傳輸+YOLO), 加上抖動 0.4 s 有五倍餘裕。
        # status 那行的「太舊」不是 0 的話就是不夠, 調大 (代價只有記憶體)。
        p('rewind', True)
        p('rewind_horizon', 0.4)
        # 絕對量測的單次修正上限 (m)。比 max_pos_correction 大是刻意的 ——
        # 絕對量測本來就該有把跑掉的估計拉回來的權力, 那正是它存在的理由。
        p('abs_max_correction', 1.0)
        # 連續這麼久沒有任何絕對量測通過閘門 -> 進入恢復模式 (強制接受, R x9),
        # 直到有一則自己通過為止。處理的是「P 很小但是錯的」: 綁架、遞推跑掉、
        # 地圖原點對錯。實測 (test/test_fusion.py [F], 車子被搬走 3 m):
        #   有逃生口 -> 5.0 秒收斂回 20 cm 以內; 沒有 -> **永遠回不來**
        p('abs_reject_time', 3.0)
        p('chi2_scale', 1.0)

        # 每個來源的收件政策 —— 三道防線都**不看濾波器狀態**, 見 sources.py
        p('camera_enabled', True)
        p('camera_sigma_floor', 0.03)    # 相機 2.1 cm (單軸) 的保守下限
        p('camera_sigma_max', 0.0)       # 相機沒有「鎖到對稱解」這種失效模式
        p('camera_r_inflate', 1.5)
        p('camera_min_dt', 0.0)          # 0 = 不限流 (量出來限流反而更差, 見 README [G])
        p('camera_use_yaw', False)       # 相機的 yaw 是速度方向推的, 15~22 度
        p('camera_timeout', 0.5)
        # 時戳跟濾波器差超過這麼多就認定是**時鐘基準差**, 量一次鎖定並扣掉。
        # 2026-09-10 那一輪實測 /rgb 比 /imu 早 **625.55 秒** (std 10 ms)。
        # 這個功能只救得回「不要壞掉」, 救不回精度 —— 時鐘一偏, 真正的延遲就
        # 沒辦法從資料裡分離出來, 要拿回 v x 80 ms 只能去源頭修時鐘。
        p('camera_clock_max_offset', 1.0)
        # 時鐘沒對好時, 手動把量出來的常數延遲加回去 (car_loc_camera 量到 0.0795)。
        # 時鐘是對的就**不要設** —— 那會變成扣兩次。
        p('camera_extra_delay', 0.0)

        p('lidar_enabled', True)
        p('lidar_sigma_floor', 0.04)     # 地圖解析度 5 cm -> 誤差 ~3.5 cm
        # **追丟偵測用來源自己回報的 sigma。** 實測 (collect_data_node):
        # 正常 ~0.0015, 鎖到 180 度 ~0.0145 —— 差一個數量級, 門檻 0.0025 三輪
        # 資料 0 漏網。這道防線在「只有 LiDAR」時是關鍵: 鎖住之後那些量測彼此
        # 一致, 卡方閘門擋久了反而會觸發逃生口, 把估計搬到鏡射的位置
        # (test_fusion.py [D]: 有閘門 2.8 cm, 沒有 338 cm)。
        p('lidar_sigma_max', 0.0025)
        p('lidar_r_inflate', 1.5)
        p('lidar_min_dt', 0.0)
        p('lidar_use_yaw', True)         # LiDAR 的 yaw 是幾何配準出來的, 2.9 度
        p('lidar_yaw_sigma', 0.05)
        p('lidar_timeout', 1.0)
        p('lidar_clock_max_offset', 1.0)
        p('lidar_extra_delay', 0.0)

        p('enable_wheel', True)
        p('enable_zupt', True)
        p('enable_zaru', True)
        p('enable_anchor', True)
        p('enable_slip_gate', True)
        p('wheel_timeout', 0.5)
        p('status_period', 2.0)

        g = self.get_parameter
        self.map_frame = g('map_frame').value
        self.base_frame = g('base_frame').value
        self.gravity_mode = g('gravity_mode').value
        self.yaw_source = g('yaw_source').value
        self.imu_yaw_sigma = float(g('imu_yaw_sigma').value)
        self.initial_pose = [float(v) for v in g('initial_pose').value]
        self.init_yaw_from_imu = bool(g('initial_yaw_from_imu').value)
        self.wait_for_absolute = bool(g('wait_for_absolute').value)
        self.calib_time = float(g('calib_time').value)
        self.calib_timeout = float(g('calib_timeout').value)
        self.wheel_timeout = float(g('wheel_timeout').value)
        self.en = {k: bool(g(f'enable_{k}').value)
                   for k in ('wheel', 'zupt', 'zaru', 'anchor', 'slip_gate')}

        if self.gravity_mode == 'complementary' and float(g('sigma_acc').value) < 0.2:
            self.get_logger().warn(
                f"gravity_mode=complementary 但 sigma_acc={g('sigma_acc').value} 太小。"
                '傾角是**估**出來的, 那個估計誤差本身就是持續存在的假加速度 '
                '(傾斜 1 度 = 0.17 m/s^2), 過程雜訊蓋不住它的話, 正確的量測會被'
                '卡方閘門一直擋掉。真車請給 0.35。')

        self.ekf = self._make_ekf()
        self.reader = WheelReader(names=[str(n) for n in g('wheel_names').value],
                                  radius=float(g('wheel_radius').value))
        self.det = WheelStillDetector(wheel_th=float(g('still_wheel').value),
                                      gyro_th=float(g('still_gyro').value),
                                      window=float(g('still_window').value))
        self.tilt = TiltTracker(alpha=float(g('tilt_alpha').value))

        self.src = {
            'camera': AbsSource(
                'camera', enabled=bool(g('camera_enabled').value),
                sigma_floor=float(g('camera_sigma_floor').value),
                sigma_max=float(g('camera_sigma_max').value),
                r_inflate=float(g('camera_r_inflate').value),
                min_dt=float(g('camera_min_dt').value),
                timeout=float(g('camera_timeout').value),
                clock_max_offset=float(g('camera_clock_max_offset').value),
                extra_delay=float(g('camera_extra_delay').value),
                use_yaw=bool(g('camera_use_yaw').value)),
            'lidar': AbsSource(
                'lidar', enabled=bool(g('lidar_enabled').value),
                sigma_floor=float(g('lidar_sigma_floor').value),
                sigma_max=float(g('lidar_sigma_max').value),
                r_inflate=float(g('lidar_r_inflate').value),
                min_dt=float(g('lidar_min_dt').value),
                timeout=float(g('lidar_timeout').value),
                clock_max_offset=float(g('lidar_clock_max_offset').value),
                extra_delay=float(g('lidar_extra_delay').value),
                use_yaw=bool(g('lidar_use_yaw').value),
                yaw_sigma=float(g('lidar_yaw_sigma').value)),
        }

        # 狀態
        self.started = self.calib_done = False
        self.t0 = self.calib_start = None
        self.calib_buf = []
        self.last_gyro = np.zeros(3)
        self.last_a_fwd = 0.0
        self.last_t = None
        self.pending_wheel = None        # (t, v_wheel, spread)
        self.t_wheel = -1e9
        self.n_imu = self.n_joint = self.n_still = 0
        self.travelled = 0.0
        self._last_pos = None
        self.wheel_warned = self.name_warned = False
        self._clock_warned = set()
        self._init_clock_warned = set()

        self.pub_odom = self.create_publisher(Odometry, g('odom_topic').value, 10)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, g('pose_topic').value, 10)
        self.tf = TransformBroadcaster(self) if g('publish_tf').value else None

        self.create_subscription(Imu, g('imu_topic').value, self.on_imu, QOS)
        self.create_subscription(JointState, g('joint_states_topic').value,
                                 self.on_joint, QOS)
        self.create_subscription(Odometry, g('camera_topic').value,
                                 lambda m: self.on_abs(m, 'camera'), QOS)
        self.create_subscription(Odometry, g('lidar_topic').value,
                                 lambda m: self.on_abs(m, 'lidar'), QOS)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_timer(max(float(g('status_period').value), 0.5), self.status)

        self.get_logger().info(
            '方法五 (全感測器融合) 啟動 —— 遞推吃 /imu + /joint_states, '
            f"絕對量測吃 {g('camera_topic').value} 與 {g('lidar_topic').value}。"
            '那兩條要另外開起來 (fusion_loc.launch.py 會一起開)。')

    # ------------------------------------------------------------------
    def _make_ekf(self) -> FusionEkf:
        g = self.get_parameter
        return FusionEkf(
            rewind=bool(g('rewind').value),
            rewind_horizon=float(g('rewind_horizon').value),
            abs_max_correction=float(g('abs_max_correction').value),
            abs_reject_time=float(g('abs_reject_time').value),
            chi2_scale=float(g('chi2_scale').value),
            forward_deg=float(g('forward_deg').value),
            wheel_scale=float(g('wheel_scale').value),
            sigma_gyro=float(g('sigma_gyro').value),
            sigma_gyro_scale=float(g('sigma_gyro_scale').value),
            sigma_acc=float(g('sigma_acc').value),
            sigma_acc_omega=float(g('sigma_acc_omega').value),
            sigma_cross=float(g('sigma_cross').value),
            sigma_bg=float(g('sigma_bg').value),
            sigma_ba=float(g('sigma_ba').value),
            sigma_k=float(g('sigma_k').value),
            wheel_sigma=float(g('wheel_sigma').value),
            wheel_sigma_slip=float(g('wheel_sigma_slip').value),
            slip_spread=float(g('slip_spread').value),
            slip_spread_max=float(g('slip_spread_max').value),
            wheel_reject_time=float(g('wheel_reject_time').value),
            zupt_sigma=float(g('zupt_sigma').value),
            zaru_sigma=float(g('zaru_sigma').value),
            anchor_sigma=float(g('anchor_sigma').value),
            anchor_after=float(g('anchor_after').value),
            v_max=float(g('v_max').value),
            max_pos_correction=float(g('max_pos_correction').value))

    def on_reset(self, req, res):
        self.ekf = self._make_ekf()
        self.started = self.calib_done = False
        self.calib_buf.clear()
        self.calib_start = None
        self.travelled = 0.0
        self._last_pos = None
        for s in self.src.values():
            s.last_used = None
        res.success = True
        res.message = '已重設 —— 會重新做靜止校正, 並用下一則絕對量測當起點'
        self.get_logger().warn(res.message)
        return res

    # ------------------------------------------------------------------ 輪速
    def on_joint(self, msg: JointState):
        r = self.reader.read(list(msg.name), list(msg.velocity))
        if r is None:
            return
        v_wheel, spread, omega4 = r
        self.n_joint += 1
        if not self.reader.by_name and not self.name_warned:
            self.name_warned = True
            self.get_logger().warn(
                f'/joint_states 裡找不到 {self.reader.names} —— 退回用前四個 velocity。'
                '順序不是 [FL, FR, RL, RR] 的話打滑偵測會配錯對。')
        t = stamp_sec(msg.header.stamp)
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        self.t_wheel = t
        self.det.add(t, omega4, float(self.last_gyro[2]))
        # **輪速不在這裡更新, 存起來讓下一個 IMU step 帶進去。**
        # 倒帶重放要能把每一步原樣重做一次, 所以所有高頻更新都必須掛在 Step 上;
        # 在這裡直接更新的話, 那次更新在倒帶之後就消失了 (而且症狀是隨機的 ——
        # 取決於當時有沒有絕對量測進來)。
        self.pending_wheel = (t, v_wheel, spread)

    # ------------------------------------------------------------------ IMU
    def on_imu(self, msg: Imu):
        t = stamp_sec(msg.header.stamp)
        self.n_imu += 1
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                         msg.angular_velocity.z])
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        self.last_gyro = gyro
        dt = None if self.last_t is None else t - self.last_t
        self.last_t = t

        still = self.det.is_still()
        self.n_still += int(still)
        a_fwd = self._forward_accel(msg, gyro, acc, dt, still)
        self.last_a_fwd = a_fwd

        if not self.calib_done:
            if self._calibrate(t, msg, gyro, acc, a_fwd, still):
                return
        if not self.started:
            return

        v_wheel = spread = None
        if self.en['wheel'] and self.pending_wheel is not None:
            wt, v_wheel, spread = self.pending_wheel
            self.pending_wheel = None
            if not self.en['slip_gate']:
                spread = 0.0

        yaw_meas = None
        if self.yaw_source == 'imu_orientation':
            q = msg.orientation
            yaw_meas = quat_to_yaw((q.x, q.y, q.z, q.w))

        self.ekf.step(Step(t, a_fwd, float(gyro[2]),
                           v_wheel=v_wheel, spread=(spread or 0.0),
                           still=(still and self.en['zupt']),
                           yaw_meas=yaw_meas, yaw_sigma=self.imu_yaw_sigma,
                           dt=(dt or 0.0)))

        if self.en['wheel'] and t - self.t_wheel > self.wheel_timeout:
            if not self.wheel_warned:
                self.wheel_warned = True
                self.get_logger().warn(
                    f'{self.wheel_timeout:.1f} 秒沒有收到 /joint_states —— 遞推那一路'
                    '退回純 IMU (誤差回到 t^2), 絕對量測還在, 所以還撐得住, '
                    '但兩者之間的空窗會變差。')
        elif self.wheel_warned and t - self.t_wheel <= self.wheel_timeout:
            self.wheel_warned = False
            self.get_logger().info('/joint_states 回來了')

        pos = self.ekf.pos
        if self._last_pos is not None:
            self.travelled += float(np.linalg.norm(pos - self._last_pos))
        self._last_pos = pos
        self.publish(msg)

    # -------------------------------------------------------------- 絕對量測
    def on_abs(self, msg: Odometry, which: str):
        s = self.src[which]
        if not s.enabled:
            return
        t_raw = stamp_sec(msg.header.stamp)
        if t_raw <= 0.0:
            return
        p = msg.pose.pose.position
        if not (math.isfinite(p.x) and math.isfinite(p.y)):
            return
        # **先把時戳換到濾波器的時鐘上。** 來源跟 IMU 不一定在同一個時鐘基準:
        # 2026-09-10 實測 /rgb 比 /imu 早 625.55 秒, 而那會讓緩衝區永遠清不掉、
        # 整段歷史被重複套用, 最後估計凍結在起點 (見 sources.py 的「時鐘」)。
        # 參考時鐘: 濾波器初始化之前 ins.t 還是 None, 用最新一則 IMU 的時戳頂上。
        # 以前這裡直接傳 ins.t -> 初始化之前**完全不量時鐘偏移**, 而第一則絕對量測
        # 正是拿來初始化的那一則。2026-09-15 實測: run_car.sh 第二個場景 Isaac
        # 重新 Play 後 /clock、/imu、/scan 歸零, 相機時戳卻延續上一個場景 (+292 s)。
        # 相機先到 -> 用 t=300 初始化 -> 之後 t=8,9,10... 的 IMU 全部 dt<0 被丟掉
        # -> 整輪凍結在起點, sigma 還很小, 下游完全察覺不到。
        ref_t = self.ekf.ins.t if self.ekf.ins.t is not None else self.last_t
        t = s.to_filter_clock(t_raw, ref_t)
        if s.clock_flagged and which not in self._clock_warned:
            self._clock_warned.add(which)
            self.get_logger().error(
                f'{which} 的時戳跟 /imu 差 {s.clock_offset:+.2f} 秒 —— 那不是延遲, '
                f'是**時鐘基準不一樣**。已經自動扣掉, 定位不會壞掉, 但是:\n'
                f'  * 真正的延遲沒辦法再從資料裡分離出來 -> 延遲補償等於沒有,\n'
                f'    誤差會多出 v x 延遲 (0.8 m/s x 80 ms = 6 cm, 3 m/s = 24 cm)\n'
                f'  * 正確的修法是去源頭把時鐘對好 (那個節點的 use_sim_time, '
                f'或發布端用 /clock 蓋時戳)\n'
                f'  * 暫時的話可以用 {which}_extra_delay 把量出來的常數延遲加回去')
        cov = msg.pose.covariance
        rep = math.sqrt(max(cov[0] + cov[7], 0.0))
        now = self.get_clock().now().nanoseconds * 1e-9
        ok, why = s.accept(t, rep, now)
        if not ok:
            return

        yaw = quat_to_yaw((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        m = AbsMeas(t, which, p.x, p.y, s.sigma_for(rep),
                    yaw=yaw, yaw_sigma=s.yaw_sigma)

        first = not self.ekf.initialized
        if first and not self.started:
            # 還在做靜止校正 —— 先不要用, 不然零偏還沒量完就開始跑
            return
        if (first and not s.clock_locked and ref_t is not None
                and abs(t - ref_t) > s.clock_max_offset):
            # 時鐘偏移還沒量完, 這一則又跟 IMU 時鐘差一大截 -> 不能拿來當起點。
            # 等 clock_samples 筆量完、偏移鎖定之後, 換算過的時戳就會落在 IMU 附近。
            if which not in self._init_clock_warned:
                self._init_clock_warned.add(which)
                self.get_logger().warn(
                    f'{which} 的時戳跟 /imu 差 {t - ref_t:+.1f} 秒, 時鐘偏移還沒量完, '
                    '先不拿它當起點 (等偏移鎖定)')
            return
        if self.ekf.absolute(m, use_yaw=s.use_yaw):
            s.mark_used(t)
            if first:
                self.get_logger().info(
                    f'用 {which} 的第一則 pose 當起點: '
                    f'({p.x:+.2f}, {p.y:+.2f}) yaw {math.degrees(yaw):+.1f}° '
                    '—— 融合不需要 initial_pose, 這是它相對於純航位推算的差別。')
        else:
            s.counts['gate'] += 1

    # ------------------------------------------------------------------
    def _forward_accel(self, msg, gyro, acc, dt, still) -> float:
        if self.gravity_mode == 'orientation':
            q = msg.orientation
            R = quat_to_matrix((q.x, q.y, q.z, q.w))
            aw = R @ acc - np.array([0.0, 0.0, G])
            h = self.ekf.heading
            return float(math.cos(h) * aw[0] + math.sin(h) * aw[1])
        if self.gravity_mode == 'complementary':
            if dt is not None and dt > 0:
                self.tilt.update(gyro, acc, dt, trust_accel=bool(still))
            a_body = self.tilt.gravity_free(acc)
        else:
            a_body = np.asarray(acc, dtype=np.float64)[:2]
        return float(a_body[0] * math.cos(self.ekf.ins.phi)
                     + a_body[1] * math.sin(self.ekf.ins.phi))

    def _calibrate(self, t, msg, gyro, acc, a_fwd, still) -> bool:
        """開機靜止校正 —— 融合一樣要做。

        絕對量測會把**位置**按住, 但陀螺零偏不會被它修掉 (那要靠 ZARU 或校正),
        而零偏會讓兩則絕對量測之間的遞推歪掉。校正過的話那一段是乾淨的。
        """
        if self.t0 is None:
            self.t0 = t
        if still:
            if self.calib_start is None:
                self.calib_start = t
            self.calib_buf.append((gyro.copy(), acc.copy()))
        else:
            self.calib_start = None
            self.calib_buf.clear()

        elapsed = 0.0 if self.calib_start is None else t - self.calib_start
        if elapsed >= self.calib_time and len(self.calib_buf) >= 15:
            gm = np.mean([b[0] for b in self.calib_buf], axis=0)
            am = np.mean([b[1] for b in self.calib_buf], axis=0)
            self.ekf.ins.x[4] = float(gm[2])
            if self.gravity_mode == 'complementary':
                self.tilt.roll = math.atan2(am[1], am[2])
                self.tilt.pitch = math.atan2(-am[0], math.hypot(am[1], am[2]))
                self.tilt.inited = True
            self.ekf.ins.x[5] = self._forward_accel(msg, gm, am, None, True)
            self._start(t, msg)
            self.get_logger().info(
                f'靜止校正完成 ({len(self.calib_buf)} 筆): '
                f'陀螺零偏 z {gm[2] * 1e3:+.2f} mrad/s, '
                f'前進加速度零偏 {self.ekf.ins.x[5]:+.4f} m/s^2')
            return True

        if t - self.t0 >= self.calib_timeout:
            why = ('都沒有收到 /joint_states' if self.n_joint == 0
                   else f'輪子都沒有連續停 {self.calib_time:.1f} 秒')
            self.get_logger().warn(
                f'等了 {self.calib_timeout:.0f} 秒{why}, 零偏當 0 開始推。'
                '融合撐得住 (絕對量測會把位置按住), 但兩則之間的遞推會差一點。')
            self._start(t, msg)
            return True
        if self.n_imu % 120 == 1:
            self.get_logger().info(
                f'靜止校正中 ({elapsed:.1f}/{self.calib_time:.1f} s, '
                f'輪速 {self.n_joint} 筆) —— 請讓車子不要動')
        return True

    def _start(self, t, msg):
        self.calib_done = self.started = True
        if self.wait_for_absolute:
            self.get_logger().info(
                '等第一則絕對量測當起點 (相機或 LiDAR)。'
                '真的沒有那兩條的話用 wait_for_absolute:=false + initial_pose。')
            return
        ip = self.initial_pose
        yaw = math.radians(ip[2])
        if self.yaw_source == 'imu_orientation' and self.init_yaw_from_imu:
            q = msg.orientation
            yaw = quat_to_yaw((q.x, q.y, q.z, q.w))
        self.ekf.set_pose(ip[0], ip[1], yaw, t)
        self.get_logger().info(
            f'起點 ({ip[0]:+.2f}, {ip[1]:+.2f}) yaw {math.degrees(yaw):+.1f}°')

    # ------------------------------------------------------------------
    def publish(self, msg: Imu):
        if not self.ekf.initialized:
            return
        x, y = self.ekf.pos
        yaw = self.ekf.yaw
        qz, qw = math.sin(yaw * 0.5), math.cos(yaw * 0.5)

        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = self.map_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.pose.covariance = self.ekf.pose_cov().ravel().tolist()
        od.twist.twist.linear.x = float(self.ekf.speed)
        od.twist.twist.angular.z = float(self.ekf.ins.last_omega)
        od.twist.covariance = self.ekf.twist_cov().ravel().tolist()
        self.pub_odom.publish(od)

        pc = PoseWithCovarianceStamped()
        pc.header = od.header
        pc.pose.pose = od.pose.pose
        pc.pose.covariance = od.pose.covariance
        self.pub_pose.publish(pc)

        if self.tf is not None:
            tfm = TransformStamped()
            tfm.header.stamp = msg.header.stamp
            tfm.header.frame_id = self.map_frame
            tfm.child_frame_id = self.base_frame
            tfm.transform.translation.x = float(x)
            tfm.transform.translation.y = float(y)
            tfm.transform.rotation.z = qz
            tfm.transform.rotation.w = qw
            self.tf.sendTransform(tfm)

    # ------------------------------------------------------------------
    def status(self):
        if self.n_imu == 0:
            self.get_logger().warn('還沒收到 /imu')
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        srcs = ' '.join(s.report(now) for s in self.src.values())
        if not self.ekf.initialized:
            self.get_logger().warn(
                f'還沒有起點 —— 在等第一則絕對量測。來源: {srcs} '
                '(兩條都沒有的話, 是那兩個節點沒開, 或 topic 名字不對)')
            return
        c = self.ekf.ins.counts
        self.get_logger().info(
            f'IMU {self.n_imu} / 輪速 {self.n_joint} 筆 '
            f'(靜止 {100.0 * self.n_still / self.n_imu:.0f}%) | '
            f'x={self.ekf.pos[0]:+.3f} y={self.ekf.pos[1]:+.3f} '
            f'yaw={math.degrees(self.ekf.yaw):+.1f}° v={self.ekf.speed:+.2f} | '
            f'sigma {self.ekf.sigma_pos() * 100:.1f} cm 里程 {self.travelled:.1f} m | '
            f'{srcs} | {self.ekf.report()} | '
            f"輪速 {c['wheel']} (打滑 {c['slip']}, 擋 {c['rejected']}, "
            f"強制 {c['forced']}) ZUPT {c['zupt']}"
            + ('  ** 恢復模式 **' if self.ekf.recovering else ''))


def main(args=None):
    rclpy.init(args=args)
    node = FusionLocalizer()
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
