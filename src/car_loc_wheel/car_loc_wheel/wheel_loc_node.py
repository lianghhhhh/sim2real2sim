#!/usr/bin/env python3
"""IMU + 四輪輪速的航位推算。

    /imu          ──> 扣重力 ──> 投影到車頭方向 ──┐
                                                   ├──> EKF ──> /wheel_loc/odom
    /joint_states ──> 取名字 -> 四輪中位數 ────────┘             /wheel_loc/pose
                              └-> 同側轉速差 -> 打滑指標          TF map->base_link

**只訂閱 /imu 與 /joint_states。** 沒有地圖、沒有雷射、沒有相機、沒有 ground truth。

為什麼要加輪速 (回答「只用 IMU 一定會飄嗎」)
--------------------------------------------
會。加速度的誤差要積分**兩次**, 所以位置誤差是 `0.5 * b_a * t^2` —— 時間的
二次式, 而且**跟車子有沒有在動無關**, 停著不動照樣長。

輪速是**速度的直接量測**, 不是積分出來的。加進來之後誤差的形狀就變了:

    純 IMU      位置誤差 ~ t^2         (停著也會長)
    IMU + 輪速  位置誤差 ~ 走過的距離   (停著就不長)

但它**還是會飄** —— 輪速給速度、陀螺儀給角速度, 兩個都還要積分一次。要真的
不漂就得有絕對量測 (相機 / LiDAR / 地圖)。

分工是量出來的, 不是猜的 (詳見 README 與 wheel_ins.py):

    輪速   -> 前進速度。**不要拿它算 yaw** (skid-steer 一定滑, 相關係數只有 0.41;
              端到端 RMS 0.58 m -> 11.85 m)
    陀螺儀 -> 角速度
    加速度 -> 輪速打滑時的備援, 以及兩筆輪速之間的內插

實測 (car_run_data/sim_data.csv, 228 秒, 含蓄意打滑與 12 次高速自旋):

| | RMS | 最大 | 漂移率 |
| --- | --- | --- | --- |
| 全開 | **0.58 m** | 1.91 m | 2.9% |
| 不擋打滑 | 0.69 m | 1.88 m | 2.8% |
| yaw 改用陀螺儀積分 (真車 6 軸) | 2.26 m | 6.19 m | 9.4% |
| yaw 改用左右輪速差 | 11.85 m | 26.76 m | 40.4% |
"""
from __future__ import annotations

import os

for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_v, '1')

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger

from .wheel_ins import (G, WHEELS, TiltTracker, WheelIns, WheelReader,
                        WheelStillDetector, quat_to_matrix, quat_to_yaw)

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=20)


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class WheelLocalizer(Node):

    def __init__(self):
        super().__init__('wheel_localizer')
        p = self.declare_parameter

        p('imu_topic', '/imu')
        p('joint_states_topic', '/joint_states')
        p('odom_topic', '/wheel_loc/odom')
        p('pose_topic', '/wheel_loc/pose')
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('publish_tf', True)

        # --- 車體 -----------------------------------------------------------------
        p('wheel_names', list(WHEELS))
        p('wheel_radius', 0.075)      # car.usd: cylinder radius 0.5 * scale 0.15
        # v_true = wheel_scale * wheel_radius * omega。
        # 尺度誤差是**系統性**的 (走 100 m 就是幾公尺, 停車不會讓它消失)。
        # 它**跟牽引狀態有關, 不是幾何常數**: 有扭矩時輪胎一定有縱向滑移, 輪子
        # 永遠比車快一點 -> 有效半徑比幾何值小。實測同一台車 ——
        #   控制器驅動的四份資料 (n 到 1966): 全部 0.927~0.932
        #   自旋後**滑行**的那一份 (spin12):    1.008 (= 幾何值, 沒有扭矩就沒有滑移)
        # 定位時車子是被開著的, 所以用有扭矩的那個工作點。
        # 端到端 (214 秒那一輪): 1.0 -> RMS 0.74, 0.93 -> 0.68。
        # 要自己校就錄一段**正常開的直線** (不是滑行、不是自旋):
        #   python3 test/replay_bag.py <bag> --measure
        p('wheel_scale', 0.93)
        # base_link 的 +X 量到車頭的角度 (度)。REP-103 的車是 0, 但 car.usd 的
        # base_link 是 **+X 朝左、車頭 -Y**, 所以是 -90。從 sim_data.csv 量到
        # atan2(dy, dx) - gt_yaw 的中位數 = -90.0° (n=1718)。
        p('forward_deg', -90.0)

        # --- 重力 / yaw 的來源 (跟 car_loc_imu 同一套) -----------------------------
        p('gravity_mode', 'orientation')   # orientation | complementary | none
        p('yaw_source', 'imu_orientation')  # imu_orientation | gyro
        p('imu_yaw_sigma', 0.02)
        p('tilt_alpha', 0.02)

        # 起點。航位推算沒辦法知道自己在世界的哪裡, 只能從給定的起點開始推。
        p('initial_pose', [0.0, 0.0, 0.0])   # x, y, yaw(度)
        p('initial_yaw_from_imu', True)

        # --- 開機靜止校正 ---------------------------------------------------------
        # 有輪速之後這一步變得容易很多: 「輪子不轉」就是靜止, 不用像純 IMU 那樣
        # 靠加速度變異數去猜。
        p('calib_time', 1.0)
        p('calib_timeout', 10.0)

        # --- 靜止偵測 -------------------------------------------------------------
        # 實測 (sim_data.csv): 靜止時各輪 |w| 的 p95 = 0.062 rad/s。
        # 0.5 是那個雜訊底線的 8 倍, 而「四輪都低於門檻但車子在動」是 0.00%。
        p('still_wheel', 0.5)         # rad/s, 四輪都要低於這個
        p('still_gyro', 0.05)         # rad/s, 排除原地打轉
        p('still_window', 0.15)

        # --- 濾波器 ---------------------------------------------------------------
        p('sigma_gyro', 0.02)
        p('sigma_gyro_scale', 0.01)
        p('sigma_acc', 0.05)
        p('sigma_acc_omega', 0.15)    # 過程雜訊要跟著角速度走, 見 wheel_ins.py
        p('sigma_cross', 0.02)        # 側向過程雜訊 (NHC 不是完美的)
        p('sigma_bg', 1e-4)
        p('sigma_ba', 1e-3)
        p('sigma_k', 0.0)             # 0 = 凍結輪速尺度 (預設, 見 README 的可觀測性)

        # 輪速量測雜訊。實測一般行駛段殘差 std = 0.051 m/s (四輪中位數),
        # 但那個 std 裡也包含 ground truth 用 20 Hz 位置微分帶進來的雜訊,
        # 所以 0.07 是偏保守的值。
        p('wheel_sigma', 0.07)
        # 嚴重打滑時放大到這個。**要給得很大** —— 三份真實資料一起掃, 都是越大
        # 越好, 而原本的 1.5 剛好卡在最差的位置 (非單調 = 「信一半」最糟)。
        # 給 10 之後增益 ~1e-4, 效果等於丟掉, 但卡方閘門還看得到它 (逃生口要那個
        # 訊號)。詳見 wheel_ins.wheel_sigma_for。
        p('wheel_sigma_slip', 10.0)
        # 同側 (前後) 輪速差 -> 打滑指標。實測誤差 p95:
        #   <0.5 rad/s -> 0.12 m/s | 0.5-2 -> 0.18 | 2-5 -> 0.28 | >5 -> 3.84
        p('slip_spread', 2.0)
        p('slip_spread_max', 8.0)
        # 輪速連續被卡方閘門擋掉超過這麼久 (秒) 就強制接受一次 (R 放大 3 倍)。
        # 必要的逃生口: ZUPT 之後 P[v] 被壓到很小, 只靠過程雜訊慢慢長回來, 所以
        # 濾波器的速度信念一旦錯得夠遠 (加速度計壞掉、gravity_mode 配錯、打滑之後
        # 重新咬地), 閘門會**一直**擋 —— 實測 0.5 m/s 的落差要等 14 秒。
        # 要給得長: 打滑期間閘門本來就應該一直擋, 那不是病態 (實測 0.33 秒的門檻
        # 會在打滑中介入, RMS 0.114 -> 0.304 m)。0 = 不設逃生口。
        p('wheel_reject_time', 2.0)

        p('zupt_sigma', 0.01)
        p('zaru_sigma', 0.002)
        p('anchor_sigma', 0.02)
        p('anchor_after', 0.5)
        p('v_max', 4.0)
        p('max_pos_correction', 0.5)

        p('enable_wheel', True)       # 關掉 = 退化成純 IMU (拿來做 A/B)
        p('enable_zupt', True)
        p('enable_zaru', True)
        p('enable_anchor', True)
        p('enable_slip_gate', True)   # 關掉 = 打滑時 R 不放大 (A/B)

        # 收不到輪速的時候要怎麼辦。輪子是主要的速度來源, 掉了就退回純 IMU ——
        # 這是可以撐一下的 (car_loc_imu 就是那樣跑), 但誤差會回到 t^2 那條曲線,
        # 所以要講出來。
        p('wheel_timeout', 0.5)

        p('status_period', 2.0)

        g = self.get_parameter
        self.map_frame = g('map_frame').value
        self.base_frame = g('base_frame').value
        self.do_tf = bool(g('publish_tf').value)
        self.gravity_mode = g('gravity_mode').value
        self.yaw_source = g('yaw_source').value
        self.imu_yaw_sigma = float(g('imu_yaw_sigma').value)
        self.init_yaw_from_imu = bool(g('initial_yaw_from_imu').value)
        self.calib_time = float(g('calib_time').value)
        self.calib_timeout = float(g('calib_timeout').value)
        self.wheel_timeout = float(g('wheel_timeout').value)
        self.en = {k: bool(g('enable_' + k).value)
                   for k in ('wheel', 'zupt', 'zaru', 'anchor', 'slip_gate')}

        self.ins = self._make_ins()
        self.reader = WheelReader(names=list(g('wheel_names').value),
                                  radius=float(g('wheel_radius').value))
        self.det = WheelStillDetector(wheel_th=float(g('still_wheel').value),
                                      gyro_th=float(g('still_gyro').value),
                                      window=float(g('still_window').value))
        self.tilt = TiltTracker(alpha=float(g('tilt_alpha').value))

        self.initial_pose = list(g('initial_pose').value)
        self.started = False
        self.calib_done = False
        self.t0 = None
        self.calib_buf = []
        self.calib_start = None

        self.n_imu = self.n_joint = self.n_still = 0
        self.last_t = None
        self.last_gyro = np.zeros(3)
        self.last_a_fwd = 0.0
        self.t_wheel = -1e9
        self.wheel_warned = False
        self.name_warned = False
        self.stale_wheel = 0
        self.travelled = 0.0
        self._last_pos = None
        self._dt_wheel = []           # 輪速訊息相對於濾波器時間的偏移

        self.pub_odom = self.create_publisher(Odometry, g('odom_topic').value, 20)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, g('pose_topic').value, 20)
        self.tf = tf2_ros.TransformBroadcaster(self) if self.do_tf else None
        self.create_subscription(Imu, g('imu_topic').value, self.on_imu, SENSOR_QOS)
        self.create_subscription(JointState, g('joint_states_topic').value,
                                 self.on_joint, SENSOR_QOS)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_timer(float(g('status_period').value), self.status)

        self.get_logger().info(
            f"等 {g('imu_topic').value} + {g('joint_states_topic').value} "
            '(IMU + 四輪輪速, 不吃 LiDAR / 相機 / 地圖) | '
            f'重力: {self.gravity_mode}, yaw: {self.yaw_source} | '
            f'車頭 {self.ins.forward_deg:+.0f}° (從 base_link +X 量起) | '
            f'輪徑 {self.reader.r:.4f} m × scale {self.ins.scale:.3f} '
            f'= 有效 {self.reader.r * self.ins.scale:.4f} m | '
            f"開: {', '.join(k for k, v in self.en.items() if v) or '全關'}")
        if self.gravity_mode == 'complementary' and self.ins.sa < 0.2:
            self.get_logger().warn(
                f'gravity_mode=complementary 但 sigma_acc 只有 {self.ins.sa} —— '
                '傾角是估出來的, 那個估計誤差本身就是持續存在的假加速度 '
                '(傾斜 1 度 = 0.17 m/s^2)。6 軸 IMU 請給 sigma_acc:=0.35。')
        if not self.en['wheel']:
            self.get_logger().warn(
                'enable_wheel:=false —— 這樣就退化成純 IMU 了 (誤差回到 t^2)。'
                '這個模式是拿來做 A/B 的, 不是拿來用的。')

    # ------------------------------------------------------------------
    def _make_ins(self) -> WheelIns:
        """照參數建濾波器。~/reset 也走這裡 —— 直接 WheelIns() 會把 forward_deg
        之類的設定悄悄變回預設值, 重設之後就換了一組行為。"""
        g = self.get_parameter
        return WheelIns(
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
        ip = self.initial_pose
        self.ins = self._make_ins()
        self.ins.set_pose(ip[0], ip[1], math.radians(ip[2]))
        self.started = False
        self.calib_done = False
        self.calib_buf.clear()
        self.calib_start = None
        self.travelled = 0.0
        self._last_pos = None
        res.success = True
        res.message = f'已重設到 ({ip[0]:.2f}, {ip[1]:.2f}, {ip[2]:.1f}°) 並重新校正零偏'
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
                '順序如果不是 [FL, FR, RL, RR], 打滑偵測 (同側前後輪配對) 會配錯對, '
                '請用 wheel_names 參數指定。')

        t = stamp_sec(msg.header.stamp)
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        self.t_wheel = t
        self.det.add(t, omega4, float(self.last_gyro[2]))

        if not self.started:
            return

        # 輪速訊息通常比 IMU 慢 (Isaac: joint 60 Hz / IMU 60 Hz, 但兩條線不同步)。
        # 對齊策略: 如果輪速的時戳還在濾波器前面, 先用最後一組 IMU 讀數推到那裡
        # 再更新; 已經落後了就直接更新 —— 落後量通常是幾毫秒, 造成的速度誤差是
        # a*dt (1 m/s^2 × 5 ms = 0.005 m/s), 比 wheel_sigma 小一個數量級。
        # 落後量會記在 status 那行, 真的很大 (>50 ms) 的話要去查時戳來源。
        if self.ins.t is not None:
            off = t - self.ins.t
            self._dt_wheel.append(off)
            if len(self._dt_wheel) > 400:
                del self._dt_wheel[:200]
            if off > 0:
                self.ins.predict(t, self.last_a_fwd, float(self.last_gyro[2]))
            elif off < -0.2:
                self.stale_wheel += 1
                return

        if self.en['wheel']:
            self.ins.update_wheel(v_wheel, spread if self.en['slip_gate'] else 0.0)

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

        self.ins.predict(t, a_fwd, float(gyro[2]))

        if self.yaw_source == 'imu_orientation':
            q = msg.orientation
            self.ins.update_yaw(quat_to_yaw((q.x, q.y, q.z, q.w)),
                                self.imu_yaw_sigma, dt or 0.0)

        if still:
            if self.ins.still_since is None:
                self.ins.still_since = t
            if self.en['zupt']:
                self.ins.zupt()
                self.ins.zero_accel(a_fwd)
            if self.en['zaru']:
                self.ins.zaru(float(gyro[2]))
            if self.en['anchor'] and t - self.ins.still_since >= self.ins.anchor_after:
                self.ins.anchor_position()
        else:
            self.ins.still_since = None
            self.ins.anchor = None

        # 輪速斷了要講 —— 沒有輪速就退回純 IMU, 誤差回到 t^2 的曲線
        if self.en['wheel'] and t - self.t_wheel > self.wheel_timeout:
            if not self.wheel_warned:
                self.wheel_warned = True
                self.get_logger().warn(
                    f'{self.wheel_timeout:.1f} 秒沒有收到 /joint_states —— '
                    '現在等於純 IMU 在跑, 位置誤差會以 t^2 長大。'
                    '請確認 Isaac 的 ROS2PublishJointState 有在發。')
        elif self.wheel_warned and t - self.t_wheel <= self.wheel_timeout:
            self.wheel_warned = False
            self.get_logger().info('/joint_states 回來了')

        pos = self.ins.pos
        if self._last_pos is not None:
            self.travelled += float(np.linalg.norm(pos - self._last_pos))
        self._last_pos = pos
        self.publish(msg)

    # ------------------------------------------------------------------
    def _forward_accel(self, msg, gyro, acc, dt, still) -> float:
        """扣掉重力、投影到車頭方向的加速度 (純量)。

        側向那一維直接丟掉 —— 狀態裡沒有側向速度 (NHC 是結構性的)。
        """
        if self.gravity_mode == 'orientation':
            q = msg.orientation
            R = quat_to_matrix((q.x, q.y, q.z, q.w))
            aw = R @ acc - np.array([0.0, 0.0, G])
            # 世界座標的水平加速度直接投影到「車頭」方向 (= 狀態的 heading),
            # 不必先轉回車體再轉一次
            h = self.ins.heading
            return float(math.cos(h) * aw[0] + math.sin(h) * aw[1])
        if self.gravity_mode == 'complementary':
            if dt is not None and dt > 0:
                # trust_accel 由**輪速**的靜止偵測給 —— 比 car_loc_imu 用慣性
                # 靜止偵測可靠得多 (輪子不轉 = 車子沒有加速度)
                self.tilt.update(gyro, acc, dt, trust_accel=bool(still))
            a_body = self.tilt.gravity_free(acc)
        else:
            a_body = np.asarray(acc, dtype=np.float64)[:2]
        return float(a_body[0] * math.cos(self.ins.phi)
                     + a_body[1] * math.sin(self.ins.phi))

    # ------------------------------------------------------------------
    def _calibrate(self, t, msg, gyro, acc, a_fwd, still) -> bool:
        """開機靜止校正。回傳 True 表示「這一則先不要進濾波器」。

        要等**輪速**說靜止 —— 開機時如果還沒收到 /joint_states, 靜止偵測不會
        成立, 所以這裡也順便當成「兩個 topic 都到齊了」的檢查。
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
            self.ins.x[4] = float(gm[2])                     # b_g
            if self.gravity_mode == 'complementary':
                self.tilt.roll = math.atan2(am[1], am[2])
                self.tilt.pitch = math.atan2(-am[0], math.hypot(am[1], am[2]))
                self.tilt.inited = True
            ba = self._forward_accel(msg, gm, am, None, True)
            self.ins.x[5] = ba                               # b_a
            self._start(t, msg)
            self.get_logger().info(
                f'靜止校正完成 ({len(self.calib_buf)} 筆): '
                f'陀螺零偏 z {gm[2] * 1e3:+.2f} mrad/s, '
                f'前進加速度零偏 {ba:+.4f} m/s^2'
                + (f', 傾角 roll {math.degrees(self.tilt.roll):+.2f}° '
                   f'pitch {math.degrees(self.tilt.pitch):+.2f}°'
                   if self.gravity_mode == 'complementary' else ''))
            return True

        if t - self.t0 >= self.calib_timeout:
            why = ('都沒有收到 /joint_states' if self.n_joint == 0
                   else f'輪子都沒有連續停 {self.calib_time:.1f} 秒')
            self.get_logger().warn(
                f'等了 {self.calib_timeout:.0f} 秒{why}, 零偏當 0 開始推。'
                '陀螺零偏沒校正的話 yaw 會持續漂, 而 yaw 誤差乘上行走距離就是'
                '位置誤差 —— 下次請讓車子先停著一秒再開。')
            self._start(t, msg)
            return True
        if self.n_imu % 120 == 1:
            self.get_logger().info(
                f'靜止校正中 ({elapsed:.1f}/{self.calib_time:.1f} s, '
                f'輪速 {self.n_joint} 筆) —— 請讓車子不要動')
        return True

    def _start(self, t, msg):
        ip = self.initial_pose
        yaw = math.radians(ip[2])
        if self.yaw_source == 'imu_orientation' and self.init_yaw_from_imu:
            q = msg.orientation
            yaw = quat_to_yaw((q.x, q.y, q.z, q.w))
        self.ins.set_pose(ip[0], ip[1], yaw, t)
        self.calib_done = True
        self.started = True
        self.get_logger().info(
            f'起點 ({ip[0]:+.2f}, {ip[1]:+.2f}) yaw {math.degrees(yaw):+.1f}° '
            '—— 航位推算不知道自己在世界的哪裡, 只能從一個給定的起點開始推。')

    # ------------------------------------------------------------------
    def publish(self, msg: Imu):
        if not self.started:
            return
        x, y = self.ins.pos
        yaw = self.ins.yaw
        qz, qw = math.sin(yaw * 0.5), math.cos(yaw * 0.5)

        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = self.map_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.pose.covariance = self.ins.pose_cov().ravel().tolist()
        # twist 用車頭方向的分量 (linear.x = 前進速度), 跟 /cmd_vel 的慣例一致 ——
        # base_link 的 x 軸其實朝左, 但所有遙控工具都把前進放在 linear.x
        od.twist.twist.linear.x = float(self.ins.speed)
        od.twist.twist.angular.z = float(self.ins.last_omega)
        od.twist.covariance = self.ins.twist_cov().ravel().tolist()
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
        if self.n_imu == 0 or self.n_joint == 0:
            self.get_logger().warn(
                f'IMU {self.n_imu} 筆, 輪速 {self.n_joint} 筆 —— '
                '兩個都要有才跑得起來')
            return
        if not self.started:
            return
        c = self.ins.counts
        off = (f'{1e3 * float(np.median(self._dt_wheel)):+.0f} ms'
               if self._dt_wheel else 'n/a')
        self.get_logger().info(
            f'IMU {self.n_imu} / 輪速 {self.n_joint} 筆 '
            f'(靜止 {100.0 * self.n_still / self.n_imu:.0f}%) | '
            f'x={self.ins.x[0]:+.3f} y={self.ins.x[1]:+.3f} '
            f'yaw={math.degrees(self.ins.yaw):+.1f}° v={self.ins.speed:+.2f} m/s | '
            f'sigma {self.ins.sigma_pos() * 100:.1f} cm, '
            f'推算里程 {self.travelled:.1f} m | '
            f"輪速更新 {c['wheel']} (打滑放寬 {c['slip']}, 擋掉 {c['rejected']}, "f"強制 {c['forced']}) "
            f"ZUPT {c['zupt']} ZARU {c['zaru']} 錨定 {c['anchor']} "
            f"限幅 {c['damped']} | b_g {self.ins.gyro_bias * 1e3:+.2f} mrad/s, "
            f'b_a {self.ins.acc_bias:+.3f}, scale {self.ins.scale:.4f} | '
            f'輪速時戳 {off}' + (f', 過期丟棄 {self.stale_wheel}'
                                 if self.stale_wheel else ''))


def main(args=None):
    rclpy.init(args=args)
    node = WheelLocalizer()
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
