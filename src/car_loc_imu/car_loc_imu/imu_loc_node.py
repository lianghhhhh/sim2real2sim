#!/usr/bin/env python3
"""方法三: 只用 IMU 定位 (慣性推算 + 抗漂移)。

    /imu ──> 扣重力 ──> EKF 遞推 ──┬── ZUPT  (靜止時速度 = 0)
                                   ├── ZARU  (靜止時陀螺儀讀到的全是零偏)
                                   ├── 零加速度 (靜止時水平加速度 = 0 -> 修 b_a)
                                   ├── 位置錨定 (停久了不要隨機遊走)
                                   └── NHC   (車不會橫著走 -> 側向速度 = 0)
                                                 │
                                                 v
                    /imu_loc/odom + /imu_loc/pose + TF map->base_link

**這個節點只訂閱 /imu。** 沒有地圖、沒有雷射、沒有相機。

要有心理準備的事
----------------
純 IMU **一定會漂**, 因為加速度的誤差要積分兩次才變成位置。這裡做的每一件事
都只是在壓成長速度。離線實測 (60 Hz, 60 秒, 8 字形每 15 秒停 3 秒,
陀螺零偏 0.004 rad/s, 加速度零偏 0.02 m/s^2):

| 開了什麼 | 位置誤差 RMS |
| --- | --- |
| 純積分 | 4.84 m |
| + ZUPT / ZARU | 0.50 m |
| + NHC (只有 NHC) | 0.08 m |
| 全開 | **0.13 m** |

以上是「IMU 給精確姿態」(Isaac / 9 軸) 的情況。真車的 6 軸 IMU 要自己估傾角,
同一條軌跡全開是 0.69 m; 而如果車身有 2 度的傾角沒估準, 會掉到 2.89 m ——
**姿態誤差比零偏更致命**, 傾斜 1 度就是 0.17 m/s^2 的假加速度。

還有一件不能忘的: 上面的數字之所以好看, 是因為那條軌跡**每 15 秒就停 3 秒**。
一直開不停的話 ZUPT 完全不會觸發, 誤差就是自由累積。純 IMU 的可用時間長度
取決於「多久停一次」, 不是取決於參數。
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
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from .ins import (G, ImuIns, SpinDetector, StillConfirm, StillDetector, TiltTracker,
                  quat_to_matrix, quat_to_yaw)

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=20)


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class ImuLocalizer(Node):

    def __init__(self):
        super().__init__('imu_localizer')
        p = self.declare_parameter

        p('imu_topic', '/imu')
        p('odom_topic', '/imu_loc/odom')
        p('pose_topic', '/imu_loc/pose')
        # 扣掉估計零偏之後的 IMU (陀螺儀 z、加速度 xy)。空字串 = 不發。
        p('corrected_imu_topic', '/imu_loc/imu_corrected')
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('publish_tf', True)

        # 重力怎麼扣 —— 這是整個方法最重要的一個選擇, 見 ins.py 的說明。
        #   orientation   : 用訊息裡的 orientation (Isaac / 9 軸 IMU)。模擬用這個。
        #   complementary : 陀螺儀積分 + 靜止時用加速度校正 (6 軸 IMU / 真車)
        #   none          : 假設車身永遠水平, 直接用 ax, ay
        p('gravity_mode', 'orientation')
        # yaw 從哪裡來:
        #   imu_orientation : 用訊息的 orientation (仍然只是 IMU 的資料)
        #   gyro            : 只靠陀螺儀積分 (真車的 6 軸 IMU)
        p('yaw_source', 'imu_orientation')
        p('imu_yaw_sigma', 0.02)
        p('tilt_alpha', 0.02)

        # 起點。純 IMU 沒有任何辦法知道自己在世界的哪裡 —— 它只能從一個給定的
        # 起點開始推。這是方法三跟另外兩條路最根本的差別。
        p('initial_pose', [0.0, 0.0, 0.0])       # x, y, yaw(度)
        # yaw_source:=imu_orientation 時, 起始 yaw 直接用第一則 IMU 訊息的,
        # 忽略 initial_pose 的第三項
        p('initial_yaw_from_imu', True)

        # --- 開機靜止校正 ---------------------------------------------------------
        # 車子還沒動的時候把零偏量掉。最便宜也最有效的一步; 沒做的話那個零偏
        # 會被積分兩次, 幾十秒就是幾公尺。
        p('calib_time', 1.5)          # 要連續靜止這麼久 (s)
        p('calib_timeout', 10.0)      # 等這麼久還沒靜止過就放棄, 零偏當 0

        # --- 靜止偵測 -------------------------------------------------------------
        # ZUPT 不能只看陀螺儀: 等速直線行駛時陀螺儀與加速度都接近靜止值,
        # 純慣性量測分不出來。所以再看一段視窗內加速度的**變異數**。
        p('still_gyro', 0.03)         # rad/s
        p('still_acc', 0.25)          # |a| 跟 g 的差 (m/s^2)
        p('still_var', 0.05)          # 視窗內 a 的標準差 (m/s^2)
        p('still_window', 0.30)
        # 靜止的第二階段確認 (見 ins.StillConfirm): 上面的條件只看原始讀數, 路面
        # 平的時候「平順加減速 / 自旋後的慢轉 / 等速直線」三種都過得了。各設 0
        # 就是關掉那一道。
        p('still_acc_mean', 0.10)     # 平均水平加速度 (扣零偏) 要小於這個 (m/s^2)
        p('still_gyro_mean', 0.002)   # 平均角速度 (扣零偏) 要小於這個 (rad/s)
        p('still_speed_gate', 0.20)   # 估計速度還大於這個就不承認靜止 (m/s)
        p('still_speed_frac', 0.15)   # 門檻再加上 這個比例 x 離開靜止後的速度變化量
        p('still_speed_trust', 15.0)  # 離開靜止超過這麼久就不再用速度否決 (s)
        p('still_speed_max_omega', 3.0)  # 離開靜止後轉速超過這個, 速度那一道就不算數

        # --- 濾波器 ---------------------------------------------------------------
        p('sigma_gyro', 0.02)
        p('sigma_gyro_scale', 0.01)
        # 加速度的過程雜訊 = sqrt(sigma_acc^2 + (sigma_acc_omega*|w|)^2)。
        # **一定要有 w 那一項。** 實測 (test/replay_bag.py --measure, spin12 bag):
        # IMU 扣完重力的水平加速度跟 ground truth 的真實加速度差多少 ——
        # |w|<0.2 時 0.064 m/s^2, |w|>12 時 14.7 m/s^2, 差 230 倍。
        # 只給常數的話直線行駛時 P 被灌大幾十倍 (速度偽量測的增益逼近 1,
        # 一次更新就把位置搬走好幾公尺), 高速自旋時 P 又太小 (正確的 ZUPT/NHC
        # 被卡方閘門擋掉)。兩頭都錯而且方向相反。
        p('sigma_acc', 0.03)
        p('sigma_acc_omega', 0.15)
        # 零偏模型: b = b0 (開機校正量掉) + b_dyn (濾波器線上估計)。
        #   gm : 一階 Gauss-Markov, db/dt = -b/tau + n_b
        #   rw : 隨機遊走 (GM 在 tau -> 無限大的極限)
        # tau_* / sigma_gm_* 用 fit_noise.py 對靜止資料做 Allan variance 擬合
        # 得到, 寫在 config/imu_noise_fit.yaml。
        p('bias_model', 'gm')
        p('tau_bg', 300.0)
        p('sigma_gm_bg', 1.2e-3)
        p('tau_ba', 300.0)
        p('sigma_gm_ba', 1.2e-2)
        p('sigma_bg', 1e-4)           # 只有 bias_model: rw 會用到
        p('sigma_ba', 1e-3)
        p('zupt_sigma', 0.02)
        p('zaru_sigma', 0.002)
        p('nhc_sigma', 2.0)           # 不是側滑的大小 —— 見 ins.py 的 nhc
        p('nhc_min_speed', 0.20)
        p('nhc_max_omega', 1.5)
        # NHC 單次更新最多能把位置搬多少 (m)。負數 = 不另外設限 (只受
        # max_pos_correction 管)。見 ins.py 的 nhc。
        p('nhc_pos_cap', -1.0)
        # base_link 的 +X 量到車頭的角度 (度)。REP-103 的車是 0, 但 car.usd 的
        # base_link 是 +X 朝左、車頭 -Y, 所以這台車是 -90。NHC 約束的是「垂直
        # 車頭」的那一軸 —— 給錯的話它會把前進速度當側滑歸零, 比不開還糟。
        p('forward_deg', 0.0)
        p('anchor_sigma', 0.02)
        p('anchor_after', 0.5)
        p('v_max', 4.0)
        # 單次更新最多能把位置搬多少 (m)。ZUPT/ZARU/NHC 都是**速度**的偽量測,
        # 能改到位置全靠 P 裡的位置-速度相關項; P 一大, 一個側滑 innovation
        # 就能瞬間搬走幾公尺 (實測最大 27 公尺, 同一步車子只走 4 公分)。
        # 卡方閘門擋不住這種事 —— P 大 S 也大, NIS 反而變小。0 = 不設限。
        p('max_pos_correction', 0.5)

        # 原地自旋時宣告「平移速度 = 0」。ZUPT 要求陀螺儀也要小, 所以
        # skid-steer 原地打轉時完全不會觸發 —— 但那時平移速度確實是 0。
        # **預設關**: 偵測本身是準的 (spin12 bag 實測 85% 精確度), 但端到端
        # 的誤差量不出差別 (RMS 14.32 -> 14.18 m, 在雜訊裡)。留著是因為那是
        # 真的有一個洞, 而且你自己的 bag 可能不一樣 ——
        #   python3 test/replay_bag.py <bag>   會把開/關兩列一起印出來。
        p('enable_spin_zupt', False)
        p('spin_omega_min', 0.5)
        p('spin_omega_max', 6.0)     # 60 Hz 撐不住更快的自旋, 硬做反而更糟
        p('spin_acc_th', 0.2)
        p('spin_zupt_sigma', 0.15)

        p('enable_zupt', True)
        p('enable_zaru', True)
        p('enable_nhc', True)
        p('enable_anchor', True)

        p('publish_rate', 0.0)        # 0 = 每則 IMU 都發 (通常就是 60 Hz)
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
        self.en = {k: bool(g('enable_' + k).value)
                   for k in ('zupt', 'zaru', 'nhc', 'anchor', 'spin_zupt')}

        self.ins = self._make_ins()
        self.det = StillDetector(gyro_th=float(g('still_gyro').value),
                                 acc_th=float(g('still_acc').value),
                                 var_th=float(g('still_var').value),
                                 window=float(g('still_window').value))
        self.confirm = StillConfirm(
            window=float(g('still_window').value),
            acc_mean=float(g('still_acc_mean').value),
            gyro_mean=float(g('still_gyro_mean').value),
            speed_gate=float(g('still_speed_gate').value),
            speed_frac=float(g('still_speed_frac').value),
            trust=float(g('still_speed_trust').value),
            speed_max_omega=float(g('still_speed_max_omega').value))
        self.tilt = TiltTracker(alpha=float(g('tilt_alpha').value))
        self.spin_det = SpinDetector(omega_th=float(g('spin_omega_min').value),
                                     omega_max=float(g('spin_omega_max').value),
                                     acc_th=float(g('spin_acc_th').value))

        self.initial_pose = list(g('initial_pose').value)
        self.started = False
        self.t0 = None
        self.calib_buf = []
        self.calib_start = None
        self.calib_done = False
        self.n_imu = 0
        self.n_still = 0
        self.last_still = False
        self.last_t = None
        self.travelled = 0.0
        self._last_pos = None

        self.pub_odom = self.create_publisher(Odometry, g('odom_topic').value, 20)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, g('pose_topic').value, 20)
        ct = g('corrected_imu_topic').value
        self.pub_imu = self.create_publisher(Imu, ct, 20) if ct else None
        self.tf = tf2_ros.TransformBroadcaster(self) if self.do_tf else None
        self.create_subscription(Imu, g('imu_topic').value, self.on_imu, SENSOR_QOS)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_timer(float(g('status_period').value), self.status)
        self.get_logger().info(
            f"等 {g('imu_topic').value} ... (只用 IMU, 不吃 LiDAR / 相機) | "
            f'重力: {self.gravity_mode}, yaw: {self.yaw_source} | '
            f'零偏模型: {self._bias_model_text()} | '
            f'車頭 {self.ins.forward_deg:+.0f}° (從 base_link +X 量起) | '
            f"防線: {', '.join(k for k, v in self.en.items() if v) or '全關'}")
        if self.gravity_mode == 'complementary' and self.ins.sa < 0.2:
            self.get_logger().warn(
                f'gravity_mode=complementary 但 sigma_acc 只有 {self.ins.sa} —— '
                '傾角是估出來的, 那個估計誤差本身就是持續存在的假加速度 '
                '(傾斜 1 度 = 0.17 m/s^2), 過程雜訊蓋不住的話 NHC 會一直被'
                '卡方閘門擋掉。6 軸 IMU 請給 sigma_acc:=0.35。')

    # ------------------------------------------------------------------
    def _bias_model_text(self) -> str:
        i = self.ins
        if i.bias_model == 'gm':
            return (f'gm (陀螺 tau {i.tau_bg:.0f} s / sigma {i.sgm_bg:.2e}, '
                    f'加速度 tau {i.tau_ba:.0f} s / sigma {i.sgm_ba:.2e})')
        return f'rw (陀螺 {i.sbg:.1e}, 加速度 {i.sba:.1e})'

    def _make_ins(self) -> ImuIns:
        """照參數建一個濾波器。~/reset 也走這裡 —— 直接 ImuIns() 會把
        forward_deg 之類的設定悄悄變回預設值, 重設之後就換了一組行為。"""
        g = self.get_parameter
        return ImuIns(
            sigma_gyro=float(g('sigma_gyro').value),
            sigma_gyro_scale=float(g('sigma_gyro_scale').value),
            sigma_acc=float(g('sigma_acc').value),
            sigma_acc_omega=float(g('sigma_acc_omega').value),
            bias_model=g('bias_model').value,
            tau_bg=float(g('tau_bg').value),
            sigma_gm_bg=float(g('sigma_gm_bg').value),
            tau_ba=float(g('tau_ba').value),
            sigma_gm_ba=float(g('sigma_gm_ba').value),
            sigma_bg=float(g('sigma_bg').value),
            sigma_ba=float(g('sigma_ba').value),
            zupt_sigma=float(g('zupt_sigma').value),
            zaru_sigma=float(g('zaru_sigma').value),
            nhc_sigma=float(g('nhc_sigma').value),
            spin_zupt_sigma=float(g('spin_zupt_sigma').value),
            nhc_min_speed=float(g('nhc_min_speed').value),
            nhc_max_omega=float(g('nhc_max_omega').value),
            forward_deg=float(g('forward_deg').value),
            anchor_sigma=float(g('anchor_sigma').value),
            anchor_after=float(g('anchor_after').value),
            v_max=float(g('v_max').value),
            max_pos_correction=float(g('max_pos_correction').value),
            nhc_pos_cap=(None if float(g('nhc_pos_cap').value) < 0.0
                         else float(g('nhc_pos_cap').value)))

    # ------------------------------------------------------------------
    def on_reset(self, req, res):
        ip = self.initial_pose
        self.ins = self._make_ins()
        self.ins.set_pose(ip[0], ip[1], math.radians(ip[2]))
        self.started = False
        self.calib_done = False
        self.calib_buf.clear()
        self.calib_start = None
        self.confirm.reset()
        self.travelled = 0.0
        self._last_pos = None
        res.success = True
        res.message = f'已重設到 ({ip[0]:.2f}, {ip[1]:.2f}, {ip[2]:.1f}°) 並重新校正零偏'
        self.get_logger().warn(res.message)
        return res

    # ------------------------------------------------------------------
    def _gravity_free(self, msg, gyro, acc, dt, still):
        """回傳「扣掉重力之後的車體水平加速度」(2,)。"""
        if self.gravity_mode == 'orientation':
            q = msg.orientation
            R = quat_to_matrix((q.x, q.y, q.z, q.w))
            aw = R @ acc - np.array([0.0, 0.0, G])
            # 轉回車體水平面 (只去掉 yaw —— yaw 由狀態自己維護)
            yaw = quat_to_yaw((q.x, q.y, q.z, q.w))
            c, s = math.cos(yaw), math.sin(yaw)
            return np.array([c * aw[0] + s * aw[1], -s * aw[0] + c * aw[1]])
        if self.gravity_mode == 'complementary':
            if dt is not None and dt > 0:
                # trust_accel 由靜止偵測決定 —— 用 |a|≈g 當條件幾乎沒有鑑別力,
                # 會讓傾角估計去追隨車子的加速度 (見 TiltTracker 的說明)
                self.tilt.update(gyro, acc, dt, trust_accel=still)
            return self.tilt.gravity_free(acc)
        return np.asarray(acc, dtype=np.float64)[:2]

    # ------------------------------------------------------------------
    def on_imu(self, msg: Imu):
        t = stamp_sec(msg.header.stamp)
        self.n_imu += 1
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                         msg.angular_velocity.z])
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        self.det.add(t, gyro, acc)
        still = self.det.is_still()
        dt = None if self.last_t is None else t - self.last_t
        self.last_t = t

        if not self.calib_done:
            if self._calibrate(t, msg, gyro, acc, still):
                return

        acc_xy = self._gravity_free(msg, gyro, acc, dt, still)
        still = self.confirm.update(t, dt, still, acc_xy, float(gyro[2]), self.ins)
        self.last_still = still
        self.n_still += int(still)

        self.ins.predict(t, acc_xy, float(gyro[2]))

        if self.yaw_source == 'imu_orientation':
            q = msg.orientation
            self.ins.update_yaw(quat_to_yaw((q.x, q.y, q.z, q.w)),
                                self.imu_yaw_sigma, dt or 0.0)

        if still and (self.en['zupt'] or self.en['zaru']):
            if self.ins.still_since is None:
                self.ins.still_since = t
            if self.en['zupt']:
                self.ins.zupt()
                self.ins.zero_accel(acc_xy)
            if self.en['zaru']:
                self.ins.zaru(float(gyro[2]))
            if self.en['anchor'] and t - self.ins.still_since >= self.ins.anchor_after:
                self.ins.anchor_position()
        else:
            self.ins.still_since = None
            self.ins.anchor = None

        self.spin_det.add(t, float(gyro[2]), acc_xy)
        if (self.en['spin_zupt'] and not still
                and self.spin_det.is_spinning_in_place()):
            self.ins.spin_zupt()

        if self.en['nhc']:
            self.ins.nhc()

        pos = self.ins.pos
        if self._last_pos is not None:
            self.travelled += float(np.linalg.norm(pos - self._last_pos))
        self._last_pos = pos
        self.publish(msg)

    # ------------------------------------------------------------------
    def _calibrate(self, t, msg, gyro, acc, still) -> bool:
        """開機靜止校正。回傳 True 表示「這一則先不要進濾波器」。"""
        if self.t0 is None:
            self.t0 = t
        if still:
            if self.calib_start is None:
                self.calib_start = t
            self.calib_buf.append((gyro.copy(), acc.copy()))
        else:
            self.calib_start = None
            self.calib_buf.clear()

        elapsed_still = 0.0 if self.calib_start is None else t - self.calib_start
        if elapsed_still >= self.calib_time and len(self.calib_buf) >= 20:
            gm = np.mean([b[0] for b in self.calib_buf], axis=0)
            am = np.mean([b[1] for b in self.calib_buf], axis=0)
            if self.gravity_mode == 'complementary':
                # 靜止時加速度的方向就是重力反方向 —— 直接定出 roll/pitch
                self.tilt.roll = math.atan2(am[1], am[2])
                self.tilt.pitch = math.atan2(-am[0], math.hypot(am[1], am[2]))
                self.tilt.inited = True
            res = self._gravity_free(msg, gm, am, None, True)
            self.ins.set_bias0(float(gm[2]), res)  # b0: 之後濾波器只估動態部分
            self._start(t, msg)
            self.get_logger().info(
                f'靜止校正完成 ({len(self.calib_buf)} 筆): '
                f'陀螺零偏 z {gm[2] * 1e3:+.2f} mrad/s, '
                f'加速度零偏 ({res[0]:+.4f}, {res[1]:+.4f}) m/s^2'
                + (f', 傾角 roll {math.degrees(self.tilt.roll):+.2f}° '
                   f'pitch {math.degrees(self.tilt.pitch):+.2f}°'
                   if self.gravity_mode == 'complementary' else ''))
            return True

        if t - self.t0 >= self.calib_timeout:
            self.get_logger().warn(
                f'等了 {self.calib_timeout:.0f} 秒都沒有連續靜止 {self.calib_time:.1f} 秒, '
                '零偏當 0 開始推。**車子一開始就在動的話漂移會明顯大很多** —— '
                '下次請讓車子先停著幾秒再開。')
            if self.ins.bias_model == 'gm':
                # GM 的衰減項假設「扣掉 b0 之後的零偏平均是 0」。b0 沒量到的話
                # 它會把濾波器學到的零偏一直往 0 拉, 所以這一輪退回隨機遊走。
                self.ins.bias_model = 'rw'
                self.get_logger().warn(
                    '開機零偏 b0 沒量到, 零偏模型這一輪退回 rw (隨機遊走)。')
            # 靜止的加速度 / 角速度確認是拿估計的零偏當基準 —— b0 不知道的話
            # 基準是錯的, 會把真的靜止全部否決掉, 零偏就永遠沒機會被修。
            self.confirm.acc_mean = 0.0
            self.confirm.gyro_mean = 0.0
            self._start(t, msg)
            return True
        if self.n_imu % 120 == 1:
            self.get_logger().info(
                f'靜止校正中 ({elapsed_still:.1f}/{self.calib_time:.1f} s) —— '
                '請讓車子不要動')
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
            '—— 純 IMU 只能從一個給定的起點開始推, 它不知道自己在世界的哪裡。')

    # ------------------------------------------------------------------
    def publish(self, msg: Imu):
        if not self.started:
            return
        x, y = self.ins.pos
        yaw = self.ins.yaw
        qz, qw = math.sin(yaw * 0.5), math.cos(yaw * 0.5)
        v = self.ins.vel

        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = self.map_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.pose.covariance = self.ins.pose_cov().ravel().tolist()
        c, s = math.cos(yaw), math.sin(yaw)
        od.twist.twist.linear.x = float(c * v[0] + s * v[1])
        od.twist.twist.linear.y = float(-s * v[0] + c * v[1])
        od.twist.twist.angular.z = float(self.ins.last_omega)
        od.twist.covariance = self.ins.twist_cov().ravel().tolist()
        self.pub_odom.publish(od)

        pc = PoseWithCovarianceStamped()
        pc.header = od.header
        pc.pose.pose = od.pose.pose
        pc.pose.covariance = od.pose.covariance
        self.pub_pose.publish(pc)

        if self.pub_imu is not None:
            # 「去除零偏之後的 IMU」: 陀螺儀 z 與加速度 xy 扣掉目前估計的總零偏
            # (b0 + 動態部分)。白雜訊還在 —— 那一部分原理上扣不掉。
            # 加速度零偏定義在「車身水平」的座標, 傾角大的時候這裡是近似。
            ci = Imu()
            ci.header = msg.header
            ci.orientation = msg.orientation
            ci.orientation_covariance = msg.orientation_covariance
            ci.angular_velocity.x = msg.angular_velocity.x
            ci.angular_velocity.y = msg.angular_velocity.y
            ci.angular_velocity.z = msg.angular_velocity.z - self.ins.gyro_bias
            ba = self.ins.acc_bias
            ci.linear_acceleration.x = msg.linear_acceleration.x - float(ba[0])
            ci.linear_acceleration.y = msg.linear_acceleration.y - float(ba[1])
            ci.linear_acceleration.z = msg.linear_acceleration.z
            ci.angular_velocity_covariance = msg.angular_velocity_covariance
            ci.linear_acceleration_covariance = msg.linear_acceleration_covariance
            self.pub_imu.publish(ci)

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
            self.get_logger().warn('還沒有收到 IMU 資料')
            return
        if not self.started:
            return
        c = self.ins.counts
        ba = self.ins.acc_bias
        self.get_logger().info(
            f'{self.n_imu} 筆 (靜止 {100.0 * self.n_still / self.n_imu:.0f}%) | '
            f'x={self.ins.x[0]:+.3f} y={self.ins.x[1]:+.3f} '
            f'yaw={math.degrees(self.ins.yaw):+.1f}° '
            f'v={self.ins.speed:.2f} (車頭向 {self.ins.forward_speed:+.2f}) m/s | '
            f'sigma {self.ins.sigma_pos() * 100:.1f} cm, 推算里程 {self.travelled:.1f} m | '
            f"ZUPT {c['zupt']} ZARU {c['zaru']} NHC {c['nhc']} 錨定 {c['anchor']} "
            f"限幅 {c['damped']} 自旋 {c['spin']} "
            f"靜止否決 加速度 {self.confirm.rejected['acc']} / 角速度 "
            f"{self.confirm.rejected['gyro']} / 速度 {self.confirm.rejected['speed']} "
            f"擋掉 {c['rejected']} | b_g {self.ins.gyro_bias * 1e3:+.2f} mrad/s, "
            f'b_a ({ba[0]:+.3f}, {ba[1]:+.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = ImuLocalizer()
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
