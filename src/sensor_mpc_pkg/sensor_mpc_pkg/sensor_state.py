"""感測器定位 -> MPC 的 6 維狀態, 外加「現在能不能信」的判斷。

狀態 (跟 mlp_control_pkg/utils.py 的 STATE 一樣): [x, y, theta, vx, vy, wz]
  (舊版是 10 維, 後面加四輪輪速 wFL, wFR, wRL, wRR; 模型改成不含輪速後, 輪速那段先註解掉。
   joint_states_topic 還是會訂閱 (n_joint 計數), 但不再用來組狀態, 也不再擋 get_state)

| 分量 | 來源 | 備註 |
| --- | --- | --- |
| x, y, theta | odom_topic 的 pose (/fusion_loc/odom) | theta 在這裡 unwrap 成連續的 (MPC 的 CasADi 不 wrap) |
| vx, wz | odom 的 twist.linear.x / angular.z | 融合節點給的是車頭方向速度與 (扣零偏的) gyro |
| vy | 預設 0 | 融合不估側向速度; vy_source:=diff 用位姿差分算 (旋到車體座標 + 低通) |
| ~~四輪輪速~~ | ~~joint_states_topic (模擬 /joint_states, 實體車 /wheel_feedback)~~ | 已註解掉 (模型不含輪速) |

velocity_source:=diff 時 vx, wz 也改用位姿差分 (拿來跟 twist 版本比較用; 平常用 twist)。

延遲補償 (pose_lead_mode):
  * none  (預設) 不補
  * stamp 用「現在 - 位姿訊息的時戳」當延遲 (= 定位鏈的處理延遲), 再加 pose_lead_extra
  * fixed 只用 pose_lead_extra (常數)
  補法: 沿車頭方向用 vx、wz 外插 lead 秒 (x += vx cosθ, y += vx sinθ, θ += wz)。最多 pose_lead_max。
  每個週期的延遲 (pose_age) 與實際用的 lead 會記進 MPC 的 CSV, 先看 pose_age 再決定要不要補:
  模擬裡 8 台相機那條量到位置落後 GT 約 0.25 s (相機時戳的問題, 融合用 bbox_extra_delay 補比較對),
  LiDAR 單獨那條沒有落後。

能不能信 (get_state() 回 (state, None) 才可以用, 否則 (None, 原因)):
  * 位姿超過 pose_timeout 沒更新 (輪速的 wheel_timeout 檢查已註解掉)
  * 位姿 1-sigma (sqrt(cov_xx + cov_yy)) > pose_sigma_max
  * 絕對量測 (相機 / LiDAR) 斷超過 abs_age_max (融合的 abs_age topic; 收不到就不檢查,
    跟 calibrate_env_pkg 的 control_car_node 同一套規則 —— 這一條比 sigma 可靠)
Isaac 的 GT /odom 也可以餵進來 (odom_topic:=/odom) 當 A/B, 那時 sigma / abs_age 都不存在,
自然不會擋。
"""
from __future__ import annotations

import math

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

WHEELS = ['front_left_joint', 'front_right_joint', 'rear_left_joint', 'rear_right_joint']

ODOM_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=10)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class SensorStateNode(Node):

    def __init__(self):
        super().__init__('sensor_state_node')
        p = self.declare_parameter
        p('odom_topic', '/fusion_loc/odom')
        p('joint_states_topic', '/joint_states')     # 實體車 /wheel_feedback
        p('abs_age_topic', '')                       # 空 = 由 odom_topic 推 (/xxx/odom -> /xxx/abs_age)
        p('gt_topic', '')                            # 模擬裡拿 Isaac 的 /odom 對答案 (只記錄, 不控制)
        p('velocity_source', 'twist')                # twist | diff
        p('vy_source', 'zero')                       # zero | diff
        p('diff_alpha', 0.4)                         # 差分速度的低通 (新值的權重)
        p('pose_timeout', 0.3)                       # s
        p('wheel_timeout', 0.3)                      # s
        p('pose_sigma_max', 0.3)                     # m; 0 = 不檢查
        p('abs_age_max', 1.0)                        # s; 0 = 不檢查
        p('pose_lead_mode', 'none')                  # none | stamp | fixed (見檔頭)
        p('pose_lead_extra', 0.0)                    # s
        p('pose_lead_max', 0.5)                      # s
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.odom_topic = str(g('odom_topic'))
        self.velocity_source = str(g('velocity_source'))
        self.vy_source = str(g('vy_source'))
        if self.velocity_source not in ('twist', 'diff'):
            raise ValueError(f'velocity_source 要是 twist 或 diff, 收到 {self.velocity_source!r}')
        if self.vy_source not in ('zero', 'diff'):
            raise ValueError(f'vy_source 要是 zero 或 diff, 收到 {self.vy_source!r}')
        self.alpha = float(g('diff_alpha'))
        self.pose_timeout = float(g('pose_timeout'))
        self.wheel_timeout = float(g('wheel_timeout'))
        self.pose_sigma_max = float(g('pose_sigma_max'))
        self.abs_age_max = float(g('abs_age_max'))
        self.lead_mode = str(g('pose_lead_mode'))
        if self.lead_mode not in ('none', 'stamp', 'fixed'):
            raise ValueError(f'pose_lead_mode 要是 none / stamp / fixed, 收到 {self.lead_mode!r}')
        self.lead_extra = float(g('pose_lead_extra'))
        self.lead_max = float(g('pose_lead_max'))
        self.last_pose_age = float('nan')   # 最近一次 get_state 時位姿訊息幾秒前的 (診斷用)
        self.last_lead = 0.0                # 最近一次實際外插了幾秒

        self.latest_odom = None
        self.latest_joint = None
        self.t_odom = None          # 收到的時刻 (節點時鐘)
        self.t_joint = None
        self._theta = None          # unwrap 過的 yaw
        self._prev = None           # (stamp, x, y, yaw) 差分用
        self.v_diff = np.zeros(3)   # 差分出來的 (vx, vy, wz), 車體座標, 低通過
        self.pose_sigma = 0.0
        self.abs_age = None         # (值, 收到的時刻)
        self.gt = None              # (x, y, yaw)
        self.n_odom = self.n_joint = 0

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, ODOM_QOS)
        self.create_subscription(JointState, str(g('joint_states_topic')), self.on_joint, ODOM_QOS)
        age_topic = str(g('abs_age_topic')) or (
            self.odom_topic[:-len('odom')] + 'abs_age' if self.odom_topic.endswith('/odom') else '')
        self.age_topic = age_topic
        if age_topic and self.abs_age_max > 0:
            self.create_subscription(Float32, age_topic, self.on_abs_age, 10)
        gt_topic = str(g('gt_topic'))
        if gt_topic and gt_topic != self.odom_topic:
            self.create_subscription(Odometry, gt_topic, self.on_gt, ODOM_QOS)
        self.get_logger().info(
            f"位姿 {self.odom_topic}, 輪速 {g('joint_states_topic')}, "
            f"速度 {self.velocity_source} / vy {self.vy_source}"
            + (f', abs_age {age_topic} (上限 {self.abs_age_max} s)' if age_topic and self.abs_age_max > 0 else '')
            + (f', GT {gt_topic} (只記錄)' if gt_topic else '')
            + (f', 延遲補償 {self.lead_mode} +{self.lead_extra} s' if self.lead_mode != 'none' else ''))

    # ------------------------------------------------------------------ callbacks
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        self.latest_odom = msg
        self.t_odom = self._now()
        self.n_odom += 1
        cov = msg.pose.covariance
        s = cov[0] + cov[7]
        self.pose_sigma = math.sqrt(s) if s > 0.0 else 0.0

        # 位姿差分 (用訊息自己的時戳; 沒填的話用收到時間)
        t = stamp_sec(msg.header) or self.t_odom
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = yaw_of(msg.pose.pose.orientation)
        if self._prev is not None:
            t0, x0, y0, yaw0 = self._prev
            dt = t - t0
            if 1e-3 < dt < 0.5:
                dx, dy = x - x0, y - y0
                c, s_ = math.cos(yaw0), math.sin(yaw0)
                v = np.array([(c * dx + s_ * dy) / dt, (-s_ * dx + c * dy) / dt,
                              wrap(yaw - yaw0) / dt])
                self.v_diff = (1.0 - self.alpha) * self.v_diff + self.alpha * v
        if self._prev is None or t > self._prev[0]:
            self._prev = (t, x, y, yaw)

    def on_joint(self, msg: JointState):
        self.latest_joint = msg
        self.t_joint = self._now()
        self.n_joint += 1

    def on_abs_age(self, msg: Float32):
        self.abs_age = (float(msg.data), self._now())

    def on_gt(self, msg: Odometry):
        self.gt = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                   yaw_of(msg.pose.pose.orientation))

    # ------------------------------------------------------------------ 查詢
    def abs_age_now(self):
        if self.abs_age is None:
            return None
        v, t = self.abs_age
        return v + max(self._now() - t, 0.0)

    def reset(self):
        """模擬時間倒退 (Isaac Stop / Play) 時由控制節點呼叫: 舊的訊息與 unwrap 都作廢。"""
        self.latest_odom = self.latest_joint = None
        self.t_odom = self.t_joint = None
        self._theta = None
        self._prev = None
        self.v_diff = np.zeros(3)
        self.abs_age = None
        self.gt = None

    def health(self):
        """None = 可以用; 否則回不能用的原因 (字串)。"""
        now = self._now()
        if self.latest_odom is None:
            return f'還沒收到位姿 {self.odom_topic}'
        # if self.latest_joint is None:        # 舊版: 狀態要輪速
        #     return '還沒收到輪速回授'
        if now - self.t_odom > self.pose_timeout:
            return f'位姿 {now - self.t_odom:.2f} s 沒更新'
        # if now - self.t_joint > self.wheel_timeout:
        #     return f'輪速回授 {now - self.t_joint:.2f} s 沒更新'
        if self.pose_sigma_max > 0 and self.pose_sigma > self.pose_sigma_max:
            return f'位姿 sigma {self.pose_sigma:.2f} m > {self.pose_sigma_max}'
        age = self.abs_age_now()
        if self.abs_age_max > 0 and age is not None and age > self.abs_age_max:
            return f'絕對量測斷了 {age:.1f} s (相機 / LiDAR 都沒進來, 只剩遞推)'
        return None

    def get_state(self):
        """回 (state (6,), None) 或 (None, 原因)。 (舊版 (10,), 含四輪輪速)"""
        reason = self.health()
        if reason is not None:
            return None, reason
        js, od = self.latest_joint, self.latest_odom
        # try:                                   # 舊版: 狀態要四輪輪速
        #     idx = [js.name.index(n) for n in WHEELS]
        # except ValueError:
        #     return None, f'輪速回授的關節名字對不上 {WHEELS} (收到 {list(js.name)})'
        # if len(js.velocity) <= max(idx):
        #     return None, '輪速回授沒有 velocity'

        pose, twist = od.pose.pose, od.twist.twist
        yaw = yaw_of(pose.orientation)
        self._theta = yaw if self._theta is None else self._theta + wrap(yaw - self._theta)

        if self.velocity_source == 'twist':
            vx, wz = twist.linear.x, twist.angular.z
        else:
            vx, wz = self.v_diff[0], self.v_diff[2]
        vy = self.v_diff[1] if self.vy_source == 'diff' else 0.0

        # 延遲補償: 位姿是 lead 秒前的, 用目前的速度往前推
        stamp = stamp_sec(od.header)
        self.last_pose_age = (self._now() - stamp) if stamp > 0 else float('nan')
        lead = 0.0
        if self.lead_mode == 'fixed':
            lead = self.lead_extra
        elif self.lead_mode == 'stamp':
            lead = self.lead_extra + (self.last_pose_age if math.isfinite(self.last_pose_age) else 0.0)
        lead = min(max(lead, 0.0), self.lead_max)
        self.last_lead = lead
        x, y, th = pose.position.x, pose.position.y, self._theta
        if lead > 0.0:
            th_mid = th + 0.5 * wz * lead
            x += (vx * math.cos(th_mid) - vy * math.sin(th_mid)) * lead
            y += (vx * math.sin(th_mid) + vy * math.cos(th_mid)) * lead
            th += wz * lead

        return np.array([
            x, y, th,
            vx, vy, wz,
            # *[js.velocity[i] for i in idx],   # 四輪輪速 (模型已不含)
        ], dtype=float), None
