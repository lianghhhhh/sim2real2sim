#!/usr/bin/env python3
"""/cmd_vel (Twist) -> /joint_command (JointState velocity) 的輪速換算。

這台車 (kros_car.usd / 實體 KROS car) 是**輪速控制**: /joint_command 的 `velocity`
放四輪的 rad/s, 馬達自己的速度環去追。所以這一層只做運動學換算, 不需要像舊的
effort 版那樣自己做 PI 控制器:

    左輪 = (v - w * T_eff / 2) / r        右輪 = (v + w * T_eff / 2) / r

T_eff 是 skid-steer 的**等效輪距**, 不是幾何輪距 (0.27): 轉彎時四個輪子都在橫向
滑, 車體實際轉得比「輪速差 / 輪距」慢。理想 Coulomb 摩擦下
    T_eff = 2 * (a^2 + b^2) / b = 0.577       (a 半軸距 0.144, b 半輪距 0.135)
無頭 PhysX 實測是 0.68~1.04 (mu 大、轉得慢時比較大); 2026-10-13 kros_car.usd
(12.5 kg, 地面 0.5) 兩輪實跑量到 0.945 / 1.055, 預設取 0.95。

兩種模式 (`yaw_trim_gain`):

  * **0 (預設): 固定輪速。** 同一組 /cmd_vel 永遠換成同一組輪速, 不看任何回授。
    鍵盤的 w/s/a/d 各自對應一組固定的四輪輪速, 模擬與實體車收到的指令完全一樣。
    代價是實際轉多快看地面: 同樣 0.6 rad/s, 換個地板可能差兩三成。
    手動開車 (建圖) 用這個 —— 人看著畫面自己會補。
  * **> 0: 用 gyro 修正轉速。** 把「要求的轉速 - gyro 量到的轉速」積分加回轉速指令,
    angular.z 要 0.5 rad/s 車體就真的轉 0.5 rad/s。給 nav2 這類假設車子照指令動的
    程式用。缺點 (2026-10-14 實測): 原地轉時四輪載重在對角之間跳, 車體轉速一直抖,
    修正會帶著輪速指令起伏; 車子卡住、轉不動時修正會一路加到輪速上限。

兩件跟舊版一樣的事:
  * 指令逾時 (cmd_timeout) -> 目標速度歸零。
  * **每個週期都送**, 就算目標是 0: 實體車 0.3 秒沒收到指令輪子會停, 而 Isaac 會
    一直維持最後一筆 —— 兩邊都靠持續送來保證行為一致。關掉節點前也會補送 0。

這個節點不碰任何 ground truth, 用的都是真車上也有的感測器。
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=10)

JOINT_NAMES = ['front_left_joint', 'front_right_joint',
               'rear_left_joint', 'rear_right_joint']


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')
        p = self.declare_parameter
        p('cmd_vel_topic', '/cmd_vel')
        p('joint_command_topic', '/joint_command')
        p('joint_states_topic', '/joint_states')
        p('imu_topic', '/imu')

        # 從 kros_car.usd 量出來的
        p('wheel_radius', 0.05035)
        p('wheel_track', 0.27)            # 幾何輪距 (接地中心 ±0.135), 只拿來印
        # 等效輪距, 見檔頭。scripts/estimate_friction.py 的 B1 會印實測值。
        p('effective_track', 0.95)
        # 輪速指令上限 (rad/s)。USD 是 200 rpm = 20.94; 實體車目前 max_rpm 100 = 10.47。
        p('max_wheel_speed', 10.0)

        # 安全上限。max_linear 不能超過 max_wheel_speed * r (= 0.50 m/s), 超過的部分
        # 會被輪速上限等比縮掉。
        p('max_linear', 0.5)              # m/s
        p('max_angular', 1.2)             # rad/s
        p('accel_limit', 0.8)             # m/s^2, 對「目標速度」做斜率限制
        p('angular_accel_limit', 2.5)     # rad/s^2

        # 車體轉速的積分修正 (rad/s per rad): 補等效輪距給得不準的部分。
        # 0 = 關掉 = 固定輪速 (預設, 見檔頭); 給 nav2 用時設 1.0。
        p('yaw_trim_gain', 0.0)
        p('yaw_trim_limit', 0.6)          # rad/s, 修正量上限

        p('control_rate', 50.0)
        p('cmd_timeout', 0.5)             # 這麼久沒收到 cmd_vel -> 目標速度歸零
        p('feedback_timeout', 0.5)        # IMU 這麼久沒來 -> 只用前饋
        p('status_period', 2.0)

        g = self.get_parameter
        self.r = float(g('wheel_radius').value)
        self.t_eff = float(g('effective_track').value)
        self.wmax = float(g('max_wheel_speed').value)
        self.max_v = float(g('max_linear').value)
        self.max_w = float(g('max_angular').value)
        self.acc = float(g('accel_limit').value)
        self.aacc = float(g('angular_accel_limit').value)
        self.k_trim = float(g('yaw_trim_gain').value)
        self.trim_lim = float(g('yaw_trim_limit').value)
        self.cmd_timeout = float(g('cmd_timeout').value)
        self.fb_timeout = float(g('feedback_timeout').value)

        self.v_req = self.w_req = 0.0        # 使用者要的
        self.v_cmd = self.w_cmd = 0.0        # 斜率限制之後的
        self.v_meas = self.w_meas = 0.0      # 輪速換算的車速 / gyro 的轉速 (v 只拿來印)
        self.trim = 0.0
        self.t_cmd = self.t_imu = -1e9

        self.pub = self.create_publisher(JointState, g('joint_command_topic').value, 10)
        self.create_subscription(Twist, g('cmd_vel_topic').value, self.on_cmd, 10)
        self.create_subscription(JointState, g('joint_states_topic').value,
                                 self.on_joint, QOS)
        self.create_subscription(Imu, g('imu_topic').value, self.on_imu, QOS)

        self.dt = 1.0 / float(g('control_rate').value)
        self.create_timer(self.dt, self.tick)
        self.create_timer(float(g('status_period').value), self.status)
        self.get_logger().info(
            f'輪速換算啟動: 上限 {self.max_v} m/s / {self.max_w} rad/s, '
            f'輪速 {self.wmax:g} rad/s ({self.wmax * self.r:.2f} m/s), {1 / self.dt:.0f} Hz\n'
            f'  輪半徑 {self.r} m, 等效輪距 {self.t_eff} m (幾何 {g("wheel_track").value})\n'
            f'  訂閱 {g("cmd_vel_topic").value}; '
            + (f'轉速用 {g("imu_topic").value} 修正 (yaw_trim_gain {self.k_trim:g})'
               if self.k_trim > 0.0 else '固定輪速 (yaw_trim_gain 0, 不看 gyro)'))

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg: Twist):
        # linear.x = 車頭方向的速度。kros_car 的 base_link 照 REP-103 (+X 是車頭),
        # 所以跟所有現成的遙控工具 (teleop_twist_keyboard、Foxglove 的 Teleop 面板) 一致。
        self.v_req = clamp(float(msg.linear.x), -self.max_v, self.max_v)
        self.w_req = clamp(float(msg.angular.z), -self.max_w, self.max_w)
        self.t_cmd = self._now()

    def on_joint(self, msg: JointState):
        if not msg.velocity:
            return
        if msg.name and len(msg.name) == len(msg.velocity):
            table = dict(zip(msg.name, msg.velocity))
            w = [float(table.get(n, 0.0)) for n in JOINT_NAMES]
        else:
            w = [float(v) for v in msg.velocity[:4]]
            if len(w) < 4:
                return
        self.v_meas = self.r * sum(w) / 4.0

    def on_imu(self, msg: Imu):
        self.w_meas = float(msg.angular_velocity.z)
        self.t_imu = self._now()

    # ------------------------------------------------------------------
    def wheel_speeds(self, v, w):
        """車體速度 (m/s, rad/s) -> [FL, FR, RL, RR] 輪速 (rad/s)。

        超過輪速上限時左右**等比縮放**, 保住 v:w 的比例 (各自 clamp 的話直線速度
        會吃掉轉向, 車子轉不過來)。"""
        half = 0.5 * self.t_eff * w
        left, right = (v - half) / self.r, (v + half) / self.r
        peak = max(abs(left), abs(right))
        if peak > self.wmax:
            k = self.wmax / peak
            left, right = left * k, right * k
        return [left, right, left, right]

    def tick(self):
        now = self._now()
        if now - self.t_cmd > self.cmd_timeout:
            self.v_req = self.w_req = 0.0

        self.v_cmd += clamp(self.v_req - self.v_cmd, -self.acc * self.dt,
                            self.acc * self.dt)
        self.w_cmd += clamp(self.w_req - self.w_cmd, -self.aacc * self.dt,
                            self.aacc * self.dt)

        # 轉速的積分修正。只在「有要它轉」而且 IMU 有在來的時候積分; 目標轉速是 0
        # 時讓修正量衰減回 0, 免得停著的時候還留著一個偏置讓車子慢慢自轉。
        if (self.k_trim > 0.0 and now - self.t_imu < self.fb_timeout
                and abs(self.w_cmd) > 0.05):
            self.trim = clamp(self.trim + self.k_trim * (self.w_cmd - self.w_meas) * self.dt,
                              -self.trim_lim, self.trim_lim)
        else:
            self.trim *= 0.9

        self._publish(self.wheel_speeds(self.v_cmd, self.w_cmd + self.trim))

    def _publish(self, speeds):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(JOINT_NAMES)
        m.velocity = [float(v) for v in speeds]
        self.pub.publish(m)

    def status(self):
        self.get_logger().info(
            f'目標 v={self.v_cmd:+.2f} w={self.w_cmd:+.2f} | '
            f'實測 v={self.v_meas:+.2f} m/s (輪速) w={self.w_meas:+.2f} rad/s (gyro) | '
            f'轉速修正 {self.trim:+.2f}')

    def brake(self):
        """關掉之前補送幾次輪速 0 —— Isaac 會一直維持最後一筆指令。"""
        for _ in range(10):
            self._publish([0.0] * 4)
            rclpy.spin_once(self, timeout_sec=0.02)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.brake()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
