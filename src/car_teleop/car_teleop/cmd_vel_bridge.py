#!/usr/bin/env python3
"""/cmd_vel (Twist) -> /joint_command (JointState effort) 的速度控制器。

為什麼需要這個節點, 而不是直接發 effort:

    car.usd 的車子是**扭矩控制**的 —— /joint_command 給的是 effort, 不是速度。
    而這台車幾乎沒有滾動阻力: 從 car_run_data/sim_data.csv 量到, 固定 effort 3.7
    的情況下車速從 0.87 m/s 一路加速到 2.24 m/s (2 秒), 完全沒有收斂的跡象。
    也就是說「按著前進鍵」在開迴路下等於「一路加速到撞牆」, 而且放開鍵也不會停,
    因為沒有阻力讓它慢下來。

    所以這一層是必要的: 把「我要 0.3 m/s」翻譯成扭矩, 而且放開鍵時要**主動煞停**
    (指令逾時 -> 目標速度 0, 控制器會自己給反向扭矩), 不是把 effort 歸零。

回授來源:
    線速度 <- /joint_states 的輪速 (Isaac 的 ROS2PublishJointState 有發)。
             實測沒打滑時 r*w 跟真值差 0.002 m/s; 打滑/撞牆時輪速會飆高,
             控制器因此會自動收油 —— 這是想要的行為。
    角速度 <- /imu 的 gyro z。這是直接量測, 比用輪速差推算可靠得多
             (skid-steer 轉彎時輪子一定在滑)。

這個節點不碰任何 ground truth, 用的都是真車上也有的感測器。
"""
from __future__ import annotations


import numpy as np

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

        # 從 car.usd 量出來的
        p('wheel_radius', 0.075)      # cylinder radius 0.5 * scale 0.15
        p('wheel_track', 0.25)        # 左右輪距 (±0.125)
        p('max_effort', 10.0)

        # 安全上限。建圖用的預設值刻意訂得很慢 —— 這台車 effort 開到 4 就有
        # 3.6 m/s, 在 10x6 的房間裡兩秒就撞牆。
        p('max_linear', 0.6)          # m/s
        p('max_angular', 1.2)         # rad/s
        p('accel_limit', 0.8)         # m/s^2, 對「目標速度」做斜率限制
        p('angular_accel_limit', 2.5)  # rad/s^2

        # PI 增益。由 sim_data.csv 回歸出的 a=0.34*throttle / alpha=0.57*steer
        # 推出來的量級, 不是精細調校的結果 —— 在 Isaac 裡覺得軟或會晃就調這裡。
        p('kp_v', 3.0)
        p('ki_v', 1.5)
        p('kp_w', 2.0)
        p('ki_w', 1.0)
        p('i_clamp', 5.0)

        p('control_rate', 50.0)
        p('cmd_timeout', 0.5)         # 這麼久沒收到 cmd_vel -> 目標速度歸零 (主動煞停)
        p('feedback_timeout', 0.5)    # 感測器這麼久沒來 -> 退回開迴路, 並且限制輸出
        p('status_period', 2.0)

        g = self.get_parameter
        self.r = float(g('wheel_radius').value)
        self.track = float(g('wheel_track').value)
        self.max_effort = float(g('max_effort').value)
        self.max_v = float(g('max_linear').value)
        self.max_w = float(g('max_angular').value)
        self.acc = float(g('accel_limit').value)
        self.aacc = float(g('angular_accel_limit').value)
        self.kp_v, self.ki_v = float(g('kp_v').value), float(g('ki_v').value)
        self.kp_w, self.ki_w = float(g('kp_w').value), float(g('ki_w').value)
        self.i_clamp = float(g('i_clamp').value)
        self.cmd_timeout = float(g('cmd_timeout').value)
        self.fb_timeout = float(g('feedback_timeout').value)

        self.v_req = self.w_req = 0.0        # 使用者要的
        self.v_cmd = self.w_cmd = 0.0        # 斜率限制之後的
        self.v_meas = self.w_meas = 0.0
        self.i_v = self.i_w = 0.0
        self.t_cmd = self.t_joint = self.t_imu = -1e9
        self.warned = False

        self.pub = self.create_publisher(JointState, g('joint_command_topic').value, 10)
        self.create_subscription(Twist, g('cmd_vel_topic').value, self.on_cmd, 10)
        self.create_subscription(JointState, g('joint_states_topic').value,
                                 self.on_joint, QOS)
        self.create_subscription(Imu, g('imu_topic').value, self.on_imu, QOS)

        self.dt = 1.0 / float(g('control_rate').value)
        self.create_timer(self.dt, self.tick)
        self.create_timer(float(g('status_period').value), self.status)
        self.get_logger().info(
            f'速度控制器啟動: 上限 {self.max_v} m/s / {self.max_w} rad/s, '
            f'{1 / self.dt:.0f} Hz\n'
            f'  訂閱 {g("cmd_vel_topic").value}, 回授 '
            f'{g("joint_states_topic").value} + {g("imu_topic").value}')

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg: Twist):
        # linear.x 一律當「車頭方向的速度」。car.usd 的 base_link 其實是 x 朝左、
        # 車頭 -Y, 但所有現成的遙控工具 (teleop_twist_keyboard、Foxglove 的 Teleop
        # 面板、搖桿) 都是把前進放在 linear.x, 所以這裡照它們的慣例。
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
        self.v_meas = self.r * float(np.mean(w))
        self.t_joint = self._now()

    def on_imu(self, msg: Imu):
        self.w_meas = float(msg.angular_velocity.z)
        self.t_imu = self._now()

    # ------------------------------------------------------------------
    def tick(self):
        now = self._now()
        if now - self.t_cmd > self.cmd_timeout:
            # 逾時不是「把 effort 歸零」而是「目標速度歸零」—— 這台車沒有阻力,
            # 鬆油門它會一直滑下去, 必須主動煞。
            self.v_req = self.w_req = 0.0

        self.v_cmd += clamp(self.v_req - self.v_cmd, -self.acc * self.dt,
                            self.acc * self.dt)
        self.w_cmd += clamp(self.w_req - self.w_cmd, -self.aacc * self.dt,
                            self.aacc * self.dt)

        fb_ok = (now - self.t_joint < self.fb_timeout
                 and now - self.t_imu < self.fb_timeout)
        if not fb_ok:
            if not self.warned:
                self.get_logger().warn(
                    '收不到 /joint_states 或 /imu -> 沒有速度回授。'
                    '這台車開迴路會一路加速, 所以先停住不動。'
                    '請確認 Isaac 在 Play。')
                self.warned = True
            self.i_v = self.i_w = 0.0
            self._publish([0.0] * 4)
            return
        self.warned = False

        ev = self.v_cmd - self.v_meas
        ew = self.w_cmd - self.w_meas
        thr_p, steer_p = self.kp_v * ev, self.kp_w * ew
        thr = thr_p + self.i_v
        steer = steer_p + self.i_w

        # 反飽和: 只有在輸出還沒卡在上限、或誤差方向會把它拉離上限時才積分
        lim = self.max_effort
        if abs(thr) < lim or ev * thr < 0:
            self.i_v = clamp(self.i_v + self.ki_v * ev * self.dt,
                             -self.i_clamp, self.i_clamp)
        if abs(steer) < lim or ew * steer < 0:
            self.i_w = clamp(self.i_w + self.ki_w * ew * self.dt,
                             -self.i_clamp, self.i_clamp)
        thr = clamp(thr_p + self.i_v, -lim, lim)
        steer = clamp(steer_p + self.i_w, -lim, lim)

        # 混控: [FL, FR, RL, RR]。steer > 0 -> 右側扭矩大 -> wz > 0
        # (照 car.usd 實測: 左 -10 / 右 +10 得到 wz = +16.4 rad/s)
        left = clamp(thr - steer, -lim, lim)
        right = clamp(thr + steer, -lim, lim)
        self._publish([left, right, left, right])

    def _publish(self, efforts):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(JOINT_NAMES)
        m.effort = [float(v) for v in efforts]
        self.pub.publish(m)

    def status(self):
        self.get_logger().info(
            f'目標 v={self.v_cmd:+.2f} w={self.w_cmd:+.2f} | '
            f'實測 v={self.v_meas:+.2f} m/s w={self.w_meas:+.2f} rad/s | '
            f'積分 {self.i_v:+.2f}/{self.i_w:+.2f}')

    def brake(self):
        """關掉之前先把車子停下來 —— 沒有阻力, 不主動煞它會一直滑。"""
        for _ in range(20):
            ev = -self.v_meas
            ew = -self.w_meas
            thr = clamp(4.0 * ev, -self.max_effort, self.max_effort)
            steer = clamp(3.0 * ew, -self.max_effort, self.max_effort)
            self._publish([thr - steer, thr + steer, thr - steer, thr + steer])
            rclpy.spin_once(self, timeout_sec=0.02)
        self._publish([0.0] * 4)


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
