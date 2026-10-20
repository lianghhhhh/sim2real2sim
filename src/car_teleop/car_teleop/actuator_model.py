#!/usr/bin/env python3
"""模擬端的輪子致動器模型: /joint_command -> /joint_command_isaac。

Isaac 的輪子是理想的速度驅動: 指令下去 1 幀就到速。實體車的 DDSM115 不是 ——
2026-10-20 的懸空輪子測試 (scripts/wheel_bench.py, car_run_data/bench_real.csv,
圖在 car_run_data/bench_compare.png) 量到:

  * 純延遲 ~0.08 s (送指令到回授看到輪子開始動; 含 ESP32 每 20 ms 一次的輪詢)
  * 目標 != 0 (起步、加速、反轉): 輪速以固定 ~30 rad/s^2 直線爬, 不管階躍多大
    斜率都一樣, 反轉時也是同一條斜線直接穿過 0。加速時扭矩只用 ~0.15 N·m,
    所以這是驅動器自己的速度斜坡, 跟負載無關
  * 目標 == 0 (停車): 一階指數衰減, tau ~0.10 s -> 不論從多快停都約 0.34 s

這三件事 PhysX 的參數做不出來 (沒有純延遲; 調 maxForce / damping 只能在懸空時
對, 落地後車重讓它變慢, 也會讓速度環變軟), 所以放在指令進 Isaac 之前:

    控制程式 --/joint_command--> [這個節點] --/joint_command_isaac--> Isaac

  1. 延遲 `delay` 秒
  2. 目標量化成整數 rpm (實體車的驅動只收整數 rpm), 再 clamp 在 ±max_rpm
  3. `command_timeout` 秒沒收到指令 -> 目標歸零 (實體車 wheel_velocity_controller
     的 command_timeout 0.3)
  4. 目標 != 0: 每秒最多變 `accel` rad/s; 目標 == 0: 以 `stop_tau` 衰減,
     小於 `stop_epsilon` 就直接歸零
  5. 最後過一個 `smooth_tau` 的一階低通: 實體車的轉角是圓的 (起步先慢慢加速、
     快到目標時慢慢靠近、停車時減速度也是慢慢加上去), 只有斜坡的話是直角。
     低通本身會多一點延遲, 所以 delay 要跟著減。

參數 (2026-10-20 對 bench_real.csv 擬合): 不加低通時 delay 0.09 / tau 0.09,
誤差 0.116 rad/s; 加 0.04 s 低通時 delay 0.06 / tau 0.08, 誤差 0.107, 停車的
圓角幾乎完全對上。這是「實體車的總延遲」; Isaac 本身還有一兩幀的延遲, 而且每個
環境不一樣 (docker 複製環境跟使用者的 GUI 差 ~0.05 s), 所以預設的 delay 是在
使用者的 Isaac 裡校的 (0.035; docker 複製環境是 0.05), 換環境要用 wheel_bench +
scripts/plot_wheel_bench.py 重對一次。Isaac 每次 Play 的延遲會差 ±0.01~0.015 s, 調到 0.01 s 就夠了。
實體車的 ESP32 每 20 ms 才送一次指令, 所以實體車自己每一次的延遲就會差 0-20 ms,
兩條線不可能每一段都疊在一起, 只能平均對齊。

控制程式 (control_car_node、wheel_bench、MPC) 都照舊發 /joint_command, 跟實體車
同一個介面。要生效, USD 的 ActionGraph_odom/ros2_subscribe_joint_state 的
topicName 要改成 joint_command_isaac; 沒改的話 Isaac 還是直接吃 /joint_command,
這個節點的輸出沒有人收。

時間一律用 ROS 時間 (模擬時跟 /clock), 延遲與斜率都以模擬時間計。
每個週期都送, 就算是 0: Isaac 會一直維持最後一筆。

**還沒驗證的部分**: 資料只有「從 0 起步、停到 0、正負反轉」三種。從一個非 0
速度降到另一個非 0 速度 (例如 10 -> 5) 這裡用 `accel` 的斜坡, 實體車是不是這樣
還沒量過。斜坡在落地時是否還是 30 rad/s^2 也還要用實體車的地面資料確認。
"""
from __future__ import annotations

import math
from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

JOINT_NAMES = ['front_left_joint', 'front_right_joint',
               'rear_left_joint', 'rear_right_joint']
RPM = 2.0 * math.pi / 60.0          # 1 rpm = 0.1047 rad/s


def snap_rpm(w, step):
    """輪速 (rad/s) -> 最接近的 step rpm 的整數倍。step <= 0 = 不量化。"""
    if step <= 0:
        return float(w)
    q = step * RPM
    return round(float(w) / q) * q


class ActuatorModel(Node):

    def __init__(self):
        super().__init__('actuator_model')
        p = self.declare_parameter
        p('input_topic', '/joint_command')
        p('output_topic', '/joint_command_isaac')
        # 從 car_run_data/bench_real.csv 擬合、再在 Isaac 裡校過的 (見檔頭)。
        # 改了之後用 wheel_bench 重跑一次對照。
        p('delay', 0.035)                 # s (使用者的 Isaac, 2026-10-20 校; docker 複製環境是 0.05)
        p('accel', 30.0)                  # rad/s^2, 目標 != 0 時的斜率上限
        p('stop_tau', 0.08)               # s, 目標 == 0 時的衰減時間常數
        p('smooth_tau', 0.04)             # s, 輸出的一階低通 (圓角), 0 = 不加
        p('stop_epsilon', 0.02)           # rad/s, 衰減到這以下直接歸零
        p('rpm_step', 1.0)                # 目標量化 (rpm), 0 = 不量化
        # 實體車的 hardware interface 把指令 clamp 在 ±max_rpm (kros_car_system_io.cpp,
        # hardware.yaml 的 max_rpm 100 = 10.47 rad/s); USD 的上限是 200 rpm。<= 0 = 不限。
        p('max_rpm', 100.0)
        p('command_timeout', 0.3)         # s, 這麼久沒收到指令 -> 目標歸零
        p('rate', 120.0)                  # Hz, 積分與發布的頻率
        # false = 直接轉發 (只保留 timeout), 給 A/B 對照用
        p('enabled', True)
        p('status_period', 2.0)

        g = self.get_parameter
        self.delay = max(float(g('delay').value), 0.0)
        self.accel = float(g('accel').value)
        self.tau = float(g('stop_tau').value)
        self.smooth = max(float(g('smooth_tau').value), 0.0)
        self.eps = float(g('stop_epsilon').value)
        self.rpm_step = float(g('rpm_step').value)
        max_rpm = float(g('max_rpm').value)
        self.wmax = max_rpm * RPM if max_rpm > 0 else math.inf
        self.timeout = float(g('command_timeout').value)
        self.enabled = bool(g('enabled').value)

        self.queue = deque()               # (收到的時間, [四輪目標])
        self.latest = [0.0] * 4            # 最近一筆指令 (合併部分輪子的訊息用)
        self.target = [0.0] * 4            # 延遲之後生效的目標
        self.t_target = -1e9               # 生效的那一筆是什麼時候收到的
        self.ref = [0.0] * 4               # 斜坡 / 衰減之後的輪速
        self.out = [0.0] * 4               # 再過低通 = 送給 Isaac 的輪速
        self.t_last = None

        self.pub = self.create_publisher(JointState, g('output_topic').value, 10)
        self.create_subscription(JointState, g('input_topic').value, self.on_cmd, 10)
        self.create_timer(1.0 / float(g('rate').value), self.tick)
        self.create_timer(float(g('status_period').value), self.status)
        self.get_logger().info(
            f'致動器模型 {g("input_topic").value} -> {g("output_topic").value}: '
            + (f'延遲 {self.delay:g} s, 斜率 {self.accel:g} rad/s², 停車 tau {self.tau:g} s, '
               f'低通 {self.smooth:g} s, '
               f'量化 {self.rpm_step:g} rpm, 上限 {max_rpm:g} rpm, 逾時 {self.timeout:g} s'
               if self.enabled else f'關閉 (直接轉發, 只保留逾時 {self.timeout:g} s)'))

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg: JointState):
        if not msg.velocity:
            return
        if msg.name and len(msg.name) == len(msg.velocity):
            table = dict(zip(msg.name, msg.velocity))
            w = [float(table[n]) if n in table else self.latest[i]
                 for i, n in enumerate(JOINT_NAMES)]
        else:
            w = [float(v) for v in msg.velocity[:4]]
            if len(w) < 4:
                return
        if not all(math.isfinite(v) for v in w):
            return
        self.latest = [min(max(snap_rpm(v, self.rpm_step), -self.wmax), self.wmax) for v in w]
        self.queue.append((self._now(), list(self.latest)))

    def _reset(self):
        self.queue.clear()
        self.latest = [0.0] * 4
        self.target = [0.0] * 4
        self.t_target = -1e9
        self.ref = [0.0] * 4
        self.out = [0.0] * 4

    def tick(self):
        now = self._now()
        if self.t_last is not None and now < self.t_last:
            # 模擬時間倒退 = Isaac 按了 Stop 再 Play, 從頭開始
            self.get_logger().info('模擬時間倒退, 狀態歸零')
            self._reset()
        dt = 0.0 if self.t_last is None else now - self.t_last
        self.t_last = now

        # 1) 延遲: 取「收到時間 <= now - delay」的最新一筆
        horizon = now - (self.delay if self.enabled else 0.0)
        while self.queue and self.queue[0][0] <= horizon:
            self.t_target, self.target = self.queue.popleft()
        # 2) 逾時 (在延遲後的時間軸上算, 跟實體車一樣是驅動器那一端的事)
        target = self.target if horizon - self.t_target <= self.timeout else [0.0] * 4

        if not self.enabled:
            self.ref = list(target)
            self.out = list(target)
        elif dt > 0.0:
            # 3) 斜坡 / 停車衰減
            step = self.accel * dt
            decay = math.exp(-dt / self.tau) if self.tau > 0.0 else 0.0
            # 4) 輸出低通
            k = 1.0 - math.exp(-dt / self.smooth) if self.smooth > 0.0 else 1.0
            for i, g in enumerate(target):
                x = self.ref[i]
                if g == 0.0:
                    x *= decay
                    if abs(x) < self.eps:
                        x = 0.0
                else:
                    x += min(max(g - x, -step), step)
                self.ref[i] = x
                y = self.out[i] + (x - self.out[i]) * k
                if x == 0.0 and abs(y) < self.eps:
                    y = 0.0
                self.out[i] = y
        self._publish(self.out)

    def _publish(self, speeds):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(JOINT_NAMES)
        m.velocity = [float(v) for v in speeds]
        self.pub.publish(m)

    def status(self):
        self.get_logger().info(
            '指令 [' + ' '.join(f'{v:+.2f}' for v in self.latest) + '] -> 輸出 ['
            + ' '.join(f'{v:+.2f}' for v in self.out) + '] rad/s')

    def brake(self):
        """關掉之前補送幾次輪速 0 —— Isaac 會一直維持最後一筆指令。"""
        for _ in range(10):
            self._publish([0.0] * 4)
            rclpy.spin_once(self, timeout_sec=0.02)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorModel()
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
