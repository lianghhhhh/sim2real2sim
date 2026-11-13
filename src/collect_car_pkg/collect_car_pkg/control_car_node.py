#!/usr/bin/env python3
"""動態模型訓練資料的激發駕駛 (只給 Isaac 模擬用, 輪速控制版)。

    ros2 launch collect_car_pkg collect_car.launch.py csv_filename:=dyn_mu05.csv

送隨機的四輪輪速指令到 /joint_command, 讓 calibrate_env_pkg 的 collect_data_node
把指令與車子的反應 (輪速、輪子扭矩、IMU、GT) 記下來, 之後拿去訓練動態模型。

**一直跑到 Ctrl-C**, 結束時補送 0。**沒有撞牆保護**: 給沒有牆的開放場地用, 車子照指令
一直開。OU 的 throttle 偏前進, 10 分鐘大約會離開起點幾十公尺 —— 地面要夠大 (或是無限大
的平面)。

────────────────────────────────────────────────────────────────────────
跟舊的扭矩控制版差在哪
────────────────────────────────────────────────────────────────────────
* 送 JointState.velocity (rad/s), 不是 effort。輪子的速度環 (模擬: actuator_model +
  Isaac 的速度驅動) 自己去追。
* throttle / steer 是正規化的 [-1, 1], 乘上 max_wheel_speed 才是輪速:
      左輪 = throttle - steer,  右輪 = throttle + steer     (steer > 0 = 往左轉, wz > 0)
  超過上限時四輪**等比縮放**, 不各自截斷 (截斷會改變轉彎半徑)。舊版「effort 差 10
  才轉得動」那套校正不需要了: 速度控制下左右輪速有差就會轉。
* 負的 throttle 現在是**倒車**, 不是煞車。煞車 = 指令 0 (速度環主動煞停) 或降到比較
  低的速度; 舊版 Hold-and-Ramp 的「滑行 / 急煞」那兩類已經改掉。
* 送出去的輪速量化成整數 rpm (實體車的 DDSM115 只收整數 rpm), 記下來的指令才會跟
  實體車執行的一樣。
* Ctrl-C 會補送 0: Isaac 會一直維持最後一筆指令。

────────────────────────────────────────────────────────────────────────
激發方式 (mode)
────────────────────────────────────────────────────────────────────────
  ou         (預設) 兩個 Ornstein-Uhlenbeck 過程: throttle 偏前進、steer 左右對稱,
             再過兩層低通 (ou_smooth_tau) -> 平順地連續變化, 像人一直微調油門與方向
  hold_ramp  維持一段時間 -> 換新的目標值; 30% 直接跳 (像按鍵盤), 其餘線性 ramp

預設不做斜率限制 (rate_limit 0): 模型要學到實體車的延遲 (~0.06 s) 與 30 rad/s² 的
加速斜坡 (actuator_model 加上的), 指令被事先抹平就學不到。

同一個 seed = 同一串指令: 產生器只看自己的亂數與狀態 (不看車子), 換一個摩擦係數的
環境跑出來的指令序列完全一樣。

段落標籤 (collect_data_node 會記進 CSV 的 phase / scenario_name 欄):
    /test_phase      measure (激發中) | idle (開始前)
    /test_scenario   mode 名稱 (ou / hold_ramp); 開始前 Start

計時用 ROS 時鐘 (use_sim_time:=true 時是 Isaac 的模擬時間)。產生器每個週期固定前進
1/rate 秒, 不看實際間隔 —— 模擬時鐘 60 Hz 跳一次, 20 Hz 剛好每 3 跳一次。
"""
import math
import random
import signal

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINT_NAMES = ['front_left_joint', 'front_right_joint',
               'rear_left_joint', 'rear_right_joint']
RPM = 2.0 * math.pi / 60.0               # 1 rpm = 0.1047 rad/s
MODES = ('ou', 'hold_ramp')

# Hold-and-Ramp: 換目標值時直接跳 (階躍) 的比例。其餘用 0.3 ~ 1.5 s 線性 ramp。
STEP_PROB = 0.3


def snap_rpm(w, step=1.0):
    """輪速 (rad/s) -> 最接近的 step rpm 的整數倍 (同 calibrate_env_pkg)。step <= 0 = 不量化。"""
    if step <= 0.0:
        return float(w)
    q = step * RPM
    return round(float(w) / q) * q


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _rate_limit(target, current, max_rate, dt):
    step = max_rate * dt
    return current + _clamp(target - current, -step, step)


# ═══════════════════════════════════════════════════════════ 激發產生器
# 都回傳正規化的 (throttle, steer), 範圍 [-1, 1]。

class _OU:
    """兩個 OU 過程, 都在 [-1, 1] 裡跑, 再各自過兩層一階低通。

    throttle: theta 0.4, mu +0.5 (偏前進), sigma 0.6 -> 一個值大約維持 2 ~ 3 s
    steer:    theta 0.12, mu 0 (左右對稱), sigma 0.44 -> 一個方向大約維持 5 s
    (參數沿用舊版; 舊版 steer 直接用扭矩單位跑 sigma 2.2 x 1.4 = 3.08、上限 7,
    換成正規化就是 0.44。)

    為什麼要低通: OU 每一步都加一次獨立的雜訊, 直接送出去的話一半的步數指令方向
    在來回跳 (中位數 27 rad/s² 的鋸齒), 不像人開車。兩層 tau = 0.25 s 之後 (50 分鐘
    的指令統計): 來回跳 52% -> 7% (跟 hold_ramp 差不多), 輪速變化率中位數 27 -> 6 rad/s²、
    p99 86 -> 19 rad/s² (都在實體車的 30 rad/s² 斜坡以內), 可行的 (車速, 轉速) 範圍
    覆蓋 98% -> 97%。smooth_tau <= 0 = 不濾 (舊版的鋸齒)。
    """

    def __init__(self, rng, smooth_tau=0.25):
        self.rng = rng
        self.smooth_tau = smooth_tau
        self.thr = 0.0
        self.st = 0.0
        self.filt = [[0.0, 0.0], [0.0, 0.0]]     # 兩層低通的 (throttle, steer)

    def step(self, dt):
        g, sq = self.rng.gauss, math.sqrt(dt)
        self.thr += 0.40 * (0.50 - self.thr) * dt + 0.60 * g(0.0, 1.0) * sq
        self.thr = _clamp(self.thr, -1.0, 1.0)
        self.st += 0.12 * (0.0 - self.st) * dt + 0.44 * g(0.0, 1.0) * sq
        self.st = _clamp(self.st, -1.0, 1.0)
        if self.smooth_tau <= 0.0:
            return self.thr, self.st
        k = 1.0 - math.exp(-dt / self.smooth_tau)
        x = (self.thr, self.st)
        for f in self.filt:
            f[0] += k * (x[0] - f[0])
            f[1] += k * (x[1] - f[1])
            x = (f[0], f[1])
        return x


def _throttle_target(rng):
    r = rng.random()
    if r < 0.45:
        return rng.uniform(0.4, 1.0)     # 前進巡航
    if r < 0.65:
        return rng.uniform(0.1, 0.4)     # 慢速前進
    if r < 0.80:
        return 0.0                       # 停 (指令 0 = 速度環主動煞停)
    if r < 0.90:
        return rng.uniform(-0.1, 0.1)    # 低速, 前後都有
    return rng.uniform(-0.6, -0.1)       # 倒車


def _steer_target(rng):
    r = rng.random()
    if r < 0.25:
        return rng.uniform(-0.05, 0.05)  # 直行 (舊版只有 10%, 縱向動態要多一點)
    if r < 0.625:
        return rng.uniform(0.05, 1.0)    # 左轉
    return rng.uniform(-1.0, -0.05)      # 右轉


class _HoldRampVar:
    """維持 0.5 ~ 2.5 s -> 換新目標: STEP_PROB 的機率直接跳, 否則 0.3 ~ 1.5 s 線性 ramp。"""

    def __init__(self, rng, target_fn):
        self.rng = rng
        self.target_fn = target_fn
        self.val = self.start = self.target = 0.0
        self.phase = 'hold'
        self.timer = 0.0
        self.hold = rng.uniform(0.5, 2.5)
        self.ramp = 0.0

    def step(self, dt):
        self.timer += dt
        if self.phase == 'hold':
            if self.timer < self.hold:
                return self.val
            self.start = self.val
            self.target = self.target_fn(self.rng)
            self.ramp = 0.0 if self.rng.random() < STEP_PROB else self.rng.uniform(0.3, 1.5)
            self.phase, self.timer = 'ramp', 0.0
        if self.timer >= self.ramp:
            self.val = self.target
            self.phase, self.timer = 'hold', 0.0
            self.hold = self.rng.uniform(0.5, 2.5)
        else:
            self.val = self.start + (self.target - self.start) * self.timer / self.ramp
        return self.val


class _HoldRamp:
    def __init__(self, rng):
        self.thr = _HoldRampVar(rng, _throttle_target)
        self.st = _HoldRampVar(rng, _steer_target)

    def step(self, dt):
        return self.thr.step(dt), self.st.step(dt)


# ═══════════════════════════════════════════════════════════════ 節點
class CollectCarControl(Node):

    def __init__(self):
        super().__init__('collect_car_control')
        p = self.declare_parameter
        p('mode', 'ou')                   # ou | hold_ramp
        p('seed', 0)                      # 同一個 seed = 同一串指令
        p('ou_smooth_tau', 0.25)          # s, OU 兩層低通的時間常數; 0 = 不濾 (見 _OU)
        p('rate', 20.0)                   # Hz, 送指令的頻率
        p('joint_command_topic', '/joint_command')
        # 輪速上限 (rad/s)。10 = 實體車目前的 max_rpm 100 (10.47)。
        p('max_wheel_speed', 10.0)
        # 輪速指令的斜率上限 (rad/s²); 0 = 不限制 (預設, 見檔頭)
        p('rate_limit', 0.0)
        p('command_rpm_step', 1.0)        # 指令量化成幾 rpm 的整數倍; 0 = 不量化
        p('status_period', 10.0)

        g = self.get_parameter
        self.mode = str(g('mode').value)
        if self.mode not in MODES:
            raise ValueError(f'mode 要是 {MODES} 其中之一, 收到 {self.mode!r}')
        self.dt = 1.0 / float(g('rate').value)
        self.wmax = float(g('max_wheel_speed').value)
        self.rate_limit = float(g('rate_limit').value)
        self.rpm_step = float(g('command_rpm_step').value)

        seed = int(g('seed').value)
        rng = random.Random(seed)
        self.gen = (_OU(rng, float(g('ou_smooth_tau').value)) if self.mode == 'ou'
                    else _HoldRamp(rng))

        self.cmd_pub = self.create_publisher(JointState, g('joint_command_topic').value, 10)
        self.scenario_pub = self.create_publisher(String, 'test_scenario', 10)
        self.phase_pub = self.create_publisher(String, 'test_phase', 10)

        self.phase = 'idle'               # idle -> measure, 跑到 Ctrl-C
        self.phase_start = None
        self.last_now = None
        self.ticks = 0
        self.current_cmd = [0.0] * 4

        self.create_timer(self.dt, self.control_callback)
        self.create_timer(float(g('status_period').value), self.status)
        self.get_logger().info(
            f'激發駕駛: mode={self.mode} seed={seed}, {1.0 / self.dt:.0f} Hz, '
            f'輪速上限 {self.wmax} rad/s, 跑到 Ctrl-C (沒有撞牆保護)')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def measure_time(self):
        return self.ticks * self.dt

    def _mix(self, throttle, steer):
        """[FL, FR, RL, RR] (rad/s)。超過上限四輪等比縮放, 保住 throttle:steer 的比例。"""
        left, right = throttle - steer, throttle + steer
        peak = max(abs(left), abs(right))
        if peak > self.wmax:
            k = self.wmax / peak
            left, right = left * k, right * k
        return [left, right, left, right]

    def control_callback(self):
        now = self._now()
        if now <= 0.0:
            return                                   # 模擬時鐘還沒來
        if self.last_now is not None and now < self.last_now - 0.5:
            # Isaac 按了 Stop / Play: 時間倒退, 先停一下再接著跑 (指令序列接續, 不重來)
            self.get_logger().warn('模擬時間倒退 (Isaac 重新 Play?), 停 1 秒再接著跑')
            self.phase, self.phase_start = 'idle', None
        self.last_now = now
        if self.phase_start is None:
            self.phase_start = now
        elapsed = now - self.phase_start

        if self.phase == 'idle':
            self._send('idle', 'Start', [0.0] * 4)
            if elapsed > 1.0:
                self.phase, self.phase_start = 'measure', now
            return

        thr, st = self.gen.step(self.dt)
        self._send('measure', self.mode, self._mix(thr * self.wmax, st * self.wmax),
                   rate_limited=self.rate_limit > 0.0)
        self.ticks += 1

    def _send(self, phase, name, target, rate_limited=False):
        if rate_limited:
            self.current_cmd = [_rate_limit(t, c, self.rate_limit, self.dt)
                                for t, c in zip(target, self.current_cmd)]
        else:
            self.current_cmd = list(target)
        self._send_wheels(self.current_cmd)
        self.scenario_pub.publish(String(data=name))
        self.phase_pub.publish(String(data=phase))

    def _send_wheels(self, speeds):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.velocity = [snap_rpm(v, self.rpm_step) for v in speeds]
        self.cmd_pub.publish(msg)

    def stop_car(self):
        """結束前補送幾次 0 (Isaac 會一直維持最後一筆指令)。"""
        self.current_cmd = [0.0] * 4
        try:
            for _ in range(5):
                self._send_wheels(self.current_cmd)
        except Exception:            # context 已經關掉了就算了
            pass

    def status(self):
        if self.phase_start is None:
            return
        c = self.current_cmd
        self.get_logger().info(
            f'[{self.phase}] {self.mode} {self.measure_time:.0f} s | '
            f'左 {c[0]:+.1f} 右 {c[1]:+.1f} rad/s')


def _raise_interrupt(signum, frame):
    # 只理第一次: launch 收到 Ctrl-C 會再轉發一次 SIGINT, 不要讓它打斷 stop_car()
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt


def main(args=None):
    # Ctrl-C 自己處理: rclpy 預設的 SIGINT handler 會先把 context 關掉, 之後 stop_car()
    # 的 0 就送不出去 (Isaac 會照最後一筆指令一直開)。關掉它, 自己把 SIGINT / SIGTERM
    # 變成 KeyboardInterrupt (明確裝上去: 從背景啟動的行程會繼承「忽略 SIGINT」),
    # context 還活著, 送完 0 再 shutdown。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = CollectCarControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f'[collect_car_control] 結束: 激發 {node.measure_time:.0f} s, 補送輪速 0', flush=True)
        node.stop_car()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
