#!/usr/bin/env python3
"""節點層的煙霧測試 —— **不需要 ROS**。

    python3 test/test_node.py

`test_wheel_ins.py` 測的是濾波器, 這支測的是**節點**: 參數有沒有接對、兩條
訊息流有沒有正確交錯、車頭軸有沒有搞反、開機校正會不會卡住。這些都是不會在
濾波器單元測試裡出現、但一上車就爆炸的東西。

做法是用 `_stub_ros.py` 假造 `rclpy` / `sensor_msgs` / `tf2_ros`, 直接把
`WheelLocalizer` 生出來然後手動呼叫 `on_imu` / `on_joint`。

**這支抓到過的真問題:**

* 卡方閘門會把輪速**永久**擋在外面。ZUPT 之後 `P[v]` 被壓到 1e-4, 只靠過程雜訊
  慢慢長回來; 如果濾波器的速度信念跟輪速差 0.5 m/s, 要等 **14 秒**才放行 ——
  那段時間車子在估計裡停在原地。修法見 `wheel_ins.update_wheel` 的
  `wheel_reject_time`。案例 [2] 就是那個情境。
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..'))

import _stub_ros                                     # noqa: E402,F401  (要先載)

from car_loc_wheel.wheel_loc_node import WheelLocalizer   # noqa: E402
from sensor_msgs.msg import Imu, JointState               # noqa: E402

DT = 1.0 / 60.0
RADIUS = 0.075
# **輪速用節點自己設定的 scale 反推, 不要寫死一個數字。** 這支測的是「節點有沒有
# 把參數接對、有沒有照自己的模型積分」, 不是「0.93 這個值對不對」(那是
# replay_csv.py / replay_bag.py 拿真資料在量的)。寫死的話預設一改測試就假 fail。


def imu_msg(t, a_fwd=0.0, wz=0.0):
    m = Imu()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    # 車頭是 -Y (forward_deg = -90), 所以前進加速度落在 body 的 -y
    m.linear_acceleration.x = 0.0
    m.linear_acceleration.y = -float(a_fwd)
    m.linear_acceleration.z = 9.80665
    m.angular_velocity.z = float(wz)
    return m


def joint_msg(t, omega):
    m = JointState()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    m.name = ['front_left_joint', 'front_right_joint',
              'rear_left_joint', 'rear_right_joint']
    m.velocity = [float(w) for w in np.atleast_1d(omega) * np.ones(4)]
    return m


def drive(node, v, a=None, wz=None, omega=None):
    """把一段速度剖面餵進節點。回傳 (真值 x, 真值 y)。"""
    n = len(v)
    t = np.arange(n) * DT
    a = np.gradient(v) / DT if a is None else a
    wz = np.zeros(n) if wz is None else wz
    omega = v / (node.ins.scale * RADIUS) if omega is None else omega
    yaw = np.cumsum(wz) * DT
    heading = yaw - math.pi / 2                       # forward_deg = -90
    gx = float(np.sum(v * np.cos(heading)) * DT)
    gy = float(np.sum(v * np.sin(heading)) * DT)
    for i in range(n):
        node.on_joint(joint_msg(t[i], omega[i]))
        node.on_imu(imu_msg(t[i], a[i], wz[i]))
    return gx, gy


def profile(dur, hold=2.0, speed=0.5, ramp=0.3):
    """前 `hold` 秒靜止 (開機校正用), 之後平滑加速到 `speed`。"""
    t = np.arange(int(dur / DT)) * DT
    return np.where(t < hold, 0.0,
                    speed * 0.5 * (1 + np.tanh((t - hold - ramp * 1.5) / ramp)))


def case(name, fn):
    print(f'\n--- {name} ---')
    ok, msg = fn()
    print(('  PASS  ' if ok else '  FAIL  ') + msg)
    return ok


# ----------------------------------------------------------------------------
def t1_straight():
    """[1] 直線: 位置要往 -Y 走 (車頭軸), 誤差要在幾公分內。"""
    n = WheelLocalizer()
    v = profile(10.0)
    gx, gy = drive(n, v)
    ex, ey = n.ins.x[0], n.ins.x[1]
    err = math.hypot(ex - gx, ey - gy)
    c = n.ins.counts
    print(f'  估計 ({ex:+.3f}, {ey:+.3f})  真值 ({gx:+.3f}, {gy:+.3f})  '
          f'誤差 {err:.3f} m  |  輪速 {c["wheel"]} 擋掉 {c["rejected"]} '
          f'強制 {c["forced"]}')
    if abs(gy) < 1.0 or abs(gx) > 0.01:
        return False, '真值本身就不對 —— 車頭應該是 -Y'
    return err < 0.1 and c['rejected'] == 0, f'誤差 {err:.3f} m, 沒有輪速被擋掉'


def t2_accel_dead():
    """[2] 加速度計整個壞掉 (永遠讀 0) —— 輪速不能被卡方閘門永久鎖死。

    這是這支測試存在的理由。速度的不確定度在 ZUPT 之後被壓到 1e-4, 之後只靠
    過程雜訊長回來; 沒有逃生口的話 0.5 m/s 的落差要等 14 秒才放行。
    """
    n = WheelLocalizer()
    # 用**階躍**的速度剖面: 這是最惡劣的情況 —— 輪速一瞬間從 0 跳到 0.6 m/s,
    # 而加速度計什麼都沒說, 所以 innovation 一開始就遠超過閘門。
    t = np.arange(int(12.0 / DT)) * DT
    v = np.where(t < 2.0, 0.0, 0.6)
    gx, gy = drive(n, v, a=np.zeros(len(v)))          # 加速度計讀 0
    err = math.hypot(n.ins.x[0] - gx, n.ins.x[1] - gy)
    c = n.ins.counts
    print(f'  估計 ({n.ins.x[0]:+.3f}, {n.ins.x[1]:+.3f})  真值 ({gx:+.3f}, {gy:+.3f})  '
          f'誤差 {err:.3f} m  |  擋掉 {c["rejected"]} 強制 {c["forced"]} '
          f'速度 {n.ins.speed:.3f} (真 {v[-1]:.3f})')
    # 對照組: 關掉逃生口
    n2 = WheelLocalizer()
    n2.ins.wheel_reject_time = 0.0
    drive(n2, v, a=np.zeros(len(v)))
    c2 = n2.ins.counts
    print(f'  對照 (關掉逃生口): 速度 {n2.ins.speed:.3f}, 輪速被擋掉 '
          f'{c2["rejected"]} 筆 (全部), 車子在估計裡完全沒有動過')
    print('  (加速度計沒訊號 -> 起步那段只能靠輪速把速度拉起來, 誤差主要是那段的')
    print('   落後量。重點是**有沒有追上**, 不是誤差多小。)')
    return (abs(n.ins.speed - v[-1]) < 0.05 and abs(n2.ins.speed) < 0.05), \
        (f'有逃生口速度追上了 ({n.ins.speed:.3f}), '
         f'沒有的話永遠追不上 ({n2.ins.speed:.3f})')


def t3_slip():
    """[3] 一顆輪子空轉 3 秒 —— 位置不可以被拖走。"""
    n = WheelLocalizer()
    v = profile(12.0)
    om = v / (n.ins.scale * RADIUS)
    W = np.tile(om, (4, 1))
    t = np.arange(len(v)) * DT
    m = (t >= 5.0) & (t < 8.0)
    W[0, m] *= 6.0                                    # 左前輪空轉
    gx, gy = drive(n, v, omega=W.T)
    err = math.hypot(n.ins.x[0] - gx, n.ins.x[1] - gy)
    c = n.ins.counts
    print(f'  誤差 {err:.3f} m  |  打滑放寬 {c["slip"]} 擋掉 {c["rejected"]} '
          f'強制 {c["forced"]}')
    return err < 0.3 and c['slip'] > 0, \
        f'誤差 {err:.3f} m, 打滑偵測觸發 {c["slip"]} 次'


def t4_wheel_names():
    """[4] JointState 的名字對不上 -> 要警告, 而且要還能跑。"""
    n = WheelLocalizer()
    t = 0.0
    for i in range(120):
        t = i * DT
        m = JointState()
        m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
        m.name = ['wheel_a', 'wheel_b', 'wheel_c', 'wheel_d']
        m.velocity = [0.0] * 4
        n.on_joint(m)
        n.on_imu(imu_msg(t))
    return n.name_warned and n.started, '有警告, 而且仍然完成了開機校正'


def t5_no_wheels():
    """[5] /joint_states 完全沒來 -> 要**大聲**降級成純 IMU, 不可以默默跑。

    開機校正等的是「輪速說靜止」, 所以沒有輪速就永遠等不到。逾時之後要照樣
    啟動 (零偏當 0) 並且說清楚為什麼 —— 卡在那裡不動比降級更糟, 因為使用者
    看不出是哪裡壞了。
    """
    n = WheelLocalizer()
    for i in range(int(12.0 / DT)):
        n.on_imu(imu_msg(i * DT))
    return n.started and n.n_joint == 0 and n.wheel_warned, \
        '逾時之後降級啟動, 而且警告了「沒有 /joint_states」與「誤差會以 t^2 長大」'


def main():
    ok = all([
        case('直線行駛', t1_straight),
        case('加速度計壞掉 (輪速不能被永久鎖死)', t2_accel_dead),
        case('單輪空轉 3 秒', t3_slip),
        case('JointState 名字對不上', t4_wheel_names),
        case('完全收不到 /joint_states', t5_no_wheels),
    ])
    print('\n' + ('全部通過' if ok else '有失敗'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
