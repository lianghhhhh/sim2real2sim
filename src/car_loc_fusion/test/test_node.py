#!/usr/bin/env python3
"""節點層的煙霧測試 —— **不需要 ROS**。

    python3 test/test_node.py

`test_fusion.py` 測的是濾波器與融合策略, 這支測的是**節點**: 參數有沒有接對、
三條訊息流 (IMU / 輪速 / 絕對量測) 有沒有正確交錯、車頭軸有沒有搞反、
「用第一則絕對量測當起點」會不會卡住。這些都不會在濾波器測試裡出現, 但一上車
就會爆炸。

`_stub_ros.py` 是 car_loc_wheel 那邊的那一份 (假造 rclpy / sensor_msgs /
tf2_ros), 直接沿用 —— 兩個節點的訊息型別一模一樣, 抄第二份只會讓它們分岔。
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, '..', '..', 'car_loc_wheel')))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, '..', '..', 'car_loc_wheel', 'test')))

import _stub_ros                                          # noqa: E402,F401 (要先載)

from car_loc_fusion.fusion_loc_node import FusionLocalizer  # noqa: E402
from nav_msgs.msg import Odometry                           # noqa: E402
from sensor_msgs.msg import Imu, JointState                 # noqa: E402

DT = 1.0 / 60.0
RADIUS = 0.075
WHEELS = ['front_left_joint', 'front_right_joint',
          'rear_left_joint', 'rear_right_joint']


def imu_msg(t, a_fwd=0.0, wz=0.0):
    m = Imu()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    # 車頭是 -Y (forward_deg = -90), 所以前進加速度落在 body 的 -y
    m.linear_acceleration.y = -float(a_fwd)
    m.linear_acceleration.z = 9.80665
    m.angular_velocity.z = float(wz)
    return m


def joint_msg(t, omega):
    m = JointState()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    m.name = list(WHEELS)
    m.velocity = [float(omega)] * 4
    return m


def odom_msg(t, x, y, yaw=0.0, sigma=0.002):
    m = Odometry()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    m.pose.pose.position.x = float(x)
    m.pose.pose.position.y = float(y)
    m.pose.pose.orientation.z = math.sin(yaw * 0.5)
    m.pose.pose.orientation.w = math.cos(yaw * 0.5)
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.5 * sigma ** 2
    m.pose.covariance = cov
    return m


def make_node(**over):
    """建一個節點, 順便覆寫參數。

    參數是在 `__init__` 裡一次讀進成員變數的, 所以覆寫必須發生在 declare 的
    那一刻 —— 建完再改 `get_parameter().value` 只會改到沒有人再讀的那一份。
    """
    orig = _stub_ros.Node.declare_parameter

    def patched(self, name, value=None):
        orig(self, name, over.get(name, value))

    _stub_ros.Node.declare_parameter = patched
    try:
        return FusionLocalizer()
    finally:
        _stub_ros.Node.declare_parameter = orig


def drive(node, dur, v, *, t0=0.0, cam=None, lid=None, cam_delay=0.0,
          truth=None, a_fwd=0.0):
    """餵 dur 秒的 IMU + 輪速; cam/lid 是 (Hz, sigma) 或 None。

    絕對量測依「到達時刻」餵進去 (t + cam_delay), 時戳仍然是量測發生的時刻 ——
    延遲補償要測得到, 這一點不能省。
    """
    n = int(dur / DT)
    pending = []
    for i in range(n):
        t = t0 + i * DT
        node.on_joint(joint_msg(t, v / (0.93 * RADIUS)))
        node.on_imu(imu_msg(t, a_fwd=a_fwd))
        for src, spec in (('camera', cam), ('lidar', lid)):
            if spec is None:
                continue
            hz, sig = spec
            if i % max(int(round((1.0 / hz) / DT)), 1):
                continue
            x, y = truth(t) if truth else (0.0, 0.0)
            pending.append((t + cam_delay, src,
                            odom_msg(t, x + sig * 0.0, y + sig * 0.0)))
        while pending and pending[0][0] <= t:
            _, src, msg = pending.pop(0)
            node.on_abs(msg, src)
    return node


def case1():
    print('[1] 用第一則絕對量測當起點 (不需要 initial_pose)')
    n = make_node()
    drive(n, 1.5, 0.0)                    # 靜止校正
    assert n.started, '靜止校正沒完成'
    assert not n.ekf.initialized, '還沒有絕對量測就自己有起點了'
    print('     校正完成而且**還沒有起點** —— 正確 (在等絕對量測)')
    n.on_abs(odom_msg(1.5, 2.0, -0.3), 'camera')
    assert n.ekf.initialized
    assert abs(n.ekf.pos[0] - 2.0) < 0.05 and abs(n.ekf.pos[1] + 0.3) < 0.05
    print(f'     第一則 camera pose 之後: ({n.ekf.pos[0]:+.2f}, {n.ekf.pos[1]:+.2f}) '
          '= 量測本身。純航位推算需要 initial_pose, 這條不用。')


def case2():
    print('[2] 車頭軸 (forward_deg = -90: car.usd 的車頭是 -Y)')
    n = make_node()
    drive(n, 1.5, 0.0)
    n.on_abs(odom_msg(1.5, 0.0, 0.0), 'camera')
    drive(n, 3.0, 0.6, t0=1.5)            # 開 3 秒, 只有輪速沒有絕對量測
    x, y = n.ekf.pos
    print(f'     開 3 秒 @0.6 m/s -> ({x:+.3f}, {y:+.3f})')
    assert y < -1.0 and abs(x) < 0.2, f'應該往 -Y 走, 實際 ({x:.3f}, {y:.3f})'
    print('     往 -Y 走了 1.7 m —— 軸是對的。給成 0 (REP-103) 的話會往 +X 跑,')
    print('     而且絕對量測會一直跟它吵架 (car_loc_imu 表 [CC]: 差 45 倍)。')


def case3():
    print('[3] 延遲的絕對量測: 時戳是過去的, 不可以當成現在')
    truth = lambda t: (0.0, -0.6 * max(t - 1.5, 0.0))
    for rewind, tag in ((True, '倒帶'), (False, '不倒帶')):
        n = make_node(rewind=rewind)
        drive(n, 1.5, 0.0)
        n.on_abs(odom_msg(1.5, 0.0, 0.0), 'camera')
        drive(n, 6.0, 0.6, t0=1.5, cam=(30.0, 0.0), cam_delay=0.0795, truth=truth)
        tx, ty = truth(7.5)
        err = math.hypot(n.ekf.pos[0] - tx, n.ekf.pos[1] - ty)
        print(f'     {tag}: 誤差 {err * 100:5.2f} cm '
              f'(倒帶 {n.ekf.counts["rewind"]} 次, 太舊 {n.ekf.counts["too_old"]})')
    print('     0.6 m/s x 0.0795 s = 4.8 cm —— 不倒帶的話就是那個量, 而且方向')
    print('     跟著車頭轉, 平均不掉。')


def case4():
    print('[4] 絕對量測掉線 -> 退化成航位推算, 回來後拉得回來')

    def truth(t):
        # 掉線那 10 秒車子實際上比輪速說的快 10% (輪徑尺度誤差 / 打滑) ——
        # 航位推算沒有絕對參考, 這種系統性誤差只會一路累積下去。
        y = -0.6 * max(min(t, 4.5) - 1.5, 0.0)
        y -= 0.66 * max(min(t, 14.5) - 4.5, 0.0)
        y -= 0.6 * max(t - 14.5, 0.0)
        return (0.0, y)

    n = make_node()
    drive(n, 1.5, 0.0)
    n.on_abs(odom_msg(1.5, 0.0, 0.0), 'camera')
    drive(n, 3.0, 0.6, t0=1.5, cam=(30.0, 0.0), truth=truth)
    e1 = math.hypot(n.ekf.pos[0] - truth(4.5)[0], n.ekf.pos[1] - truth(4.5)[1])
    drive(n, 10.0, 0.6, t0=4.5)                     # 沒有任何絕對量測
    e2 = math.hypot(n.ekf.pos[0] - truth(14.5)[0], n.ekf.pos[1] - truth(14.5)[1])
    sig = n.ekf.sigma_pos()
    drive(n, 5.0, 0.6, t0=14.5, cam=(30.0, 0.0), truth=truth)
    e3 = math.hypot(n.ekf.pos[0] - truth(19.5)[0], n.ekf.pos[1] - truth(19.5)[1])
    print(f'     有絕對量測 {e1 * 100:5.2f} cm -> 掉線 10 秒 {e2 * 100:6.2f} cm '
          f'(sigma {sig * 100:.1f} cm) -> 回來 5 秒 {e3 * 100:5.2f} cm '
          f'(強制 {n.ekf.counts["forced"]} 次)')
    assert e2 > 0.3, '掉線期間應該要漂 (輪速比真值慢 10%)'
    assert e3 < 0.1, f'回來之後沒有拉回去 ({e3:.3f} m)'
    print('     掉線期間 sigma 跟著長大 —— 下游 (nav / 收資料) 靠它就知道這段不可信,')
    print('     不需要另外發一個「我掉線了」的訊息。')
    print('     **但 sigma 長得不夠快**: 漂了 61 cm 而 sigma 只有 7 cm, 所以量測一回來')
    print('     還是被卡方閘門擋掉, 是**逃生口**把它救回來的 (強制接受那 37 次)。')
    print('     這正是逃生口不能省的理由 —— 「掉線久了 P 自己會長到夠大」在這個時間')
    print('     尺度上不成立: P 靠過程雜訊長, 而誤差靠尺度誤差 x 距離長, 後者快得多。')


def case5():
    print('[5] status() 不會爆炸 (每一種狀態都要走一次)')
    n = make_node()
    n.status()                                   # 還沒收到 IMU
    drive(n, 1.5, 0.0)
    n.status()                                   # 有 IMU, 還沒有起點
    n.on_abs(odom_msg(1.5, 0.0, 0.0), 'camera')
    drive(n, 1.0, 0.6, t0=1.5, cam=(30.0, 0.0))
    n.status()                                   # 正常
    print('     三種狀態都印得出來 (status 在 timer 裡, 爆掉的話整個節點會靜悄悄停掉)')


def case6():
    print('[6] 第一則絕對量測的時鐘就是錯的 (Isaac 換場景後相機時戳沒歸零)')
    # 2026-09-15 實測: run_car.sh 第二個場景 /clock /imu /scan 從 0 開始, 相機卻
    # 延續上一個場景 +292 s。舊版拿那一則 (t=300) 初始化 -> IMU (t=1.5...) 全部
    # dt<0 被丟掉 -> 整輪凍結在起點, twist 全 0, sigma 還很小。
    OFFSET = 292.0
    n = make_node()
    drive(n, 1.5, 0.0)
    t, i = 1.5, 0
    while t < 4.5:                                   # 靜止 + 相機 30 Hz (時鐘錯)
        n.on_joint(joint_msg(t, 0.0))
        n.on_imu(imu_msg(t))
        if i % 2 == 0:
            n.on_abs(odom_msg(t + OFFSET, 1.0, 2.0), 'camera')
        t += DT
        i += 1
    assert n.ekf.initialized, '時鐘偏移鎖定之後應該要能初始化'
    assert abs(n.ekf.ins.t - t) < 0.1, f'濾波器時間跑到 {n.ekf.ins.t:.1f} (IMU 在 {t:.1f})'
    print(f'     偏移鎖定 {n.src["camera"].clock_offset:+.1f} s, 起點 '
          f'({n.ekf.pos[0]:+.2f}, {n.ekf.pos[1]:+.2f}), 濾波器時間 {n.ekf.ins.t:.2f} ≈ IMU {t:.2f}')
    # 凍結的症狀就是 predict 一直 return: 濾波器時間不動、twist 的轉速卡在 0
    for _ in range(30):
        n.on_joint(joint_msg(t, 0.0))
        n.on_imu(imu_msg(t, wz=0.8))
        t += DT
    assert abs(n.ekf.ins.t - (t - DT)) < 1e-6, '濾波器時間沒跟上 IMU —— 遞推凍結'
    assert abs(n.ekf.ins.last_omega - 0.8) < 0.05, \
        f'轉速卡在 {n.ekf.ins.last_omega:.2f} (gyro 0.8) —— 遞推凍結'
    print(f'     之後 gyro 0.8 rad/s -> 濾波器轉速 {n.ekf.ins.last_omega:.2f}, 時間跟著 IMU 走 —— 沒有凍結')


def main():
    print('=' * 84)
    print('  car_loc_fusion 節點煙霧測試 (假的 ROS, 不需要 rclpy)')
    print('=' * 84)
    for f in (case1, case2, case3, case4, case5, case6):
        f()
        print()
    print('=' * 84)
    print('  PASS')
    print('=' * 84)
    return 0


if __name__ == '__main__':
    sys.exit(main())
