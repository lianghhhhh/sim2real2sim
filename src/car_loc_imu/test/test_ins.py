#!/usr/bin/env python3
"""離線量純 IMU 的漂移, 以及四道防線各值多少 —— 不需要 ROS, 不需要 Isaac。

    cd src/car_loc_imu && python3 test/test_ins.py

造一條「走走停停」的軌跡 -> 合成帶零偏/雜訊/傾斜的 IMU -> 跑 EKF -> 跟真值比。
README 裡的三張表就是這個腳本印出來的。
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from car_loc_imu.ins import (G, ImuIns, StillDetector,      # noqa: E402
                             TiltTracker, rot2, wrap_pi)

RATE = 60.0
DT = 1.0 / RATE
CYC = 15.0          # 每 15 秒一個循環: 開 11 秒 -> 平滑煞停 -> 停 3 秒
W = 0.5


def smoothstep(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3 - 2 * u)


def gate(t):
    """速度倍率 0..1。平滑起停 —— 不平滑的話合成 IMU 會出現 delta 函數,
    量到的就不是演算法的誤差而是模擬的假象。"""
    ph = np.asarray(t) % CYC
    return np.minimum(smoothstep(ph / 0.8), 1.0 - smoothstep((ph - 11.0) / 0.8))


# s(t) = gate 的積分 (弧長參數)。先在細網格上算好再內插。
_TG = np.arange(0.0, 300.0, 0.001)
_SG = np.concatenate([[0.0], np.cumsum(gate(_TG[:-1]) * 0.001)])


def truth(t):
    s = float(np.interp(t, _TG, _SG))
    g = float(gate(t))
    px, py = 1.6 * math.sin(W * s), 1.1 * math.sin(2 * W * s)
    dpx, dpy = 1.6 * W * math.cos(W * s), 2.2 * W * math.cos(2 * W * s)
    # yaw 是「沿路徑的方向」, 停著的時候也有定義 (速度是 0 但方向還在)
    return px, py, dpx * g, dpy * g, math.atan2(dpy, dpx)


def _d1(f, t, h=2e-3):
    return (f(t + h) - f(t - h)) / (2 * h)


def synth(t, bias_g, bias_a, ng, na, tilt_deg, rng, fwd_deg=0.0):
    """合成 IMU。

    `fwd_deg` 是「車體 +X 量到車頭的角度」—— REP-103 的車是 0, 但 car.usd 的
    base_link 是 +X 朝左、車頭 -Y, 也就是 -90。車體座標整個轉了, 所以車身的
    yaw 是**路徑方向減掉 fwd_deg**, IMU 量到的加速度也跟著轉。
    """
    px, py, vx, vy, yaw = truth(t)
    a = np.array([_d1(lambda u: truth(u)[2], t), _d1(lambda u: truth(u)[3], t)])
    w = wrap_pi(truth(t + 2e-3)[4] - truth(t - 2e-3)[4]) / 4e-3
    yaw = wrap_pi(yaw - math.radians(fwd_deg))     # 車身朝向 (不是路徑方向)
    ab = rot2(-yaw) @ a
    tr = math.radians(tilt_deg)
    # IMU 量到的是比力: 傾斜時重力會漏進水平軸
    acc = np.array([ab[0] + math.sin(tr) * G + bias_a[0], ab[1] + bias_a[1],
                    G * math.cos(tr)]) + rng.normal(0, na, 3)
    gyro = np.array([0.0, 0.0, w + bias_g]) + rng.normal(0, ng, 3)
    return acc, gyro, (px, py, vx, vy, yaw)


def run(*, zupt=True, zaru=True, nhc=True, yaw_imu=True, tilt_deg=0.0,
        gravity='orientation', bias_a=(0.02, -0.015), bias_g=0.004,
        T=60.0, seed=0, fwd_deg=0.0, ins_fwd_deg=None, sigma_acc=None):
    """`fwd_deg` 是車子真正的車頭軸, `ins_fwd_deg` 是告訴濾波器的那個
    (預設兩個一樣)。給不同的值就是在量「NHC 軸設錯要付多少代價」。

    `sigma_acc` 沒給的話**依重力模式選**, 不是用同一個值:

      orientation   : 0.03  —— 姿態是精確的, 加速度的殘差就只剩感測器雜訊
                      (Isaac 實測 |w|<0.2 時 0.064 m/s^2 -> 連續時間 0.008)
      complementary : 0.35  —— 傾角是**估**出來的, 那個估計誤差本身就是一個
                      持續存在的假加速度 (傾斜 1 度 = 0.17 m/s^2)。過程雜訊
                      要蓋得住它, 蓋不住的話 NHC 會一直被卡方閘門擋掉。
                      實測差距: 0.03 -> 6.20 m, 0.35 -> 2.54 m。
    """
    if sigma_acc is None:
        sigma_acc = 0.35 if gravity == 'complementary' else 0.03
    rng = np.random.default_rng(seed)
    ins = ImuIns(sigma_acc=sigma_acc,
                 forward_deg=fwd_deg if ins_fwd_deg is None else ins_fwd_deg)
    det = StillDetector(gyro_th=0.05, acc_th=0.35, var_th=0.08)
    tilt = TiltTracker()
    px, py, _, _, yaw = truth(0.0)
    ins.set_pose(px, py, wrap_pi(yaw - math.radians(fwd_deg)), 0.0)
    errs = []
    for k in range(1, int(T * RATE) + 1):
        t = k * DT
        acc, gyro, gt = synth(t, bias_g, np.array(bias_a), 0.002, 0.02,
                              tilt_deg, rng, fwd_deg)
        det.add(t, gyro, acc)
        still = det.is_still() if (zupt or zaru) else False

        if gravity == 'orientation':
            tr = math.radians(tilt_deg)
            R = np.array([[math.cos(tr), 0, math.sin(tr)], [0, 1, 0],
                          [-math.sin(tr), 0, math.cos(tr)]])
            acc_xy = (R @ acc - np.array([0, 0, G]))[:2]
        else:
            tilt.update(gyro, acc, DT, trust_accel=still)
            acc_xy = tilt.gravity_free(acc)

        ins.predict(t, acc_xy, gyro[2])
        if yaw_imu:
            ins.update_yaw(gt[4], 0.02)
        if still:
            if ins.still_since is None:
                ins.still_since = t
            if zupt:
                ins.zupt()
                ins.zero_accel(acc_xy)
            if zaru:
                ins.zaru(gyro[2])
            if t - ins.still_since >= ins.anchor_after:
                ins.anchor_position()
        else:
            ins.still_since = None
            ins.anchor = None
        if nhc:
            ins.nhc()
        errs.append(math.hypot(ins.x[0] - gt[0], ins.x[1] - gt[1]))
    return np.array(errs), ins


def show(label, **kw):
    e, ins = run(**kw)
    print(f'  {label:34s} RMS {np.sqrt((e ** 2).mean()):7.2f}  '
          f'p95 {np.percentile(e, 95):7.2f}  最大 {e.max():8.2f}  '
          f'60 秒時 {e[-1]:8.2f} m   | ZUPT {ins.counts["zupt"]:5d}, '
          f'NHC {ins.counts["nhc"]:5d}')


def main():
    print(f'純 IMU {RATE:.0f} Hz, 60 秒。陀螺零偏 0.004 rad/s, '
          '加速度零偏 (0.02, -0.015) m/s^2')
    print('軌跡: 8 字形, 每 15 秒平滑停 3 秒。表格是位置誤差。\n')

    print('[A] Isaac 情境 —— IMU 給精確姿態 (重力扣得準, yaw 有絕對值), 車身水平')
    show('純積分 (什麼防線都不開)', zupt=0, zaru=0, nhc=0)
    show('+ ZUPT / ZARU', zupt=1, zaru=1, nhc=0)
    show('+ NHC (只有 NHC)', zupt=0, zaru=0, nhc=1)
    show('全開', zupt=1, zaru=1, nhc=1)

    print('\n[B] 真車情境 —— 6 軸 IMU: yaw 只能靠陀螺儀積分, 傾角靠互補濾波')
    print('    (互補濾波模式的 sigma_acc 要給 0.35 —— 傾角估計誤差是額外的假加速度)')
    show('完全不修 (連傾角都不修)', zupt=0, zaru=0, nhc=0, yaw_imu=0,
         gravity='complementary')
    show('+ ZUPT / ZARU', zupt=1, zaru=1, nhc=0, yaw_imu=0, gravity='complementary')
    show('全開', zupt=1, zaru=1, nhc=1, yaw_imu=0, gravity='complementary')

    print('\n[C] 車身固定傾斜 2 度 (重力漏進水平軸 = 0.34 m/s^2 的假加速度)')
    show('全開, 互補濾波估傾角', zupt=1, zaru=1, nhc=1, yaw_imu=0, tilt_deg=2.0,
         gravity='complementary')
    show('全開, IMU 給精確姿態', zupt=1, zaru=1, nhc=1, tilt_deg=2.0)

    print('\n[CC] car.usd 的車頭是 -Y (base_link +X 朝左) —— NHC 軸給錯的代價')
    show('軸給對 (forward_deg=-90)', fwd_deg=-90.0)
    show('軸給錯 (照 REP-103 當 0)', fwd_deg=-90.0, ins_fwd_deg=0.0)
    show('同一條軌跡, 乾脆不開 NHC', fwd_deg=-90.0, ins_fwd_deg=0.0, nhc=0)

    print('\n[D] 靜止 10 秒: ZUPT 開/關的共變異數 (sigma 是濾波器自己說它有多不確定)')
    for use in (False, True):
        ins = ImuIns()
        ins.set_pose(1.0, 2.0, 0.5, 0.0)
        for k in range(1, 601):
            ins.predict(k / 60.0, np.array([0.0, 0.0]), 0.0)
            if use:
                if ins.still_since is None:
                    ins.still_since = k / 60.0
                ins.zupt()
                ins.zaru(0.0)
                ins.zero_accel(np.array([0.0, 0.0]))
                ins.anchor_position()
        print(f"  ZUPT {'開' if use else '關'}: sigma {ins.sigma_pos() * 100:8.2f} cm")


if __name__ == '__main__':
    main()
