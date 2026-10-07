#!/usr/bin/env python3
"""把錄下來的 rosbag2 (sqlite3) 裡的 /imu 跟 /odom 讀出來, 離線重跑濾波器。

    python3 test/replay_bag.py <bag 目錄> [--sigma-acc 0.05] [--no-nhc] ...

**不需要 ROS**, 只用 sqlite3 + 手刻 CDR 解析。為什麼要有這個東西: 參數要怎麼
調不能用猜的, 也不該每次都重開 Isaac 跑一輪 —— 錄一次 bag, 之後所有 A/B 都在
這裡兩秒跑完, 而且每次餵的是**完全一樣**的資料, 差異才真的是參數造成的。

`test_ins.py` 是合成資料 (乾淨、假設都成立), 這支是**真的 Isaac 資料**
(有側滑、有離散化、有真實的雜訊)。兩支要一起看。
"""
from __future__ import annotations

import argparse
import math
import os
import sqlite3
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from car_loc_imu.imu_io import Cdr, parse_imu               # noqa: E402
from car_loc_imu.ins import (G, ImuIns, SpinDetector,      # noqa: E402
                             StillConfirm, StillDetector, TiltTracker,
                             quat_to_matrix, quat_to_yaw)


def parse_odom(buf):
    c = Cdr(buf)
    t = c.header()
    c.string()                           # child_frame_id
    p = c.f64a(3)
    q = c.f64a(4)
    c.f64a(36)
    v = c.f64a(3)
    return t, np.array(p[:2]), quat_to_yaw(q), np.array(v[:2])


def read_bag(path):
    db = [f for f in os.listdir(path) if f.endswith('.db3')]
    if not db:
        raise SystemExit(f'{path} 裡沒有 .db3')
    con = sqlite3.connect(os.path.join(path, db[0]))
    ids = dict(con.execute('SELECT name, id FROM topics'))
    for want in ('/imu', '/odom'):
        if want not in ids:
            raise SystemExit(f'bag 裡沒有 {want} (有的是 {sorted(ids)})')
    imu, odom = [], []
    for tid, data in con.execute(
            'SELECT topic_id, data FROM messages ORDER BY timestamp'):
        if tid == ids['/imu']:
            imu.append(parse_imu(data))
        elif tid == ids['/odom']:
            odom.append(parse_odom(data))
    con.close()
    return imu, odom


# ---------------------------------------------------------------- 重跑
def replay(imu, odom, *, gravity='orientation', yaw_imu=True, zupt=True,
           zaru=True, nhc=True, anchor=True, spin=False,
           still=(0.03, 0.25, 0.05, 0.30), confirm=None, calib_time=1.5,
           **ins_kw):
    """`confirm`: StillConfirm 的參數 (dict); None = 預設值, {} 以外想整個關掉
    就給 dict(acc_mean=0, gyro_mean=0, speed_gate=0)。"""
    ins = ImuIns(**ins_kw)
    det = StillDetector(gyro_th=still[0], acc_th=still[1], var_th=still[2],
                        window=still[3])
    spin_det = SpinDetector()
    conf = StillConfirm(window=still[3], **(confirm or {}))
    tilt = TiltTracker()

    ot = np.array([o[0] for o in odom])
    op = np.array([o[1] for o in odom])
    ov = np.array([o[3] for o in odom])
    order = np.argsort(ot)
    ot, op, ov = ot[order], op[order], ov[order]

    started = False
    cbuf, cstart = [], None
    last_t = None
    rows = []
    for t, q, w, a in imu:
        det.add(t, w, a)
        st = det.is_still()
        dt = None if last_t is None else t - last_t
        last_t = t

        if not started:                              # 開機靜止校正
            if st:
                if cstart is None:
                    cstart = t
                cbuf.append((w, a))
            else:
                cstart, cbuf = None, []
            if cstart is not None and t - cstart >= calib_time and len(cbuf) >= 20:
                gm = np.mean([b[0] for b in cbuf], axis=0)
                am = np.mean([b[1] for b in cbuf], axis=0)
                if gravity == 'complementary':
                    tilt.roll = math.atan2(am[1], am[2])
                    tilt.pitch = math.atan2(-am[0], math.hypot(am[1], am[2]))
                    tilt.inited = True
                # 起點與起始 yaw 直接用 ground truth 的第一筆 —— 純 IMU 本來就
                # 要有人告訴它起點, 這裡不是作弊, 是把「對起點」這件事排除掉,
                # 才量得到演算法本身的誤差。
                k = int(np.searchsorted(ot, t))
                k = min(k, len(ot) - 1)
                ins.set_bias0(float(gm[2]),
                              _grav_free(gravity, q, gm, am, None, True, tilt))
                ins.set_pose(op[k, 0], op[k, 1],
                             quat_to_yaw(q) if yaw_imu else 0.0, t)
                started = True
            continue

        acc_xy = _grav_free(gravity, q, w, a, dt, st, tilt)
        st = conf.update(t, dt, st, acc_xy, float(w[2]), ins)
        ins.predict(t, acc_xy, float(w[2]))
        if yaw_imu:
            ins.update_yaw(quat_to_yaw(q), 0.02)
        if st and (zupt or zaru):
            if ins.still_since is None:
                ins.still_since = t
            if zupt:
                ins.zupt()
                ins.zero_accel(acc_xy)
            if zaru:
                ins.zaru(float(w[2]))
            if anchor and t - ins.still_since >= ins.anchor_after:
                ins.anchor_position()
        else:
            ins.still_since = None
            ins.anchor = None
        spin_det.add(t, float(w[2]), acc_xy)
        if spin and not st and spin_det.is_spinning_in_place():
            ins.spin_zupt()
        if nhc:
            ins.nhc()
        rows.append((t, ins.x[0], ins.x[1], ins.sigma_pos(), st,
                     ins.gyro_bias, *ins.acc_bias))

    if not rows:
        raise SystemExit('濾波器一次都沒跑起來 —— 開機靜止校正沒完成。'
                         '通常是 still_* 門檻太緊, 或 bag 開頭車子就在動。')
    r = np.array([row[:4] for row in rows])
    stills = np.array([row[4] for row in rows])
    bias = np.array([row[5:] for row in rows])
    gx = np.interp(r[:, 0], ot, op[:, 0])
    gy = np.interp(r[:, 0], ot, op[:, 1])
    gv = np.hypot(np.interp(r[:, 0], ot, ov[:, 0]), np.interp(r[:, 0], ot, ov[:, 1]))
    err = np.hypot(r[:, 1] - gx, r[:, 2] - gy)
    jump = np.hypot(np.diff(r[:, 1]), np.diff(r[:, 2]))
    return dict(t=r[:, 0], err=err, sigma=r[:, 3], jump=jump, still=stills,
                gt_speed=gv, ins=ins, x=r[:, 1], y=r[:, 2], gt_x=gx, gt_y=gy,
                gyro_bias=bias[:, 0], acc_bias=bias[:, 1:])


def _grav_free(mode, q, gyro, acc, dt, still, tilt):
    if mode == 'orientation':
        R = quat_to_matrix(q)
        aw = R @ acc - np.array([0.0, 0.0, G])
        yaw = quat_to_yaw(q)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * aw[0] + s * aw[1], -s * aw[0] + c * aw[1]])
    if mode == 'complementary':
        if dt:
            tilt.update(gyro, acc, dt, trust_accel=still)
        return tilt.gravity_free(acc)
    return np.asarray(acc)[:2]


def measure(imu, odom):
    """從 bag 量出該用的參數。**每一個門檻都該是量出來的, 不是猜的。**

    量四件事:
      1. 車頭是哪一軸 (forward_deg)  —— NHC 要約束哪一軸
      2. 靜止時的雜訊底線           —— still_gyro / still_var / 零偏
      3. 加速度過程雜訊 vs 角速度    —— sigma_acc / sigma_acc_omega
      4. IMU 的槓桿臂               —— 離旋轉中心多遠 (向心加速度會被誤當成車子的加速度)
    """
    t = np.array([m[0] for m in imu])
    q = np.array([m[1] for m in imu])
    w = np.array([m[2] for m in imu])
    a = np.array([m[3] for m in imu])
    ot = np.array([o[0] for o in odom])
    op = np.array([o[1] for o in odom])
    oy = np.unwrap(np.array([o[2] for o in odom]))
    ov = np.array([o[3] for o in odom])
    rate = len(t) / max(t[-1] - t[0], 1e-9)

    # --- 1. 車頭軸 -----------------------------------------------------------
    vx, vy = np.gradient(op[:, 0], ot), np.gradient(op[:, 1], ot)
    sp = np.hypot(vx, vy)
    # 只在「真的在走」而且「不是在原地打轉」的時候, 航向才代表車頭方向。
    wzi = np.abs(np.interp(ot, t, w[:, 2]))
    m = (sp > 0.3) & (wzi < 1.0)
    dl = np.arctan2(np.sin(np.arctan2(vy[m], vx[m]) - oy[m]),
                    np.cos(np.arctan2(vy[m], vx[m]) - oy[m]))
    # 倒車時航向整個差 180 度, 但**車頭軸是同一條**。這是「軸向資料」,
    # 不能直接取平均或中位數 (前進與倒車的樣本會互相抵消到中間去)。
    # 標準做法: 把角度加倍再平均, 最後除以 2 —— 加倍之後 0 度與 180 度重合。
    z = np.exp(2j * dl).mean()
    fwd = math.degrees(np.angle(z)) / 2.0
    spread = math.degrees(math.sqrt(max(-2.0 * math.log(abs(z)), 0.0))) / 2.0
    print('1) 車頭軸  forward_deg = %+.1f 或 %+.1f (差 180 度對 NHC 等價)'
          % (fwd, fwd + 180 if fwd < 0 else fwd - 180))
    print('   離散程度 %.1f°, n=%d  (只取 |v|>0.3 且 |wz|<1 的樣本)' % (spread, m.sum()))
    print('   REP-103 的車是 0; car.usd 是 -90 (+X 朝左、車頭 -Y)')

    # --- 2. 靜止時的雜訊底線 --------------------------------------------------
    gs = np.interp(t, ot, sp)
    rest = (gs < 0.01) & (np.abs(w[:, 2]) < 0.02)
    if rest.sum() < 50:
        print('\n2) 靜止樣本太少 (%d), 跳過雜訊量測' % rest.sum())
    else:
        gn = np.linalg.norm(w[rest], axis=1)
        R = np.array([quat_to_matrix(x) for x in q[rest]])
        aw = np.einsum('nij,nj->ni', R, a[rest]) - np.array([0.0, 0.0, G])
        sd = a[rest].std(axis=0)
        print('\n2) 靜止 %d 筆 (%.0f%%)。門檻要**壓過**這些數字, 不然 ZUPT 不會觸發:'
              % (rest.sum(), 100 * rest.mean()))
        print('   |gyro| 最大 %.4f rad/s          -> still_gyro >= %.3f'
              % (gn.max(), max(0.01, 1.5 * gn.max())))
        print('   accel 三軸標準差和 %.4f m/s^2    -> still_var  >= %.3f'
              % (math.sqrt((sd ** 2).sum()), max(0.02, 1.5 * math.sqrt((sd ** 2).sum())))) 
        print('   ||a|-g| %.4f m/s^2               -> still_acc  >= %.3f'
              % (abs(np.linalg.norm(a[rest], axis=1).mean() - G),
                 max(0.05, 5 * abs(np.linalg.norm(a[rest], axis=1).mean() - G))))
        print('   陀螺零偏 %+.5f rad/s, 加速度零偏 (%+.4f, %+.4f) m/s^2'
              % (w[rest, 2].mean(), aw[:, 0].mean(), aw[:, 1].mean()))

    # --- 3. 加速度過程雜訊 vs 角速度 -------------------------------------------
    R = np.array([quat_to_matrix(x) for x in q])
    aw = np.einsum('nij,nj->ni', R, a) - np.array([0.0, 0.0, G])
    yaw = np.array([quat_to_yaw(x) for x in q])
    c, s_ = np.cos(yaw), np.sin(yaw)
    ab = np.stack([c * aw[:, 0] + s_ * aw[:, 1], -s_ * aw[:, 0] + c * aw[:, 1]], 1)
    vwx = np.interp(t, ot, np.gradient(op[:, 0], ot))
    vwy = np.interp(t, ot, np.gradient(op[:, 1], ot))
    awt = np.stack([np.gradient(vwx, t), np.gradient(vwy, t)], 1)
    abt = np.stack([c * awt[:, 0] + s_ * awt[:, 1],
                    -s_ * awt[:, 0] + c * awt[:, 1]], 1)
    res = ab - abt
    wz = np.abs(w[:, 2])
    print('\n3) 加速度過程雜訊 (IMU 扣完重力 vs ground truth 微分出來的真實加速度)')
    print('   |wz| 區間      樣本    殘差 std      連續時間 (/sqrt(%.0f Hz))' % rate)
    lo_sd = hi_sd = hi_w = None
    for lo, hi in ((0, 0.2), (0.2, 0.5), (0.5, 1), (1, 2), (2, 4), (4, 8),
                   (8, 12), (12, 20), (20, 40)):
        m = (wz >= lo) & (wz < hi)
        if m.sum() < 40:
            continue
        sd = float(np.sqrt((res[m] ** 2).sum(1).mean() / 2))
        print(f'   {lo:5.1f}-{hi:5.1f}  {m.sum():6d}   {sd:9.3f} m/s^2   {sd / math.sqrt(rate):9.4f}')
        if hi <= 0.5:
            lo_sd = sd if lo_sd is None else min(lo_sd, sd)
        if lo >= 4:
            hi_sd, hi_w = sd, 0.5 * (lo + hi)
    if lo_sd:
        print('   -> sigma_acc       ~ %.3f' % (lo_sd / math.sqrt(rate)))
    if hi_sd:
        print('   -> sigma_acc_omega ~ %.3f  (= %.2f / %.1f / sqrt(%.0f))'
              % (hi_sd / hi_w / math.sqrt(rate), hi_sd, hi_w, rate))

    # --- 4. 槓桿臂 -----------------------------------------------------------
    al = np.gradient(w[:, 2], t)
    m = (gs < 0.05) & (wz > 3.0)
    if m.sum() > 100:
        A = np.zeros((2 * m.sum(), 2))
        b = np.zeros(2 * m.sum())
        A[0::2, 0] = -w[m, 2] ** 2
        A[0::2, 1] = -al[m]
        b[0::2] = ab[m, 0]
        A[1::2, 0] = al[m]
        A[1::2, 1] = -w[m, 2] ** 2
        b[1::2] = ab[m, 1]
        lv, *_ = np.linalg.lstsq(A, b, rcond=None)
        print('\n4) IMU 離旋轉中心 (%+.4f, %+.4f) m, |r| = %.4f m  —— 解釋掉殘差的 %.1f%%'
              % (lv[0], lv[1], np.hypot(*lv), 100 * (1 - np.var(b - A @ lv) / np.var(b))))
        print('   |r| 夠小 (<1 cm) 就不用管; 大的話自旋時的向心加速度會被當成車子在加速')
    else:
        print('\n4) 沒有足夠的原地自旋樣本, 量不出槓桿臂')


def summary(label, r):
    e, j = r['err'], r['jump']
    print(f'  {label:32s} RMS {np.sqrt((e ** 2).mean()):6.2f}  '
          f'中位 {np.median(e):6.2f}  最大 {e.max():7.2f}  最後 {e[-1]:6.2f} m | '
          f'單步跳動 最大 {j.max():5.2f} m, >0.2m {int((j > 0.2).sum()):4d} 次 | '
          f'sigma峰 {r["sigma"].max():6.2f} m | 靜止 {100 * r["still"].mean():.0f}% | '
          f"自旋ZUPT {r['ins'].counts['spin']:5d} | "
          f"擋掉 {r['ins'].counts['rejected']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--measure', action='store_true',
                    help='從 bag 量出該用的參數 (不跑濾波器)')
    ap.add_argument('--forward-deg', type=float, default=-90.0)
    args = ap.parse_args()

    imu, odom = read_bag(args.bag)
    print(f'/imu {len(imu)} 筆, /odom {len(odom)} 筆, '
          f'{imu[-1][0] - imu[0][0]:.1f} 秒, '
          f'{len(imu) / max(imu[-1][0] - imu[0][0], 1e-9):.0f} Hz\n')

    if args.measure:
        measure(imu, odom)
        return

    base = dict(forward_deg=args.forward_deg)
    print('A/B (每一列餵的都是同一份資料, 差異只來自參數):')
    for lab, kw in (
            ('沒有 w 項的過程雜訊 + 不限幅', dict(sigma_acc=0.35, sigma_acc_omega=0.0,
                                          max_pos_correction=0.0)),
            ('只加位置修正限幅', dict(sigma_acc=0.35, sigma_acc_omega=0.0,
                               max_pos_correction=0.5)),
            ('只加 w 項', dict(max_pos_correction=0.0)),
            ('現在的預設 (兩個都有, 零偏 GM)', dict()),
            ('零偏改回隨機遊走 (rw)', dict(bias_model='rw')),
            ('靜止不做第二階段確認', dict(confirm=dict(acc_mean=0.0, gyro_mean=0.0,
                                                speed_gate=0.0))),
            ('NHC 用舊的份量 (sigma 0.15)', dict(nhc_sigma=0.15)),
            ('關掉 NHC', dict(nhc=False)),
            ('關掉 ZUPT/ZARU', dict(zupt=False, zaru=False)),
            ('開啟原地自旋 ZUPT (預設關)', dict(spin=True)),
            ('車頭軸給錯 (當 0)', dict(forward_deg=0.0)),
    ):
        kw = dict(base, **kw)
        summary(lab, replay(imu, odom, **kw))


if __name__ == '__main__':
    main()
