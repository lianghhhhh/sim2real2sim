#!/usr/bin/env python3
"""把錄下來的 rosbag2 讀出來, 離線重跑濾波器。這支吃的是**你自己錄的**資料。

    ros2 bag record -o my_run /imu /joint_states /odom     # 跑一輪, 錄下來
    python3 test/replay_bag.py my_run
    python3 test/replay_bag.py my_run --measure

**不需要 ROS**, 只用 sqlite3 + 手刻 CDR 解析 (`--selftest` 驗這一段)。為什麼
要有這個: 參數不能用猜的, 也不該每次都重開 Isaac 跑一輪 —— 錄一次 bag, 之後
所有 A/B 兩秒跑完, 而且每次餵的是**完全一樣**的資料, 差異才真的是參數造成的。

> **錄的時候一定要把 `/joint_states` 也錄進去。** 現有的
> `car_run_data/isaac/bags/spin12` 只有 `/imu` `/odom` `/lidar` `/yolo`,
> 沒有輪速, 所以這支跑不了它 (會直接告訴你缺什麼)。

跟另外兩支的分工:

    test_wheel_ins.py   合成資料 —— 假設都成立的乾淨情況, 也是唯一能做
                        「純 IMU vs IMU+輪速」對照的地方 (誤差模型是我們放的)
    replay_csv.py       car_run_data/sim_data.csv —— 真的 Isaac 資料, 但沒有
                        原始加速度, 所以只驗證「輪速 + 陀螺儀」那一半
    replay_bag.py       **你自己的 bag** —— 完整的一條路 (原始 IMU + 輪速),
                        而且參數是對著你自己的車量出來的

`--measure` 量出來的每一項都直接對應一個參數:

| 量什麼 | 決定 |
| --- | --- |
| ground truth 的行進方向 vs yaw | `forward_deg` |
| 有效輪半徑 (最小平方對 ground truth) | `wheel_scale` |
| 靜止時的輪速雜訊底線 | `still_wheel` |
| 同側前後輪轉速差 vs 輪速誤差 | `slip_spread` / `slip_spread_max` |
| 輪速殘差 | `wheel_sigma` |
| 左右輪速差算出來的角速度 vs 真值 | 能不能用輪速算 yaw (在這台車上不行) |
"""
from __future__ import annotations

import argparse
import math
import os
import sqlite3
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from car_loc_wheel.wheel_ins import (G, WheelIns, WheelReader,     # noqa: E402
                                     WheelStillDetector, quat_to_matrix,
                                     quat_to_yaw, wrap_pi)

WHEELS = ('front_left_joint', 'front_right_joint',
          'rear_left_joint', 'rear_right_joint')


# ---------------------------------------------------------------- CDR 解析
class Cdr:
    """最小的 CDR reader。CDR 每個基本型別都要對齊到自己的大小, 而且對齊是
    相對於 **encapsulation header 之後**的位置算的。"""

    def __init__(self, buf: bytes):
        self.b = buf
        self.little = buf[1] == 1
        self.o = 4

    def _align(self, n):
        pad = (self.o - 4) % n
        if pad:
            self.o += n - pad

    def _get(self, fmt, n):
        self._align(n)
        v = struct.unpack_from(('<' if self.little else '>') + fmt, self.b, self.o)[0]
        self.o += n
        return v

    def i32(self):
        return self._get('i', 4)

    def u32(self):
        return self._get('I', 4)

    def f64(self):
        return self._get('d', 8)

    def string(self):
        n = self.u32()
        s = self.b[self.o:self.o + n - 1].decode('utf-8', 'replace')
        self.o += n
        return s

    def f64a(self, n):
        return [self.f64() for _ in range(n)]

    def f64seq(self):
        return self.f64a(self.u32())

    def strseq(self):
        return [self.string() for _ in range(self.u32())]

    def header(self):
        sec = self.i32()
        nsec = self.u32()
        self.string()
        return sec + nsec * 1e-9


def parse_imu(buf):
    c = Cdr(buf)
    t = c.header()
    q = c.f64a(4)                        # x, y, z, w
    c.f64a(9)
    w = c.f64a(3)
    c.f64a(9)
    a = c.f64a(3)
    return t, np.array(q), np.array(w), np.array(a)


def parse_joint(buf):
    """sensor_msgs/JointState: header, string[] name, float64[] position,
    velocity, effort。"""
    c = Cdr(buf)
    t = c.header()
    name = c.strseq()
    c.f64seq()                           # position
    vel = c.f64seq()
    return t, name, vel


def parse_odom(buf):
    c = Cdr(buf)
    t = c.header()
    c.string()
    p = c.f64a(3)
    q = c.f64a(4)
    return t, np.array(p[:2]), quat_to_yaw(q)


def read_bag(path):
    db = [f for f in os.listdir(path) if f.endswith('.db3')]
    if not db:
        raise SystemExit(f'{path} 裡沒有 .db3')
    con = sqlite3.connect(os.path.join(path, db[0]))
    ids = dict(con.execute('SELECT name, id FROM topics'))
    for want in ('/imu', '/joint_states'):
        if want not in ids:
            raise SystemExit(f'bag 裡沒有 {want} (有的是 {sorted(ids)})。'
                             '錄的時候要 ros2 bag record /imu /joint_states /odom')
    if '/odom' not in ids:
        print('警告: bag 裡沒有 /odom (ground truth), 只能跑不能評分')
    imu, joint, odom = [], [], []
    for tid, data in con.execute(
            'SELECT topic_id, data FROM messages ORDER BY timestamp'):
        if tid == ids['/imu']:
            imu.append(parse_imu(data))
        elif tid == ids['/joint_states']:
            joint.append(parse_joint(data))
        elif tid == ids.get('/odom'):
            odom.append(parse_odom(data))
    con.close()
    return imu, joint, odom


# ---------------------------------------------------------------- CDR 自我測試
class _CdrWriter:
    """把訊息編回 CDR —— 只給 --selftest 用。

    為什麼需要它: 上面的解析器是手刻的, 而 CDR 的對齊是**相對於 encapsulation
    header 之後**算的, 所以 `frame_id` 的長度會把後面所有欄位的對齊推移。
    `/imu` 與 `/odom` 的解析可以拿真的 bag 交叉驗證 (兩個 topic 各自解出來的
    yaw 中位差 0.038 度), 但 `/joint_states` 在手邊的 bag 裡沒有錄 —— 而它偏偏
    是唯一有**變長字串陣列**的, 對齊最容易錯。所以用 round-trip 補。
    """

    def __init__(self):
        self.b = bytearray(b'\x00\x01\x00\x00')      # little-endian

    def _align(self, n):
        pad = (len(self.b) - 4) % n
        if pad:
            self.b += b'\x00' * (n - pad)

    def u32(self, v):
        self._align(4)
        self.b += struct.pack('<I', v)

    def i32(self, v):
        self._align(4)
        self.b += struct.pack('<i', v)

    def f64(self, v):
        self._align(8)
        self.b += struct.pack('<d', v)

    def string(self, t):
        e = t.encode() + b'\x00'
        self.u32(len(e))
        self.b += e

    def f64seq(self, a):
        self.u32(len(a))
        for x in a:
            self.f64(x)

    def strseq(self, a):
        self.u32(len(a))
        for x in a:
            self.string(x)


def selftest() -> int:
    names = list(WHEELS)
    vel = [6.83, -1.5, 6.83, 0.0]
    ok = True
    print('CDR 解析自我測試 (JointState round-trip)')
    # frame_id 換長度 -> 把後面欄位的對齊推移, 逼出 off-by-alignment
    for frame in ('', 'base_link', 'a_longer_frame_id_x'):
        w = _CdrWriter()
        w.i32(12)
        w.u32(345678901)
        w.string(frame)
        w.strseq(names)
        w.f64seq([0.11, 0.22, 0.33, 0.44])
        w.f64seq(vel)
        w.f64seq([1.0, 2.0, 3.0, 4.0])
        t, n, v = parse_joint(bytes(w.b))
        good = (abs(t - 12.345678901) < 1e-9 and n == names
                and all(abs(x - y) < 1e-12 for x, y in zip(v, vel)))
        ok &= good
        print(f"  frame_id {frame!r:22s} -> t={t:.9f} 名字對 {n == names} "
              f"vel={[round(x, 3) for x in v]}  {'OK' if good else '**FAIL**'}")
    # Isaac 有時只發 position, velocity 是空的
    w = _CdrWriter()
    w.i32(0)
    w.u32(0)
    w.string('b')
    w.strseq(names)
    w.f64seq([0.0] * 4)
    w.f64seq([])
    w.f64seq([])
    _, _, v = parse_joint(bytes(w.b))
    ok &= (v == [])
    print(f"  velocity 是空的 -> {v}  {'OK' if v == [] else '**FAIL**'}")
    print('  (/imu 與 /odom 的解析請拿真的 bag 驗: 兩者各自解出來的 yaw 應該一致,'
          '\n   實測 car_run_data/isaac/bags/spin12 上中位差 0.038 度。)')
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


# ---------------------------------------------------------------- 工具
def interp_truth(odom, t):
    """在 ground truth 裡內插出 t 時刻的 (x, y, yaw)。"""
    ts = np.array([o[0] for o in odom])
    i = int(np.searchsorted(ts, t))
    if i <= 0 or i >= len(ts):
        return None
    a, b = odom[i - 1], odom[i]
    k = 0.0 if b[0] <= a[0] else (t - a[0]) / (b[0] - a[0])
    return (a[1][0] + k * (b[1][0] - a[1][0]), a[1][1] + k * (b[1][1] - a[1][1]),
            a[2] + k * wrap_pi(b[2] - a[2]))


def wheel_arrays(joint):
    """把 JointState 串列變成 (t, 4×omega)。名字對不上就退回前四個。"""
    t = np.array([j[0] for j in joint])
    W = np.zeros((4, len(joint)))
    by_name = True
    for i, (_, name, vel) in enumerate(joint):
        if name and len(name) == len(vel) and all(n in name for n in WHEELS):
            tab = dict(zip(name, vel))
            W[:, i] = [tab[n] for n in WHEELS]
        else:
            by_name = False
            W[:, i] = list(vel[:4]) + [0.0] * max(0, 4 - len(vel))
    return t, W, by_name


# ---------------------------------------------------------------- 量參數
def measure(imu, joint, odom, radius):
    if not odom:
        raise SystemExit('--measure 需要 /odom (ground truth)')
    jt, W, by_name = wheel_arrays(joint)
    print('=' * 78)
    print(f'  量參數: IMU {len(imu)} 筆 ({len(imu) / max(imu[-1][0] - imu[0][0], 1e-9):.0f} Hz), '
          f'輪速 {len(joint)} 筆 ({len(jt) / max(jt[-1] - jt[0], 1e-9):.0f} Hz), '
          f'ground truth {len(odom)} 筆')
    if not by_name:
        print(f'  警告: /joint_states 裡找不到 {WHEELS}, 用了前四個 velocity。')
        print('        順序不是 [FL, FR, RL, RR] 的話打滑偵測會配錯對。')
    print('=' * 78)

    # ground truth 在每一筆輪速的時刻
    gt = [interp_truth(odom, t) for t in jt]
    ok = np.array([g is not None for g in gt])
    gx = np.array([g[0] if g else 0.0 for g in gt])
    gy = np.array([g[1] if g else 0.0 for g in gt])
    gyaw = np.array([g[2] if g else 0.0 for g in gt])
    dt = np.gradient(jt)
    vx, vy = np.gradient(gx) / dt, np.gradient(gy) / dt
    wz_gt = np.gradient(np.unwrap(gyaw)) / dt

    print('\n[1] 車頭是哪一軸 (forward_deg)')
    mov = ok & (np.hypot(vx, vy) > 0.15)
    ang = np.degrees(np.arctan2(np.sin(np.arctan2(vy, vx) - gyaw),
                                np.cos(np.arctan2(vy, vx) - gyaw)))[mov]
    print(f'    atan2(dy, dx) - yaw: 中位 {np.median(ang):+.1f}° '
          f'(標準差 {ang.std():.1f}°, n={mov.sum()})')
    print('    -> forward_deg 就給這個值。REP-103 的車是 0, car.usd 是 -90。')

    fwd = gyaw + math.radians(float(np.median(ang)))
    vf = vx * np.cos(fwd) + vy * np.sin(fwd)
    vlat = -vx * np.sin(fwd) + vy * np.cos(fwd)
    med = np.median(W, axis=0)
    spread = np.maximum(np.abs(W[0] - W[2]), np.abs(W[1] - W[3]))

    print('\n[2] 有效輪半徑 (wheel_scale)')
    clean = ok & (np.abs(vf) > 0.1) & (spread < 1.0) & (np.abs(wz_gt) < 0.5)
    if clean.sum() < 30:
        print('    乾淨行駛的樣本太少, 沒辦法量 —— 錄一段平順的直線再試')
    else:
        k = float(np.sum(med[clean] * vf[clean]) / np.sum(med[clean] ** 2))
        e = k * med[clean] - vf[clean]
        print(f'    n={clean.sum()}  有效半徑 {k:.4f} m (幾何 {radius}, '
              f'差 {100 * (k / radius - 1):+.1f}%)  -> wheel_scale {k / radius:.3f}')
        print(f'    殘差 std {e.std():.4f} p95 {np.percentile(np.abs(e), 95):.4f} m/s '
              f'-> wheel_sigma 給 {max(e.std(), 0.02):.2f} 左右')

    print('\n[3] 靜止門檻 (still_wheel)')
    rest = ok & (np.abs(vf) < 0.01) & (np.abs(wz_gt) < 0.02)
    if rest.sum() > 20:
        print(f'    靜止 n={rest.sum()}: 各輪 |w| p95 '
              f'{np.percentile(np.abs(W[:, rest]), 95):.4f} '
              f'max {np.abs(W[:, rest]).max():.4f} rad/s')
    for th in (0.2, 0.5, 1.0, 2.0):
        quiet = ok & (np.max(np.abs(W), axis=0) < th)
        bad = quiet & (np.abs(vf) > 0.1)
        print(f'    still_wheel={th:.1f}: 判為靜止 {100 * quiet.mean():4.1f}%, '
              f'其中車子其實在動的 {100 * bad.mean():.2f}%')
    print('    選「誤判率還接近 0」裡面最大的那個。')

    print('\n[4] 打滑指標 (slip_spread)')
    scale = 1.0
    if clean.sum() >= 30:
        scale = float(np.sum(med[clean] * vf[clean]) / np.sum(med[clean] ** 2)) / radius
    err = scale * radius * med - vf
    for lo, hi in ((0, 0.5), (0.5, 2), (2, 5), (5, 1e9)):
        m = ok & (np.abs(vf) > 0.05) & (spread >= lo) & (spread < hi)
        if m.sum() < 10:
            continue
        print(f'    同側差 {lo:4.1f}-{hi if hi < 1e8 else float("inf"):>4.1f}: '
              f'n={m.sum():5d} |輪速誤差| med {np.median(np.abs(err[m])):.3f} '
              f'p95 {np.percentile(np.abs(err[m]), 95):.3f} '
              f'max {np.abs(err[m]).max():.2f} m/s')
    print('    slip_spread 給「誤差開始明顯變大」的那一格, max 給「已經沒救」的那格。')

    print('\n[5] 能不能用左右輪速差算 yaw (這台車不行)')
    m = ok & (np.abs(vf) > 0.05)
    est = radius * ((W[1] + W[3]) / 2 - (W[0] + W[2]) / 2) / 0.25
    print(f'    r*(w_R-w_L)/track vs 真值: 相關係數 {np.corrcoef(est[m], wz_gt[m])[0, 1]:.3f}')
    m3 = m & (np.abs(wz_gt) < 3)
    print(f'    最小平方反推的有效輪距 '
          f'{float(np.sum(est[m3] * 0.25 * wz_gt[m3]) / np.sum(wz_gt[m3] ** 2)):.3f} m '
          '(幾何 0.25)')
    print('    相關係數遠低於 1 就表示輪子在轉彎時一直在滑 -> yaw 要用陀螺儀。')

    print('\n[6] 側向速度 (sigma_cross)')
    if clean.sum() > 30:
        print(f'    乾淨段 p95 {np.percentile(np.abs(vlat[clean]), 95):.4f} m/s')
    sp = m & (np.abs(wz_gt) > 3)
    if sp.sum() > 10:
        print(f'    高速自旋 p95 {np.percentile(np.abs(vlat[sp]), 95):.3f} m/s')
    print('=' * 78)


# ---------------------------------------------------------------- 重跑
def replay(imu, joint, odom, *, wheel=True, slip_gate=True, zupt=True,
           gravity='orientation', yaw_imu=True, radius=0.075, wheel_scale=0.976,
           forward_deg=-90.0, **kw):
    """照節點的邏輯把 IMU 與輪速依時間交錯餵進濾波器。"""
    ins = WheelIns(forward_deg=forward_deg, wheel_scale=wheel_scale, sigma_k=0.0, **kw)
    reader = WheelReader(radius=radius)
    det = WheelStillDetector()
    jt, W, _ = wheel_arrays(joint)

    # 兩條串流按時間合併
    stream = ([(t, 'i', k) for k, (t, *_) in enumerate(imu)]
              + [(t, 'j', k) for k, t in enumerate(jt)])
    stream.sort(key=lambda r: r[0])

    if odom:
        g0 = interp_truth(odom, imu[0][0]) or (0.0, 0.0, 0.0)
    else:
        g0 = (0.0, 0.0, 0.0)
    ins.set_pose(g0[0], g0[1], g0[2], imu[0][0])

    last_t = None
    last_gyro = 0.0
    errs, yerrs, ts = [], [], []
    for t, kind, k in stream:
        if kind == 'i':
            _, q, w, a = imu[k]
            last_gyro = float(w[2])
            dt = None if last_t is None else t - last_t
            last_t = t
            if gravity == 'orientation':
                R = quat_to_matrix(q)
                aw = R @ a - np.array([0.0, 0.0, G])
                h = ins.heading
                a_fwd = float(math.cos(h) * aw[0] + math.sin(h) * aw[1])
            else:
                a_fwd = float(a[0] * math.cos(ins.phi) + a[1] * math.sin(ins.phi))
            ins.predict(t, a_fwd, last_gyro)
            if yaw_imu:
                ins.update_yaw(quat_to_yaw(q), 0.02, dt or 0.0)
            if det.is_still() and zupt:
                if ins.still_since is None:
                    ins.still_since = t
                ins.zupt()
                ins.zero_accel(a_fwd)
                ins.zaru(last_gyro)
                if t - ins.still_since >= ins.anchor_after:
                    ins.anchor_position()
            else:
                ins.still_since = None
                ins.anchor = None
            if odom:
                g = interp_truth(odom, t)
                if g is not None:
                    errs.append(math.hypot(ins.x[0] - g[0], ins.x[1] - g[1]))
                    yerrs.append(abs(math.degrees(wrap_pi(ins.yaw - g[2]))))
                    ts.append(t)
        else:
            det.add(t, W[:, k], last_gyro)
            if wheel:
                r = reader.read(None, list(W[:, k]))
                if r is not None:
                    v_w, spread, _ = r
                    ins.update_wheel(v_w, spread if slip_gate else 0.0)
    if not errs:
        return None
    e, ye = np.array(errs), np.array(yerrs)
    gt_dist = float(np.sum(np.hypot(np.diff([o[1][0] for o in odom]),
                                    np.diff([o[1][1] for o in odom]))))
    return {'rms': float(np.sqrt((e ** 2).mean())), 'max': float(e.max()),
            'final': float(e[-1]), 'yaw': float(ye[-1]), 'yaw_max': float(ye.max()),
            'curve': e, 't': np.array(ts), 'gt_dist': gt_dist,
            'drift': 100 * float(e.max()) / max(gt_dist, 1e-6),
            'counts': dict(ins.counts)}


def row(name, r):
    if r is None:
        print(f'  {name:26s} (沒有 ground truth, 跳過)')
        return
    c = r['counts']
    print(f'  {name:26s} {r["rms"]:7.3f} {r["max"]:7.3f} {r["final"]:7.3f} '
          f'{r["drift"]:6.1f}% {r["yaw"]:7.1f} | 輪速{c["wheel"]:5d} '
          f'放寬{c["slip"]:4d} 擋{c["rejected"]:4d} ZUPT{c["zupt"]:5d} '
          f'零加速跳過{c["za_skip"]:4d}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', nargs='?')
    ap.add_argument('--measure', action='store_true')
    ap.add_argument('--selftest', action='store_true',
                    help='驗手刻的 CDR 解析, 不需要 bag')
    ap.add_argument('--radius', type=float, default=0.075)
    ap.add_argument('--scale', type=float, default=0.976)
    ap.add_argument('--forward-deg', type=float, default=-90.0)
    ap.add_argument('--gravity', default='orientation',
                    choices=('orientation', 'none'))
    ap.add_argument('--gyro-yaw', action='store_true',
                    help='yaw 只靠陀螺儀積分 (真車的 6 軸 IMU)')
    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest())
    if not args.bag:
        ap.error('要給 bag 目錄 (或用 --selftest)')

    imu, joint, odom = read_bag(args.bag)
    if not imu or not joint:
        raise SystemExit('bag 裡的 /imu 或 /joint_states 是空的')
    if args.measure:
        measure(imu, joint, odom, args.radius)
        return

    base = dict(radius=args.radius, wheel_scale=args.scale,
                forward_deg=args.forward_deg, gravity=args.gravity,
                yaw_imu=not args.gyro_yaw)
    dur = imu[-1][0] - imu[0][0]
    print('=' * 104)
    print(f'  {args.bag}: {dur:.1f} 秒, IMU {len(imu)} 筆, 輪速 {len(joint)} 筆')
    print('=' * 104)
    print(f'  {"":26s} {"RMS":>7} {"最大":>7} {"終點":>7} {"漂移率":>7} {"yaw°":>7}')
    full = replay(imu, joint, odom, **base)
    row('全開 (預設)', full)
    row('不擋打滑', replay(imu, joint, odom, **base, slip_gate=False))
    row('不用輪速 (純 IMU)', replay(imu, joint, odom, **base, wheel=False))
    row('沒有 ZUPT / ZARU', replay(imu, joint, odom, **base, zupt=False))
    row('scale = 1.0', replay(imu, joint, odom, **{**base, 'wheel_scale': 1.0}))

    if full:
        print(f'\n  ground truth 走了 {full["gt_dist"]:.1f} m。誤差隨時間:')
        e, ts = full['curve'], full['t']
        for f in (0.1, 0.25, 0.5, 0.75, 1.0):
            i = min(int(len(e) * f), len(e) - 1)
            print(f'    第 {ts[i] - ts[0]:6.1f} 秒: {e[i]:7.3f} m')
        print('\n  「不用輪速」那一列就是純 IMU —— 兩者的差距是這個 package 的價值。')
        print('  「零加速跳過」是靜止偵測比加速度慢半拍時被擋下來的筆數 (見 wheel_ins.py')
        print('   的 zero_accel), 每次停車前後各幾筆是正常的。')
    print('=' * 104)


if __name__ == '__main__':
    main()
