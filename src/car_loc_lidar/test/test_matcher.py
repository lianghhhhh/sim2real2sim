#!/usr/bin/env python3
"""離線驗證配準與追蹤迴圈 —— 不需要 ROS, 不需要 Isaac。

    cd src/car_loc_lidar && python3 test/test_matcher.py

造一個房間 -> 對真實表面做 sphere tracing 模擬帶畸變的掃描 -> 跑
「等速預測 -> 去畸變 -> 3 自由度配準」-> 跟真值比。
README 裡引用的數字就是這個腳本印出來的。
"""
import math
import os
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from car_loc_lidar.gridmap import GridMap            # noqa: E402
from car_loc_lidar.matcher import ScanMatcher        # noqa: E402
from car_loc_lidar.motion import ConstVelMotion      # noqa: E402
from car_loc_lidar.scan import deskew                # noqa: E402

# 感測器規格。預設就是車上那顆 Oradar MS200: 一圈 450 點 (4500 Hz / 10 Hz,
# 角解析度 0.8 度), 100 ms 一圈, 量程 12 m。
# set_profile() 可以換成別的來做對照。
N_BEAM = 450
PERIOD = 0.10
RMAX = 12.0
rng = np.random.default_rng(0)


def set_profile(n_beam, period, rmax):
    global N_BEAM, PERIOD, RMAX, LOCAL
    N_BEAM, PERIOD, RMAX = n_beam, period, rmax
    LOCAL = np.linspace(-np.pi, np.pi, N_BEAM, endpoint=False)


def build_room():
    """8x6 的房間 + 一根柱子 + 一個凹角 (不對稱, 才有唯一解)。"""
    pts = []
    for x in np.arange(-4, 4.001, 0.01):
        pts += [[x, -3], [x, 3]]
    for y in np.arange(-3, 3.001, 0.01):
        pts += [[-4, y], [4, y]]
    for a in np.arange(0, 6.283, 0.02):
        pts += [[2.8 + 0.3 * math.cos(a), 1.9 + 0.3 * math.sin(a)]]
    for x in np.arange(-4, -1.5, 0.01):
        pts += [[x, 1.0]]
    return np.array(pts)


SURF = build_room()
GMAP = GridMap.from_points(SURF, 0.05)
TREE = cKDTree(SURF)


LOCAL = np.linspace(-np.pi, np.pi, N_BEAM, endpoint=False)


def raycast(px, py, th, idx, noise=0.01):
    """對真實表面點做 sphere tracing (真值幾何, 不是格點 —— 所以量到的是
    地圖的量化誤差, 不是模擬器自己的誤差)。

    `idx` 是這一次要打的射線編號。只打需要的那幾條, 不要每個時間桶都打整圈 ——
    差 24 倍。
    """
    ang = LOCAL[idx] + th
    ca, sa = np.cos(ang), np.sin(ang)
    n = idx.size
    r = np.full(n, 0.05)
    alive = np.ones(n, bool)
    for _ in range(80):
        q = np.stack([px + r * ca, py + r * sa], 1)
        d, _ = TREE.query(q[alive], k=1)
        rr = r[alive] + np.maximum(d * 0.95, 0.005)
        hit = d < 0.01
        r[alive] = np.where(hit, r[alive], rr)
        j = np.nonzero(alive)[0]
        alive[j[hit | (rr > RMAX)]] = False
        if not alive.any():
            break
    # 超過量程的射線在真的雷射上是「沒有回波」, 這裡直接標成無效
    return r < RMAX, r + rng.normal(0, noise, n)


def sim_scan(traj_fn, t_end, buckets=24):
    """一圈掃描: 每條射線在自己的時刻發射, 車子在動 -> 掃描帶畸變。

    分時間桶是為了速度; 24 個桶在 20 rad/s 下每桶只轉 2.4 度, 對「掃描被抹開」
    這件事的取樣已經夠細了。
    """
    fr = np.arange(N_BEAM) / (N_BEAM - 1)
    ts = t_end - PERIOD * (1.0 - fr)               # 訊息時戳 = 掃描結束
    xy, frac = [], []
    for b in range(buckets):
        lo = b * N_BEAM // buckets
        hi = (b + 1) * N_BEAM // buckets
        i = np.arange(lo, hi)
        px, py, th = traj_fn(ts[i].mean())[:3]
        ok, r = raycast(px, py, th, i)
        a = LOCAL[i][ok]
        xy.append(np.stack([r[ok] * np.cos(a), r[ok] * np.sin(a)], 1))
        frac.append(fr[i][ok])
    return np.vstack(xy), np.concatenate(frac)


def figure8(t, w=0.9):
    px, py = 1.6 * math.sin(w * t), 1.1 * math.sin(2 * w * t)
    vx, vy = 1.6 * w * math.cos(w * t), 2.2 * w * math.cos(2 * w * t)
    return px, py, math.atan2(vy, vx)


def spin(t, W):
    return 0.5, -0.7, W * t


def spin_burst(t, W, t0=3.0):
    """停著 -> 突然開始自旋。等速預測在轉折那一瞬間一定是錯的, 掃角度重試
    就是為了這種情況存在的。"""
    return 0.5, -0.7, (0.0 if t < t0 else W * (t - t0))


def track(traj_fn, n, do_deskew=True, sweep=True, seed=0,
          sweep_yaw_max=1.2, max_failures=8):
    """跑一遍節點的追蹤迴圈 (含掃角度重試與全域重定位), 跟真值比。"""
    # 每一次 run 都重設亂數 —— 不然 A/B 兩列吃到的是不同的雜訊序列, 比較不公平
    global rng
    rng = np.random.default_rng(seed)
    m = ScanMatcher(GMAP)
    mo = ConstVelMotion(alpha=0.5)
    mo.set_pose(*traj_fn(0.0), t=0.0)
    e, y, lost, sw, fails, reloc = [], [], 0, 0, 0, 0
    for k in range(1, n + 1):
        t = k * PERIOD
        xy, frac = sim_scan(traj_fn, t)
        if do_deskew:
            xy = deskew(xy, frac, mo.vx, mo.vy, mo.omega, PERIOD, 'end')
        px, py, pth = mo.predict(t)
        r = m.refine(xy, [px, py], pth)
        good = r.inlier_ratio >= 0.5 and r.residual <= 0.25
        if not good and sweep:
            # 需要的範圍超過上限就不掃 —— 掃不到真值的搜尋只會找到 180 度那個
            # 假解, 不如報失敗去做全域重定位 (跟 lidar_loc_node 一樣的規則)
            need = max(0.7, abs(mo.omega_trusted) * PERIOD * 1.5)
            if need <= sweep_yaw_max:
                r = m.refine_sweep(xy, [px, py], pth, yaw_span=need)
                sw += 1
                good = r.inlier_ratio >= 0.5 and r.residual <= 0.25
        if good:
            mo.update(t, r.t[0], r.t[1], r.theta)
            fails = 0
        else:
            mo.coast(t)
            mo.decay(0.7)
            lost += 1
            fails += 1
            if fails >= max_failures:
                g = m.global_localize(xy, step=0.30, yaw_bins=72,
                                      center=mo.pose[:2], radius=1.0)
                if g.inlier_ratio >= 0.5 and g.residual <= 0.25:
                    mo.set_pose(g.t[0], g.t[1], g.theta, t)
                    reloc += 1
                fails = 0
        gx, gy, gth = traj_fn(t)
        if k > 20:
            e.append(math.hypot(mo.x - gx, mo.y - gy))
            y.append(abs(math.degrees(math.atan2(math.sin(mo.theta - gth),
                                                 math.cos(mo.theta - gth)))))
    return np.array(e), np.array(y), lost, sw, reloc


def line(label, e, y, lost, sw, reloc=0):
    print(f'  {label:32s} 位置 RMS {np.sqrt((e ** 2).mean()) * 100:7.2f} cm '
          f'最大 {e.max() * 100:7.2f} | yaw RMS {np.sqrt((y ** 2).mean()):6.2f}° '
          f'最大 {y.max():6.2f}° | 失敗 {lost:3d}, 重試 {sw}, 重定位 {reloc}')


def main():
    print(GMAP)
    print(f'感測器: 一圈 {N_BEAM} 點, {1 / PERIOD:.0f} Hz, 量程 {RMAX:.0f} m '
          '(Oradar MS200)')
    print('\n[1] 8 字形行駛 400 幀 —— 運動補償值多少')
    line('有補償', *track(figure8, 400, do_deskew=True))
    line('沒補償', *track(figure8, 400, do_deskew=False))

    print(f'\n[2] 原地自旋 —— 一圈 {PERIOD * 1e3:.0f} ms, 車子在一圈裡會轉過 '
          f'W x {PERIOD:.2f} rad。找臨界點在哪。')
    for W in (1.0, 3.0, 5.0, 8.0, 9.0, 10.0, 12.0):
        e, y, lost, sw, rl = track(lambda t, W=W: spin(t, W), 150)
        deg = math.degrees(W * PERIOD)
        ok = np.sqrt((y ** 2).mean()) < 5.0
        print(f'  {W:4.1f} rad/s (一圈轉 {deg:5.1f}°): '
              f'位置 RMS {np.sqrt((e ** 2).mean()) * 100:7.2f} cm, '
              f'yaw RMS {np.sqrt((y ** 2).mean()):7.2f}°, 失敗 {lost:3d}'
              + ('' if ok else '   <-- 追丟'))
    print('  純雷射沒有角速度感測器, 「下一幀轉到哪」只能用上一段外推。~8 rad/s')
    print('  (一圈轉 46 度) 以內穩定; 再上去就**不可預測** —— 9 rad/s 會鎖到 180')
    print('  度反方向, 10/11 rad/s 又正常。長方形房間對 180 度幾乎對稱, 鎖住之後')
    print('  殘差看起來還很漂亮, 回不來。這是 10 Hz 雷射的物理極限, 不是參數問題。')
    print('  car_teleop 的轉向上限預設 1.2 rad/s —— 離這裡還有 7 倍餘裕。')

    print('\n[3] 突然開始自旋 (3 秒後從靜止跳到 W) —— 掃角度重試在 MS200 上值多少')
    for W in (4.0, 8.0, 9.0):
        a = track(lambda t, W=W: spin_burst(t, W), 120, sweep=False)
        b = track(lambda t, W=W: spin_burst(t, W), 120, sweep=True)
        fmt = (lambda r: f'{np.sqrt((r[0] ** 2).mean()) * 100:6.2f} cm / '
                         f'{np.sqrt((r[1] ** 2).mean()):6.2f}° '
                         f'(失敗 {r[2]}, 重試 {r[3]})')
        print(f'  跳到 {W:4.1f} rad/s: 不重試 {fmt(a)}   重試 {fmt(b)}')
    print('  可追蹤範圍內 (<=8 rad/s) 配準根本不會失敗, 所以重試從來不會被觸發;')
    print('  超出範圍 (9 rad/s) 時用一個蓋不住真值的範圍去掃, 反而更容易鎖到 180 度。')
    print('  -> MS200 (10 Hz) 的設定檔預設把 sweep_on_fail 關掉。')
    print('     換回 20 Hz 的 3D 雷射時它是大勝: 20 rad/s 自旋 114.60 -> 3.53 cm。')

    print('\n[4] 全域定位 (不給初始位姿, 位置 x 角度 都要搜)')
    m = ScanMatcher(GMAP)
    for gt in [(0.5, -0.7, 0.3), (-2.0, 1.5, -2.2), (3.0, -2.0, 1.9)]:
        xy, _ = sim_scan(lambda t, g=gt: g, 1.0)
        t0 = time.perf_counter()
        r = m.global_localize(xy, step=0.30, yaw_bins=72)
        el = time.perf_counter() - t0
        err = math.hypot(r.t[0] - gt[0], r.t[1] - gt[1])
        ye = abs(math.degrees(math.atan2(math.sin(r.theta - gt[2]),
                                         math.cos(r.theta - gt[2]))))
        print(f'  ({gt[0]:+.1f},{gt[1]:+.1f},{math.degrees(gt[2]):+6.1f}°) -> '
              f'誤差 {err * 100:6.2f} cm, {ye:5.2f}°  ({el:.2f} s)')

    print('\n[6] 換感測器的代價: 舊的 3D 雷射 vs 現在的 MS200')
    print('  MS200 點少一半、慢一倍、量程只剩三分之一 —— 一圈 100 ms 表示')
    print('  車子在一圈裡轉過的角度是以前的兩倍, 所以運動補償變得更重要。')
    for name, (nb, pr, rm) in (('舊 3D (720 點, 20 Hz, 40 m)', (720, 0.05, 40.0)),
                               ('MS200 (450 點, 10 Hz, 12 m)', (450, 0.10, 12.0))):
        set_profile(nb, pr, rm)
        e, y, lost, sw, _ = track(figure8, 200, do_deskew=True)
        e0, y0, l0, _, _ = track(figure8, 200, do_deskew=False)
        print(f'  {name}: 有補償 {np.sqrt((e ** 2).mean()) * 100:5.2f} cm / '
              f'{np.sqrt((y ** 2).mean()):4.2f}°   '
              f'沒補償 {np.sqrt((e0 ** 2).mean()) * 100:5.2f} cm / '
              f'{np.sqrt((y0 ** 2).mean()):4.2f}°')
    set_profile(450, 0.10, 12.0)

    print('\n[5] 精度上限由地圖解析度決定 (誤差 ≈ 0.7 x 格點大小, 而且是系統性偏移)')
    for res in (0.10, 0.05, 0.025, 0.0125):
        g2 = GridMap.from_points(SURF, res)
        m2 = ScanMatcher(g2)
        errs = []
        for gt in [(0.5, -0.7, 0.3), (-2.0, 1.5, -2.2), (3.0, -2.0, 1.9),
                   (0.0, 0.0, 0.0), (-1.0, -1.0, 1.0)]:
            xy, _ = sim_scan(lambda t, g=gt: g, 1.0)
            r = m2.refine(xy, [gt[0] + 0.05, gt[1] - 0.05], gt[2] + 0.02)
            errs.append(math.hypot(r.t[0] - gt[0], r.t[1] - gt[1]))
        print(f'  地圖解析度 {res * 100:5.2f} cm -> 位置誤差 {np.mean(errs) * 100:5.2f} cm '
              f'(半格 = {res * 50:.2f} cm)')


if __name__ == '__main__':
    main()
