#!/usr/bin/env python3
"""不需要 ROS 就能跑的精度驗證。

用房間的真實幾何 (從 car.usd 量出來的: 內牆 x=±5 / y=±3, 三根 r=0.5 的柱子)
合成雷射掃描, 每個測距加 2 cm 高斯雜訊 —— 對應 SICK multiScan136 規格書裡的
rangeAccuracyM = 0.02。然後看配準結果離真值差多少。

    python3 test/test_matcher.py      # 直接跑, 會把數字印出來
    pytest test/test_matcher.py       # 當單元測試跑
"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from car_localization.gridmap import GridMap          # noqa: E402
from car_localization.matcher import ScanMatcher, rot2  # noqa: E402

MAP_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'maps', 'car_usd.npz')

# /World/Room 的內側幾何
WALLS = [((-5, -3), (5, -3)), ((5, -3), (5, 3)), ((5, 3), (-5, 3)), ((-5, 3), (-5, -3))]
PILLARS = [(3.8403739996184965, -2.174036853899636, 0.5),
           (-1.4477442018661726, 2.1850387053741924, 0.5),
           (-3.517427517728824, -1.1407622480806512, 0.5)]


def cast(px, py, angles):
    """從 (px, py) 往一堆方向打射線, 回傳距離。"""
    o = np.array([px, py])
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    best = np.full(angles.shape, np.inf)
    for (ax, ay), (bx, by) in WALLS:
        a = np.array([ax, ay])
        e = np.array([bx - ax, by - ay])
        den = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
        q = a - o
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (q[0] * (-e[1]) - q[1] * (-e[0])) / den
            u = (d[:, 0] * q[1] - d[:, 1] * q[0]) / den
        ok = np.isfinite(t) & (t > 1e-6) & (u >= 0) & (u <= 1)
        best = np.where(ok & (t < best), t, best)
    for cx, cy, r in PILLARS:
        f = o - np.array([cx, cy])
        b = 2 * (d @ f)
        c = f @ f - r * r
        disc = b * b - 4 * c
        ok = disc >= 0
        t1 = (-b - np.sqrt(np.where(ok, disc, 0))) / 2
        ok = ok & (t1 > 1e-6)
        best = np.where(ok & (t1 < best), t1, best)
    return best


def make_scan(rng, px, py, yaw, n=675, noise=0.02):
    """回傳感測器座標下的 2D 點 (N, 2)。"""
    az = np.linspace(0, 2 * np.pi, n, endpoint=False)
    r = cast(px, py, az + yaw) + rng.normal(0, noise, n)
    ok = np.isfinite(r) & (r > 0.3) & (r < 40)
    r, az = r[ok], az[ok]
    return np.stack([r * np.cos(az), r * np.sin(az)], axis=1)


def _poses(rng, k):
    out = []
    while len(out) < k:
        px, py = rng.uniform(-4.3, 4.3), rng.uniform(-2.3, 2.3)
        if min(math.hypot(px - c[0], py - c[1]) for c in PILLARS) < 0.9:
            continue
        out.append((px, py, rng.uniform(-np.pi, np.pi)))
    return out


def _matcher():
    assert os.path.exists(MAP_PATH), (
        f'找不到 {MAP_PATH}\n先在 host 上跑一次 ./scripts/make_map_from_usd.py')
    gmap = GridMap.load(MAP_PATH)
    assert gmap.exact, '這張地圖的距離場不是用表面點算的, 會有 ~2.5 cm 的定值偏差'
    return gmap, ScanMatcher(gmap)


def test_refine_yaw_locked():
    """yaw 由 IMU 給定 (預設模式): 只解 x/y。"""
    gmap, m = _matcher()
    rng = np.random.default_rng(0)
    errs = []
    for px, py, yaw in _poses(rng, 50):
        base = make_scan(rng, px, py, yaw) @ rot2(yaw).T
        guess = np.array([px, py]) + rng.normal(0, 0.20, 2)
        r = m.refine(base, guess, 0.0, lock_yaw=True)
        errs.append(np.linalg.norm(r.t - [px, py]))
    errs = np.array(errs)
    print(f'  yaw 鎖定  : 平均 {errs.mean() * 100:.2f} cm  最大 {errs.max() * 100:.2f} cm')
    assert errs.mean() < 0.01, errs.mean()
    assert errs.max() < 0.05, errs.max()


def test_refine_free_yaw():
    """yaw 也要解 (真車路線)。"""
    gmap, m = _matcher()
    rng = np.random.default_rng(1)
    pe, ye = [], []
    for px, py, yaw in _poses(rng, 30):
        guess_yaw = yaw + math.radians(rng.normal(0, 5))
        base = make_scan(rng, px, py, yaw) @ rot2(guess_yaw).T
        r = m.refine(base, np.array([px, py]) + rng.normal(0, 0.20, 2), 0.0,
                     lock_yaw=False)
        pe.append(np.linalg.norm(r.t - [px, py]))
        ye.append(abs(math.degrees((guess_yaw + r.delta) - yaw)))
    print(f'  yaw 自由  : 位置平均 {np.mean(pe) * 100:.2f} cm  '
          f'yaw 平均 {np.mean(ye):.3f} deg  最大 {max(ye):.3f} deg')
    assert np.mean(pe) < 0.01
    assert np.mean(ye) < 0.2


def test_global_localize():
    """完全不給初始位姿, 在整張地圖上找。"""
    gmap, m = _matcher()
    rng = np.random.default_rng(2)
    errs, ms = [], []
    for px, py, yaw in _poses(rng, 12):
        base = make_scan(rng, px, py, yaw) @ rot2(yaw).T
        t0 = time.perf_counter()
        r = m.global_localize(base, lock_yaw=True)
        ms.append((time.perf_counter() - t0) * 1e3)
        errs.append(np.linalg.norm(r.t - [px, py]))
    errs = np.array(errs)
    print(f'  全域(yaw 已知): {int((errs < 0.05).sum())}/{len(errs)} 次在 5 cm 內, '
          f'平均 {errs.mean() * 100:.2f} cm, {np.mean(ms):.0f} ms')
    assert (errs < 0.05).all()


def test_global_localize_unknown_yaw():
    """連 yaw 都不知道 —— 車子剛開機、還沒有任何先驗的情況。"""
    gmap, m = _matcher()
    rng = np.random.default_rng(3)
    ok = 0
    poses = _poses(rng, 5)
    for px, py, yaw in poses:
        base = make_scan(rng, px, py, yaw)
        r = m.global_localize(base, yaw_search=True, lock_yaw=False)
        e = np.linalg.norm(r.t - [px, py])
        ey = abs(math.degrees((r.delta - yaw + np.pi) % (2 * np.pi) - np.pi))
        ok += (e < 0.05 and ey < 1.0)
    print(f'  全域(yaw 未知): {ok}/{len(poses)} 次成功')
    assert ok == len(poses)


def test_map_sanity():
    """地圖本身要長得像那個房間。"""
    gmap, _ = _matcher()
    x0, y0, x1, y1 = gmap.bounds
    assert x0 < -6 and x1 > 6 and y0 < -4 and y1 > 4, gmap.bounds
    # 內牆周長 2*(10+6)=32 m, 三根柱子各 pi m -> 約 (32+9.4)/0.05 = 828 格
    assert 700 < gmap.n_occupied < 1100, gmap.n_occupied
    # 幾個已知答案的抽查點 (期望值直接從 /World/Room 的幾何算)
    probes = [
        ((0.0, -2.0), 1.0, '到 y=-3 的牆'),
        ((4.0, 0.0), 1.0, '到 x=+5 的牆'),
        ((0.0, 0.0), math.hypot(1.4477442018661726, 2.1850387053741924) - 0.5,
         '房間中心到最近的柱子'),
    ]
    q = np.array([p for p, _, _ in probes])
    d, _, _, _ = gmap.sample(q)
    for (pt, want, why), got in zip(probes, d):
        assert abs(got - want) < 0.01, f'{pt} {why}: 量到 {got:.4f}, 應為 {want:.4f}'
    print(f'  地圖: {gmap.n_occupied} 格, 抽查 {len(probes)} 個已知距離全部吻合 '
          f'(誤差 < 1 cm)')


if __name__ == '__main__':
    print('用房間真實幾何合成掃描 (675 點, 測距雜訊 2 cm) 驗證配準精度\n')
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
            except AssertionError as e:
                fails += 1
                print(f'  !! {name} 失敗: {e}')
    print('\n全部通過' if not fails else f'\n{fails} 項失敗')
    sys.exit(1 if fails else 0)
