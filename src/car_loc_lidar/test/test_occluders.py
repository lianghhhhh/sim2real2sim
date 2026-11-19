#!/usr/bin/env python3
"""地圖上沒有的東西 (箱子) 對配準的影響, occluder_dist 開 / 關對照 —— 不需要 ROS / Isaac:

    cd src/car_loc_lidar && python3 test/test_occluders.py

用真的 maps/room.yaml, 對佔據格 + 方塊做射線模擬 MS200 (450 點, 雜訊 5 mm)。每個情境比較:
    pos/yaw 誤差、殘差、inlier、sigma (cov 的位置標準差)、
    鎖死偵測會不會判 (殘差 > 2.5 cm 或 inlier < 85%; 開的時候再加穿牆 > 3%)、融合會不會擋 (sigma > 2.5 mm)

情境:
  A 正確位姿附近 (偏 2 cm / 1°) 起配, 沒有箱子          -> 開關應該一樣
  B 同上, 車子前方 1.4 m 一個 0.4 m 箱子 (2026-11-18 Isaac 那一幕)
  C 同上, 三個箱子
  D 從 180° 對稱解起配 (鎖死)                        -> 開了之後仍然要被判鎖死 (靠穿牆比例)
  E 從偏 0.5 m / 20° 起配 (追丟)                      -> 不能因為拿掉點就變成「看起來很好」
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from car_loc_lidar.gridmap import GridMap      # noqa: E402
from car_loc_lidar.matcher import ScanMatcher  # noqa: E402

MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps', 'room.yaml')
LASER_X = 0.045
N_BEAM, RMAX = 450, 12.0
LOCK_RES, LOCK_INL, LOCK_ST, SIGMA_MAX = 0.025, 0.85, 0.03, 0.0025
rng = np.random.default_rng(0)


def raycast(gmap, x, y, th, boxes):
    lx, ly = x + LASER_X * math.cos(th), y + LASER_X * math.sin(th)
    ang = np.linspace(-np.pi, np.pi, N_BEAM, endpoint=False)
    rs = np.arange(0.05, RMAX, 0.01)
    px = lx + np.outer(np.cos(th + ang), rs)
    py = ly + np.outer(np.sin(th + ang), rs)
    col, row, ok = gmap._to_cell(np.c_[px.ravel(), py.ravel()])
    hit = np.zeros(px.size, dtype=bool)
    hit[ok] = gmap.occ[row[ok], col[ok]]
    hit = hit.reshape(px.shape)
    for cx, cy, h in boxes:
        hit |= (np.abs(px - cx) <= h) & (np.abs(py - cy) <= h)
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    r = np.where(first >= 0, rs[np.maximum(first, 0)], np.nan) + rng.normal(0, 0.005, N_BEAM)
    keep = np.isfinite(r)
    # 車體座標 (前端已經加上雷射外參)
    return np.c_[LASER_X + r[keep] * np.cos(ang[keep]), r[keep] * np.sin(ang[keep])]


def box_ahead(x, y, th, dist, size=0.4, lateral=0.0):
    return (x + dist * math.cos(th) - lateral * math.sin(th),
            y + dist * math.sin(th) + lateral * math.cos(th), size / 2)


def run(m, xy, start, truth):
    r = m.refine(xy, start[:2], start[2])
    sig = math.sqrt(max(r.cov[0, 0], r.cov[1, 1])) if r.cov is not None else float('inf')
    e = math.hypot(r.t[0] - truth[0], r.t[1] - truth[1])
    eth = abs(math.degrees(math.atan2(math.sin(r.theta - truth[2]), math.cos(r.theta - truth[2]))))
    st = m.see_through_ratio(xy, r.t, r.theta)
    lock = r.residual > LOCK_RES or r.inlier_ratio < LOCK_INL or (m.occluder_dist > 0 and st > LOCK_ST)
    return dict(e=e, eth=eth, res=r.residual, inl=r.inlier_ratio, sig=sig, lock=lock,
                gate=sig > SIGMA_MAX, occ=r.n_occluded, st=st)


def fmt(d):
    return (f"pos {d['e'] * 100:6.2f} cm  yaw {d['eth']:5.2f}°  殘差 {d['res'] * 100:5.2f} cm  "
            f"inlier {d['inl']:4.0%}  sigma {d['sig'] * 1000:6.2f} mm  拿掉 {d['occ']:3d}  穿牆 {d['st']:5.1%}  "
            f"{'鎖死!' if d['lock'] else '     '} {'融合擋' if d['gate'] else ''}")


def main():
    gmap = GridMap.load_nav2(MAP)
    off = ScanMatcher(gmap, occluder_dist=0.0)
    on = ScanMatcher(gmap, occluder_dist=0.15, occluder_max_ratio=0.50)
    print(f'{gmap}, 房間內部 {on.interior.sum() * gmap.resolution ** 2:.1f} m²\n')

    # 房間裡的測試位姿 (離牆 / 柱子 >= 0.6 m), 含 Isaac 那一幕
    poses = [(-2.77, -1.66, math.radians(18.7)), (0.0, 0.0, 0.0), (1.5, 1.2, math.radians(-120)),
             (-3.5, 2.0, math.radians(-45)), (3.8, -0.5, math.radians(170)), (0.5, -2.2, math.radians(60))]
    summary = {}
    for name, mk_boxes, start_fn in [
        ('A 沒箱子', lambda p: [], lambda p: (p[0] + 0.02, p[1] - 0.02, p[2] + math.radians(1))),
        ('B 前方 1.4 m 箱子', lambda p: [box_ahead(*p, 1.4)],
         lambda p: (p[0] + 0.02, p[1] - 0.02, p[2] + math.radians(1))),
        ('C 三個箱子', lambda p: [box_ahead(*p, 1.4), box_ahead(*p, 1.0, lateral=0.8),
                                 box_ahead(*p, -1.2, lateral=-0.5)],
         lambda p: (p[0] + 0.02, p[1] - 0.02, p[2] + math.radians(1))),
        ('D 鎖到 180°', lambda p: [], lambda p: (-p[0], -p[1], p[2] + math.pi)),
        ('D2 鎖到 180° + 箱子', lambda p: [box_ahead(*p, 1.4)], lambda p: (-p[0], -p[1], p[2] + math.pi)),
        ('E 追丟 (0.5 m / 20°)', lambda p: [box_ahead(*p, 1.4)],
         lambda p: (p[0] + 0.35, p[1] - 0.35, p[2] + math.radians(20))),
    ]:
        print(f'== {name}')
        for p in poses:
            xy = raycast(gmap, *p, mk_boxes(p))
            st = start_fn(p)
            a, b = run(off, xy, st, p), run(on, xy, st, p)
            print(f'  ({p[0]:+.2f},{p[1]:+.2f},{math.degrees(p[2]):+4.0f}°) 關 {fmt(a)}')
            print(f'  {"":20s} 開 {fmt(b)}')
            summary.setdefault(name, []).append((a, b))
        print()

    print('== 摘要 (6 個位姿)')
    for name, rows in summary.items():
        for k, label in ((0, '關'), (1, '開')):
            ds = [r[k] for r in rows]
            print(f'  {name:22s} {label}: 位置誤差 max {max(d["e"] for d in ds) * 100:6.2f} cm, '
                  f'鎖死判定 {sum(d["lock"] for d in ds)}/6, 融合擋 {sum(d["gate"] for d in ds)}/6')

    # 判定
    ok = True
    for a, b in summary['A 沒箱子']:
        ok &= abs(a['e'] - b['e']) < 1e-6 and b['occ'] == 0
    for _, b in summary['B 前方 1.4 m 箱子']:
        ok &= b['e'] < 0.01 and not b['lock'] and not b['gate']
    for _, b in summary['C 三個箱子']:                      # 有一個箱子離牆 0.13 m, 允許 2 cm
        ok &= b['e'] < 0.02 and not b['lock'] and not b['gate']
    # 鎖到 180° 要被抓到; 只有「從這個位置看不到柱子」那種 (兩個方向的掃描一模一樣) 允許漏,
    # 6 個裡最多 1 個 (關的時候是 0/6 抓到)
    for key in ('D 鎖到 180°', 'D2 鎖到 180° + 箱子'):
        miss = sum(not ((b['e'] < 0.05) or b['lock'] or b['gate']) for _, b in summary[key])
        print(f'  {key}: 開 漏抓 {miss}/6')
        ok &= miss <= 1
    for _, b in summary['E 追丟 (0.5 m / 20°)']:
        ok &= (b['e'] < 0.05) or b['lock'] or b['gate']
    print('\n結果:', '通過' if ok else '**不通過**')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
