"""nav_mpc_pkg/geometry.py 的離線測試 (純 numpy, host 就能跑):

    cd src/nav_mpc_pkg && python3 test/test_geometry.py      # 或 pytest
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from nav_mpc_pkg.geometry import (PathProgress, corridor_hits, outside_box,  # noqa: E402
                                  resample_polyline, scan_to_points, to_world)


def test_resample_even_spacing():
    raw = np.array([[0, 0], [0.03, 0], [0.03, 0], [1.0, 0], [1.0, 0.55]])
    pts = resample_polyline(raw, 0.1)
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert np.allclose(pts[0], [0, 0]) and np.allclose(pts[-1], [1.0, 0.55])
    # 弧長間距 1.55 m / 16 段 = 0.0969; 跨過直角的那一段弦長會短一點
    assert d.max() <= 0.1 + 1e-9 and d.min() > 0.06
    assert len(pts) == 17


def test_progress_monotonic_on_self_crossing_path():
    # 往 +x 走 2 m, 繞回來從同一條線上方 0.1 m 往 -x 走回去: 回程時不能跳回去程那段
    t = np.linspace(0, math.pi, 30)
    pts = np.r_[np.column_stack([np.linspace(0, 2, 21), np.zeros(21)]),
                np.column_stack([2 + 0.05 * np.sin(t), 0.05 - 0.05 * np.cos(t)])[1:],
                np.column_stack([np.linspace(2, 0, 21), np.full(21, 0.1)])[1:]]
    pts = resample_polyline(pts, 0.05)
    prog = PathProgress(pts)
    s_prev = -1
    for k in range(0, len(pts), 2):
        s, d = prog.update(pts[k] + [0.0, 0.01])
        assert s >= s_prev - 1e-9, (k, s, s_prev)
        assert d < 0.02
        s_prev = s
    assert s_prev > prog.total - 0.2


def test_sub_polyline():
    pts = resample_polyline(np.array([[0, 0], [2, 0], [2, 2]]), 0.1)
    prog = PathProgress(pts)
    sub = prog.sub_polyline(1.55, 2.55)
    assert np.allclose(sub[0], [1.55, 0]) and np.allclose(sub[-1], [2, 0.55])
    L = np.linalg.norm(np.diff(sub, axis=0), axis=1).sum()
    assert abs(L - 1.0) < 1e-9


def test_corridor_hits():
    poly = np.array([[0.0, 0.0], [1.5, 0.0]])
    pts = np.array([
        [1.0, 0.10],     # 在走廊裡 -> 擋路, along 1.0
        [0.5, -0.24],    # 邊緣內 -> 擋路
        [0.5, 0.30],     # 走廊外 (旁邊的牆) -> 不算
        [-0.1, 0.0],     # 車子後面 -> 不算
        [1.6, 0.0],      # 超過 1.5 m 一點但離終點 0.1 < 0.25 -> 算 (終點附近有東西)
        [2.5, 0.0],      # 太遠 -> 不算
    ])
    mask, along = corridor_hits(pts, poly, 0.25)
    assert mask.tolist() == [True, True, False, False, True, False]
    assert abs(along[0] - 1.0) < 1e-9 and np.isinf(along[2])


def test_corridor_follows_turn():
    # 路線左轉: 原本車頭正前方 (直線延伸) 的東西不在路上, 轉彎後的路上的才算
    pts = resample_polyline(np.r_[[[0, 0]], [[0.5 + 0.5 * math.sin(a), 0.5 - 0.5 * math.cos(a)]
                                             for a in np.linspace(0, math.pi / 2, 20)]], 0.05)
    prog = PathProgress(pts)
    s0, _ = prog.update([0.0, 0.0])
    poly = prog.sub_polyline(s0, s0 + 1.5)
    # (1.2, 0) 在原本車頭正前方 (離圓弧 0.36 m); (0.9, 0.3) 在圓弧上 (離 5 cm)
    mask, _ = corridor_hits(np.array([[1.2, 0.0], [0.9, 0.3]]), poly, 0.25)
    assert mask.tolist() == [False, True]


def test_scan_transform_and_footprint():
    # 雷射在 base_link 前方 0.045 m; 車子在 (1, 2) 朝 +y。正前方 1 m 的回波應該在 map (1, 3.045)
    ranges = np.full(360, np.inf)
    ranges[180] = 1.0                       # angle_min -pi + 180 * 1° = 0 rad
    ranges[0] = 0.1                         # 正後方 0.1 m: 打到自己車身
    pts = scan_to_points(ranges, -math.pi, math.radians(1.0), 0.05, 8.0)
    base = to_world(pts, 0.045, 0.0, 0.0)
    base = base[outside_box(base, [-0.3, 0.3, -0.22, 0.22])]
    assert len(base) == 1
    world = to_world(base, 1.0, 2.0, math.pi / 2)
    assert np.allclose(world[0], [1.0, 3.045], atol=1e-9)


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'{name}: ok')
