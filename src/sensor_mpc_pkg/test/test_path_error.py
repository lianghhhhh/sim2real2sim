#!/usr/bin/env python3
"""path_error.PathError 的測試 (只要 numpy): python3 test/test_path_error.py 或 pytest。"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sensor_mpc_pkg.path_error import PathError   # noqa: E402


def circle(R, step, ccw=True):
    n = int(round(2 * math.pi * R / step))
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    sgn = 1.0 if ccw else -1.0
    return np.c_[R * np.sin(a), sgn * (R - R * np.cos(a))], a


def test_circle_sign_heading_kappa():
    for ccw in (True, False):
        P, a = circle(1.2, 0.2, ccw)
        pe = PathError(P, loop=True)
        assert pe.loop
        th = a[7]
        x, y = P[7]
        yaw = th if ccw else -th
        left = np.array([-math.sin(yaw), math.cos(yaw)])
        r = pe.query(x + 0.01 * left[0], y + 0.01 * left[1], yaw)
        assert abs(r['cross'] - 0.01) < 1e-3, r            # 左邊 1 cm -> +1 cm
        assert abs(r['head_err']) < math.radians(0.5), r    # 頂點的切線 = 車頭
        assert abs(r['kappa'] - (1 if ccw else -1) / 1.2) < 0.01, r
        r = pe.query(x - 0.02 * left[0], y - 0.02 * left[1], yaw + 0.05)
        assert abs(r['cross'] + 0.02) < 1e-3 and abs(r['head_err'] - 0.05) < 0.01, r


def test_on_path_is_zero_even_in_curve():
    # 舊算法 (參考軌跡第 0 列) 在 R1.2 會給 ~0.83 cm; 這裡車子就在路徑上 -> 0
    P, a = circle(1.2, 0.1)
    pe = PathError(P, loop=True)
    r = pe.query(*P[30], a[30])
    assert abs(r['cross']) < 1e-9


def test_closed_by_repeat_and_open():
    P, _ = circle(1.0, 0.2)
    assert PathError(np.r_[P, P[:1]]).loop
    pe = PathError(np.c_[np.linspace(0, 3, 4), np.zeros(4)])
    assert not pe.loop
    r = pe.query(1.5, -0.1, 0.1)
    assert abs(r['cross'] + 0.1) < 1e-9 and abs(r['s'] - 1.5) < 1e-9 and r['kappa'] == 0.0


def test_figure8_hint_follows_lobe():
    up, _ = circle(1.0, 0.1, ccw=True)
    dn, _ = circle(1.0, 0.1, ccw=False)
    P = np.r_[up, dn]
    pe = PathError(P, loop=True)
    traj = np.r_[up, dn]                        # 照順序走一圈: 經過原點兩次
    r = pe.query_many(traj[:, 0], traj[:, 1])
    assert np.nanmax(np.abs(r['cross'])) < 1e-9
    assert np.all(np.diff(r['seg']) >= 0)       # 線段 index 單調往前, 沒有在原點跳到另一圈


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('OK', name)
