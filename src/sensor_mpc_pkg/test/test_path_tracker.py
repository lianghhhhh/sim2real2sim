#!/usr/bin/env python3
"""path_tracker 的離線測試 (不需要 ROS / Isaac, 秒級):

    cd src/sensor_mpc_pkg && python3 test/test_path_tracker.py

1. 封閉路徑 (config/path_kros_car_mpc.csv): 讓一台理想車沿路徑走兩圈, 目標點永遠在前方、
   航向連續、沒有進回歸模式、idx 單調繞圈
2. 車頭反向時自動選 BACKWARD
3. 離路徑 3 m: 回歸曲線從車子出發、終點在路徑上
4. 不封閉路徑: 走到底回 finished, 目標點停在終點
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sensor_mpc_pkg.path_tracker import PathTracker, load_path_csv, wrap_pi  # noqa: E402

CSV = os.path.join(os.path.dirname(__file__), '..', 'config', 'path_kros_car_mpc.csv')


def test_loop_follow():
    pts = load_path_csv(CSV)
    tr = PathTracker(pts, loop=True)
    # 理想車: 每步從第一個目標點前進, 朝向目標航向
    xy = pts[0] + np.array([0.05, -0.03])
    d = pts[1] - pts[0]
    yaw = math.atan2(d[1], d[0])
    r = tr.update(xy, yaw)
    assert tr.forward, '車頭順著點序, 應該是 FORWARD'
    seg_total = np.linalg.norm(np.diff(pts, axis=0), axis=1).sum() + np.linalg.norm(pts[0] - pts[-1])
    segs = np.diff(np.r_[pts, pts[:1]], axis=0)
    segang = np.arctan2(segs[:, 1], segs[:, 0])
    max_corner = np.abs([wrap_pi(a) for a in np.diff(np.r_[segang, segang[:1]])]).max()
    n_steps = int(2 * seg_total / 0.1) + 20
    idxs, max_dist, max_dpsi = [], 0.0, 0.0
    prev_psi = None
    for step in range(n_steps):
        r = tr.update(xy, yaw)
        assert not r['recovery'], f"不該進回歸模式 (dist {r['dist']:.3f})"
        assert r['targets'].shape == (10, 2) and r['headings'].shape == (10,)
        # 目標點都在車子前方 (沿車頭方向的投影 > 0)
        fwd = np.array([math.cos(yaw), math.sin(yaw)])
        ahead = (r['targets'] - xy) @ fwd
        assert ahead[0] > 0.0, f'第一個目標點在車子後面: {ahead[0]:.3f}'
        # 連續的目標點間距 ~ 0.1 m
        gaps = np.linalg.norm(np.diff(r['targets'], axis=0), axis=1)
        assert np.all(gaps < 0.1 + 1e-6) and np.all(gaps > 0.02), gaps
        # 航向連續
        dpsi = np.abs([wrap_pi(a) for a in np.diff(r['headings'])])
        max_dpsi = max(max_dpsi, dpsi.max())
        if prev_psi is not None:       # 航向只會在轉角處跳, 跳的量不超過最尖的轉角
            assert abs(wrap_pi(r['headings'][0] - prev_psi)) < max_corner + 0.05
        prev_psi = r['headings'][0]
        if step > 0:                   # 第 0 步是刻意放在路徑外 0.058 m 的起點
            max_dist = max(max_dist, r['dist'])
        idxs.append(r['idx'])
        xy = r['targets'][0].copy()
        yaw = r['headings'][0]
    assert max_dist < 0.02, f'理想車偏離路徑 {max_dist:.3f} m'
    # idx 繞了兩圈: 每一步 idx 只能 +0/+1 或從尾巴跳回 0
    steps = np.diff(idxs)
    n = len(pts)
    assert np.all((steps == 0) | (steps == 1) | (steps == -(n - 1))), set(steps.tolist())
    assert np.sum(steps == -(n - 1)) >= 1, '應該至少繞回起點一次'
    print(f'[1] 封閉路徑 {n_steps} 步 OK: 最大偏離 {max_dist:.4f} m, 目標航向最大跳 '
          f'{math.degrees(max_dpsi):.1f}° (路徑最尖的轉角 {math.degrees(max_corner):.1f}°)')


def test_backward():
    pts = load_path_csv(CSV)
    tr = PathTracker(pts, loop=True)
    d = pts[1] - pts[0]
    yaw = math.atan2(-d[1], -d[0])          # 車頭反向
    r = tr.update(pts[0], yaw)
    assert not tr.forward, '車頭反向應該是 BACKWARD'
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    assert ((r['targets'][0] - pts[0]) @ fwd) > 0
    assert abs(wrap_pi(r['headings'][0] - yaw)) < 0.6
    tr2 = PathTracker(pts, loop=True, direction='forward')
    tr2.update(pts[0], yaw)
    assert tr2.forward
    print('[2] 方向判斷 OK (auto -> BACKWARD, direction=forward 強制)')


def test_recovery():
    pts = load_path_csv(CSV)
    tr = PathTracker(pts, loop=True, far_threshold=1.5)
    d = pts[1] - pts[0]
    nrm = np.array([-d[1], d[0]]) / np.linalg.norm(d)
    xy = pts[0] + 3.0 * nrm                 # 離路徑 3 m
    yaw = math.atan2(d[1], d[0])
    r = tr.update(xy, yaw)
    true_dist = _dist_to_polyline(xy, pts)          # 路徑別的地方可能比 3 m 近
    assert r['recovery'], f"dist {r['dist']:.2f} (真正 {true_dist:.2f}) 應該進回歸模式"
    assert abs(r['dist'] - true_dist) < 0.05, (r['dist'], true_dist)
    # 第一個目標點靠近車子, 最後一個在路徑上
    assert np.linalg.norm(r['targets'][0] - xy) < 1.0
    last = r['targets'][-1]
    dmin = min(np.linalg.norm(pts - last, axis=1).min(),
               _dist_to_polyline(last, pts))
    assert dmin < 0.05, f'回歸曲線終點離路徑 {dmin:.3f} m'
    # 走回去: 跟著目標點, 最後應該回到正常模式
    for _ in range(200):
        r = tr.update(xy, yaw)
        xy = r['targets'][0].copy()
        yaw = r['headings'][0]
        if not r['recovery']:
            break
    assert not r['recovery'], '沿回歸曲線走 200 步還沒回到路徑'
    print(f"[3] 回歸曲線 OK: 回到路徑時 dist {r['dist']:.3f} m")


def _dist_to_polyline(p, pts):
    best = np.inf
    n = len(pts)
    for k in range(n):
        a, b = pts[k], pts[(k + 1) % n]
        ab = b - a
        t = np.clip((p - a) @ ab / (ab @ ab), 0, 1)
        best = min(best, np.linalg.norm(p - (a + t * ab)))
    return best


def test_open_path():
    pts = np.c_[np.linspace(0, 5, 26), np.zeros(26)]      # 5 m 直線
    tr = PathTracker(pts, loop=False, goal_tolerance=0.15)
    xy, yaw = np.array([0.0, 0.02]), 0.0
    finished_at = None
    for k in range(200):
        r = tr.update(xy, yaw)
        assert r['remaining'] is not None
        if r['finished']:
            finished_at = k
            break
        xy = r['targets'][0].copy()
        yaw = r['headings'][0]
    assert finished_at is not None, '直線 5 m 走 200 步還沒 finished'
    assert xy[0] > 4.8, xy
    r = tr.update(np.array([5.5, 0.0]), 0.0)              # 衝過頭
    assert r['finished'] and np.allclose(r['targets'], [5.0, 0.0], atol=1e-6)
    print(f'[4] 不封閉路徑 OK: 第 {finished_at} 步到終點, 終點後目標點固定在終點')


if __name__ == '__main__':
    test_loop_follow()
    test_backward()
    test_recovery()
    test_open_path()
    print('all OK')
