"""車子相對整條路徑的誤差 (純 numpy, 不需要 ROS): 橫向距離、路徑航向、曲率。

為什麼不用參考軌跡第 0 列
------------------------
2026-11-18 以前 sensor_mpc_node 的 cte / head_err 是對 build_reference() 的第 0 列算的。那一點是
從前方第 1、2 個目標點 (0.1 m、0.2 m) 的連線往後延伸出來的, 在半徑 R 的彎道裡落在圓弧**外側**約
0.1 x 0.2 / (2R) (R1.2 時 0.83 cm), 航向也是 0.1 m 前方的切線 (R1.2 時差 4.8°)。所以舊的 cte 左轉
偏正、右轉偏負, 跟車子實際離路徑多遠無關。這裡改成量到路徑折線 (= path_tracker 用的那條) 的
真正距離。

定義
----
    cross    到最近線段的距離, 車在路徑左邊為正 (m)
    heading  投影點的路徑切線 (rad): 頂點的切線 = 前後兩段方向的平均, 線段中間線性內插
    head_err wrap(yaw - heading)
    kappa    投影點所在線段的曲率 (rad/m, 左轉正) = 兩端頂點切線差 / 段長
    s        投影點沿路徑的弧長 (m)
    seg      最近線段的 index

首尾相同 (或 loop=True) 的路徑當成封閉。
"""
from __future__ import annotations

import math

import numpy as np


def _wrap(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


class PathError:

    def __init__(self, points, loop=None):
        p = np.asarray(points, dtype=float)[:, :2]
        keep = np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-9]
        p = p[keep]
        if len(p) < 2:
            raise ValueError('路徑至少要 2 個點')
        closed = len(p) > 2 and np.linalg.norm(p[0] - p[-1]) < 1e-6
        if closed:
            p = p[:-1]
        self.loop = bool(closed if loop is None else (loop or closed))
        self.pts = p
        if self.loop:
            self.a, self.b = p, np.r_[p[1:], p[:1]]
        else:
            self.a, self.b = p[:-1], p[1:]
        ab = self.b - self.a
        self.ab = ab
        self.len2 = np.einsum('ij,ij->i', ab, ab)
        self.length = np.sqrt(self.len2)
        self.s0 = np.r_[0.0, np.cumsum(self.length)[:-1]]
        self.total = float(self.length.sum())
        seg_ang = np.arctan2(ab[:, 1], ab[:, 0])
        n = len(seg_ang)
        # 每個線段兩端頂點的切線
        if self.loop:
            prev = np.roll(seg_ang, 1)
            vt = seg_ang + 0.5 * _wrap(prev - seg_ang)        # 頂點 i (線段 i 的起點)
            self.t_start = vt
            self.t_end = np.roll(vt, -1)
        else:
            vt = np.empty(n + 1)
            vt[0], vt[-1] = seg_ang[0], seg_ang[-1]
            vt[1:-1] = seg_ang[:-1] + 0.5 * _wrap(seg_ang[1:] - seg_ang[:-1])
            self.t_start, self.t_end = vt[:-1], vt[1:]
        self.dtheta = _wrap(self.t_end - self.t_start)
        self.kappa = self.dtheta / self.length

    def query(self, x, y, yaw=None, hint=None, window=None):
        """-> dict(cross, heading, head_err, kappa, s, seg)。
        hint / window: 只在線段 hint ± window 裡找 (自己交叉或很近的路線用; None = 全部)。"""
        if hint is not None and window is not None:
            n = len(self.a)
            idx = np.arange(int(hint) - window, int(hint) + window + 1)
            idx = idx % n if self.loop else np.unique(np.clip(idx, 0, n - 1))
        else:
            idx = slice(None)
        a, ab, l2 = self.a[idx], self.ab[idx], self.len2[idx]
        q = np.array([x, y], dtype=float)
        t = np.clip(np.einsum('ij,ij->i', q - a, ab) / l2, 0.0, 1.0)
        c = a + t[:, None] * ab
        d = np.linalg.norm(q - c, axis=1)
        j = int(np.argmin(d))
        seg = int(np.arange(len(self.a))[idx][j])
        side = ab[j, 0] * (y - a[j, 1]) - ab[j, 1] * (x - a[j, 0])
        cross = float(d[j]) if side >= 0 else -float(d[j])
        tj = float(t[j])
        heading = float(_wrap(self.t_start[seg] + tj * self.dtheta[seg]))
        out = dict(cross=cross, heading=heading, kappa=float(self.kappa[seg]),
                   s=float(self.s0[seg] + tj * self.length[seg]), seg=seg, head_err=float('nan'))
        if yaw is not None:
            out['head_err'] = float(_wrap(yaw - heading))
        return out

    def query_many(self, xs, ys, yaws=None):
        """一整條軌跡 (陣列) -> dict of 陣列。逐點找最近線段, 用上一點的線段當 hint (視窗 = 全部線段的 1/4,
        至少 20), 第一點全域找; 8 字交叉、平行車道都不會跳到另一段。"""
        xs, ys = np.asarray(xs, float), np.asarray(ys, float)
        yaws = None if yaws is None else np.asarray(yaws, float)
        win = max(20, len(self.a) // 4)
        keys = ('cross', 'heading', 'head_err', 'kappa', 's', 'seg')
        res = {k: np.full(len(xs), np.nan) for k in keys}
        hint = None
        for i in range(len(xs)):
            if not (math.isfinite(xs[i]) and math.isfinite(ys[i])):
                continue
            r = self.query(xs[i], ys[i], None if yaws is None else yaws[i], hint,
                           None if hint is None else win)
            for k in keys:
                res[k][i] = r[k]
            hint = r['seg']
        return res
