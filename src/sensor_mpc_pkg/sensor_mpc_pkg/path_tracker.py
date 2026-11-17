"""路徑追蹤 (純 numpy, 不需要 ROS): 車子位姿 + 路徑點 -> 前方 n 個目標點與航向。

這是 Isaac OmniGraph ScriptNode 版 (mlp_control_pkg/temp_curve.py) 的移植。那一版讀的是
Isaac 裡車子的 ground truth 位姿, 實體車沒有, 所以這裡把同一套演算法搬到 ROS 端, 位姿由
感測器定位 (/fusion_loc/odom 之類) 提供。演算法跟原版一樣:

  1. 在上一次最近點附近 (±search_radius 個點) 找離車最近的路徑點, 投影到它前後兩段裡
     比較近的那一段線段上
  2. 從投影點往前 spacing, 2*spacing, ... 取 n_targets 個目標點 (沿弧長線性內插)
  3. 離路徑超過 far_threshold 時改走「回歸曲線」: 從車子目前位置與朝向, 用三次 Bezier
     平滑接回路徑前方 max(3 m, 2 x 距離) 的地方, 目標點取在那條曲線上
  4. 第一次呼叫時決定要順著點的順序走還是倒過來走 (車頭跟哪個方向比較接近)

跟原版不一樣的地方:
  * 航向輸出的是**絕對航向** (rad), 不是「路徑航向 - 車子航向」。原版輸出相對值再由控制
    節點加回車子的 yaw, 兩邊用的 yaw 如果不是同一則訊息就會差一點; 這裡直接給絕對值
  * 支援不封閉的路徑 (loop=False): 走到底就把剩下的目標點都放在終點, 並回報 finished
  * 方向判斷用 ±切線跟車頭比 (原版兩個候選其實都是正向切線)

路徑點是 BasisCurves 的**控制點**連成的折線 (原版也是這樣用), 不是 Isaac 畫出來的平滑曲線。
"""
from __future__ import annotations

import math

import numpy as np


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_path_csv(path):
    """讀 export_path_from_usd.py 產生的 CSV (x,y[,z]; 以 # 開頭的是註解, 第一列可以是表頭)。
    回傳 (n, 2) 的 x, y。"""
    rows = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            parts = [p.strip() for p in s.replace(';', ',').split(',')]
            try:
                rows.append((float(parts[0]), float(parts[1])))
            except (ValueError, IndexError):
                continue        # 表頭
    if len(rows) < 2:
        raise ValueError(f'{path}: 路徑點不到 2 個')
    return np.asarray(rows, dtype=float)


class PathTracker:

    def __init__(self, points, loop=True, n_targets=10, spacing=0.1, search_radius=20,
                 far_threshold=1.5, direction='auto', goal_tolerance=0.15,
                 recovery_min_merge=3.0, recovery_merge_gain=2.0, recovery_ctrl_gain=1.2):
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[0] < 2:
            raise ValueError('路徑至少要 2 個點')
        pts = pts[:, :2].copy()
        keep = np.r_[True, np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-9]
        pts = pts[keep]                       # 去掉連續重複的點
        if loop and len(pts) > 2 and np.linalg.norm(pts[0] - pts[-1]) < 1e-9:
            pts = pts[:-1]                    # 首尾同一點的封閉路徑: 最後一點不要重複
        if direction not in ('auto', 'forward', 'backward'):
            raise ValueError(f"direction 要是 auto / forward / backward, 收到 {direction!r}")
        self.pts0 = pts
        self.loop = bool(loop)
        self.n_targets = int(n_targets)
        self.spacing = float(spacing)
        self.search_radius = int(search_radius)
        self.far_threshold = float(far_threshold)
        self.direction = direction
        self.goal_tolerance = float(goal_tolerance)
        self.rec_min_merge = float(recovery_min_merge)
        self.rec_merge_gain = float(recovery_merge_gain)
        self.rec_ctrl_gain = float(recovery_ctrl_gain)
        self.reset()

    # ------------------------------------------------------------------ 狀態
    def reset(self):
        """忘掉方向與上一次的最近點 (車子被搬走、或要重新決定方向時用)。"""
        self.pts = None
        self.s = None             # 每個點的累積弧長
        self.total = None
        self.last_idx = None
        self.forward = None

    @property
    def initialized(self):
        return self.pts is not None

    @property
    def n(self):
        return len(self.pts0)

    def _next(self, i):
        if self.loop:
            return (i + 1) % len(self.pts)
        return min(i + 1, len(self.pts) - 1)

    def _init(self, xy, yaw):
        pts = self.pts0
        i = int(np.argmin(np.linalg.norm(pts - xy, axis=1)))
        n = len(pts)
        nxt = (i + 1) % n if self.loop else min(i + 1, n - 1)
        prv = (i - 1) % n if self.loop else max(i - 1, 0)
        tan = pts[nxt] - pts[prv]             # 這一點附近的正向切線
        if np.linalg.norm(tan) < 1e-9:
            tan = np.array([1.0, 0.0])
        fwd_diff = abs(wrap_pi(math.atan2(tan[1], tan[0]) - yaw))
        bwd_diff = abs(wrap_pi(math.atan2(-tan[1], -tan[0]) - yaw))
        if self.direction == 'forward':
            self.forward = True
        elif self.direction == 'backward':
            self.forward = False
        else:
            self.forward = fwd_diff <= bwd_diff
        self.pts = pts if self.forward else pts[::-1].copy()
        self.last_idx = i if self.forward else n - 1 - i
        seg = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        self.s = np.r_[0.0, np.cumsum(seg)]
        self.total = float(self.s[-1] + (np.linalg.norm(self.pts[0] - self.pts[-1])
                                         if self.loop else 0.0))

    # ------------------------------------------------------------------ 主要
    def update(self, xy, yaw):
        """xy (2,), yaw (rad) -> dict:
            targets (n_targets, 2)  目標點 (世界座標)
            headings (n_targets,)   每個目標點的絕對路徑航向 (rad)
            idx                     車子所在線段的起點 (在**原始**點序裡的 index)
            closest (2,)            車子在路徑上的投影點
            dist                    車子離路徑多遠 (m)
            proj                    車子沿 idx 那一段的位置 (m, 可以超出 [0, 段長])
            recovery                True = 離太遠, 目標點走的是回歸曲線
            finished                不封閉的路徑走到終點了 (封閉路徑永遠 False)
            remaining               到終點還有幾公尺 (封閉路徑是 None)
        """
        xy = np.asarray(xy, dtype=float).reshape(2)
        if not self.initialized:
            self._init(xy, yaw)
        pts, n = self.pts, len(self.pts)

        # 1. 最近的頂點 (上一次附近的視窗), 再在它前後兩段線段裡挑離車最近的那一段投影。
        #    原版只投影到「最近頂點之後那一段」, 車在前一段的後半時投影會被夾在頂點上,
        #    0.5 m 間距的折線轉角處目標點會跑到路徑外面 (test_path_tracker 抓到 0.37 m)。
        win = np.arange(self.last_idx - self.search_radius, self.last_idx + self.search_radius + 1)
        win = win % n if self.loop else np.unique(np.clip(win, 0, n - 1))
        i = int(win[np.argmin(np.linalg.norm(pts[win] - xy, axis=1))])
        self.last_idx = i

        best = None
        for k in ((i - 1) % n if self.loop else i - 1, i):
            if k < 0 or self._next(k) == k:
                continue
            a, b = pts[k], pts[self._next(k)]
            seg = b - a
            L2 = float(seg @ seg)
            if L2 <= 0.0:
                continue
            t = float(np.clip((xy - a) @ seg / L2, 0.0, 1.0))
            c = a + t * seg
            d = float(np.linalg.norm(xy - c))
            if best is None or d < best[0]:
                best = (d, k, seg, c)
        if best is None:                              # 只有一個點 (不會發生, 建構時擋掉了)
            best = (float(np.linalg.norm(xy - pts[i])), i, pts[i] - pts[i - 1], pts[i].copy())
        dist, k, tangent, closest = best
        unit_t = tangent / np.linalg.norm(tangent)
        proj = float((xy - pts[k]) @ unit_t)          # 車在這一段上的位置 (m), 可以超出 [0, 段長]

        remaining = None
        finished = False
        if not self.loop:
            remaining = float(self.total - (self.s[k] + proj))
            finished = remaining <= self.goal_tolerance

        recovery = dist > self.far_threshold
        if recovery:
            targets, tangents = self._recovery_targets(xy, yaw, k, proj, dist)
        else:
            targets, tangents = self._lookahead_targets(k, proj)

        headings = np.arctan2(tangents[:, 1], tangents[:, 0])
        idx0 = k if self.forward else n - 1 - k
        return dict(targets=targets, headings=headings, idx=idx0, closest=closest,
                    dist=dist, proj=proj, recovery=recovery, finished=finished,
                    remaining=remaining)

    # ------------------------------------------------------------------ 目標點
    def _walk(self, start_idx, distances):
        """從 pts[start_idx] 沿路徑走 distances (遞增, m) 回傳每個距離的 (點, 切線)。
        不封閉的路徑走到底時剩下的全放在終點。"""
        pts = self.pts
        out_p, out_t = [], []
        cur, acc, k = start_idx, 0.0, 0
        m = len(distances)
        guard = 0
        while k < m:
            nxt = self._next(cur)
            if nxt == cur:                            # 終點
                tan = pts[cur] - pts[cur - 1]
                while k < m:
                    out_p.append(pts[cur].copy())
                    out_t.append(tan)
                    k += 1
                break
            pc, pn = pts[cur], pts[nxt]
            seg = pn - pc
            seg_len = float(np.linalg.norm(seg))
            while k < m and acc + seg_len >= distances[k]:
                r = (distances[k] - acc) / seg_len if seg_len > 0 else 0.0
                out_p.append(pc + seg * r)
                out_t.append(seg)
                k += 1
            acc += seg_len
            cur = nxt
            guard += 1
            if guard > 4 * len(pts) + 10:            # 不應該發生 (距離遠大於路徑長)
                while k < m:
                    out_p.append(pts[cur].copy())
                    out_t.append(seg)
                    k += 1
        return np.asarray(out_p), np.asarray(out_t)

    def _lookahead_targets(self, i, proj):
        d = self.spacing * np.arange(1, self.n_targets + 1) + proj
        return self._walk(i, d)

    def _recovery_targets(self, xy, yaw, i, proj, dist):
        p0 = xy
        fwd = np.array([math.cos(yaw), math.sin(yaw)])
        merge = proj + max(self.rec_min_merge, dist * self.rec_merge_gain)
        p3s, t3s = self._walk(i, [merge])
        p3, t3 = p3s[0], t3s[0]
        t3 = t3 / (np.linalg.norm(t3) + 1e-9)
        ctrl = dist * self.rec_ctrl_gain
        p1 = p0 + fwd * ctrl
        p2 = p3 - t3 * ctrl
        ts = (np.arange(self.n_targets) + 1.0) / self.n_targets
        us = 1.0 - ts
        pos = (us ** 3)[:, None] * p0 + (3 * us ** 2 * ts)[:, None] * p1 \
            + (3 * us * ts ** 2)[:, None] * p2 + (ts ** 3)[:, None] * p3
        tan = (3 * us ** 2)[:, None] * (p1 - p0) + (6 * us * ts)[:, None] * (p2 - p1) \
            + (3 * ts ** 2)[:, None] * (p3 - p2)
        bad = np.linalg.norm(tan, axis=1) < 1e-9
        tan[bad] = t3
        return pos, tan
