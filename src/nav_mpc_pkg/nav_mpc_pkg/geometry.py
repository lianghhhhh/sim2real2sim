"""純 numpy 的幾何 (不需要 ROS; test/test_geometry.py 測)。

* resample_polyline: Nav2 給的路徑點間距是 costmap 的解析度 (0.025~0.05 m) 而且不均勻,
  重新沿弧長取成固定間距再交給 path_tracker
* PathProgress: 車子在路徑上走到哪 (弧長 s)。只在上一次附近往前找, 路徑繞回來經過同一個地方時
  不會跳到另一段
* scan_to_points / to_world: LaserScan -> 雷射座標的點 -> 車體座標 / map 座標
* corridor_hits: 「前方的路」上有沒有東西 —— 從車子目前的投影點沿路徑往前 check_distance 公尺,
  兩側各 half_width 的走廊, 落在裡面的掃描點就是擋路的障礙物
"""
from __future__ import annotations

import math

import numpy as np


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cumulative_s(pts):
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return np.zeros(len(pts))
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]


def resample_polyline(pts, spacing):
    """(n, 2) 折線 -> 沿弧長每 spacing 公尺一點 (含起點與終點)。重複點會先去掉。"""
    pts = np.asarray(pts, dtype=float)[:, :2]
    keep = np.r_[True, np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-6]
    pts = pts[keep]
    if len(pts) < 2:
        return pts.copy()
    s = cumulative_s(pts)
    total = float(s[-1])
    n = max(1, int(math.ceil(total / spacing - 1e-9)))
    ss = np.linspace(0.0, total, n + 1)
    return np.column_stack([np.interp(ss, s, pts[:, 0]), np.interp(ss, s, pts[:, 1])])


def _project_segments(xy, a, b):
    """點 xy (M, 2) 到線段 a->b (K, 2) 的 (距離 (M, K), 未夾的 t (M, K), 線段長 (K,))。"""
    seg = b - a                                            # (K, 2)
    L2 = np.einsum('ij,ij->i', seg, seg)
    L2 = np.where(L2 > 0.0, L2, 1e-12)
    rel = xy[:, None, :] - a[None, :, :]                   # (M, K, 2)
    t_raw = np.einsum('mkj,kj->mk', rel, seg) / L2
    t = np.clip(t_raw, 0.0, 1.0)
    closest = a[None, :, :] + t[..., None] * seg[None, :, :]
    d = np.linalg.norm(xy[:, None, :] - closest, axis=2)
    return d, t_raw, np.sqrt(L2)


class PathProgress:
    """車子沿一條不封閉路徑的進度。update(xy) -> (s, 離路徑的距離)。

    只在上一次所在線段的 [-back, +ahead] 個線段裡找最近的, 所以路徑自己交叉或繞回來時
    不會跳到另一段 (跟 sensor_mpc_pkg 的 path_tracker 一樣的想法)。
    """

    def __init__(self, pts, back=5, ahead=40):
        self.pts = np.asarray(pts, dtype=float)[:, :2]
        if len(self.pts) < 2:
            raise ValueError('路徑至少要 2 個點')
        self.s = cumulative_s(self.pts)
        self.total = float(self.s[-1])
        self.back = int(back)
        self.ahead = int(ahead)
        self.k = None             # 上一次所在的線段

    def update(self, xy):
        xy = np.asarray(xy, dtype=float).reshape(1, 2)
        nseg = len(self.pts) - 1
        if self.k is None:
            lo, hi = 0, nseg
        else:
            lo, hi = max(0, self.k - self.back), min(nseg, self.k + self.ahead + 1)
        a, b = self.pts[lo:hi], self.pts[lo + 1:hi + 1]
        d, t_raw, L = _project_segments(xy, a, b)
        j = int(np.argmin(d[0]))
        self.k = lo + j
        t = float(np.clip(t_raw[0, j], 0.0, 1.0))
        return float(self.s[self.k] + t * L[j]), float(d[0, j])

    def sub_polyline(self, s0, s1):
        """弧長 [s0, s1] 那一段折線 (含兩端的內插點)。"""
        s0 = float(np.clip(s0, 0.0, self.total))
        s1 = float(np.clip(s1, s0, self.total))
        inner = (self.s > s0) & (self.s < s1)
        ss = np.r_[s0, self.s[inner], s1]
        return np.column_stack([np.interp(ss, self.s, self.pts[:, 0]),
                                np.interp(ss, self.s, self.pts[:, 1])])


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """LaserScan -> 雷射座標的 (M, 2) 點。inf / nan / 超出範圍的丟掉。"""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, ang = r[ok], ang[ok]
    return np.column_stack([r * np.cos(ang), r * np.sin(ang)])


def to_world(pts, x, y, yaw):
    """(M, 2) 局部座標 -> 世界座標 (平移 x, y, 旋轉 yaw)。"""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    return np.column_stack([x + c * pts[:, 0] - s * pts[:, 1],
                            y + s * pts[:, 0] + c * pts[:, 1]])


def outside_box(pts, box):
    """box = [x_min, x_max, y_min, y_max] (車體座標)。回傳不在框裡的點的 mask。"""
    if box is None:
        return np.ones(len(pts), dtype=bool)
    x0, x1, y0, y1 = box
    inside = (pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1)
    return ~inside


def corridor_hits(points, poly, half_width):
    """points (M, 2) 世界座標, poly (K+1, 2) 前方的那段路 (第 0 點 = 車子在路徑上的投影)。

    回傳 (mask (M,), along (M,)): mask = 離 poly 不到 half_width, 而且不在起點後面
    (投影到第一段線段的 t < 0 = 車子旁邊或後面的東西, 不擋路); along = 那個點投影在 poly 上的
    弧長 (從車子算起, m), 不在走廊裡的是 inf。
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    poly = np.asarray(poly, dtype=float).reshape(-1, 2)
    M = len(points)
    if M == 0 or len(poly) < 2:
        return np.zeros(M, dtype=bool), np.full(M, np.inf)
    a, b = poly[:-1], poly[1:]
    d, t_raw, L = _project_segments(points, a, b)
    j = np.argmin(d, axis=1)
    dmin = d[np.arange(M), j]
    t = np.clip(t_raw[np.arange(M), j], 0.0, 1.0)
    s_seg = np.r_[0.0, np.cumsum(L)]
    along = s_seg[j] + t * L[j]
    behind = (j == 0) & (t_raw[np.arange(M), 0] < 0.0)
    mask = (dmin < half_width) & ~behind
    return mask, np.where(mask, along, np.inf)
