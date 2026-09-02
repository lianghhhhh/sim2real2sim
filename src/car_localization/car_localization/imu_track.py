#!/usr/bin/env python3
"""IMU 的時間序列緩衝 —— 給運動補償 (deskew) 查「那一瞬間車子朝哪」用。

Isaac 的 IsaacReadIMU 會輸出 orientation, 那是模擬器直接給的車體世界姿態,
在模擬裡是精確值 (沒有積分漂移、沒有磁力計問題)。這個 package 預設就吃它,
所以 yaw 這個自由度可以直接鎖死, 雷射只需要解 x/y 兩個自由度。

真車上沒有這種東西 —— 6 軸 IMU 的 yaw 一定會漂。把 yaw_source 換成 'gyro'
就會改成「陀螺儀積分 + 由掃描比對修正 yaw」的模式, 那條路才是真車能用的,
但精度會比模擬裡差一截。這個差距是真的, 不是設定調得不夠好。
"""
from __future__ import annotations

import math

import numpy as np


def quat_to_matrix(q) -> np.ndarray:
    """ROS 順序 (x, y, z, w) -> 3x3 旋轉矩陣。"""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def quat_to_yaw(q) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def matrix_to_quat(R) -> np.ndarray:
    """3x3 -> (x, y, z, w)。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ImuTrack:
    """固定長度的環狀緩衝, 可以在任意時刻內插出姿態與角速度。"""

    def __init__(self, capacity: int = 4000, gyro_bias_window: float = 1.0,
                 still_gyro: float = 0.02, still_acc: float = 0.20):
        self.cap = int(capacity)
        self.ts = np.zeros(self.cap)
        self.q = np.zeros((self.cap, 4))
        self.g = np.zeros((self.cap, 3))
        self.a = np.zeros((self.cap, 3))
        self.n = 0
        self.head = 0
        self.gyro_bias = np.zeros(3)
        self._bias_win = float(gyro_bias_window)
        self._still_gyro = float(still_gyro)
        self._still_acc = float(still_acc)
        self._bias_done = False

    # ------------------------------------------------------------------
    def add(self, t: float, quat, gyro, accel):
        i = self.head
        self.ts[i] = t
        self.q[i] = quat
        self.g[i] = gyro
        self.a[i] = accel
        self.head = (i + 1) % self.cap
        self.n = min(self.n + 1, self.cap)
        if not self._bias_done:
            self._try_bias()

    def _ordered(self):
        """回傳 (依時間排好的索引, 對應的時戳)。索引一定是陣列 —— 不要回 slice,
        後面會拿它做 idx[j] 這種取值。"""
        if self.n < self.cap:
            idx = np.arange(self.n)
        else:
            idx = np.concatenate([np.arange(self.head, self.cap),
                                  np.arange(0, self.head)])
        return idx, self.ts[idx]

    def _try_bias(self):
        """車子還沒動的時候把陀螺儀零偏量掉。Isaac 的 IMU 本來就沒有零偏,
        這一步是為了讓同一份程式搬到真車上也不用改。"""
        idx, ts = self._ordered()
        if ts.size < 20 or ts[-1] - ts[0] < self._bias_win:
            return
        g = self.g[idx]
        a = self.a[idx]
        still = (np.linalg.norm(g, axis=1) < self._still_gyro) & \
                (np.abs(np.linalg.norm(a, axis=1) - np.linalg.norm(a[0])) < self._still_acc)
        if still.mean() > 0.9:
            self.gyro_bias = g[still].mean(axis=0)
        self._bias_done = True

    # ------------------------------------------------------------------
    @property
    def ready(self) -> bool:
        return self.n >= 2

    @property
    def latest_time(self) -> float:
        if self.n == 0:
            return float('-inf')
        return float(self.ts[(self.head - 1) % self.cap])

    @property
    def earliest_time(self) -> float:
        if self.n == 0:
            return float('inf')
        idx, ts = self._ordered()
        return float(ts[0])

    def latest(self):
        i = (self.head - 1) % self.cap
        return float(self.ts[i]), self.q[i].copy(), self.g[i].copy(), self.a[i].copy()

    def _locate(self, t: float):
        idx, ts = self._ordered()
        j = int(np.searchsorted(ts, t))
        if j <= 0:
            return idx[0], idx[0], 0.0
        if j >= ts.size:
            return idx[-1], idx[-1], 0.0
        i0, i1 = idx[j - 1], idx[j]
        dt = ts[j] - ts[j - 1]
        k = 0.0 if dt <= 0 else (t - ts[j - 1]) / dt
        return i0, i1, float(np.clip(k, 0.0, 1.0))

    def quat_at(self, t: float) -> np.ndarray:
        """姿態內插。用 nlerp: 60 Hz 之間的角度差很小, 跟 slerp 的差別
        遠小於量測本身的誤差, 但便宜很多。"""
        if self.n == 0:
            return np.array([0.0, 0.0, 0.0, 1.0])
        i0, i1, k = self._locate(t)
        q0, q1 = self.q[i0], self.q[i1]
        if float(q0 @ q1) < 0:
            q1 = -q1
        q = (1 - k) * q0 + k * q1
        nrm = np.linalg.norm(q)
        return q / nrm if nrm > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])

    def gyro_at(self, t: float) -> np.ndarray:
        if self.n == 0:
            return np.zeros(3)
        i0, i1, k = self._locate(t)
        return (1 - k) * self.g[i0] + k * self.g[i1] - self.gyro_bias

    def is_still(self, t: float, window: float = 0.15) -> bool:
        idx, ts = self._ordered()
        sel = (ts >= t - window) & (ts <= t + window)
        if sel.sum() < 3:
            return False
        g = self.g[idx][sel] - self.gyro_bias
        return bool(np.linalg.norm(g, axis=1).max() < self._still_gyro)
