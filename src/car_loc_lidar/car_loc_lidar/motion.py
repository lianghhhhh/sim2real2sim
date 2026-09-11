#!/usr/bin/env python3
"""等速運動模型 —— 只有雷射的時候, 「下一幀車子大概在哪」只能靠這個。

有 IMU 的做法在這裡是用陀螺儀積分給預測值; 沒有 IMU 就只剩「上一段的速度會
延續下去」這個假設。它在 50 ms 的尺度上夠用 (20 Hz 掃描), 但兩件事要小心:

* **急轉彎時預測會落後。** 所以配準的搜尋範圍不能太小, 而且 LM 的單步位移
  上限 (0.5 m) 是必要的護欄。
* **速度是從配準結果差分出來的**, 配準一爛速度就爛, 爛速度又讓運動補償更爛 ——
  這是一個會自我強化的迴圈。所以速度有低通、有上限, 而且配準失敗的那幾幀
  **不更新速度** (寧可維持舊值, 也不要餵一個假的進去)。
"""
from __future__ import annotations

import math

import numpy as np


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ConstVelMotion:

    def __init__(self, *, alpha: float = 0.5, v_max: float = 4.0,
                 omega_max: float = 25.0):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.t = None
        # 車體座標的速度 (ROS 慣例)
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        # 最後一次**配準成功**時的角速度。decay() 不會動它 ——
        # 判斷「這一幀的預測可能錯多少」要用這個, 不能用被 decay 過的 omega:
        # 配準一失敗 omega 就被縮小, 於是「需要掃多大範圍」跟著假性變小,
        # 掃描重試又被打開, 用一個蓋不住真值的範圍去找 -> 鎖到 180 度。
        self.omega_trusted = 0.0
        self.alpha = float(alpha)        # 速度低通: 1 = 完全信最新的差分
        self.v_max = float(v_max)
        self.omega_max = float(omega_max)

    # ------------------------------------------------------------------
    @property
    def pose(self):
        return np.array([self.x, self.y, self.theta])

    def set_pose(self, x, y, theta, t=None, reset_twist=True):
        self.x, self.y, self.theta = float(x), float(y), wrap_pi(float(theta))
        if t is not None:
            self.t = float(t)
        if reset_twist:
            self.vx = self.vy = self.omega = 0.0
            self.omega_trusted = 0.0

    def predict(self, t: float):
        """回傳 (x, y, theta) 在時刻 t 的預測。不改變內部狀態。"""
        if self.t is None:
            return self.x, self.y, self.theta
        dt = t - self.t
        if dt <= 0.0:
            return self.x, self.y, self.theta
        dt = min(dt, 0.5)                # 斷太久就不要一次外推一整段
        th = self.theta + self.omega * dt
        # 車體速度轉到世界座標, 用區間中點的朝向 (二階比用起點準)
        mid = self.theta + 0.5 * self.omega * dt
        c, s = math.cos(mid), math.sin(mid)
        return (self.x + (c * self.vx - s * self.vy) * dt,
                self.y + (s * self.vx + c * self.vy) * dt,
                wrap_pi(th))

    def update(self, t: float, x, y, theta):
        """配準成功之後叫這個: 落實位姿, 並用差分更新速度。"""
        if self.t is not None:
            dt = t - self.t
            if 1e-4 < dt < 0.5:
                dx, dy = x - self.x, y - self.y
                dth = wrap_pi(theta - self.theta)
                # 世界座標的位移轉回車體座標 (用區間中點的朝向)
                mid = self.theta + 0.5 * dth
                c, s = math.cos(mid), math.sin(mid)
                nvx = (c * dx + s * dy) / dt
                nvy = (-s * dx + c * dy) / dt
                nom = dth / dt
                a = self.alpha
                self.vx = (1 - a) * self.vx + a * nvx
                self.vy = (1 - a) * self.vy + a * nvy
                self.omega = (1 - a) * self.omega + a * nom
                self.clamp()
        self.x, self.y, self.theta = float(x), float(y), wrap_pi(float(theta))
        self.t = float(t)
        self.omega_trusted = self.omega

    def coast(self, t: float):
        """配準失敗時用: 照等速推過去, 但**不更新速度**。"""
        x, y, th = self.predict(t)
        self.x, self.y, self.theta = x, y, th
        self.t = float(t)

    def clamp(self):
        v = math.hypot(self.vx, self.vy)
        if v > self.v_max:
            self.vx *= self.v_max / v
            self.vy *= self.v_max / v
        self.omega = float(np.clip(self.omega, -self.omega_max, self.omega_max))

    def decay(self, k: float = 0.7):
        """連續配準失敗時把速度收掉 —— 與其拿一個過期的速度一直外推,
        不如承認「不知道車在動多快」, 讓位置停在原地等配準回來。"""
        self.vx *= k
        self.vy *= k
        self.omega *= k
