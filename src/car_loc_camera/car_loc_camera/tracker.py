#!/usr/bin/env python3
"""等速模型卡爾曼濾波 —— 把「一連串位置量測」變成「位姿 + 速度 + 朝向」。

為什麼相機那條路需要濾波器
--------------------------
單張影像給的是**一個點**: 車體中心的 (x, y)。它沒有速度、沒有朝向, 而且
每一幀都獨立地帶著 ~7 cm 的校正殘差 (跟偵測無關, 是投影模型本身的)。
直接把它當定位輸出有三個問題, 濾波器把三個一起解掉:

* **抖。** 逐幀獨立的 7 cm 誤差是白雜訊, 等速模型會把它平均掉;
  代價是對真的加速度反應慢一點 (由 `accel_sigma` 調)。
* **會斷。** 車開到柱子後面、YOLO 漏掉一幀, 輸出就整個消失。
  濾波器在沒有量測的時候照樣推, 只是共變異數會長大。
* **沒有朝向。** yaw 從速度方向來 —— 這是唯一不需要另一顆感測器的來源。

狀態: [x, y, vx, vy] (世界座標)。

離群值防線
----------
每個量測進來先算 NIS (normalized innovation squared): 「這個量測跟預測差幾個
sigma」。誤判到影子上通常會差好幾公尺, NIS 破表, 直接丟掉。但**連續被擋太多次
就強制接受** —— 不然車子真的被搬走 (或濾波器自己發散) 之後永遠回不來。
"""
from __future__ import annotations

import math

import numpy as np

CHI2_99_2DOF = 9.210


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ConstVelTracker:

    def __init__(self, *, accel_sigma: float = 2.0, meas_sigma: float = 0.075,
                 v_max: float = 4.0, chi2: float = CHI2_99_2DOF,
                 force_accept_after: int = 8,
                 yaw_min_speed: float = 0.15, yaw_init_speed: float = 0.30,
                 yaw_slew: float = 12.0, allow_reverse: bool = False):
        self.x = np.zeros(4)
        self.P = np.diag([1e3, 1e3, 4.0, 4.0])
        self.t = None
        # 未建模的加速度 (m/s^2) —— 過程雜訊。**這是唯一真的需要調的參數,**
        # 而且規則很簡單: 設成車子的**峰值加速度**。
        # 離線實測 (test/test_tracker.py, 量測雜訊 7.26 cm, 20 Hz):
        #   峰值加速度 0.29 m/s^2 -> sa=1 最好 (4.63 cm), sa=8 是 6.70 cm
        #   峰值加速度 1.66 m/s^2 -> sa=3 最好 (6.10 cm), sa=1 是 12.35 cm
        # 設太小比設太大危險得多: sa=1 在高機動下不只誤差翻倍, 連卡方閘門都會
        # 開始擋掉**正確的**量測 (800 幀擋掉 91 幀), 那是濾波器過度自信的典型症狀。
        # 預設 2.0 是三種機動程度下都不會出事的折衷。
        self.sa = float(accel_sigma)
        self.sm = float(meas_sigma)       # 量測雜訊 (m), 預設就是校正殘差
        self.v_max = float(v_max)
        self.chi2 = float(chi2)
        self.force_after = int(force_accept_after)

        self.yaw = 0.0
        self.has_yaw = False
        self.yaw_min_speed = float(yaw_min_speed)
        # 第一次認定 yaw 的門檻要比維持的門檻高: 剛開始速度估計還很粗,
        # 在 0.15 m/s 上就定案有機會直接定到反方向, 而 allow_reverse 之後
        # 只會忠實地把那個錯誤維持下去。
        self.yaw_init_speed = float(max(yaw_init_speed, yaw_min_speed))
        self.yaw_slew = float(yaw_slew)   # yaw 每秒最多轉多少 rad
        self.allow_reverse = bool(allow_reverse)

        self.initialized = False
        self.rejected_streak = 0
        self.n_accepted = 0
        self.n_rejected = 0
        self.last_meas_t = None
        self.last_nis = 0.0

    # ------------------------------------------------------------------
    @property
    def pos(self):
        return self.x[:2].copy()

    @property
    def vel(self):
        return self.x[2:].copy()

    @property
    def speed(self) -> float:
        return float(math.hypot(self.x[2], self.x[3]))

    def age(self, t: float) -> float:
        """距離上一個被採信的量測過了多久 (s)。"""
        return float('inf') if self.last_meas_t is None else t - self.last_meas_t

    def reset(self, t: float, x: float, y: float):
        self.x = np.array([x, y, 0.0, 0.0])
        # 速度只能填 0, 但車子可能正在跑 —— 所以速度的不確定度一定要給大,
        # 不然第一批量測會被閘門擋掉, 速度永遠學不到。
        self.P = np.diag([self.sm ** 2, self.sm ** 2, 4.0, 4.0])
        self.t = t
        self.last_meas_t = t
        self.initialized = True
        self.rejected_streak = 0

    # ------------------------------------------------------------------
    def predict(self, t: float):
        if not self.initialized:
            return
        dt = t - self.t
        if dt <= 0:
            return
        dt = min(dt, 1.0)                 # 斷線太久就不要一次推一整段
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        # 等速模型的標準過程雜訊 (未建模加速度當白雜訊積出來的)
        q = self.sa ** 2
        d2, d3, d4 = dt * dt, dt ** 3, dt ** 4
        Q = q * np.array([[d4 / 4, 0, d3 / 2, 0],
                          [0, d4 / 4, 0, d3 / 2],
                          [d3 / 2, 0, d2, 0],
                          [0, d3 / 2, 0, d2]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        # 速度硬上限: 濾波器發散時第一個爆掉的就是速度, 夾住它可以讓
        # 「暫時看不到車」不會演變成「位置飛到場外」。
        s = self.speed
        if s > self.v_max:
            self.x[2:] *= self.v_max / s
        self.t = t
        self._update_yaw(dt)

    def peek(self, t: float):
        """把狀態推到 t, 但**不改動濾波器**。回傳 (x, P)。

        給「下游想知道車現在在哪」用的。節點發出去的 /camera_loc/odom 是
        **影像時刻**的狀態 (時戳也是影像時刻), 那對依時戳內插的融合節點才是
        對的; 但拿最新一則當「現在」的下游 (TF、nav) 收到的就是 delay + 半個
        影像週期 (實測中位數 100 ms) 之前的位置。用等速模型往前推可以補回來:
        離線實測 (scripts/replay_camera_csv.py 的資料, 50 Hz 查詢) ——
          不外推: RMS 7.25 cm, spd>1.5 時 24.98 cm
          外推:   RMS 2.65 cm, spd>1.5 時  8.95 cm
        代價是靜止時略差 (0.55 -> 0.63 cm, 推的是速度雜訊), 以及急加速時
        等速模型會衝過頭 (|a|>3 m/s^2: 17.48 -> 7.49 cm, 還是賺很多)。
        P 也一起長大, 所以下游看共變異數就知道這是推出來的。
        """
        if not self.initialized:
            return None, None
        dt = t - self.t
        if dt <= 0:
            return self.x.copy(), self.P.copy()
        dt = min(dt, 1.0)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.sa ** 2
        d2, d3, d4 = dt * dt, dt ** 3, dt ** 4
        Q = q * np.array([[d4 / 4, 0, d3 / 2, 0],
                          [0, d4 / 4, 0, d3 / 2],
                          [d3 / 2, 0, d2, 0],
                          [0, d3 / 2, 0, d2]])
        x = F @ self.x
        s = float(np.hypot(x[2], x[3]))
        if s > self.v_max:
            x[2:] *= self.v_max / s
        return x, F @ self.P @ F.T + Q

    def update(self, t: float, z, sigma: float = None) -> bool:
        """吃一個位置量測。回傳有沒有被採信。"""
        z = np.asarray(z, dtype=np.float64).reshape(2)
        if not self.initialized:
            self.reset(t, z[0], z[1])
            self.n_accepted += 1
            return True
        self.predict(t)

        s = float(sigma if sigma is not None else self.sm)
        R = np.eye(2) * s * s
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        y = z - self.x[:2]
        S = H @ self.P @ H.T + R
        try:
            Si = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False
        nis = float(y @ Si @ y)
        self.last_nis = nis

        if nis > self.chi2:
            self.rejected_streak += 1
            self.n_rejected += 1
            # 逃生門: 連續擋掉這麼多次就代表「錯的是濾波器, 不是量測」
            # (車被搬走 / 濾波器發散)。整個重設到最新的量測上。
            if self.rejected_streak >= self.force_after:
                self.reset(t, z[0], z[1])
                return True
            return False

        K = self.P @ H.T @ Si
        self.x = self.x + K @ y
        I_KH = np.eye(4) - K @ H
        # Joseph form: 數值上對稱且保持正定, 長時間跑不會慢慢變成非對稱矩陣
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.rejected_streak = 0
        self.n_accepted += 1
        self.last_meas_t = t
        return True

    # ------------------------------------------------------------------
    def _update_yaw(self, dt: float):
        """朝向來自速度方向 —— 相機看得到的唯一朝向線索。

        兩個一定要處理的細節:

        * **車速太低時速度方向沒有意義** (7 cm 的量測雜訊除以 dt 是好幾 m/s 的
          假速度方向)。低於 `yaw_min_speed` 就維持上一個 yaw, 不要跟著轉。
        * **倒車時速度方向跟車頭差 180 度**, 而單一個 bbox 中心**沒有任何資訊**
          可以分辨這兩者。預設 (`allow_reverse=False`) 就直接把移動方向當車頭 ——
          誠實但倒車時會差 180 度。開 `allow_reverse` 是改用「車身不會瞬間翻面」
          的連續性去猜, 倒車就對了, 但代價是**萬一第一次定案定反了, 之後會一路
          錯下去** (連續性只會忠實地維持那個錯誤)。要真的解決得換成 YOLO 的
          OBB 定向框模型, 那需要重新標注資料。
        """
        v = self.x[2:]
        sp = float(np.hypot(v[0], v[1]))
        if sp < self.yaw_min_speed:
            return
        h = math.atan2(v[1], v[0])
        if not self.has_yaw:
            if sp < self.yaw_init_speed:
                return
            self.yaw = h
            self.has_yaw = True
            return
        if self.allow_reverse and abs(wrap_pi(h - self.yaw)) > math.pi / 2:
            h = wrap_pi(h + math.pi)
        # 轉向速率上限: 擋掉單幀量測雜訊造成的 yaw 亂跳
        d = wrap_pi(h - self.yaw)
        lim = self.yaw_slew * max(dt, 1e-3)
        self.yaw = wrap_pi(self.yaw + (d if abs(d) <= lim else math.copysign(lim, d)))

    # ------------------------------------------------------------------
    def pose_cov(self) -> np.ndarray:
        """6x6 的 ROS 位姿共變異數 (x, y, z, roll, pitch, yaw)。"""
        c = np.zeros((6, 6))
        c[0, 0] = self.P[0, 0]
        c[0, 1] = c[1, 0] = self.P[0, 1]
        c[1, 1] = self.P[1, 1]
        c[2, 2] = 1e-6
        c[3, 3] = c[4, 4] = 1e-6
        # yaw 是從速度方向推的, 不確定度隨速度變: 車越慢, 方向越不可信
        s = max(self.speed, 1e-3)
        sv = math.sqrt(max(self.P[2, 2] + self.P[3, 3], 1e-9))
        c[5, 5] = min((sv / s) ** 2, (math.pi / 2) ** 2) if self.has_yaw else 1e3
        return c

    def twist_cov(self) -> np.ndarray:
        c = np.zeros((6, 6))
        c[0, 0] = self.P[2, 2]
        c[0, 1] = c[1, 0] = self.P[2, 3]
        c[1, 1] = self.P[3, 3]
        c[2, 2] = c[3, 3] = c[4, 4] = c[5, 5] = 1e-3
        return c
