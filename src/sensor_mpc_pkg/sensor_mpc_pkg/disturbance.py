"""Offset-free MPC 的擾動估計 (純 numpy + 模型函式, 不需要 ROS)。

為什麼需要
----------
MPC 完全靠動態模型預測未來。車子如果有模型不知道的固定偏差 (例如每秒固定多往右轉一點), MPC 每一步
都以為再修一點就好, 誤差會停在一個穩態值。實體車的輪胎、地面、馬達一定跟訓練資料有差, 所以要讓
MPC 自己量出這個差。(2026-11-17 Isaac 上一度以為整路偏左 1.4 cm 是這個原因, 後來確認是 head 權重
太重; 模擬裡模型和車幾乎一樣, 估計器開或關沒差, 估到的差約 yaw rate -0.011 rad/s。)

怎麼估
------
每個週期都知道「上一週期的 window + 實際送出的指令」, 模型預測的下一步位姿差 delta_pred
(車體座標的前進 / 左側 / theta), 跟實際量到的 delta_meas 比:

    r = delta_meas * (dt / 實際經過的時間) - delta_pred     (只取位姿那 3 維)
    d <- d + (dt / tau) * (r - d)                          (一階低通, tau 預設 1 s)
    d  夾在 ±limit 裡

d 當成 acados 的參數, 在整個 horizon 裡加到模型預測的位姿差上 (mlp_control_pkg.utils 的
createAcadosSolver(disturbance=True))。速度那幾維不修: 模型的輸入還是量到的速度 / 輪速。

tau 是取捨: 太小 = 把定位雜訊當成擾動 (MPC 會抖), 太大 = 彎道的暫態跟不上。
"""
from __future__ import annotations

import numpy as np

from mlp_control_pkg.utils import NS, body_delta_np, build_window_np

N_DIST = 3


class DisturbanceEstimator:

    def __init__(self, model_func, dt, tau=1.0, limit=(0.01, 0.005, 0.005)):
        """model_func: CasADi Function (SEQ*NF,) -> (NS,) 未正規化的車體座標差 (loadModelFunc 的第一個回傳)。
        dt: MPC 週期 (s)。limit: 每一步 (前進 m, 左側 m, theta rad) 的上限。"""
        self.f = model_func
        self.dt = float(dt)
        self.tau = max(float(tau), self.dt)
        self.limit = np.asarray(limit, dtype=float)
        self.reset()

    def reset(self):
        self.d = np.zeros(N_DIST)
        self.last_resid = np.full(N_DIST, np.nan)
        self.n = 0

    def predict(self, states, controls):
        """states (SEQ, NS), controls (SEQ, NU) (最後一個 = 這一步送出的指令) -> 模型預測的差 (NS,)。"""
        win = build_window_np(np.asarray(states, float), np.asarray(controls, float))
        return np.asarray(self.f(win), dtype=float).reshape(-1)[:NS]

    def update(self, states, controls, new_state, elapsed):
        """上一週期的 window (states 結尾 = 那時的狀態, controls 結尾 = 那時送出的指令) +
        現在量到的狀態 -> 更新 d。elapsed = 兩次之間實際經過的時間 (s); 離 dt 太遠就不更新。"""
        if not (0.5 * self.dt <= elapsed <= 2.0 * self.dt):
            return self.d
        pred = self.predict(states, controls)
        meas = body_delta_np(np.asarray(states, float)[-1], np.asarray(new_state, float))
        meas[:N_DIST] *= self.dt / elapsed
        r = meas[:N_DIST] - pred[:N_DIST]
        if not np.all(np.isfinite(r)):
            return self.d
        self.last_resid = r
        self.d = self.d + (self.dt / self.tau) * (r - self.d)
        self.d = np.clip(self.d, -self.limit, self.limit)
        self.n += 1
        return self.d

    def per_second(self):
        """d 換成每秒: (前進 m/s, 左側 m/s, yaw rate rad/s), 給人看的。"""
        return self.d / self.dt

