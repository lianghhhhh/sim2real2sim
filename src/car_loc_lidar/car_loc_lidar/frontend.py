#!/usr/bin/env python3
"""雷射訊息 -> 可以拿去配準的 2D 點集 (車體座標)。

兩個節點 (里程計、定位) 前面這一段是一樣的, 所以放在這裡:

    訊息 -> 解析 -> 濾無效回波 -> 濾高度 -> 套外參 (感測器->車體)
         -> 運動補償 -> 等間隔降採樣 -> (N,2)

高度過濾的假設要講清楚
----------------------
這個 package **不吃 IMU**, 所以不知道車子當下的俯仰角, 只能假設車身是水平的,
直接用感測器座標的 z 去切。平地上這個假設成立; 車子急煞俯仰、或地面有斜坡時,
切出來的高度帶會跟著歪, 地面回波可能混進來。真的要處理那個情境就得吃 IMU ——
那就是方法二刻意不做的事。
"""
from __future__ import annotations

import math

import numpy as np

from .scan import (deskew, laserscan_to_xyz, pointcloud2_to_xyz,
                   stride_subsample, valid_mask)


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]], dtype=np.float64)


class Frame:
    __slots__ = ('t', 'xy', 'frac', 'n_raw', 'n_kept')

    def __init__(self, t, xy, frac, n_raw, n_kept):
        self.t = float(t)
        self.xy = xy              # (N,2) 車體座標, 已補償
        self.frac = frac          # (N,) 每個點在這一圈裡的時間比例
        self.n_raw = int(n_raw)
        self.n_kept = int(n_kept)


class ScanFrontend:

    def __init__(self, *, input_type='scan',
                 translation=(0.0, 0.0, 0.20), rpy_deg=(0.0, 0.0, 0.0),
                 range_min=0.05, range_max=12.5,
                 z_min=0.15, z_max=0.90, max_points=1500,
                 scan_period=0.0, scan_stamp='end', do_deskew=True,
                 time_order='forward', time_model='linear',
                 scan_frames=6, seam_deg=0.0, lead_frames=0.5):
        self.input_type = input_type
        self.T = np.asarray(translation, dtype=np.float64).reshape(3)
        self.R = rpy_to_matrix(*np.radians(np.asarray(rpy_deg, dtype=np.float64)))
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.max_points = int(max_points)
        self.scan_stamp = scan_stamp
        self.do_deskew = bool(do_deskew)
        # 索引方向 vs 時間方向。Isaac 的 laser_scan 是**依方位角遞增**排序的,
        # 而 MS200 是 CW 旋轉 -> 索引順序跟發射順序相反, 要 'reverse'。
        # 猜錯的症狀: 直線走正常, 一轉彎殘差就變兩倍。見 README 第 5 節 ——
        # 那裡記了一次「照殘差投票改成 forward 結果把地圖建爛」的教訓。
        self.time_order = time_order
        # 每個點是什麼時候打的。
        #   'linear'  真的旋轉雷射: 一圈裡時間跟著方位角**連續**走 (實體 MS200)。
        #   'frames'  Isaac 的 RTX 雷射: 一圈是 `scan_frames` 個算繪幀拼起來的,
        #             **同一幀裡的點是同一個瞬間**, 幀與幀之間差一個算繪週期 ——
        #             時間對方位角是階梯, 不是斜線。而且最新那一幀從 `seam_deg`
        #             開始, 不是從 ±180 度 (陣列的頭尾); 訊息的時戳也不一定是
        #             最新那一幀, `lead_frames` = 最新一幀比時戳晚幾幀。
        # 2026-10-13 用 ground truth 逐扇區量 (kros_car.usd, 10 Hz / 60 fps,
        # 1.5 rad/s 自旋), 每個扇區相對於時戳的時間:
        #     [+60,+120) 0 ms  [+120,180) -16.7  [-180,-120) -33.3
        #     [-120,-60) -50   [-60,0) -66.7
        #     [0,+60)  **+16.7** (一般的 Isaac app: seam 0, lead 1)
        #              **-83.3** (scripts/isaac_headless.py: seam 60, lead 0)
        #   對牆面的中位殘差: linear reverse/end 9.97 cm -> frames 1.54 cm (= 靜止時)。
        # !! **seam / lead 不是固定的**: 一般的 Isaac app 裡每次按 Play 都可能不一樣
        #    (同一個 app Stop/Play 四次量到 seam 0 lead 1 兩次、seam 0 lead 0 兩次;
        #    使用者實跑的兩輪看起來又是別的接縫)。所以:
        #    * seam (哪個扇區是最新的) 決定掃描的**形狀**, 填錯一格那 1/6 的點差
        #      100 ms, 1.5 rad/s 自旋差 4 cm -> lidar_loc_node 開機後自己用配準殘差
        #      投票決定 (auto_scan_seam)。
        #    * lead 是**整圈一起**早或晚一幀, 不改變形狀, 只讓位姿的時戳差 17 ms
        #      (0.5 m/s 是 0.8 cm) —— 從掃描本身看不出來, 所以預設取 0.5 (不管實際
        #      是 0 還是 1, 最多差半幀 8 ms)。
        # 用 scripts/check_scan_timing.py 可以對 ground truth 直接量這一次 Play 的值。
        self.time_model = time_model
        self.scan_frames = max(int(scan_frames), 1)
        self.seam_deg = float(seam_deg)
        self.lead_frames = float(lead_frames)

        self._period_fixed = float(scan_period)
        self.period = float(scan_period) if scan_period > 0 else 0.05
        self._last_t = None
        self._dts = []

    # ------------------------------------------------------------------
    def _update_period(self, t: float):
        """一圈要多久 —— 從連續兩則訊息的時間差自己量, 不要寫死。"""
        if self._period_fixed > 0:
            return
        if self._last_t is not None:
            dt = t - self._last_t
            if 0.005 < dt < 1.0:
                self._dts.append(dt)
                if len(self._dts) > 40:
                    del self._dts[:20]
                self.period = float(np.median(self._dts))
        self._last_t = t

    # ------------------------------------------------------------------
    def process(self, msg, t: float, vx=0.0, vy=0.0, omega=0.0,
                stamp_at=None, time_order=None, seam_deg=None) -> Frame:
        self._update_period(t)

        if self.input_type == 'scan':
            xyz = laserscan_to_xyz(msg)
        else:
            xyz = pointcloud2_to_xyz(msg)
        n_raw = xyz.shape[0]
        if n_raw == 0:
            return Frame(t, np.empty((0, 2)), np.empty(0), 0, 0)

        # 索引對得上時間 —— 一定要先算出比例再過濾, 不能邊走邊刪
        order = time_order or self.time_order
        frames = self.time_model == 'frames'
        if frames:
            # 從最新那一幀的起點沿著**時間倒退**的方向數過去是第幾幀。CW ('reverse')
            # 的雷射方位角隨時間遞減, 所以愈舊的幀方位角愈大。無效回波是 (0, 0),
            # 算出來的方位角沒有意義, 但下面的 valid_mask 會把它們濾掉。
            az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
            seam = self.seam_deg if seam_deg is None else float(seam_deg)
            d = (az - seam) if order == 'reverse' else (seam - az)
            k = np.floor((d % 360.0) / (360.0 / self.scan_frames))
            # 1.0 = 訊息時戳那一刻 (搭配 deskew 的 'end'); 最新那一幀比時戳晚
            # lead_frames 幀, 所以它的 frac 會超過 1 —— deskew 算的是時間差, 沒關係。
            frac = 1.0 + (self.lead_frames - k) / self.scan_frames
        else:
            frac = np.arange(n_raw, dtype=np.float64) / max(n_raw - 1, 1)
            if order == 'reverse':
                frac = 1.0 - frac
        keep = valid_mask(xyz, self.range_min, self.range_max)
        if not keep.any():
            return Frame(t, np.empty((0, 2)), np.empty(0), n_raw, 0)

        # 感測器 -> 車體
        pts = xyz[keep] @ self.R.T + self.T
        frac = frac[keep]

        if self.input_type != 'scan':
            # 2D 雷射本來就在一個平面上, 沒有高度可以濾
            band = (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max)
            pts = pts[band]
            frac = frac[band]
        if pts.shape[0] == 0:
            return Frame(t, np.empty((0, 2)), np.empty(0), n_raw, 0)

        sel = stride_subsample(pts.shape[0], self.max_points)
        xy = pts[sel, :2]
        frac = frac[sel]

        if self.do_deskew:
            # frames 模型的時間是直接對時戳算的, 沒有頭/中/尾可以選
            xy = deskew(xy, frac, vx, vy, omega, self.period,
                        'end' if frames else (stamp_at or self.scan_stamp))
        return Frame(t, xy, frac, n_raw, xy.shape[0])
