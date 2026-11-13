"""把 temp_curve 給的路徑點換成 MPC 每一個 stage 的參考點 (純 numpy, 不需要 ROS)。

temp_curve 給 10 個點, 從車子在路徑上的投影點往前 spacing (0.1 m) 開始、每 spacing 一個。
MPC 第 t 個 stage 是 t*dt 秒之後, 應該在路徑上走了 v_ref*t*dt 的地方 —— 直接拿第 t 個點
(間距 0.1 m / 0.05 s = 2 m/s) 會叫車子用 2 m/s 追, 所以沿弧長重新取樣。
"""
import numpy as np


def curvature_ahead(xy, ang, min_arc_length=0.05):
    """路徑前方的平均轉彎率 (rad/m)。xy (n, 2), ang (n,) 絕對航向 (rad)。"""
    if len(xy) < 2:
        return 0.0
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    dth = np.abs(np.angle(np.exp(1j * np.diff(ang))))
    ok = seg > 1e-6
    total = seg[ok].sum()
    return float(dth[ok].sum() / total) if total >= min_arc_length else 0.0


def build_reference(xy, ang, spacing, v_ref, dt, n_stage, car_xy=None, car_angle=0.0):
    """回傳 (n_stage, 4) 的 [x, y, sin(psi), cos(psi)], 第 t 列是弧長 v_ref*dt*t 的位置。

    xy (n, 2): 路徑點 (世界座標), 第一個點在投影點前方 spacing。ang (n,): 每個點的絕對航向。
    範圍外用頭尾那一段直線外插 (航向用端點的)。點不夠 (< 2) 時從車子位置沿 car_angle 直走。
    """
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    ang = np.unwrap(np.asarray(ang, dtype=float).reshape(-1))
    if len(xy) >= 2:
        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        keep = np.r_[True, seg > 1e-6]              # 去掉重複的點 (路徑尾端的補點)
        xy, ang = xy[keep], ang[keep]
    q = v_ref * dt * np.arange(n_stage)
    if len(xy) < 2:
        base = np.asarray(car_xy if car_xy is not None else (xy[0] if len(xy) else (0.0, 0.0)), float)
        psi = np.full(n_stage, car_angle)
        pts = base + q[:, None] * np.c_[np.cos(psi), np.sin(psi)]
        return np.c_[pts, np.sin(psi), np.cos(psi)]

    s = spacing + np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    k = np.clip(np.searchsorted(s, q, side='right') - 1, 0, len(s) - 2)
    r = (q - s[k]) / (s[k + 1] - s[k])
    pts = xy[k] + r[:, None] * (xy[k + 1] - xy[k])
    psi = ang[k] + np.clip(r, 0.0, 1.0) * (ang[k + 1] - ang[k])
    return np.c_[pts, np.sin(psi), np.cos(psi)]
