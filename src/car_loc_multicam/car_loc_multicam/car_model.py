#!/usr/bin/env python3
"""方法 B: 用車的 3D 外形反推位置。

這個模組跟 ROS 無關。

    給定 (x, y, yaw) -> 把車的 3D 外形投影到某台相機 -> 外接框 [u1, v1, u2, v2]

YOLO 給的也是外接框, 所以「找一個 (x, y) 讓預測的框跟 YOLO 的框最吻合」就是一個
小小的非線性最小平方 (`solve_position`)。融合節點則把同一個預測式當成 EKF 的量測
模型 (見 multicam_ekf.py), 這裡也順便給出它的 Jacobian。

為什麼不再用「框的中心」
------------------------
斜拍時框的中心**不是**車的任何一個固定點: 車頂比車底離相機近, 投影就往畫面上方
偏; 偏多少跟距離、相機俯角、車的朝向都有關。car_loc_camera 那種「框中心 -> 單應性」
在正下方拍時誤差 ~1 cm, 斜拍 20 度、5 m 外就是十幾公分, 而且方向跟著相機走 ——
8 台相機各偏各的, 平均起來也不會變成 0。

外形用什麼表示
--------------
**凸包的頂點** (`config/car_model.yaml`, 由 scripts/export_from_usd.py 從 USD 的
visual mesh 算出來)。任何形狀投影之後的外接框 = 它所有頂點投影的 min/max, 而凸包
外面那一圈頂點就足夠決定它 —— 所以這不是近似成長方體, 是 USD 裡那台車真正的外形
(含手臂、輪子、LiDAR)。

外接框的四個邊各由一個頂點決定 (最左、最上、最右、最下那個), Jacobian 就是那四個點
的投影導數。車子轉動時「哪一個點是最左」會換人, 所以這個量測函數是分段平滑的 ——
EKF 每一次更新都重新挑, 不影響。

不在模型裡的東西 (見 README「已知的限制」)
-------------------------------------------
* 手臂的姿勢 —— 凸包要在**按下 Play 之後**匯出 (手臂會被關節驅動到別的姿勢),
  手臂動了就要重新匯出。
* 車身俯仰 / 側傾 —— 假設地板是平的, base_link 永遠在 `base_z` 高度。
* YOLO 框跟真正外接框的系統性差 (標注習慣) —— 兩種:
    - YOLO 不框的細長零件 (夾爪尖端、車頂 LiDAR) -> 匯出時就排除 (--exclude)
    - 剩下每個邊固定差幾 px (Isaac 實測: 上緣 YOLO 低 ~3 px, 下緣高 ~1 px)
      -> `edge_bias_px`, 由 scripts/fit_car_model.py 量
"""
from __future__ import annotations

import math

import numpy as np

from .camera_model import Z_MIN


class CarModel:

    def __init__(self, points, base_z: float = 0.0, edge_bias_px=(0.0, 0.0, 0.0, 0.0)):
        # 外形頂點, base_link 座標 (m), (N, 3)
        self.points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        # base_link 原點離地面多高 (m)。kros_car 的輪子底在 base_link z = -0.042,
        # 所以車停在地上時 base_link 在 +0.042。
        self.base_z = float(base_z)
        # YOLO 的框 - 外形的外接框, 每個邊 [u1, v1, u2, v2] 的平均差 (px)。
        # 加在預測框上, 讓預測的是「YOLO 會框成什麼樣子」而不是幾何上的外接框。
        self.edge_bias = np.asarray(edge_bias_px, dtype=np.float64).reshape(4)
        lo, hi = self.points.min(axis=0), self.points.max(axis=0)
        self.size = hi - lo
        self.center_body = 0.5 * (lo + hi)

    @classmethod
    def from_yaml(cls, path: str) -> 'CarModel':
        import yaml
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        if 'points' in cfg:
            pts = cfg['points']
        else:
            # 沒有匯出凸包時的備案: 一個長方體 (8 個角)
            b = cfg['box']
            lo, hi = b['min'], b['max']
            pts = [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                   for z in (lo[2], hi[2])]
        # 整個外形平移 (YOLO 的框跟凸包差一個固定量時用, 見 car_model.yaml)
        pts = (np.asarray(pts, dtype=np.float64).reshape(-1, 3)
               + np.asarray(cfg.get('points_shift', [0.0, 0.0, 0.0]), dtype=np.float64))
        return cls(pts, base_z=cfg.get('base_z', 0.0),
                   edge_bias_px=cfg.get('edge_bias_px', [0.0, 0.0, 0.0, 0.0]))

    # ------------------------------------------------------------------
    def world_points(self, x, y, yaw):
        """外形頂點的世界座標 (N, 3), 以及它們對 yaw 的導數 (N, 3)。"""
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by, bz = self.points[:, 0], self.points[:, 1], self.points[:, 2]
        Pw = np.stack([x + c * bx - s * by, y + s * bx + c * by,
                       self.base_z + bz], axis=1)
        dyaw = np.stack([-s * bx - c * by, c * bx - s * by, np.zeros_like(bx)], axis=1)
        return Pw, dyaw

    def predict_bbox(self, cam, x, y, yaw, jac: bool = True):
        """預測的外接框 [u1, v1, u2, v2] 與 d框/d(x, y, yaw) (4, 3)。

        有任何一個頂點在相機後面 (或太近) 就回 (None, None) —— 那種情況下外接框
        沒有定義 (投影會翻到另一邊), 這台相機這一次就不用。
        """
        Pw, dyaw = self.world_points(x, y, yaw)
        Pc = cam.to_cam(Pw)
        if np.any(Pc[:, 2] < Z_MIN):
            return None, None
        uv = cam.project(Pc)
        idx = (int(np.argmin(uv[:, 0])), int(np.argmin(uv[:, 1])),
               int(np.argmax(uv[:, 0])), int(np.argmax(uv[:, 1])))
        box = np.array([uv[idx[0], 0], uv[idx[1], 1],
                        uv[idx[2], 0], uv[idx[3], 1]]) + self.edge_bias
        if not jac:
            return box, None
        pick = list(idx)
        Jp = cam.project_jac(Pc[pick])                     # (4, 2, 3) d像素/d相機座標
        # d相機座標/d(x, y, yaw) = R_cw @ [e_x, e_y, dP/dyaw]
        J = np.zeros((4, 3))
        for row, (k, comp) in enumerate(zip(pick, (0, 1, 0, 1))):
            dPw = np.stack([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], dyaw[k]], axis=1)  # (3, 3)
            J[row] = Jp[row, comp] @ (cam.R_cw @ dPw)
        return box, J

    def initial_guess(self, cam, box):
        """框的中心那條視線打到車身中段高度的位置 —— 給最小平方當起點用。"""
        u = 0.5 * (box[0] + box[2])
        v = 0.5 * (box[1] + box[3])
        return cam.ground_point(u, v, self.base_z + self.center_body[2])

    # ------------------------------------------------------------------ 方法 B
    def solve_position(self, obs, yaw, x0=None, iters: int = 10,
                       sigma_px: float = 3.0, size_frac: float = 0.03,
                       border_px: float = 4.0, solve_yaw: bool = False):
        """給定 yaw, 找最吻合所有框的 (x, y) (Gauss-Newton)。

        obs: [(cam, box), ...] —— 一台就是方法 B 本身; 多台就是同時刻的多相機三角定位。
        solve_yaw=True 時連 yaw 一起解 (只有多台相機從不同方向看才解得穩)。

        回傳 dict(x, y, yaw, cov (2x2 或 3x3), rms (px), rows, per_cam rms), 失敗回 None。

        **yaw 給錯, 位置就跟著錯** —— 斜拍時車子轉 10 度, 外接框的寬度與中心都會變,
        這裡會把那個變化當成位置來解釋。這正是為什麼融合節點不用這個函數的輸出,
        而是把框直接當量測、讓 yaw 也是 EKF 狀態的一部分 (見 multicam_ekf.py)。
        """
        if not obs:
            return None
        if x0 is None:
            g = [self.initial_guess(c, b) for c, b in obs]
            g = [p for p in g if p is not None]
            if not g:
                return None
            x0 = np.mean(np.array(g), axis=0)
        n = 3 if solve_yaw else 2
        th = np.array([float(x0[0]), float(x0[1]), float(yaw)])
        sig = [edge_sigma(b, sigma_px, size_frac) for _, b in obs]
        for _ in range(iters):
            st = self._stack(obs, th, sig, border_px, n)
            if st is None or len(st[0]) < n:
                return None
            r, A, w = st[0], st[1] * st[2][:, None], st[2]
            try:
                dx = np.linalg.solve(A.T @ A + 1e-9 * np.eye(n), A.T @ (r * w))
            except np.linalg.LinAlgError:
                return None
            # 一步不要跨太大 (起點可能差 1 m 以上, 線性化在那裡不準)
            step = float(np.linalg.norm(dx[:2]))
            if step > 0.5:
                dx *= 0.5 / step
            th[:n] += dx
            if step < 1e-4:
                break
        # 殘差與共變異數用**最終**的狀態重算
        st = self._stack(obs, th, sig, border_px, n)
        if st is None:
            return None
        r, J, w, per_cam = st
        A = J * w[:, None]
        try:
            cov = np.linalg.inv(A.T @ A)
        except np.linalg.LinAlgError:
            cov = np.eye(n) * 1e3
        return dict(x=float(th[0]), y=float(th[1]), yaw=float(th[2]), cov=cov,
                    rms=float(np.sqrt(np.mean(r * r))), rows=len(r), per_cam=per_cam)

    def _stack(self, obs, th, sig, border_px, n):
        """所有相機能用的邊疊起來: (殘差, Jacobian, 1/sigma, 每台的 rms)。"""
        rs, Js, ws, per_cam = [], [], [], []
        for (cam, box), s in zip(obs, sig):
            pred, J = self.predict_bbox(cam, th[0], th[1], th[2])
            m = None if pred is None else edge_mask(cam, box, pred, border_px)
            if m is None or not m.any():
                per_cam.append(None)
                continue
            rr = (np.asarray(box, dtype=np.float64) - pred)[m]
            rs.append(rr)
            Js.append(J[m][:, :n])
            ws.append(1.0 / s[m])
            per_cam.append(float(np.sqrt(np.mean(rr * rr))))
        if not rs:
            return None
        return np.concatenate(rs), np.vstack(Js), np.concatenate(ws), per_cam


def edge_sigma(box, sigma_px: float, size_frac: float) -> np.ndarray:
    """框的四個邊各自的量測雜訊 (px)。

    兩項: 偵測器本身的抖動 (常數 px), 加上模型誤差 (跟框的大小成比例 —— 凸包跟
    YOLO 學到的「框要框到哪裡」不會完全一致, 框越大差的 px 越多)。
    """
    b = np.asarray(box, dtype=np.float64)
    size = max(b[2] - b[0], b[3] - b[1], 1.0)
    return np.full(4, math.hypot(float(sigma_px), float(size_frac) * size))


def edge_mask(cam, box, pred, border_px: float) -> np.ndarray:
    """四個邊裡哪幾個能用。

    車子一部分出了畫面時, YOLO 的框會被裁在畫面邊緣 —— 那個邊量到的是「畫面在哪」,
    不是「車在哪」, 一定要丟掉。量到的邊貼著邊緣 (border_px 以內), 或預測的邊已經在
    畫面外面, 都不用。另外三個邊照常用, 所以車子半出畫面時這台相機還是有貢獻。
    """
    W, H = cam.width, cam.height
    b = np.asarray(box, dtype=np.float64)
    p = np.asarray(pred, dtype=np.float64)
    e = float(border_px)
    return np.array([b[0] > e and p[0] > 0.0,
                     b[1] > e and p[1] > 0.0,
                     b[2] < W - e and p[2] < W,
                     b[3] < H - e and p[3] < H])


def chi2_99(k: int) -> float:
    """k 自由度卡方分佈的 99% 分位數 (Wilson-Hilferty 近似, k>=1 誤差 < 1%)。"""
    k = max(int(k), 1)
    table = {1: 6.635, 2: 9.210, 3: 11.345, 4: 13.277}
    if k in table:
        return table[k]
    z = 2.3263
    a = 2.0 / (9.0 * k)
    return k * (1.0 - a + z * math.sqrt(a)) ** 3


def associate(model, cam, boxes, state=None, gate: float = 1.5, pad_px: float = 30.0):
    """這台相機的候選框裡哪一個是車。回傳 index 或 None。

    有預測位置 (濾波器已初始化) 時挑「中心離預測框中心最近」的, 距離以預測框的大小
    正規化, 超過 `gate` 就都不要 —— 跟 car_loc_camera 一樣, 誤判 (影子、反光)
    幾乎都在別的地方, 而車子不會瞬間移動。沒有預測時挑信心最高的。

    這只是粗篩 (時戳差個幾十 ms 沒關係); 真正的離群判斷是 EKF 裡的卡方閘門。
    boxes: [(box, conf), ...]
    """
    if not boxes:
        return None
    if state is None:
        return max(range(len(boxes)), key=lambda i: boxes[i][1])
    pred, _ = model.predict_bbox(cam, *state, jac=False)
    if pred is None:
        return None
    pc = 0.5 * (pred[:2] + pred[2:])
    size = max(pred[2] - pred[0], pred[3] - pred[1]) + float(pad_px)
    best, best_d = None, float('inf')
    for i, (b, _) in enumerate(boxes):
        b = np.asarray(b, dtype=np.float64)
        d = float(np.linalg.norm(0.5 * (b[:2] + b[2:]) - pc)) / size
        if d < best_d:
            best, best_d = i, d
    return best if best_d <= gate else None


def initial_pose(model, obs, yaw=None, *, max_rms: float = 15.0,
                 yaw_step_deg: float = 15.0, **kw):
    """從同一時刻的一組框 (每台相機一個) 找起點 (x, y, yaw)。

    yaw 給了 (Isaac 的 IMU 有絕對姿態) 就只解位置; 沒給就每 `yaw_step_deg` 試一次,
    挑殘差最小的, 再 (兩台以上時) 連 yaw 一起微調。
    **沒給 yaw 時前後 180 度幾乎分不出來** (車身前後差不多對稱), 所以回傳的
    sigma_yaw 會很大 —— 真車沒有絕對 yaw 時請用 LiDAR 或 initial_pose 起頭。

    三台以上時, 殘差特別大的那台 (很可能認錯) 會被丟掉再解一次。
    回傳 dict(x, y, yaw, sigma_p, sigma_yaw, rms, used) 或 None。
    """
    if not obs:
        return None
    yaws = ([float(yaw)] if yaw is not None
            else list(np.radians(np.arange(0.0, 360.0, yaw_step_deg))))

    def best_for(ob):
        best = None
        for yw in yaws:
            r = model.solve_position(ob, yw, **kw)
            if r is not None and (best is None or r['rms'] < best['rms']):
                best = r
        if best is not None and yaw is None and len(ob) >= 2:
            r = model.solve_position(ob, best['yaw'], x0=(best['x'], best['y']),
                                     solve_yaw=True, **kw)
            if r is not None and r['rms'] <= best['rms']:
                best = r
        return best

    res = best_for(obs)
    if res is None:
        return None
    used = list(obs)
    if len(obs) >= 3:
        pc = [v for v in res['per_cam'] if v is not None]
        lim = max(3.0 * float(np.median(pc)), max_rms)
        keep = [o for o, v in zip(obs, res['per_cam']) if v is not None and v <= lim]
        if 0 < len(keep) < len(obs):
            r2 = best_for(keep)
            if r2 is not None:
                res, used = r2, keep
    if res['rms'] > max_rms:
        return None
    cov = res['cov'][:2, :2]
    sp = float(np.sqrt(max(np.linalg.eigvalsh(cov).max(), 0.0)))
    if yaw is not None:
        sy = 0.05
    elif res['cov'].shape[0] == 3:
        sy = float(np.sqrt(max(res['cov'][2, 2], 0.0)))
        sy = min(max(sy, 0.2), 1.0)
    else:
        sy = 1.0
    return dict(x=res['x'], y=res['y'], yaw=res['yaw'], sigma_p=max(sp, 0.05),
                sigma_yaw=sy, rms=res['rms'], used=[c.name for c, _ in used])
