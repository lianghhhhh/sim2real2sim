#!/usr/bin/env python3
"""像素 -> 世界地面座標。

這個模組跟 ROS 無關, 可以直接拿 CSV 進來驗:

    python3 -m car_loc_camera.projection config/camera_ground.yaml 960 768

模型 (三段, 由內而外):

    1. 正規化   d  = ([px, py] - center_px) / norm_scale
    2. 去畸變   d' = d * (1 + k1|d|^2 + k2|d|^4)
    3. 單應性   [X, Y, w] = H @ [d'x, d'y, 1];  (x, y) = (X/w, Y/w)

為什麼是單應性而不是「像素偏移乘一個比例」
------------------------------------------
天花板相機看的是一個平面, 針孔相機看平面的**精確**模型就是單應性 —— 相機傾斜、
主點偏移、安裝旋轉、以及視差 (相機在 z=2.7 m, 但 bbox 中心看到的是車身 z≈0.10 m
而不是接地點, 量到的半徑會大 2.7/(2.7-0.10) = 3.7%) 全部會被它一起吸收掉。
用比例縮放的舊做法漏掉的就是這些。

解析度不寫死
------------
校正參數是在 `image_width x image_height` 上擬合的。實際影像不同解析度時先按
比例換算回去, 所以改 Isaac 的 render product 解析度不會默默算錯。
"""
from __future__ import annotations

import math

import numpy as np


def aabb_center_offset(dx, dy, yaw, *, h_cam, z_ref, h_top, half_len, half_wid):
    """軸對齊 bbox 中心相對於「車體中心投影」的偏移 (參考平面上的世界單位)。

    YOLO 給的是**軸對齊**的框, 但車子是一個有高度的長方體: 頂面離相機比較近、
    投影比較大, 所以框的中心落在頂面與底面投影之間, 而且偏移量隨 yaw 改變
    (旋轉矩形在 x/y 方向的外接寬度隨 yaw 變)。這是可以精確算掉的幾何。

    高度 z 的一點投影到參考平面 z_ref 上會被放大 s(z) = (h_cam-z_ref)/(h_cam-z) 倍
    (以相機正下方那一點為中心)。`dx, dy` 就是車體中心相對於那一點的位置。

    量級只有 0.5~1 cm —— 不是誤差主因, 預設關著 (`use_aabb: false`), 但校正做細
    之後它就是還剩下的東西之一。
    """
    s0 = (h_cam - z_ref) / h_cam                    # 底面 z=0
    s1 = (h_cam - z_ref) / (h_cam - h_top)          # 頂面 z=h_top
    a, b = float(half_len), float(half_wid)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    U = a * c + b * s                               # x 方向的外接半寬
    W = a * s + b * c                               # y 方向的外接半寬

    def one(d, E):
        d = np.asarray(d, dtype=np.float64)
        # |d| >= E: 最外側的角來自頂面, 最內側來自底面
        far = 0.5 * (s0 + s1) * d + 0.5 * (s1 - s0) * E * np.sign(d)
        # |d| <  E: 兩側最外的角都來自頂面
        near = s1 * d
        # 參考平面就設在車身高度, 所以「真值」就是 d 本身; 偏移 = 視在 - 真值
        return np.where(np.abs(d) >= E, far, near) - d

    return float(one(dx, U)), float(one(dy, W))


class GroundProjection:
    """從 yaml 讀出來的地面投影模型。"""

    def __init__(self, cfg: dict):
        self.ref_w = float(cfg['image_width'])
        self.ref_h = float(cfg['image_height'])
        self.center = np.asarray(cfg['center_px'], dtype=np.float64).reshape(2)
        self.scale = float(cfg['norm_scale'])
        k = list(cfg.get('distortion', [0.0, 0.0])) + [0.0, 0.0]
        self.k1, self.k2 = float(k[0]), float(k[1])
        self.H = np.asarray(cfg['homography'], dtype=np.float64).reshape(3, 3)
        # 影像時刻比訊息時刻早多少秒 (曝光 + 傳輸 + YOLO 推論)。
        # 校正腳本掃描得到; 沒量過就是 0, 效果等同不修正。
        self.delay = float(cfg.get('delay', 0.0))
        self.use_aabb = bool(cfg.get('use_aabb', False))
        a = dict(cfg.get('aabb', {}))
        self.aabb = dict(h_cam=float(a.get('h_cam', 2.7)),
                         z_ref=float(a.get('z_ref', 0.10)),
                         h_top=float(a.get('h_top', 0.20)),
                         half_len=float(a.get('half_len', 0.21)),
                         half_wid=float(a.get('half_wid', 0.14)))
        self.cam_xy = np.asarray(a.get('cam_xy', [0.0, 0.0]), dtype=np.float64)
        # 校正腳本自己報的殘差 (m)。節點拿它當量測雜訊 R 的預設值 ——
        # 「這個模型有多準」本來就該由校正檔自己說, 不該在節點裡另外猜一個。
        self.sigma = float(cfg.get('residual_rms', 0.075))

    @classmethod
    def from_yaml(cls, path: str) -> 'GroundProjection':
        import yaml
        with open(path) as f:
            return cls(yaml.safe_load(f))

    # ------------------------------------------------------------------
    def __call__(self, px: float, py: float, width=None, height=None,
                 yaw=None) -> tuple:
        """(px, py) -> (x, y)。給了 width/height 就先換算成校正時的解析度。"""
        px = float(px)
        py = float(py)
        if width:
            px *= self.ref_w / float(width)
        if height:
            py *= self.ref_h / float(height)

        d = (np.array([px, py]) - self.center) / self.scale
        r2 = float(d @ d)
        d = d * (1.0 + self.k1 * r2 + self.k2 * r2 * r2)

        v = self.H @ np.array([d[0], d[1], 1.0])
        if abs(v[2]) < 1e-9:
            raise ValueError('單應性退化 (w≈0), 校正檔可能不對')
        x, y = float(v[0] / v[2]), float(v[1] / v[2])

        if self.use_aabb and yaw is not None:
            # 偏移量本身跟車體中心有關, 所以做幾次定點迭代 (偏移只有 ~1 cm,
            # 一次就幾乎收斂了)
            xa, ya = x, y
            for _ in range(3):
                ox, oy = aabb_center_offset(x - self.cam_xy[0], y - self.cam_xy[1],
                                            yaw, **self.aabb)
                x, y = xa - ox, ya - oy
        return x, y


def _cli(argv):
    if len(argv) < 4:
        print('用法: python3 -m car_loc_camera.projection <camera_ground.yaml> <px> <py>')
        return 1
    p = GroundProjection.from_yaml(argv[1])
    x, y = p(float(argv[2]), float(argv[3]))
    print(f'({argv[2]}, {argv[3]}) px  ->  ({x:+.4f}, {y:+.4f}) m'
          f'   [校正殘差 {p.sigma * 100:.1f} cm, 延遲 {p.delay * 1000:.0f} ms]')
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(_cli(sys.argv))
