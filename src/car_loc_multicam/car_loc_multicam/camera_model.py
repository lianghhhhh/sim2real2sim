#!/usr/bin/env python3
"""環境相機的幾何模型: 世界座標的點 <-> 像素。

這個模組跟 ROS 無關。

為什麼不能沿用 car_loc_camera 的單應性
--------------------------------------
單應性是「相機看**一個平面**」的精確模型。天花板正下方拍的時候, 車身高度那一層
差不多就是一個平面, 所以它很好用。斜拍就不行了: 車子是一個 40 x 33 x 25 cm 的
立體, 斜著看時車頂與車底投影到畫面上的位置差很多 (2.75 m 高、20 度俯角看 5 m 外,
車頂與輪子底差 25 cm 高 = 畫面上 30~40 px), 而且差多少跟車的位置與朝向都有關,
不是一個平面映射能吸收的。所以這裡改用完整的針孔相機 (內參 + 外參), 再把車的
3D 外形丟進去投影 (見 car_model.py)。

座標慣例 (這裡一律用 OpenCV 的)
--------------------------------
相機座標: x 向右, y 向下, z 朝前 (光軸)。
**USD / Isaac 的相機不是這樣**: 它朝自己的 -Z 看、畫面上方是 +Y。從 USD 讀進來的
旋轉要先乘上 diag(1, -1, -1) 才是 OpenCV 的相機座標 —— 這一步在 from_dict 裡做,
別的地方不用管。

yaml 裡的外參可以用三種寫法之一 (見 config/env_cameras.yaml):
  * `usd_quat_wxyz`     USD 相機的世界姿態 (export_from_usd.py 直接從 stage 讀)
  * `usd_rotate_zx_deg` spawn_env_camera*.py 的 xformOp: rotateZ 再 rotateX
  * `rvec` + `tvec`     OpenCV solvePnP 的輸出 (實體相機用地面標記點校正時)
"""
from __future__ import annotations

import math
import os

import numpy as np

# 點離相機這麼近 (沿光軸, m) 以內就當成「在相機後面」, 不投影
Z_MIN = 0.05

# USD 相機 (-Z 朝前, +Y 朝上) -> OpenCV 相機 (+Z 朝前, +Y 朝下)
_USD_TO_CV = np.diag([1.0, -1.0, -1.0])


def quat_wxyz_to_matrix(q) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rodrigues(rvec) -> np.ndarray:
    r = np.asarray(rvec, dtype=np.float64).reshape(3)
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    k = r / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def _rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


class EnvCamera:
    """一台固定在世界座標的相機。"""

    def __init__(self, name, *, width, height, fx, fy, cx, cy, R_cw, center,
                 fisheye_k=None, image_topic='', detection_topic=''):
        self.name = str(name)
        self.width = int(width)
        self.height = int(height)
        self.fx, self.fy = float(fx), float(fy)
        self.cx, self.cy = float(cx), float(cy)
        # 世界 -> 相機 (OpenCV): p_c = R_cw @ (p_w - center)
        self.R_cw = np.asarray(R_cw, dtype=np.float64).reshape(3, 3)
        self.center = np.asarray(center, dtype=np.float64).reshape(3)
        # OpenCV fisheye (Kannala-Brandt) k1..k4; None = 純針孔 (Isaac 預設的相機)
        self.fisheye_k = (None if fisheye_k is None
                          else [float(v) for v in fisheye_k][:4])
        self.image_topic = image_topic or f'/{self.name}/rgb'
        self.detection_topic = detection_topic or f'/{self.name}/car_bbox'

    # ------------------------------------------------------------------ yaml
    @classmethod
    def from_dict(cls, d: dict, defaults: dict = None) -> 'EnvCamera':
        g = dict(defaults or {})
        g.update(d)
        C = np.asarray(g.get('position', g.get('center')), dtype=np.float64)
        if 'rvec' in g:
            R_cw = rodrigues(g['rvec'])
            tv = np.asarray(g['tvec'], dtype=np.float64).reshape(3)
            C = -R_cw.T @ tv
        else:
            if 'usd_quat_wxyz' in g:
                R_wu = quat_wxyz_to_matrix(g['usd_quat_wxyz'])
            elif 'usd_rotate_zx_deg' in g:
                rz, rx = g['usd_rotate_zx_deg']
                # xformOpOrder = [translate, rotateZ, rotateX]: 點先繞 X 再繞 Z
                R_wu = _rot_z(rz) @ _rot_x(rx)
            else:
                raise ValueError(f"相機 {g.get('name')} 沒有外參 "
                                 '(usd_quat_wxyz / usd_rotate_zx_deg / rvec+tvec 擇一)')
            R_cw = (R_wu @ _USD_TO_CV).T
        # 主點預設在畫面正中間 —— 用「連續」像素座標 (左上角那一格的左緣 = 0),
        # 跟 YOLO 的 xyxy 與 Isaac 的投影同一個慣例, 所以是 W/2 不是 (W-1)/2
        return cls(g['name'], width=g['width'], height=g['height'],
                   fx=g['fx'], fy=g.get('fy', g['fx']),
                   cx=g.get('cx', float(g['width']) / 2.0),
                   cy=g.get('cy', float(g['height']) / 2.0),
                   R_cw=R_cw, center=C, fisheye_k=g.get('fisheye_k'),
                   image_topic=g.get('image_topic', ''),
                   detection_topic=g.get('detection_topic', ''))

    # ------------------------------------------------------------------ 投影
    def to_cam(self, Pw: np.ndarray) -> np.ndarray:
        return (np.asarray(Pw, dtype=np.float64) - self.center) @ self.R_cw.T

    def project(self, Pc: np.ndarray) -> np.ndarray:
        """相機座標的點 (N, 3) -> 像素 (N, 2)。呼叫的人要先確定 Z > Z_MIN。"""
        Pc = np.atleast_2d(Pc)
        a = Pc[:, 0] / Pc[:, 2]
        b = Pc[:, 1] / Pc[:, 2]
        if self.fisheye_k is not None:
            r = np.hypot(a, b)
            th = np.arctan(r)
            t2 = th * th
            k1, k2, k3, k4 = self.fisheye_k
            thd = th * (1 + k1 * t2 + k2 * t2 ** 2 + k3 * t2 ** 3 + k4 * t2 ** 4)
            s = np.where(r > 1e-12, thd / np.maximum(r, 1e-12), 1.0)
            a, b = a * s, b * s
        return np.stack([self.fx * a + self.cx, self.fy * b + self.cy], axis=1)

    def project_jac(self, Pc: np.ndarray) -> np.ndarray:
        """d(像素) / d(相機座標), (N, 2, 3)。"""
        Pc = np.atleast_2d(np.asarray(Pc, dtype=np.float64))
        if self.fisheye_k is None:
            X, Y, Z = Pc[:, 0], Pc[:, 1], Pc[:, 2]
            J = np.zeros((len(Pc), 2, 3))
            J[:, 0, 0] = self.fx / Z
            J[:, 0, 2] = -self.fx * X / (Z * Z)
            J[:, 1, 1] = self.fy / Z
            J[:, 1, 2] = -self.fy * Y / (Z * Z)
            return J
        # 魚眼的解析式很長, 而這裡只對 bbox 四個邊上的點求 (最多 4 點), 數值微分就夠
        h = 1e-6 * np.maximum(np.abs(Pc[:, 2:3]), 1.0)
        J = np.zeros((len(Pc), 2, 3))
        for k in range(3):
            d = np.zeros_like(Pc)
            d[:, k] = h[:, 0]
            J[:, :, k] = (self.project(Pc + d) - self.project(Pc - d)) / (2 * h)
        return J

    def ray(self, u: float, v: float) -> np.ndarray:
        """像素 -> 世界座標的單位視線方向 (從相機中心出發)。"""
        a = (float(u) - self.cx) / self.fx
        b = (float(v) - self.cy) / self.fy
        if self.fisheye_k is not None:
            thd = math.hypot(a, b)
            th = thd
            k1, k2, k3, k4 = self.fisheye_k
            for _ in range(20):            # 牛頓法反解 theta_d(theta) = thd
                t2 = th * th
                f = th * (1 + k1 * t2 + k2 * t2 ** 2 + k3 * t2 ** 3 + k4 * t2 ** 4) - thd
                df = 1 + 3 * k1 * t2 + 5 * k2 * t2 ** 2 + 7 * k3 * t2 ** 3 + 9 * k4 * t2 ** 4
                th -= f / df
            s = math.tan(th) / thd if thd > 1e-12 else 1.0
            a, b = a * s, b * s
        d = self.R_cw.T @ np.array([a, b, 1.0])
        return d / np.linalg.norm(d)

    def ground_point(self, u: float, v: float, z: float):
        """像素那條視線打到世界 z 平面的位置 (x, y); 打不到 (朝上看) 回 None。"""
        d = self.ray(u, v)
        if d[2] > -1e-6:
            return None
        s = (z - self.center[2]) / d[2]
        p = self.center + s * d
        return float(p[0]), float(p[1])


def load_cameras(path: str) -> list:
    """讀 env_cameras.yaml。最外層的 key (width / fx / ...) 是每台相機的預設值。"""
    import yaml
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    defaults = {k: v for k, v in cfg.items() if k != 'cameras'}
    return [EnvCamera.from_dict(c, defaults) for c in cfg['cameras']
            if c.get('enabled', True)]


def share_file(name: str) -> str:
    """本 package 的 config/<name>: 裝好的 share 目錄優先, 沒有就用原始碼目錄。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('car_loc_multicam'), 'config', name)
    except Exception:
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', name)
