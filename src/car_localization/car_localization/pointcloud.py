#!/usr/bin/env python3
"""PointCloud2 的解析 —— 不依賴 ros2_numpy / sensor_msgs_py。

Isaac 的 ROS2RtxLidarHelper (type=point_cloud) 發出來的是 float32 的 xyz,
而且 car.usd 裡 sensor 設了 skipDroppingInvalidPoints=1, 所以「沒打到東西」
的射線也會留在陣列裡 (座標是 0 或 NaN)。這件事很重要:
點的「索引」因此對得上發射順序, 也就對得上時間 —— 運動補償 (deskew) 就是
靠這個把一整圈掃描裡每個點的發射時刻還原出來的。
"""
from __future__ import annotations

import numpy as np

# sensor_msgs/PointField.datatype -> numpy
_PF_DTYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
             5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def pointcloud2_to_xyz(msg) -> np.ndarray:
    """(N, 3) float64。保持原本的點順序, 不做任何過濾。"""
    fields = {f.name: f for f in msg.fields if f.name in ('x', 'y', 'z')}
    if len(fields) < 3:
        return np.empty((0, 3))

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.point_step == 0:
        return np.empty((0, 3))
    rows = raw.size // msg.point_step
    declared = msg.width * msg.height
    n = min(rows, declared) if declared else rows
    if n == 0:
        return np.empty((0, 3))
    raw = raw[:rows * msg.point_step].reshape(rows, msg.point_step)[:n]

    out = np.empty((n, 3), dtype=np.float64)
    order = '>' if msg.is_bigendian else '<'
    for i, name in enumerate(('x', 'y', 'z')):
        f = fields[name]
        dt = np.dtype(_PF_DTYPE[f.datatype]).newbyteorder(order)
        out[:, i] = raw[:, f.offset:f.offset + dt.itemsize].copy().view(dt).reshape(-1)
    return out


def valid_mask(xyz: np.ndarray, range_min: float, range_max: float) -> np.ndarray:
    """濾掉無效回波: NaN/Inf、原點附近的空洞、超出量測範圍的。"""
    if xyz.size == 0:
        return np.zeros(0, dtype=bool)
    finite = np.isfinite(xyz).all(axis=1)
    r2 = np.einsum('ij,ij->i', xyz, xyz, optimize=True)
    with np.errstate(invalid='ignore'):
        return finite & (r2 >= range_min ** 2) & (r2 <= range_max ** 2)


def stride_subsample(n: int, max_points: int) -> np.ndarray:
    """等間隔取樣。用等間隔而不是隨機, 是因為索引就是時間 ——
    等間隔才能保證取出來的點在整圈掃描的時間上仍然是均勻的。"""
    if n <= max_points:
        return np.arange(n)
    return np.linspace(0, n - 1, max_points).astype(np.int64)


def laserscan_to_xyz(msg):
    """LaserScan -> (N, 3) float64 (z 一律 0) + 每個點的索引比例。

    實體機器人多半是 2D 雷射 (wildbot 用的 oradar 就是), 發的是 LaserScan 而不是
    PointCloud2。這裡照樣保留原本的點順序, 索引比例就是它在這一圈裡的發射時刻。
    無效回波 (NaN / inf / 超出量程) 會留在陣列裡當佔位, 由 valid_mask 濾掉 ——
    這樣索引跟角度的對應才不會錯位。
    """
    r = np.asarray(msg.ranges, dtype=np.float64)
    ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
    bad = ~np.isfinite(r)
    r = np.where(bad, 0.0, r)
    return np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros_like(r)], axis=1)
