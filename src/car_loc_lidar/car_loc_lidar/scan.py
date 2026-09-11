#!/usr/bin/env python3
"""雷射資料的解析與整形 —— 不依賴 ros2_numpy / sensor_msgs_py。

Isaac 的 ROS2RtxLidarHelper 發的資料裡, 「沒打到東西」的射線也會留在陣列裡
(座標是 0 或 NaN) —— car.usd 的 sensor 設了 skipDroppingInvalidPoints=1。
這件事很重要:

    點的**索引**因此對得上射線編號, 也就對得上時間。

運動補償 (deskew) 就是靠這個把一整圈掃描裡每個點的發射時刻還原出來的。過濾
無效點的時候要保留索引 (先算 mask, 再一起取), 不能邊走邊刪。

索引的方向不一定等於時間的方向
------------------------------
`LaserScan` 那條路要特別小心。Isaac 的 laser_scan 走的是
`IsaacComputeRTXLidarFlatScan`, 它的輸出文件寫得很清楚:

    "Linear depth measurements from full scan, **ordered by increasing azimuth**"

也就是說排序是按**方位角**, 不是按時間。而 MS200 (以及大部分旋轉式雷射) 是
**CW** 旋轉 —— 方位角隨時間遞減, 所以索引順序跟發射順序是**相反的**。
拿索引直接當時間會讓運動補償補到反邊去, 而且症狀很賊: 車子直線走的時候完全
正常, 一轉彎殘差就變兩倍。

所以 `frac` 要能反向, 由 `ScanFrontend(time_order=...)` 控制;
`lidar_loc_node` 的 `auto_scan_stamp` 會把正反兩個方向都試一遍用殘差投票,
不用自己猜。
"""
from __future__ import annotations

import math

import numpy as np

# sensor_msgs/PointField.datatype -> numpy
_PF_DTYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
             5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def pointcloud2_to_xyz(msg) -> np.ndarray:
    """(N, 3) float64。保持原本的點順序, 不做任何過濾。"""
    fields = {f.name: f for f in msg.fields if f.name in ('x', 'y', 'z')}
    if len(fields) < 3 or msg.point_step == 0:
        return np.empty((0, 3))
    raw = np.frombuffer(msg.data, dtype=np.uint8)
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


def laserscan_to_xyz(msg) -> np.ndarray:
    """LaserScan -> (N, 3) float64 (z 一律 0)。

    實體車多半是 2D 雷射 (wildbot 用的 oradar 就是), 發的是 LaserScan。
    無效回波會留在陣列裡當佔位 (歸零, 由 valid_mask 濾掉) —— 這樣索引跟角度的
    對應才不會錯位, 運動補償才算得對。

    **負數也算無效, 不能只擋 NaN/inf。** Isaac 的 IsaacComputeRTXLidarFlatScan
    對「這一批沒掃到的 bin」填的是 **-1**, 它是有限值。留著的話會被算成
    (-1*cos, -1*sin) = 半徑 1 m、方向反轉 180 度的一個點, 而 valid_mask 是用
    r**2 判斷的, 平方之後正負號就沒了 (1.0 >= range_min**2) 直接放行。
    實測 fullScan 關掉時 450 個 bin 有 298 個是 -1 -> 每幀 2/3 的點是一個完美的
    單位圓: 對旋轉完全對稱, 對 yaw 一點約束都給不出來, 卻在配準裡佔壓倒性權重,
    而且會被烤進地圖變成中央那個半徑 1 m 的幽靈環。
    """
    r = np.asarray(msg.ranges, dtype=np.float64)
    ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
    r = np.where(np.isfinite(r) & (r > 0.0), r, 0.0)
    return np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros_like(r)], axis=1)


def valid_mask(xyz: np.ndarray, range_min: float, range_max: float) -> np.ndarray:
    """濾掉無效回波: NaN/Inf、原點附近的空洞、超出量測範圍的。"""
    if xyz.size == 0:
        return np.zeros(0, dtype=bool)
    finite = np.isfinite(xyz).all(axis=1)
    r2 = np.einsum('ij,ij->i', xyz, xyz, optimize=True)
    with np.errstate(invalid='ignore'):
        return finite & (r2 >= range_min ** 2) & (r2 <= range_max ** 2)


def stride_subsample(n: int, max_points: int) -> np.ndarray:
    """等間隔取樣。用等間隔而不是隨機, 是因為索引就是時間 —— 等間隔才能保證
    取出來的點在整圈掃描的時間上仍然是均勻的。"""
    if n <= max_points:
        return np.arange(n)
    return np.linspace(0, n - 1, max_points).astype(np.int64)


def deskew(xy: np.ndarray, frac: np.ndarray, vx: float, vy: float, omega: float,
           period: float, stamp_at: str = 'end') -> np.ndarray:
    """把一整圈掃描裡每個點都補償到「同一個時刻」(掃描時戳那一刻)。

    為什麼一定要做: 一圈掃描是 50 ms 累積出來的, 這台車原地可以轉到 20 rad/s
    以上 —— 那 50 ms 裡車子會轉超過 50 度。不補償的話掃描圖形會被抹開,
    配準必錯。

    **這裡用的速度是掃描比對自己估出來的**, 不是 IMU 的 —— 這個 package 不吃
    IMU。等速假設在 50 ms 的尺度上夠用, 但它比 IMU 差一截, 而且開機頭幾幀
    速度還沒估出來 (那幾幀不要拿去建圖, 見 scan_odom_node)。

    參數:
        xy     (N,2) 感測器座標的點
        frac   (N,)  每個點在這一圈裡的時間比例 0..1 (= 索引比例)
        vx,vy        車體座標的線速度 (m/s)
        omega        角速度 (rad/s)
        period       一圈的時間 (s)
        stamp_at     訊息的時戳對應到一圈的哪裡: 'end' | 'mid' | 'start'
    """
    if xy.size == 0 or period <= 0.0:
        return xy
    ref = {'end': 1.0, 'mid': 0.5, 'start': 0.0}.get(stamp_at, 1.0)
    dt = (frac - ref) * period                   # 每個點相對於參考時刻的時間差
    a = omega * dt
    c, s = np.cos(a), np.sin(a)
    # 點在發射瞬間的車體位姿 -> 參考時刻的車體位姿
    out = np.empty_like(xy)
    out[:, 0] = c * xy[:, 0] - s * xy[:, 1] + vx * dt
    out[:, 1] = s * xy[:, 0] + c * xy[:, 1] + vy * dt
    return out


def to_laserscan(msg_cls, pts_xy: np.ndarray, stamp, frame_id: str,
                 bins: int = 720, range_min: float = 0.05,
                 range_max: float = 60.0):
    """把 2D 點集包成 LaserScan —— slam_toolbox / nav2 只吃這個。

    比 pointcloud_to_laserscan 好的地方是**這些點已經去過畸變了**: 車子邊轉邊掃
    出來的那一圈, 沒補償的版本是歪的, 拿去建圖就會建出一道糊掉的牆。
    """
    scan = msg_cls()
    scan.header.stamp = stamp
    scan.header.frame_id = frame_id
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = 2.0 * math.pi / bins
    scan.time_increment = 0.0
    scan.scan_time = 0.0
    scan.range_min = float(range_min)
    scan.range_max = float(range_max)

    r = np.full(bins, float('inf'), dtype=np.float32)
    if pts_xy.size:
        rng = np.hypot(pts_xy[:, 0], pts_xy[:, 1])
        ang = np.arctan2(pts_xy[:, 1], pts_xy[:, 0])
        idx = np.floor((ang - scan.angle_min) / scan.angle_increment).astype(np.int64)
        np.clip(idx, 0, bins - 1, out=idx)
        ok = (rng >= range_min) & (rng <= range_max)
        # 同一個角度格有多個點時取最近的 —— 那才是真正擋住視線的那個回波
        np.minimum.at(r, idx[ok], rng[ok].astype(np.float32))
    scan.ranges = r.tolist()
    return scan
