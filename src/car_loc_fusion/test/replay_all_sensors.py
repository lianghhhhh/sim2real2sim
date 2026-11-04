#!/usr/bin/env python3
"""拿實際錄的資料重放「多相機 + LiDAR + IMU + 輪速」—— 不需要 ROS / Isaac / YOLO。

    python3 test/replay_all_sensors.py ../../car_run_data/mc_friction_1102.csv

吃 `collect_data_node` 那一輪的三個檔 (`<csv>`, `<csv>_bbox.csv`, `<csv>_imu.csv`), 照當時
的**到達順序**把訊息餵給**真正的節點類別** (`MultiCamLocalizer`, ROS 用假模組頂替),
所以挑框、綁組、時鐘偵測、起點、倒帶、閘門全部是線上那一份程式碼。

跟線上的差別 (A/B 的差距可信, 絕對值以線上 evaluate 為準):

| | 線上 | 這裡 |
| --- | --- | --- |
| IMU 姿態 | /imu 的 orientation | CSV 沒錄 -> 用 GT yaw 內插頂替 (Isaac 裡兩者幾乎一樣) |
| ROI 回授 | 偵測節點讀融合的輸出 | 框是那一輪錄好的, 不會因為換設定而變 |
| LiDAR | 真的掃描 | /lidar_loc/odom 那一輪的輸出 (主 CSV 取樣, 到達時刻量化成 20 ms) |
"""
from __future__ import annotations

import math
import os
import sys
import types

import numpy as np
import pandas as pd
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.abspath(os.path.join(_HERE, '..', '..'))
for p in (os.path.join(_SRC, 'car_loc_fusion'), os.path.join(_SRC, 'car_loc_wheel'),
          os.path.join(_SRC, 'car_loc_wheel', 'test'), os.path.join(_SRC, 'car_loc_multicam')):
    sys.path.insert(0, p)

import _stub_ros                                          # noqa: E402 (要先載)


# 融合節點比 car_loc_wheel 多用到的訊息型別
def _ns(**k):
    return types.SimpleNamespace(**k)


_gm = sys.modules['geometry_msgs.msg']
_gm.Point = lambda **k: _ns(**{'x': 0.0, 'y': 0.0, 'z': 0.0, **k})
_gm.Point32 = _gm.Point
_gm.PolygonStamped = lambda: _ns(header=_stub_ros._hdr(), polygon=_ns(points=[]))
_stub_ros.mod('visualization_msgs')
_stub_ros.mod('visualization_msgs.msg', Marker=object, MarkerArray=object)

from car_loc_multicam import fusion_node                             # noqa: E402
from car_loc_multicam.fusion_node import MultiCamLocalizer           # noqa: E402
from nav_msgs.msg import Odometry                                    # noqa: E402
from sensor_msgs.msg import Imu, JointState                          # noqa: E402

_PKG = os.path.join(_SRC, 'car_loc_multicam')
WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
JOINTS = [f'{w}_joint' for w in WHEELS]


def _stamp(obj, t):
    obj.sec = int(math.floor(t))
    obj.nanosec = int((t - math.floor(t)) * 1e9)


def yaml_params():
    """跟 fusion_loc.launch.py 同一個疊法: 多相機的設定在下, fusion_loc.yaml 蓋上去。"""
    with open(os.path.join(_PKG, 'config', 'multicam_loc.yaml'), encoding='utf-8') as f:
        over = dict(yaml.safe_load(f)['multicam_fusion']['ros__parameters'])
    with open(os.path.join(_SRC, 'car_loc_fusion', 'config', 'fusion_loc.yaml'),
              encoding='utf-8') as f:
        over.update(yaml.safe_load(f)['fusion_localizer']['ros__parameters'])
    over['cameras_file'] = os.path.join(_PKG, 'config', 'env_cameras.yaml')
    over['car_model_file'] = os.path.join(_PKG, 'config', 'car_model.yaml')
    over['imu_lever_arm'] = [0.0, 0.0]       # Isaac 的 IMU 不模擬安裝位置 (節點看 use_sim_time)
    over['publish_markers'] = False
    over['publish_tf'] = False
    return over


def make_node(**over):
    params = {**yaml_params(), **over}
    orig = _stub_ros.Node.declare_parameter

    def patched(self, name, value=None):
        orig(self, name, params.get(name, value))

    _stub_ros.Node.declare_parameter = patched
    try:
        return MultiCamLocalizer()
    finally:
        _stub_ros.Node.declare_parameter = orig


# ---------------------------------------------------------------------- 讀資料
class Run:
    def __init__(self, path):
        base = path[:-4]
        d = pd.read_csv(path, low_memory=False)
        self.imu = pd.read_csv(base + '_imu.csv', low_memory=False)
        self.bbox = pd.read_csv(base + '_bbox.csv', low_memory=False)

        gt = d[['odom_stamp', 'car_position_x', 'car_position_y', 'gt_yaw',
                'car_linear_velocity_x', 'car_linear_velocity_y']].apply(
                    pd.to_numeric, errors='coerce').dropna()
        gt = gt.drop_duplicates('odom_stamp').sort_values('odom_stamp')
        self.gt_t = gt.odom_stamp.to_numpy()
        self.gt_x = gt.car_position_x.to_numpy()
        self.gt_y = gt.car_position_y.to_numpy()
        self.gt_yaw = np.unwrap(gt.gt_yaw.to_numpy())
        self.gt_v = np.hypot(gt.car_linear_velocity_x, gt.car_linear_velocity_y).to_numpy()

        lid = d[['timestamp', 'lid_stamp', 'lid_x', 'lid_y', 'lid_yaw', 'lid_sigma']].apply(
            pd.to_numeric, errors='coerce').dropna().drop_duplicates('lid_stamp')
        self.lidar = lid
        self.recorded_mc = d

    def gt_at(self, t):
        return (np.interp(t, self.gt_t, self.gt_x), np.interp(t, self.gt_t, self.gt_y),
                np.interp(t, self.gt_t, self.gt_yaw), np.interp(t, self.gt_t, self.gt_v))

    def events(self):
        """[(到達時刻, 順序, 種類, 內容)]。框的 recv 是 0 的 (時鐘還沒起來) 用中位偏移補。"""
        ev = []
        for r in self.imu.itertuples(index=False):
            if math.isfinite(r.stamp) and math.isfinite(r.acc_x):
                ev.append((float(r.recv), 1, 'imu', r))
        for r in self.lidar.itertuples(index=False):
            ev.append((float(r.timestamp), 2, 'lidar', r))
        b = self.bbox
        off = float((b.stamp - b.recv)[b.recv > 0].median())
        for (stamp, cam), g in b.groupby(['stamp', 'cam'], sort=False):
            recv = float(g.recv.iloc[0])
            ev.append((recv if recv > 0 else stamp - off, 3, 'bbox', (stamp, cam, g)))
        ev.sort(key=lambda e: (e[0], e[1]))
        return ev


def imu_msg(r, yaw):
    m = Imu()
    _stamp(m.header.stamp, r.stamp)
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = r.gyro_x, r.gyro_y, r.gyro_z
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = r.acc_x, r.acc_y, r.acc_z
    m.orientation.z, m.orientation.w = math.sin(yaw * 0.5), math.cos(yaw * 0.5)
    return m


def joint_msg(r):
    v = [getattr(r, f'{w}_velocity') for w in WHEELS]
    if not all(math.isfinite(x) for x in v):
        return None
    m = JointState()
    _stamp(m.header.stamp, r.stamp)
    m.name = list(JOINTS)
    m.velocity = [float(x) for x in v]
    return m


def lidar_msg(r):
    m = Odometry()
    _stamp(m.header.stamp, r.lid_stamp)
    m.pose.pose.position.x, m.pose.pose.position.y = r.lid_x, r.lid_y
    m.pose.pose.orientation.z = math.sin(r.lid_yaw * 0.5)
    m.pose.pose.orientation.w = math.cos(r.lid_yaw * 0.5)
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.5 * r.lid_sigma ** 2
    m.pose.covariance = cov
    return m


def bbox_msg(stamp, cam, g):
    m = _gm.PolygonStamped()
    _stamp(m.header.stamp, stamp)
    m.header.frame_id = cam
    for r in g.itertuples(index=False):
        if int(r.n_boxes) > 0 and math.isfinite(r.u1):
            m.polygon.points.append(_gm.Point32(x=r.u1, y=r.v1, z=r.conf))
            m.polygon.points.append(_gm.Point32(x=r.u2, y=r.v2, z=r.cls))
    return m


# ---------------------------------------------------------------------- 重放
def replay(run, events, *, t_end=None, mutate=None, quiet=True, **over):
    """回傳 (node, 輸出紀錄 ndarray [t x y yaw sigma nis_pos])。`mutate(kind, msg, t)` 可以改訊息。"""
    clock = [0.0]
    fusion_node.time.monotonic = lambda: clock[0]
    logger = None
    if quiet:
        logger = _ns(info=lambda m: None, warn=lambda m: None, error=lambda m: None)
    node = make_node(**over)
    if logger is not None:
        node.get_logger = lambda: logger
    out = []
    for arr, _, kind, c in events:
        if t_end is not None and arr > t_end:
            break
        clock[0] = arr
        node.flush()
        if kind == 'imu':
            yaw = float(np.interp(c.stamp, run.gt_t, run.gt_yaw))
            j = joint_msg(c)
            if j is not None:
                node.on_joint(j)
            node.on_imu(imu_msg(c, yaw))
            if node.ekf.initialized:
                P = node.ekf.ins.P
                out.append((c.stamp, node.ekf.pos[0], node.ekf.pos[1], node.ekf.yaw,
                            node.ekf.sigma_pos(), P[0, 0], P[0, 1], P[1, 1]))
        elif kind == 'lidar':
            m = lidar_msg(c)
            if mutate is not None:
                m = mutate('lidar', m, arr)
            if m is not None:
                node.on_abs(m, 'lidar')
        else:
            stamp, cam, g = c
            m = bbox_msg(stamp, cam, g)
            if mutate is not None:
                m = mutate('bbox', m, arr)
            cb = node.subs.get(f'/{cam}/car_bbox')    # multicam_enabled:=false 時沒有訂閱
            if m is not None and cb is not None:
                cb(m)
    return node, np.array(out)


def stats(run, out, mask_fn=None):
    t = out[:, 0]
    gx, gy, gyaw, gv = run.gt_at(t)
    ex, ey = out[:, 1] - gx, out[:, 2] - gy
    e = np.hypot(ex, ey)
    ey_ = np.arctan2(np.sin(out[:, 3] - gyaw), np.cos(out[:, 3] - gyaw))
    det = out[:, 5] * out[:, 7] - out[:, 6] ** 2
    with np.errstate(divide='ignore', invalid='ignore'):
        nees = (out[:, 7] * ex * ex - 2 * out[:, 6] * ex * ey + out[:, 5] * ey * ey) / det
    keep = np.ones(len(t), bool) if mask_fn is None else mask_fn(t, gv)
    e, ey_, nees = e[keep] * 100, np.degrees(np.abs(ey_[keep])), nees[keep]
    nees = nees[np.isfinite(nees)]
    return dict(n=len(e), rms=float(np.sqrt(np.mean(e * e))), med=float(np.median(e)),
                p95=float(np.percentile(e, 95)), max=float(e.max()),
                yaw95=float(np.percentile(ey_, 95)), nees=float(np.median(nees)),
                sig=float(np.median(out[keep, 4]) * 100))


def line(label, s):
    return (f"  {label:30s} RMS {s['rms']:5.2f}  中位 {s['med']:5.2f}  p95 {s['p95']:5.2f}  "
            f"最大 {s['max']:6.2f} cm | yaw p95 {s['yaw95']:5.2f}° | "
            f"sigma {s['sig']:4.2f} cm  NEES中位 {s['nees']:5.2f}")


def counts(n):
    b = n.ekf.bbox_counts
    li = n.src['lidar'].counts
    return (f"     框 {b['used']}/{b['obs']} 擋{b['gate']} 強制{b['forced']} | "
            f"LiDAR 用{li['used']} sigma擋{li['sigma']} 閘門擋{li['gate']} | "
            f"倒帶{n.ekf.counts['rewind']} 太舊{n.ekf.counts['too_old']} "
            f"恢復中={n.ekf.recovering}")


def recorded(run):
    """那一輪線上 multicam_fusion 自己錄下來的 mc_ —— 拿來確認這個重放的可信度。"""
    d = run.recorded_mc
    x = pd.to_numeric(d.mc_x, errors='coerce')
    gx = pd.to_numeric(d.car_position_x, errors='coerce')
    gy = pd.to_numeric(d.car_position_y, errors='coerce')
    e = np.hypot(x - gx, pd.to_numeric(d.mc_y, errors='coerce') - gy).dropna().to_numpy() * 100
    return float(np.sqrt(np.mean(e * e))), float(np.median(e)), float(np.percentile(e, 95)), float(e.max())


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        _SRC, '..', 'car_run_data', 'mc_friction_1102.csv')
    run = Run(path)
    ev = run.events()
    print(f'{os.path.basename(path)}: IMU {len(run.imu)} / LiDAR {len(run.lidar)} / '
          f'框 {len(run.bbox)} 則, GT {run.gt_t[0]:.1f}~{run.gt_t[-1]:.1f} s')
    r = recorded(run)
    print(f'  線上 mc_ (對照)                RMS {r[0]:5.2f}  中位 {r[1]:5.2f}  p95 {r[2]:5.2f}  '
          f'最大 {r[3]:6.2f} cm')
    for label, over in (('[A] 多相機 + 遞推', dict(lidar_enabled=False)),
                        ('[B] 全部 (相機+LiDAR+遞推)', dict(lidar_enabled=True)),
                        ('[C] 只有 LiDAR + 遞推', dict(lidar_enabled=True, multicam_enabled=False))):
        n, out = replay(run, ev, **over)
        print(line(label, stats(run, out)))
        print(counts(n))
    return 0


if __name__ == '__main__':
    sys.exit(main())
