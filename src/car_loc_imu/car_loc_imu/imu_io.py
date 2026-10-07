#!/usr/bin/env python3
"""離線讀 IMU 資料 —— rosbag2 (sqlite3) 或 CSV。**不需要 ROS。**

回傳的格式一律是:
    t    (N,)    時戳 (s)
    q    (N, 4)  姿態四元數 x, y, z, w  (CSV 沒有的話是 0, 0, 0, 1)
    gyro (N, 3)  角速度 (rad/s)
    acc  (N, 3)  線加速度 (m/s^2)
"""
from __future__ import annotations

import csv
import os
import sqlite3
import struct

import numpy as np


class Cdr:
    """最小的 CDR reader。CDR 每個基本型別都要對齊到自己的大小,
    而且對齊是相對於 **encapsulation header 之後**的位置算的。"""

    def __init__(self, buf: bytes):
        self.b = buf
        self.little = buf[1] == 1        # encapsulation: 0x0001 = LE
        self.o = 4                       # 跳過 4 bytes 的 encapsulation header

    def _align(self, n):
        pad = (self.o - 4) % n
        if pad:
            self.o += n - pad

    def _get(self, fmt, n):
        self._align(n)
        v = struct.unpack_from(('<' if self.little else '>') + fmt, self.b, self.o)[0]
        self.o += n
        return v

    def i32(self):
        return self._get('i', 4)

    def u32(self):
        return self._get('I', 4)

    def f64(self):
        return self._get('d', 8)

    def string(self):
        n = self.u32()
        s = self.b[self.o:self.o + n - 1].decode('utf-8', 'replace')
        self.o += n
        return s

    def f64a(self, n):
        return [self.f64() for _ in range(n)]

    def header(self):
        sec = self.i32()
        nsec = self.u32()
        self.string()                    # frame_id
        return sec + nsec * 1e-9


def parse_imu(buf):
    """sensor_msgs/Imu 的 CDR -> (t, q, gyro, acc)。"""
    c = Cdr(buf)
    t = c.header()
    q = c.f64a(4)                        # x, y, z, w
    c.f64a(9)
    w = c.f64a(3)
    c.f64a(9)
    a = c.f64a(3)
    return t, np.array(q), np.array(w), np.array(a)


def _pack(rows):
    t = np.array([r[0] for r in rows], dtype=np.float64)
    q = np.array([r[1] for r in rows], dtype=np.float64)
    w = np.array([r[2] for r in rows], dtype=np.float64)
    a = np.array([r[3] for r in rows], dtype=np.float64)
    o = np.argsort(t, kind='stable')
    return t[o], q[o], w[o], a[o]


def read_bag_imu(path: str, topic: str = '/imu'):
    """讀 rosbag2 目錄 (或直接給 .db3) 裡的一個 sensor_msgs/Imu topic。

    只支援 sqlite3 儲存格式。新版 ROS 2 預設錄成 mcap, 錄的時候要加
    `-s sqlite3`: `ros2 bag record -s sqlite3 -o static /imu`。
    """
    if os.path.isdir(path):
        db = sorted(f for f in os.listdir(path) if f.endswith('.db3'))
        if not db:
            raise SystemExit(f'{path} 裡沒有 .db3 —— 如果是 .mcap, '
                             '請用 `ros2 bag record -s sqlite3` 重錄')
        files = [os.path.join(path, f) for f in db]
    else:
        files = [path]
    rows = []
    for f in files:
        con = sqlite3.connect(f)
        ids = dict(con.execute('SELECT name, id FROM topics'))
        if topic not in ids:
            con.close()
            raise SystemExit(f'{f} 裡沒有 {topic} (有的是 {sorted(ids)})')
        for (data,) in con.execute(
                'SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp',
                (ids[topic],)):
            rows.append(parse_imu(data))
        con.close()
    if not rows:
        raise SystemExit(f'{path} 的 {topic} 沒有任何訊息')
    return _pack(rows)


def read_csv_imu(path: str):
    """讀 CSV。需要欄位 stamp, gyro_x/y/z, acc_x/y/z (car_run_data 的格式);
    有 quat_x/y/z/w 的話一併讀。"""
    rows = []
    with open(path, newline='') as fh:
        rd = csv.DictReader(fh)
        need = ['stamp', 'gyro_x', 'gyro_y', 'gyro_z', 'acc_x', 'acc_y', 'acc_z']
        miss = [k for k in need if k not in (rd.fieldnames or [])]
        if miss:
            raise SystemExit(f'{path} 少了欄位 {miss}')
        has_q = all(k in rd.fieldnames for k in ('quat_x', 'quat_y', 'quat_z', 'quat_w'))
        for r in rd:
            try:
                t = float(r['stamp'])
                w = [float(r['gyro_x']), float(r['gyro_y']), float(r['gyro_z'])]
                a = [float(r['acc_x']), float(r['acc_y']), float(r['acc_z'])]
                q = ([float(r['quat_x']), float(r['quat_y']), float(r['quat_z']),
                      float(r['quat_w'])] if has_q else [0.0, 0.0, 0.0, 1.0])
            except (TypeError, ValueError):
                continue
            rows.append((t, q, w, a))
    if not rows:
        raise SystemExit(f'{path} 沒有讀到任何有效的資料列')
    return _pack(rows)


def read_imu(path: str, topic: str = '/imu'):
    """依副檔名選 CSV 或 rosbag2。"""
    if path.lower().endswith('.csv'):
        return read_csv_imu(path)
    return read_bag_imu(path, topic)


def write_csv_imu(path: str, t, gyro, acc, q=None):
    """寫成 read_csv_imu 讀得回來的格式。"""
    cols = ['stamp', 'gyro_x', 'gyro_y', 'gyro_z', 'acc_x', 'acc_y', 'acc_z']
    data = [np.asarray(t)[:, None], np.asarray(gyro), np.asarray(acc)]
    if q is not None:
        cols += ['quat_x', 'quat_y', 'quat_z', 'quat_w']
        data.append(np.asarray(q))
    np.savetxt(path, np.hstack(data), delimiter=',', header=','.join(cols),
               comments='', fmt=['%.6f'] + ['%.9g'] * (len(cols) - 1))
