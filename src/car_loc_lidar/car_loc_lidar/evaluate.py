#!/usr/bin/env python3
"""拿 Isaac 的 ground truth /odom 當尺, 量方法二 (LiDAR) 到底差幾公分。

    ros2 run car_loc_lidar lidar_loc_eval
    ros2 run car_loc_lidar lidar_loc_eval --ros-args -p csv:=/workspaces/car_run_data/lidar_eval.csv

**先看「常數偏移」那一行再看誤差。** 這條路的地圖是 slam_toolbox 建的, 而
slam_toolbox 的 `map` 原點是**車子按 Play 那一刻的位置**, 不是 USD 的世界原點 ——
所以位置一定會跟 Isaac 的 `/odom` 差一個固定平移。那不是定位在漂, 是座標系
原點不同。扣掉常數偏移之後剩下的才是真正的定位誤差。

yaw 誤差在這條路上特別重要: 沒有 IMU 的時候 yaw 是配準自己解出來的, 而 yaw
誤差會透過「距離 x 角度」放大成位置誤差。所以這裡多報一行「yaw 誤差換算成
10 m 外的位置誤差是多少」。
"""
from __future__ import annotations

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=20)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class LidarEval(Node):

    def __init__(self):
        super().__init__('lidar_loc_eval')
        p = self.declare_parameter
        p('truth_topic', '/odom')
        p('estimate_topic', '/lidar_loc/odom')
        p('report_period', 5.0)
        p('max_pair_dt', 0.03)
        p('lever_arm', 10.0)      # yaw 誤差換算成位置誤差時用的距離 (m)
        p('csv', '')

        self.truth = []
        self.pairs = []
        self.csv_path = self.get_parameter('csv').value
        self.csv = open(self.csv_path, 'w') if self.csv_path else None
        if self.csv:
            self.csv.write('t,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,'
                           'err_x,err_y,err_pos,err_yaw_deg\n')

        self.create_subscription(Odometry, self.get_parameter('truth_topic').value,
                                 self.on_truth, QOS)
        self.create_subscription(Odometry, self.get_parameter('estimate_topic').value,
                                 self.on_est, QOS)
        self.create_timer(float(self.get_parameter('report_period').value), self.report)
        self.get_logger().info(
            f"比對 {self.get_parameter('estimate_topic').value} vs "
            f"{self.get_parameter('truth_topic').value} (ground truth)")

    def on_truth(self, m: Odometry):
        p = m.pose.pose.position
        self.truth.append((stamp_sec(m.header.stamp), p.x, p.y,
                           yaw_of(m.pose.pose.orientation)))
        if len(self.truth) > 20000:
            del self.truth[:10000]

    def on_est(self, m: Odometry):
        t = stamp_sec(m.header.stamp)
        g = self._truth_at(t)
        if g is None:
            return
        p = m.pose.pose.position
        e = (p.x, p.y, yaw_of(m.pose.pose.orientation))
        self.pairs.append((t, g[0], g[1], g[2], e[0], e[1], e[2]))
        if self.csv:
            ex, ey = e[0] - g[0], e[1] - g[1]
            self.csv.write(f'{t:.6f},{g[0]:.6f},{g[1]:.6f},{g[2]:.6f},'
                           f'{e[0]:.6f},{e[1]:.6f},{e[2]:.6f},'
                           f'{ex:.6f},{ey:.6f},{math.hypot(ex, ey):.6f},'
                           f'{math.degrees(wrap(e[2] - g[2])):.6f}\n')

    def _truth_at(self, t: float):
        if len(self.truth) < 2:
            return None
        ts = [r[0] for r in self.truth]
        i = int(np.searchsorted(ts, t))
        if i <= 0 or i >= len(ts):
            return None
        t0, t1 = ts[i - 1], ts[i]
        mx = float(self.get_parameter('max_pair_dt').value)
        if (t - t0) > mx and (t1 - t) > mx:
            return None
        k = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        a, b = self.truth[i - 1], self.truth[i]
        return (a[1] + k * (b[1] - a[1]), a[2] + k * (b[2] - a[2]),
                a[3] + k * wrap(b[3] - a[3]))

    @staticmethod
    def _stats(arr):
        a = np.asarray(arr, dtype=np.float64)
        return (float(np.sqrt(np.mean(a ** 2))), float(np.mean(np.abs(a))),
                float(np.percentile(np.abs(a), 95)), float(np.max(np.abs(a))))

    def report(self, final=False):
        if len(self.pairs) < 5:
            self.get_logger().info(f'樣本 {len(self.pairs)} 筆, 還不夠')
            return
        a = np.array(self.pairs)
        ex, ey = a[:, 4] - a[:, 1], a[:, 5] - a[:, 2]
        ep = np.hypot(ex, ey)
        eyaw = np.degrees([wrap(v) for v in (a[:, 6] - a[:, 3])])
        travelled = float(np.sum(np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2]))))
        bx, by = float(np.mean(ex)), float(np.mean(ey))
        dr = np.hypot(ex - bx, ey - by)

        pr, _, p95, _ = self._stats(ep)
        dpr, _, _, dmx = self._stats(dr)
        yr, _, _, ymx = self._stats(eyaw)
        self.get_logger().info(
            f"[{'final' if final else 'live '}] {len(self.pairs)} 筆, "
            f'GT 走了 {travelled:.2f} m | 位置誤差 RMS {pr * 100:.2f} cm '
            f'(扣常數偏移後 {dpr * 100:.2f} cm), p95 {p95 * 100:.2f} cm | '
            f'yaw RMS {yr:.2f}°, 最大 {ymx:.2f}°')

        if not final:
            return
        lever = float(self.get_parameter('lever_arm').value)
        print('\n' + '=' * 74)
        print(f'  方法二 (只用 LiDAR)   樣本 {len(self.pairs)} 筆   '
              f'GT 總行走 {travelled:.2f} m')
        print('=' * 74)
        for name, v, unit, k in (('位置誤差 |dp|', ep, 'cm', 100),
                                 ('  其中 dx  ', ex, 'cm', 100),
                                 ('  其中 dy  ', ey, 'cm', 100),
                                 ('yaw 誤差    ', eyaw, 'deg', 1)):
            r, m, q, x = self._stats(v)
            print(f'  {name}:  RMS {r * k:8.3f} {unit}   平均 {m * k:8.3f} {unit}   '
                  f'p95 {q * k:8.3f} {unit}   最大 {x * k:8.3f} {unit}')

        print(f'\n  常數偏移: dx {bx * 100:+.2f} cm, dy {by * 100:+.2f} cm')
        print(f'  扣掉常數偏移後的位置誤差: RMS {dpr * 100:.3f} cm, 最大 {dmx * 100:.3f} cm')
        print('  (常數偏移大而扣掉後小 = 地圖原點跟 /odom 原點差一個平移, 不是在漂。')
        print('   slam_toolbox 的 map 原點是車子按 Play 那一刻的位置, 一定會這樣。)')
        if math.hypot(bx, by) > 0.05:
            print('\n  要讓地圖座標跟 /odom 對齊, 把地圖 .yaml 的 origin 減掉這個偏移:')
            print(f'      origin_new = [origin_x - ({bx:.4f}), origin_y - ({by:.4f}), 0]')

        print(f'\n  yaw 誤差的槓桿效應: RMS {yr:.3f}° 在 {lever:.0f} m 外 = '
              f'{math.radians(yr) * lever * 100:.2f} cm 的位置誤差')
        print('  (沒有 IMU 的時候 yaw 是配準自己解的。yaw 這一項如果比位置誤差')
        print('   換算過來還大, 代表瓶頸在角度不在位置 —— 要找更多不同朝向的牆。)')
        if self.csv_path:
            print(f'\n  逐點資料已寫到 {self.csv_path}')
        print('=' * 74)


def main(args=None):
    rclpy.init(args=args)
    node = LidarEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report(final=True)
        if node.csv:
            node.csv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
