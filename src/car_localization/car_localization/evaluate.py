#!/usr/bin/env python3
"""把 LiDAR+IMU 估出來的位姿跟 Isaac 的 ground truth /odom 逐點比對。

這個節點存在的理由很簡單: 「定位準不準」不能用看的。Isaac 的 /odom 是模擬器
直接給的真值, 拿它當尺, 才知道估計值差幾公分。

用法:
    ros2 run car_localization localization_eval
    ros2 run car_localization localization_eval --ros-args -p csv:=/workspaces/car_run_data/loc_eval.csv

開著車跑一段 (直線、轉彎、原地打轉都要試), 然後 Ctrl-C 看總結。
"""
from __future__ import annotations

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=20)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class Evaluator(Node):

    def __init__(self):
        super().__init__('localization_eval')
        self.declare_parameter('truth_topic', '/odom')
        self.declare_parameter('estimate_topic', '/localization/odom')
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('max_pair_dt', 0.02)
        self.declare_parameter('csv', '')
        # /odom 的原點是「車子按下 Play 當下的位姿」, 地圖的原點是 USD 的世界原點。
        # car.usd 裡車子就停在世界原點, 兩者本來就重合; 如果你把車子挪過位置,
        # 就會差一個常數。開這個之後會另外報「扣掉常數偏移之後」的誤差,
        # 用來分辨「定位在漂」還是「只是座標原點差一個常數」。
        self.declare_parameter('report_debiased', True)

        self.truth = []          # (t, x, y, yaw)
        self.pairs = []          # (t, gx, gy, gyaw, ex, ey, eyaw)
        self.csv_path = self.get_parameter('csv').value
        self.csv = open(self.csv_path, 'w') if self.csv_path else None
        if self.csv:
            self.csv.write('t,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,'
                           'err_x,err_y,err_pos,err_yaw_deg\n')

        self.create_subscription(Odometry, self.get_parameter('truth_topic').value,
                                 self.on_truth, SENSOR_QOS)
        self.create_subscription(Odometry, self.get_parameter('estimate_topic').value,
                                 self.on_est, SENSOR_QOS)
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
        """把 ground truth 內插到估計值的時刻。時間對不上就不配對 ——
        寧可少幾筆樣本, 也不要拿差了一個 frame 的真值去算誤差。"""
        if len(self.truth) < 2:
            return None
        ts = [r[0] for r in self.truth]
        i = int(np.searchsorted(ts, t))
        if i <= 0 or i >= len(ts):
            return None
        t0, t1 = ts[i - 1], ts[i]
        if (t - t0) > float(self.get_parameter('max_pair_dt').value) and \
           (t1 - t) > float(self.get_parameter('max_pair_dt').value):
            return None
        k = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        a, b = self.truth[i - 1], self.truth[i]
        return (a[1] + k * (b[1] - a[1]),
                a[2] + k * (b[2] - a[2]),
                a[3] + k * wrap(b[3] - a[3]))

    # ------------------------------------------------------------------
    def _stats(self, arr):
        a = np.asarray(arr)
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

        pr, pm, p95, pmx = self._stats(ep)
        yr, ym, y95, ymx = self._stats(eyaw)
        head = 'final' if final else 'live '
        self.get_logger().info(
            f'[{head}] {len(self.pairs)} 筆, GT 走了 {travelled:.2f} m | '
            f'位置誤差 RMS {pr * 100:.2f} cm, 平均 {pm * 100:.2f} cm, '
            f'p95 {p95 * 100:.2f} cm, 最大 {pmx * 100:.2f} cm | '
            f'yaw 誤差 RMS {yr:.3f} deg, 最大 {ymx:.3f} deg')

        if final:
            print('\n' + '=' * 74)
            print(f'  樣本 {len(self.pairs)} 筆   ground truth 總行走 {travelled:.2f} m')
            print('=' * 74)
            for name, v, unit, k in (('位置誤差 |dp|', ep, 'cm', 100),
                                     ('  其中 dx  ', ex, 'cm', 100),
                                     ('  其中 dy  ', ey, 'cm', 100),
                                     ('yaw 誤差    ', eyaw, 'deg', 1)):
                r, m, q, x = self._stats(v)
                print(f'  {name}:  RMS {r * k:8.3f} {unit}   平均 {m * k:8.3f} {unit}   '
                      f'p95 {q * k:8.3f} {unit}   最大 {x * k:8.3f} {unit}')
            if bool(self.get_parameter('report_debiased').value):
                bx, by = float(np.mean(ex)), float(np.mean(ey))
                byaw = float(np.mean(eyaw))
                dr = np.hypot(ex - bx, ey - by)
                r, m, q, x = self._stats(dr)
                print(f'\n  常數偏移: dx {bx * 100:+.2f} cm, dy {by * 100:+.2f} cm, '
                      f'dyaw {byaw:+.3f} deg')
                print(f'  扣掉常數偏移後的位置誤差: RMS {r * 100:.3f} cm, '
                      f'最大 {x * 100:.3f} cm')
                print('  (常數偏移大而扣掉後很小 = 地圖原點跟 /odom 原點差一個平移,'
                      '\n   不是定位在漂。SLAM 建的地圖一定會這樣 —— slam_toolbox 的'
                      '\n   map 原點是車子按下 Play 那一刻的位置, 不是 USD 的世界原點。)')
                if math.hypot(bx, by) > 0.05:
                    print('\n  要讓地圖座標跟 /odom 對齊, 把地圖 .yaml 的 origin 減掉這個偏移:')
                    print(f'      origin_new = [origin_x - ({bx:.4f}), '
                          f'origin_y - ({by:.4f}), 0]')
                    print('  (改完重跑一次, 常數偏移應該會掉到 1 cm 以內。'
                          '.npz 地圖沒有這個問題, 它本來就在世界座標。)')
            if self.csv_path:
                print(f'\n  逐點資料已寫到 {self.csv_path}')
            print('=' * 74)


def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
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
