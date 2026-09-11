#!/usr/bin/env python3
"""量 IMU + 輪速航位推算的漂移。

    ros2 run car_loc_wheel wheel_loc_eval
    ros2 run car_loc_wheel wheel_loc_eval --ros-args -p csv:=/workspaces/car_run_data/wheel_eval.csv

**航位推算不能用「平均誤差」評價。** 誤差是單調長大的, 平均值只是「你剛好跑了
多久」的函數。有意義的是:

* **漂移率 = 誤差 / 已走距離 (%)** —— 這才是航位推算的標準指標。純 IMU 用
  「多久之後超過 X 公尺」(時間), 加了輪速之後誤差跟**距離**走, 所以看距離。
* **多久 / 走多遠之後超過 X 公尺**。
* **停車有沒有讓誤差停止長大** —— 這是輪速最直接的好處: 車子沒動的時候輪速
  說速度是 0, 位置誤差應該完全不長。誤差曲線在停車段應該是**水平的**。
"""
from __future__ import annotations

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=50)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class WheelEval(Node):

    def __init__(self):
        super().__init__('wheel_loc_eval')
        p = self.declare_parameter
        p('truth_topic', '/odom')
        p('estimate_topic', '/wheel_loc/odom')
        p('report_period', 5.0)
        p('max_pair_dt', 0.03)
        p('thresholds', [0.1, 0.25, 0.5, 1.0])   # m
        p('still_speed', 0.05)
        p('csv', '')

        self.truth = []
        self.pairs = []          # (t, gx, gy, gyaw, ex, ey, eyaw, gt_dist)
        self.gt_dist = 0.0
        self._last_gt = None
        self.t_start = None

        self.csv_path = self.get_parameter('csv').value
        self.csv = open(self.csv_path, 'w') if self.csv_path else None
        if self.csv:
            self.csv.write('t,dt,gt_dist,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,'
                           'err_pos,err_yaw_deg\n')

        self.create_subscription(Odometry, self.get_parameter('truth_topic').value,
                                 self.on_truth, QOS)
        self.create_subscription(Odometry, self.get_parameter('estimate_topic').value,
                                 self.on_est, QOS)
        self.create_timer(float(self.get_parameter('report_period').value), self.report)
        self.get_logger().info(
            f"比對 {self.get_parameter('estimate_topic').value} vs "
            f"{self.get_parameter('truth_topic').value} (ground truth)")

    def on_truth(self, m: Odometry):
        t = stamp_sec(m.header.stamp)
        p = m.pose.pose.position
        if self._last_gt is not None:
            self.gt_dist += math.hypot(p.x - self._last_gt[0], p.y - self._last_gt[1])
        self._last_gt = (p.x, p.y)
        self.truth.append((t, p.x, p.y, yaw_of(m.pose.pose.orientation), self.gt_dist))
        if len(self.truth) > 40000:
            del self.truth[:20000]

    def on_est(self, m: Odometry):
        t = stamp_sec(m.header.stamp)
        g = self._truth_at(t)
        if g is None:
            return
        if self.t_start is None:
            self.t_start = t
        p = m.pose.pose.position
        e = (p.x, p.y, yaw_of(m.pose.pose.orientation))
        self.pairs.append((t, g[0], g[1], g[2], e[0], e[1], e[2], g[3]))
        if self.csv:
            err = math.hypot(e[0] - g[0], e[1] - g[1])
            self.csv.write(f'{t:.6f},{t - self.t_start:.3f},{g[3]:.4f},'
                           f'{g[0]:.6f},{g[1]:.6f},{g[2]:.6f},'
                           f'{e[0]:.6f},{e[1]:.6f},{e[2]:.6f},{err:.6f},'
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
                a[3] + k * wrap(b[3] - a[3]), a[4] + k * (b[4] - a[4]))

    # ------------------------------------------------------------------
    def report(self, final=False):
        if len(self.pairs) < 20:
            self.get_logger().info(f'樣本 {len(self.pairs)} 筆, 還不夠')
            return
        a = np.array(self.pairs)
        rel = a[:, 0] - a[0, 0]
        ep = np.hypot(a[:, 4] - a[:, 1], a[:, 5] - a[:, 2])
        eyaw = np.degrees([wrap(v) for v in (a[:, 6] - a[:, 3])])
        dist = a[:, 7] - a[0, 7]

        self.get_logger().info(
            f"[{'final' if final else 'live '}] 跑了 {rel[-1]:.1f} s / "
            f'{dist[-1]:.1f} m | 現在誤差 {ep[-1]:.3f} m '
            f'(最大 {ep.max():.3f} m), yaw {eyaw[-1]:+.2f}° | '
            f'漂移率 {100 * ep[-1] / max(dist[-1], 1e-6):.2f}% 的行走距離')

        if not final:
            return
        print('\n' + '=' * 74)
        print(f'  IMU + 四輪輪速   {rel[-1]:.1f} 秒   ground truth 走了 {dist[-1]:.2f} m')
        print('=' * 74)
        print('  航位推算的誤差跟**走過的距離**走 (純 IMU 是跟時間走), 所以:\n')
        for th in list(self.get_parameter('thresholds').value):
            over = np.nonzero(ep > float(th))[0]
            if over.size:
                i = int(over[0])
                print(f'    誤差第一次超過 {float(th):4.2f} m: 走了 {dist[i]:6.2f} m '
                      f'(第 {rel[i]:6.1f} 秒)')
            else:
                print(f'    誤差第一次超過 {float(th):4.2f} m: 整段都沒有超過 '
                      f'(最大 {ep.max():.3f} m)')

        print(f'\n  漂移率: 結束時 {100 * ep[-1] / max(dist[-1], 1e-6):.2f}%, '
              f'用最大誤差算 {100 * ep.max() / max(dist[-1], 1e-6):.2f}% '
              f'(最大 {ep.max():.3f} m)')
        print(f'  yaw 誤差: 結束時 {eyaw[-1]:+.2f}°, 最大 {np.abs(eyaw).max():.2f}°')
        print(f'  走了 {dist[-1]:.1f} m 而 yaw 差 {np.abs(eyaw).max():.2f}° '
              f'-> 光是 yaw 就能造成 '
              f'{dist[-1] * math.radians(np.abs(eyaw).max()):.2f} m 的側向誤差')
        print('  (長期而言 yaw 是主導項, 不是輪速的尺度誤差 —— 誤差如果遠大於')
        print('   上面那個數字, 才輪到懷疑 wheel_scale。)')

        print('\n  誤差 vs 走過的距離:')
        n = len(ep)
        for f in (0.1, 0.25, 0.5, 0.75, 1.0):
            i = min(int(n * f), n - 1)
            print(f'    走了 {dist[i]:6.2f} m (第 {rel[i]:6.1f} 秒): '
                  f'誤差 {ep[i]:7.3f} m ({100 * ep[i] / max(dist[i], 1e-6):5.2f}%), '
                  f'yaw {eyaw[i]:+7.2f}°')

        # 停車段: 輪速最直接的好處就是「車子沒動 -> 誤差不長」
        gv = np.zeros(n)
        gv[1:] = np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])) / np.maximum(
            np.diff(a[:, 0]), 1e-6)
        still = gv < float(self.get_parameter('still_speed').value)
        if still.sum() > 10:
            d_still = float(np.abs(np.diff(ep))[still[1:]].sum())
            d_move = float(np.abs(np.diff(ep))[~still[1:]].sum())
            print(f'\n  ground truth 有 {100 * float(still.mean()):.0f}% 的時間靜止。')
            print(f'    靜止時誤差總變動 {d_still:.3f} m, 行駛時 {d_move:.3f} m')
            print('    (輪速最直接的好處: 車子沒動時輪速說速度是 0, 誤差**應該幾乎')
            print('     不長**。靜止那一欄如果跟行駛那一欄同量級, 表示 ZUPT 或輪速')
            print('     沒有在動 —— 去看節點 status 那行的「靜止 xx%」。)')
        else:
            print('\n  ground truth 幾乎沒有停過車。純 IMU 在這種情況下會自由累積,')
            print('  輪速則不受影響 —— 這正是加輪速最值錢的場景。')
        if self.csv_path:
            print(f'\n  逐點資料已寫到 {self.csv_path}')
        print('=' * 74)


def main(args=None):
    rclpy.init(args=args)
    node = WheelEval()
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
