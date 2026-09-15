#!/usr/bin/env python3
"""量融合定位的誤差 —— 拿 Isaac 的 ground truth `/odom` 當尺。

    ros2 run car_loc_fusion fusion_loc_eval
    ros2 run car_loc_fusion fusion_loc_eval --ros-args -p csv:=/workspaces/car_run_data/fusion_eval.csv

Ctrl-C 印總結。

**融合要看的東西跟航位推算不一樣。**
`car_loc_wheel` 的 evaluate 報「漂移率 (誤差/距離)」, 因為那條路的誤差是單調
長大的。融合有絕對參考, 誤差**不隨時間長大**, 所以要看的是分布 (RMS / p95) 與
**尾巴**: 最差的那幾次是什麼時候發生的, 以及濾波器自己知不知道。

四件事:

1. **常數偏移**。整段誤差的平均向量。不是 0 的話通常不是定位在漂, 是座標系沒對
   (地圖原點、相機校正的原點)。它會把 RMS 整個抬高, 但那是一個減一下就好的東西 ——
   所以要單獨報, 不要混在 RMS 裡。
2. **誤差 vs 車速**。延遲補償沒做對的話, 誤差會跟車速成正比 (v x delay), 靜止時
   卻很小。這張表比單一個 RMS 更能指出病因。
3. **一致性 (誤差 / 濾波器自己報的 sigma)**。理想是 ~1。遠大於 1 = 過度自信
   (接下來它會開始擋掉正確的量測), 遠小於 1 = 過度保守 (絕對量測進不來)。
   **這是融合特有的檢查** —— 單一方法的 sigma 只反映它自己, 融合的 sigma 是
   「所有來源合起來還剩多少不確定」, 錯了會直接害到下游。
4. **輸出有沒有斷**。融合的賣點就是不會斷: 絕對量測斷了還有遞推。真的斷了要知道。
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
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class FusionEval(Node):

    def __init__(self):
        super().__init__('fusion_loc_eval')
        p = self.declare_parameter
        p('truth_topic', '/odom')
        p('estimate_topic', '/fusion_loc/odom')
        p('report_period', 5.0)
        p('max_pair_dt', 0.05)
        p('gap_warn', 0.5)
        p('csv', '')

        self.truth = []          # (t, x, y, yaw, speed)
        self.rows = []           # (t, ex, ey, eyaw, sigma, speed)
        self._last_est_t = None
        self.gaps = []

        self.create_subscription(Odometry, self.get_parameter('truth_topic').value,
                                 self.on_truth, QOS)
        self.create_subscription(Odometry,
                                 self.get_parameter('estimate_topic').value,
                                 self.on_est, QOS)
        self.create_timer(float(self.get_parameter('report_period').value),
                          self.report)
        self.get_logger().info(
            f"比 {self.get_parameter('estimate_topic').value} vs "
            f"{self.get_parameter('truth_topic').value} (ground truth)")

    def on_truth(self, msg: Odometry):
        t = stamp_sec(msg.header.stamp)
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.truth.append((t, p.x, p.y, yaw_of(msg.pose.pose.orientation),
                           math.hypot(v.x, v.y)))
        if len(self.truth) > 20000:
            del self.truth[:10000]

    def on_est(self, msg: Odometry):
        t = stamp_sec(msg.header.stamp)
        if self._last_est_t is not None:
            gap = t - self._last_est_t
            if gap > float(self.get_parameter('gap_warn').value):
                self.gaps.append((self._last_est_t, gap))
        self._last_est_t = t

        # **依時戳內插對齊**, 不要拿「最新的一筆 truth」直接減 —— 車子 1 m/s 時
        # 50 ms 的錯位就是 5 cm, 跟要量的東西同一個量級。
        if len(self.truth) < 2:
            return
        ts = [r[0] for r in self.truth]
        if not (ts[0] <= t <= ts[-1]):
            return
        i = np.searchsorted(ts, t)
        if i == 0 or i >= len(ts):
            return
        t0, x0, y0, yaw0, s0 = self.truth[i - 1]
        t1, x1, y1, yaw1, s1 = self.truth[i]
        if t1 - t0 <= 0 or (t1 - t0) > float(self.get_parameter('max_pair_dt').value) * 4:
            return
        a = (t - t0) / (t1 - t0)
        gx = x0 + a * (x1 - x0)
        gy = y0 + a * (y1 - y0)
        gyaw = yaw0 + a * wrap(yaw1 - yaw0)
        spd = s0 + a * (s1 - s0)

        p = msg.pose.pose.position
        cov = msg.pose.covariance
        sig = math.sqrt(max(cov[0] + cov[7], 0.0))
        self.rows.append((t, p.x - gx, p.y - gy,
                          wrap(yaw_of(msg.pose.pose.orientation) - gyaw), sig, spd))

    # ------------------------------------------------------------------
    def report(self):
        if len(self.rows) < 10:
            self.get_logger().warn(
                f'配對到 {len(self.rows)} 筆 —— 檢查兩個 topic 都在發, '
                '而且時戳在同一個時鐘上 (use_sim_time)')
            return
        r = np.array(self.rows)
        e = np.hypot(r[:, 1], r[:, 2])
        self.get_logger().info(
            f'{len(e)} 筆 | RMS {np.sqrt((e ** 2).mean()) * 100:.2f} cm '
            f'中位 {np.median(e) * 100:.2f} p95 {np.percentile(e, 95) * 100:.2f} '
            f'最大 {e.max() * 100:.2f} | 常數偏移 ({r[:, 1].mean() * 100:+.1f}, '
            f'{r[:, 2].mean() * 100:+.1f}) cm | yaw 中位 '
            f'{math.degrees(np.median(np.abs(r[:, 3]))):.2f}°')

    def summary(self):
        if len(self.rows) < 10:
            print('資料太少, 沒有東西可以報')
            return
        r = np.array(self.rows)
        e = np.hypot(r[:, 1], r[:, 2])
        bias = np.array([r[:, 1].mean(), r[:, 2].mean()])
        ec = np.hypot(r[:, 1] - bias[0], r[:, 2] - bias[1])
        line = '=' * 78
        print('\n' + line)
        print(f'  融合定位評估 —— {len(e)} 筆配對, {r[-1, 0] - r[0, 0]:.1f} 秒')
        print(line)
        print(f'  位置誤差   RMS {np.sqrt((e ** 2).mean()) * 100:7.2f} cm  '
              f'中位 {np.median(e) * 100:6.2f}  p95 {np.percentile(e, 95) * 100:6.2f}  '
              f'最大 {e.max() * 100:7.2f}')
        print(f'  yaw 誤差   中位 {math.degrees(np.median(np.abs(r[:, 3]))):6.2f}°  '
              f'p95 {math.degrees(np.percentile(np.abs(r[:, 3]), 95)):6.2f}°')
        print(f'\n  常數偏移 ({bias[0] * 100:+.2f}, {bias[1] * 100:+.2f}) cm '
              f'= {np.linalg.norm(bias) * 100:.2f} cm')
        print(f'  扣掉之後 RMS {np.sqrt((ec ** 2).mean()) * 100:.2f} cm')
        if np.linalg.norm(bias) > 0.03:
            print('  ** 偏移 > 3 cm: 這通常不是定位在漂, 是座標系沒對 (地圖原點 /')
            print('     相機校正原點)。先修那個, 不要調濾波器參數。')

        print('\n  誤差 vs 車速 (延遲補償沒做對的話, 誤差會跟車速成正比)')
        edges = [0.0, 0.1, 0.5, 1.0, 2.0, 99.0]
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = (r[:, 5] >= lo) & (r[:, 5] < hi)
            if m.sum() < 5:
                continue
            print(f'    {lo:4.1f}-{hi:4.1f} m/s  n={m.sum():5d}  '
                  f'RMS {np.sqrt((e[m] ** 2).mean()) * 100:6.2f} cm  '
                  f'p95 {np.percentile(e[m], 95) * 100:6.2f}')

        ok = r[:, 4] > 1e-9
        if ok.sum() > 10:
            ratio = e[ok] / r[ok, 4]
            print(f'\n  一致性 (誤差 / 濾波器自己報的 sigma): 中位 '
                  f'{np.median(ratio):.2f}, p95 {np.percentile(ratio, 95):.2f}')
            print(f'    理想是 ~1。>2 = 過度自信 (接下來會開始擋掉正確的量測);')
            print(f'    <0.3 = 過度保守 (絕對量測進不來)。')

        if self.gaps:
            g = np.array([x[1] for x in self.gaps])
            print(f'\n  ** 輸出中斷 {len(g)} 次, 最長 {g.max():.2f} s **')
            print('     融合不該斷 —— 絕對量測斷了還有遞推。斷了就是 /imu 沒來。')
        else:
            print('\n  輸出沒有中斷 (這是融合相對於單一絕對定位的主要好處)')
        print(line)

        path = self.get_parameter('csv').value
        if path:
            import csv as _csv
            with open(path, 'w', newline='') as f:
                w = _csv.writer(f)
                w.writerow(['t', 'err_x', 'err_y', 'err_yaw', 'sigma', 'gt_speed'])
                w.writerows(self.rows)
            print(f'  逐點資料 -> {path}')


def main(args=None):
    rclpy.init(args=args)
    node = FusionEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
