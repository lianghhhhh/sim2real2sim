#!/usr/bin/env python3
"""量出 collect_data_node 需要的 filtered_rotation_deg / filtered_yaw_offset_deg。

用途: /odometry/filtered 的座標系是「LiDAR 啟動瞬間的朝向」, Isaac ground truth
      /odom 則是世界座標。兩者差一個固定旋轉 —— 這個旋轉只取決於 LiDAR 在 USD
      裡的掛載朝向, 換車或改掛載方式就要重量一次。

用法 (Isaac 在 Play、兩個 launch 都在跑的狀態下):
    ros2 run 一個會讓車子移動的情境, 同時執行:
        ./scripts/measure_odom_alignment.py
    讓車子走超過 1 公尺 (直線或轉彎都可以), 然後 Ctrl-C, 它會印出兩個角度。

注意: Isaac 各 publisher 可能用不同時間源 (差幾百甚至幾千秒), 所以這裡用
      「各自從第一筆算起的相對時間」來配對, 常數偏移會自動抵銷。
      不要用抵達順序配對 —— 兩個 topic 的佇列會漂移出約 1 秒的隨機錯位,
      在轉彎的軌跡上足以讓結果完全失真。
"""
import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry


def yaw_of(o):
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Align(Node):
    def __init__(self, gt_topic, est_topic, min_step):
        super().__init__('measure_odom_alignment')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, gt_topic, self.gt_cb, qos)
        self.create_subscription(Odometry, est_topic, self.est_cb, qos)
        self.min_step = min_step
        self.gt = []             # (相對時間, x, y, yaw)
        self.est = []
        self.t0 = {}
        self.get_logger().info(f'比對 {est_topic} vs {gt_topic}; 請開著車子走一段, '
                               f'走夠了按 Ctrl-C')

    def _rel_t(self, key, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        # 兩個 topic 可能用不同時間源, 各自扣掉自己的第一筆 -> 常數偏移抵銷
        self.t0.setdefault(key, t)
        return t - self.t0[key]

    def gt_cb(self, m):
        p = m.pose.pose.position
        self.gt.append((self._rel_t('gt', m), p.x, p.y, yaw_of(m.pose.pose.orientation)))

    def est_cb(self, m):
        p = m.pose.pose.position
        self.est.append((self._rel_t('est', m), p.x, p.y, yaw_of(m.pose.pose.orientation)))
        if len(self.est) % 100 == 0:
            self.get_logger().info(f'已收集 {len(self.est)} 筆估計樣本')

    def _build_pairs(self):
        """用相對時間把 GT 內插到每一筆估計值的時刻。"""
        if len(self.gt) < 2 or len(self.est) < 2:
            return []
        g = np.array(self.gt)
        gt_t = g[:, 0]
        pairs, last = [], None
        for t, ex, ey, ea in self.est:
            if t < gt_t[0] or t > gt_t[-1]:
                continue
            i = int(np.searchsorted(gt_t, t))
            i = max(1, min(i, len(gt_t) - 1))
            t0, t1 = gt_t[i - 1], gt_t[i]
            k = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            gx = g[i - 1, 1] + k * (g[i, 1] - g[i - 1, 1])
            gy = g[i - 1, 2] + k * (g[i, 2] - g[i - 1, 2])
            ga = g[i - 1, 3] + k * wrap(g[i, 3] - g[i - 1, 3])
            cur = (np.array([gx, gy]), np.array([ex, ey]), ga, ea)
            # 只在真的移動了才記錄一筆, 避免靜止時塞滿重複樣本
            if last is None or np.linalg.norm(cur[0] - last) > self.min_step:
                pairs.append(cur)
                last = cur[0]
        return pairs

    def report(self):
        self.pairs = self._build_pairs()
        if len(self.pairs) < 3:
            print('\n樣本不足 (需要車子實際移動)。請讓車子走超過 1 公尺再試。')
            return
        g = np.array([p[0] for p in self.pairs])
        e = np.array([p[1] for p in self.pairs])
        total_gt = float(np.sum(np.linalg.norm(np.diff(g, axis=0), axis=1)))
        # 用「整條軌跡」做 2D Kabsch, 而不是相鄰樣本的微小位移:
        # 兩個 topic 是按抵達順序配對的, 配對誤差跟單步位移同量級, 用微小位移
        # 擬合方向會被雜訊淹沒; 整條軌跡的尺度遠大於配對誤差, 結果穩定得多。
        gc, ec = g - g.mean(axis=0), e - e.mean(axis=0)
        rot = math.atan2(float(np.sum(ec[:, 0] * gc[:, 1] - ec[:, 1] * gc[:, 0])),
                         float(np.sum(ec[:, 0] * gc[:, 0] + ec[:, 1] * gc[:, 1])))
        c, sn = math.cos(rot), math.sin(rot)
        aligned = np.stack([c * ec[:, 0] - sn * ec[:, 1],
                            sn * ec[:, 0] + c * ec[:, 1]], axis=1)
        resid = float(np.sqrt(np.mean(np.sum((aligned - gc) ** 2, axis=1))))
        scale = float(np.sqrt(np.sum(gc ** 2) / max(np.sum(ec ** 2), 1e-12)))
        yaw_off = np.array([wrap(p[2] - p[3]) for p in self.pairs])
        yo = math.atan2(float(np.mean(np.sin(yaw_off))), float(np.mean(np.cos(yaw_off))))
        spread = math.degrees(float(np.std([wrap(a - yo) for a in yaw_off])))

        print(f'\n樣本 {len(self.pairs)} 筆, GT 總行走 {total_gt:.2f} m')
        print(f'  位置旋轉角 = {math.degrees(rot):+7.2f} 度')
        print(f'  yaw 偏移   = {math.degrees(yo):+7.2f} 度  (樣本標準差 {spread:.2f} 度)')
        print(f'  長度比 GT/估計 = {scale:.4f}  (理想 1.0; 明顯偏離代表定位還有問題)')
        print(f'  對齊後殘差 RMS = {resid:.3f} m  (越小代表這個旋轉角越可信)')
        if total_gt < 1.0:
            print('  ⚠ GT 行走不到 1 m, 角度可能不準, 建議多走一點再量')
        if spread > 5.0:
            print('  ⚠ yaw 偏移的離散度偏大, 表示兩邊的旋轉量對不起來 (不只是固定偏移)')
        print(f'\n填進 collect_data_node:')
        print(f'  ros2 run calibrate_env_pkg calibrate_env_node --ros-args \\')
        print(f'      -p filtered_rotation_deg:={math.degrees(rot):.2f} \\')
        print(f'      -p filtered_yaw_offset_deg:={math.degrees(yo):.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gt-topic', default='/odom')
    ap.add_argument('--est-topic', default='/odometry/filtered')
    ap.add_argument('--min-step', type=float, default=0.02,
                    help='GT 位移超過多少公尺才記錄一筆樣本')
    args = ap.parse_args()

    rclpy.init()
    node = Align(args.gt_topic, args.est_topic, args.min_step)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
