#!/usr/bin/env python3
"""拿 Isaac 的 ground truth /odom 當尺, 量方法一 (相機) 到底差幾公分。

    ros2 run car_loc_camera camera_loc_eval
    ros2 run car_loc_camera camera_loc_eval --ros-args -p csv:=/workspaces/car_run_data/cam_eval.csv

除了位置/朝向誤差, 這裡還特別報兩件只有相機那條路才會發生的事:

* **輸出中斷。** YOLO 漏偵測 / 車子被柱子擋住的時候輸出會停。
  濾波器會繼續推, 但推久了就不能信 —— 中斷長度分布比平均誤差更能說明
  「這條路在這個場地能不能用」。
* **常數偏移。** 校正檔的單應性如果原點沒對準, 誤差會是一個不會變的平移,
  那不是「定位在漂」, 是校正該重做。兩者要分開報, 不然會誤判。

yaw 誤差如果穩定地是 90 度左右, 先去看 camera_loc.yaml 的 `yaw_offset_deg`,
不要先懷疑定位 —— 濾波器只算得出**行進方向**, base_link 的 x 軸不一定就是
行進方向 (car.usd 差 90 度)。那個參數沒設對的話這裡會報一個假的 90 度誤差。

估計值的 topic 預設是 /camera_loc/odom, 也就是**影像時刻**的狀態配影像時刻的
時戳 —— 這裡是依時戳把真值內插過去比的, 所以要比的就是它。
不要改成 /camera_loc/odom_now: 那是外推到現在的版本, 給 TF / nav 用的,
拿它來比會把外推誤差算進校正誤差裡。
"""
from __future__ import annotations

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=20)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class CameraEval(Node):

    def __init__(self):
        super().__init__('camera_loc_eval')
        p = self.declare_parameter
        p('truth_topic', '/odom')
        p('estimate_topic', '/camera_loc/odom')
        p('report_period', 5.0)
        p('max_pair_dt', 0.05)
        p('gap_threshold', 0.3)     # 輸出間隔超過這個就算一次中斷 (s)
        p('csv', '')

        self.truth = []             # (t, x, y, yaw)
        self.pairs = []             # (t, gx, gy, gyaw, ex, ey, eyaw)
        self.gaps = []
        self.last_est_t = None

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

    # ------------------------------------------------------------------
    def on_truth(self, m: Odometry):
        p = m.pose.pose.position
        self.truth.append((stamp_sec(m.header.stamp), p.x, p.y,
                           yaw_of(m.pose.pose.orientation)))
        if len(self.truth) > 20000:
            del self.truth[:10000]

    def on_est(self, m: Odometry):
        t = stamp_sec(m.header.stamp)
        if self.last_est_t is not None:
            dt = t - self.last_est_t
            if dt > float(self.get_parameter('gap_threshold').value):
                self.gaps.append(dt)
        self.last_est_t = t

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
        """把 ground truth 內插到估計值的時刻。時間對不上就不配對 —— 寧可少幾筆
        樣本, 也不要拿差了一個 frame 的真值去算誤差 (那會直接變成假的誤差)。"""
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

    # ------------------------------------------------------------------
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
        span = float(a[-1, 0] - a[0, 0])

        pr, pm, p95, pmx = self._stats(ep)
        yr, _, _, ymx = self._stats(eyaw)
        self.get_logger().info(
            f"[{'final' if final else 'live '}] {len(self.pairs)} 筆, "
            f'GT 走了 {travelled:.2f} m | 位置誤差 RMS {pr * 100:.2f} cm, '
            f'平均 {pm * 100:.2f} cm, p95 {p95 * 100:.2f} cm | '
            f'yaw RMS {yr:.2f}°, 最大 {ymx:.2f}° | 中斷 {len(self.gaps)} 次')

        if not final:
            return
        print('\n' + '=' * 74)
        print(f'  方法一 (相機 + YOLO)   樣本 {len(self.pairs)} 筆   '
              f'{span:.1f} s   GT 總行走 {travelled:.2f} m')
        print('=' * 74)
        for name, v, unit, k in (('位置誤差 |dp|', ep, 'cm', 100),
                                 ('  其中 dx  ', ex, 'cm', 100),
                                 ('  其中 dy  ', ey, 'cm', 100),
                                 ('yaw 誤差    ', eyaw, 'deg', 1)):
            r, m, q, x = self._stats(v)
            print(f'  {name}:  RMS {r * k:8.3f} {unit}   平均 {m * k:8.3f} {unit}   '
                  f'p95 {q * k:8.3f} {unit}   最大 {x * k:8.3f} {unit}')

        bx, by = float(np.mean(ex)), float(np.mean(ey))
        dr = np.hypot(ex - bx, ey - by)
        r, _, _, x = self._stats(dr)
        print(f'\n  常數偏移: dx {bx * 100:+.2f} cm, dy {by * 100:+.2f} cm')
        print(f'  扣掉常數偏移後: RMS {r * 100:.3f} cm, 最大 {x * 100:.3f} cm')
        if math.hypot(bx, by) > 0.03:
            print('  (常數偏移大而扣掉後小 = 校正檔的單應性原點沒對準, 不是定位在漂。')
            print('   重新擬合校正檔, 或直接把這個偏移併進 homography 的第三行。)')

        print(f'\n  輸出: {len(self.pairs) / max(span, 1e-6):.1f} Hz')
        if self.gaps:
            g = np.asarray(self.gaps)
            print(f'  中斷 {len(g)} 次 (>{float(self.get_parameter("gap_threshold").value)} s): '
                  f'最長 {g.max():.2f} s, 合計 {g.sum():.2f} s '
                  f'({100 * g.sum() / max(span, 1e-6):.1f}% 的時間)')
            print('  (中斷 = YOLO 連續漏偵測。這條路的可用性由它決定, 不是由平均誤差。)')
        else:
            print('  沒有輸出中斷 —— 整段都看得到車。')
        if self.csv_path:
            print(f'\n  逐點資料已寫到 {self.csv_path}')
        print('=' * 74)


def main(args=None):
    rclpy.init(args=args)
    node = CameraEval()
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
