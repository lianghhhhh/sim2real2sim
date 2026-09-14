#!/usr/bin/env python3
"""繞著房間邊緣開一圈 (或幾圈), 發 /cmd_vel。建圖或評估定位時當「可重複的駕駛」用。

    ros2 run car_teleop cmd_vel_bridge &          # /cmd_vel -> /joint_command
    python3 /workspaces/scripts/tour_drive.py --loops 2

為什麼要另外寫一個而不是用 teleop_key: 鍵盤要 TTY, 而且每次開的路徑都不一樣,
不同方法就沒得比。這支用 Isaac 的 ground truth /odom 做路徑點追蹤, 每次跑的
軌跡幾乎一樣 —— **它只是拿真值來開車, 不參與定位, 不影響評估的公正性。**

路徑點是從地圖的距離場挑出來的 (淨空 >= 0.75 m), 四個角落都繞得到, 而且回到
起點, 給 slam_toolbox 一個可以閉的回環。
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)

# 從 car_usd.npz 的距離場**規劃**出來的一圈 (不是憑感覺挑的)。
#
# 憑感覺挑會出事: 第一版我照著地圖的外框挑了六個角落, 結果 y=3.0 那裡有一道牆,
# 三個路徑點落在牆的另一邊 —— 車子朝著牆頂了四分鐘, 而且因為沒有卡住偵測,
# 整條建圖流程就這樣無聲地掛住。實際上車子出生的那個房間只有
# x -4.58..4.58, y -2.58..2.58, 比 car.usd 的整張圖小很多。
#
# 產生方式: 對淨空 > 0.45 m 的格子從車子出生點做連通區域填充, 取四個象限裡
# 離中心最遠的點當角落, 再用 BFS 在格子上把相鄰角落接起來, 每 0.9 m 取一點。
# 這樣每一段都保證走得通 (實測所有段的最小淨空 0.39 m)。
WAYPOINTS = [
    (-3.13, -1.47), (-3.13, -0.52), (-3.13, 0.43),
    (-3.28, 1.28), (-3.92, 1.93), (-4.33, 2.33),
    (-3.38, 2.33), (-2.53, 2.12), (-1.78, 1.73),
    (-0.88, 1.68), (0.08, 1.68), (0.98, 1.68),
    (1.92, 1.68), (2.82, 1.68), (3.73, 1.73),
    (4.33, 2.33), (4.32, 1.38), (4.32, 0.43),
    (4.32, -0.47), (4.33, -1.12), (3.42, -1.12),
    (2.52, -1.12), (1.62, -1.12), (0.67, -1.12),
    (-0.23, -1.12), (-1.17, -1.12), (-2.08, -1.12),
    (-2.73, -1.78), (-3.28, -2.32),
]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Tour(Node):
    def __init__(self, loops, v_max, w_max, tol, stuck_timeout=25.0,
                 forward_axis_deg=-90.0):
        super().__init__('tour_drive')
        # /odom 的 yaw 是 base_link +X 的方向, 但 car.usd 的 base_link 是 +X 朝左、
        # 車頭 -Y。cmd_vel_bridge 的 linear.x 是「車頭方向」, 所以追點要用車頭的
        # 方向算角度誤差, 不是 base_link +X —— 差 90° 的話車子會繞著路徑點打轉,
        # 一路甩到牆上。跟 control_car_node 的 forward_axis_deg 是同一件事。
        self.fwd_off = math.radians(forward_axis_deg)
        self._check = None  # (x, y, heading) 用來驗證車頭方向真的對
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, SENSOR_QOS)
        self.pose = None
        self.i = 0
        self.loops = loops
        self.done_loops = 0
        self.v_max, self.w_max, self.tol = v_max, w_max, tol
        self.finished = False
        # 卡住偵測: 撞到東西又沒有避障的話, 純追點會朝著牆頂到天荒地老。
        # 第一版沒有這個, 建圖流程就這樣無聲掛了四分鐘才被發現。
        self.stuck_timeout = stuck_timeout
        self._best_dist = float('inf')
        self._last_progress = None
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'繞場 {loops} 圈, {len(WAYPOINTS)} 個路徑點, 上限 {v_max} m/s')

    def on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, wrap(yaw + self.fwd_off))

    def tick(self):
        if self.pose is None or self.finished:
            return
        x, y, yaw = self.pose
        tx, ty = WAYPOINTS[self.i]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_progress is None or dist < self._best_dist - 0.05:
            self._best_dist, self._last_progress = dist, now
        elif now - self._last_progress > self.stuck_timeout:
            self.get_logger().warn(
                f'{self.stuck_timeout:.0f} 秒沒有靠近路徑點 {self.i} '
                f'{WAYPOINTS[self.i]} (還差 {dist:.2f} m), 跳過它。'
                '車子大概頂在什麼東西上 —— 這支沒有避障。')
            self._advance()
            return

        if dist < self.tol:
            self._advance()
            return
        err = wrap(math.atan2(dy, dx) - yaw)
        t = Twist()
        # 角度差太大就先原地轉, 免得畫大圈撞到東西
        t.angular.z = max(-self.w_max, min(self.w_max, 1.5 * err))
        t.linear.x = 0.0 if abs(err) > 0.7 else min(self.v_max, 0.8 * dist) \
            * (1.0 - min(abs(err) / 0.7, 1.0) * 0.6)
        self.pub.publish(t)
        self._check_heading(x, y, yaw, t.linear.x)

    def _check_heading(self, x, y, heading, v_cmd):
        """直線前進時, 實際位移方向應該跟算出來的車頭方向一致。
        不一致代表 --forward-axis-deg 給錯, 繼續開只會撞牆, 所以直接停。"""
        if v_cmd < 0.2:
            self._check = None
            return
        if self._check is None:
            self._check = (x, y, heading)
            return
        x0, y0, h0 = self._check
        moved = math.hypot(x - x0, y - y0)
        if moved < 0.3:
            return
        self._check = None
        if abs(wrap(heading - h0)) > 0.3:   # 中間有在轉, 這段不準
            return
        off = wrap(math.atan2(y - y0, x - x0) - h0)
        if abs(off) > math.radians(45):
            self.get_logger().error(
                f'實際行進方向跟車頭差 {math.degrees(off):+.0f}° —— '
                '--forward-axis-deg 大概給錯了 (試試加上這個角度)。先停車。')
            self.finished = True
            self.pub.publish(Twist())


    def _advance(self):
        self._best_dist, self._last_progress = float('inf'), None
        self.i += 1
        if self.i >= len(WAYPOINTS):
            self.i = 0
            self.done_loops += 1
            self.get_logger().info(f'完成第 {self.done_loops} 圈')
            if self.done_loops >= self.loops:
                self.finished = True
                self.pub.publish(Twist())
                return
        self.get_logger().info(f'-> 路徑點 {self.i} {WAYPOINTS[self.i]}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--loops', type=int, default=1)
    ap.add_argument('--v-max', type=float, default=0.55)
    ap.add_argument('--w-max', type=float, default=0.9)
    ap.add_argument('--tol', type=float, default=0.35)
    ap.add_argument('--stuck-timeout', type=float, default=25.0,
                    help='這麼久沒有更靠近目標就跳過它 (秒)')
    ap.add_argument('--forward-axis-deg', type=float, default=-90.0,
                    help='base_link +X 量到車頭的角度; car.usd 車頭是 -Y 所以 -90')
    a = ap.parse_args()
    rclpy.init()
    n = Tour(a.loops, a.v_max, a.w_max, a.tol, a.stuck_timeout,
             a.forward_axis_deg)
    try:
        while rclpy.ok() and not n.finished:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    n.pub.publish(Twist())
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
