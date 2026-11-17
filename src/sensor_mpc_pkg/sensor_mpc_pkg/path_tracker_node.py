"""path_tracker: 感測器位姿 + 路徑檔 -> MPC 的目標點 (取代 Isaac 裡的 temp_curve ScriptNode)。

    ros2 run sensor_mpc_pkg path_tracker --ros-args \\
        -p path_file:=/workspaces/src/sensor_mpc_pkg/config/path_kros_car_mpc.csv \\
        -p odom_topic:=/fusion_loc/odom

輸出 (預設 /sensor_mpc/nearest_points, std_msgs/Float32MultiArray):
    [idx, x1, y1, psi1, x2, y2, psi2, ..., x10, y10, psi10]
idx = 最近的路徑點 (原始點序), psi = 該目標點的**絕對**路徑航向 (度)。排列跟 Isaac 的
/nearest_curve_point 一樣, 只差 psi 是絕對值而不是「路徑航向 - 車子航向」。

另外發:
    /sensor_mpc/path          nav_msgs/Path  整條路徑 (latched, 給 Foxglove / RViz 看)
    /sensor_mpc/targets       nav_msgs/Path  目前的 10 個目標點
    /sensor_mpc/finished      std_msgs/Bool  不封閉的路徑走到終點 (封閉路徑永遠 false)
服務 /path_tracker/reset (std_srvs/Trigger): 重新決定方向與最近點 (車子被搬走時)。

路徑來源二選一: path_file (CSV, 用 scripts/export_path_from_usd.py 從 USD 匯出) 或
path_topic (nav_msgs/Path, 收到就整條換掉並 reset)。座標要跟 odom_topic 同一個 frame
(模擬是 Isaac 的世界座標 = 定位的 map; 實體車是 LiDAR 地圖的 map)。
"""
from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger

from .path_tracker import PathTracker, load_path_csv

ODOM_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pose(x, y, yaw, frame, stamp):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = stamp
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z = math.sin(0.5 * yaw)
    ps.pose.orientation.w = math.cos(0.5 * yaw)
    return ps


class PathTrackerNode(Node):

    def __init__(self):
        super().__init__('path_tracker')
        p = self.declare_parameter
        p('path_file', '')
        p('path_topic', '')                   # nav_msgs/Path; 空 = 不訂閱
        p('odom_topic', '/fusion_loc/odom')
        p('output_topic', '/sensor_mpc/nearest_points')
        p('path_viz_topic', '/sensor_mpc/path')
        p('targets_viz_topic', '/sensor_mpc/targets')
        p('finished_topic', '/sensor_mpc/finished')
        p('frame_id', 'map')
        p('loop', True)                       # 封閉路徑 (走完一圈接著走)
        p('direction', 'auto')                # auto | forward | backward
        p('n_targets', 10)
        p('spacing', 0.1)                     # 目標點間距 (m); MPC 的 path_point_spacing 要一樣
        p('search_radius', 20)                # 最近點只在上一次附近 ±這麼多個點找
        p('far_threshold', 1.5)               # 離路徑超過這麼遠 (m) 就走回歸曲線
        p('goal_tolerance', 0.15)             # 不封閉路徑: 離終點這麼近算到了
        p('rate', 50.0)                       # Hz; 0 = 每一則 odom 算一次
        p('pose_timeout', 0.5)                # 位姿超過這麼久沒更新就不發目標點
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.frame = str(g('frame_id'))
        self.tracker = None
        self.tracker_kw = dict(
            loop=bool(g('loop')), n_targets=int(g('n_targets')), spacing=float(g('spacing')),
            search_radius=int(g('search_radius')), far_threshold=float(g('far_threshold')),
            direction=str(g('direction')), goal_tolerance=float(g('goal_tolerance')))
        self.pose_timeout = float(g('pose_timeout'))
        self.odom = None
        self.t_odom = None
        self._dir_logged = False
        self._recovery = False
        self._finished = False

        self.pub_out = self.create_publisher(Float32MultiArray, g('output_topic'), 10)
        self.pub_path = self.create_publisher(Path, g('path_viz_topic'), LATCHED)
        self.pub_targets = self.create_publisher(Path, g('targets_viz_topic'), 5)
        self.pub_finished = self.create_publisher(Bool, g('finished_topic'), 5)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, ODOM_QOS)
        if g('path_topic'):
            self.create_subscription(Path, g('path_topic'), self.on_path, LATCHED)
        self.create_service(Trigger, '~/reset', self.on_reset)

        if g('path_file'):
            self.set_path(load_path_csv(os.path.expanduser(str(g('path_file')))),
                          f"檔案 {g('path_file')}")
        elif not g('path_topic'):
            raise ValueError('path_file 與 path_topic 至少要給一個')
        else:
            self.get_logger().info(f"等 {g('path_topic')} 送路徑進來")

        rate = float(g('rate'))
        self.timer = self.create_timer(1.0 / rate, self.step) if rate > 0 else None
        self.get_logger().info(
            f"位姿 {g('odom_topic')} -> 目標點 {g('output_topic')} "
            f"({self.tracker_kw['n_targets']} 點, 間距 {self.tracker_kw['spacing']} m, "
            f"{'封閉' if self.tracker_kw['loop'] else '不封閉'}, 方向 {self.tracker_kw['direction']})")

    # ------------------------------------------------------------------ 路徑
    def set_path(self, pts, source):
        self.tracker = PathTracker(pts, **self.tracker_kw)
        self._dir_logged = False
        self._finished = False
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.get_logger().info(
            f'路徑來自 {source}: {len(pts)} 點, 長 {seg.sum():.2f} m, '
            f'x [{pts[:, 0].min():+.2f}, {pts[:, 0].max():+.2f}] '
            f'y [{pts[:, 1].min():+.2f}, {pts[:, 1].max():+.2f}]')
        self.publish_path_viz()

    def publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        pts = self.tracker.pts0
        n = len(pts)
        for k in range(n):
            if self.tracker.loop:
                d = pts[(k + 1) % n] - pts[k]
            else:
                d = pts[k + 1] - pts[k] if k < n - 1 else pts[k] - pts[k - 1]
            msg.poses.append(_pose(pts[k, 0], pts[k, 1], math.atan2(d[1], d[0]),
                                   self.frame, msg.header.stamp))
        if self.tracker.loop:
            msg.poses.append(msg.poses[0])
        self.pub_path.publish(msg)

    def on_path(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn('收到的 Path 不到 2 個點, 忽略')
            return
        pts = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses])
        self.set_path(pts, f'topic ({msg.header.frame_id or "?"} frame)')

    def on_reset(self, req, res):
        if self.tracker is not None:
            self.tracker.reset()
            self._dir_logged = False
            self._finished = False
        res.success = True
        res.message = 'path tracker reset'
        return res

    # ------------------------------------------------------------------ 位姿
    def on_odom(self, msg: Odometry):
        self.odom = msg
        self.t_odom = self.get_clock().now().nanoseconds * 1e-9
        if self.timer is None:
            self.step()

    def step(self):
        if self.tracker is None or self.odom is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.t_odom > self.pose_timeout:
            self.get_logger().warn(f'位姿 {now - self.t_odom:.1f} s 沒更新, 不發目標點',
                                   throttle_duration_sec=2.0)
            return
        pose = self.odom.pose.pose
        xy = (pose.position.x, pose.position.y)
        yaw = yaw_of(pose.orientation)
        r = self.tracker.update(xy, yaw)

        if not self._dir_logged:
            self._dir_logged = True
            self.get_logger().info(
                f"方向: {'順著點序 (FORWARD)' if self.tracker.forward else '倒過來 (BACKWARD)'}, "
                f"起點離路徑 {r['dist']:.2f} m (idx {r['idx']})")
        if r['recovery'] != self._recovery:
            self._recovery = r['recovery']
            if r['recovery']:
                self.get_logger().warn(f"離路徑 {r['dist']:.2f} m > far_threshold, 走回歸曲線")
            else:
                self.get_logger().info('回到路徑附近, 恢復正常循跡')
        if r['finished'] and not self._finished:
            self.get_logger().info('走到終點')
        self._finished = r['finished']

        out = Float32MultiArray()
        data = [float(r['idx'])]
        for (x, y), psi in zip(r['targets'], r['headings']):
            data += [float(x), float(y), float(math.degrees(psi))]
        out.data = data
        self.pub_out.publish(out)

        fin = Bool()
        fin.data = bool(r['finished'])
        self.pub_finished.publish(fin)

        tv = Path()
        tv.header.frame_id = self.frame
        tv.header.stamp = self.odom.header.stamp
        for (x, y), psi in zip(r['targets'], r['headings']):
            tv.poses.append(_pose(x, y, psi, self.frame, tv.header.stamp))
        self.pub_targets.publish(tv)


def main():
    rclpy.init()
    node = PathTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
