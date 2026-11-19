#!/usr/bin/env python3
"""不開 Isaac 測整條流程用的假世界 (要 torch, 在 sim2real2sim 容器裡跑):

    python3 fake_world.py --ros-args -p obstacle_trigger_dist:=0.6

* 車子 = 動態模型自己 (mlp_control_pkg.offline_check.Plant, 跟 drive_paths_offline 一樣), 吃 /joint_command
* 發 /fusion_loc/odom (+ /fusion_loc/abs_age)、TF map -> base_link、base_link -> laser_frame、/joint_states
* /scan: 對 car_loc_lidar/maps/room.pgm 的牆 / 圓柱 + 假障礙物 (方塊) 做射線, 10 Hz, 360 條
* 障礙物: 車子離起點超過 obstacle_trigger_dist 時, 在目前路線 (/nav_mpc/path) 上、車子前方 obstacle_ahead
  公尺的地方放一個 obstacle_size 的方塊 (只放一次)。也可以發 /fake/obstacle [x, y, 邊長] 自己放
* 結束 (Ctrl-C) 時印: 車子離障礙物表面最近多少、有沒有撞到

use_sim_time 不用 (牆上時間), 所以 nav_mpc.launch.py 要 use_sim_time:=false (不會開 actuator_model)。
"""
import math
import os
import sys

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float32, Float32MultiArray

from mlp_control_pkg.offline_check import Plant
from mlp_control_pkg.utils import NS, loadConfig, loadTorchModel

WHEELS = ['front_left_joint', 'front_right_joint', 'rear_left_joint', 'rear_right_joint']
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)


def load_grid(yaml_path):
    import PIL.Image
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = np.array(PIL.Image.open(os.path.join(os.path.dirname(yaml_path), meta['image'])))
    occ = img < 100                         # 黑 = 牆 (trinary: 0 佔用, 254 空, 205 未知)
    return occ[::-1].copy(), float(meta['resolution']), meta['origin'][:2]


class FakeWorld(Node):

    def __init__(self):
        super().__init__('fake_world')
        p = self.declare_parameter
        p('map_yaml', '')
        p('start', [0.0, 0.0, 0.0])
        p('obstacle_trigger_dist', 0.6)         # 車子離起點超過這麼遠就放障礙物; 很大 = 不放
        p('obstacle_ahead', 1.8)                # m, 沿路線在車子前方多遠
        p('obstacle_size', 0.4)                 # m, 方塊邊長
        p('laser_x', 0.045)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.occ, self.res, self.origin = load_grid(g('map_yaml'))
        cfg = loadConfig(None)
        self.dt = float(cfg['dt'])
        s0 = np.zeros(NS)
        s0[:3] = g('start')
        self.plant = Plant(loadTorchModel(cfg), s0)
        self.u = np.zeros(2)
        self.trigger = float(g('obstacle_trigger_dist'))
        self.start = np.array(g('start')[:2], dtype=float)
        self.ahead = float(g('obstacle_ahead'))
        self.size = float(g('obstacle_size'))
        self.laser_x = float(g('laser_x'))
        self.boxes = []                         # (cx, cy, half)
        self.path = None
        self.paths = []                         # 收到的每一條路線 (結束時存 fake_world_paths.npz)
        self.traj = []                          # (t, x, y, yaw)
        self.min_clear = math.inf

        self.pub_odom = self.create_publisher(Odometry, '/fusion_loc/odom', 10)
        self.pub_age = self.create_publisher(Float32, '/fusion_loc/abs_age', 10)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.tf = tf2_ros.TransformBroadcaster(self)
        self.stf = tf2_ros.StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id, st.child_frame_id = 'base_link', 'laser_frame'
        st.transform.translation.x, st.transform.translation.z = self.laser_x, 0.236
        st.transform.rotation.w = 1.0
        self.stf.sendTransform(st)
        self.create_subscription(JointState, '/joint_command', self.on_cmd, 10)
        self.create_subscription(Path, '/nav_mpc/path', self.on_path, LATCHED)
        self.create_subscription(Float32MultiArray, '/fake/obstacle', self.on_obstacle, 10)
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.scan)
        self.get_logger().info(f'假世界: 車子從 {g("start")} 出發, 離起點 > {self.trigger} m 時在路線前方 '
                               f'{self.ahead} m 放 {self.size} m 方塊')

    def on_cmd(self, msg):
        v = dict(zip(msg.name, msg.velocity))
        self.u = np.array([v.get('front_left_joint', 0.0), v.get('front_right_joint', 0.0)])

    def on_path(self, msg):
        self.path = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses])
        self.paths.append(self.path)

    def on_obstacle(self, msg):
        x, y, a = msg.data[:3]
        self.boxes.append((x, y, a / 2))
        self.get_logger().warn(f'放障礙物 ({x:+.2f}, {y:+.2f}) 邊長 {a:.2f}')

    def _maybe_spawn(self, s):
        if self.boxes or self.path is None or np.linalg.norm(s[:2] - self.start) < self.trigger:
            return
        d = np.linalg.norm(self.path - s[:2], axis=1)
        k = int(np.argmin(d))
        acc = 0.0
        while k < len(self.path) - 1 and acc < self.ahead:
            acc += np.linalg.norm(self.path[k + 1] - self.path[k])
            k += 1
        x, y = self.path[k]
        self.boxes.append((x, y, self.size / 2))
        self.get_logger().warn(f'放障礙物 ({x:+.2f}, {y:+.2f}) 邊長 {self.size:.2f} '
                               f'(車子在 ({s[0]:+.2f}, {s[1]:+.2f}), 沿路線前方 {acc:.2f} m)')

    def step(self):
        s = self.plant.step(self.u)
        stamp = self.get_clock().now().to_msg()
        x, y, th = s[0], s[1], s[2]
        self.traj.append((self.get_clock().now().nanoseconds * 1e-9, x, y, th))
        self._maybe_spawn(s)
        # 車身 (0.39 x 0.33 m 的矩形) 離方塊表面最近多少
        for cx, cy, h in self.boxes:
            c, sn = math.cos(th), math.sin(th)
            corners = [(x + c * a - sn * b, y + sn * a + c * b)
                       for a in (-0.195, 0.0, 0.195) for b in (-0.165, 0.0, 0.165)]
            for px, py in corners:
                dx, dy = max(abs(px - cx) - h, 0.0), max(abs(py - cy) - h, 0.0)
                self.min_clear = min(self.min_clear, math.hypot(dx, dy))
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id, od.child_frame_id = 'map', 'base_link'
        od.pose.pose.position.x, od.pose.pose.position.y = float(x), float(y)
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        od.pose.covariance[0] = od.pose.covariance[7] = 1e-4
        od.pose.covariance[35] = 1e-4
        od.twist.twist.linear.x, od.twist.twist.linear.y = float(s[3]), float(s[4])
        od.twist.twist.angular.z = float(s[5])
        self.pub_odom.publish(od)
        self.pub_age.publish(Float32(data=0.05))
        js = JointState()
        js.header.stamp = stamp
        js.name = list(WHEELS)
        js.velocity = [float(v) for v in s[6:10]]
        self.pub_js.publish(js)
        t = TransformStamped()
        t.header = od.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = float(x), float(y)
        t.transform.rotation = od.pose.pose.orientation
        self.tf.sendTransform(t)

    def scan(self):
        s = self.plant.S[-1]
        th = s[2]
        lx, ly = s[0] + self.laser_x * math.cos(th), s[1] + self.laser_x * math.sin(th)
        n = 360
        ang = -math.pi + np.arange(n) * (2 * math.pi / n)
        rs = np.arange(0.05, 8.0, self.res * 0.8)
        wa = th + ang
        px = lx + np.outer(np.cos(wa), rs)
        py = ly + np.outer(np.sin(wa), rs)
        ix = ((px - self.origin[0]) / self.res).astype(int)
        iy = ((py - self.origin[1]) / self.res).astype(int)
        ok = (ix >= 0) & (iy >= 0) & (ix < self.occ.shape[1]) & (iy < self.occ.shape[0])
        hit = np.zeros_like(ok)
        hit[ok] = self.occ[iy[ok], ix[ok]]
        for cx, cy, h in self.boxes:
            hit |= (np.abs(px - cx) <= h) & (np.abs(py - cy) <= h)
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        ranges = np.where(first >= 0, rs[np.maximum(first, 0)], np.inf)
        ranges = ranges + np.random.normal(0.0, 0.005, n) * np.isfinite(ranges)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min, msg.angle_increment = -math.pi, 2 * math.pi / n
        msg.angle_max = msg.angle_min + (n - 1) * msg.angle_increment
        msg.scan_time, msg.range_min, msg.range_max = 0.1, 0.05, 12.0
        msg.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(msg)

    def report(self):
        np.savez('fake_world_paths.npz', traj=np.array(self.traj), boxes=np.array(self.boxes),
                 **{f'path{i}': pth for i, pth in enumerate(self.paths)})
        s = self.plant.S[-1]
        self.get_logger().info(f'結束位姿 x={s[0]:+.3f} y={s[1]:+.3f} yaw={math.degrees(s[2]):+.1f}°; '
                               + (f'車身離障礙物最近 {self.min_clear:.3f} m'
                                  + (' **撞到了**' if self.min_clear <= 0.0 else '')
                                  if self.boxes else '沒有放障礙物'))


def main():
    rclpy.init(args=sys.argv)
    node = FakeWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
