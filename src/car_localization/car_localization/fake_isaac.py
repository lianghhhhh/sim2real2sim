#!/usr/bin/env python3
"""假的 Isaac —— 照 car.usd 的規格合成 /clock, /imu, /lidar/point_cloud, /odom。

這不是玩具, 是用來把「定位演算法有沒有問題」跟「Isaac 那邊有沒有問題」分開的
工具。開著 Isaac 除錯很痛苦: 點雲不對可能是外參錯、可能是 z 濾波錯、可能是
時間戳對不上, 而你沒有真值可以逐項比對。這個節點的每一項都是已知的:

    * 房間幾何 = /World/Room 的實際尺寸 (內牆 x=±5 / y=±3, 牆高 1 m, 三根 r=0.5 柱子)
    * LiDAR    = SICK multiScan136 的規格 (20 Hz, 675x16 = 10800 點,
                 仰角 -22.5°~+42.5°, 測距雜訊 0.02 m), 掛在車體上方 0.200 m
    * IMU      = 60 Hz, orientation 是精確值 (跟 Isaac 的 IsaacReadIMU 一樣)
    * 一整圈掃描在 50 ms 內掃完, 每個點用它自己那一刻的車體位姿 -> 真的會有
      運動抹除, 拿來驗運動補償有沒有做對

用法:
    ros2 run car_localization fake_isaac
    ros2 run car_localization fake_isaac --ros-args -p motion:=spin -p spin_rate:=8.0

    # 另一個 terminal
    ros2 launch car_localization localization.launch.py evaluate:=true
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, PointCloud2, PointField

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=5)

ROOM = (-5.0, 5.0, -3.0, 3.0, 0.0, 1.0)          # xmin xmax ymin ymax zmin zmax
PILLARS = [(3.8403739996184965, -2.174036853899636, 0.5),
           (-1.4477442018661726, 2.1850387053741924, 0.5),
           (-3.517427517728824, -1.1407622480806512, 0.5)]
PILLAR_TOP = 1.0

# 車子的驅動特性 —— 從 car_run_data/sim_data.csv 回歸出來的:
#     a     = ACCEL_PER_EFFORT * throttle_effort   (m/s^2)
#     alpha = ALPHA_PER_EFFORT * steer_effort      (rad/s^2)
# 注意 R^2 只有 0.17 / 0.02: 那份資料裡有大量撞牆與打滑的樣本, 微分速度又很吵。
# 所以這兩個值只是「數量級對的標稱值」, 不是精確模型。它足夠拿來驗遙控迴路會不會
# 發散、建圖流程順不順, 但真正的手感還是要在 Isaac 裡調。
#
# 最重要的一件事這份資料講得很清楚: **固定 effort 會讓車子一直加速**。
# 這台車幾乎沒有滾動阻力, effort 是扭矩不是速度 —— 所以遙控一定要閉迴路,
# 開迴路按著前進鍵車子會一路加速到撞牆。
ACCEL_PER_EFFORT = 0.34
ALPHA_PER_EFFORT = 0.57
WHEEL_RADIUS = 0.075        # 0.5 (cylinder radius) * 0.15 (scale)
WHEEL_TRACK = 0.25          # 左右輪距 (±0.125)
JOINT_NAMES = ['front_left_joint', 'front_right_joint',
               'rear_left_joint', 'rear_right_joint']


def cast3d(origin, dirs, rng, noise=0.02, rmax=60.0):
    """對房間 (四面牆 + 地板 + 三根柱子) 打射線。沒打到的回 inf。"""
    o = np.asarray(origin, dtype=np.float64)
    d = np.asarray(dirs, dtype=np.float64)
    xmin, xmax, ymin, ymax, zmin, zmax = ROOM
    best = np.full(d.shape[0], np.inf)

    def hit(t, cond):
        nonlocal best
        ok = np.isfinite(t) & (t > 1e-6) & cond
        best = np.where(ok & (t < best), t, best)

    with np.errstate(divide='ignore', invalid='ignore'):
        # 四面牆: 只有打在 z in [0, 1] 而且落在牆的範圍內才算
        for axis, val in ((0, xmin), (0, xmax), (1, ymin), (1, ymax)):
            t = (val - o[axis]) / d[:, axis]
            p = o[None, :] + t[:, None] * d
            other = 1 - axis
            lo, hi = (ymin, ymax) if axis == 0 else (xmin, xmax)
            hit(t, (p[:, 2] >= zmin) & (p[:, 2] <= zmax)
                & (p[:, other] >= lo) & (p[:, other] <= hi))
        # 地板
        t = (0.0 - o[2]) / d[:, 2]
        p = o[None, :] + t[:, None] * d
        hit(t, (p[:, 0] >= xmin) & (p[:, 0] <= xmax)
            & (p[:, 1] >= ymin) & (p[:, 1] <= ymax))
        # 柱子 (垂直圓柱, 高 1 m)
        for cx, cy, r in PILLARS:
            f = o[:2] - np.array([cx, cy])
            a = d[:, 0] ** 2 + d[:, 1] ** 2
            b = 2 * (d[:, :2] @ f)
            c = f @ f - r * r
            disc = b * b - 4 * a * c
            t = (-b - np.sqrt(np.where(disc >= 0, disc, 0))) / (2 * a)
            z = o[2] + t * d[:, 2]
            hit(t, (disc >= 0) & (z >= 0) & (z <= PILLAR_TOP))

    best = np.where(best > rmax, np.inf, best)
    ok = np.isfinite(best)
    best[ok] += rng.normal(0, noise, int(ok.sum()))
    return best


class FakeIsaac(Node):

    def __init__(self):
        super().__init__('fake_isaac')
        p = self.declare_parameter
        # figure8 | circle | spin | still = 照腳本走
        # drive = 吃 /joint_command, 用上面回歸出來的模型跑物理 (可以手動開)
        p('motion', 'figure8')
        p('speed', 0.8)               # m/s
        p('spin_rate', 6.0)           # rad/s, motion=spin 時用
        p('lidar_z', 0.20)
        p('imu_z', 0.075)
        p('scan_hz', 20.0)
        p('imu_hz', 60.0)
        p('n_azimuth', 675)
        p('n_elevation', 16)
        p('range_noise', 0.02)
        p('rate_scale', 1.0)          # 模擬時間跑多快 (1.0 = 即時)
        p('seed', 0)
        p('drag', 0.05)               # 一點點阻尼, 免得數值上完全沒有回復力
        p('start_pose', [0.0, 0.0, 0.0])   # drive 模式的起點 x, y, yaw(度)

        g = self.get_parameter
        self.motion = g('motion').value
        self.speed = float(g('speed').value)
        self.spin_rate = float(g('spin_rate').value)
        self.lidar_z = float(g('lidar_z').value)
        self.scan_dt = 1.0 / float(g('scan_hz').value)
        self.imu_dt = 1.0 / float(g('imu_hz').value)
        self.rng = np.random.default_rng(int(g('seed').value))

        na, ne = int(g('n_azimuth').value), int(g('n_elevation').value)
        # 方位角當外圈 -> 點的索引順序就是發射順序, 跟真的 multiScan136 一樣
        az = np.linspace(0, 2 * np.pi, na, endpoint=False)
        el = np.deg2rad(np.linspace(-22.5, 42.5, ne))
        A, E = np.meshgrid(az, el, indexing='ij')
        self.az = A.ravel()
        self.el = E.ravel()
        self.dir_local = np.stack([np.cos(self.el) * np.cos(self.az),
                                   np.cos(self.el) * np.sin(self.az),
                                   np.sin(self.el)], axis=1)
        self.n_pts = self.dir_local.shape[0]
        self.frac = np.arange(self.n_pts) / max(self.n_pts - 1, 1)
        self.noise = float(g('range_noise').value)

        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu', QOS)
        self.pub_cloud = self.create_publisher(PointCloud2, '/lidar/point_cloud', QOS)
        self.pub_odom = self.create_publisher(Odometry, '/odom', QOS)

        # --- drive 模式的狀態 ---
        self.drive = (self.motion == 'drive')
        sp0 = [float(v) for v in g('start_pose').value]
        self.state = np.array([sp0[0], sp0[1], math.radians(sp0[2]), 0.0, 0.0])
        self.cmd_effort = np.zeros(4)
        self.cmd_time = -1e9
        self.drag = float(g('drag').value)
        self.hist = deque(maxlen=4000)
        self.pub_joint = self.create_publisher(JointState, '/joint_states', QOS)
        if self.drive:
            self.create_subscription(JointState, '/joint_command',
                                     self.on_joint_command, QOS)

        self.t = 0.0
        self.next_scan = self.scan_dt
        self.prev = self.pose_at(-self.imu_dt)
        period = self.imu_dt / max(float(g('rate_scale').value), 1e-3)
        self.create_timer(period, self.tick)
        self.get_logger().info(
            f'假 Isaac: motion={self.motion}, 每圈 {self.n_pts} 點 @ '
            f'{1 / self.scan_dt:.0f} Hz, IMU {1 / self.imu_dt:.0f} Hz'
            + ('\n  drive 模式: 訂 /joint_command, 發 /joint_states。'
               '可以用 car_teleop 手動開。' if self.drive else ''))

    # ------------------------------------------------------------------ 驅動
    def on_joint_command(self, msg: JointState):
        """吃 /joint_command 的 effort。名稱對不上就照順序當 FL/FR/RL/RR。"""
        if not msg.effort:
            return
        e = np.zeros(4)
        if msg.name and len(msg.name) == len(msg.effort):
            table = dict(zip(msg.name, msg.effort))
            for i, n in enumerate(JOINT_NAMES):
                e[i] = float(table.get(n, 0.0))
        else:
            for i in range(min(4, len(msg.effort))):
                e[i] = float(msg.effort[i])
        self.cmd_effort = e
        self.cmd_time = self.t

    def step_physics(self, dt: float):
        x, y, yaw, v, wz = self.state
        if self.t - self.cmd_time > 0.5:     # 指令逾時 -> 鬆油門 (只是不再加速)
            thr = steer = 0.0
        else:
            left = (self.cmd_effort[0] + self.cmd_effort[2]) * 0.5
            right = (self.cmd_effort[1] + self.cmd_effort[3]) * 0.5
            thr = (left + right) * 0.5
            steer = (right - left) * 0.5     # steer > 0 -> wz > 0 (照 car.usd 實測)
        v += (ACCEL_PER_EFFORT * thr - self.drag * v) * dt
        wz += (ALPHA_PER_EFFORT * steer - self.drag * wz) * dt
        yaw = math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))
        # 車頭是 -Y: 世界座標的前進方向 = (sin(yaw), -cos(yaw))
        nx = x + v * math.sin(yaw) * dt
        ny = y - v * math.cos(yaw) * dt
        # 很粗糙的碰撞: 撞到牆或柱子就停下來 (不然車子會開出房間, 雷射就沒東西打了)
        hit = not (ROOM[0] + 0.25 < nx < ROOM[1] - 0.25
                   and ROOM[2] + 0.25 < ny < ROOM[3] - 0.25)
        for cx, cy, r in PILLARS:
            if math.hypot(nx - cx, ny - cy) < r + 0.25:
                hit = True
        if hit:
            v = 0.0
        else:
            x, y = nx, ny
        self.state = np.array([x, y, yaw, v, wz])

    def publish_joint_states(self, stamp):
        """輪速 = 理想無滑移。真車/Isaac 在打滑時輪速會比車速快, 這裡沒有模擬,
        所以用這個測出來的遙控迴路在打滑情境下會比實際樂觀。"""
        v, wz = self.state[3], self.state[4]
        wl = (v - wz * WHEEL_TRACK * 0.5) / WHEEL_RADIUS
        wr = (v + wz * WHEEL_TRACK * 0.5) / WHEEL_RADIUS
        m = JointState()
        m.header.stamp = stamp
        m.name = list(JOINT_NAMES)
        m.velocity = [wl, wr, wl, wr]
        m.position = [0.0] * 4
        self.pub_joint.publish(m)

    # ------------------------------------------------------------------ 軌跡
    def pose_at(self, t: float):
        """回傳 (x, y, yaw)。base_link 的軸向照 car.usd: 車頭 -Y, 左邊 +X。"""
        if self.drive:
            if not self.hist:
                return float(self.state[0]), float(self.state[1]), float(self.state[2])
            ts = [h[0] for h in self.hist]
            i = min(max(int(np.searchsorted(ts, t)), 1), len(ts) - 1)
            t0, t1 = ts[i - 1], ts[i]
            k = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            k = min(max(k, 0.0), 1.0)
            a, b = self.hist[i - 1], self.hist[i]
            dy = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
            return (a[1] + k * (b[1] - a[1]), a[2] + k * (b[2] - a[2]), a[3] + k * dy)
        if self.motion == 'still':
            return 0.0, 0.0, 0.0
        if self.motion == 'spin':
            return 0.0, -0.5, self.spin_rate * t
        if self.motion == 'circle':
            r, w = 2.0, self.speed / 2.0
            return r * math.cos(w * t), r * math.sin(w * t), w * t
        # figure8: 直線、轉彎、加減速都會經歷到
        w = self.speed / 2.5
        return 3.0 * math.sin(w * t), 1.8 * math.sin(2 * w * t), 1.2 * math.sin(w * t)

    # ------------------------------------------------------------------
    def tick(self):
        self.t += self.imu_dt
        t = self.t
        if self.drive:
            self.step_physics(self.imu_dt)
            self.hist.append((t, float(self.state[0]), float(self.state[1]),
                              float(self.state[2])))
        x, y, yaw = self.pose_at(t)
        px, py, pyaw = self.prev
        self.prev = (x, y, yaw)

        stamp = rclpy.time.Time(seconds=t).to_msg()
        c = Clock()
        c.clock = stamp
        self.pub_clock.publish(c)

        wz = (yaw - pyaw) / self.imu_dt
        vx, vy = (x - px) / self.imu_dt, (y - py) / self.imu_dt

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'sim_imu'
        imu.orientation.z = math.sin(yaw / 2)
        imu.orientation.w = math.cos(yaw / 2)
        imu.angular_velocity.z = wz
        # 車體座標的線加速度 (不含重力, 跟 IsaacReadIMU 的 readGravity=False 一樣)
        c_, s_ = math.cos(yaw), math.sin(yaw)
        imu.linear_acceleration.x = c_ * vx + s_ * vy
        imu.linear_acceleration.y = -s_ * vx + c_ * vy
        self.pub_imu.publish(imu)
        self.publish_joint_states(stamp)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z = imu.orientation.z
        od.pose.pose.orientation.w = imu.orientation.w
        od.twist.twist.angular.z = wz
        self.pub_odom.publish(od)

        if t >= self.next_scan:
            self.publish_scan(t)
            self.next_scan += self.scan_dt

    def publish_scan(self, t_end: float):
        """一整圈在 [t_end - 0.05, t_end] 之間掃完, 每個點用它自己那一刻的位姿。"""
        t_pt = t_end - self.scan_dt * (1.0 - self.frac)
        rng_out = np.empty(self.n_pts)
        # 分段算: 每一小段共用一個位姿, 夠細就跟逐點算沒有差別, 但快很多
        nseg = 32
        edges = np.linspace(0, self.n_pts, nseg + 1).astype(int)
        for k in range(nseg):
            a, b = edges[k], edges[k + 1]
            if a >= b:
                continue
            tc = float(t_pt[(a + b) // 2])
            x, y, yaw = self.pose_at(tc)
            c_, s_ = math.cos(yaw), math.sin(yaw)
            R = np.array([[c_, -s_, 0.0], [s_, c_, 0.0], [0.0, 0.0, 1.0]])
            d_world = self.dir_local[a:b] @ R.T
            rng_out[a:b] = cast3d((x, y, self.lidar_z), d_world, self.rng, self.noise)

        bad = ~np.isfinite(rng_out)
        rng_out[bad] = 0.0
        xyz = (self.dir_local * rng_out[:, None]).astype(np.float32)
        xyz[bad] = 0.0                       # skipDroppingInvalidPoints=1 的行為

        m = PointCloud2()
        m.header.stamp = rclpy.time.Time(seconds=t_end).to_msg()
        m.header.frame_id = 'sim_lidar'
        m.height = 1
        m.width = self.n_pts
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(('x', 'y', 'z'))]
        m.is_bigendian = False
        m.point_step = 12
        m.row_step = 12 * m.width
        m.is_dense = False
        m.data = xyz.tobytes()
        self.pub_cloud.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FakeIsaac()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
