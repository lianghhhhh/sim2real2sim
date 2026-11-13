import math

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEELS = ['front_left_joint', 'front_right_joint', 'rear_left_joint', 'rear_right_joint']


def compute_angle(qx, qy, qz, qw):
    """四元數 -> yaw。kros_car 車頭朝 +x, 跟訓練資料一樣不取負號。"""
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class CarStateNode(Node):
    """/odom + /joint_states -> 模型的 10 維狀態
    [x, y, theta, vx, vy, wz, wFL, wFR, wRL, wRR] (見 utils.py 檔頭)。"""

    def __init__(self):
        super().__init__('car_state_node')
        self.latest_joint_state = None
        self.latest_odom = None
        self._theta = None            # unwrap 過的 yaw (連續, 不在 ±pi 跳號)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def get_car_state(self):
        js, od = self.latest_joint_state, self.latest_odom
        if js is None or od is None:
            return None
        try:
            idx = [js.name.index(n) for n in WHEELS]
        except ValueError:
            return None
        if len(js.velocity) <= max(idx):
            return None

        pose, twist = od.pose.pose, od.twist.twist
        q = pose.orientation
        yaw = compute_angle(q.x, q.y, q.z, q.w)
        # MPC 裡 theta 的差是直接相減 (CasADi 不 wrap), 所以給它連續的角度
        self._theta = yaw if self._theta is None else self._theta + wrap(yaw - self._theta)

        return np.array([
            pose.position.x, pose.position.y, self._theta,
            twist.linear.x, twist.linear.y, twist.angular.z,
            *[js.velocity[i] for i in idx],
        ], dtype=float)
