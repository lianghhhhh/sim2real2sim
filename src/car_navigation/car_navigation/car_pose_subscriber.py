import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class CarPoseSubscriber(Node):
    def __init__(self):
        super().__init__('car_pose_subscriber')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        topic = self.get_parameter('odom_topic').value
        self.sub = self.create_subscription(Odometry, topic, self.odom_callback, 10)
        self.get_logger().info(f'listening on {topic}')

    def odom_callback(self, msg: Odometry):
        # 1. 取得位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 2. 四元數轉 Yaw (偏航角)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 3. 取得線速度
        vx = msg.twist.twist.linear.x

        self.get_logger().info(
            f"Position: x={x:.3f}, y={y:.3f} | Heading: {math.degrees(yaw):.2f}° "
            f"| Speed: {vx:.2f} m/s",
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = CarPoseSubscriber()
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
