#!/usr/bin/env python3
"""
scan_qos_bridge.py

pointcloud_to_laserscan 發布 /scan 時使用的是 BEST_EFFORT QoS，
而 rf2o_laser_odometry 訂閱時要求 RELIABLE，兩者不相容導致
rf2o 完全收不到任何 LaserScan 訊息 (rf2o 端不會報錯，只會一直
印 "Waiting for laser_scans....")。

這個節點以 BEST_EFFORT 訂閱 /scan，再用 RELIABLE 重新發布成
/scan_reliable，讓 rf2o 改訂閱這個新 topic 即可正常收到資料。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanQosBridge(Node):
    def __init__(self):
        super().__init__('scan_qos_bridge')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_reliable')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # 對應 pointcloud_to_laserscan 實際使用的 QoS (sensor data: best effort)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # rf2o_laser_odometry 訂閱端要求的 QoS
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(LaserScan, output_topic, pub_qos)
        self.sub = self.create_subscription(
            LaserScan, input_topic, self.cb, sub_qos
        )

        self.get_logger().info(
            f'Relaying {input_topic} (BEST_EFFORT) -> {output_topic} (RELIABLE)'
        )

    def cb(self, msg: LaserScan):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanQosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()