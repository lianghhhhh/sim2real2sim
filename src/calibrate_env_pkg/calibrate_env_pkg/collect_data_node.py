import os
import csv
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32MultiArray

class CollectDataNode(Node):
    def __init__(self):
        super().__init__('collect_data_node')
        self.get_logger().info("CollectDataNode has been started.")

        self.command_subscriber = self.create_subscription(
            JointState,
            'joint_command',
            self.joint_command_callback,
            10
        )

        self.joint_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # 訂閱 control_car_node 廣播的目前測試情境名稱，
        # 讓每一列資料都能標記對應的 throttle/steer 組合與重複次數，
        # 之後做摩擦力分析時才能依情境分組比較
        self.scenario_subscriber = self.create_subscription(
            String,
            'test_scenario',
            self.scenario_callback,
            10
        )
        self.latest_scenario_name = "Unknown"

        self.yolo_coord_subscriber = self.create_subscription(
            Float32MultiArray,
            '/yolo/detections_coord',
            self.yolo_coord_callback,
            10
        )

        # 記下 bbox 中心的原始像素, 之後重新校正相機時可以直接拿這個 CSV 去擬合,
        # 不必從 yolo_x/yolo_y 反推當時用的公式 (公式一改就對不回去了)。
        self.yolo_px_subscriber = self.create_subscription(
            Float32MultiArray,
            '/yolo/detection_px',
            self.yolo_px_callback,
            10
        )
        self.latest_yolo_px = (float('nan'), float('nan'), float('nan'))

        self.filtered_odom_subscriber = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_filtered_callback,
            10
        )

        # 可透過 ROS2 參數自訂輸出資料夾與檔名，例如：
        #   ros2 run calibrate_env_pkg calibrate_env_node --ros-args \
        #       -p output_dir:=/workspaces -p csv_filename:=gravel_run.csv
        # 實測 (車子直走 2.8 m 撞牆): GT=(0.000, -2.825), 估計=(-0.009, -2.814)
        # -> 旋轉角 0.18 度、長度比 0.996, 也就是兩個座標系本來就對齊, 不需要轉換。
        # 原因: 本節點的 odom 座標系是「感測器啟動瞬間的朝向」, 而 multiScan136 在
        # USD 裡是無旋轉 (xformOp:orient = (1,0,0,0)), 車子出生時也沒轉,
        # 所以感測器軸 = 世界軸, 跟 Isaac GT 的世界座標一致。
        # 之後若把感測器改成有旋轉地掛載, 用 scripts/measure_odom_alignment.py 重新量。
        self.declare_parameter('filtered_rotation_deg', 0.0)
        self.declare_parameter('filtered_yaw_offset_deg', 0.0)
        self.declare_parameter('output_dir', '/workspaces/car_run_data')
        self.declare_parameter('csv_filename', 'sim_data.csv')
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value

        self.filepath = os.path.join(output_dir, csv_filename)
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        self.get_logger().info(f"Data will be logged to: {self.filepath}")
        self.count = 0

        with open(self.filepath, 'w') as f:
            writer = csv.writer(f)
            header = [
                'timestamp',
                'scenario_name',
                'effort_command_front_left',
                'effort_command_front_right',
                'effort_command_rear_left',
                'effort_command_rear_right',
                'front_left_angle',
                'front_right_angle',
                'rear_left_angle',
                'rear_right_angle',
                'front_left_velocity',
                'front_right_velocity',
                'rear_left_velocity',
                'rear_right_velocity',
                'car_position_x', 'yolo_x', 'filtered_car_position_x',
                'car_position_y', 'yolo_y', 'filtered_car_position_y',
                'yolo_px', 'yolo_py', 'yolo_conf',
                'car_position_z',
                'car_orientation_x', 'filtered_car_orientation_x',
                'car_orientation_y', 'filtered_car_orientation_y',
                'car_orientation_z', 'filtered_car_orientation_z',
                'car_orientation_w', 'filtered_car_orientation_w',
                'car_linear_velocity_x',
                'car_linear_velocity_y',
                'car_linear_velocity_z',
                'car_angular_velocity_x',
                'car_angular_velocity_y',
                'car_angular_velocity_z'
            ]
            writer.writerow(header)

        self.timer = self.create_timer(0.05, self.log_data)

    def joint_command_callback(self, msg):
        self.latest_joint_command = msg

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def odom_filtered_callback(self, msg):
        # 把 /odometry/filtered 轉到 Isaac ground truth 的座標系。
        # 兩個角度取決於 LiDAR 在 USD 裡的掛載朝向; 目前的掛載是無旋轉,
        # 所以兩者都是 0 (等於不做轉換)。換車/換掛載方式後請重新量:
        #   ros2 run 之後開著車跑一段, 再跑 scripts/measure_odom_alignment.py
        rot = math.radians(
            self.get_parameter('filtered_rotation_deg').get_parameter_value().double_value)
        yaw_off = math.radians(
            self.get_parameter('filtered_yaw_offset_deg').get_parameter_value().double_value)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        rotated_x = x * math.cos(rot) - y * math.sin(rot)
        rotated_y = x * math.sin(rot) + y * math.cos(rot)
        rotated_yaw = yaw + yaw_off

        self.latest_odom_filtered = Odometry()
        self.latest_odom_filtered.header = msg.header
        self.latest_odom_filtered.pose.pose.position.x = rotated_x
        self.latest_odom_filtered.pose.pose.position.y = rotated_y
        self.latest_odom_filtered.pose.pose.position.z = msg.pose.pose.position.z
        self.latest_odom_filtered.pose.pose.orientation.x = 0.0
        self.latest_odom_filtered.pose.pose.orientation.y = 0.0
        self.latest_odom_filtered.pose.pose.orientation.z = math.sin(rotated_yaw /2)
        self.latest_odom_filtered.pose.pose.orientation.w = math.cos(rotated_yaw /2)
        self.latest_odom_filtered.twist = msg.twist

    def scenario_callback(self, msg):
        self.latest_scenario_name = msg.data

    def yolo_px_callback(self, msg):
        self.latest_yolo_px = msg.data

    def yolo_coord_callback(self, msg):
        # 將 YOLO 的座標資訊也記錄到 CSV 中，或是做其他處理
        self.latest_yolo_coords = msg.data

    def log_data(self):
        if hasattr(self, 'latest_joint_command') and \
            hasattr(self, 'latest_joint_state') and \
            hasattr(self, 'latest_odom') and \
            hasattr(self, 'latest_odom_filtered') and \
            hasattr(self, 'latest_yolo_coords'):
            names = self.latest_joint_state.name
            positions = self.latest_joint_state.position
            velocities = self.latest_joint_state.velocity

            # SAFE MAPPING: Find the exact index for each joint
            try:
                fl_idx = names.index('front_left_joint')
                fr_idx = names.index('front_right_joint')
                rl_idx = names.index('rear_left_joint')
                rr_idx = names.index('rear_right_joint')

                cmd_names = self.latest_joint_command.name
                cmd_efforts = self.latest_joint_command.effort
                cmd_fl_idx = cmd_names.index('front_left_joint')
                cmd_fr_idx = cmd_names.index('front_right_joint')
                cmd_rl_idx = cmd_names.index('rear_left_joint')
                cmd_rr_idx = cmd_names.index('rear_right_joint')
            except ValueError:
                # If a joint is missing from the message, skip saving this row to avoid crashes
                return

            with open(self.filepath, 'a') as f:
                writer = csv.writer(f)
                row = [
                    self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9,
                    self.latest_scenario_name,
                    cmd_efforts[cmd_fl_idx], cmd_efforts[cmd_fr_idx], cmd_efforts[cmd_rl_idx], cmd_efforts[cmd_rr_idx],
                    positions[fl_idx], positions[fr_idx], positions[rl_idx], positions[rr_idx],
                    velocities[fl_idx], velocities[fr_idx], velocities[rl_idx], velocities[rr_idx],
                    self.latest_odom.pose.pose.position.x, self.latest_yolo_coords[0], self.latest_odom_filtered.pose.pose.position.x,
                    self.latest_odom.pose.pose.position.y, self.latest_yolo_coords[1], self.latest_odom_filtered.pose.pose.position.y,
                    self.latest_yolo_px[0], self.latest_yolo_px[1], self.latest_yolo_px[2],
                    self.latest_odom.pose.pose.position.z,
                    self.latest_odom.pose.pose.orientation.x, self.latest_odom_filtered.pose.pose.orientation.x,
                    self.latest_odom.pose.pose.orientation.y, self.latest_odom_filtered.pose.pose.orientation.y,
                    self.latest_odom.pose.pose.orientation.z, self.latest_odom_filtered.pose.pose.orientation.z,
                    self.latest_odom.pose.pose.orientation.w, self.latest_odom_filtered.pose.pose.orientation.w,
                    self.latest_odom.twist.twist.linear.x, self.latest_odom.twist.twist.linear.y, self.latest_odom.twist.twist.linear.z,
                    self.latest_odom.twist.twist.angular.x, self.latest_odom.twist.twist.angular.y, self.latest_odom.twist.twist.angular.z
                ]
                writer.writerow(row)
                self.count += 1

            if self.count % 1000 == 0:
                self.get_logger().info(f"Logged {self.count} rows of data.")