#!/usr/bin/env python3
"""把 ground truth 跟五條定位結果記到同一個 CSV。

一列 = 一個時刻的六個答案:

    /odom              Isaac ground truth      -> car_position_x/y, gt_yaw
    /camera_loc/odom   相機 + YOLO             -> cam_x/y/yaw
    /lidar_loc/odom    LiDAR 對地圖配準        -> lid_x/y/yaw
    /imu_loc/odom      純 IMU 慣性導航         -> imu_x/y/yaw
    /wheel_loc/odom    IMU + 四輪輪速航位推算  -> whl_x/y/yaw
    /fusion_loc/odom   **全部結合** (方法五)   -> fus_x/y/yaw

前四條各自獨立 —— 誰沒開就是誰那幾欄是 NaN, 其他照記。這是刻意的: 想單獨
評估某一條時不必把另外三條也開起來。

**第五條 (`fus_`) 不一樣: 它吃前兩條的輸出。** `car_loc_fusion` 訂閱
`/camera_loc/odom` 與 `/lidar_loc/odom` 當絕對量測, 自己再吃 `/imu` 與
`/joint_states` 做遞推。所以:

* `fus_` 有值但 `cam_`/`lid_` 是 NaN 是**正常的** (融合節點有訂到, 只是那兩個
  節點的 odom 沒被這裡記到 —— 不會發生, 但反過來會: 相機被擋住時 `cam_` 斷了
  `fus_` 還在, 那正是融合的賣點)。
* 拿 `fus_` 跟 `cam_`/`lid_` 比是**公平的** (同一份輸入), 但它們**不獨立** ——
  不要把五條當成五個獨立樣本去做統計。

**每一欄都配一個 `_stamp` 跟 `_age`**, 不要省。原因:

  _stamp  是那個來源自己的時戳。同一列的四個值**不是同一個時刻**的 ——
          相機 ~30 Hz、LiDAR 10 Hz、IMU 200 Hz、本節點 20 Hz。算誤差時要拿
          _stamp 去內插對齊, 直接相減會把時間差當成定位誤差 (車子 0.8 m/s
          時, 50 ms 的錯位就是 4 cm, 跟 LiDAR 的真實誤差同一個量級)。

  _age    是「這個值放多久了」= 記錄當下 - 收到那則訊息的時刻。節點掛掉或
          topic 斷掉時, 最後一個值會被一路複製到檔案結束, 看起來像車子停在
          那裡不動 —— 統計出來的誤差是假的。

  _sigma  是定位節點**自己回報的**位置 1-sigma。純雷射在長方形房間裡會鎖到
          180 度的對稱解, 鎖住之後位置看起來還合理但 yaw 差 180 度, 而且回不
          來。sigma 對這件事很靈敏 (正常 ~0.0015, 追丟 ~0.0145), 拿它當過濾器
          比拿角速度準 —— 車子停下來之後還是鎖著的, 角速度濾不掉。

分析時**兩個條件都要**:

    df[(df.lid_age < 0.3) & (df.lid_sigma < 0.0025)]

`fus_sigma` 的讀法又不一樣: 融合有絕對參考, 所以它**不是**單調長大的 (那是
imu_/whl_), 也**不是**「這一幀追丟了」的指標 (那是 lid_)。它是「所有來源合起來
還剩多少不確定」—— 絕對量測掉線時它會長大, 回來之後縮回去。**用它切出「那幾秒
只有遞推在撐」的片段**, 那正是融合值不值得的地方。

0.0025 這個門檻是量出來的 (三輪資料都是 0 漏網; 用 0.005 有一輪漏掉 14 幀,
yaw RMS 就從 1.69 被拉到 9.15 度)。**看中位數不要只看 RMS** —— 追丟幀的 yaw
誤差接近 180 度, 漏個十幾幀 RMS 就翻倍, 中位數幾乎不動。

不用自己寫: `scripts/eval_loc_csv.py` 就是做這件事的, 還會依角速度分組
(自旋超過 8 rad/s 的追丟是物理極限, 不是 bug) 並掃 sigma 門檻。

    ./scripts/eval_loc_csv.py --sweep

座標系
------
五個來源都必須在**同一個世界座標系**才能逐列比。camera / lidar 的 map 原點
就是 USD 的世界原點, 天生對齊。**兩條航位推算 (imu_ 與 whl_) 要自己對齊** ——
它們是從 `initial_pose` 開始積分的, 沒給就是從 (0,0) 起跑, 車子若不是生在原點,
整段會差一個常數平移, 看起來像超大飄移但其實是沒對起點:

    ros2 run car_loc_imu imu_localizer --ros-args \
        -p initial_pose:="[2.0,-0.3,0.0]"
    ros2 run car_loc_wheel wheel_localizer --ros-args \
        -p initial_pose:="[2.0,-0.3,0.0]"

跑完先看 status log 印的即時誤差, 常數偏移一眼就看得出來。

**而且開錄之前要讓車子停著一兩秒。** 那兩條都要靠開機靜止校正把陀螺零偏量掉,
車子一開始就在動的話零偏會一路帶著跑 —— 那不是方法的問題, 是資料的問題,
事後看 CSV 分不出來。

`_sigma` 對這兩條的意義跟 camera / lidar 不一樣: 航位推算的 sigma 是**單調長大**
的共變異數, 不是「這一幀追丟了」的指標 (它們根本沒有絕對參考可以追丟)。所以
`eval_loc_csv.py` 的 sigma 門檻掃描對 imu_/whl_ 只是在切「跑了多久」, 不要照
lid_ 那套讀。看它們要看**誤差 / 已走距離**。
"""
import os
import csv
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32MultiArray

NAN = float('nan')


def quat_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Source:
    """一條定位線的最新狀態。

    沒收到過任何訊息時所有欄位都是 NaN —— 不是 0。0 是一個合法的座標,
    寫 0 會讓「沒開這條線」跟「車子剛好在原點」分不出來。
    """

    __slots__ = ('name', 'x', 'y', 'yaw', 'stamp', 'sigma', 'recv', 'count')

    def __init__(self, name):
        self.name = name
        self.x = self.y = self.yaw = NAN
        self.stamp = self.sigma = NAN
        self.recv = NAN
        self.count = 0

    def update(self, msg, now: float):
        p = msg.pose.pose
        self.x = p.position.x
        self.y = p.position.y
        self.yaw = quat_yaw(p.orientation)
        self.stamp = (msg.header.stamp.sec
                      + msg.header.stamp.nanosec * 1e-9)
        # 位置的 1-sigma 半徑。定位節點自己回報的可信度, 分析時可以用它
        # 加權, 或是抓出「節點自己就知道這一幀不準」的片段。
        cov = msg.pose.covariance
        s = cov[0] + cov[7]
        self.sigma = math.sqrt(s) if s > 0.0 else NAN
        self.recv = now
        self.count += 1

    def row(self, now: float):
        age = NAN if math.isnan(self.recv) else now - self.recv
        return [self.x, self.y, self.yaw, self.stamp, age, self.sigma]


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

        # --- topic 名稱全部可以改, 換 namespace / 同時比兩份設定時用得到 ---
        p = self.declare_parameter
        p('gt_odom_topic', '/odom')
        p('camera_odom_topic', '/camera_loc/odom')
        p('lidar_odom_topic', '/lidar_loc/odom')
        p('imu_odom_topic', '/imu_loc/odom')
        p('wheel_odom_topic', '/wheel_loc/odom')
        p('fusion_odom_topic', '/fusion_loc/odom')
        p('legacy_camera_pose_topic', '/camera/pose')
        p('legacy_loc_odom_topic', '/localization/odom')
        p('status_period', 10.0)

        def topic(n):
            return self.get_parameter(n).get_parameter_value().string_value

        self.odom_subscriber = self.create_subscription(
            Odometry,
            topic('gt_odom_topic'),
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

        # control_car_node 廣播的階段標籤:
        #   measure / reposition / brake / aborted / idle
        # 分析時**只取 measure** —— reposition 是閉迴路開回起點 (控制器會補償掉
        # 摩擦力差異, 正是我們要量的東西), brake 是段落之間的主動煞停,
        # aborted 是撞牆保護中止的殘缺資料。三者都會汙染回歸。
        self.phase_subscriber = self.create_subscription(
            String,
            'test_phase',
            self.phase_callback,
            10
        )
        self.latest_phase = "unknown"

        # ------------------------------------------------------------------
        # 五條定位線。誰沒開就是 NaN, 不影響其他欄位。
        # imu_ 與 whl_ 是**同一類**的東西 (都是航位推算, 都從 initial_pose 起算),
        # 差在 whl_ 多吃了 /joint_states 的四輪輪速 —— 這一組是拿來直接回答
        # 「加了輪速值多少」的, 所以兩條要**同時開**才有意義。
        # fus_ 是方法五 (car_loc_fusion): 它吃 cam_ 與 lid_ 的輸出當絕對量測,
        # 再加上 /imu + /joint_states 的遞推。要看它的價值就跟另外四條一起錄,
        # 尤其要看**相機或 LiDAR 斷掉的那幾秒** —— 平均誤差看不出融合的好處,
        # 中斷才看得出來。
        # ------------------------------------------------------------------
        self.src_cam = Source('camera')
        self.src_lid = Source('lidar')
        self.src_imu = Source('imu')
        self.src_whl = Source('wheel')
        self.src_fus = Source('fusion')
        for name, src in (('camera_odom_topic', self.src_cam),
                          ('lidar_odom_topic', self.src_lid),
                          ('imu_odom_topic', self.src_imu),
                          ('wheel_odom_topic', self.src_whl),
                          ('fusion_odom_topic', self.src_fus)):
            self.create_subscription(
                Odometry, topic(name),
                lambda msg, s=src: s.update(msg, self._now()), 10)

        # ------------------------------------------------------------------
        # 以下兩條是**舊做法**的欄位 (car_inference 的 /camera/pose 與
        # car_localization 的 /localization/odom)。那兩個 package 已經刪掉, 所以
        # 現在這幾欄**永遠是 NaN** —— 留著只是為了 CSV 欄位格式不變, 讓舊資料跟
        # 舊的分析腳本還對得起來。新的評估請用上面 cam_/lid_/imu_/whl_/fus_ 那五組。
        # ------------------------------------------------------------------
        self.camera_pose_subscriber = self.create_subscription(
            PoseStamped,
            topic('legacy_camera_pose_topic'),
            self.camera_pose_callback,
            10
        )
        self.latest_yolo_coords = (NAN, NAN)
        self.latest_yolo_stamp = NAN

        # 記下 bbox 中心的原始像素, 之後重新校正相機時可以直接拿這個 CSV 去擬合,
        # 不必從 yolo_x/yolo_y 反推當時用的公式 (公式一改就對不回去了)。
        # car_loc_camera 也發同名內容到 /camera_loc/detection_px, 要記那一份就
        # 用 --ros-args -r /yolo/detection_px:=/camera_loc/detection_px。
        self.yolo_px_subscriber = self.create_subscription(
            Float32MultiArray,
            '/yolo/detection_px',
            self.yolo_px_callback,
            10
        )
        self.latest_yolo_px = (NAN, NAN, NAN)

        self.loc_odom_subscriber = self.create_subscription(
            Odometry,
            topic('legacy_loc_odom_topic'),
            self.loc_odom_callback,
            10
        )
        self.latest_loc_odom = self._nan_odom()

        # 可透過 ROS2 參數自訂輸出資料夾與檔名，例如：
        #   ros2 run calibrate_env_pkg calibrate_env_node --ros-args \
        #       -p output_dir:=/workspaces -p csv_filename:=gravel_run.csv
        p('output_dir', '/workspaces/car_run_data')
        p('csv_filename', 'sim_data.csv')
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
                'phase',
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
                'car_position_x', 'yolo_x', 'loc_car_position_x',
                'car_position_y', 'yolo_y', 'loc_car_position_y',
                'yolo_px', 'yolo_py', 'yolo_conf',
                # 各來源自己的時戳 (秒)。分析時用它逐筆內插對齊, 而不是假設
                # 同一列的三個來源是同一個時刻。
                'odom_stamp', 'yolo_stamp', 'loc_stamp',
                'car_position_z',
                'car_orientation_x', 'loc_car_orientation_x',
                'car_orientation_y', 'loc_car_orientation_y',
                'car_orientation_z', 'loc_car_orientation_z',
                'car_orientation_w', 'loc_car_orientation_w',
                'car_linear_velocity_x',
                'car_linear_velocity_y',
                'car_linear_velocity_z',
                'car_angular_velocity_x',
                'car_angular_velocity_y',
                'car_angular_velocity_z',
                # --- 五種定位 + GT 的 yaw。上面那些欄位一個都沒動, 而且新的
                #     whl_* / fus_* 都是**接在最後面**的, 舊腳本照樣讀得到
                #     (pandas 是照欄名取的, 多幾欄不影響)。
                'gt_yaw',
            ]
            for pre in ('cam', 'lid', 'imu', 'whl', 'fus'):
                header += [f'{pre}_x', f'{pre}_y', f'{pre}_yaw',
                           f'{pre}_stamp', f'{pre}_age', f'{pre}_sigma']
            writer.writerow(header)

        self.timer = self.create_timer(0.05, self.log_data)
        self.create_timer(
            max(float(self.get_parameter('status_period').value), 1.0),
            self.status)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def joint_command_callback(self, msg):
        self.latest_joint_command = msg

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    @staticmethod
    def _nan_odom():
        # 定位節點沒開的時候用它佔位, 讓 log_data 那邊不用為了少一個來源多寫分支。
        od = Odometry()
        od.pose.pose.position.x = NAN
        od.pose.pose.position.y = NAN
        od.pose.pose.orientation.x = NAN
        od.pose.pose.orientation.y = NAN
        od.pose.pose.orientation.z = NAN
        od.pose.pose.orientation.w = NAN
        return od

    def loc_odom_callback(self, msg):
        # 不做座標轉換: 舊的 car_localization (已刪) 用的地圖是從 car.usd 的幾何
        # 直接切出來的, 原點就是世界原點, 跟 Isaac 的 /odom 同一個座標系。改用
        # slam_toolbox 建的地圖時兩者會差一個常數平移, 那要在地圖 .yaml 的 origin
        # 修, 不是在這裡修。
        self.latest_loc_odom = msg

    def scenario_callback(self, msg):
        self.latest_scenario_name = msg.data

    def phase_callback(self, msg):
        self.latest_phase = msg.data

    def yolo_px_callback(self, msg):
        # 補滿 3 個: 發布端只給 (px, py) 沒給 conf 的話, log_data 那邊的 [2]
        # 會丟 IndexError —— timer callback 裡爆掉就整個記錄靜悄悄停掉。
        d = list(msg.data)[:3]
        self.latest_yolo_px = tuple(d + [NAN] * (3 - len(d)))

    @staticmethod
    def _stamp_sec(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def camera_pose_callback(self, msg):
        self.latest_yolo_coords = (msg.pose.position.x, msg.pose.position.y)
        self.latest_yolo_stamp = self._stamp_sec(msg.header)

    # ------------------------------------------------------------------
    def status(self):
        """每 10 秒回報四條線的狀況 —— 跑完才發現某一條整段是 NaN 太浪費了。

        誤差是「同一列直接相減」, 沒有做時間對齊, 所以只能當**健康檢查**看
        (常數偏移 = 起點沒對齊, 一路長大 = 飄移)。正式數字要用 _stamp 內插算。
        """
        if not hasattr(self, 'latest_odom'):
            self.get_logger().warn('還沒收到 ground truth /odom')
            return
        gx = self.latest_odom.pose.pose.position.x
        gy = self.latest_odom.pose.pose.position.y
        now = self._now()
        parts = []
        for src in (self.src_cam, self.src_lid, self.src_imu, self.src_whl,
                    self.src_fus):
            if src.count == 0:
                parts.append(f'{src.name}=off')
                continue
            age = now - src.recv
            if age > 1.0:
                parts.append(f'{src.name}=stale({age:.0f}s)')
                continue
            e = math.hypot(src.x - gx, src.y - gy)
            parts.append(f'{src.name}={e * 100:.1f}cm')
        self.get_logger().info(
            f'{self.count} 列 | GT({gx:+.2f},{gy:+.2f}) | ' + ' '.join(parts))

    def log_data(self):
        # ground truth 是唯一的硬需求 —— 沒有它這一列沒得比。joint_command /
        # joint_state 缺了就寫 NaN, **不整列丟掉**: 用 car_teleop 手開車蒐定位
        # 資料時沒有 control_car_node, 舊的寫法會一列都記不到。摩擦力分析本來
        # 就只取 phase=='measure', 那段一定有 joint_command, 不受影響。
        if not hasattr(self, 'latest_odom'):
            return

        eff = [NAN] * 4
        pos = [NAN] * 4
        vel = [NAN] * 4
        names = ('front_left_joint', 'front_right_joint',
                 'rear_left_joint', 'rear_right_joint')
        js = getattr(self, 'latest_joint_state', None)
        if js is not None:
            try:
                idx = [js.name.index(n) for n in names]
                pos = [js.position[i] for i in idx]
                vel = [js.velocity[i] for i in idx]
            except (ValueError, IndexError):
                pass
        jc = getattr(self, 'latest_joint_command', None)
        if jc is not None:
            try:
                idx = [jc.name.index(n) for n in names]
                eff = [jc.effort[i] for i in idx]
            except (ValueError, IndexError):
                pass

        now = self._now()
        gt = self.latest_odom
        loc = self.latest_loc_odom
        with open(self.filepath, 'a') as f:
            writer = csv.writer(f)
            row = [
                now,
                self.latest_scenario_name,
                self.latest_phase,
                eff[0], eff[1], eff[2], eff[3],
                pos[0], pos[1], pos[2], pos[3],
                vel[0], vel[1], vel[2], vel[3],
                gt.pose.pose.position.x, self.latest_yolo_coords[0],
                loc.pose.pose.position.x,
                gt.pose.pose.position.y, self.latest_yolo_coords[1],
                loc.pose.pose.position.y,
                self.latest_yolo_px[0], self.latest_yolo_px[1],
                self.latest_yolo_px[2],
                self._stamp_sec(gt.header),
                self.latest_yolo_stamp,
                self._stamp_sec(loc.header),
                gt.pose.pose.position.z,
                gt.pose.pose.orientation.x, loc.pose.pose.orientation.x,
                gt.pose.pose.orientation.y, loc.pose.pose.orientation.y,
                gt.pose.pose.orientation.z, loc.pose.pose.orientation.z,
                gt.pose.pose.orientation.w, loc.pose.pose.orientation.w,
                gt.twist.twist.linear.x, gt.twist.twist.linear.y,
                gt.twist.twist.linear.z,
                gt.twist.twist.angular.x, gt.twist.twist.angular.y,
                gt.twist.twist.angular.z,
                quat_yaw(gt.pose.pose.orientation),
            ]
            row += self.src_cam.row(now)
            row += self.src_lid.row(now)
            row += self.src_imu.row(now)
            row += self.src_whl.row(now)
            row += self.src_fus.row(now)
            writer.writerow(row)
            self.count += 1


def main(args=None):
    """只跑蒐集節點, 不跑 control_car_node 的自動測試腳本。

    `calibrate_env_node` 那個進入點會同時啟動 ControlCarNode, 車子自己就開始
    跑摩擦力測試腳本, 而且腳本跑完整個 process 就結束 —— 想「自己用 teleop
    開車, 一路記五種定位 + GT」的話要用這一個, 它會一直跑到 Ctrl-C:

        ros2 run calibrate_env_pkg collect_data_node --ros-args \
            -p output_dir:=/workspaces/car_run_data -p csv_filename:=all_loc.csv
    """
    import rclpy
    rclpy.init(args=args)
    node = CollectDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'共記錄 {node.count} 筆資料 -> {node.filepath}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
