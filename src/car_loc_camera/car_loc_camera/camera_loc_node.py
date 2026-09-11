#!/usr/bin/env python3
"""方法一: 只用天花板相機 + YOLO 定位。

    /rgb ──> YOLO ──> bbox 中心 (px) ──> 地面投影 ──> (x, y) 世界座標
                                                        │
                              等速卡爾曼濾波 (含 NIS 閘門) ┘
                                                        v
                          /camera_loc/odom + /camera_loc/pose + TF map->base_link

**這個節點不訂閱 /imu, 不訂閱任何雷射 topic。** 這是刻意的 —— 三條定位路線
要能互相當對照, 就不能偷用彼此的資料。

四個關鍵決定
------------
1. **投影用單應性, 不用「像素偏移乘一個比例」。** 天花板相機看的是一個平面,
   單應性就是針孔相機看平面的精確模型, 相機傾斜、主點偏移、視差全部一起吸收。
2. **挑框用「離預測位置最近」, 不是「信心最高」, 更不是 boxes[0]。**
   畫面裡只要多一個誤判 (影子、反光), 用排序或信心挑都會整段跳掉;
   而濾波器本來就知道車子大概在哪。
3. **時戳用影像的, 而且扣掉校正檔量到的延遲。** 曝光 + 傳輸 + YOLO 推論加起來
   是幾十到上百毫秒, 車速 0.8 m/s 時那就是好幾公分的固定偏差, 而且偏差方向
   跟著車頭轉 —— 投影模型吸收不掉。
4. **輸出是濾波後的, 不是逐幀量測。** 逐幀的 7 cm 白雜訊會被平均掉 (實測到
   3~4 cm), 而且 YOLO 漏幀時輸出不會斷, 只是共變異數長大。
"""
from __future__ import annotations

import os

for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_v, '1')

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

from .detector import CarDetector
from .projection import GroundProjection
from .tracker import ConstVelTracker, wrap_pi

# depth=1: YOLO 推論比影像來得慢, 佇列留深了只會拿到過期的幀。
# 寧可丟幀也要處理最新的那一張。
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


def to_stamp(t: float):
    from builtin_interfaces.msg import Time
    s = Time()
    s.sec = int(math.floor(t))
    s.nanosec = int(round((t - s.sec) * 1e9))
    if s.nanosec >= 1000000000:
        s.sec += 1
        s.nanosec -= 1000000000
    return s


class CameraLocalizer(Node):

    def __init__(self):
        super().__init__('camera_localizer')
        p = self.declare_parameter

        # --- topics / frames -------------------------------------------------
        p('image_topic', '/rgb')
        p('odom_topic', '/camera_loc/odom')
        p('pose_topic', '/camera_loc/pose')
        p('detection_topic', '/camera_loc/detections')
        p('pixel_topic', '/camera_loc/detection_px')
        p('latency_topic', '/camera_loc/latency')   # 空 = 不發
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('publish_tf', True)
        p('publish_annotated', True)

        # --- YOLO -------------------------------------------------------------
        p('model_path', '')       # 空 = 自己找 (見 detector.find_model)
        p('imgsz', 512)           # 必須跟 best.onnx 匯出時的尺寸一致
        p('conf', 0.25)
        p('class_id', -1)         # -1 = 不限類別

        # --- 校正 --------------------------------------------------------------
        p('calibration_path', '')  # 空 = 用 package 內附的 config/camera_ground.yaml
        # 校正檔裡的 delay 是「量出來的」; 這裡可以再覆蓋 (負值 = 不覆蓋)
        p('delay_override', -1.0)

        # --- 挑框 --------------------------------------------------------------
        # 離預測位置這麼遠以內的框才考慮 (m)。設 0 = 關掉, 退回「挑信心最高的」。
        p('gate_radius', 1.0)
        # 但車子真的被搬走時預測位置是錯的, 所以連續這麼多幀都沒有框落在閘門內,
        # 就放棄閘門直接用信心最高的那個 (然後濾波器的逃生門會接手重設)。
        p('gate_giveup', 10)

        # --- 濾波器 -----------------------------------------------------------
        # 未建模的加速度 (m/s^2)。設成車子的峰值加速度; 設太小比設太大危險得多
        p('accel_sigma', 2.0)
        p('meas_sigma', -1.0)     # <0 = 用校正檔自己報的殘差
        p('v_max', 4.0)
        p('chi2', 9.21)           # 2 自由度卡方 99%
        p('force_accept_after', 8)
        p('yaw_min_speed', 0.15)
        p('yaw_init_speed', 0.30)
        # 車頭朝向 vs 移動方向: 見 tracker._update_yaw 的說明。
        p('allow_reverse', False)
        # 濾波器只能從速度方向推出**行進方向**; base_frame 的 x 軸不一定就是
        # 行進方向。這個角度是「base_frame 的 x 軸領先行進方向多少度」,
        # 發布時會加上去, 位姿的四元數與車體座標的 twist 都會跟著轉。
        #
        # *** 這一項必須量, 不能猜, 而且猜 0 是錯的。***
        # 量法: 開一段直線, 比對 ground truth 的 yaw 與 GT 位置微分出來的
        # 行進方向 —— atan2(dy, dx) - yaw 的中位數就是它 (取負號)。
        # car.usd 這台車量出來是 **+90 度** (行進方向 - gt_yaw = -90.00°,
        # IQR [-90, -90], n=508), 也就是 base_link 的前方是 -y 不是 x。
        # 不填的話發出去的 map->base_link 會整個轉 90 度, 而且 twist 會把
        # 前進速度掛到錯的軸上 —— 位置再準也沒用。
        p('yaw_offset_deg', 0.0)
        # 這麼久沒有被採信的量測就當作追丟, 只推不修 (共變異數會一直長大)
        p('lost_after', 1.0)
        # 追丟超過這麼久就整個重設到下一個量測上
        p('reset_after', 3.0)

        # --- 輸出 --------------------------------------------------------------
        p('status_period', 2.0)
        # 額外發一個「外推到現在」的位姿, 給拿最新一則當現在位置的下游用
        # (TF / nav)。0 = 關。見 tracker.peek 的說明與那裡的實測數字。
        # /camera_loc/odom 本身**不受影響** —— 它永遠是影像時刻的狀態配影像
        # 時刻的時戳, 那才是依時戳內插的融合節點要的東西。
        p('predict_rate', 0.0)
        p('now_odom_topic', '/camera_loc/odom_now')
        # TF 改成由外推那條路發 (時戳 = 現在)。tf2 不會自己往前推, 所以想讓
        # lookup_transform(..., now) 拿到合理的值就要開這個。
        p('tf_predict', False)

        g = self.get_parameter
        self.map_frame = g('map_frame').value
        self.base_frame = g('base_frame').value
        self.do_tf = bool(g('publish_tf').value)
        self.do_annot = bool(g('publish_annotated').value)
        self.gate_radius = float(g('gate_radius').value)
        self.gate_giveup = int(g('gate_giveup').value)
        self.lost_after = float(g('lost_after').value)
        self.reset_after = float(g('reset_after').value)
        self.yaw_offset = math.radians(float(g('yaw_offset_deg').value))
        self.tf_predict = bool(g('tf_predict').value)

        # --- 校正檔 ------------------------------------------------------------
        calib = g('calibration_path').value
        if not calib:
            from ament_index_python.packages import get_package_share_directory
            calib = os.path.join(get_package_share_directory('car_loc_camera'),
                                 'config', 'camera_ground.yaml')
        if not os.path.exists(calib):
            raise FileNotFoundError(f'找不到校正檔 {calib}')
        self.proj = GroundProjection.from_yaml(calib)
        d = float(g('delay_override').value)
        if d >= 0.0:
            self.proj.delay = d
        self.get_logger().info(
            f'校正檔 {calib} (擬合解析度 {self.proj.ref_w:.0f}x{self.proj.ref_h:.0f}, '
            f'殘差 {self.proj.sigma * 100:.1f} cm, 延遲 {self.proj.delay * 1000:.0f} ms)')

        # --- 濾波器 ------------------------------------------------------------
        ms = float(g('meas_sigma').value)
        self.tracker = ConstVelTracker(
            accel_sigma=float(g('accel_sigma').value),
            meas_sigma=self.proj.sigma if ms < 0 else ms,
            v_max=float(g('v_max').value),
            chi2=float(g('chi2').value),
            force_accept_after=int(g('force_accept_after').value),
            yaw_min_speed=float(g('yaw_min_speed').value),
            yaw_init_speed=float(g('yaw_init_speed').value),
            allow_reverse=bool(g('allow_reverse').value))

        # --- YOLO --------------------------------------------------------------
        self.det = CarDetector(model_path=g('model_path').value,
                               imgsz=int(g('imgsz').value),
                               conf=float(g('conf').value),
                               class_id=int(g('class_id').value))
        self.get_logger().info(f'模型 {self.det.path} (imgsz={self.det.imgsz})')

        from cv_bridge import CvBridge
        self.bridge = CvBridge()

        # --- ROS 介面 ----------------------------------------------------------
        self.pub_odom = self.create_publisher(Odometry, g('odom_topic').value, 10)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, g('pose_topic').value, 10)
        self.pub_px = self.create_publisher(
            Float32MultiArray, g('pixel_topic').value, 10)
        # [傳輸 ms, 總處理 ms, YOLO 推論 ms] —— 錄下來就能看延遲隨負載怎麼跑
        self.pub_lat = (self.create_publisher(
            Float32MultiArray, g('latency_topic').value, 10)
            if g('latency_topic').value else None)
        self.pub_img = (self.create_publisher(Image, g('detection_topic').value, 2)
                        if self.do_annot else None)
        self.tf = tf2_ros.TransformBroadcaster(self) if self.do_tf else None
        rate = float(g('predict_rate').value)
        self.pub_now = None
        if rate > 0.0:
            self.pub_now = self.create_publisher(
                Odometry, g('now_odom_topic').value, 10)
            self.create_timer(1.0 / rate, self.publish_now)
        elif self.tf_predict:
            self.get_logger().warn(
                'tf_predict:=true 但 predict_rate=0, TF 還是會用影像時刻發')
            self.tf_predict = False
        self.create_subscription(Image, g('image_topic').value,
                                 self.on_image, IMAGE_QOS)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_timer(float(g('status_period').value), self.status)

        # --- 統計 --------------------------------------------------------------
        self.width = None
        self.height = None
        self.n_img = 0
        self.n_det = 0
        self.miss_streak = 0
        self.gate_miss = 0
        self.infer_ms = 0.0
        self.last_report = 0.0
        # 處理延遲 (訊息時戳 -> 發出位姿), ROS 時鐘秒。見 _note_latency。
        self._lat = []
        self._lat_in = 0.0
        self._warned_delay = False
        self.get_logger().info(
            f"等 {g('image_topic').value} ... (只用相機, 不吃 IMU / LiDAR)")

    # ------------------------------------------------------------------
    def on_reset(self, req, res):
        self.tracker.initialized = False
        self.tracker.has_yaw = False
        res.success = True
        res.message = '濾波器已重設, 下一個偵測會重新初始化'
        self.get_logger().warn(res.message)
        return res

    # ------------------------------------------------------------------
    def _pick(self, dets, t):
        """從候選框裡挑出「車」。

        濾波器已經初始化的時候, 「離預測位置最近」比「信心最高」可靠得多 ——
        誤判通常在別的地方, 而車子不會瞬間移動。但預測位置本身也可能是錯的
        (車被搬走 / 剛啟動), 所以連續 gate_giveup 幀都沒有框落在閘門內就放棄
        閘門, 讓濾波器的逃生門去處理。
        """
        if not dets:
            return None
        if self.gate_radius <= 0 or not self.tracker.initialized:
            return max(dets, key=lambda d: d.conf)

        # 用「預測到這一刻」的位置當閘門中心, 不是上一幀的位置
        pred = self.tracker.pos + self.tracker.vel * max(
            0.0, t - (self.tracker.t if self.tracker.t is not None else t))
        best, best_d = None, float('inf')
        for d in dets:
            try:
                x, y = self.proj(d.px, d.py, self.width, self.height)
            except ValueError:
                continue
            dist = math.hypot(x - pred[0], y - pred[1])
            if dist < best_d:
                best, best_d = d, dist
        if best is not None and best_d <= self.gate_radius:
            self.gate_miss = 0
            return best
        self.gate_miss += 1
        if self.gate_miss >= self.gate_giveup:
            self.get_logger().warn(
                f'連續 {self.gate_miss} 幀沒有框落在預測位置 {self.gate_radius:.1f} m 內, '
                f'改用信心最高的 (最近的差 {best_d:.2f} m)')
            return max(dets, key=lambda d: d.conf)
        return None

    # ------------------------------------------------------------------
    def on_image(self, msg: Image):
        self.n_img += 1
        # 影像的時刻 = 訊息時戳 - 延遲。
        #
        # *** delay 是「拍照比訊息時戳早多少」, 不是「處理花了多久」。***
        # 相機如果是收到影像才蓋時戳 (很多真實相機驅動是這樣), delay > 0;
        # 如果是拍的當下就蓋 (Isaac 是這樣), delay = 0。
        # 處理花多久跟這個無關 —— 不管算 10 ms 還是 200 ms, 這張照片拍的那
        # 一刻都沒有變。把處理時間填進 delay 會讓發出去的時戳整個提早,
        # 融合節點就會拿錯時刻的資料去對齊。self._lat 就是在量處理時間,
        # 用來抓這個錯 (見 status())。
        t_img = stamp_sec(msg.header.stamp)
        t = t_img - self.proj.delay
        self._lat_in = self._clock_s() - t_img

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'影像轉換失敗: {e}')
            return
        self.height, self.width = bgr.shape[:2]

        t0 = time.perf_counter()
        try:
            dets = self.det.detect(bgr)
        except Exception as e:
            self.get_logger().error(f'YOLO 推論失敗: {e}')
            return
        self.infer_ms = 0.9 * self.infer_ms + 0.1 * (time.perf_counter() - t0) * 1e3

        if self.pub_img is not None:
            try:
                out = self.bridge.cv2_to_imgmsg(self.det.annotated(), encoding='bgr8')
                out.header = msg.header
                self.pub_img.publish(out)
            except Exception:
                pass

        d = self._pick(dets, t)
        if d is None:
            self.miss_streak += 1
            if self.miss_streak in (30, 100) or self.miss_streak % 300 == 0:
                self.get_logger().warn(f'連續 {self.miss_streak} 幀沒有可用的偵測')
            # 沒有量測也要推 —— 下游拿得到「車大概在哪 + 有多不確定」,
            # 而不是整個斷掉
            if self.tracker.initialized:
                self.tracker.predict(t)
                if self.tracker.age(t) > self.reset_after:
                    self.tracker.initialized = False
                    self.tracker.has_yaw = False
                    self.get_logger().warn(
                        f'追丟超過 {self.reset_after:.1f} s, 重設濾波器')
                else:
                    self.publish(t)
            return

        self.miss_streak = 0
        self.n_det += 1

        try:
            # yaw 餵自己上一刻的估計 —— 只有開 use_aabb 時才用得到,
            # 而且用的是本節點自己的輸出, 不是別的感測器的
            yaw = self.tracker.yaw if self.tracker.has_yaw else None
            x, y = self.proj(d.px, d.py, self.width, self.height, yaw=yaw)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        self.pub_px.publish(Float32MultiArray(
            data=[float(d.px), float(d.py), float(d.conf)]))
        # 信心低的框就宣告比較大的量測雜訊, 讓濾波器自己少信一點
        sigma = self.proj.sigma * (1.0 + max(0.0, 0.6 - d.conf))
        self.tracker.update(t, (x, y), sigma=sigma)
        self.publish(t)
        self._note_latency(t_img)

    # ------------------------------------------------------------------
    def _clock_s(self) -> float:
        """現在的 ROS 時刻 (秒)。use_sim_time 時是 sim time —— 跟影像時戳同一個
        時鐘, 所以相減不需要知道 RTF, 也不會被 RTF 變動影響。"""
        return self.get_clock().now().nanoseconds * 1e-9

    def _note_latency(self, t_img: float):
        """記一次「訊息時戳 -> 發出位姿」花了多久 (處理延遲)。

        這**不是** delay。這是節點自己的處理時間, 量它有兩個用途:
          1. 看節點有沒有跟上 (跟影像週期比)。
          2. 抓 delay 被誤填成處理時間的錯 —— 那是最容易犯又最難發現的,
             因為它只影響時戳, 不影響位置, 所以「位置誤差」那類指標
             完全看不出來。status() 會直接警告。
        """
        lat = self._clock_s() - t_img
        self._lat.append(lat)
        if len(self._lat) > 400:
            del self._lat[:200]
        if self.pub_lat is not None:
            self.pub_lat.publish(Float32MultiArray(data=[
                float(self._lat_in * 1e3), float(lat * 1e3),
                float(self.infer_ms)]))

    def _lat_median(self) -> float:
        if not self._lat:
            return 0.0
        return float(sorted(self._lat)[len(self._lat) // 2])

    def _odom(self, stamp, x, pose_cov, twist_cov):
        """把狀態組成 Odometry。x 是 [px, py, vx, vy] (世界座標)。

        yaw_offset 在這裡加 —— 濾波器算的是**行進方向**, base_frame 的 x 軸
        不一定就是行進方向 (car.usd 差 90 度)。位姿與 twist 要用同一個 yaw,
        不然「前進速度掛在哪一軸」跟位姿會互相矛盾。
        """
        yaw = wrap_pi(self.tracker.yaw + self.yaw_offset)
        qz, qw = math.sin(yaw * 0.5), math.cos(yaw * 0.5)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.map_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(x[0])
        od.pose.pose.position.y = float(x[1])
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.pose.covariance = pose_cov.ravel().tolist()
        # twist 照 ROS 慣例是**車體座標**的
        c, s = math.cos(yaw), math.sin(yaw)
        od.twist.twist.linear.x = float(c * x[2] + s * x[3])
        od.twist.twist.linear.y = float(-s * x[2] + c * x[3])
        od.twist.covariance = twist_cov.ravel().tolist()
        return od, qz, qw

    def _send_tf(self, stamp, x, qz, qw):
        tfm = TransformStamped()
        tfm.header.stamp = stamp
        tfm.header.frame_id = self.map_frame
        tfm.child_frame_id = self.base_frame
        tfm.transform.translation.x = float(x[0])
        tfm.transform.translation.y = float(x[1])
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw
        self.tf.sendTransform(tfm)

    def publish(self, t: float):
        tr = self.tracker
        if not tr.initialized:
            return
        stamp = to_stamp(t)
        od, qz, qw = self._odom(stamp, tr.x, tr.pose_cov(), tr.twist_cov())
        self.pub_odom.publish(od)

        pc = PoseWithCovarianceStamped()
        pc.header = od.header
        pc.pose.pose = od.pose.pose
        pc.pose.covariance = od.pose.covariance
        self.pub_pose.publish(pc)

        # tf_predict 開著的時候 TF 由 publish_now 那條路發 (時戳 = 現在)
        if self.tf is not None and not self.tf_predict:
            self._send_tf(stamp, tr.x, qz, qw)

    def publish_now(self):
        """把狀態外推到**現在**再發一次 —— 給拿最新一則當現在位置的下游。

        /camera_loc/odom 是影像時刻的狀態配影像時刻的時戳 (依時戳內插的融合
        節點要的是那個); 但 TF / nav 問的是「現在在哪」, 而最新一則已經是
        delay + 半個影像週期之前的事了。實測數字見 tracker.peek。
        """
        tr = self.tracker
        if not tr.initialized:
            return
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        # 追丟太久就別再推了 —— 推出來的位置只會愈來愈假, 而且 nav 會當真。
        if tr.t is not None and (t - tr.t) > self.lost_after:
            return
        x, P = tr.peek(t)
        if x is None:
            return
        pose_cov = tr.pose_cov()
        pose_cov[0, 0], pose_cov[1, 1] = P[0, 0], P[1, 1]
        pose_cov[0, 1] = pose_cov[1, 0] = P[0, 1]
        stamp = now.to_msg()
        od, qz, qw = self._odom(stamp, x, pose_cov, tr.twist_cov())
        self.pub_now.publish(od)
        if self.tf is not None and self.tf_predict:
            self._send_tf(stamp, x, qz, qw)

    # ------------------------------------------------------------------
    def status(self):
        if self.n_img == 0:
            self.get_logger().warn('還沒有收到任何影像')
            return
        tr = self.tracker
        rate = 100.0 * self.n_det / max(self.n_img, 1)
        lost = ''
        if tr.initialized and tr.t is not None and tr.age(tr.t) > self.lost_after:
            lost = f'  [追丟 {tr.age(tr.t):.1f} s]'
        sp = math.sqrt(max(tr.P[0, 0] + tr.P[1, 1], 0.0))
        self.get_logger().info(
            f'{self.n_img} 幀, 偵測率 {rate:.0f}%, 推論 {self.infer_ms:.0f} ms | '
            f'x={tr.x[0]:+.3f} y={tr.x[1]:+.3f} '
            # 印發出去的那個 yaw (含 yaw_offset), 不是濾波器內部的行進方向 ——
            # 不然對著 RViz 的車頭 debug 會一直對不上
            f'yaw={math.degrees(wrap_pi(tr.yaw + self.yaw_offset)):+.1f}° '
            f'v={tr.speed:.2f} m/s sigma={sp * 100:.1f} cm | '
            f'處理延遲 {self._lat_median() * 1e3:.0f} ms | '
            f'量測 採信 {tr.n_accepted} / 擋掉 {tr.n_rejected}{lost}')

        # delay 被誤填成處理延遲是最容易犯的錯 —— 它只影響時戳不影響位置,
        # 所以任何「位置誤差」指標都看不出來, 只有融合的下游會受害。
        # 這裡直接把兩個數字擺在一起比。
        lat = self._lat_median()
        if (not self._warned_delay and lat > 0.005
                and self.proj.delay > 0.5 * lat):
            self._warned_delay = True
            self.get_logger().warn(
                f'校正檔的 delay={self.proj.delay * 1e3:.0f} ms, 但量到的**處理**'
                f'延遲是 {lat * 1e3:.0f} ms —— 兩個數字太接近了。\n'
                f'  delay 的定義是「拍照比訊息時戳早多少」(相機端), '
                f'不是「處理花多久」(節點端)。\n'
                f'  不管算 10 ms 還是 200 ms, 照片拍的那一刻都沒有變, '
                f'所以處理時間不該進 delay。\n'
                f'  填錯的話發出去的時戳會提早 {self.proj.delay * 1e3:.0f} ms, '
                f'融合節點拿錯時刻對齊, 而位置誤差看起來完全正常。\n'
                f'  Isaac 的 /rgb 是拍照當下就蓋時戳, 那 delay 應該是 0。\n'
                f'  想看「位置有多舊」請用 /camera_loc/odom_now (外推到現在)。')


def main(args=None):
    rclpy.init(args=args)
    node = CameraLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch 送 SIGINT 時 rclpy 的訊號處理可能已經關掉 context,
        # 再關一次會丟 RCLError 讓行程以 exit code 1 收場, 看起來像節點掛了
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
