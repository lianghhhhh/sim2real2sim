#!/usr/bin/env python3
"""8 台環境相機 -> YOLO 車框 -> /env_cam_i/car_bbox。

    /env_cam_0/rgb ─┐                                   ┌─> /env_cam_0/car_bbox
          ...       ├─> (ROI 裁切) ─> YOLO ─> 框 (px) ──┤        ...
    /env_cam_7/rgb ─┘        ^                          └─> /env_cam_7/car_bbox
                             │
          /multicam_loc/odom (融合的預測位置: 車大概在畫面哪裡)

**這個節點只做偵測, 不做定位。** 挑哪一個框是車、怎麼變成位置, 是融合節點的事
(它知道預測位置與不確定性)。實體系統裡這一段可以搬到每台 Pi 上 (見
report_env_camera_plan_2026-10-28.md), 訊息格式見 bbox_msg.py。

YOLO 模型沿用 car_loc_camera 的 best.onnx (import 它的 CarDetector, 沒有複製)。

ROI 裁切 —— 讓車在 YOLO 眼中維持「訓練時的大小」
-------------------------------------------------
best.onnx 是用天花板正下方、車約 22 px (在 512 的輸入裡) 的畫面訓練的。**它只認得
那個大小的車**: 2026-10-30 在 Isaac 的 8 台斜拍畫面上實測, 車在房間中央 (原圖 50~80 px)
時 ——
    整張 1456 縮到 512 (車 18~28 px)      8 台都認得, conf 0.31~0.90
    裁 320 px 再放大到 512 (車 ~100 px)   8 台幾乎都認不得, 只剩碎片框
    裁「預測框的 18 倍」(至少 512 px)     8 台都認得, conf 0.79~0.92, 邊差 1~7 px
反過來, 車**太近**也一樣認不得: 車離相機 2~3 m 時原圖有 120~160 px, 就算整張縮到 512
還有 40~70 px —— 2026-10-30 那一輪 (整個房間繞一圈) 2~3 m 內的框有 83% 跟外形差
超過 10 px, 6 m 以外是 0%。
所以 ROI 的工作是**把車縮放到固定大小**, 不是裁切: 以預測框為中心取邊長
`roi_scale` x 框的視窗 (至少 `roi_min`), 整個縮進一張 imgsz x imgsz 的畫布, 視窗
超出原圖的部分補灰 (114, 跟 YOLO 的 letterbox 一樣)。車在 YOLO 眼中永遠是
imgsz / roi_scale ≈ 28 px。同一張近距離畫面實測: 視窗被原圖大小卡住 (舊做法)
conf 0.19、邊差 16 px; 補灰縮小 conf 0.89、邊差 3~9 px。
預測說車不在這台相機的畫面裡時就不跑 YOLO (省 GPU), 但每 `full_frame_period` 秒
還是跑一次整張, 免得預測錯了就永遠找不回來。

**模型是用正上方視角訓練的, 斜拍的外觀它沒看過。** 偵測率要實際量 (status 會印
每台的偵測率); 太低就要用這 8 台的畫面重新標注、重新訓練 —— 這是程式碼解決不了的。
"""
from __future__ import annotations

import os

for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')

import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from car_loc_camera.detector import CarDetector

from . import bbox_msg
from .camera_model import load_cameras, share_file
from .car_model import CarModel

# 影像訂閱的 QoS。depth=1: 寧可丟幀也要處理最新的那一張。
#
# **要 RELIABLE, 不能 BEST_EFFORT** (參數 image_reliable)。一張 1456x1088 rgb 是 4.75 MB,
# DDS 拆成 ~75 個 UDP 片段; Isaac (host) -> 容器 (docker bridge) 這條路 socket 接收緩衝
# 只有 208 KB (host 的 net.core.rmem_max, 容器裡改不了), 8 台同時灌的時候片段大量溢出。
# BEST_EFFORT 少一片整張作廢、不重送 -> 2026-10-30 在使用者的 Isaac 實測 8 台同時訂閱時
# 每台 0~0.4 張/秒 (該有 15), 那一輪定位 RMS 10 cm。同一條路改 RELIABLE (掉的片段會被
# NACK 補回來): 8 台都是完整的 15 Hz (sim time), 一張不漏。
# Isaac 那邊的 publisher 本來就是 RELIABLE (KEEP_LAST 10), 慢的 reader 只會漏舊的那幾張,
# 不會卡住模擬 (實測 real-time factor 不變)。


def image_qos(reliable: bool) -> QoSProfile:
    return QoSProfile(reliability=(ReliabilityPolicy.RELIABLE if reliable
                                   else ReliabilityPolicy.BEST_EFFORT),
                      history=HistoryPolicy.KEEP_LAST, depth=1)


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


class CamState:
    __slots__ = ('cam', 'pub', 'pub_dbg', 'last_t', 'last_full', 'n_img', 'n_det',
                 'n_roi', 'n_full', 'n_skip', 'clock_buf', 'clock_offset', 'clock_locked')

    def __init__(self, cam):
        self.cam = cam
        self.pub = self.pub_dbg = None
        self.last_t = -1e18
        self.last_full = -1e18
        self.n_img = self.n_det = self.n_roi = self.n_full = self.n_skip = 0
        self.clock_buf = []
        self.clock_offset = 0.0
        self.clock_locked = False


# 影像時戳跟 odom 差超過這個 (s) 就認定是時鐘基準不一樣 (跟 car_loc_fusion 的
# clock_max_offset 同一個規則)。2026-10-30 使用者的 Isaac: 8 台裡 6 台的影像時戳比
# /clock 快 508 -> 1319 s (每次 Stop/Play 都沒歸零, 8 個 graph 的設定一模一樣),
# 只有 2 台是對的。不扣掉的話那 6 台的 ROI 預測永遠「太舊」-> 每張都跑整張。
CLOCK_MAX_OFFSET = 1.0
CLOCK_SAMPLES = 15


class MultiCamDetector(Node):

    def __init__(self):
        super().__init__('multicam_detector')
        p = self.declare_parameter
        p('cameras_file', '')           # 空 = 本 package 的 config/env_cameras.yaml
        p('car_model_file', '')         # 空 = 本 package 的 config/car_model.yaml
        p('model_path', '')             # 空 = car_loc_camera 的 resource/best.onnx
        p('imgsz', 512)                 # 必須跟 best.onnx 匯出時一致
        p('conf', 0.25)
        p('class_id', -1)
        # 每台相機最多處理到這個頻率 (Hz)。8 台全速 = 8 倍的 YOLO, GPU 不一定吃得消;
        # 融合那邊 10~15 Hz 就很夠 (IMU + 輪速在中間補)。0 = 不限。
        p('max_rate', 15.0)
        # ROI 裁切 (見模組說明)
        p('roi', True)
        p('odom_topic', '/multicam_loc/odom')
        p('roi_scale', 18.0)        # 見模組說明: 太小反而認不得
        p('roi_min', 512)           # = YOLO 輸入大小, 不要比它小 (會把車放大)
        p('roi_timeout', 0.5)           # 預測位置比影像舊超過這麼多 (s) 就不信它
        p('full_frame_period', 1.0)     # 沒用 ROI 時至少每這麼久跑一次整張 (s)
        p('forward_deg', 0.0)           # base_link +X 到車頭的角度 (外推預測位置用)
        p('image_reliable', True)       # 影像訂閱用 RELIABLE (見 image_qos 上面的說明)
        p('publish_debug', False)       # /env_cam_i/car_bbox_debug: 預測框/ROI/偵測框
        p('status_period', 5.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cams = load_cameras(g('cameras_file') or share_file('env_cameras.yaml'))
        self.car = CarModel.from_yaml(g('car_model_file') or share_file('car_model.yaml'))
        self.min_dt = 1.0 / float(g('max_rate')) if float(g('max_rate')) > 0 else 0.0
        self.roi = bool(g('roi'))
        self.roi_scale = float(g('roi_scale'))
        self.roi_min = int(g('roi_min'))
        self.roi_timeout = float(g('roi_timeout'))
        self.full_period = float(g('full_frame_period'))
        self.phi = math.radians(float(g('forward_deg')))
        self.debug = bool(g('publish_debug'))

        self.det = CarDetector(model_path=g('model_path'), imgsz=int(g('imgsz')),
                               conf=float(g('conf')), class_id=int(g('class_id')))
        self._warm_up()
        from cv_bridge import CvBridge
        self.bridge = CvBridge()
        self.odom = None                 # (t, x, y, yaw, v, wz)
        self.infer_ms = 0.0

        qos = image_qos(bool(g('image_reliable')))
        self.state = {}
        for cam in self.cams:
            s = CamState(cam)
            s.pub = self.create_publisher(bbox_msg.PolygonStamped, cam.detection_topic, 10)
            if self.debug:
                s.pub_dbg = self.create_publisher(Image, cam.detection_topic + '_debug', 2)
            self.create_subscription(Image, cam.image_topic,
                                     lambda m, s=s: self.on_image(m, s), qos)
            self.state[cam.name] = s
        if self.roi:
            self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        self.create_timer(float(g('status_period')), self.status)
        self.get_logger().info(
            f'{len(self.cams)} 台相機: ' + ', '.join(c.image_topic for c in self.cams)
            + f' | 模型 {self.det.path} | ROI {"開" if self.roi else "關"}')

    def _warm_up(self):
        """第一次推論在訂閱之前做掉, 不要讓它卡在第一張影像的 callback 裡。

        2026-10-30 在 RTX PRO 6000 (Blackwell) 上: **新的容器**第一次推論要 ~31 s
        (CUDA 在編 kernel), 編好的快取在 ~/.nv/ComputeCache, 同一個容器之後只要 0.6 s。
        以前沒有暖機時, 這 30 s 是卡在第一張影像的 callback 裡: 看起來像當掉,
        status 不印、Ctrl-C 也收不掉 (要等 CUDA 編完才回得到 Python)。
        """
        self.get_logger().info('YOLO 暖機中 —— 新的容器第一次要 ~30 s (CUDA 在編 kernel, '
                               '之後有快取), 這段時間 Ctrl-C 收不掉是正常的')
        t0 = time.perf_counter()
        try:
            n = self.det.imgsz
            self.det.detect(np.full((n, n, 3), 114, dtype=np.uint8))
        except Exception as e:
            self.get_logger().error(f'YOLO 暖機失敗: {e}')
            return
        self.get_logger().info(f'YOLO 暖機完成 ({time.perf_counter() - t0:.1f} s)')

    # ------------------------------------------------------------------
    def on_odom(self, m: Odometry):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (stamp_sec(m.header.stamp), m.pose.pose.position.x,
                     m.pose.pose.position.y, yaw, m.twist.twist.linear.x,
                     m.twist.twist.angular.z)

    def _predict(self, t):
        """影像那一刻車大概在哪 (等速外推)。太舊就回 None。"""
        if self.odom is None:
            return None
        t0, x, y, yaw, v, wz = self.odom
        dt = t - t0
        if abs(dt) > self.roi_timeout:
            return None
        h = yaw + self.phi
        return (x + v * math.cos(h) * dt, y + v * math.sin(h) * dt, yaw + wz * dt)

    def _to_odom_clock(self, s: CamState, t: float) -> float:
        """影像時戳 -> odom 的時鐘。前 CLOCK_SAMPLES 張量 median(影像 - odom), 差太多就鎖定扣掉。

        只用在 ROI 預測; 發出去的框還是原始時戳 (融合節點自己會量、自己扣)。
        """
        if not s.clock_locked and self.odom is not None:
            s.clock_buf.append(t - self.odom[0])
            if len(s.clock_buf) >= CLOCK_SAMPLES:
                med = sorted(s.clock_buf)[len(s.clock_buf) // 2]
                if abs(med) > CLOCK_MAX_OFFSET:
                    s.clock_offset = med
                    self.get_logger().warn(
                        f'{s.cam.name} 的影像時戳跟 odom 差 {med:+.1f} s —— 時鐘基準不一樣, '
                        f'ROI 預測會扣掉它。根本解法是重開 Isaac (Stop/Play 不會讓它歸零)')
                s.clock_locked = True
        return t - s.clock_offset

    def on_image(self, msg: Image, s: CamState):
        t = stamp_sec(msg.header.stamp)
        if self.min_dt and t - s.last_t < self.min_dt and t >= s.last_t:
            return
        s.last_t = t
        s.n_img += 1
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'{s.cam.name} 影像轉換失敗: {e}')
            return
        H, W = bgr.shape[:2]
        # 影像像素 -> 校正解析度的像素
        sx, sy = s.cam.width / float(W), s.cam.height / float(H)

        roi = None
        pred = None
        st = self._predict(self._to_odom_clock(s, t)) if self.roi else None
        if st is not None:
            pred, _ = self.car.predict_bbox(s.cam, *st, jac=False)
            if pred is not None:
                b = pred / np.array([sx, sy, sx, sy])
                if b[2] > 0 and b[3] > 0 and b[0] < W and b[1] < H:
                    side = int(max(max(b[2] - b[0], b[3] - b[1]) * self.roi_scale,
                                   self.roi_min))
                    x0 = int(round(0.5 * (b[0] + b[2]))) - side // 2
                    y0 = int(round(0.5 * (b[1] + b[3]))) - side // 2
                    # 視窗放得進原圖就推回圖內 (少補灰); 比原圖大就置中, 外面補灰
                    if side <= W:
                        x0 = min(max(x0, 0), W - side)
                    if side <= H:
                        y0 = min(max(y0, 0), H - side)
                    roi = (x0, y0, side)

        boxes = []
        due_full = t - s.last_full >= self.full_period or t < s.last_full
        if roi is not None:
            boxes = self._detect(bgr, roi)
            s.n_roi += 1
            full = not boxes and due_full        # ROI 裡沒有: 偶爾看一下整張
        else:
            # 沒有預測 (還沒初始化 / 預測太舊) -> 每張都跑整張;
            # 預測說車不在這台的畫面裡 -> 只偶爾跑
            full = st is None or due_full
            s.n_skip += int(not full)
        if full:
            boxes = self._detect(bgr, None)
            s.last_full = t
            s.n_full += 1

        out = [(x1 * sx, y1 * sy, x2 * sx, y2 * sy, c, k) for x1, y1, x2, y2, c, k in boxes]
        s.n_det += int(bool(out))
        s.pub.publish(bbox_msg.encode(msg.header, s.cam.name, out))
        if s.pub_dbg is not None:
            self._debug(msg, bgr, s, roi, pred, sx, sy, boxes)

    def _detect(self, bgr, roi):
        """roi = None: 整張; (x0, y0, side): 那個視窗縮進 imgsz x imgsz 的畫布 (見模組說明)。"""
        if roi is None:
            img, k, ox, oy = bgr, 1.0, 0, 0
        else:
            img, k, ox, oy = self._canvas(bgr, roi)
            if img is None:
                return []
        t0 = time.perf_counter()
        try:
            dets = self.det.detect(np.ascontiguousarray(img))
        except Exception as e:
            self.get_logger().error(f'YOLO 推論失敗: {e}')
            return []
        self.infer_ms = 0.9 * self.infer_ms + 0.1 * (time.perf_counter() - t0) * 1e3
        return [((d.px - d.w / 2) / k + ox, (d.py - d.h / 2) / k + oy,
                 (d.px + d.w / 2) / k + ox, (d.py + d.h / 2) / k + oy, d.conf, d.cls)
                for d in dets]

    def _canvas(self, bgr, roi):
        """把視窗縮放 k = imgsz / side 倍貼到灰色畫布上。回傳 (畫布, k, 視窗左上角)。"""
        import cv2
        H, W = bgr.shape[:2]
        x0, y0, side = roi
        n = self.det.imgsz
        k = n / float(side)
        xa, ya, xb, yb = max(x0, 0), max(y0, 0), min(x0 + side, W), min(y0 + side, H)
        if xb <= xa or yb <= ya:
            return None, k, x0, y0
        w = max(1, min(n, int(round((xb - xa) * k))))
        h = max(1, min(n, int(round((yb - ya) * k))))
        crop = cv2.resize(bgr[ya:yb, xa:xb], (w, h),
                          interpolation=cv2.INTER_AREA if k < 1 else cv2.INTER_LINEAR)
        can = np.full((n, n, 3), 114, dtype=np.uint8)
        px, py = int(round((xa - x0) * k)), int(round((ya - y0) * k))
        w, h = min(w, n - px), min(h, n - py)
        can[py:py + h, px:px + w] = crop[:h, :w]
        return can, k, x0, y0

    def _debug(self, msg, bgr, s, roi, pred, sx, sy, boxes):
        import cv2
        img = bgr.copy()
        if roi is not None:
            cv2.rectangle(img, (roi[0], roi[1]), (roi[0] + roi[2], roi[1] + roi[2]),
                          (0, 255, 255), 2)
        if pred is not None:
            p = (pred / np.array([sx, sy, sx, sy])).astype(int)
            cv2.rectangle(img, (p[0], p[1]), (p[2], p[3]), (255, 0, 0), 2)
        for x1, y1, x2, y2, c, _ in boxes:
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(img, f'{c:.2f}', (int(x1), int(y1) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = msg.header
        s.pub_dbg.publish(out)

    def status(self):
        # 同一台相機的框有兩個以上的人在發 = 開了兩個偵測節點 (例如 collect_all 之外又開了
        # multicam_loc.launch.py)。兩個 YOLO 各訂閱 8 台 RELIABLE 影像, 2026-11-03 那兩輪
        # Isaac 的相機與 LiDAR 一起斷了 51 s / 130 s。不自己退出 —— 兩個同時起來會一起退。
        n_pub = self.count_publishers(self.cams[0].detection_topic) if self.cams else 0
        if n_pub > 1:
            self.get_logger().error(
                f'{self.cams[0].detection_topic} 有 {n_pub} 個節點在發 —— 開了兩個偵測節點! '
                '關掉其中一個 (collect_all / friction_test 已經包含方法六, 不要再開 '
                'multicam_loc.launch.py; 真的要開就給 detector:=false)')
        parts = []
        for s in self.state.values():
            if s.n_img == 0:
                parts.append(f'{s.cam.name[-1]}:無影像')
                continue
            parts.append(f'{s.cam.name[-1]}:{100.0 * s.n_det / s.n_img:.0f}%'
                         f'(roi{s.n_roi}/整張{s.n_full}/略過{s.n_skip})')
        self.get_logger().info(f'偵測率 {" ".join(parts)} | 推論 {self.infer_ms:.0f} ms'
                               + ('' if self.odom else ' | 還沒有預測位置 (全部跑整張)'))


def main(args=None):
    rclpy.init(args=args)
    node = MultiCamDetector()
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
