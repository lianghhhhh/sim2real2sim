"""天花板相機 + YOLO -> 車子世界座標。

跟舊版比的三個改動 (依重要性排序):

1) 座標轉換從「像素偏移乘一個比例」改成「單應性 (homography)」。
   舊版假設影像中心對應世界原點, 而且 x/y 各乘一個手量的比例。這個模型
   漏掉了視差: 相機在 z=2.7 m, 但 bbox 中心看到的是車身 (z ≈ 0.10 m)
   而不是接地點, 所以量到的半徑比真值大 2.7/(2.7-0.10) = 3.7%。實測擬合
   出來的比例正好是 0.964 —— 對得上。
   單應性是針孔相機看一個平面的精確模型, 把它直接擬合到「車身高度那個
   平面」, 視差、相機傾斜、主點偏移、安裝旋轉全部一起吸收掉。
   實測 (1072 筆, 5-fold 交叉驗證): RMSE 0.136 m -> 0.073 m。

2) 解析度不再寫死。舊版寫 1920x1536, 但相機的內參是照 1920x1200 抄的,
   兩者搭不起來; 現在改成用 msg.width / msg.height, 並依比例縮放校正參數,
   所以改 render product 解析度不會默默算錯。

3) 加發 PoseStamped, 而且 header.stamp 沿用影像的時戳。
   舊版只發 Float32MultiArray (沒有時戳), 下游只能「取最新值」, 無法跟
   ground truth 做時間對齊。同時多發原始像素座標, 之後要重新校正時
   可以直接拿 log 來擬合, 不用反推。
"""
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO


class GroundProjector:
    """像素 -> 地面世界座標 (含徑向去畸變 + 單應性)。"""

    def __init__(self, cfg):
        self.ref_w = float(cfg['image_width'])
        self.ref_h = float(cfg['image_height'])
        self.center = np.asarray(cfg['center_px'], dtype=np.float64)
        self.scale = float(cfg['norm_scale'])
        self.k1, self.k2 = (list(cfg.get('distortion', [0.0, 0.0])) + [0.0, 0.0])[:2]
        self.H = np.asarray(cfg['homography'], dtype=np.float64)

    def __call__(self, px, py, width, height):
        # 校正參數是在 ref_w x ref_h 上擬合的; 影像解析度不同就先換算回去,
        # 不然改一次 render product 解析度就整個歪掉。
        px = px * (self.ref_w / width)
        py = py * (self.ref_h / height)

        d = (np.array([px, py]) - self.center) / self.scale
        r2 = float(d @ d)
        d = d * (1.0 + self.k1 * r2 + self.k2 * r2 * r2)

        v = self.H @ np.array([d[0], d[1], 1.0])
        if abs(v[2]) < 1e-9:
            raise ValueError('單應性退化 (w≈0), 校正檔可能不對')
        return float(v[0] / v[2]), float(v[1] / v[2])


class CarInferenceNode(Node):
    def __init__(self):
        super().__init__('car_inference_node')

        share = get_package_share_directory('car_inference')
        self.declare_parameter('model_path', os.path.join(share, 'resource', 'best.onnx'))
        self.declare_parameter('calibration_path',
                               os.path.join(share, 'config', 'camera_ground.yaml'))
        self.declare_parameter('image_topic', '/rgb')
        # 必須跟 best.onnx 匯出時的尺寸一致。這個模型是用固定的 512x512 匯出的
        # (ONNX 沒有 dynamic axes), 填別的值會直接在推論時報
        # "Got invalid dimensions for input: images"。
        #
        # 想提高解析度不是改這個參數就好, 要重新匯出模型:
        #     yolo export model=best.pt format=onnx imgsz=960
        # 不過不見得值得: 實測 bbox 中心的逐幀抖動只有 1.5 px (0.8 cm),
        # 遠小於校正殘差 (7 cm), 提高解析度不會動到誤差的主要來源。
        self.declare_parameter('imgsz', 512)
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_annotated', True)

        model_path = self.get_parameter('model_path').value
        calib_path = self.get_parameter('calibration_path').value
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.conf = float(self.get_parameter('conf').value)
        self.world_frame = self.get_parameter('world_frame').value
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)

        if not os.path.exists(model_path):
            raise FileNotFoundError(f'找不到模型 {model_path}')
        if not os.path.exists(calib_path):
            raise FileNotFoundError(
                f'找不到校正檔 {calib_path}; '
                f'請跑 scripts/calibrate_camera_ground.py 產生')

        with open(calib_path) as f:
            cfg = yaml.safe_load(f)
        self.project = GroundProjector(cfg)
        self.get_logger().info(f'校正檔: {calib_path} (擬合解析度 '
                               f'{cfg["image_width"]}x{cfg["image_height"]})')

        self.model = YOLO(model_path, task='detect')
        self.bridge = CvBridge()
        self.get_logger().info(f'模型載入完成: {model_path}')

        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self.image_callback, 10)

        self.yolo_publisher = self.create_publisher(Image, '/yolo/detections', 10)
        self.yolo_coord_publisher = self.create_publisher(
            Float32MultiArray, '/yolo/detections_coord', 10)
        # 帶時戳的版本 —— 下游要跟 odom 做時間對齊時用這個
        self.pose_publisher = self.create_publisher(PoseStamped, '/yolo/pose', 10)
        # 原始像素 [px, py, conf]; 重新校正時直接餵給 calibrate_camera_ground.py
        self.pixel_publisher = self.create_publisher(
            Float32MultiArray, '/yolo/detection_px', 10)

        self.miss_count = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.predict(source=cv_image, imgsz=self.imgsz,
                                         conf=self.conf, verbose=False)
            result = results[0]

            if self.publish_annotated:
                img_msg = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
                img_msg.header = msg.header
                self.yolo_publisher.publish(img_msg)

            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                self.miss_count += 1
                if self.miss_count % 30 == 1:
                    self.get_logger().warn(f'第 {self.miss_count} 次沒有偵測到車子')
                return
            self.miss_count = 0

            # 舊版固定取 boxes[0]。畫面裡只要多一個誤判 (影子、反光), 排序一變
            # 就會跳到別的框上, 軌跡出現整段偏移。改成取信心最高的那個。
            best = int(np.argmax(boxes.conf.cpu().numpy()))
            x1, y1, x2, y2 = boxes.xyxy[best].cpu().numpy().astype(np.float64)
            conf = float(boxes.conf[best].cpu().numpy())
            px, py = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            h, w = cv_image.shape[:2]
            real_x, real_y = self.project(px, py, w, h)

            self.yolo_coord_publisher.publish(
                Float32MultiArray(data=[real_x, real_y]))
            self.pixel_publisher.publish(
                Float32MultiArray(data=[float(px), float(py), conf]))

            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp     # 沿用影像時戳, 不是「現在」
            pose.header.frame_id = self.world_frame
            pose.pose.position.x = real_x
            pose.pose.position.y = real_y
            pose.pose.orientation.w = 1.0
            self.pose_publisher.publish(pose)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CarInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch 送 SIGINT 時 rclpy 的訊號處理可能已經關掉 context, 再關一次會丟
        # RCLError 並讓行程以 exit code 1 收場, 看起來像節點掛了。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
