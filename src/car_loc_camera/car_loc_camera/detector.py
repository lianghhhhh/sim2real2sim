#!/usr/bin/env python3
"""YOLO 偵測的薄包裝。

只做一件事: 影像 -> 這一幀所有候選框的 (中心像素, 信心, 框大小)。
**要挑哪一個是節點的事**, 不是這裡的事 —— 因為挑框需要「上一刻車在哪」,
那是濾波器才有的資訊 (見 camera_loc_node 的 _pick)。

舊做法固定取 `boxes[0]`, 畫面裡只要多一個誤判 (影子、反光), 排序一變就會跳到
別的框上, 軌跡出現整段偏移。
"""
from __future__ import annotations

import os

import numpy as np


class Detection:
    __slots__ = ('px', 'py', 'conf', 'w', 'h', 'cls')

    def __init__(self, px, py, conf, w, h, cls=0):
        self.px = float(px)
        self.py = float(py)
        self.conf = float(conf)
        self.w = float(w)
        self.h = float(h)
        self.cls = int(cls)

    def __repr__(self):
        return (f'Detection(px={self.px:.1f}, py={self.py:.1f}, '
                f'conf={self.conf:.2f}, {self.w:.0f}x{self.h:.0f})')


def find_model(explicit: str = '') -> str:
    """依序找模型檔: 參數 -> 環境變數 -> 本 package 的 resource (share, 再來原始碼)。

    (以前最後還會退回 car_inference 的 resource; 那個 package 已經刪掉,
    best.onnx 已經搬進本 package 的 resource/。)
    """
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [explicit, os.environ.get('CAR_YOLO_MODEL', '')]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(
            get_package_share_directory('car_loc_camera'), 'resource', 'best.onnx'))
    except Exception:
        pass
    cands.append(os.path.join(here, '..', 'resource', 'best.onnx'))

    for c in cands:
        if c and os.path.exists(c):
            return os.path.abspath(c)
    raise FileNotFoundError(
        '找不到 YOLO 模型。給 -p model_path:=/path/to/best.onnx, 或設環境變數 '
        'CAR_YOLO_MODEL, 或把模型複製到 src/car_loc_camera/resource/best.onnx')


class CarDetector:

    def __init__(self, model_path: str = '', imgsz: int = 512, conf: float = 0.25,
                 class_id: int = -1):
        from ultralytics import YOLO
        self.path = find_model(model_path)
        # imgsz 必須跟 best.onnx 匯出時的尺寸一致。這個模型是用固定的 512x512
        # 匯出的 (ONNX 沒有 dynamic axes), 填別的值會直接在推論時報
        # "Got invalid dimensions for input: images"。要換解析度得重新匯出:
        #     yolo export model=best.pt format=onnx imgsz=960
        # 不過不見得值得: bbox 中心的逐幀抖動只有 1.5 px (0.8 cm), 遠小於
        # 校正殘差 (7 cm), 提高解析度不會動到誤差的主要來源。
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.class_id = int(class_id)
        self.model = YOLO(self.path, task='detect')
        self._last = None

    def detect(self, bgr) -> list:
        res = self.model.predict(source=bgr, imgsz=self.imgsz, conf=self.conf,
                                 verbose=False)[0]
        self._last = res
        boxes = res.boxes
        out = []
        if boxes is None or len(boxes) == 0:
            return out
        xyxy = boxes.xyxy.cpu().numpy().astype(np.float64)
        cf = boxes.conf.cpu().numpy().astype(np.float64)
        cl = (boxes.cls.cpu().numpy().astype(np.int64)
              if boxes.cls is not None else np.zeros(len(cf), dtype=np.int64))
        for (x1, y1, x2, y2), c, k in zip(xyxy, cf, cl):
            if self.class_id >= 0 and int(k) != self.class_id:
                continue
            out.append(Detection((x1 + x2) / 2.0, (y1 + y2) / 2.0, c,
                                 x2 - x1, y2 - y1, k))
        return out

    def annotated(self):
        """上一次 detect 的標註圖 (BGR)。沒有偵測過就回 None。"""
        return None if self._last is None else self._last.plot()
