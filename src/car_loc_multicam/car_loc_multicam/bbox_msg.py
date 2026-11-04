#!/usr/bin/env python3
"""偵測結果的訊息格式: `geometry_msgs/PolygonStamped`。

為什麼不用 vision_msgs/Detection2DArray: 容器 (sim2real2sim:v1) 裡沒有裝 vision_msgs,
而自己定義 .msg 要多一個 ament_cmake 的 interface package。這裡要的只是
「時戳 + 相機名字 + 幾個框」, PolygonStamped 剛好有 header, 所以借它裝:

    header.stamp     = **曝光時刻** (Isaac: 影像的時戳; 實體 Pi: SensorTimestamp 換成 PTP 時間)
    header.frame_id  = 相機名字 (env_cam_0 ...), 要跟 env_cameras.yaml 的 name 一樣
    polygon.points   = 每個框兩個點:
                         (u1, v1, conf)   左上角 + 信心
                         (u2, v2, class)  右下角 + 類別
    沒有點 = 這一幀這台相機沒看到車 (還是要發, 融合節點用它知道相機活著)

像素一律是 **env_cameras.yaml 的解析度** (width x height), 連續座標 (左上角那一格的
左緣 = 0, 跟 YOLO 的 xyxy 一樣)。影像實際解析度不同時由偵測節點換算。
實體相機在 Pi 上跑 YOLO 的話, 照這個格式發就能直接接進融合節點。
"""
from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point32, PolygonStamped


def encode(header, frame_id, boxes) -> PolygonStamped:
    """boxes: [(x1, y1, x2, y2, conf, cls), ...]"""
    m = PolygonStamped()
    m.header.stamp = header.stamp
    m.header.frame_id = str(frame_id)
    for x1, y1, x2, y2, conf, cls in boxes:
        m.polygon.points.append(Point32(x=float(x1), y=float(y1), z=float(conf)))
        m.polygon.points.append(Point32(x=float(x2), y=float(y2), z=float(cls)))
    return m


def decode(msg: PolygonStamped) -> list:
    """-> [(box (4,), conf), ...]"""
    pts = msg.polygon.points
    out = []
    for i in range(0, len(pts) - 1, 2):
        a, b = pts[i], pts[i + 1]
        out.append((np.array([a.x, a.y, b.x, b.y], dtype=np.float64), float(a.z)))
    return out
