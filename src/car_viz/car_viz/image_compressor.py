#!/usr/bin/env python3
"""/rgb (sensor_msgs/Image) -> /rgb/compressed (sensor_msgs/CompressedImage, JPEG)。

為什麼需要這個節點:

    Isaac 發的 /rgb 是 1920x1536 rgb8, 一張 8.85 MB, 50 Hz —— 每秒 440 MB。
    foxglove_bridge 的 send_buffer_limit 預設才 10 MB, 一張圖就快塞滿, Foxglove
    裡點一下 /rgb, bridge 就會開始丟訊息、卡死, 連帶地圖和位姿全部停住。

    只壓縮不夠: 全解析度 JPEG 一張 18 ms、370 KB, 50 Hz 編不完, 就算編得完也是
    每秒 18 MB 往 SSH tunnel 塞。所以這裡三件事一起做:

        限頻   max_rate   (預設 10 Hz)  —— 多的幀直接丟, 連解碼都不做
        縮圖   max_width  (預設 960 px) —— 實測編碼 18 ms -> 4.4 ms
        壓縮   jpeg_quality (預設 70)   —— Isaac 畫面實測一張 ~18 KB

    預設值下實測 ~9 Hz x 18 KB = 每秒 0.16 MB, 原本的 1/2700。(影像 50 Hz 進來、
    限頻是「兩張至少隔 100 ms」, 所以實際會落在 9 Hz 上下, 不是剛好 10。)

為什麼不用 image_transport republish: 它只做壓縮, 不能限頻也不能縮圖, 在 50 Hz
全解析度下還是一樣會把 bridge 塞爆。

沒有人訂 /rgb/compressed 的時候, 連 /rgb 都不訂 —— 光是 DDS 把 440 MB/s 送進這個
process 就是一筆開銷, Isaac 那邊也得多餵一個訂閱者。每秒檢查一次有沒有人在看。
"""
from __future__ import annotations

import array
import time

import cv2
import numpy as np

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage, Image

# 只要最新的一張: 處理不過來就丟, 不要在佇列裡堆 8.85 MB 的舊圖
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)

# encoding -> (通道數, 轉成 OpenCV 要的 BGR / 灰階的 cvtColor 代碼, None = 不用轉)
ENCODINGS = {
    'rgb8': (3, cv2.COLOR_RGB2BGR),
    'bgr8': (3, None),
    'rgba8': (4, cv2.COLOR_RGBA2BGR),
    'bgra8': (4, cv2.COLOR_BGRA2BGR),
    'mono8': (1, None),
}


def to_cv(msg: Image) -> np.ndarray:
    """Image -> BGR (或灰階) ndarray。不經過 cv_bridge, frombuffer 不複製資料。"""
    if msg.encoding not in ENCODINGS:
        raise ValueError(f'不支援的 encoding "{msg.encoding}" (只吃 {", ".join(ENCODINGS)})')
    ch, code = ENCODINGS[msg.encoding]
    # step 可能比 width*ch 大 (每列尾端補齊), 先照 step 切再裁掉
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * ch]
    if ch > 1:
        img = img.reshape(msg.height, msg.width, ch)
    return cv2.cvtColor(img, code) if code is not None else img


class ImageCompressor(Node):

    def __init__(self):
        super().__init__('image_compressor')
        p = self.declare_parameter
        p('image_topic', '/rgb')
        p('output_topic', '')          # 空 = <image_topic>/compressed (image_transport 慣例)
        p('max_rate', 10.0)            # Hz, <= 0 不限頻
        p('max_width', 960)            # px, <= 0 不縮圖; 只縮不放大, 保持長寬比
        p('jpeg_quality', 70)          # 1~100
        p('status_period', 10.0)       # 秒, <= 0 不印; 沒人在看的時候不印

        g = self.get_parameter
        self.src = g('image_topic').value
        self.dst = g('output_topic').value or self.src.rstrip('/') + '/compressed'
        rate = float(g('max_rate').value)
        self.min_dt = 1.0 / rate if rate > 0 else 0.0
        self.max_width = int(g('max_width').value)
        self.params = [cv2.IMWRITE_JPEG_QUALITY, int(np.clip(g('jpeg_quality').value, 1, 100))]

        self.pub = self.create_publisher(CompressedImage, self.dst, 2)
        self.sub = None
        # 兩個 timer 都掛牆鐘 (steady), 不能用節點預設的時鐘: launch 給 use_sim_time:=true,
        # 模擬時間的 timer 只在 /clock 有在跳時才會觸發 —— Isaac 沒開 / 暫停 / bag 沒加
        # --clock 的時候, 「有沒有人在看」就永遠不會被檢查, 永遠不訂, Foxglove 一片黑。
        # 看的是 bridge 的需求跟頻寬, 本來就跟模擬時間無關 (限頻也是用牆鐘)。
        wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self.follow_demand, clock=wall)

        self.last_sent = 0.0
        self._reset_stats()
        self.warned = set()
        period = float(g('status_period').value)
        if period > 0:
            self.create_timer(period, self.report, clock=wall)

        self.get_logger().info(
            f'{self.src} -> {self.dst}  (最多 {rate:g} Hz, 寬 <= {self.max_width} px, '
            f'JPEG q{self.params[1]}; 有人訂 {self.dst} 才去訂 {self.src})')

    def follow_demand(self):
        """有人看才訂原始影像, 沒人看就退訂。"""
        watching = self.pub.get_subscription_count() > 0
        if watching and self.sub is None:
            self.sub = self.create_subscription(Image, self.src, self.on_image, IMAGE_QOS)
            self._reset_stats()
            self.get_logger().info(f'有人在看 {self.dst}, 開始訂 {self.src}')
        elif not watching and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            self.get_logger().info(f'沒人看了, 退訂 {self.src}')

    def _reset_stats(self):
        self.t0 = time.monotonic()
        self.n_in = self.n_out = 0
        self.bytes_in = self.bytes_out = 0
        self.encode_s = 0.0

    def on_image(self, msg: Image):
        self.n_in += 1
        # 限頻用牆鐘: 要保護的是 bridge 的頻寬, 跟模擬時間跑多快無關
        now = time.monotonic()
        if now - self.last_sent < self.min_dt:
            return

        t = time.perf_counter()
        try:
            img = to_cv(msg)
        except ValueError as e:
            if msg.encoding not in self.warned:     # 每種 encoding 只講一次, 不洗版
                self.warned.add(msg.encoding)
                self.get_logger().error(str(e))
            return
        if 0 < self.max_width < msg.width:
            h = max(1, round(msg.height * self.max_width / msg.width))
            img = cv2.resize(img, (self.max_width, h), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', img, self.params)
        if not ok:
            self.get_logger().error('cv2.imencode 失敗')
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        # 直接給 bytes 的話 setter 會逐元素檢查型別 (實測 408 KB 要 16 ms, 50 KB 也要 ~2 ms);
        # array('B') 直接過, 0.01 ms
        out.data = array.array('B', buf.tobytes())
        self.pub.publish(out)

        self.last_sent = now
        self.encode_s += time.perf_counter() - t
        self.n_out += 1
        self.bytes_in += len(msg.data)
        self.bytes_out += len(out.data)

    def report(self):
        if self.sub is None:
            return
        dt = time.monotonic() - self.t0
        if self.n_out == 0:
            self.get_logger().warn(f'有人在看但 {dt:.0f} 秒內沒收到 {self.src}, Isaac 有在跑嗎?')
        else:
            self.get_logger().info(
                f'收 {self.n_in / dt:.1f} Hz -> 發 {self.n_out / dt:.1f} Hz, '
                f'每張 {self.bytes_out / self.n_out / 1e3:.0f} KB '
                f'(原本 {self.bytes_in / self.n_out / 1e6:.2f} MB), '
                f'編碼 {self.encode_s / self.n_out * 1e3:.1f} ms, '
                f'頻寬 {self.bytes_out / dt / 1e6:.2f} MB/s')
        self._reset_stats()


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C 時終端機跟 launch 會各送一次 SIGINT, 第二發常常剛好落在收尾這裡;
        # 不接住的話會印 traceback、launch 報 "process has died", 看起來像節點掛了
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
