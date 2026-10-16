"""Two-face accelerometer sanity (simplified 6-face).

Pose 1: flat -> expect accel ≈ (0, 0, +g).
Pose 2: front pitched up ~90° -> expect accel ≈ (+g, 0, 0) (validates x-forward).
TODO: full 6-face procedure after competition.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

from wildbot_calibration.common import prompts
from wildbot_calibration.common import yaml_store


# ============================================================
# Topics / frames discovered from user's existing packages
#   Source: wildbot_imu at ../wildbot_imu/
#     - IMU topic: src/wildbot_imu_node.cpp:70 -> "/imu/data"
# ============================================================
IMU_TOPIC = "/imu/data"

POSE_DURATION = 10.0
GRAVITY_NOMINAL = 9.80665
DEVIATION_WARN = 1.0     # m/s^2
FIRST_MSG_TIMEOUT = 5.0


class _AccelCollector(Node):
    def __init__(self) -> None:
        super().__init__("imu_accel_two_face")
        qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(Imu, IMU_TOPIC, self._on_imu, qos)
        self._latest: Optional[Imu] = None
        self._collecting = False
        self._samples: list[list[float]] = []

    def _on_imu(self, msg: Imu) -> None:
        self._latest = msg
        if not self._collecting:
            return
        self._samples.append(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]
        )

    def wait_first(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest is not None:
                return True
        return False

    def collect(self, seconds: float) -> np.ndarray:
        self._samples = []
        self._collecting = True
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._collecting = False
        return np.asarray(self._samples)


def _collect_pose(node: _AccelCollector, label: str, instruction: str) -> Optional[np.ndarray]:
    print()
    print(f"  姿態:{label}")
    print(f"  指示:{instruction}")
    if not prompts.confirm("  已就定位、車完全靜止?", default=False):
        print("  已取消。")
        return None
    prompts.countdown(3, f"  開始收集 {POSE_DURATION:.0f} 秒...")
    arr = node.collect(POSE_DURATION)
    if len(arr) < 30:
        print(f"  ✗ 樣本太少({len(arr)})。")
        return None
    mean = arr.mean(axis=0)
    print(f"    accel mean (m/s^2)  ax={mean[0]:+.4f}  ay={mean[1]:+.4f}  az={mean[2]:+.4f}")
    return mean


def main() -> None:
    rclpy.init()
    node = _AccelCollector()
    try:
        prompts.header("IMU 加速度計兩姿態驗證")
        print("  姿態 1:車水平正放,靜止 10 秒")
        print("  姿態 2:車前端朝天花板(pitch ≈ 90°),靜止 10 秒")
        print(f"  IMU topic = {IMU_TOPIC}")
        print()

        print(f"  等待 IMU 訊息(timeout {FIRST_MSG_TIMEOUT:.0f} 秒)...")
        if not node.wait_first(FIRST_MSG_TIMEOUT):
            print("  ✗ Timeout。")
            return

        flat = _collect_pose(node, "水平正放", "車四輪著地、保持完全靜止")
        if flat is None:
            return
        pitched = _collect_pose(
            node,
            "前端朝天 ~90°",
            "把車前端立起、後輪靠地(像準備往天花板飛),靜止 10 秒。注意不要摔到!",
        )
        if pitched is None:
            return

        prompts.header("Summary")
        flat_expect = np.array([0.0, 0.0, GRAVITY_NOMINAL])
        pitch_expect = np.array([GRAVITY_NOMINAL, 0.0, 0.0])

        print("  姿態 1(水平),理論 (0, 0, +g):")
        flat_dev = flat - flat_expect
        print(f"    ax={flat[0]:+.4f}  (偏差 {flat_dev[0]:+.4f})")
        print(f"    ay={flat[1]:+.4f}  (偏差 {flat_dev[1]:+.4f})")
        print(f"    az={flat[2]:+.4f}  (偏差 {flat_dev[2]:+.4f})")
        print()
        print("  姿態 2(前端朝天),理論 (+g, 0, 0):")
        pitch_dev = pitched - pitch_expect
        print(f"    ax={pitched[0]:+.4f}  (偏差 {pitch_dev[0]:+.4f})")
        print(f"    ay={pitched[1]:+.4f}  (偏差 {pitch_dev[1]:+.4f})")
        print(f"    az={pitched[2]:+.4f}  (偏差 {pitch_dev[2]:+.4f})")
        print()

        warnings: list[str] = []
        max_flat = float(np.max(np.abs(flat_dev)))
        max_pitch = float(np.max(np.abs(pitch_dev)))
        if max_flat > DEVIATION_WARN:
            warnings.append(
                f"⚠️  水平姿態偏差最大 {max_flat:.3f} m/s^2 > {DEVIATION_WARN};"
                " IMU 沒水平或軸向錯。"
            )
        if max_pitch > DEVIATION_WARN:
            warnings.append(
                f"⚠️  pitch 姿態偏差最大 {max_pitch:.3f} m/s^2 > {DEVIATION_WARN};"
                " 抬車角度沒到 90°,或 x 軸方向有問題。"
            )
        for w in warnings:
            print(f"  {w}")
        if not warnings:
            print("  ✓ 兩姿態都在容差內。")

        print()
        if not prompts.confirm("  以水平姿態的 ax, ay 更新 accel_bias_xy 並寫入 YAML?", default=True):
            print("  未寫入。")
            return

        path = yaml_store.update_section(
            "imu",
            {
                "accel_bias_xy": [float(flat[0]), float(flat[1])],
            },
        )
        print(f"  YAML 已更新:{path}")
        print("  (完整六面校正:待賽後實作。)")
    except KeyboardInterrupt:
        print("\n  使用者中斷。")
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
