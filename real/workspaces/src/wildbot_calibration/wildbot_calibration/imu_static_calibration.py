"""IMU static calibration: gyro/accel bias and covariance from 30 s static."""

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
#     - IMU topic: src/wildbot_imu_node.cpp:70
#       create_publisher<sensor_msgs::msg::Imu>("imu/data", 10)
#       node name "imu" with default namespace "/" -> "/imu/data"
# ============================================================
IMU_TOPIC = "/imu/data"

STATIC_DURATION_SEC = 30.0
GRAVITY_NOMINAL = 9.80665
FIRST_MSG_TIMEOUT = 5.0

GYRO_STD_WARN = 0.01     # rad/s
GRAVITY_DEV_WARN = 0.5   # m/s^2
ACCEL_XY_WARN = 1.0      # m/s^2


class StaticCollector(Node):
    def __init__(self) -> None:
        super().__init__("imu_static_calibration")
        qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(Imu, IMU_TOPIC, self._on_imu, qos)
        self._latest: Optional[Imu] = None
        self._collecting = False
        self._gyro: list[list[float]] = []
        self._accel: list[list[float]] = []

    def _on_imu(self, msg: Imu) -> None:
        self._latest = msg
        if not self._collecting:
            return
        self._gyro.append(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        )
        self._accel.append(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]
        )

    def wait_first_message(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest is not None:
                return True
        return False

    def collect(self, seconds: float) -> tuple[np.ndarray, np.ndarray]:
        self._gyro = []
        self._accel = []
        self._collecting = True
        end = time.monotonic() + seconds
        last_print = 0.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_print >= 1.0:
                remaining = end - now
                print(f"\r  收集中... 剩 {remaining:5.1f} 秒, 樣本 {len(self._gyro)}", end="")
                last_print = now
        print()
        self._collecting = False
        return np.asarray(self._gyro), np.asarray(self._accel)


def _print_summary(gyro: np.ndarray, accel: np.ndarray) -> dict:
    gyro_mean = gyro.mean(axis=0)
    gyro_std = gyro.std(axis=0)
    gyro_var = gyro.var(axis=0)
    accel_mean = accel.mean(axis=0)
    accel_std = accel.std(axis=0)
    accel_var = accel.var(axis=0)
    gravity = float(np.linalg.norm(accel_mean))

    prompts.header("Summary")
    print(f"  樣本數:{len(gyro)} (持續 {STATIC_DURATION_SEC:.0f} 秒)")
    print()
    print("  Gyro (rad/s)        x          y          z")
    print(f"    mean (bias)  {gyro_mean[0]:+10.6f} {gyro_mean[1]:+10.6f} {gyro_mean[2]:+10.6f}")
    print(f"    std          {gyro_std[0]:10.6f} {gyro_std[1]:10.6f} {gyro_std[2]:10.6f}")
    print(f"    variance     {gyro_var[0]:.3e} {gyro_var[1]:.3e} {gyro_var[2]:.3e}")
    print()
    print("  Accel (m/s^2)       x          y          z")
    print(f"    mean         {accel_mean[0]:+10.6f} {accel_mean[1]:+10.6f} {accel_mean[2]:+10.6f}")
    print(f"    std          {accel_std[0]:10.6f} {accel_std[1]:10.6f} {accel_std[2]:10.6f}")
    print(f"    variance     {accel_var[0]:.3e} {accel_var[1]:.3e} {accel_var[2]:.3e}")
    print()
    print(f"  |g| = {gravity:.4f} m/s^2 (理論 {GRAVITY_NOMINAL:.4f})")
    print()

    warnings: list[str] = []
    if (gyro_std > GYRO_STD_WARN).any():
        warnings.append(
            f"⚠️  gyro std 超過 {GYRO_STD_WARN} rad/s,車可能在動或 IMU 受振動。"
        )
    if abs(gravity - GRAVITY_NOMINAL) > GRAVITY_DEV_WARN:
        warnings.append(
            f"⚠️  |g| 偏離 {GRAVITY_DEV_WARN} m/s^2,IMU 可能沒水平,或加速度計範圍/單位有問題。"
        )
    if abs(accel_mean[0]) > ACCEL_XY_WARN or abs(accel_mean[1]) > ACCEL_XY_WARN:
        warnings.append(
            f"⚠️  accel x/y 偏離 0 超過 {ACCEL_XY_WARN} m/s^2,IMU 可能斜放。"
        )

    for w in warnings:
        print(f"  {w}")
    if not warnings:
        print("  ✓ 各項 sanity check 通過。")

    return {
        "gyro_mean": gyro_mean,
        "gyro_var": gyro_var,
        "accel_mean": accel_mean,
        "accel_var": accel_var,
        "gravity": gravity,
    }


def main() -> None:
    rclpy.init()
    node = StaticCollector()
    try:
        prompts.header("IMU 靜態校正(bias + covariance)")
        print(f"  將收集 {STATIC_DURATION_SEC:.0f} 秒靜態資料,計算 gyro bias、accel xy bias、")
        print("  並用樣本變異數作為 covariance diagonal。")
        print(f"  IMU topic = {IMU_TOPIC}")
        print()

        existing = yaml_store.get_section("imu")
        if not existing.get("axis_sanity_passed", False):
            print("  ⚠️  imu.axis_sanity_passed 為 false,建議先跑 imu_axis_sanity。")
            if not prompts.confirm("  仍要繼續?", default=False):
                print("  已取消。")
                return

        print()
        print("  等待第一筆 IMU 訊息...")
        if not node.wait_first_message(FIRST_MSG_TIMEOUT):
            print("  ✗ Timeout,收不到 IMU 資料。")
            return

        if not prompts.confirm("  車已完全靜止、不會被碰到嗎?", default=False):
            print("  已取消。")
            return
        prompts.countdown(5, "  即將開始收集...")

        gyro, accel = node.collect(STATIC_DURATION_SEC)
        if len(gyro) < 30:
            print(f"  ✗ 樣本太少({len(gyro)}),IMU rate 太低或斷線。")
            return

        stats = _print_summary(gyro, accel)
        print()
        if not prompts.confirm("  寫入 YAML?", default=True):
            print("  未寫入。")
            return

        gyro_mean = stats["gyro_mean"]
        gyro_var = stats["gyro_var"]
        accel_mean = stats["accel_mean"]
        accel_var = stats["accel_var"]
        gravity = stats["gravity"]

        path = yaml_store.update_section(
            "imu",
            {
                "gyro_bias": [float(gyro_mean[0]), float(gyro_mean[1]), float(gyro_mean[2])],
                "gyro_covariance_diag": [
                    float(gyro_var[0]),
                    float(gyro_var[1]),
                    float(gyro_var[2]),
                ],
                "accel_bias_xy": [float(accel_mean[0]), float(accel_mean[1])],
                "accel_covariance_diag": [
                    float(accel_var[0]),
                    float(accel_var[1]),
                    float(accel_var[2]),
                ],
                "accel_gravity_magnitude": float(gravity),
            },
        )
        print(f"  YAML 已更新:{path}")
        print()
        print("  📝 提示:請更新你的 IMU driver 讓它從 ~/.ros/calibration_results.yaml")
        print("     讀取 gyro_bias / gyro_covariance_diag,從 raw reading 扣除 bias、")
        print("     並把 covariance 寫入 sensor_msgs/Imu 的 *_covariance 欄位。")
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
