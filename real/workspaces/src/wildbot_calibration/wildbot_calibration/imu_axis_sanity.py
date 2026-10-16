"""IMU axis sanity check.

Passively observes IMU during four operator-driven motions, verifies the sign
of the dominant axis matches expectation. No cmd_vel publishing.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
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
#     - frame_id: launch/imu.launch.py:29  default "imu_link"
#   Source: kros_car at ../kros_car/
#     - base_frame_id: config/controllers.yaml:27 "base_link"
#
# If you change the IMU node name, namespace, or frame, update the constants
# below and re-build.
# ============================================================
IMU_TOPIC = "/imu/data"
BASE_FRAME = "base_link"
IMU_FRAME = "imu_link"

ACCEL_PEAK_THRESHOLD = 0.5    # m/s^2
GYRO_PEAK_THRESHOLD = 0.1     # rad/s
COLLECT_SECONDS = 3.0
FIRST_MSG_TIMEOUT = 5.0


@dataclass
class _Buffer:
    ax: list[float] = field(default_factory=list)
    ay: list[float] = field(default_factory=list)
    az: list[float] = field(default_factory=list)
    gx: list[float] = field(default_factory=list)
    gy: list[float] = field(default_factory=list)
    gz: list[float] = field(default_factory=list)
    qw: list[float] = field(default_factory=list)
    qx: list[float] = field(default_factory=list)
    qy: list[float] = field(default_factory=list)
    qz: list[float] = field(default_factory=list)


class AxisSanityNode(Node):
    """One-shot node: collect IMU samples on demand, no spin loop."""

    def __init__(self) -> None:
        super().__init__("imu_axis_sanity")
        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(Imu, IMU_TOPIC, self._on_imu, qos)
        self._collecting = False
        self._buf = _Buffer()
        self._latest: Optional[Imu] = None

    def _on_imu(self, msg: Imu) -> None:
        self._latest = msg
        if not self._collecting:
            return
        self._buf.ax.append(msg.linear_acceleration.x)
        self._buf.ay.append(msg.linear_acceleration.y)
        self._buf.az.append(msg.linear_acceleration.z)
        self._buf.gx.append(msg.angular_velocity.x)
        self._buf.gy.append(msg.angular_velocity.y)
        self._buf.gz.append(msg.angular_velocity.z)
        self._buf.qw.append(msg.orientation.w)
        self._buf.qx.append(msg.orientation.x)
        self._buf.qy.append(msg.orientation.y)
        self._buf.qz.append(msg.orientation.z)

    def wait_first_message(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest is not None:
                return True
        return False

    def collect(self, seconds: float) -> _Buffer:
        self._buf = _Buffer()
        self._collecting = True
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._collecting = False
        return self._buf


def _signed_peak(values: list[float]) -> float:
    """Value with the largest absolute magnitude (signed)."""
    if not values:
        return 0.0
    arr = np.asarray(values, dtype=float)
    idx = int(np.argmax(np.abs(arr)))
    return float(arr[idx])


def _integrate_gz(gz: list[float], duration: float) -> float:
    if not gz:
        return 0.0
    dt = duration / len(gz)
    return float(np.sum(np.asarray(gz, dtype=float)) * dt)


def _quat_to_pitch(qw: float, qx: float, qy: float, qz: float) -> float:
    """Pitch (rotation about y) from a quaternion. Returns radians."""
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


def _check_motion(
    name: str,
    instruction: str,
    expected: str,
    pass_predicate,
    diagnostic_fn,
    node: AxisSanityNode,
) -> tuple[bool, str]:
    """Prompt -> collect -> evaluate. Returns (passed, message)."""
    while True:
        print()
        print(f"  動作:{instruction}")
        print(f"  預期:{expected}")
        if not prompts.confirm("  準備好做這個動作了嗎?", default=True):
            return False, f"{name}: 使用者取消"
        prompts.countdown(2, "  資料收集即將開始,請於倒數結束後執行動作。")
        print(f"  收集 {COLLECT_SECONDS:.0f} 秒資料中...")
        buf = node.collect(COLLECT_SECONDS)
        if not buf.ax:
            print("  ✗ 沒有收到 IMU 訊息,請檢查 driver,重做。")
            continue
        too_small, magnitude = diagnostic_fn(buf)
        if too_small:
            print(f"  ✗ 動作幅度太小(峰值 {magnitude:.3f}),請更用力或更慢,重做。")
            if not prompts.confirm("  再試一次?", default=True):
                return False, f"{name}: 動作幅度不足"
            continue
        ok, message = pass_predicate(buf)
        if ok:
            print(f"  ✓ 通過:{message}")
            return True, f"{name}: PASS - {message}"
        print(f"  ✗ 失敗:{message}")
        if not prompts.confirm("  再試一次?", default=True):
            return False, f"{name}: FAIL - {message}"


def _diag_accel_x(buf: _Buffer) -> tuple[bool, float]:
    peak = _signed_peak(buf.ax)
    return abs(peak) < ACCEL_PEAK_THRESHOLD, peak


def _diag_gyro_z(buf: _Buffer) -> tuple[bool, float]:
    peak = _signed_peak(buf.gz)
    return abs(peak) < GYRO_PEAK_THRESHOLD, peak


def _diag_pitch(buf: _Buffer) -> tuple[bool, float]:
    if not buf.qw:
        return True, 0.0
    pitch_end = _quat_to_pitch(buf.qw[-1], buf.qx[-1], buf.qy[-1], buf.qz[-1])
    pitch_start = _quat_to_pitch(buf.qw[0], buf.qx[0], buf.qy[0], buf.qz[0])
    delta = pitch_end - pitch_start
    return abs(delta) < math.radians(10.0), delta


def _check_forward(buf: _Buffer) -> tuple[bool, str]:
    peak = _signed_peak(buf.ax)
    if peak > 0:
        return True, f"ax 峰值 = {peak:+.3f} m/s² (正,符合 x 軸朝前)"
    return False, (
        f"ax 峰值 = {peak:+.3f} m/s²,預期正值。"
        " x 軸可能朝後;檢查 IMU 安裝方向或 URDF 中 imu_link 的旋轉。"
    )


def _check_backward(buf: _Buffer) -> tuple[bool, str]:
    peak = _signed_peak(buf.ax)
    if peak < 0:
        return True, f"ax 峰值 = {peak:+.3f} m/s² (負,符合 x 軸朝前)"
    return False, (
        f"ax 峰值 = {peak:+.3f} m/s²,預期負值。"
        " x 軸方向有問題,跟『向前推』結果合起來看一下。"
    )


def _check_left_turn(buf: _Buffer) -> tuple[bool, str]:
    peak = _signed_peak(buf.gz)
    integ = _integrate_gz(buf.gz, COLLECT_SECONDS)
    if peak > 0 and integ > 0:
        return True, (
            f"gz 峰值 = {peak:+.3f} rad/s, 積分 = {integ:+.3f} rad"
            " (正,符合 ROS 左轉為正)"
        )
    return False, (
        f"gz 峰值 = {peak:+.3f} rad/s, 積分 = {integ:+.3f} rad,預期正值。"
        " z 軸可能朝下,或 driver yaw 慣例反了。"
    )


def _check_pitch_up(buf: _Buffer) -> tuple[bool, str]:
    if not buf.qw:
        return False, "沒有 orientation 資料,無法檢查 pitch。"
    pitch_end = _quat_to_pitch(buf.qw[-1], buf.qx[-1], buf.qy[-1], buf.qz[-1])
    pitch_start = _quat_to_pitch(buf.qw[0], buf.qx[0], buf.qy[0], buf.qz[0])
    delta = pitch_end - pitch_start
    if delta > 0:
        return True, f"pitch 變化 = {math.degrees(delta):+.1f}° (正,符合車前端抬高為正)"
    return False, (
        f"pitch 變化 = {math.degrees(delta):+.1f}°,預期正值。"
        " y 軸方向或 quaternion 慣例可能反了。"
    )


def main() -> None:
    rclpy.init()
    node = AxisSanityNode()
    notes_lines: list[str] = []
    all_passed = True

    try:
        prompts.header("IMU 軸向 Sanity Check")
        print("  目的:確認 IMU 三軸方向跟 base_link 對齊。")
        print("  ⚠️  IMU 軸向裝反是 silent killer,先抓出來避免後面所有校正全錯。")
        print(f"  IMU topic = {IMU_TOPIC} (frame_id 預期 = {IMU_FRAME})")
        print(f"  Base frame = {BASE_FRAME}")
        print()
        prompts.wait_enter("按 ENTER 開始 (Ctrl+C 隨時取消)...")

        print()
        print(f"  等待第一筆 IMU 訊息(timeout {FIRST_MSG_TIMEOUT:.0f} 秒)...")
        if not node.wait_first_message(FIRST_MSG_TIMEOUT):
            print("  ✗ Timeout!收不到 IMU 資料。")
            print(f"  檢查:imu node 是否已啟動?topic 名稱是否為 {IMU_TOPIC}?")
            return
        print("  ✓ 收到 IMU 資料。")

        motions = [
            (
                "forward",
                "用手把車向前推約 30 公分後停下",
                "ax 出現正向峰值",
                _check_forward,
                _diag_accel_x,
            ),
            (
                "backward",
                "用手把車向後推約 30 公分後停下",
                "ax 出現負向峰值",
                _check_backward,
                _diag_accel_x,
            ),
            (
                "left_turn",
                "把車原地向左(逆時針)轉約 45 度",
                "gz 為正,積分 ≈ +0.78 rad",
                _check_left_turn,
                _diag_gyro_z,
            ),
            (
                "pitch_up",
                "抬高車前端約 30 度,維持住直到收集結束",
                "pitch 變化為正",
                _check_pitch_up,
                _diag_pitch,
            ),
        ]

        for name, instruction, expected, pred, diag in motions:
            ok, msg = _check_motion(name, instruction, expected, pred, diag, node)
            notes_lines.append(msg)
            if not ok:
                all_passed = False

        prompts.header("結果")
        for line in notes_lines:
            print(f"  {line}")
        print()
        notes = " | ".join(notes_lines)

        if all_passed:
            print("  ✓ 全部通過,寫入 YAML。")
            path = yaml_store.update_section(
                "imu",
                {"axis_sanity_passed": True, "axis_sanity_notes": notes},
            )
            print(f"  YAML 已更新:{path}")
        else:
            print("  ✗ 有項目失敗,不寫 axis_sanity_passed=true。")
            print("  建議:")
            print("    - 確認 IMU 安裝方向(箭頭指向車前)")
            print("    - 在 URDF 中加入 imu_link 的 RPY 旋轉")
            print("    - 或調整 driver 的 frame_id / orientation 慣例")
            try:
                yaml_store.update_section(
                    "imu",
                    {"axis_sanity_passed": False, "axis_sanity_notes": notes},
                )
            except Exception as exc:  # noqa: BLE001
                print(f"  (寫入 notes 失敗:{exc})")
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
