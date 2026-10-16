"""Wheel linear (radius) calibration: drive 2 m, measure with tape, scale radius."""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

from wildbot_calibration.common import prompts
from wildbot_calibration.common import yaml_store
from wildbot_calibration.common.motion import MotionPublisher


# ============================================================
# Topics / frames discovered from user's existing packages
#   Source: kros_car at ../kros_car/
#     - cmd_vel:  /base_controller/cmd_vel
#                 (config/controllers.yaml:8 controller name "base_controller",
#                  diff_drive_controller; README.md:45 confirms type)
#     - cmd_vel msg type: geometry_msgs/TwistStamped
#                 (kros_car README.md:45 + L165-167 example)
#     - odom:    /base_controller/odom (controller default; README.md:46)
#     - base_frame: config/controllers.yaml:27 "base_link"
#     - wheel_radius: config/controllers.yaml:23  0.05035 m
# ============================================================
CMD_VEL_TOPIC = "/base_controller/cmd_vel"
CMD_VEL_MSG_TYPE = "geometry_msgs/TwistStamped"
ODOM_TOPIC = "/base_controller/odom"
BASE_FRAME = "base_link"
WHEEL_RADIUS_NOMINAL = 0.05035   # meters

DRIVE_LINEAR = 0.1               # m/s
DRIVE_DURATION = 20.0            # s -> theoretical 2.0 m
TARGET_DISTANCE = DRIVE_LINEAR * DRIVE_DURATION
ODOM_TIMEOUT = 5.0


class _OdomTracker(Node):
    def __init__(self) -> None:
        super().__init__("wheel_linear_calibration")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos)
        self._first: Optional[Odometry] = None
        self._latest: Optional[Odometry] = None

    def _on_odom(self, msg: Odometry) -> None:
        if self._first is None:
            self._first = msg
        self._latest = msg

    def wait_first(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._first is not None:
                return True
        return False

    def reset_origin(self) -> None:
        self._first = self._latest

    def distance_traveled(self) -> Optional[float]:
        if self._first is None or self._latest is None:
            return None
        dx = self._latest.pose.pose.position.x - self._first.pose.pose.position.x
        dy = self._latest.pose.pose.position.y - self._first.pose.pose.position.y
        return math.hypot(dx, dy)


def main() -> None:
    rclpy.init()
    node = _OdomTracker()
    motion = MotionPublisher(node, CMD_VEL_TOPIC, CMD_VEL_MSG_TYPE, BASE_FRAME)
    try:
        prompts.header("輪徑(線性)校正")
        print(f"  將以 {DRIVE_LINEAR:.2f} m/s 直線前進 {DRIVE_DURATION:.0f} 秒,理論 {TARGET_DISTANCE:.2f} m。")
        print("  你會用捲尺實測,我用實測 / 理論算出 wheel_radius 校正係數。")
        print(f"  ⚠️  需要至少 {TARGET_DISTANCE + 1.0:.1f} 公尺以上的直線空間。")
        print(f"  cmd_vel topic = {CMD_VEL_TOPIC} ({CMD_VEL_MSG_TYPE})")
        print(f"  wheel_radius (nominal) = {WHEEL_RADIUS_NOMINAL} m")
        print()

        if not prompts.confirm("  準備好開始?", default=False):
            print("  已取消。")
            return
        if DRIVE_DURATION > 5.0:
            if not prompts.confirm("  場地夠長嗎(會跑超過 5 秒)?", default=False):
                print("  已取消。")
                return

        print()
        print(f"  等待 odom 訊息(timeout {ODOM_TIMEOUT:.0f} 秒)...")
        if not node.wait_first(ODOM_TIMEOUT):
            print(f"  ⚠️  收不到 {ODOM_TOPIC} 訊息;校正照跑,但 odom 參考值會缺。")
        node.reset_origin()

        print()
        print("  在地面上做個起點記號,然後按 ENTER 出發...")
        prompts.wait_enter()
        prompts.countdown(3, "  即將出發...")

        motion.drive_for(linear=DRIVE_LINEAR, angular=0.0, duration=DRIVE_DURATION)
        # Settle a moment so odom catches up.
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        odom_dist = node.distance_traveled()
        prompts.header("實測")
        print(f"  理論距離 = {TARGET_DISTANCE:.4f} m")
        if odom_dist is not None:
            print(f"  Odometry 距離(僅供參考) = {odom_dist:.4f} m")
        print()
        measured = prompts.prompt_float("  請輸入捲尺實測距離(公尺)")

        if measured <= 0:
            print("  ✗ 實測距離必須為正。")
            return

        linear_scale = measured / TARGET_DISTANCE
        wheel_radius_corrected = WHEEL_RADIUS_NOMINAL * linear_scale
        change_pct = (wheel_radius_corrected - WHEEL_RADIUS_NOMINAL) / WHEEL_RADIUS_NOMINAL * 100.0

        prompts.header("Summary")
        print(f"  實測 / 理論 = {measured:.4f} / {TARGET_DISTANCE:.4f} = {linear_scale:.6f}")
        print(f"  wheel_radius:  {WHEEL_RADIUS_NOMINAL:.6f}  ->  {wheel_radius_corrected:.6f} m")
        print(f"                  (變化 {change_pct:+.2f}%)")
        print()
        if not prompts.confirm("  寫入 YAML?", default=True):
            print("  未寫入。")
            return

        path = yaml_store.update_section(
            "base",
            {
                "linear_scale": float(linear_scale),
                "wheel_radius_nominal": float(WHEEL_RADIUS_NOMINAL),
                "wheel_radius_corrected": float(wheel_radius_corrected),
            },
        )
        print(f"  YAML 已更新:{path}")
        print()
        print("  📝 提示:請手動將 controllers.yaml 中的 base_controller.wheel_radius 改成:")
        print(f"            {WHEEL_RADIUS_NOMINAL}  ->  {wheel_radius_corrected:.6f}")
        print("     (路徑:kros_car/config/controllers.yaml,line 23)")
    except KeyboardInterrupt:
        print("\n  使用者中斷,已嘗試煞車。")
    finally:
        motion.stop()
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
