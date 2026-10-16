"""Wheel angular (separation multiplier) calibration via IMU yaw integration."""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

from wildbot_calibration.common import prompts
from wildbot_calibration.common import yaml_store
from wildbot_calibration.common.motion import MotionPublisher


# ============================================================
# Topics / frames discovered from user's existing packages
#   Source: kros_car at ../kros_car/
#     - cmd_vel: /base_controller/cmd_vel  (geometry_msgs/TwistStamped)
#                config/controllers.yaml:8 + README.md:45
#     - base_frame: config/controllers.yaml:27 "base_link"
#     - wheel_separation: 0.274 m  (config/controllers.yaml:19)
#     - wheel_separation_multiplier: 2.21  (config/controllers.yaml:20)
#       Effective separation = 0.274 * 2.21 = 0.60554 m.
#       Per user direction (2026-05-15), we calibrate the *multiplier* and leave
#       the base wheel_separation alone. So WHEEL_SEPARATION_MULTIPLIER_NOMINAL
#       is the value to scale; the corrected value goes back into the same line.
#   Source: wildbot_imu at ../wildbot_imu/
#     - IMU topic: /imu/data  (src/wildbot_imu_node.cpp:70)
# ============================================================
CMD_VEL_TOPIC = "/base_controller/cmd_vel"
CMD_VEL_MSG_TYPE = "geometry_msgs/TwistStamped"
IMU_TOPIC = "/imu/data"
BASE_FRAME = "base_link"
WHEEL_SEPARATION_MULTIPLIER_NOMINAL = 2.21
NUM_REVOLUTIONS = 5

DRIVE_ANGULAR = 0.5              # rad/s
DRIVE_DURATION = NUM_REVOLUTIONS * (2.0 * math.pi) / DRIVE_ANGULAR
TARGET_DEGREES = NUM_REVOLUTIONS * 360.0
IMU_TIMEOUT = 5.0


class _YawIntegrator(Node):
    """Subscribe to IMU; integrate signed yaw delta from quaternion."""

    def __init__(self) -> None:
        super().__init__("wheel_angular_calibration")
        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(Imu, IMU_TOPIC, self._on_imu, qos)
        self._latest: Optional[Imu] = None
        self._tracking = False
        self._prev_yaw: Optional[float] = None
        self._accum_yaw: float = 0.0

    @staticmethod
    def _yaw_from_quat(qw: float, qx: float, qy: float, qz: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _on_imu(self, msg: Imu) -> None:
        self._latest = msg
        if not self._tracking:
            return
        q = msg.orientation
        yaw = self._yaw_from_quat(q.w, q.x, q.y, q.z)
        if self._prev_yaw is None:
            self._prev_yaw = yaw
            return
        # Wrap-aware delta: shortest signed rotation in (-pi, pi].
        delta = yaw - self._prev_yaw
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta <= -math.pi:
            delta += 2.0 * math.pi
        self._accum_yaw += delta
        self._prev_yaw = yaw

    def wait_first(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest is not None:
                return True
        return False

    def start_tracking(self) -> None:
        self._prev_yaw = None
        self._accum_yaw = 0.0
        self._tracking = True

    def stop_tracking(self) -> float:
        self._tracking = False
        return self._accum_yaw


def main() -> None:
    rclpy.init()
    node = _YawIntegrator()
    motion = MotionPublisher(node, CMD_VEL_TOPIC, CMD_VEL_MSG_TYPE, BASE_FRAME)
    try:
        prompts.header("輪距(角度)校正")
        print(f"  將以 {DRIVE_ANGULAR:.2f} rad/s 原地左轉 {NUM_REVOLUTIONS} 圈,約 {DRIVE_DURATION:.1f} 秒。")
        print("  使用 IMU yaw 累積值自動量轉了幾度,不需要人眼判斷。")
        if NUM_REVOLUTIONS < 5:
            print("  ⚠️  圈數 < 5,精度會明顯下降(IMU bias 影響相對變大)。")
        print(f"  cmd_vel = {CMD_VEL_TOPIC}, imu = {IMU_TOPIC}")
        print(f"  wheel_separation_multiplier (nominal) = {WHEEL_SEPARATION_MULTIPLIER_NOMINAL}")
        print()

        existing_imu = yaml_store.get_section("imu")
        if not existing_imu.get("axis_sanity_passed", False):
            print("  ⚠️  imu.axis_sanity_passed 為 false,IMU yaw 量到的方向不一定正確。")
        gyro_bias = existing_imu.get("gyro_bias", [0.0, 0.0, 0.0])
        if all(abs(float(b)) < 1e-12 for b in gyro_bias):
            print("  ⚠️  imu.gyro_bias 全為 0,可能還沒跑 imu_static_calibration。")
        if (not existing_imu.get("axis_sanity_passed", False)
                or all(abs(float(b)) < 1e-12 for b in gyro_bias)):
            if not prompts.confirm("  仍要繼續?", default=False):
                print("  已取消。")
                return

        if not prompts.confirm("  準備好開始?", default=False):
            print("  已取消。")
            return
        if not prompts.confirm(f"  原地旋轉空間夠嗎(車身周圍至少留 1 公尺)?", default=False):
            print("  已取消。")
            return

        print()
        print(f"  等待 IMU 訊息(timeout {IMU_TIMEOUT:.0f} 秒)...")
        if not node.wait_first(IMU_TIMEOUT):
            print("  ✗ Timeout,收不到 IMU 資料。")
            return

        prompts.countdown(3, "  即將開始旋轉...")
        node.start_tracking()
        motion.drive_for(linear=0.0, angular=DRIVE_ANGULAR, duration=DRIVE_DURATION)

        # Drain IMU for a short tail.
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        accum_rad = node.stop_tracking()
        measured_degrees = abs(math.degrees(accum_rad))
        target_degrees = TARGET_DEGREES
        error_degrees = measured_degrees - target_degrees

        if measured_degrees < 1.0:
            print("  ✗ IMU 累積 yaw 太小,可能根本沒在轉(driver 沒收到 cmd_vel?軸向錯?)")
            return

        angular_scale = target_degrees / measured_degrees
        multiplier_corrected = WHEEL_SEPARATION_MULTIPLIER_NOMINAL * angular_scale
        change_pct = (
            (multiplier_corrected - WHEEL_SEPARATION_MULTIPLIER_NOMINAL)
            / WHEEL_SEPARATION_MULTIPLIER_NOMINAL
            * 100.0
        )

        prompts.header("Summary")
        print(f"  目標旋轉 = {target_degrees:.1f}° ({NUM_REVOLUTIONS} 圈)")
        print(f"  IMU 實測 = {measured_degrees:.2f}°  (誤差 {error_degrees:+.2f}°)")
        print(f"  angular_scale = target / measured = {angular_scale:.6f}")
        print(f"  wheel_separation_multiplier:")
        print(f"    {WHEEL_SEPARATION_MULTIPLIER_NOMINAL:.6f}  ->  {multiplier_corrected:.6f}")
        print(f"    (變化 {change_pct:+.2f}%)")
        print()
        if not prompts.confirm("  寫入 YAML?", default=True):
            print("  未寫入。")
            return

        path = yaml_store.update_section(
            "base",
            {
                "angular_scale": float(angular_scale),
                "wheel_separation_multiplier_nominal": float(WHEEL_SEPARATION_MULTIPLIER_NOMINAL),
                "wheel_separation_multiplier_corrected": float(multiplier_corrected),
            },
        )
        print(f"  YAML 已更新:{path}")
        print()
        print("  📝 提示:請手動將 controllers.yaml 中的")
        print("     base_controller.wheel_separation_multiplier 改成:")
        print(f"            {WHEEL_SEPARATION_MULTIPLIER_NOMINAL}  ->  {multiplier_corrected:.6f}")
        print("     (路徑:kros_car/config/controllers.yaml,line 20)")
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
