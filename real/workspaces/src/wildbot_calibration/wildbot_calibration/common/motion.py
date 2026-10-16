"""cmd_vel publisher with safety brake on exit/signals.

Each calibration script imports MotionPublisher with the topic, message type,
and frame_id its file-header constants chose. drive_for() runs a timed loop
publishing the requested twist, then publishes zero >=10x. atexit + SIGINT/SIGTERM
hooks publish zero too, so a Ctrl+C or unhandled exception still stops the robot.
"""

from __future__ import annotations

import atexit
import signal
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped


_TWIST_MSG_TYPES = {
    "geometry_msgs/Twist": Twist,
    "geometry_msgs/msg/Twist": Twist,
    "geometry_msgs/TwistStamped": TwistStamped,
    "geometry_msgs/msg/TwistStamped": TwistStamped,
}


def _resolve_msg_type(name: str) -> Any:
    try:
        return _TWIST_MSG_TYPES[name]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported cmd_vel message type {name!r}. "
            "Supported: geometry_msgs/Twist, geometry_msgs/TwistStamped."
        ) from exc


class MotionPublisher:
    """Publish cmd_vel safely. Always emit zero on shutdown."""

    def __init__(
        self,
        node: Node,
        topic: str,
        msg_type_name: str,
        frame_id: str = "base_link",
    ) -> None:
        self._node = node
        self._frame_id = frame_id
        self._msg_cls = _resolve_msg_type(msg_type_name)
        self._is_stamped = self._msg_cls is TwistStamped

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = node.create_publisher(self._msg_cls, topic, qos)
        self._stopped = False

        atexit.register(self._safe_brake)
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, self._signal_brake)
            except (ValueError, OSError):
                # Not in main thread, ignore.
                pass

    def _make_msg(self, linear: float, angular: float) -> Any:
        if self._is_stamped:
            msg = TwistStamped()
            msg.header.frame_id = self._frame_id
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.twist.linear.x = float(linear)
            msg.twist.angular.z = float(angular)
            return msg
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        return msg

    def publish(self, linear: float, angular: float) -> None:
        self._pub.publish(self._make_msg(linear, angular))

    def stop(self, repeats: int = 10, gap: float = 0.02) -> None:
        for _ in range(max(1, repeats)):
            try:
                self.publish(0.0, 0.0)
            except Exception:
                break
            time.sleep(gap)

    def drive_for(
        self,
        linear: float,
        angular: float,
        duration: float,
        rate_hz: float = 50.0,
    ) -> None:
        if duration <= 0:
            return
        period = 1.0 / max(1.0, rate_hz)
        t_end = time.monotonic() + duration
        try:
            while time.monotonic() < t_end:
                self.publish(linear, angular)
                # Spin a little so subscribers/callbacks in the same node still pump.
                rclpy.spin_once(self._node, timeout_sec=0.0)
                time.sleep(period)
        finally:
            self.stop()

    # --- safety hooks -------------------------------------------------

    def _safe_brake(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        try:
            self.stop()
        except Exception:
            pass

    def _signal_brake(self, signum, frame) -> None:  # noqa: D401, ANN001
        self._safe_brake()
        # Re-raise default behavior: KeyboardInterrupt for SIGINT.
        if signum == signal.SIGINT:
            raise KeyboardInterrupt
        raise SystemExit(128 + signum)
