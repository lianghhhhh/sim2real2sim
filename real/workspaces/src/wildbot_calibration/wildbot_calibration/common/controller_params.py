"""Read the parameters the running diff_drive_controller is actually using.

Calibration ratios (measured / commanded) only mean something relative to the
wheel_radius / wheel_separation_multiplier the controller had loaded while the
robot drove, so read them live instead of trusting a hard-coded nominal that
drifts out of sync with controllers.yaml.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters


CONTROLLER_NODE = "/base_controller"
SERVICE_TIMEOUT = 5.0


def _value(pv) -> Optional[float]:
    if pv.type == ParameterType.PARAMETER_DOUBLE:
        return float(pv.double_value)
    if pv.type == ParameterType.PARAMETER_INTEGER:
        return float(pv.integer_value)
    return None


def get_controller_params(
    node: Node,
    names: list[str],
    controller: str = CONTROLLER_NODE,
    timeout: float = SERVICE_TIMEOUT,
) -> Optional[dict[str, float]]:
    """Return {name: value} from the controller, or None if any is unavailable."""
    client = node.create_client(GetParameters, f"{controller}/get_parameters")
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            print(f"  ✗ 找不到 {controller}/get_parameters service(kros_car 有啟動嗎?ROS_DOMAIN_ID 一致嗎?)")
            return None
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        result = future.result()
        if result is None:
            print(f"  ✗ 讀取 {controller} 參數 timeout。")
            return None
        if len(result.values) != len(names):
            print(f"  ✗ {controller} 沒有參數 {names}(回傳 {len(result.values)} 個值)。")
            return None
        params: dict[str, float] = {}
        for name, pv in zip(names, result.values):
            value = _value(pv)
            if value is None:
                print(f"  ✗ {controller} 沒有數值參數 {name!r}。")
                return None
            params[name] = value
        return params
    finally:
        node.destroy_client(client)
