"""學到的動態模型 + acados MPC 循跡 (只給 Isaac 模擬用, 吃 /odom 的 GT 位姿)。

    MPC_CONFIG_PATH=/workspaces/model/mpc_config.json \\
        ros2 run mlp_control_pkg mlp_control_node --ros-args -p use_sim_time:=true

use_sim_time 一定要開: 模型的時間步是 0.05 s 模擬時間, GUI 只有 ~0.55 倍即時。
"""
import signal

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from mlp_control_pkg.car_control_node import CarControlNode
from mlp_control_pkg.car_state_node import CarStateNode
from mlp_control_pkg.path_points_node import PathPointsNode
from mlp_control_pkg.utils import loadConfig


def _raise_interrupt(signum, frame):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt


def main():
    # 自己處理 Ctrl-C: rclpy 預設的 handler 會先關掉 context, 結束時的輪速 0 就送不出去
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)

    config = loadConfig()
    car_state_node = CarStateNode()
    path_points_node = PathPointsNode()
    car_control_node = CarControlNode(car_state_node, path_points_node, config)

    executor = MultiThreadedExecutor()
    for n in (car_state_node, path_points_node, car_control_node):
        executor.add_node(n)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        car_control_node.stop_car()
        car_control_node.report_tracking_error()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
