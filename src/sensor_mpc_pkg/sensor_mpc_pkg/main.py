"""感測器定位 + 學到的動態模型 + acados MPC 循跡 (模擬與實體車都用這一個)。

    # 模擬 (Isaac 開 kros_car_mpc.usd; 定位、path_tracker、actuator_model 由 launch 開)
    ros2 launch sensor_mpc_pkg sensor_mpc.launch.py

    # 只開 MPC 節點本身 (定位與 path_tracker 另外開)
    MPC_CONFIG_PATH=/workspaces/model/mpc_config.json \\
        ros2 run sensor_mpc_pkg sensor_mpc_node --ros-args -p use_sim_time:=true \\
        -p odom_topic:=/fusion_loc/odom -p joint_states_topic:=/joint_states

模擬一定要 use_sim_time:=true (模型的時間步是 0.05 s 模擬時間); 實體車 use_sim_time:=false。
"""
import signal

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .mpc_node import SensorMpcNode
from .sensor_state import SensorStateNode


def _raise_interrupt(signum, frame):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt


def main():
    # 自己處理 Ctrl-C: rclpy 預設的 handler 會先關掉 context, 結束時的輪速 0 就送不出去
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)

    state_node = SensorStateNode()
    mpc_node = SensorMpcNode(state_node)

    executor = MultiThreadedExecutor()
    executor.add_node(state_node)
    executor.add_node(mpc_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        mpc_node.stop_car()
        mpc_node.report()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
