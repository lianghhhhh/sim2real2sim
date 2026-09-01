import sys
import time
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from calibrate_env_pkg.collect_data_node import CollectDataNode
from calibrate_env_pkg.control_car_node import ControlCarNode


def main():
    # 用 sys.argv 初始化，才能吃到 `ros2 run ... --ros-args -p key:=value` 這種參數覆蓋
    rclpy.init(args=sys.argv)

    collect_data_node = CollectDataNode()
    control_car_node = ControlCarNode()

    executor = MultiThreadedExecutor()
    executor.add_node(collect_data_node)
    executor.add_node(control_car_node)

    # 把 executor.spin() 丟到背景執行緒跑，維持原本 MultiThreadedExecutor
    # 該有的全速回呼處理（20Hz 的 timer、各種 subscription）。
    # 主執行緒則只負責輪詢 control_car_node.finished 旗標，
    # 一旦測試腳本跑完，就主動觸發收尾流程並讓整個 process 結束，
    # 這樣 run_car.sh 裡的 `ros2 run ...` 才會返回，接著才能關閉 Isaac Sim。
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok() and not control_car_node.finished:
            time.sleep(0.1)

        collect_data_node.get_logger().info(
            f"測試腳本執行完畢，共記錄 {collect_data_node.count} 筆資料，準備關閉節點..."
        )
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down...")
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        collect_data_node.destroy_node()
        control_car_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()