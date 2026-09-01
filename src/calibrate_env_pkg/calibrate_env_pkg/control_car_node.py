import itertools
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# ─────────────────────────────────────────────────────────────
# 基礎設定
# ─────────────────────────────────────────────────────────────
MAX_EFFORT = 10.0
_SCALE = MAX_EFFORT / 5.0

# 矩陣掃描的參數範圍（可依需求調整）
THROTTLE_LEVELS = [1.0, 2.0, 3.0, 4.0, 5.0]      # 由小到大，涵蓋靜摩擦~動摩擦區間
STEER_LEVELS    = [-8.0, -4.0, 0.0, 4.0, 8.0]     # 涵蓋直行到接近側滑邊界
HOLD_DURATION   = 3.0   # 每個 throttle/steer 組合持續時間，確保進入穩態
REST_DURATION   = 1.0   # 每組之間插入短暫 Stop，重設輪速/滑動狀態
REPEATS         = 3     # 同一情境重複次數，用來平均掉隨機/數值誤差

# 爬行測試（估計靜摩擦臨界值）
CREEP_START, CREEP_END, CREEP_DURATION = 0.0, 5.0, 10.0

# 急煞測試（估計動摩擦/滾動阻力：先加速到穩態，再瞬間斷油觀察自然減速）
BRAKE_PRE_THROTTLE, BRAKE_PRE_DURATION = 5.0, 4.0
BRAKE_COAST_DURATION = 4.0


def _build_scenarios():
    """
    產生完整測試腳本：
      1. Creep test（緩慢線性升高油門，估計靜摩擦臨界點）
      2. Emergency-brake test（穩態後瞬間斷油，觀察自然滑行減速）
      3. Throttle x Steer 矩陣掃描，每組重複 REPEATS 次，組間插入 Stop
    每個 scenario 用 "type" 區分行為：
      - "const"   : 固定 throttle/steer，維持 duration 秒（走 rate limiter，模擬漸進控制）
      - "ramp"    : throttle 從 start 線性升到 end，steer 固定
      - "instant" : 該筆立刻跳到目標值，略過 rate limiter（用於模擬瞬間放開油門）
    """
    scenarios = []

    # 1) Creep test
    scenarios.append({
        "name": "Creep (ramp throttle)",
        "type": "ramp",
        "throttle_start": CREEP_START,
        "throttle_end": CREEP_END,
        "steer": 0.0,
        "duration": CREEP_DURATION,
    })
    scenarios.append({"name": "Stop", "type": "const", "throttle": 0.0, "steer": 0.0, "duration": REST_DURATION})

    # 2) Emergency-brake test
    scenarios.append({
        "name": "Pre-brake accel",
        "type": "const",
        "throttle": BRAKE_PRE_THROTTLE,
        "steer": 0.0,
        "duration": BRAKE_PRE_DURATION,
    })
    scenarios.append({
        "name": "Emergency brake (instant cutoff)",
        "type": "instant",
        "throttle": 0.0,
        "steer": 0.0,
        "duration": BRAKE_COAST_DURATION,
    })
    scenarios.append({"name": "Stop", "type": "const", "throttle": 0.0, "steer": 0.0, "duration": REST_DURATION})

    # 3) Throttle x Steer 矩陣掃描（含重複試驗）
    combos = list(itertools.product(THROTTLE_LEVELS, STEER_LEVELS))
    for rep in range(REPEATS):
        for throttle, steer in combos:
            scenarios.append({
                "name": f"Matrix T={throttle} S={steer} (rep{rep+1})",
                "type": "const",
                "throttle": throttle,
                "steer": steer,
                "duration": HOLD_DURATION,
            })
            scenarios.append({"name": "Stop", "type": "const", "throttle": 0.0, "steer": 0.0, "duration": REST_DURATION})

    return scenarios


class ControlCarNode(Node):
    def __init__(self):
        super().__init__('control_car_node')
        self.get_logger().info("車輛控制節點已啟動，開始執行矩陣式測試腳本...")

        # 建立 Publisher，發佈至 /joint_command
        self.effort_pub = self.create_publisher(JointState, '/joint_command', 10)
        # 額外廣播目前情境名稱，方便 collect_data_node 把每一列資料標上情境標籤
        self.scenario_pub = self.create_publisher(String, '/test_scenario', 10)

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # 初始四輪 effort 狀態 (FL, FR, RL, RR)
        self.current_efforts = [0.0, 0.0, 0.0, 0.0]

        # 供 main.py 偵測「所有測試情境已跑完」用的旗標。
        # 注意：不要在這個 node 內部直接呼叫 rclpy.shutdown()，
        # 因為它是被 MultiThreadedExecutor 一起管理的兩個 node 之一，
        # 由單一 node 打斷共用 context 容易造成 race condition。
        # 正確做法是設旗標，讓 main.py 的主迴圈偵測後統一收尾。
        self.finished = False

        # ─────────────────────────────────────────────────────────────
        # 產生完整測試情境清單：creep + emergency brake + throttle x steer 矩陣 x repeats
        # ─────────────────────────────────────────────────────────────
        self.scenarios = _build_scenarios()
        self.get_logger().info(f"共產生 {len(self.scenarios)} 個測試情境。")

        self.scenario_idx = 0
        self.scenario_start_time = self.start_time

        # 設定執行頻率為 20Hz (每 0.05 秒執行一次)
        self.hz  = 20.0
        self.dt  = 1.0 / self.hz
        self.timer = self.create_timer(self.dt, self.control_callback)

    # ═══════════════════════════════════════════════════════════
    #  MAIN CONTROL LOOP
    # ═══════════════════════════════════════════════════════════
    def control_callback(self):

        # # test
        # name_msg = String()
        # name_msg.data = 'test'
        # self.scenario_pub.publish(name_msg)

        # msg = JointState()
        # msg.name   = ['front_left_joint', 'front_right_joint',
        #                 'rear_left_joint',  'rear_right_joint']
        # effort = [1.0, 1.0, 1.0, 1.0]  # 測試用固定值
        # msg.effort = effort
        # self.effort_pub.publish(msg)
        # return
        # ###

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_in_scenario = current_time - self.scenario_start_time

        # 檢查情境進度
        if self.scenario_idx >= len(self.scenarios):
            target_throttle, target_steer = 0.0, 0.0
            instant = False
            current_scenario_name = "Finished"
            if elapsed_in_scenario > 2.0 and not self.finished:  # 確保最後煞停 2 秒後再標記完成
                self.get_logger().info("所有測試情境已完成，設定 finished 旗標，等待主程式收尾。")
                self.finished = True
            # 持續發佈煞停指令（下方仍會 publish），直到 main.py 偵測到 finished 才會真正結束 process
        else:
            scenario = self.scenarios[self.scenario_idx]
            current_scenario_name = scenario["name"]
            stype = scenario.get("type", "const")
            instant = (stype == "instant")

            if stype == "ramp":
                frac = min(max(elapsed_in_scenario / scenario["duration"], 0.0), 1.0)
                throttle_now = scenario["throttle_start"] + frac * (scenario["throttle_end"] - scenario["throttle_start"])
                target_throttle = throttle_now * _SCALE
                target_steer    = scenario["steer"] * _SCALE
            else:
                target_throttle = scenario["throttle"] * _SCALE
                target_steer    = scenario["steer"] * _SCALE

            if elapsed_in_scenario > scenario["duration"]:
                self.scenario_idx += 1
                self.scenario_start_time = current_time
                next_name = self.scenarios[self.scenario_idx]['name'] if self.scenario_idx < len(self.scenarios) else 'Finished'
                self.get_logger().info(f"切換情境 -> {next_name}")

        # 廣播目前情境名稱，供 collect_data_node 標記資料
        name_msg = String()
        name_msg.data = current_scenario_name
        self.scenario_pub.publish(name_msg)

        # 混控
        target_efforts = self._mix(target_throttle, target_steer)
        human_max_rate = 3.0 * _SCALE  # 可微調此數值來決定踩油門的急促程度

        if instant:
            # 略過 rate limiter，模擬瞬間放開油門（急煞測試用），
            # 讓車輛的減速完全由物理摩擦力決定，而非人為漸進控制
            self.current_efforts = list(target_efforts)
        else:
            for i in range(4):
                self.current_efforts[i] = _rate_limit(
                    target   = target_efforts[i],
                    current  = self.current_efforts[i],
                    max_rate = human_max_rate,
                    dt       = self.dt,
                )

        msg = JointState()
        msg.name   = ['front_left_joint', 'front_right_joint',
                      'rear_left_joint',  'rear_right_joint']
        msg.effort = self.current_efforts
        self.effort_pub.publish(msg)

    @staticmethod
    def _mix(throttle: float, steer: float):
        left  = max(min(throttle - steer, MAX_EFFORT), -MAX_EFFORT)
        right = max(min(throttle + steer, MAX_EFFORT), -MAX_EFFORT)
        return [left, right, left, right]


def _rate_limit(target, current, max_rate, dt):
    max_delta = max_rate * dt
    delta     = target - current
    if   delta >  max_delta: return current + max_delta
    elif delta < -max_delta: return current - max_delta
    else:                    return target


def main(args=None):
    rclpy.init(args=args)
    node = ControlCarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()