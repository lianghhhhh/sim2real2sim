"""nav_mpc_manager: 目標點 -> Nav2 規劃 -> MPC 循跡; 前方有新障礙物就停車、重新規劃、繼續走。

    ros2 launch nav_mpc_pkg nav_mpc.launch.py           # 一起開 (定位 + Nav2 + tracker + MPC + 這個)

流程 (狀態 = /nav_mpc/state):

    IDLE ──(/goal_pose)──> PLANNING ──(有路線)──> DRIVING ──(path_tracker 說走到了)──> ALIGNING ──> ARRIVED
                              ^  │ (規劃失敗 / 新路線前面還是擋住)        │
                              │  v                                       │ (走廊裡有掃描點,
                           WAIT_REPLAN <──────────────────────────────────┘  連續 confirm_scans 幀)
                    (車停穩 -> 走廊偵測停的再等 replan_wait 秒 / costmap 驗證停的不用等 -> 規劃;
                     新路線還穿過剛剛的掃描點 = costmap 還沒畫上去, 隔 stale_retry_wait 秒再試, 最多 stale_retries 次)

* 停車是靠 /nav_mpc/hold (std_msgs/Bool, hold_rate Hz 一直發): 不是 DRIVING 就發 true,
  sensor_mpc_node 收到 true 就送 0。這個節點掛了 -> 訊號斷 -> MPC 的 hold_timeout 也會停車
* 路線: Nav2 planner_server 的 ComputePathToPose action (起點 = 目前的融合位姿, use_start)。
  回來的路徑沿弧長重取樣成 path_spacing 間距, 發到 /nav_mpc/path (latched) 給 path_tracker
* 障礙物: 每一幀 /scan 轉到 map 座標 (位姿 = odom_topic, 雷射外參 = TF base_link -> scan 的 frame),
  去掉自己車身框 (footprint_box) 裡的點, 看「從車子的投影點沿路徑往前 check_distance 公尺、
  兩側各 corridor_half_width」的走廊裡有幾個點。靜態地圖的牆在規劃時已經離路徑 >= robot_radius
  (nav2_params.yaml), 所以走廊裡出現的點就是新的東西 (或定位 / 循跡誤差大到離路徑 > 20 cm)
* 路線驗證: DRIVING 時每收到一張 Nav2 的 costmap (/global_costmap/costmap, 2 Hz) 就檢查剩下的路線
  (車子前方 validate_distance 公尺) 離障礙物格子 (cost 100) 有沒有 < validate_clearance (0.35 m)。
  規劃用的是 robot_radius 0.45 m, 驗證刻意鬆 0.1 m: Hybrid-A* 的路線會貼著膨脹區的邊走, 用同一個門檻的話
  雷射每多看到障礙物一格, 剛規劃好的路線就又不合格, 會一直停車重新規劃 (假世界實測 2026-11-18)。
  雷射一次只看得到障礙物朝向車子的那一面, 繞過去的路線會貼著還沒看到的背面走; 車子走近看到側面 / 背面時
  costmap 會更新, 這裡就停車重新規劃 (跟走廊偵測同一個流程), 不會等到障礙物在車子旁邊才發現
* 終點對準 (ALIGNING, align_heading): path_tracker 在離終點 0.15 m 就算到了, 而 Hybrid-A* 的路線常在
  最後一小段才轉向目標方向, 所以車頭會差很多 (2026-11-18 Isaac: 53°)。到了之後原地旋轉:
  yaw rate 目標 = clip(align_kp x 誤差, ±align_wz_max) (至少 align_wz_min, 不然克服不了靜摩擦),
  換成輪速 u = yaw rate x align_wheel_per_yaw 發到 /nav_mpc/spin, sensor_mpc_node 在 hold 中送 左 -u / 右 +u。
  誤差在 align_tolerance 內連續 align_settle 秒就 ARRIVED; 超過 align_timeout、或雷射看到東西在車子
  align_clearance 以內 (原地轉會掃到) 就放棄 (一樣 ARRIVED, events 記 align_abort)
* 換車頭方向重新規劃 (alt_headings): Hybrid-A* 只往前開、R0.5, 障礙物太近 (停下來時前面不到轉彎的空間) 或
  目標方向被夾住時會一直沒路線。但車子是滑移轉向, 可以原地轉。所以主規劃器「沒有路線」時, 再試
  (a) 起點車頭 +-alt_offsets_deg (先原地轉過去再走; 車子 align_clearance 內有東西就不試) 與
  (b) 目標車頭 +-alt_offsets_deg (開到之後 ALIGNING 轉回原本的方向; 要 align_heading),
  分數 = 路線長 + alt_turn_cost x 起點多轉的角度 + alt_goal_turn_cost x 終點多轉的角度, 挑最小的。(a) 轉好之後從實際位姿正常再規劃一次。
  (a)(b) 都失敗 (例如前面被擋、目標方向又只能從另一邊進去) 才試 (c) 起點 x 目標一起換 (7 x 7 種):
  先原地轉, 轉好直接用那個目標方向規劃。
  一次搜尋最多 alt_budget 秒; 還沒開出去前累積原地轉的角度不超過 alt_max_turn_deg (360°), 超過的起點方向不試。events: alt_path / alt_failed / turned
  全部失敗時存一份 costmap 快照 (events_csv 旁邊的 nav_fail_*.npz; src/nav_mpc_pkg/scripts/show_nav_fail.py 畫圖)
* 起點方向檢查 (start_align_deg): 新路線起點的方向跟車頭差太多 (Grid 不管車頭方向, 可能給一條朝後的路線)
  就先原地轉過去 (ALIGNING, 跟終點對準同一套), 轉好才把那條路線發出去。原地轉會掃到東西就不發, 隔 replan_period
  再規劃。events: start_turn -> turned / turn_abort, start_turn_blocked
* 初始位姿 (/initialpose, Foxglove "Publish pose estimate"): LiDAR 定位自己會訂閱並從那裡局部配準;
  這裡負責停車, 並在融合位姿跟給的差太多時叫 /fusion_localizer/reset (融合會用下一則絕對量測當起點)
"""
from __future__ import annotations

import csv
import math
import os
from datetime import datetime, timedelta, timezone

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, ColorRGBA, Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import (PathProgress, corridor_hits, outside_box, resample_polyline,
                       scan_to_points, to_world, wrap_pi, yaw_of)

ODOM_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

IDLE, PLANNING, DRIVING, WAIT_REPLAN, ALIGNING, ARRIVED = (
    'IDLE', 'PLANNING', 'DRIVING', 'WAIT_REPLAN', 'ALIGNING', 'ARRIVED')
EVENT_COLS = ['t', 'event', 'state', 'x', 'y', 'yaw', 'goal_x', 'goal_y', 'goal_yaw',
              'path_len', 'plan_ms', 'obstacle_along', 'n_hits', 'detail']


def _unique_path(path, stamp):
    """檔案已經存在就在副檔名前加 _<時間> (再撞到就加 _2, _3 ...), 不覆寫舊資料。"""
    if not os.path.exists(path):
        return path
    root, ext = os.path.splitext(path)
    if not root.endswith(stamp):
        root = f'{root}_{stamp}'
    cand, k = root + ext, 2
    while os.path.exists(cand):
        cand, k = f'{root}_{k}{ext}', k + 1
    return cand


def _pose_msg(x, y, yaw, frame, stamp):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = stamp
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z = math.sin(0.5 * yaw)
    ps.pose.orientation.w = math.cos(0.5 * yaw)
    return ps


class NavMpcManager(Node):

    def __init__(self):
        super().__init__('nav_mpc_manager')
        p = self.declare_parameter
        p('map_frame', 'map')
        p('odom_topic', '/fusion_loc/odom')
        p('scan_topic', '/scan')
        p('goal_topic', '/goal_pose')                 # Foxglove "Publish pose" (Pose topic 要設成這個) / RViz 2D Goal Pose
        p('initialpose_topic', '/initialpose')        # Foxglove "Publish pose estimate"; '' = 不管
        p('fusion_reset_service', '/fusion_localizer/reset')   # '' = 不叫
        p('initialpose_reset_dist', 0.3)              # m, 融合位姿跟給的差超過這個 (或角度) 才重設融合
        p('initialpose_reset_yaw_deg', 15.0)
        p('path_topic', '/nav_mpc/path')              # -> path_tracker 的 path_topic
        p('hold_topic', '/nav_mpc/hold')              # -> sensor_mpc_node 的 hold_topic
        p('finished_topic', '/sensor_mpc/finished')   # <- path_tracker
        p('state_topic', '/nav_mpc/state')
        p('marker_topic', '/nav_mpc/markers')
        p('planner_action', '/compute_path_to_pose')
        p('planner_id', 'SmacHybrid')                 # nav2_params.yaml 的 planner_plugins 之一
        p('fallback_planner_id', '')                  # 主規劃器失敗時試這個 ('' = 不試)
        p('planner_timeout', 10.0)                    # s
        p('path_spacing', 0.1)                        # m, 重取樣間距 (跟 path_tracker 的 spacing 一樣)
        p('hold_rate', 20.0)                          # Hz
        # s, 新路線發出後過這麼久才放行 (path_tracker 先換好)。path_tracker 50 Hz、收到路線就換,
        # 0.1 s = 5 個週期。2026-11-20 從 0.3 降 (11-18/19 Isaac 15 次停車, 這段平均佔 0.37 s)
        p('release_delay', 0.1)
        p('pose_timeout', 0.5)                        # s
        # ---- 障礙物 ----
        p('obstacle_check', True)
        p('check_distance', 1.5)                      # m, 沿路徑往前看多遠 (從車子中心算)
        p('corridor_half_width', 0.25)                # m, 走廊半寬 (車寬一半 + 餘量)
        p('min_points', 3)                            # 走廊裡至少幾個點才算有東西 (濾雜訊)
        p('confirm_scans', 2)                         # 連續幾幀都有才停
        p('footprint_box', [-0.30, 0.30, -0.22, 0.22])  # 車體座標 [x0, x1, y0, y1], 框裡的點是自己
        p('range_min', 0.05)
        p('range_max', 8.0)
        p('laser_offset', [0.045, 0.0, 0.0])          # 查不到 TF 時用: 雷射在 base_link 的 x, y, yaw(度)
        p('base_frame', 'base_link')
        p('scan_timeout', 0.5)                        # s, DRIVING 時掃描斷這麼久就停 (看不到路)
        # ---- 用 Nav2 的 costmap 驗證剩下的路線 ----
        p('validate_path', True)
        p('costmap_topic', '/global_costmap/costmap')
        p('validate_distance', 4.0)                   # m, 車子前方這麼長的路線要驗證
        p('validate_clearance', 0.35)                 # m, 路線離障礙物格子 (cost 100) 小於這個才算擋住
        # m, nav2_params.yaml 的 global_costmap robot_radius (launch 自動填)。只拿來寫提示訊息與快照, 不影響規劃
        p('robot_radius', 0.45)
        # ---- 重新規劃 ----
        # s, 走廊偵測停車: 車停穩後再等這麼久才規劃 (讓 costmap 把障礙物畫上去)。planner_server 的 costmap
        # 5 Hz 更新, 煞車本身就要 ~0.46 s (Isaac 實測), 停穩時通常已經更新過 2 次; 沒趕上的話新路線會穿過
        # 剛剛的掃描點, 由下面的 stale_retry 補救。2026-11-20 從 1.0 降 (那 1 s 佔每次停車 2.04 s 的一半)
        p('replan_wait', 0.2)
        # s, costmap 驗證停車 (path_invalid): costmap 本來就已經畫上去了才會觸發, 停穩就規劃
        p('replan_wait_invalid', 0.0)
        # 新路線前方走廊還有掃描點 (= costmap 還沒跟上): 隔 stale_retry_wait 秒再試, 最多 stale_retries 次,
        # 用完才當成一般的規劃失敗 (隔 replan_period 再試)。不算進 max_replans
        p('stale_retry_wait', 0.3)
        p('stale_retries', 3)
        p('stop_speed', 0.03)                         # m/s, |v| 低於這個算停穩
        p('stop_wait_max', 3.0)                       # s, 等停穩最多等這麼久
        p('replan_period', 2.0)                       # s, 規劃失敗後隔多久再試
        p('max_replans', 0)                           # 一個目標最多重新規劃幾次; 0 = 不限
        # ---- 換車頭方向重新規劃 (主規劃器沒有路線時) ----
        p('alt_headings', True)                       # 要 align_heading (原地轉靠它)
        p('alt_offsets_deg', [45.0, 90.0, 135.0, 180.0])   # 每個都試 +-, 180 只試一次
        p('alt_turn_cost', 0.5)                       # m / rad, 起點多轉: 先原地轉 (轉 0.6 rad/s vs 開 ~0.3 m/s)
        # m / rad, 終點多轉 (目標換方向, 到了 ALIGNING 轉回來)。比起點便宜: 終點本來就要對準、原地轉 90° ~1 s。
        # 2026-11-20 Isaac: 牆角目標 (離兩面牆 0.7 m、朝東) 用同一個 0.5, 每次都選「目標 -45°」, 那個方向要從很窄的
        # 斜角開進去, 一路碰到箱子就越繞越遠 (103 s / 20.5 m, 直線 5.4 m)
        p('alt_goal_turn_cost', 0.2)
        p('alt_budget', 5.0)                          # s, 一次搜尋最多花這麼久 (失敗的規劃一次 ~0.1 s)
        # 還沒開出去 (接受新路線) 之前, 累積原地轉的角度上限 (度)。超過的起點方向 (含起點 x 目標一起換) 就不試。
        # 2026-11-20 以前是「最多轉 2 次」: Isaac 轉了 +45° 再 +90° 就不准轉, 也連帶關掉一起換, 卡了 13.7 s
        p('alt_max_turn_deg', 360.0)
        p('alt_combos', True)                         # 單邊換都失敗時, 試起點 x 目標一起換
        # 換方向也失敗時存 costmap 快照 (npz) 的資料夾; '' = events_csv 的資料夾 (沒有 events_csv 就不存)
        p('fail_snapshot_dir', '')
        # s, 換起點方向那種原地轉的穩定時間 (轉好就從實際位姿重新規劃, 不用像終點對準那麼準)
        p('turn_settle', 0.2)
        # ---- 起點在規劃器的禁區邊緣 ----
        # SmacHybrid 的 downsample_costmap: 2x2 格取最大 (0.025 -> 0.05 m), 車子停在膨脹區外 1~2 cm 時起點那一格
        # 會變成禁區 -> "Starting point in lethal space", 換什麼方向都沒用 (2026-11-20 Isaac: 箱子在車子側面 0.47 m)。
        # 這時改從 start_shift_max 內最近的合法點規劃 (路線起點偏幾公分, MPC 吃得下)。
        p('planner_downsample', 2)                    # 要跟 nav2_params.yaml 的 SmacHybrid downsampling_factor 一樣 (launch 自動填)
        p('start_shift_max', 0.15)                    # m; 0 = 不移
        # ---- 終點對準車頭 (原地旋轉) ----
        p('align_heading', True)
        p('spin_topic', '/nav_mpc/spin')              # -> sensor_mpc_node 的 spin_topic
        p('align_tolerance_deg', 5.0)
        p('align_kp', 1.5)                            # yaw rate (rad/s) / 誤差 (rad)
        p('align_wz_max', 0.6)                        # rad/s
        p('align_wz_min', 0.2)                        # rad/s, 誤差在容許外時至少這麼快 (靜摩擦)
        # 輪速 / yaw rate: 原地自旋 wz = 2 r u / T_eff -> u = wz x T_eff / (2 r)。實體車 T_eff 0.59 m
        # (2026-11-09), r 0.0514 m -> 5.7。只影響迴路增益, 不準也會收斂 (回授是融合的 yaw)
        p('align_wheel_per_yaw', 5.7)
        p('align_settle', 0.5)                        # s, 誤差在容許內連續這麼久才算對準
        p('align_timeout', 20.0)                      # s
        p('align_clearance', 0.30)                    # m, 原地轉的時候車子中心這麼近有掃描點就不轉
        p('align_only_dist', 0.2)                     # m, 目標離車子這麼近就不規劃, 直接原地轉 (只改車頭)
        # 度; 新路線起點的方向 (起點往前 start_heading_dist 那一點) 跟車頭差超過這個就先原地轉過去再發路線。
        # Hybrid-A* 的路線起點本來就是車頭方向, 會觸發的是不管車頭方向的 Grid (fallback_planner_id):
        # 2026-11-23 nav_lab_08 Grid 給了一條起點朝後 (差 180°) 的路線, 直接交給 MPC -> 輪速打滿 ±9.9 rad/s
        # 原地硬轉 (v_ref 0.39 還在要前進)。0 = 不檢查 (舊行為)
        p('start_align_deg', 45.0)
        p('start_heading_dist', 0.3)                  # m, 路線起點方向用起點到往前這麼遠那一點的連線
        p('events_csv', '')                           # 事件紀錄 (停車 / 規劃 / 到達); '' = 不存; 已存在就加時間, 可寫 {time}
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.frame = str(g('map_frame'))
        self.base_frame = str(g('base_frame'))
        self.planner_id = str(g('planner_id'))
        self.fallback_id = str(g('fallback_planner_id'))
        self.planner_timeout = float(g('planner_timeout'))
        self.spacing = float(g('path_spacing'))
        self.release_delay = float(g('release_delay'))
        self.pose_timeout = float(g('pose_timeout'))
        self.obstacle_check = bool(g('obstacle_check'))
        self.check_distance = float(g('check_distance'))
        self.half_width = float(g('corridor_half_width'))
        self.min_points = int(g('min_points'))
        self.confirm_scans = int(g('confirm_scans'))
        box = [float(v) for v in g('footprint_box')]
        self.footprint_box = box if len(box) == 4 else None
        self.range_min = float(g('range_min'))
        self.range_max = float(g('range_max'))
        lo = [float(v) for v in g('laser_offset')] + [0.0, 0.0, 0.0]
        self.laser_fallback = (lo[0], lo[1], math.radians(lo[2]))
        self.scan_timeout = float(g('scan_timeout'))
        self.replan_wait = float(g('replan_wait'))
        self.replan_wait_invalid = float(g('replan_wait_invalid'))
        self.stale_retry_wait = float(g('stale_retry_wait'))
        self.stale_retries = int(g('stale_retries'))
        self.stop_speed = float(g('stop_speed'))
        self.stop_wait_max = float(g('stop_wait_max'))
        self.replan_period = float(g('replan_period'))
        self.max_replans = int(g('max_replans'))
        self.validate = bool(g('validate_path'))
        self.validate_distance = float(g('validate_distance'))
        self.validate_clearance = float(g('validate_clearance'))
        self.robot_radius = float(g('robot_radius'))
        self.costmap = None             # (grid (h, w) int8, resolution, origin x, origin y)
        self.path_baseline = None       # 路線剛接受時就已經擋住的取樣點
        self.reset_dist = float(g('initialpose_reset_dist'))
        self.reset_yaw = math.radians(float(g('initialpose_reset_yaw_deg')))

        # ---- 狀態 ----
        self.state = IDLE
        self.state_since = self._now()
        self.goal = None                # (x, y, yaw)
        self.pose = None                # (x, y, yaw)
        self.speed = 0.0
        self.t_pose = None
        self.progress = None            # PathProgress (目前的路線)
        self.path_pts = None
        self.release_t = None
        self.seen_unfinished = False
        self.replans = 0
        self.next_plan_t = None
        self.stop_t = None              # 開始等停穩的時刻
        self.stop_wait = 0.0            # 停穩之後還要等多久才規劃 (看是哪種停車)
        self.stale_left = 0             # 這次停車還能「costmap 沒跟上」快速重試幾次
        self.plan_seq = 0               # 每次送規劃 +1, 舊的回應丟掉
        self.plan_t0 = None
        self.goal_handle = None
        self.hit_streak = 0
        self.t_scan = None
        self.last_hits = np.zeros((0, 2))
        self.world_pts = None           # 最新一幀掃描 (map 座標, 已去掉車身)
        self.corridor = None            # 最近一次檢查的走廊中心線
        self.laser_tf = None            # (x, y, yaw) 雷射在 base_link
        self.laser_frame = None
        self.initpose_check = None      # (時刻, x, y, yaw)

        # ---- ROS ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.planner = ActionClient(self, ComputePathToPose, str(g('planner_action')))
        self.pub_path = self.create_publisher(Path, str(g('path_topic')), LATCHED)
        self.pub_hold = self.create_publisher(Bool, str(g('hold_topic')), 10)
        self.pub_state = self.create_publisher(String, str(g('state_topic')), LATCHED)
        self.pub_markers = self.create_publisher(MarkerArray, str(g('marker_topic')), 5)
        self.pub_goal = self.create_publisher(PoseStamped, '/nav_mpc/goal', LATCHED)
        self.align = bool(g('align_heading'))
        self.pub_spin = self.create_publisher(Float32, str(g('spin_topic')), 10)
        self.align_tol = math.radians(float(g('align_tolerance_deg')))
        self.align_kp = float(g('align_kp'))
        self.align_wz_max = float(g('align_wz_max'))
        self.align_wz_min = float(g('align_wz_min'))
        self.align_gain = float(g('align_wheel_per_yaw'))
        self.align_settle = float(g('align_settle'))
        self.align_timeout = float(g('align_timeout'))
        self.align_clear = float(g('align_clearance'))
        self.align_only_dist = float(g('align_only_dist'))
        self.start_align = math.radians(float(g('start_align_deg')))
        self.start_heading_dist = float(g('start_heading_dist'))
        self.pending_path = None        # 先原地轉再發的路線 (pts, prog, plan_ms, detail)
        self.align_t0 = None
        self.align_in_band = None       # 誤差進到容許範圍的時刻
        self.align_yaw = None           # 原地轉的目標航向; None = 目標點的航向 (終點對準)
        self.align_then = None          # None = 轉好就 ARRIVED; 'plan' = 轉好重新規劃 (換起點方向);
        #                                 'path' = 轉好就發 pending_path (路線起點方向跟車頭差太多)
        self.alt_on = bool(g('alt_headings'))
        offs = []
        for d in g('alt_offsets_deg'):
            d = abs(float(d))
            offs += [math.radians(d)] if abs(d - 180.0) < 1e-6 else [math.radians(d), -math.radians(d)]
        self.alt_offsets = sorted(offs, key=abs)
        self.alt_turn_cost = float(g('alt_turn_cost'))
        self.alt_goal_turn_cost = float(g('alt_goal_turn_cost'))
        self.alt_budget = float(g('alt_budget'))
        self.alt_max_turn = math.radians(float(g('alt_max_turn_deg')))
        self.turn_settle = float(g('turn_settle'))
        self.alt_combos = bool(g('alt_combos'))
        self.turn_goal_off = 0.0        # 起點 x 目標一起換: 轉好之後用這個目標方向規劃
        self.snap_dir = str(g('fail_snapshot_dir')) or (
            os.path.dirname(os.path.abspath(str(g('events_csv')))) if str(g('events_csv')) else '')
        self.snap_key = None            # 同一個情況只存一次
        self.planner_ds = max(1, int(g('planner_downsample')))
        self.start_shift_max = float(g('start_shift_max'))
        self.goal_blocked_warned = False  # 這個目標已經報過「在障礙物裡」了
        self.plan_start = None          # 最近一次規劃用的起點 (可能移過)
        self.alt = None                 # 搜尋中的狀態 (dict)
        self.alt_turned = 0.0           # 還沒開出去之前累積原地轉了多少 (rad)
        self.plan_cb = None             # 目前這個規劃要求的回呼 (ok, 路線點 / 原因, plan_ms)
        self.create_subscription(Odometry, str(g('odom_topic')), self.on_odom, ODOM_QOS)
        self.create_subscription(LaserScan, str(g('scan_topic')), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, str(g('goal_topic')), self.on_goal, 10)
        self.create_subscription(Bool, str(g('finished_topic')), self.on_finished, 10)
        if str(g('costmap_topic')):
            self.create_subscription(OccupancyGrid, str(g('costmap_topic')), self.on_costmap, LATCHED)
        if str(g('initialpose_topic')):
            self.create_subscription(PoseWithCovarianceStamped, str(g('initialpose_topic')),
                                     self.on_initialpose, 10)
        self.fusion_reset = (self.create_client(Trigger, str(g('fusion_reset_service')))
                             if str(g('fusion_reset_service')) else None)
        self.create_service(Trigger, '~/cancel', self.on_cancel)
        self.create_service(Trigger, '~/replan', self.on_replan)

        self.events_csv = str(g('events_csv'))
        self._events = None
        if self.events_csv:
            # 不覆寫 (2026-11-20 以前每一輪都蓋掉上一輪): 檔名裡的 {time} 換成啟動的台灣時間; 同名檔已經存在
            # 就在後面加 _<時間> (跟 sensor_mpc 的 output_csv 一樣)
            stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
            self.events_csv = _unique_path(os.path.abspath(self.events_csv.replace('{time}', stamp)), stamp)
            os.makedirs(os.path.dirname(self.events_csv), exist_ok=True)
            self._events_f = open(self.events_csv, 'w', newline='')
            self.get_logger().info(f'事件記錄: {self.events_csv}')
            self._events = csv.DictWriter(self._events_f, fieldnames=EVENT_COLS, restval='',
                                          extrasaction='ignore')
            self._events.writeheader()

        self.create_timer(1.0 / float(g('hold_rate')), self.tick)
        self._publish_state()
        self.get_logger().info(
            f"等目標點 {g('goal_topic')} (Foxglove 3D 面板 'Publish pose', frame {self.frame}); "
            f"規劃 {g('planner_action')} [{self.planner_id}]; 路線 -> {g('path_topic')}; "
            f"停車訊號 -> {g('hold_topic')}")
        self.get_logger().info(
            f'障礙物: {"開" if self.obstacle_check else "關"}, 路徑前方 {self.check_distance} m、'
            f'半寬 {self.half_width} m 的走廊裡 >= {self.min_points} 點連續 {self.confirm_scans} 幀就停車')

    # ───────────────────────────────────────── 小工具
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_state(self, st, why=''):
        if st != self.state:
            if self.state == ALIGNING:
                self._spin(0.0)             # 離開對準: 馬上停轉 (不等 MPC 的 spin_timeout)
            self.get_logger().info(f'{self.state} -> {st}' + (f': {why}' if why else ''))
            self.state = st
            self.state_since = self._now()
            self._publish_state(why)

    def _publish_state(self, why=''):
        m = String()
        m.data = self.state + (f': {why}' if why else '')
        self.pub_state.publish(m)

    def _event(self, event, **kw):
        if self._events is None:
            return
        row = dict(t=self._now(), event=event, state=self.state, **kw)
        if self.pose is not None:
            row.update(x=self.pose[0], y=self.pose[1], yaw=self.pose[2])
        if self.goal is not None:
            row.update(goal_x=self.goal[0], goal_y=self.goal[1], goal_yaw=self.goal[2])
        self._events.writerow(row)
        self._events_f.flush()

    def _pose_ok(self):
        return self.pose is not None and (self._now() - self.t_pose) <= self.pose_timeout

    # ───────────────────────────────────────── 輸入
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.t_pose = self._now()

    def on_goal(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.frame:
            self.get_logger().error(
                f'目標點的 frame 是 {msg.header.frame_id!r}, 要 {self.frame!r} '
                f'(Foxglove 3D 面板的 Display frame 選 {self.frame}), 忽略')
            return
        q = msg.pose.orientation
        self.goal = (msg.pose.position.x, msg.pose.position.y, yaw_of(q))
        self.replans = 0
        self.alt_turned = 0.0
        self.goal_blocked_warned = False
        self.turn_goal_off = 0.0
        self.get_logger().info(f'新目標: x={self.goal[0]:+.2f} y={self.goal[1]:+.2f} '
                               f'yaw={math.degrees(self.goal[2]):+.0f}°')
        self.pub_goal.publish(_pose_msg(*self.goal, self.frame, self.get_clock().now().to_msg()))
        self._event('goal')
        self._request_plan('新目標')

    def on_finished(self, msg: Bool):
        if self.state != DRIVING:
            return
        if not msg.data:
            self.seen_unfinished = True     # 新路線換好了 (之前的 true 可能是舊路線留下來的)
            return
        if self.seen_unfinished:
            self.progress = None
            err = self._heading_error()
            self._event('arrived', detail=f'heading_err_deg={math.degrees(err):+.1f}' if err is not None else '')
            if self.align and err is not None and abs(err) > self.align_tol:
                self._start_align(f'走到目標, 車頭差 {math.degrees(err):+.0f}°, 原地旋轉對準')
            else:
                self._set_state(ARRIVED, '走到目標')

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        x, y, yaw = msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(q)
        if self.state in (PLANNING, DRIVING, WAIT_REPLAN, ALIGNING):
            self._cancel_plan()
            self._set_state(IDLE, '收到初始位姿, 先停車 (重新給目標)')
        self.get_logger().info(f'初始位姿 x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):+.0f}° '
                               '(LiDAR 定位會從這裡配準), 0.5 s 後檢查融合位姿')
        self._event('initialpose', detail=f'{x:.3f},{y:.3f},{yaw:.4f}')
        self.initpose_check = (self._now() + 0.5, x, y, yaw)

    def on_cancel(self, req, res):
        self._cancel_plan()
        self._set_state(IDLE, '手動取消')
        self.progress = None
        res.success = True
        res.message = '取消目前的目標, 停車'
        return res

    def on_replan(self, req, res):
        if self.goal is None:
            res.success = False
            res.message = '沒有目標'
            return res
        self.replans = 0
        self._request_plan('手動重新規劃')
        res.success = True
        res.message = '重新規劃'
        return res

    # ───────────────────────────────────────── 規劃
    def _cancel_plan(self, clear_alt=True):
        self.plan_seq += 1                   # 還在路上的回應作廢
        self.plan_cb = None
        if clear_alt:
            self.alt = None
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None

    def _send_plan(self, start, goal, pid, cb):
        """送一個 ComputePathToPose; 結果呼叫 cb(ok, 路線點 (N, 2) 或失敗原因, plan_ms)。舊的要求作廢。"""
        self._cancel_plan(clear_alt=False)
        msg = ComputePathToPose.Goal()
        stamp = self.get_clock().now().to_msg()
        msg.goal = _pose_msg(*goal, self.frame, stamp)
        msg.start = _pose_msg(*start, self.frame, stamp)
        msg.use_start = True
        msg.planner_id = pid
        seq = self.plan_seq
        self.plan_cb = cb
        self.plan_t0 = self._now()
        fut = self.planner.send_goal_async(msg)
        fut.add_done_callback(lambda f: self._on_goal_accepted(f, seq))

    def _on_goal_accepted(self, fut, seq):
        if seq != self.plan_seq:
            return
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._finish_plan(False, 'planner_server 拒絕這個要求', None)
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self._on_plan_result(f, seq))

    def _on_plan_result(self, fut, seq):
        if seq != self.plan_seq:
            return
        self.goal_handle = None
        res = fut.result()
        plan_ms = (self._now() - self.plan_t0) * 1000.0
        if res is None or res.status != GoalStatus.STATUS_SUCCEEDED or len(res.result.path.poses) < 2:
            code = getattr(res.result, 'error_code', None) if res is not None else None
            self._finish_plan(False, f'沒有路線 (status {getattr(res, "status", "?")}'
                              + (f', error_code {code}' if code else '') + ')', plan_ms)
            return
        raw = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in res.result.path.poses])
        self._finish_plan(True, raw, plan_ms)

    def _finish_plan(self, ok, res, plan_ms):
        cb, self.plan_cb = self.plan_cb, None
        if cb is not None:
            cb(ok, res, plan_ms)

    def _request_plan(self, why, planner_id=None, goal_off=0.0):
        self._cancel_plan()
        self.next_plan_t = None
        if self.goal is None:
            return
        if not self._pose_ok():
            self._retry_later('沒有位姿 (定位還沒好?)')
            return
        if not self.planner.server_is_ready():
            self._retry_later('planner_server 還沒好 (Nav2 lifecycle 還在啟動?)')
            return
        if math.hypot(self.goal[0] - self.pose[0], self.goal[1] - self.pose[1]) < self.align_only_dist:
            # 目標就在車子這裡 (只想改車頭方向): 規劃器會給太短的路線, 直接原地轉
            err = self._heading_error()
            if self.align and abs(err) > self.align_tol:
                self._start_align(f'目標就在車子這裡, 只轉車頭 {math.degrees(err):+.0f}°')
            else:
                self._set_state(ARRIVED, '已經在目標上')
            return
        gc = self._goal_cost()
        if gc is not None and gc >= 99:
            # 目標在障礙物 (或它膨脹出來的禁區) 裡: 規劃器、換車頭方向都不可能成功, 直接說清楚, 不白跑
            msg = (f'目標 ({self.goal[0]:+.2f}, {self.goal[1]:+.2f}) 在障礙物的膨脹區裡 (cost {gc}, '
                   f'離障礙物 < robot_radius {self.robot_radius:.2f} m), 到不了 —— 換目標或把障礙物移開')
            if not self.goal_blocked_warned:
                self.goal_blocked_warned = True
                self.get_logger().warn(msg)
                self._event('goal_blocked', detail=f'cost {gc}')
            self._retry_later('目標被障礙物佔住')
            return
        self.goal_blocked_warned = False      # 障礙物移開了; 之後又被佔住會再報一次
        pid = self.planner_id if planner_id is None else planner_id
        self._set_state(PLANNING, why)
        self.plan_start = self._plan_start(self.pose)
        goal = (self.goal[0], self.goal[1], wrap_pi(self.goal[2] + goal_off))
        tag = pid + (f' 目標車頭 {math.degrees(goal_off):+.0f}°' if goal_off else '')
        self._send_plan(self.plan_start, goal, pid,
                        lambda ok, res, ms: self._on_main_plan(pid, ok, res, ms, tag))

    def _on_main_plan(self, pid, ok, res, plan_ms, tag=None):
        if not ok:
            # 只有「規劃器說沒有路線」才值得換方向試; 被拒絕 / 逾時是 Nav2 本身的問題
            self._plan_failed(pid, res + self._start_cost_hint(), plan_ms, try_alt=res.startswith('沒有路線'))
            return
        pts = resample_polyline(res, self.spacing)
        if len(pts) < 2:
            self._plan_failed(pid, '路線太短 (已經在目標上?)', plan_ms)
            return
        prog = PathProgress(pts)
        n_hits, along = self._blocked_on(prog)
        if n_hits >= self.min_points and self.stale_left > 0:
            # 通常是 planner_server 的 costmap 還沒把剛看到的障礙物畫上去: 很快再試, 不走一般失敗的 2 s
            self.stale_left -= 1
            self.get_logger().info(f'[{pid}] 新路線前方 {along:.2f} m 還有 {n_hits} 個掃描點 (costmap 還沒跟上?), '
                                   f'{self.stale_retry_wait:.1f} s 後再規劃 (還剩 {self.stale_left} 次)')
            self._event('plan_stale', plan_ms=plan_ms, obstacle_along=along, n_hits=n_hits, detail=pid)
            self.stop_t = None
            self.next_plan_t = self._now() + self.stale_retry_wait
            self._set_state(WAIT_REPLAN, f'新路線還穿過障礙物, {self.stale_retry_wait:.1f} s 後再試')
            return
        if n_hits >= self.min_points:
            self._plan_failed(pid, f'新路線前方 {along:.2f} m 還是有 {n_hits} 個掃描點 '
                              '(costmap 還沒畫上去? 或障礙物在規劃器看不到的高度)', plan_ms)
            return
        self._accept_path(pts, prog, plan_ms, tag or pid)

    def _path_start_heading(self, pts):
        """路線起點的方向: 起點 -> 往前 start_heading_dist 那一點 (Grid 的路線是格子鋸齒, 只看第一段會亂跳)。"""
        k = min(len(pts) - 1, max(1, int(round(self.start_heading_dist / max(self.spacing, 1e-3)))))
        d = pts[k] - pts[0]
        if float(np.hypot(*d)) < 1e-3:
            return None
        return math.atan2(float(d[1]), float(d[0]))

    def _accept_path(self, pts, prog, plan_ms, detail, check_heading=True):
        self.pending_path = None
        if check_heading and self.start_align > 0 and self._pose_ok():
            h = self._path_start_heading(pts)
            err = wrap_pi(h - self.pose[2]) if h is not None else 0.0
            if abs(err) > self.start_align:
                desc = f'[{detail}] 路線起點方向跟車頭差 {math.degrees(err):+.0f}°'
                if not self.align:
                    # 原地轉要靠 /nav_mpc/spin (align_heading); 沒開就照舊交給 MPC, 只警告
                    self.get_logger().warn(f'{desc}, 但 align_heading 關著不能先原地轉, 直接交給 MPC')
                elif not self._rotation_clear():
                    self._event('start_turn_blocked', path_len=prog.total, plan_ms=plan_ms,
                                detail=f'{detail} heading_err_deg={math.degrees(err):+.0f}')
                    self._retry_later(f'{desc}, 車子 {self.align_clear} m 內有東西不能原地轉')
                    return
                else:
                    self._event('start_turn', path_len=prog.total, plan_ms=plan_ms,
                                detail=f'{detail} heading_err_deg={math.degrees(err):+.0f}')
                    self.pending_path = (pts, prog, plan_ms, detail)
                    self._start_align(f'{desc}, 先原地轉過去再走', yaw=h, then='path')
                    return
        self.alt_turned = 0.0                # 開得出去了
        self.path_pts = pts
        self.progress = prog
        self.path_baseline = self._baseline(prog)
        if self.path_baseline is not None and self.path_baseline.any():
            self.get_logger().info(f'新路線有 {int(self.path_baseline.sum())} 個取樣點 (每 {self.VALIDATE_STEP} m) '
                                   f'離障礙物已經 < {self.validate_clearance} m, 驗證時不算它們')
        self.seen_unfinished = False
        self.hit_streak = 0
        self._publish_path(pts)
        self.release_t = self._now() + self.release_delay
        L = prog.total
        self.get_logger().info(f'[{detail}] 路線 {L:.2f} m, {len(pts)} 點, 規劃 {plan_ms:.0f} ms')
        self._event('path', path_len=L, plan_ms=plan_ms, detail=detail)
        self._set_state(DRIVING, f'路線 {L:.2f} m')

    def _plan_failed(self, pid, why, plan_ms=None, try_alt=False):
        self.get_logger().warn(f'[{pid}] 規劃失敗: {why}')
        self._event('plan_failed', plan_ms=plan_ms, detail=f'{pid}: {why}')
        if try_alt and pid == self.planner_id and self._start_alt_search(why):
            return
        if self.fallback_id and pid != self.fallback_id:
            self.get_logger().info(f'改用 {self.fallback_id} 試一次')
            self._request_plan(f'{pid} 失敗', planner_id=self.fallback_id)
            return
        self._retry_later(why)

    # ───────────────────────────────────────── 換車頭方向重新規劃
    def _rotation_clear(self):
        """原地轉會不會掃到東西 (跟 ALIGNING 同一個檢查): 最新掃描 align_clearance 內沒有點。"""
        if self.world_pts is None or not len(self.world_pts):
            return True
        return np.linalg.norm(self.world_pts - np.asarray(self.pose[:2]), axis=1).min() >= self.align_clear

    def _start_alt_search(self, why):
        """主規劃器沒有路線: 試起點 / 目標換車頭方向。有東西可以試 -> True (結果非同步回來)。"""
        if not (self.alt_on and self.align) or not self._pose_ok() or self.goal is None:
            return False
        clear = self._rotation_clear()
        left = self.alt_max_turn - self.alt_turned                 # 還能轉多少
        turns = [o for o in self.alt_offsets if abs(o) <= left + 1e-6] if clear else []
        cands = []
        for off in self.alt_offsets:                       # 小角度先試 (轉得少的分數低)
            if off in turns:
                cands.append((off, 0.0))
            cands.append((0.0, off))
        if not clear:
            why_no_turn = f' (車子 {self.align_clear} m 內有東西, 不能原地轉, 只試目標方向)'
        elif len(turns) < len(self.alt_offsets):
            why_no_turn = (f' (已經累積原地轉 {math.degrees(self.alt_turned):.0f}°, '
                           f'起點只試 {math.degrees(left):.0f}° 以內的)' if turns else
                           f' (已經累積原地轉 {math.degrees(self.alt_turned):.0f}°, 這次不試起點)')
        else:
            why_no_turn = ''
        combos = []
        if turns and self.alt_combos:                      # 第二輪: 起點 x 目標一起換, 總轉角小的先試
            combos = sorted(((so, go) for so in turns for go in self.alt_offsets),
                            key=lambda c: abs(c[0]) + abs(c[1]))
        start = getattr(self, 'plan_start', None) or self.pose      # 可能是移過的起點 (_plan_start)
        self.alt = dict(queue=cands, combos=combos, best=None, t0=self._now(), pose=start, n=0, why=why,
                        tried=[])
        self.get_logger().info(f'換車頭方向試 {len(cands)} 種{why_no_turn}')
        self._set_state(PLANNING, f'沒有路線, 換車頭方向試{why_no_turn}')
        self._alt_next()
        return True

    def _alt_score(self, so, go, length=0.0):
        return length + self.alt_turn_cost * abs(so) + self.alt_goal_turn_cost * abs(go)

    def _alt_next(self):
        a = self.alt
        if a is None:
            return
        # 光是轉角的分數就比目前最好的大 -> 不用試
        if a['best'] is not None:
            a['queue'] = [c for c in a['queue'] if self._alt_score(*c) < a['best'][0]]
        elif not a['queue'] and a['combos']:
            # 單邊換方向全失敗: 第二輪試起點 x 目標一起換
            a['queue'], a['combos'] = a['combos'], []
            self.get_logger().info(f"單邊換方向 {a['n']} 種都沒有路線, 再試起點 x 目標一起換 {len(a['queue'])} 種")
        q = a['queue']
        if not q or self._now() - a['t0'] > self.alt_budget:
            self._alt_finish()
            return
        so, go = q.pop(0)
        a['n'] += 1
        x, y, yaw = a['pose']
        gx, gy, gyaw = self.goal
        self._send_plan((x, y, wrap_pi(yaw + so)), (gx, gy, wrap_pi(gyaw + go)), self.planner_id,
                        lambda ok, res, ms: self._on_alt_plan(so, go, ok, res, ms))

    def _on_alt_plan(self, so, go, ok, res, plan_ms):
        a = self.alt
        if a is None:
            return
        a['tried'].append((so, go, bool(ok)))
        if ok:
            pts = resample_polyline(res, self.spacing)
            if len(pts) >= 2:
                prog = PathProgress(pts)
                n_hits, _ = self._blocked_on(prog)
                score = self._alt_score(so, go, prog.total)
                if n_hits < self.min_points and (a['best'] is None or score < a['best'][0]):
                    a['best'] = (score, so, go, pts, prog, plan_ms)
        self._alt_next()

    def _alt_finish(self):
        a, self.alt = self.alt, None
        el = self._now() - a['t0']
        if a['best'] is None:
            why = f"{a['why']}; 換車頭方向 {a['n']} 種也都沒有路線 ({el:.1f} s)"
            self._event('alt_failed', detail=f"{a['n']} 種, {el:.1f} s")
            self._save_fail_snapshot(a)
            if self.fallback_id:
                self.get_logger().info(f'改用 {self.fallback_id} 試一次')
                self._request_plan(why, planner_id=self.fallback_id)
                return
            self._retry_later(why)
            return
        score, so, go, pts, prog, plan_ms = a['best']
        desc = ' + '.join(([f'起點車頭 {math.degrees(so):+.0f}° (先原地轉)'] if so else [])
                          + ([f'目標車頭 {math.degrees(go):+.0f}° (到了再轉回來)'] if go else []))
        self.get_logger().info(f'換車頭方向找到路線: {desc}, 路線 {prog.total:.2f} m, '
                               f"試了 {a['n']} 種, {el:.1f} s")
        self._event('alt_path', path_len=prog.total, plan_ms=el * 1000.0,
                    detail=f'start_off_deg={math.degrees(so):+.0f} goal_off_deg={math.degrees(go):+.0f} tried={a["n"]}')
        if so:
            # 先原地轉到那個方向, 轉好再從實際位姿正常規劃 (原地轉中心會偏幾公分, 不直接用這條)
            self.alt_turned += abs(so)
            self.turn_goal_off = go                # 一起換的話, 轉好之後直接用這個目標方向規劃
            self._start_align(f'原地轉 {math.degrees(so):+.0f}° 再走', yaw=wrap_pi(a['pose'][2] + so), then='plan')
            # (起點移過的話, 轉好之後 _request_plan 會從實際位姿再找一次合法起點)
            return
        # 目標換方向: 直接開這條; path_tracker 說到了以後, ALIGNING 會轉回目標原本的方向
        self._accept_path(pts, prog, plan_ms, f'{self.planner_id} 目標車頭 {math.degrees(go):+.0f}°')

    def _save_fail_snapshot(self, a):
        """換方向也全失敗: 存 costmap + 位姿 + 目標 + 掃描 + 試過哪些, 事後用 scripts/show_nav_fail.py 畫。
        同一個目標、車子沒動 (5 cm / 10°) 只存一次 (之後每 2 s 重試不會一直存)。"""
        if not self.snap_dir or self.costmap is None or self.pose is None or self.goal is None:
            return
        k = self.snap_key
        if (k is not None and np.allclose(k[0], self.goal, atol=1e-3)
                and math.hypot(self.pose[0] - k[1][0], self.pose[1] - k[1][1]) < 0.05
                and abs(wrap_pi(self.pose[2] - k[1][2])) < math.radians(10)):
            return
        self.snap_key = (tuple(self.goal), tuple(self.pose))
        grid, res, ox, oy = self.costmap
        path = os.path.join(self.snap_dir, f'nav_fail_{self._now():.1f}.npz')
        try:
            os.makedirs(self.snap_dir, exist_ok=True)
            np.savez_compressed(
                path, grid=grid, res=res, origin=np.array([ox, oy]), pose=np.array(self.pose),
                plan_start=np.array(a['pose']), goal=np.array(self.goal),
                scan=self.world_pts if self.world_pts is not None else np.zeros((0, 2)),
                old_path=self.path_pts if self.path_pts is not None else np.zeros((0, 2)),
                tried=np.array(a['tried'], dtype=float).reshape(-1, 3), planner_downsample=self.planner_ds,
                robot_radius_hint=self.robot_radius, why=str(a['why']))
            self.get_logger().warn(f'換方向也規劃不出來, costmap 快照存在 {path} '
                                   f'(python3 src/nav_mpc_pkg/scripts/show_nav_fail.py {path})')
            self._event('fail_snapshot', detail=path)
        except Exception as ex:                                  # noqa: BLE001
            self.get_logger().warn(f'存快照失敗: {ex!r}')

    def _retry_later(self, why):
        self.replans += 1
        if self.max_replans > 0 and self.replans > self.max_replans:
            self._set_state(IDLE, f'重試 {self.max_replans} 次都失敗, 放棄這個目標 ({why})')
            self._event('give_up', detail=why)
            return
        self.next_plan_t = self._now() + self.replan_period
        self.stop_t = None
        self._set_state(WAIT_REPLAN, f'{why}; {self.replan_period:.0f} s 後再試')

    def _stop_for_replan(self, wait):
        """障礙物 / costmap 驗證要停車: 先等車停穩, 再等 wait 秒才規劃 (tick 裡做)。"""
        self.stop_t = self._now()
        self.stop_wait = wait
        self.next_plan_t = None
        self.stale_left = self.stale_retries

    def _publish_path(self, pts):
        msg = Path()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        n = len(pts)
        for k in range(n):
            d = pts[min(k + 1, n - 1)] - pts[max(k - 1, 0)] if n > 1 else np.array([1.0, 0.0])
            msg.poses.append(_pose_msg(pts[k, 0], pts[k, 1], math.atan2(d[1], d[0]),
                                       self.frame, msg.header.stamp))
        self.pub_path.publish(msg)

    # ───────────────────────────────────────── costmap 驗證
    def on_costmap(self, msg: OccupancyGrid):
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        self.costmap = (grid, info.resolution, info.origin.position.x, info.origin.position.y)
        if not (self.validate and self.state == DRIVING and self.progress is not None
                and self.pose is not None and self._now() >= (self.release_t or 0.0)):
            return
        s0, _ = self.progress.update(self.pose[:2])
        along = self._path_cost_blocked(self.progress, s0)
        if along is not None:
            self.get_logger().warn(f'costmap: 路線前方 {along:.2f} m 離障礙物 < {self.validate_clearance} m '
                                   '(看到障礙物的另一面了?), 停車, 等重新規劃')
            self._event('path_invalid', obstacle_along=along)
            self._stop_for_replan(self.replan_wait_invalid)
            self._set_state(WAIT_REPLAN, f'路線前方 {along:.2f} m 被 costmap 擋住')

    def _cost_at(self, xy):
        """(M, 2) 世界座標 -> costmap 值 (OccupancyGrid: -1 未知, 0~100); 地圖外 = -1。"""
        grid, res, ox, oy = self.costmap
        ix = np.floor((xy[:, 0] - ox) / res).astype(int)
        iy = np.floor((xy[:, 1] - oy) / res).astype(int)
        ok = (ix >= 0) & (iy >= 0) & (ix < grid.shape[1]) & (iy < grid.shape[0])
        out = np.full(len(xy), -1, dtype=int)
        out[ok] = grid[iy[ok], ix[ok]]
        return out

    VALIDATE_STEP = 0.025

    def _path_samples(self, prog):
        ss = np.arange(0.0, prog.total, self.VALIDATE_STEP)
        return ss, np.column_stack([np.interp(ss, prog.s, prog.pts[:, 0]),
                                    np.interp(ss, prog.s, prog.pts[:, 1])])

    def _too_close(self, xy):
        """(M, 2) 路線取樣點 -> 每一點 validate_clearance 內有沒有障礙物格子 (cost 100)。"""
        grid, res, ox, oy = self.costmap
        r = self.validate_clearance
        out = np.zeros(len(xy), dtype=bool)
        if len(xy) == 0:
            return out
        i0 = max(int(np.floor((xy[:, 0].min() - r - ox) / res)), 0)
        i1 = min(int(np.ceil((xy[:, 0].max() + r - ox) / res)) + 1, grid.shape[1])
        j0 = max(int(np.floor((xy[:, 1].min() - r - oy) / res)), 0)
        j1 = min(int(np.ceil((xy[:, 1].max() + r - oy) / res)) + 1, grid.shape[0])
        if i1 <= i0 or j1 <= j0:
            return out
        jj, ii = np.nonzero(grid[j0:j1, i0:i1] >= 100)
        if len(ii) == 0:
            return out
        cells = np.column_stack([ox + (ii + i0 + 0.5) * res, oy + (jj + j0 + 0.5) * res])
        for k in range(0, len(xy), 64):                     # 分批, 免得 (M x 格子數) 太大
            d2 = ((xy[k:k + 64, None, :] - cells[None, :, :]) ** 2).sum(axis=2)
            out[k:k + 64] = d2.min(axis=1) < r * r
        return out

    def _baseline(self, prog):
        """路線剛被接受時就已經太近的取樣點 (理論上沒有: 規劃用 robot_radius, 驗證比它鬆)。
        之後只看「新變成擋住」的點, 免得同一條路一直被判不合格。"""
        if self.costmap is None:
            return None
        _, xy = self._path_samples(prog)
        return self._too_close(xy)

    def _path_cost_blocked(self, prog, s0):
        """路線 [s0, s0 + validate_distance] 上第一個新變成太靠近障礙物的點離車子多遠; 沒有 -> None。"""
        if self.costmap is None:
            return None
        ss, xy = self._path_samples(prog)
        sel = (ss >= s0) & (ss <= s0 + self.validate_distance)
        if not sel.any():
            return None
        bad = self._too_close(xy[sel])
        base = self.path_baseline
        if base is not None and len(base) == len(ss):
            bad &= ~base[sel]
        if not bad.any():
            return None
        return float(ss[sel][int(np.argmax(bad))] - s0)

    def _goal_cost(self):
        """目標點那一格的 costmap 值 (/global_costmap/costmap, 2 Hz); 沒有 costmap -> None。"""
        if self.costmap is None or self.goal is None:
            return None
        return int(self._cost_at(np.array([self.goal[:2]]))[0])

    def _planner_cost(self, xy):
        """(M, 2) -> 規劃器看到的 cost: costmap 降採樣 planner_downsample 倍 (每塊取最大) 後那一格。"""
        grid, res, ox, oy = self.costmap
        f = self.planner_ds
        ix = np.floor((xy[:, 0] - ox) / res).astype(int) // f * f
        iy = np.floor((xy[:, 1] - oy) / res).astype(int) // f * f
        out = np.full(len(xy), -1, dtype=int)
        for di in range(f):
            for dj in range(f):
                i, j = ix + di, iy + dj
                ok = (i >= 0) & (j >= 0) & (i < grid.shape[1]) & (j < grid.shape[0])
                c = np.full(len(xy), -1, dtype=int)
                c[ok] = grid[j[ok], i[ok]]
                out = np.maximum(out, c)
        return out

    def _plan_start(self, pose):
        """規劃起點: 車子那一格在規劃器眼中是禁區 (>= 99) 的話, 改用 start_shift_max 內最近、cost 最低的合法點
        (車頭方向不變)。找不到 / 不用移 -> 原本的位姿。"""
        if self.costmap is None or self.start_shift_max <= 0:
            return pose
        c0 = int(self._planner_cost(np.array([pose[:2]]))[0])
        if 0 <= c0 < 99:
            return pose
        res = self.costmap[1]
        ang = np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)
        for r in np.arange(res, self.start_shift_max + 1e-9, res):
            pts = np.column_stack([pose[0] + r * np.cos(ang), pose[1] + r * np.sin(ang)])
            c = self._planner_cost(pts)
            ok = (c >= 0) & (c < 99)
            if ok.any():
                k = int(np.argmin(np.where(ok, c, 1000)))
                x, y = pts[k]
                self.get_logger().info(
                    f'起點那一格在規劃器眼中是禁區 (cost {c0}; 車子在膨脹區邊緣, 規劃器 {self.planner_ds}x{self.planner_ds} '
                    f'降採樣取最大), 改從 {r * 100:.1f} cm 外的 ({x:+.3f}, {y:+.3f}) 規劃 (cost {int(c[k])})')
                self._event('start_shift', detail=f'{r * 100:.1f} cm, cost {c0} -> {int(c[k])}')
                return (float(x), float(y), pose[2])
        return pose

    def _start_cost_hint(self):
        """規劃失敗時看車子自己站的格子 (規劃器看到的, 降採樣後): 在禁區裡的話 Hybrid-A* 不會從這裡起步。"""
        # 用實際送給規劃器的起點 (可能已經移過) 判斷; 2026-11-20 以前用車子原本的位置, 起點明明移到合法的地方了
        # 還是印「15 cm 內也找不到合法起點」, 誤導
        start = self.plan_start or self.pose
        if self.costmap is None or start is None:
            return ''
        c = int(self._planner_cost(np.array([start[:2]]))[0])
        if c >= 99:
            return (f'; 車子自己在障礙物的膨脹區裡 (規劃器看到 cost {c}, 離障礙物 < robot_radius {self.robot_radius:.2f} m), '
                    f'{self.start_shift_max * 100:.0f} cm 內也找不到合法起點 —— '
                    '把障礙物或車子移開, 或調小 robot_radius / 調大 check_distance')
        return ''

    # ───────────────────────────────────────── 障礙物
    def _laser_pose(self, frame):
        """雷射在 base_link 的 (x, y, yaw)。TF 查得到就用 TF (static), 查不到用 laser_offset。"""
        if self.laser_tf is not None and frame == self.laser_frame:
            return self.laser_tf
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, frame, Time(),
                                                timeout=Duration(seconds=0.0))
            tr, q = t.transform.translation, t.transform.rotation
            self.laser_tf = (tr.x, tr.y, yaw_of(q))
            self.laser_frame = frame
            self.get_logger().info(f'雷射外參 (TF {self.base_frame} -> {frame}): '
                                   f'x={tr.x:+.3f} y={tr.y:+.3f} yaw={math.degrees(self.laser_tf[2]):+.1f}°')
            return self.laser_tf
        except Exception:
            self.get_logger().warn(f'查不到 TF {self.base_frame} -> {frame}, 先用 laser_offset',
                                   throttle_duration_sec=5.0)
            return self.laser_fallback

    def on_scan(self, msg: LaserScan):
        self.t_scan = self._now()
        if not self.obstacle_check or self.pose is None:
            return
        lx, ly, lyaw = self._laser_pose(msg.header.frame_id)
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                             max(self.range_min, msg.range_min), min(self.range_max, msg.range_max))
        base = to_world(pts, lx, ly, lyaw)                       # 雷射 -> 車體
        base = base[outside_box(base, self.footprint_box)]
        self.world_pts = to_world(base, *self.pose)              # 車體 -> map
        if self.progress is None:
            self.last_hits = np.zeros((0, 2))
            self._publish_markers(None)
            return
        n_hits, along = self._blocked_on(self.progress, update=True)
        self._publish_markers(along if n_hits >= self.min_points else None)
        if self.state != DRIVING or self._now() < (self.release_t or 0.0):
            return
        if n_hits >= self.min_points:
            self.hit_streak += 1
        else:
            self.hit_streak = 0
        if self.hit_streak >= self.confirm_scans:
            self.hit_streak = 0
            self.get_logger().warn(f'前方 {along:.2f} m 有障礙物 ({n_hits} 個掃描點), 停車, 等重新規劃')
            self._event('obstacle', obstacle_along=along, n_hits=n_hits)
            self._stop_for_replan(self.replan_wait)
            self._set_state(WAIT_REPLAN, f'前方 {along:.2f} m 有障礙物')

    def _blocked_on(self, prog, update=False):
        """(走廊裡的點數, 最近的那個點沿路徑多遠)。用最新一幀掃描。"""
        pts = self.world_pts
        if pts is None or self.pose is None or len(pts) == 0:
            self.last_hits = np.zeros((0, 2))
            return 0, math.inf
        if update:
            s0, _ = prog.update(self.pose[:2])
        else:
            s0, _ = PathProgress(prog.pts).update(self.pose[:2])
        poly = prog.sub_polyline(s0, s0 + self.check_distance)
        if len(poly) < 2 or np.linalg.norm(poly[-1] - poly[0]) < 1e-3:
            self.last_hits = np.zeros((0, 2))
            return 0, math.inf
        # 先用距離粗篩 (走廊一定在車子 check_distance + half_width 以內)
        near = np.linalg.norm(pts - np.asarray(self.pose[:2]), axis=1) < self.check_distance + self.half_width + 0.1
        mask, along = corridor_hits(pts[near], poly, self.half_width)
        self.corridor = poly
        self.last_hits = pts[near][mask]
        return int(mask.sum()), float(along.min()) if mask.any() else math.inf

    # ───────────────────────────────────────── 主迴圈
    def tick(self):
        now = self._now()
        hold = True
        if self.state == DRIVING:
            hold = now < (self.release_t or 0.0)
            if not self._pose_ok():
                hold = True
            if self.obstacle_check and (self.t_scan is None or now - self.t_scan > self.scan_timeout):
                hold = True
                self.get_logger().warn(f'掃描超過 {self.scan_timeout} s 沒進來, 看不到前面, 先停',
                                       throttle_duration_sec=2.0)
        m = Bool()
        m.data = bool(hold)
        self.pub_hold.publish(m)

        if self.state == PLANNING and self.plan_t0 is not None and now - self.plan_t0 > self.planner_timeout:
            self.plan_t0 = None
            cb = self.plan_cb
            self._cancel_plan(clear_alt=False)      # 晚到的回應作廢
            if cb is not None:
                cb(False, f'超過 {self.planner_timeout:.0f} s 沒回應', None)

        if self.state == WAIT_REPLAN:
            if self.next_plan_t is None:
                # 剛因為障礙物停車: 先等車停穩, 再等 replan_wait 讓 costmap 更新
                if self.stop_t is None:
                    self.stop_t = now
                stopped = self.speed < self.stop_speed or now - self.stop_t > self.stop_wait_max
                if stopped:
                    self.next_plan_t = now + self.stop_wait
            if self.next_plan_t is not None and now >= self.next_plan_t:   # 不用等的話同一個 tick 就規劃
                self._request_plan('重新規劃')

        if self.state == ALIGNING:
            self._align_step(now)

        if self.initpose_check is not None and now >= self.initpose_check[0]:
            _, x, y, yaw = self.initpose_check
            self.initpose_check = None
            self._check_fusion_vs_initialpose(x, y, yaw)

    # ───────────────────────────────────────── 終點對準
    def _heading_error(self):
        """目標航向 - 目前航向 (rad, ±pi); 沒有位姿 / 目標 -> None。"""
        if self.goal is None or self.pose is None:
            return None
        return wrap_pi(self.goal[2] - self.pose[2])

    def _start_align(self, why, yaw=None, then=None):
        """原地轉: yaw=None 轉到目標點的方向 (終點對準); 否則轉到 yaw, then='plan' 轉好就重新規劃。"""
        self.align_yaw = yaw
        self.align_then = then
        self.align_t0 = self._now()
        self.align_in_band = None
        self._set_state(ALIGNING, why)

    def _align_error(self):
        if self.align_yaw is None:
            return self._heading_error()
        if self.pose is None:
            return None
        return wrap_pi(self.align_yaw - self.pose[2])

    def _spin(self, u):
        m = Float32()
        m.data = float(u)
        self.pub_spin.publish(m)

    def _align_done(self, event, why, warn=False):
        self._spin(0.0)
        err = self._align_error()
        then, self.align_then, self.align_yaw = self.align_then, None, None
        if then in ('plan', 'path'):
            event = 'turned' if event == 'aligned' else 'turn_abort'
        detail = (f'heading_err_deg={math.degrees(err):+.1f}' if err is not None else '') + f' {why}'
        self._event(event, detail=detail.strip())
        (self.get_logger().warn if warn else self.get_logger().info)(
            f'{why}' + (f', 車頭差 {math.degrees(err):+.1f}°' if err is not None else ''))
        if then == 'plan':
            go, self.turn_goal_off = self.turn_goal_off, 0.0
            if event == 'turned':
                self._request_plan('原地轉好了, 重新規劃', goal_off=go)
            else:
                self._retry_later(why)
            return
        if then == 'path':
            pending, self.pending_path = self.pending_path, None
            if event == 'turned' and pending is not None:
                # 轉好了: 發剛剛那條路線 (原地轉中心偏幾公分, MPC 吃得下; 之後的障礙物 / costmap 驗證照常)
                self._accept_path(*pending, check_heading=False)
            else:
                self._retry_later(why)
            return
        self._set_state(ARRIVED, why)

    def _align_step(self, now):
        if not self._pose_ok():
            return                          # 不發 spin -> MPC 照 hold 送 0
        if now - self.align_t0 > self.align_timeout:
            self._align_done('align_abort', f'對準超過 {self.align_timeout:.0f} s, 放棄', warn=True)
            return
        if self.world_pts is not None and len(self.world_pts):
            d = np.linalg.norm(self.world_pts - np.asarray(self.pose[:2]), axis=1).min()
            if d < self.align_clear:
                self._align_done('align_abort', f'車子 {d:.2f} m 內有東西, 原地轉會掃到, 不轉了', warn=True)
                return
        err = self._align_error()
        if abs(err) <= self.align_tol:
            self._spin(0.0)
            if self.align_in_band is None:
                self.align_in_band = now
            elif now - self.align_in_band >= (self.turn_settle if self.align_then in ('plan', 'path')
                                              else self.align_settle):
                self._align_done('aligned', f'車頭對準了 ({now - self.align_t0:.1f} s)')
            return
        self.align_in_band = None
        wz = float(np.clip(self.align_kp * err, -self.align_wz_max, self.align_wz_max))
        if abs(wz) < self.align_wz_min:
            wz = math.copysign(self.align_wz_min, err)
        self._spin(wz * self.align_gain)

    def _check_fusion_vs_initialpose(self, x, y, yaw):
        if self.fusion_reset is None:
            return
        if self.pose is not None:
            dp = math.hypot(self.pose[0] - x, self.pose[1] - y)
            dy = abs(wrap_pi(self.pose[2] - yaw))
            if dp <= self.reset_dist and dy <= self.reset_yaw:
                self.get_logger().info(f'融合位姿跟給的差 {dp:.2f} m / {math.degrees(dy):.0f}°, 不用重設')
                return
            why = f'差 {dp:.2f} m / {math.degrees(dy):.0f}°'
        else:
            why = '還沒有融合位姿'
        if not self.fusion_reset.service_is_ready():
            self.get_logger().warn(f'{why}, 但 {self.fusion_reset.srv_name} 不在, 沒辦法重設融合')
            return
        self.get_logger().warn(f'融合位姿跟給的初始位姿{why}, 重設融合 (車子要靜止約 1 s)')
        self.fusion_reset.call_async(Trigger.Request())
        self._event('fusion_reset', detail=why)

    # ───────────────────────────────────────── 視覺化
    def _publish_markers(self, block_along):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        corr = Marker()
        corr.header.frame_id = self.frame
        corr.header.stamp = stamp
        corr.ns, corr.id = 'corridor', 0
        poly = self.corridor
        if self.progress is None or poly is None:
            corr.action = Marker.DELETE
        else:
            corr.type = Marker.LINE_STRIP
            corr.scale.x = 2.0 * self.half_width
            corr.pose.orientation.w = 1.0
            corr.color = (ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.35) if block_along is not None
                          else ColorRGBA(r=0.2, g=0.9, b=0.3, a=0.25))
            corr.points = [Point(x=float(px), y=float(py), z=0.02) for px, py in poly]
        arr.markers.append(corr)
        hits = Marker()
        hits.header = corr.header
        hits.ns, hits.id = 'hits', 1
        hits.type = Marker.POINTS
        hits.scale.x = hits.scale.y = 0.06
        hits.pose.orientation.w = 1.0
        hits.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        hits.points = [Point(x=float(px), y=float(py), z=0.05) for px, py in self.last_hits]
        if not hits.points:
            hits.action = Marker.DELETE
        arr.markers.append(hits)
        self.pub_markers.publish(arr)

    def shutdown(self):
        try:
            m = Bool()
            m.data = True
            for _ in range(3):
                self.pub_hold.publish(m)
        except Exception:
            pass
        if self._events is not None:
            self._events_f.close()


def main():
    rclpy.init()
    node = NavMpcManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
