"""nav_mpc_manager: 目標點 -> Nav2 規劃 -> MPC 循跡; 前方有新障礙物就停車、重新規劃、繼續走。

    ros2 launch nav_mpc_pkg nav_mpc.launch.py           # 一起開 (定位 + Nav2 + tracker + MPC + 這個)

流程 (狀態 = /nav_mpc/state):

    IDLE ──(/goal_pose)──> PLANNING ──(有路線)──> DRIVING ──(path_tracker 說走到了)──> ALIGNING ──> ARRIVED
                              ^  │ (規劃失敗 / 新路線前面還是擋住)        │
                              │  v                                       │ (走廊裡有掃描點,
                           WAIT_REPLAN <──────────────────────────────────┘  連續 confirm_scans 幀)
                    (車停穩 + 等 replan_wait 秒, 讓 costmap 把障礙物畫上去, 再規劃一次)

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
* 初始位姿 (/initialpose, Foxglove "Publish pose estimate"): LiDAR 定位自己會訂閱並從那裡局部配準;
  這裡負責停車, 並在融合位姿跟給的差太多時叫 /fusion_localizer/reset (融合會用下一則絕對量測當起點)
"""
from __future__ import annotations

import csv
import math
import os

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
        p('release_delay', 0.3)                       # s, 新路線發出後過這麼久才放行 (path_tracker 先換好)
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
        # ---- 重新規劃 ----
        p('replan_wait', 1.0)                         # s, 車停穩後再等這麼久才規劃 (costmap 更新)
        p('stop_speed', 0.03)                         # m/s, |v| 低於這個算停穩
        p('stop_wait_max', 3.0)                       # s, 等停穩最多等這麼久
        p('replan_period', 2.0)                       # s, 規劃失敗後隔多久再試
        p('max_replans', 0)                           # 一個目標最多重新規劃幾次; 0 = 不限
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
        p('events_csv', '')                           # 事件紀錄 (停車 / 規劃 / 到達); '' = 不存
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
        self.stop_speed = float(g('stop_speed'))
        self.stop_wait_max = float(g('stop_wait_max'))
        self.replan_period = float(g('replan_period'))
        self.max_replans = int(g('max_replans'))
        self.validate = bool(g('validate_path'))
        self.validate_distance = float(g('validate_distance'))
        self.validate_clearance = float(g('validate_clearance'))
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
        self.align_t0 = None
        self.align_in_band = None       # 誤差進到容許範圍的時刻
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
            os.makedirs(os.path.dirname(os.path.abspath(self.events_csv)), exist_ok=True)
            self._events_f = open(self.events_csv, 'w', newline='')
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
                self.align_t0 = self._now()
                self.align_in_band = None
                self._set_state(ALIGNING, f'走到目標, 車頭差 {math.degrees(err):+.0f}°, 原地旋轉對準')
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
    def _cancel_plan(self):
        self.plan_seq += 1                   # 還在路上的回應作廢
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None

    def _request_plan(self, why, planner_id=None):
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
                self.align_t0 = self._now()
                self.align_in_band = None
                self._set_state(ALIGNING, f'目標就在車子這裡, 只轉車頭 {math.degrees(err):+.0f}°')
            else:
                self._set_state(ARRIVED, '已經在目標上')
            return
        pid = self.planner_id if planner_id is None else planner_id
        self._set_state(PLANNING, why)
        goal = ComputePathToPose.Goal()
        stamp = self.get_clock().now().to_msg()
        goal.goal = _pose_msg(*self.goal, self.frame, stamp)
        goal.start = _pose_msg(*self.pose, self.frame, stamp)
        goal.use_start = True
        goal.planner_id = pid
        self.plan_seq += 1
        seq = self.plan_seq
        self.plan_t0 = self._now()
        fut = self.planner.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_goal_accepted(f, seq, pid))

    def _on_goal_accepted(self, fut, seq, pid):
        if seq != self.plan_seq:
            return
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._plan_failed(pid, 'planner_server 拒絕這個要求')
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self._on_plan_result(f, seq, pid))

    def _on_plan_result(self, fut, seq, pid):
        if seq != self.plan_seq:
            return
        self.goal_handle = None
        res = fut.result()
        plan_ms = (self._now() - self.plan_t0) * 1000.0
        if res is None or res.status != GoalStatus.STATUS_SUCCEEDED or len(res.result.path.poses) < 2:
            code = getattr(res.result, 'error_code', None) if res is not None else None
            self._plan_failed(pid, f'沒有路線 (status {getattr(res, "status", "?")}'
                              + (f', error_code {code}' if code else '') + ')' + self._start_cost_hint(),
                              plan_ms)
            return
        raw = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in res.result.path.poses])
        pts = resample_polyline(raw, self.spacing)
        if len(pts) < 2:
            self._plan_failed(pid, '路線太短 (已經在目標上?)', plan_ms)
            return
        prog = PathProgress(pts)
        n_hits, along = self._blocked_on(prog)
        if n_hits >= self.min_points:
            self._plan_failed(pid, f'新路線前方 {along:.2f} m 還是有 {n_hits} 個掃描點 '
                              '(costmap 還沒畫上去? 或障礙物在規劃器看不到的高度)', plan_ms)
            return
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
        self.get_logger().info(f'[{pid}] 路線 {L:.2f} m, {len(pts)} 點 (原始 {len(raw)}), '
                               f'規劃 {plan_ms:.0f} ms')
        self._event('path', path_len=L, plan_ms=plan_ms, detail=pid)
        self._set_state(DRIVING, f'路線 {L:.2f} m')

    def _plan_failed(self, pid, why, plan_ms=None):
        self.get_logger().warn(f'[{pid}] 規劃失敗: {why}')
        self._event('plan_failed', plan_ms=plan_ms, detail=f'{pid}: {why}')
        if self.fallback_id and pid != self.fallback_id:
            self.get_logger().info(f'改用 {self.fallback_id} 試一次')
            self._request_plan(f'{pid} 失敗', planner_id=self.fallback_id)
            return
        self._retry_later(why)

    def _retry_later(self, why):
        self.replans += 1
        if self.max_replans > 0 and self.replans > self.max_replans:
            self._set_state(IDLE, f'重試 {self.max_replans} 次都失敗, 放棄這個目標 ({why})')
            self._event('give_up', detail=why)
            return
        self.next_plan_t = self._now() + self.replan_period
        self.stop_t = None
        self._set_state(WAIT_REPLAN, f'{why}; {self.replan_period:.0f} s 後再試')

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
            self.stop_t = self._now()
            self.next_plan_t = None
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

    def _start_cost_hint(self):
        """規劃失敗時看車子自己站的格子: 在膨脹區裡的話 Hybrid-A* 不會從這裡起步。"""
        if self.costmap is None or self.pose is None:
            return ''
        c = int(self._cost_at(np.array([self.pose[:2]]))[0])
        if c >= 99:
            return (f'; 車子自己在障礙物的膨脹區裡 (cost {c}, 離障礙物 < robot_radius), 規劃器不會從這裡起步 —— '
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
            self.stop_t = self._now()
            self.next_plan_t = None
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
            self._cancel_plan()
            self._plan_failed(self.planner_id, f'超過 {self.planner_timeout:.0f} s 沒回應')

        if self.state == WAIT_REPLAN:
            if self.next_plan_t is None:
                # 剛因為障礙物停車: 先等車停穩, 再等 replan_wait 讓 costmap 更新
                if self.stop_t is None:
                    self.stop_t = now
                stopped = self.speed < self.stop_speed or now - self.stop_t > self.stop_wait_max
                if stopped:
                    self.next_plan_t = now + self.replan_wait
            elif now >= self.next_plan_t:
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

    def _spin(self, u):
        m = Float32()
        m.data = float(u)
        self.pub_spin.publish(m)

    def _align_done(self, event, why, warn=False):
        self._spin(0.0)
        err = self._heading_error()
        detail = (f'heading_err_deg={math.degrees(err):+.1f}' if err is not None else '') + f' {why}'
        self._event(event, detail=detail.strip())
        (self.get_logger().warn if warn else self.get_logger().info)(
            f'{why}' + (f', 車頭差 {math.degrees(err):+.1f}°' if err is not None else ''))
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
        err = self._heading_error()
        if abs(err) <= self.align_tol:
            self._spin(0.0)
            if self.align_in_band is None:
                self.align_in_band = now
            elif now - self.align_in_band >= self.align_settle:
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
