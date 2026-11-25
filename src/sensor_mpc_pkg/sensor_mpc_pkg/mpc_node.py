"""MPC 控制節點 (感測器版): 每 dt 解一次 acados, 送四輪輪速到 /joint_command。

跟 mlp_control_pkg/car_control_node.py 同一套 MPC (模型、solver、參考點的建法都直接 import
那邊的), 差在:
  * 狀態來自 SensorStateNode (感測器定位; 輪速回授已不進狀態), 不是 Isaac 的 /odom
  * 目標點來自 path_tracker (ROS 節點), 不是 Isaac 裡的 ScriptNode; 航向是絕對值
    (要吃 Isaac 的 /nearest_curve_point 做 A/B 就 path_heading_relative:=true)
  * 定位不可信 (位姿 / 輪速斷、sigma 太大、絕對量測斷太久) 或路徑走完就送 0 停車;
    停超過一個 window (SEQ*dt) 再恢復時, 歷史重新用目前狀態填
  * 循跡誤差同時記「對感測器位姿」與「對 GT」(模擬裡有給 gt_topic 時), 結束時存 CSV;
    cte = 到整條路徑折線的距離 (path_error.py, 訂閱 path_tracker 的 /sensor_mpc/path)
  * offset-free: 每個週期比較模型預測的下一步與實際量到的, 低通後當成擾動加進 MPC 的預測
    (disturbance.py; 修模型與車子不合造成的穩態誤差)
  * 上層停車訊號 (hold_topic, 預設不訂閱): nav_mpc_pkg 的導航管理節點看到前方有障礙物、或還在
    規劃新路線時發 true, 這裡就送 0; hold_timeout > 0 時這條訊號斷太久也停 (上層掛了 = 沒人看路)
  * 原地旋轉 (spin_topic, 預設不訂閱): hold 中收到新鮮的 (< spin_timeout) 旋轉輪速 u 時送 左 -u / 右 +u
    (正 = 逆時針) 而不是 0。nav_mpc 到終點後用它對準車頭。輪速指令仍然只從這個節點發, 定位 / 圍籬 /
    hold 斷線的檢查都在它前面, 而且一樣量化成整數 rpm、限制在 max_wheel_speed
  * 圍籬: 車子 (感測器位姿) 跑出 geofence_x/y 的範圍、或進到 geofence_circles 的圓裡
    (柱子), 或 MPC 預測的前 geofence_horizon 秒會出去, 就送 0 並**鎖住**, 要呼叫
    ~/release_geofence 服務才會再動 (實體車上沒有牆的保護, 這是唯一的安全網)
"""
from __future__ import annotations

import csv
import math
import os
import time
from collections import deque
from datetime import datetime, timedelta, timezone

import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64MultiArray
from std_srvs.srv import Trigger

from mlp_control_pkg.path_reference import build_reference, curvature_ahead
from mlp_control_pkg.utils import (CUR, I_TH, NU, SEQ, createAcadosSolver, loadConfig,
                                   loadModelFunc, snap_rpm)

from .disturbance import N_DIST, DisturbanceEstimator
from .path_error import PathError

JOINT_NAMES = ['front_left_joint', 'front_right_joint', 'rear_left_joint', 'rear_right_joint']

# cte / head_err (含 gt_): 到整條路徑折線的距離與切線 (path_error.py), err_src = 'path'。
# 2026-11-18 以前是對參考軌跡第 0 列算的, 彎道有 0.1*0.2/(2R) 的偏差; 收不到 /sensor_mpc/path 時
# 還是用舊算法, err_src = 'ref0'。ate 一直是對參考軌跡第 0 列 (速度跟不跟得上)。
# path_s / path_kappa: 車子 (有 GT 就用 GT) 投影點的弧長與曲率 (左轉正), 拿來分直線 / 左轉 / 右轉。
CSV_COLS = ['t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz',
            'cte', 'ate', 'head_err',
            'gt_x', 'gt_y', 'gt_yaw', 'gt_cte', 'gt_ate', 'gt_head_err',
            'path_s', 'path_kappa', 'err_src',
            'v_ref', 'cmd_l', 'cmd_r', 'solve_ms', 'ok', 'pose_age', 'lead', 'reason',
            'dist_vfwd', 'dist_vlat', 'dist_wz']

LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


TW = timezone(timedelta(hours=8))                  # 台灣時間 (沒有夏令時間, 不靠容器的 tzdata)


def _default_csv_dir():
    """<工作區>/car_run_data。工作區從這個套件的 install 位置往上找 (/workspaces 或實體車的 /s2r)。"""
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = os.path.abspath(get_package_prefix('sensor_mpc_pkg'))   # <ws>/install/sensor_mpc_pkg
        parts = prefix.split(os.sep)
        if 'install' in parts:
            ws = os.sep.join(parts[:len(parts) - 1 - parts[::-1].index('install')]) or os.sep
            return os.path.join(ws, 'car_run_data')
    except Exception:
        pass
    return os.path.join(os.getcwd(), 'car_run_data')


def _unique_csv(path, stamp):
    """檔案已經存在就在副檔名前加 _<時間> (再撞到就加 _2, _3 ...), 絕不覆寫舊資料。"""
    if not os.path.exists(path):
        return path
    root, ext = os.path.splitext(path)
    if not root.endswith(stamp):
        root = f'{root}_{stamp}'
    cand, k = root + ext, 2
    while os.path.exists(cand):
        cand, k = f'{root}_{k}{ext}', k + 1
    return cand


def _track_err(x, y, th, ref0):
    """(cross, along, heading) 相對參考點 ref0 = [x, y, sin psi, cos psi]。"""
    x_t, y_t, s_t, c_t = ref0
    dx, dy = x - x_t, y - y_t
    along = dx * c_t + dy * s_t
    cross = -dx * s_t + dy * c_t
    head = math.atan2(math.sin(th) * c_t - math.cos(th) * s_t,
                      math.cos(th) * c_t + math.sin(th) * s_t)
    return cross, along, head


class SensorMpcNode(Node):

    def __init__(self, state_node, config=None):
        super().__init__('sensor_mpc_node')
        p = self.declare_parameter
        p('mpc_config', '')                              # 空 = 環境變數 MPC_CONFIG_PATH
        p('path_topic', '/sensor_mpc/nearest_points')
        p('path_heading_relative', False)                # true = Isaac /nearest_curve_point 的相對航向
        p('full_path_topic', '/sensor_mpc/path')         # 整條路徑 (latched), 算 cte 用; '' = 舊算法
        p('finished_topic', '/sensor_mpc/finished')
        p('joint_command_topic', '/joint_command')
        p('output_csv', '')                              # 空 = <工作區>/car_run_data/sensor_mpc_{time}.csv
        p('rpm_step', 1.0)                               # 輪速量化成整數 rpm (實體車馬達只收整數)
        p('path_timeout', 0.5)                           # s, 目標點太久沒更新就停
        # 上層停車訊號 (std_msgs/Bool, true = 停)。空 = 不訂閱 (= 舊的行為)。
        # hold_timeout > 0: 訂閱了卻超過這麼久沒收到也當成 true (發訊號的節點掛了就停車)
        p('hold_topic', '')
        p('hold_timeout', 0.0)
        # 原地旋轉 (std_msgs/Float32, 輪速 rad/s, 正 = 逆時針)。只在 hold 為 true 時有效。空 = 不訂閱
        p('spin_topic', '')
        p('spin_timeout', 0.3)
        # 圍籬 (map 座標, m)。x_min >= x_max 或 y_min >= y_max = 不檢查矩形。
        p('geofence_x_min', 0.0)
        p('geofence_x_max', 0.0)
        p('geofence_y_min', 0.0)
        p('geofence_y_max', 0.0)
        p('geofence_circles', [0.0])                     # [x, y, r, x, y, r, ...] 禁入的圓 (柱子); [0.0] = 沒有
        p('geofence_horizon', 0.5)                       # s, MPC 預測的這段時間內會出去也停
        # offset-free MPC (見 disturbance.py)。false = 擾動固定 0 (= 舊的行為, 拿來 A/B)
        p('disturbance_enabled', True)
        p('disturbance_tau', 1.0)                        # s, 擾動估計的低通時間常數
        p('disturbance_limit', [0.2, 0.1, 0.1])          # 上限 (前進 m/s, 左側 m/s, yaw rate rad/s)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        config = config or loadConfig(str(g('mpc_config')) or None)
        self.config = config
        self.state_node = state_node
        self.N_steps = int(config['N_steps'])
        self.dt = float(config['dt'])
        self.v_max = float(config.get('v_max', 0.3))
        self.k_curv = float(config.get('curvature_slowdown', 0.5))
        self.spacing = float(config.get('path_point_spacing', 0.1))
        self.rel_heading = bool(g('path_heading_relative'))
        self.rpm_step = float(g('rpm_step'))
        self.path_timeout = float(g('path_timeout'))
        # 檔名裡的 {time} 換成節點啟動的台灣時間; 沒給檔名 = <工作區>/car_run_data/sensor_mpc_{time}.csv;
        # 同名檔案已經存在就自動加時間 (存檔時再檢查一次), 每一輪都是新檔
        self.csv_stamp = datetime.now(TW).strftime('%Y%m%d_%H%M%S')
        name = str(g('output_csv')) or os.path.join(_default_csv_dir(), 'sensor_mpc_{time}.csv')
        self.output_csv = _unique_csv(os.path.abspath(name.replace('{time}', self.csv_stamp)), self.csv_stamp)
        self.get_logger().info(f'這一輪的資料會存到 {self.output_csv} (Ctrl-C 時寫入)')
        self.fence = (float(g('geofence_x_min')), float(g('geofence_x_max')),
                      float(g('geofence_y_min')), float(g('geofence_y_max')))
        self.fence_box = self.fence[0] < self.fence[1] and self.fence[2] < self.fence[3]
        circ = [float(v) for v in g('geofence_circles')]
        self.fence_circles = [tuple(circ[i:i + 3]) for i in range(0, len(circ) - 2, 3)
                              if circ[i + 2] > 0.0]
        self.fence_horizon = float(g('geofence_horizon'))
        self._fence_latched = None       # 鎖住的原因 (字串); None = 沒鎖
        self.hold_topic = str(g('hold_topic'))
        self.hold_timeout = float(g('hold_timeout'))
        self._hold_req = False
        self._t_hold = None              # 上一則 hold 訊息的時刻
        if self.hold_topic:
            self.create_subscription(Bool, self.hold_topic, self.on_hold, 10)
        self.spin_topic = str(g('spin_topic'))
        self.spin_timeout = float(g('spin_timeout'))
        self.max_wheel = float(config.get('max_wheel_speed', 10.0))
        self._spin_u = 0.0
        self._t_spin = None
        if self.spin_topic:
            self.create_subscription(Float32, self.spin_topic, self.on_spin, 10)
        self.create_service(Trigger, '~/release_geofence', self._on_release)

        self.cmd_pub = self.create_publisher(JointState, str(g('joint_command_topic')), 10)
        self.pred_pub = self.create_publisher(Float64MultiArray, '/sensor_mpc/predicted_path', 10)
        self.pred_path_pub = self.create_publisher(Path, '/sensor_mpc/predicted_path_viz', 5)
        self.create_subscription(Float32MultiArray, str(g('path_topic')), self.on_path, 10)
        self.create_subscription(Bool, str(g('finished_topic')), self.on_finished, 10)
        self.path_err = None           # PathError; None = 還沒收到整條路徑, cte 用舊算法
        self._err_hint = {'est': None, 'gt': None}
        if str(g('full_path_topic')):
            self.create_subscription(Path, str(g('full_path_topic')), self.on_full_path, LATCHED)

        self.path_xy = None            # (n, 2)
        self.path_ang = None           # (n,) 絕對航向 rad (relative 模式存的是相對值)
        self.t_path = None
        self.finished = False

        self.prev_u = np.zeros(NU)
        self._solving = False
        self._cycle_count = 0
        self._stopped_since = None     # 停車 (不可信 / 走完) 從什麼時候開始
        self._last_reason = None
        self._last_now = None
        self.rows = []                 # CSV

        self.state_history = deque(maxlen=SEQ)
        self.control_history = deque(maxlen=SEQ - 1)

        self._model_func, self._lib_dir, self._lib_name = loadModelFunc(config)
        self._acados_solver = createAcadosSolver(self._model_func, self._lib_dir, self._lib_name, config,
                                                 disturbance=True)
        self.dist_enabled = bool(g('disturbance_enabled'))
        lim = [float(v) * self.dt for v in g('disturbance_limit')]
        self.dist = DisturbanceEstimator(self._model_func, self.dt, float(g('disturbance_tau')), lim)
        self._last_window = None       # (states, controls 含送出的指令, 時刻) 給擾動估計用
        self.create_timer(self.dt, self.find_control_command)
        self.get_logger().info(
            f'感測器 MPC 啟動: N={self.N_steps}, dt={self.dt}, v_max={self.v_max} m/s, '
            f'model={config["model_path"]}, 目標點 {g("path_topic")} '
            f'({"相對" if self.rel_heading else "絕對"}航向), 指令 {g("joint_command_topic")}')
        self.get_logger().info(
            f'offset-free 擾動估計 {"開" if self.dist_enabled else "關"} (tau {g("disturbance_tau")} s)')
        if self.fence_box or self.fence_circles:
            self.get_logger().info(
                '圍籬: ' + (f'x [{self.fence[0]:+.2f}, {self.fence[1]:+.2f}] y [{self.fence[2]:+.2f}, {self.fence[3]:+.2f}]'
                           if self.fence_box else '沒有矩形')
                + (f', 禁入圓 {self.fence_circles}' if self.fence_circles else '')
                + f', 預測 {self.fence_horizon} s')
        else:
            self.get_logger().warn('沒有設定圍籬 (geofence_*): 定位出錯時沒有東西會擋住車子')
        if self.hold_topic:
            self.get_logger().info(f'上層停車訊號 {self.hold_topic}'
                                   + (f' (超過 {self.hold_timeout} s 沒訊息也停)' if self.hold_timeout > 0 else ''))

    # ───────────────────────────────────────── 輸入
    def on_path(self, msg: Float32MultiArray):
        d = np.asarray(msg.data, dtype=float)
        n = (len(d) - 1) // 3
        if n < 1:
            return
        pts = d[1:1 + 3 * n].reshape(n, 3)
        self.path_xy = pts[:, :2]
        self.path_ang = np.radians(pts[:, 2])
        self.t_path = self._now()

    def on_full_path(self, msg: Path):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(pts) < 2:
            return
        self.path_err = PathError(pts)
        self._err_hint = {'est': None, 'gt': None}
        self.get_logger().info(f'整條路徑 {len(self.path_err.pts)} 點, {self.path_err.total:.1f} m, '
                               f'{"封閉" if self.path_err.loop else "不封閉"}: cte 改量到路徑的距離')

    def _path_err(self, key, x, y, yaw, ref0):
        """-> (cross, along, head_err, s, kappa, src)。along 一直對 ref0。"""
        _, along, _ = _track_err(x, y, yaw, ref0)
        if self.path_err is None:
            cross, _, head = _track_err(x, y, yaw, ref0)
            return cross, along, head, float('nan'), float('nan'), 'ref0'
        hint = self._err_hint[key]
        win = None if hint is None else max(20, len(self.path_err.a) // 4)
        r = self.path_err.query(x, y, yaw, hint, win)
        self._err_hint[key] = r['seg']
        return r['cross'], along, r['head_err'], r['s'], r['kappa'], 'path'

    def on_finished(self, msg: Bool):
        self.finished = bool(msg.data)

    def on_hold(self, msg: Bool):
        self._hold_req = bool(msg.data)
        self._t_hold = self._now()

    def on_spin(self, msg: Float32):
        self._spin_u = float(msg.data)
        self._t_spin = self._now()

    def _spin_command(self):
        """hold 中要原地旋轉的話回 [左, 右] 輪速, 否則 None。hold 訊號本身要是新鮮的 true。"""
        if not self.spin_topic or self._t_spin is None or not self._hold_req:
            return None
        now = self._now()
        if now - self._t_spin > self.spin_timeout:
            return None
        if self.hold_timeout > 0.0 and (self._t_hold is None or now - self._t_hold > self.hold_timeout):
            return None
        u = float(np.clip(self._spin_u, -self.max_wheel, self.max_wheel))
        if self.rpm_step > 0:
            u = snap_rpm(u, self.rpm_step)
        return np.array([-u, u])

    def _external_hold(self):
        """None = 不用停; 否則回原因。"""
        if not self.hold_topic:
            return None
        if self.hold_timeout > 0.0 and (self._t_hold is None
                                        or self._now() - self._t_hold > self.hold_timeout):
            return f'{self.hold_topic} 超過 {self.hold_timeout:.1f} s 沒訊息 (導航管理節點沒在跑?)'
        if self._hold_req:
            return f'上層要求停車 ({self.hold_topic})'
        return None

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ───────────────────────────────────────── 圍籬
    def _fence_violation(self, x, y):
        """None = 在圍籬裡; 否則回原因。"""
        if self.fence_box:
            x0, x1, y0, y1 = self.fence
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                return f'({x:+.2f}, {y:+.2f}) 在圍籬 x [{x0:+.2f}, {x1:+.2f}] y [{y0:+.2f}, {y1:+.2f}] 外'
        for cx, cy, r in self.fence_circles:
            if math.hypot(x - cx, y - cy) < r:
                return f'({x:+.2f}, {y:+.2f}) 進到禁入圓 ({cx:+.2f}, {cy:+.2f}) r {r:.2f}'
        return None

    def _on_release(self, req, res):
        res.success = self._fence_latched is not None
        res.message = f'解除圍籬鎖定 ({self._fence_latched})' if res.success else '沒有鎖住'
        if res.success:
            self.get_logger().warn(res.message)
        self._fence_latched = None
        return res

    # ───────────────────────────────────────── 輸出
    def publish_control_command(self, u):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.velocity = [float(u[0]), float(u[1]), float(u[0]), float(u[1])]
        self.cmd_pub.publish(msg)

    def stop_car(self):
        """結束前補送幾次 0 (實體車 0.3 s 沒指令也會自己停; Isaac 要靠 actuator_model 的逾時)。"""
        try:
            for _ in range(5):
                self.publish_control_command(np.zeros(NU))
        except Exception:
            pass

    def publish_predicted(self, predicted, stamp):
        """predicted (N+1, NS): 前 3 個位姿攤平 (跟舊節點一樣) + 整條 horizon 的 nav_msgs/Path。"""
        msg = Float64MultiArray()
        msg.data = [float(v) for row in predicted[:3] for v in row[:3]]
        self.pred_pub.publish(msg)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for row in predicted:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(row[0])
            ps.pose.position.y = float(row[1])
            ps.pose.orientation.z = math.sin(0.5 * row[2])
            ps.pose.orientation.w = math.cos(0.5 * row[2])
            path.poses.append(ps)
        self.pred_path_pub.publish(path)

    # ───────────────────────────────────────── 主迴圈
    def find_control_command(self):
        if self._solving:
            return
        self._solving = True
        try:
            self._step()
        finally:
            self._solving = False

    def _reset_history(self):
        self.state_history.clear()
        self.control_history.clear()
        self.prev_u = np.zeros(NU)
        self._cycle_count = 0
        self._last_window = None

    def _remember_window(self, u):
        """記下這一週期的 window + 送出的指令, 下一週期拿來跟量到的狀態比 (擾動估計)。"""
        # 跟 MPC 的 x0 同一個排法: SEQ 個狀態 + SEQ-1 個舊指令, 再加上這一步送出的 u
        if len(self.state_history) < SEQ or len(self.control_history) < SEQ - 1:
            self._last_window = None
            return
        controls = np.array(list(self.control_history) + [np.asarray(u, dtype=float)])
        self._last_window = (np.array(self.state_history), controls, self._now())

    def _reset_if_time_jumped(self):
        """Isaac 按 Stop / Play (模擬時間倒退): 舊的歷史與 warm start 都作廢。實體車不會發生。"""
        now = self._now()
        last, self._last_now = self._last_now, now
        if last is not None and now < last - 0.5:
            self.get_logger().warn('時間倒退 (Isaac 重新 Play?), 清掉 MPC 歷史重新開始')
            self._reset_history()
            self.state_node.reset()
            self.dist.reset()
            self.path_xy = None
            self.rows.append({'t': float('nan')})     # 分段標記

    def _hold(self, reason, state=None, u=None):
        """定位不可信 / 沒路徑 / 走完: 送 0 (或 u: 原地旋轉), 記下停了多久。"""
        u = np.zeros(NU) if u is None else np.asarray(u, dtype=float)
        self.publish_control_command(u)
        self._last_window = None
        now = self._now()
        if self._stopped_since is None:
            self._stopped_since = now
        if reason != self._last_reason:
            self.get_logger().warn(f'停車: {reason}')
            self._last_reason = reason
        self.prev_u = np.zeros(NU)
        row = {'t': now, 'ok': 0, 'cmd_l': float(u[0]), 'cmd_r': float(u[1]), 'reason': reason,
               'pose_age': self.state_node.last_pose_age, 'lead': self.state_node.last_lead}
        if state is not None:
            row.update(x=state[0], y=state[1], yaw=state[2], vx=state[3], vy=state[4], wz=state[5])
        self.rows.append(row)

    def _resume_if_needed(self):
        if self._stopped_since is None:
            return
        gap = self._now() - self._stopped_since
        self._stopped_since = None
        self._last_reason = None
        if gap > SEQ * self.dt:
            self.get_logger().info(f'停了 {gap:.1f} s 後恢復, 歷史重新填')
            self._reset_history()
        else:
            # 短暫中斷: 歷史裡補上「停著沒動」的控制
            for _ in range(int(round(gap / self.dt))):
                if self.state_history:
                    self.state_history.append(self.state_history[-1])
                    self.control_history.append(np.zeros(NU))

    def _step(self):
        t0 = time.perf_counter()
        self._reset_if_time_jumped()
        state, reason = self.state_node.get_state()
        if state is None:
            self._hold(reason)
            return
        if self._fence_latched is not None:
            self._hold(f'圍籬鎖住: {self._fence_latched} (ros2 service call '
                       f'{self.get_name()}/release_geofence std_srvs/srv/Trigger 解除)', state)
            return
        viol = self._fence_violation(state[0], state[1])
        if viol is not None:
            self._fence_latched = viol
            self.get_logger().error(f'圍籬: {viol}, 停車並鎖住')
            self._hold(f'圍籬: {viol}', state)
            return
        held = self._external_hold()
        if held is not None:
            spin = self._spin_command()
            if spin is not None:
                self._hold('原地旋轉 (上層要求)', state, u=spin)
            else:
                self._hold(held, state)
            return
        if self.finished:
            self._hold('路徑走完', state)
            return
        if self.path_xy is None or (self._now() - self.t_path) > self.path_timeout:
            self._hold('沒有目標點 (path_tracker 沒在發?)', state)
            return
        self._resume_if_needed()

        # 擾動估計: 上一週期的 window + 送出的指令 -> 模型預測的這一步, 跟量到的 state 比
        if self._last_window is not None and self.dist_enabled:
            S_prev, U_prev, t_prev = self._last_window
            self.dist.update(S_prev, U_prev, state, self._now() - t_prev)
        self._last_window = None
        d_param = self.dist.d if self.dist_enabled else np.zeros(N_DIST)

        if not self.state_history:
            self.state_history.extend([state] * SEQ)
            self.control_history.extend([np.zeros(NU)] * (SEQ - 1))
        else:
            self.state_history.append(state)

        car_angle = state[I_TH]
        xy = self.path_xy
        ang = (car_angle + self.path_ang) if self.rel_heading else self.path_ang
        v_ref = self.v_max / (1.0 + self.k_curv * curvature_ahead(xy, ang))
        ref = build_reference(xy, ang, self.spacing, v_ref, self.dt, self.N_steps + 1,
                              car_xy=state[:2], car_angle=car_angle)

        x0 = np.concatenate(list(self.state_history) + list(self.control_history))
        solver = self._acados_solver
        solver.set(0, 'lbx', x0)
        solver.set(0, 'ubx', x0)
        if self._cycle_count == 0:
            for t in range(self.N_steps + 1):
                solver.set(t, 'x', x0)
            for t in range(self.N_steps):
                solver.set(t, 'u', self.prev_u)
        else:
            for t in range(self.N_steps):
                solver.set(t, 'x', self._prev_x_traj[t + 1])
                solver.set(t, 'u', self._prev_u_traj[min(t + 1, self.N_steps - 1)])
            solver.set(self.N_steps, 'x', self._prev_x_traj[self.N_steps])
        solver.set(0, 'x', x0)
        for t in range(self.N_steps + 1):
            solver.set(t, 'p', np.concatenate([ref[t], [v_ref], d_param]))

        status = solver.solve()
        if status != 0:
            self.get_logger().error(f'acados status {status}', throttle_duration_sec=1.0)
            u = self.prev_u * 0.5                                 # 解不出來就先減速
            self.prev_u = u
            self._remember_window(u)
            self.control_history.append(u)                        # 歷史要跟實際送出的對齊
            self.publish_control_command(u)
            self.rows.append(dict(t=self._now(), x=state[0], y=state[1], yaw=state[2],
                                  vx=state[3], vy=state[4], wz=state[5],
                                  cmd_l=u[0], cmd_r=u[1], ok=0))
            return

        u_opt = solver.get(0, 'u')
        u = np.array([snap_rpm(v, self.rpm_step) if self.rpm_step > 0 else float(v) for v in u_opt])
        self._prev_x_traj = [solver.get(t, 'x') for t in range(self.N_steps + 1)]
        self._prev_u_traj = [solver.get(t, 'u') for t in range(self.N_steps)]

        # 預測的前 geofence_horizon 秒會出圍籬 -> 不送這個指令, 停車鎖住
        k_max = min(self.N_steps, int(round(self.fence_horizon / self.dt)))
        for k in range(1, k_max + 1):
            cur = self._prev_x_traj[k][CUR]
            viol = self._fence_violation(cur[0], cur[1])
            if viol is not None:
                self._fence_latched = f'預測 {k * self.dt:.2f} s 後 {viol}'
                self.get_logger().error(f'圍籬: {self._fence_latched}, 停車並鎖住')
                self._cycle_count = 0            # 下次從頭 warm start
                self._hold(f'圍籬: {self._fence_latched}', state)
                return
        self.prev_u = u
        self._remember_window(u)
        self.control_history.append(u)

        self.publish_control_command(u)
        self.publish_predicted(np.array([x[CUR] for x in self._prev_x_traj]),
                               self.get_clock().now().to_msg())
        self._cycle_count += 1

        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        cte, ate, head, ps, pk, src = self._path_err('est', state[0], state[1], state[2], ref[0])
        dps = d_param / self.dt
        row = dict(t=self._now(), x=state[0], y=state[1], yaw=state[2],
                   vx=state[3], vy=state[4], wz=state[5],
                   cte=cte, ate=ate, head_err=head, v_ref=v_ref,
                   path_s=ps, path_kappa=pk, err_src=src,
                   cmd_l=u[0], cmd_r=u[1], solve_ms=elapsed_ms, ok=1,
                   pose_age=self.state_node.last_pose_age, lead=self.state_node.last_lead,
                   dist_vfwd=dps[0], dist_vlat=dps[1], dist_wz=dps[2])
        gt = self.state_node.gt
        if gt is not None:
            gcte, gate, ghead, gs, gk, _ = self._path_err('gt', gt[0], gt[1], gt[2], ref[0])
            row.update(gt_x=gt[0], gt_y=gt[1], gt_yaw=gt[2],
                       gt_cte=gcte, gt_ate=gate, gt_head_err=ghead, path_s=gs, path_kappa=gk)
        self.rows.append(row)

        if elapsed_ms > self.dt * 1000.0:
            self.get_logger().warn(f'MPC cycle over budget: {elapsed_ms:.1f} ms')
        if self._cycle_count % 20 == 0:
            extra = f'  gt_cte={row["gt_cte"]:+.3f}' if 'gt_cte' in row else ''
            sig = self.state_node.pose_sigma
            age = self.state_node.abs_age_now()
            self.get_logger().info(
                f'v={state[3]:+.2f} m/s (ref {v_ref:.2f})  wz={state[5]:+.2f}  '
                f'cmd L/R={u[0]:+.2f}/{u[1]:+.2f} rad/s  cte={cte:+.3f} m{extra}  '
                f'sigma={sig:.3f}' + (f' abs_age={age:.2f}' if age is not None else '')
                + (f'  擾動 v_lat={dps[1] * 100:+.1f} cm/s wz={dps[2]:+.3f}' if self.dist_enabled else '')
                + f'  {elapsed_ms:.1f} ms')

    # ───────────────────────────────────────── 結束
    def report(self):
        ok = [r for r in self.rows if r.get('ok') == 1]
        log = self.get_logger().info
        if not ok:
            self.get_logger().warn('沒有任何一個成功的 MPC 週期 (定位一直不可信? 沒路徑?)')
        else:
            def stats(name, v):
                v = np.asarray(v, dtype=float)
                return (f'{name}: mean={np.mean(v):+.4f}  mean_abs={np.mean(np.abs(v)):.4f}  '
                        f'rmse={np.sqrt(np.mean(v ** 2)):.4f}  max_abs={np.max(np.abs(v)):.4f}')
            log('===== Tracking Error Summary (對感測器位姿) =====')
            srcs = {r.get('err_src') for r in ok}
            log('Cross-track / Heading = ' + ('到路徑折線的距離與切線' if srcs == {'path'} else
                f'有舊算法 (參考軌跡第 0 列, 彎道有偏差) 的週期: {sum(r.get("err_src") == "ref0" for r in ok)}'))
            log(f'Samples: {len(ok)}  (停車 {len(self.rows) - len(ok)} 個週期)')
            log(stats('Cross-track (m)', [r['cte'] for r in ok]))
            log(stats('Along-track (m)', [r['ate'] for r in ok]))
            log(stats('Heading (rad)', [r['head_err'] for r in ok]))
            gt = [r for r in ok if 'gt_cte' in r]
            if gt:
                log('----- 對 GT 位姿 -----')
                log(stats('Cross-track (m)', [r['gt_cte'] for r in gt]))
                log(stats('Along-track (m)', [r['gt_ate'] for r in gt]))
                log(stats('Heading (rad)', [r['gt_head_err'] for r in gt]))
                log(stats('定位誤差 |pos| (m)',
                          [math.hypot(r['x'] - r['gt_x'], r['y'] - r['gt_y']) for r in gt]))
            log(stats('Solve time (ms)', [r['solve_ms'] for r in ok]))
            if self.dist_enabled:
                dv = self.dist.per_second()
                log(f'擾動估計 (結束時): 前進 {dv[0] * 100:+.2f} cm/s, 左側 {dv[1] * 100:+.2f} cm/s, '
                    f'yaw rate {dv[2]:+.4f} rad/s ({self.dist.n} 次更新)')
            ages = [r['pose_age'] for r in ok if math.isfinite(r.get('pose_age', float('nan')))]
            if ages:
                log(f'位姿訊息的延遲 pose_age (s): median={np.median(ages):.3f}  p95={np.percentile(ages, 95):.3f}'
                    f'  (外插 lead median {np.median([r["lead"] for r in ok]):.3f} s)')
            log('==================================================')
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
            self.output_csv = _unique_csv(self.output_csv, self.csv_stamp)   # 跑的期間別人可能寫了同名檔
            with open(self.output_csv, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=CSV_COLS, restval='', extrasaction='ignore')
                w.writeheader()
                w.writerows(self.rows)
            log(f'逐週期資料存到 {self.output_csv}')
        except Exception as e:
            self.get_logger().warn(f'存不了 CSV: {e}')
