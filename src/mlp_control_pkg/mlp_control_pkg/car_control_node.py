import time
from collections import deque

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from mlp_control_pkg.path_reference import build_reference, curvature_ahead
from mlp_control_pkg.utils import (CUR, I_TH, NU, SEQ, createAcadosSolver,
                                   loadModelFunc, snap_rpm)

JOINT_NAMES = ['front_left_joint', 'front_right_joint', 'rear_left_joint', 'rear_right_joint']


class CarControlNode(Node):
    """20 Hz (模擬時間, 要 use_sim_time:=true) 解一次 MPC, 送四輪輪速到 /joint_command。
    要開 car_teleop actuator_model (kros_car.usd 吃的是 /joint_command_isaac)。"""

    def __init__(self, car_state_node, path_points_node, config):
        super().__init__('car_control_node')
        self.cmd_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.pred_path_pub = self.create_publisher(Float64MultiArray, 'predicted_path', 10)

        self.car_state_node = car_state_node
        self.path_points_node = path_points_node
        self.N_steps = int(config['N_steps'])
        self.dt = float(config['dt'])
        self.v_max = float(config.get('v_max', 0.3))
        self.k_curv = float(config.get('curvature_slowdown', 0.5))
        self.spacing = float(config.get('path_point_spacing', 0.1))

        self.prev_u = np.zeros(NU)
        self._solving = False
        self._cycle_count = 0
        self.tracking_errors = []   # (cross_track, along_track, heading_err)

        self.state_history = deque(maxlen=SEQ)
        self.control_history = deque(maxlen=SEQ - 1)

        self._model_func, self._lib_dir, self._lib_name = loadModelFunc(config)
        self._acados_solver = createAcadosSolver(self._model_func, self._lib_dir, self._lib_name, config)
        self.create_timer(self.dt, self.find_control_command)
        self.get_logger().info(
            f'MPC 啟動: N={self.N_steps}, dt={self.dt}, v_max={self.v_max} m/s, '
            f'model={config["model_path"]}')

    # ───────────────────────────────────────── 路徑
    def _path_arrays(self, nearest_points, car_angle):
        """NearestPoint.angle 是「路徑航向 - 車子航向」(temp_curve 用車子真正的 yaw 算的),
        這裡的 car_angle 也是真正的 yaw, 所以路徑的絕對航向 = car_angle + diff。"""
        xy = np.array([[p.x, p.y] for p in nearest_points], dtype=float)
        ang = np.array([car_angle + p.angle for p in nearest_points], dtype=float)
        return xy, ang

    def _speed_ref(self, curvature):
        return self.v_max / (1.0 + self.k_curv * curvature)

    def _record_tracking_error(self, state, ref0):
        x_t, y_t, s_t, c_t = ref0
        dx, dy = state[0] - x_t, state[1] - y_t
        along = dx * c_t + dy * s_t
        cross = -dx * s_t + dy * c_t
        heading_err = np.arctan2(np.sin(state[I_TH]) * c_t - np.cos(state[I_TH]) * s_t,
                                 np.cos(state[I_TH]) * c_t + np.sin(state[I_TH]) * s_t)
        self.tracking_errors.append((cross, along, heading_err))

    def report_tracking_error(self):
        if not self.tracking_errors:
            self.get_logger().warn('No tracking error samples recorded.')
            return
        arr = np.array(self.tracking_errors)

        arr = arr[~np.isnan(arr).any(axis=1)]

        def stats(name, v):
            return (f'{name}: mean={np.mean(v):+.4f}  mean_abs={np.mean(np.abs(v)):.4f}  '
                    f'rmse={np.sqrt(np.mean(v ** 2)):.4f}  max_abs={np.max(np.abs(v)):.4f}')

        log = self.get_logger().info
        log('===== Tracking Error Summary =====')
        log(f'Samples: {len(arr)}')
        log(stats('Cross-track (m)', arr[:, 0]))
        log(stats('Along-track (m)', arr[:, 1]))
        log(stats('Heading (rad)', arr[:, 2]))
        log('===================================')
        try:
            np.savetxt('/tmp/mpc_tracking_errors.csv', arr, delimiter=',',
                       header='cross_track,along_track,heading_err', comments='')
            log('Saved raw samples to /tmp/mpc_tracking_errors.csv')
        except Exception as e:
            self.get_logger().warn(f'Could not save CSV: {e}')

    # ───────────────────────────────────────── 輸出
    def publish_control_command(self, u):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.velocity = [float(u[0]), float(u[1]), float(u[0]), float(u[1])]
        self.cmd_pub.publish(msg)

    def stop_car(self):
        """結束前補送幾次 0 (actuator_model 0.3 s 沒收到指令也會自己歸零)。"""
        try:
            for _ in range(5):
                self.publish_control_command(np.zeros(NU))
        except Exception:
            pass

    def publish_predicted_path(self, predicted):
        """predicted (N+1, NS) -> [x0, y0, th0, x1, y1, th1, x2, y2, th2] (前 3 個位姿)。"""
        msg = Float64MultiArray()
        msg.data = [float(v) for row in predicted[:3] for v in row[:3]]
        self.pred_path_pub.publish(msg)

    # ───────────────────────────────────────── 主迴圈
    def find_control_command(self):
        if self._solving:
            return
        self._solving = True
        try:
            self._step()
        finally:
            self._solving = False

    def _reset_if_time_jumped(self):
        """Isaac 按 Stop / Play (模擬時間倒退): 車子回到起點, 舊的歷史與 warm start 都作廢。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        last, self._last_now = getattr(self, '_last_now', None), now
        if last is not None and now < last - 0.5:
            self.get_logger().warn('模擬時間倒退 (Isaac 重新 Play?), 清掉 MPC 歷史重新開始')
            self.state_history.clear()
            self.control_history.clear()
            self.prev_u = np.zeros(NU)
            self._cycle_count = 0
            self.car_state_node._theta = None
            self.car_state_node.latest_odom = None
            self.car_state_node.latest_joint_state = None
            self.tracking_errors.append((np.nan, np.nan, np.nan))   # 分段標記

    def _step(self):
        t0 = time.perf_counter()
        self._reset_if_time_jumped()
        state = self.car_state_node.get_car_state()
        if state is None:
            self.get_logger().warn('Car state not available yet.', throttle_duration_sec=2.0)
            return
        nearest_points = self.path_points_node.nearest_points
        if not nearest_points:
            self.get_logger().warn('No path points yet (/nearest_curve_point).', throttle_duration_sec=2.0)
            return

        if not self.state_history:
            self.state_history.extend([state] * SEQ)
            self.control_history.extend([np.zeros(NU)] * (SEQ - 1))
        else:
            self.state_history.append(state)

        car_angle = state[I_TH]
        xy, ang = self._path_arrays(nearest_points, car_angle)
        v_ref = self._speed_ref(curvature_ahead(xy, ang))
        ref = build_reference(xy, ang, self.spacing, v_ref, self.dt, self.N_steps + 1,
                              car_xy=state[:2], car_angle=car_angle)
        self._record_tracking_error(state, ref[0])

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
            # 把上一輪的解往前移一格當初始值 (RTI 需要整條 horizon 一致)
            for t in range(self.N_steps):
                solver.set(t, 'x', self._prev_x_traj[t + 1])
                solver.set(t, 'u', self._prev_u_traj[min(t + 1, self.N_steps - 1)])
            solver.set(self.N_steps, 'x', self._prev_x_traj[self.N_steps])
        solver.set(0, 'x', x0)

        for t in range(self.N_steps + 1):
            solver.set(t, 'p', np.append(ref[t], v_ref))

        status = solver.solve()
        if status != 0:
            self.get_logger().error(f'acados status {status}', throttle_duration_sec=1.0)
            return

        u_opt = solver.get(0, 'u')
        u = np.array([snap_rpm(v) for v in u_opt])       # 實際送出去的 = 整數 rpm
        self.prev_u = u
        self.control_history.append(u)
        self._prev_x_traj = [solver.get(t, 'x') for t in range(self.N_steps + 1)]
        self._prev_u_traj = [solver.get(t, 'u') for t in range(self.N_steps)]

        self.publish_control_command(u)
        self.publish_predicted_path(np.array([x[CUR] for x in self._prev_x_traj]))
        self._cycle_count += 1

        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        if elapsed_ms > self.dt * 1000.0:
            self.get_logger().warn(f'MPC cycle over budget: {elapsed_ms:.1f} ms')
        if self._cycle_count % 20 == 0:
            self.get_logger().info(
                f'v={state[3]:+.2f} m/s (ref {v_ref:.2f})  wz={state[5]:+.2f}  '
                f'cmd L/R={u[0]:+.2f}/{u[1]:+.2f} rad/s  cte={self.tracking_errors[-1][0]:+.3f} m  '
                f'{elapsed_ms:.1f} ms')
