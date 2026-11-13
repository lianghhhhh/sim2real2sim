"""不用 ROS / Isaac 的離線檢查: 換模型或改 MPC 之後先跑這個。

    MPC_CONFIG_PATH=/workspaces/model/mpc_config.json python3 -m mlp_control_pkg.offline_check

1. 方向檢查: 兩輪同速前進 -> vx > 0; 右輪快 -> wz > 0 (左轉)。
2. CasADi (l4casadi) 版模型跟 PyTorch 版的輸出一樣。
3. 閉迴路: 「受控體」就是模型自己 (所以只驗證 MPC 的接線、符號與速度, 不代表真車效果),
   沿直線 + 半徑 1 m 的圓弧 + 直線的路徑跑 25 s, 印出循跡誤差與每次求解的時間。
acados 會在目前資料夾產生 c_generated_code/, 建議在暫存資料夾執行。
"""
import time

import numpy as np
import torch

from mlp_control_pkg.path_reference import build_reference, curvature_ahead
from mlp_control_pkg.utils import (CUR, NS, NU, SEQ, apply_body_delta_np, build_window_np,
                                   createAcadosSolver, loadConfig, loadModelFunc,
                                   loadTorchModel, snap_rpm)


class Plant:
    """把 PyTorch 模型當成車子: 保留最近 SEQ 步的狀態與控制, 一次走 dt。"""

    def __init__(self, model, s0):
        self.model = model
        self.S = [np.array(s0, float)] * SEQ
        self.U = [np.zeros(NU)] * (SEQ - 1)

    def predict(self, u):
        win = build_window_np(np.array(self.S), np.array(self.U + [np.asarray(u, float)]))
        with torch.no_grad():
            d = self.model(torch.tensor(win, dtype=torch.float32)[None])[0].numpy().astype(float)
        return win, d

    def step(self, u):
        _, d = self.predict(u)
        s = apply_body_delta_np(self.S[-1], d)
        self.S = self.S[1:] + [s]
        self.U = self.U[1:] + [np.asarray(u, float)]
        return s

    def aug(self):
        return np.concatenate(self.S + self.U)


def make_path():
    """直線 2 m -> 左轉半徑 1 m 的半圓 -> 直線 3 m, 每 1 cm 一點。"""
    a = np.c_[np.linspace(0, 2, 201), np.zeros(201)]
    th = np.linspace(-np.pi / 2, np.pi / 2, 315)
    b = np.c_[2 + np.cos(th), 1 + np.sin(th)]
    c = np.c_[np.linspace(2, -1, 301), np.full(301, 2.0)]
    xy = np.vstack([a, b[1:], c[1:]])
    t = np.diff(xy, axis=0)
    ang = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
    return xy, np.r_[ang, ang[-1]]


def nearest_points(path_xy, path_ang, pos, spacing=0.1, n=10):
    """模仿 temp_curve: 投影點往前 spacing, 2*spacing, ... 的 n 個點與航向。"""
    i0 = int(np.argmin(np.linalg.norm(path_xy - pos, axis=1)))
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path_xy, axis=0), axis=1))]
    q = s[i0] + spacing * np.arange(1, n + 1)
    return (np.c_[np.interp(q, s, path_xy[:, 0]), np.interp(q, s, path_xy[:, 1])],
            np.interp(q, s, path_ang))


def main():
    cfg = loadConfig()
    model = loadTorchModel(cfg)
    rest = np.zeros(NS)

    # 1. 方向
    print('== 1. 方向檢查 (從靜止開始, 同一個指令維持 1 s)')
    for name, u in [('前進 5/5', [5, 5]), ('倒車 -5/-5', [-5, -5]), ('左轉 3/7', [3, 7]), ('右轉 7/3', [7, 3])]:
        p = Plant(model, rest)
        for _ in range(20):
            s = p.step(u)
        print(f'   {name:10s}  x {s[0]:+.3f}  y {s[1]:+.3f}  theta {s[2]:+.3f}  vx {s[3]:+.3f}  wz {s[5]:+.3f}')

    # 2. CasADi vs PyTorch
    print('== 2. CasADi vs PyTorch')
    func, lib_dir, lib_name = loadModelFunc(cfg)
    rng = np.random.default_rng(0)
    p = Plant(model, rest)
    worst = 0.0
    for k in range(60):
        u = rng.uniform(-8, 8, size=2) if k % 10 == 0 else p.U[-1]
        win, d_torch = p.predict(u)
        d_ca = np.array(func(win)).reshape(-1)
        worst = max(worst, float(np.max(np.abs(d_ca - d_torch))))
        p.step(u)
    print(f'   最大差異 {worst:.2e}  ({"OK" if worst < 1e-4 else "不一致!"})')

    # 3. 閉迴路
    print('== 3. 閉迴路 (受控體 = 模型自己)')
    solver = createAcadosSolver(func, lib_dir, lib_name, cfg)
    N, dt = int(cfg['N_steps']), float(cfg['dt'])
    v_max, k_curv = float(cfg.get('v_max', 0.3)), float(cfg.get('curvature_slowdown', 0.5))
    spacing = float(cfg.get('path_point_spacing', 0.1))
    path_xy, path_ang = make_path()
    plant = Plant(model, rest)
    prev_x = prev_u = None
    log, times = [], []
    for k in range(int(25.0 / dt)):
        s = plant.S[-1]
        xy, ang = nearest_points(path_xy, path_ang, s[:2], spacing)
        v_ref = v_max / (1.0 + k_curv * curvature_ahead(xy, ang))
        ref = build_reference(xy, ang, spacing, v_ref, dt, N + 1, car_xy=s[:2], car_angle=s[2])
        x0 = plant.aug()
        solver.set(0, 'lbx', x0)
        solver.set(0, 'ubx', x0)
        for t in range(N + 1):
            solver.set(t, 'x', x0 if prev_x is None else prev_x[min(t + 1, N)])
            solver.set(t, 'p', np.append(ref[t], v_ref))
        for t in range(N):
            solver.set(t, 'u', np.zeros(NU) if prev_u is None else prev_u[min(t + 1, N - 1)])
        solver.set(0, 'x', x0)
        t0 = time.perf_counter()
        status = solver.solve()
        times.append((time.perf_counter() - t0) * 1000)
        if status != 0:
            print(f'   step {k}: acados status {status}')
        prev_x = [solver.get(t, 'x') for t in range(N + 1)]
        prev_u = [solver.get(t, 'u') for t in range(N)]
        u = np.array([snap_rpm(v) for v in prev_u[0]])
        x_t, y_t, s_t, c_t = ref[0]
        cte = -(s[0] - x_t) * s_t + (s[1] - y_t) * c_t
        head = np.arctan2(np.sin(s[2]) * c_t - np.cos(s[2]) * s_t, np.cos(s[2]) * c_t + np.sin(s[2]) * s_t)
        log.append((k * dt, s[0], s[1], s[3], v_ref, cte, head, u[0], u[1]))
        plant.step(u)
        if k % 50 == 0:
            print(f'   t={k * dt:5.1f}s  pos ({s[0]:+.2f},{s[1]:+.2f})  v {s[3]:+.2f}/{v_ref:.2f}  '
                  f'cte {cte:+.3f}  head {head:+.3f}  u {u[0]:+.2f}/{u[1]:+.2f}  pred_end '
                  f'({prev_x[N][CUR][0]:+.2f},{prev_x[N][CUR][1]:+.2f})')
    a = np.array(log)
    moving = a[:, 3] > 0.05
    print(f'   循跡: |cte| 平均 {np.mean(np.abs(a[moving, 5])):.3f} m, 最大 {np.max(np.abs(a[moving, 5])):.3f} m; '
          f'|heading| 平均 {np.mean(np.abs(a[moving, 6])):.3f} rad; 走完 x={a[-1, 1]:+.2f} y={a[-1, 2]:+.2f}')
    print(f'   速度: 中位數 {np.median(a[moving, 3]):.3f} m/s (目標 {v_max})')
    print(f'   求解時間: 中位數 {np.median(times):.1f} ms, p99 {np.percentile(times, 99):.1f} ms '
          f'(預算 {dt * 1000:.0f} ms)')
    np.savetxt('offline_check_log.csv', a, delimiter=',', comments='',
               header='t,x,y,vx,v_ref,cte,heading_err,u_left,u_right')
    print('   紀錄存到 offline_check_log.csv')


if __name__ == '__main__':
    main()
