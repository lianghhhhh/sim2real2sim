#!/usr/bin/env python3
"""offset-free MPC 的離線測試 (要 torch / casadi / l4casadi / acados, 在 sim2real2sim 的容器裡跑):

    cd /tmp && MPC_CONFIG_PATH=/workspaces/model/mpc_config.json \\
        python3 /workspaces/src/sensor_mpc_pkg/test/test_offset_free.py

受控體 = 動態模型自己, 再加一個模型不知道的固定擾動 (預設: 每秒往右轉 0.03 rad + 往左滑 0.5 cm/s,
跟 2026-11-17 Isaac 上 GT 位姿那輪看到的偏差同一個量級)。在 path_kros_car_mpc_sensor.csv 上跑
40 s, 比較:
    A  擾動估計關 (= 舊的 MPC)
    B  擾動估計開
    C  受控體沒有擾動、估計開 (確認估計器不會自己製造偏差)
通過條件: B 比 A 好, 而且 B 回到「沒有擾動」那一輪的水準 (RMS 差 < 1.5 mm); C 跟沒有擾動時一樣好。

2026-11-17 結果 (mpc_config.json head 權重 0.5): A RMS 1.18 cm, B 0.88, 沒擾動 0.82, C 0.82;
估到的擾動 wz -0.0300 rad/s、v_lat +0.50 cm/s (= 設定值)。
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from mlp_control_pkg.offline_check import Plant                     # noqa: E402
from mlp_control_pkg.path_reference import build_reference, curvature_ahead  # noqa: E402
from mlp_control_pkg.utils import (CUR, NS, NU, SEQ, apply_body_delta_np,  # noqa: E402
                                   createAcadosSolver, loadConfig, loadModelFunc,
                                   loadTorchModel, snap_rpm)
from sensor_mpc_pkg.disturbance import N_DIST, DisturbanceEstimator  # noqa: E402
from sensor_mpc_pkg.path_tracker import PathTracker, load_path_csv   # noqa: E402

PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'path_kros_car_mpc_sensor.csv')
WZ_DIST = -0.03          # rad/s, 往右轉
VLAT_DIST = 0.005        # m/s, 往左滑
DURATION = 40.0


class DisturbedPlant(Plant):
    def __init__(self, model, s0, dt, wz=0.0, vlat=0.0):
        super().__init__(model, s0)
        self.extra = np.zeros(NS)
        self.extra[1] = vlat * dt
        self.extra[2] = wz * dt

    def step(self, u):
        _, d = self.predict(u)
        s = apply_body_delta_np(self.S[-1], d + self.extra)
        self.S = self.S[1:] + [s]
        self.U = self.U[1:] + [np.asarray(u, float)]
        return s


def run(name, cfg, model, func, solver, estimate, wz, vlat):
    N, dt = int(cfg['N_steps']), float(cfg['dt'])
    v_max, k_curv = float(cfg.get('v_max', 0.3)), float(cfg.get('curvature_slowdown', 0.5))
    spacing = float(cfg.get('path_point_spacing', 0.1))
    tracker = PathTracker(load_path_csv(PATH), loop=True, spacing=spacing)
    plant = DisturbedPlant(model, np.zeros(NS), dt, wz, vlat)   # 出生點 (0, 0) 朝 +x, 就在路徑起點
    est = DisturbanceEstimator(func, dt, tau=1.0, limit=(0.01, 0.005, 0.005))
    prev_x = prev_u = None
    last = None
    ctes, times = [], []
    for k in range(int(DURATION / dt)):
        s = plant.S[-1]
        if estimate and last is not None:
            est.update(last[0], last[1], s, dt)
        d = est.d if estimate else np.zeros(N_DIST)
        r = tracker.update(s[:2], s[2])
        xy, ang = r['targets'], np.unwrap(r['headings'])
        v_ref = v_max / (1.0 + k_curv * curvature_ahead(xy, ang))
        ref = build_reference(xy, ang, spacing, v_ref, dt, N + 1, car_xy=s[:2], car_angle=s[2])
        x0 = plant.aug()
        solver.set(0, 'lbx', x0)
        solver.set(0, 'ubx', x0)
        for t in range(N + 1):
            solver.set(t, 'x', x0 if prev_x is None else prev_x[min(t + 1, N)])
            solver.set(t, 'p', np.concatenate([ref[t], [v_ref], d]))
        for t in range(N):
            solver.set(t, 'u', np.zeros(NU) if prev_u is None else prev_u[min(t + 1, N - 1)])
        solver.set(0, 'x', x0)
        t0 = time.perf_counter()
        status = solver.solve()
        times.append((time.perf_counter() - t0) * 1000)
        if status != 0:
            print(f'   {name} step {k}: acados status {status}')
        prev_x = [solver.get(t, 'x') for t in range(N + 1)]
        prev_u = [solver.get(t, 'u') for t in range(N)]
        u = np.array([snap_rpm(v) for v in prev_u[0]])
        x_t, y_t, s_t, c_t = ref[0]
        ctes.append(-(s[0] - x_t) * s_t + (s[1] - y_t) * c_t)
        last = (np.array(plant.S), np.array(plant.U + [u]))
        plant.step(u)
    c = np.array(ctes[int(5.0 / dt):])        # 去掉前 5 s (估計器還沒收斂)
    dv = est.per_second()
    print(f'   {name:34s} cte 平均 {c.mean() * 100:+.2f} cm  RMS {np.sqrt(np.mean(c ** 2)) * 100:.2f} cm  '
          f'最大 {np.abs(c).max() * 100:.2f} cm | 估到 v_lat {dv[1] * 100:+.2f} cm/s wz {dv[2]:+.4f} rad/s | '
          f'solve 中位 {np.median(times):.1f} ms')
    return c, dv


def main():
    cfg = loadConfig()
    model = loadTorchModel(cfg)
    func, lib_dir, lib_name = loadModelFunc(cfg)
    solver = createAcadosSolver(func, lib_dir, lib_name, cfg, disturbance=True)
    print(f'受控體擾動: wz {WZ_DIST:+.3f} rad/s, v_lat {VLAT_DIST * 100:+.1f} cm/s; 路徑 {os.path.basename(PATH)}, {DURATION:.0f} s')
    a, _ = run('A 有擾動, 估計關 (舊 MPC)', cfg, model, func, solver, False, WZ_DIST, VLAT_DIST)
    b, dv = run('B 有擾動, 估計開', cfg, model, func, solver, True, WZ_DIST, VLAT_DIST)
    c0, _ = run('0 沒擾動, 估計關', cfg, model, func, solver, False, 0.0, 0.0)
    c, dvc = run('C 沒擾動, 估計開', cfg, model, func, solver, True, 0.0, 0.0)
    ok = True
    rms = lambda v: float(np.sqrt(np.mean(v ** 2)))  # noqa: E731
    if not rms(b) < rms(a):
        print('FAIL: 估計開之後 RMS 沒有變小'); ok = False
    if not rms(b) < rms(c0) + 0.0015:
        print(f'FAIL: 估計開之後沒有回到沒擾動的水準 ({rms(b) * 100:.2f} vs {rms(c0) * 100:.2f} cm)'); ok = False
    if not np.sqrt(np.mean(c ** 2)) < np.sqrt(np.mean(c0 ** 2)) + 0.003:
        print('FAIL: 沒有擾動時估計器讓誤差變大超過 3 mm'); ok = False
    if not abs(dv[2] - WZ_DIST) < 0.5 * abs(WZ_DIST):
        print(f'FAIL: 估到的 wz {dv[2]:+.4f} 離真值 {WZ_DIST:+.3f} 太遠'); ok = False
    print('all OK' if ok else 'FAILED')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
