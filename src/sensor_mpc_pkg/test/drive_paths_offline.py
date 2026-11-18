#!/usr/bin/env python3
"""離線試開 config/paths/ 的每一條路線 (要 torch / casadi / l4casadi / acados, 在 sim2real2sim 的容器裡跑):

    cd /tmp && MPC_CONFIG_PATH=/workspaces/model/mpc_config.json \\
        python3 /workspaces/src/sensor_mpc_pkg/test/drive_paths_offline.py [名字開頭 ...]

受控體 = 動態模型自己 (offline_check.Plant), 控制器 = sensor_mpc_node 同一套: PathTracker + build_reference +
acados MPC (擾動參數 = 0) + 整數 rpm。從 (0, 0) 朝 +x 靜止出發, 封閉路線開一圈, 不封閉的開到終點。

這只能證明「MPC + 模型」開得過這條線 (幾何、速度、輪速上限、視界), 不能證明 Isaac 或實體車開得過:
受控體就是模型, 模型沒學好的地方 (例如很小的半徑) 這裡看不出來。

cte 是車子到路徑折線的真正距離 (左正, path_error.py, 跟 sensor_mpc_node 的 CSV 同一個定義)。另外印 ref0:
2026-11-18 以前節點的算法 (參考軌跡第 0 列), 它在半徑 R 的彎道裡多了約 0.1 x 0.2 / (2R) 的偏差, 對照用。

判定 (每一條):
    不合格  離路徑 > 0.30 m、碰到圍籬 (±4.5 x ±3.0, 圓柱圓 0.9 m)、acados 失敗 > 1%、或沒開完
    注意    離路徑 > 0.10 m、或輪速頂到上限 > 20% 的時間
"""
import glob
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from mlp_control_pkg.offline_check import Plant                     # noqa: E402
from mlp_control_pkg.path_reference import build_reference, curvature_ahead  # noqa: E402
from mlp_control_pkg.utils import (NS, NU, createAcadosSolver, loadConfig,   # noqa: E402
                                   loadModelFunc, loadTorchModel, snap_rpm)
from sensor_mpc_pkg.disturbance import N_DIST                       # noqa: E402
from sensor_mpc_pkg.path_error import PathError                      # noqa: E402
from sensor_mpc_pkg.path_tracker import PathTracker, load_path_csv   # noqa: E402

PATH_DIR = os.path.join(os.path.dirname(__file__), '..', 'config', 'paths')
FENCE_X, FENCE_Y = 4.5, 3.0
PILLAR, PILLAR_FENCE = np.array([2.10, -2.04]), 0.9


def is_loop(csv):
    with open(csv) as f:
        head = f.readline()
    return 'open' not in head.split('|')[1] if '|' in head else True


def drive(csv, cfg, model, solver):
    N, dt = int(cfg['N_steps']), float(cfg['dt'])
    v_max, k_curv = float(cfg.get('v_max', 0.3)), float(cfg.get('curvature_slowdown', 0.5))
    spacing = float(cfg.get('path_point_spacing', 0.1))
    wmax = float(cfg.get('max_wheel_speed', 10.0))
    pts = load_path_csv(csv)
    loop = is_loop(csv)
    seg = np.linalg.norm(np.diff(np.r_[pts, pts[:1]] if loop else pts, axis=0), axis=1)
    length = seg.sum()
    tracker = PathTracker(pts, loop=loop, spacing=spacing)
    pe = PathError(pts, loop=loop)
    hint = None
    plant = Plant(model, np.zeros(NS))
    prev_x = prev_u = None
    d0 = np.zeros(N_DIST)
    rec = []
    fails = 0
    travelled = 0.0
    finished = False
    t_max = 3.0 * length / v_max + 10.0
    for k in range(int(t_max / dt)):
        s = plant.S[-1]
        r = tracker.update(s[:2], s[2])
        if r['finished']:
            finished = True
            break
        if loop and travelled >= length:
            finished = True
            break
        xy, ang = r['targets'], np.unwrap(r['headings'])
        v_ref = v_max / (1.0 + k_curv * curvature_ahead(xy, ang))
        ref = build_reference(xy, ang, spacing, v_ref, dt, N + 1, car_xy=s[:2], car_angle=s[2])
        x0 = plant.aug()
        solver.set(0, 'lbx', x0)
        solver.set(0, 'ubx', x0)
        for t in range(N + 1):
            solver.set(t, 'x', x0 if prev_x is None else prev_x[min(t + 1, N)])
            solver.set(t, 'p', np.concatenate([ref[t], [v_ref], d0]))
        for t in range(N):
            solver.set(t, 'u', np.zeros(NU) if prev_u is None else prev_u[min(t + 1, N - 1)])
        solver.set(0, 'x', x0)
        t0 = time.perf_counter()
        status = solver.solve()
        ms = (time.perf_counter() - t0) * 1000
        if status != 0:
            fails += 1
        prev_x = [solver.get(t, 'x') for t in range(N + 1)]
        prev_u = [solver.get(t, 'u') for t in range(N)]
        u = np.array([snap_rpm(v) for v in prev_u[0]])
        x_t, y_t, s_t, c_t = ref[0]
        cte_ref0 = -(s[0] - x_t) * s_t + (s[1] - y_t) * c_t      # 節點 CSV 的 cte 算法 (彎道有偏差)
        e = pe.query(s[0], s[1], s[2], hint, None if hint is None else 20)
        hint = e['seg']
        cte = e['cross']                                           # 到路徑折線的真正距離 (path_error.py), 左正
        rec.append((k * dt, s[0], s[1], cte, cte_ref0, s[3], u[0], u[1], ms, r['recovery']))
        s_new = plant.step(u)
        travelled += float(np.linalg.norm(s_new[:2] - s[:2]))
    a = np.array(rec, dtype=float)
    x, y = a[:, 1], a[:, 2]
    fence = (np.abs(x) > FENCE_X) | (np.abs(y) > FENCE_Y) | (np.hypot(x - PILLAR[0], y - PILLAR[1]) < PILLAR_FENCE)
    sat = (np.abs(a[:, 6:8]) >= wmax - 0.06).any(axis=1)
    c = a[:, 3]
    out = dict(name=os.path.basename(csv)[:-4], loop=loop, length=length, time=a[-1, 0] + dt,
               finished=finished, v=travelled / (a[-1, 0] + dt), rms=np.sqrt(np.mean(c ** 2)),
               maxd=np.abs(c).max(), mean=c.mean(), mean_ref0=a[:, 4].mean(), rms_ref0=np.sqrt(np.mean(a[:, 4] ** 2)), sat=sat.mean() * 100, fails=fails / len(a) * 100,
               fence=int(fence.sum()), recovery=int(a[:, 9].sum()), solve=np.median(a[:, 8]),
               dstep=np.sqrt(np.mean(np.diff(a[:, 6:8], axis=0) ** 2)) * np.sqrt(2))
    bad = out['maxd'] > 0.30 or out['fence'] > 0 or out['fails'] > 1.0 or not finished
    warn = out['maxd'] > 0.10 or out['sat'] > 20
    out['verdict'] = '不合格' if bad else ('注意' if warn else 'OK')
    return out, a


def main(argv):
    csvs = sorted(glob.glob(os.path.join(PATH_DIR, '*.csv')))
    if argv:
        csvs = [c for c in csvs if any(os.path.basename(c).startswith(a) for a in argv)]
    cfg = loadConfig()
    model = loadTorchModel(cfg)
    func, lib_dir, lib_name = loadModelFunc(cfg)
    solver = createAcadosSolver(func, lib_dir, lib_name, cfg, disturbance=True)
    print(f'v_max {cfg.get("v_max")}, max_wheel_speed {cfg.get("max_wheel_speed")}, '
          f'head {cfg["weights"]["head"]}')
    print(f'{"路線":22s} {"長m":>5s} {"秒":>5s} {"m/s":>5s} {"RMS cm":>7s} {"最大cm":>7s} {"平均cm":>7s} '
          f'{"ref0 RMS/平均":>13s} {"飽和%":>6s} {"失敗%":>5s} {"圍籬":>4s} {"回歸":>4s} {"指令變化":>8s} {"solve ms":>8s}  判定')
    rows = []
    for csv in csvs:
        o, a = drive(csv, cfg, model, solver)
        rows.append(o)
        np.savetxt(os.path.join('/tmp', f'drive_{o["name"]}.csv'), a, delimiter=',', fmt='%.5f',
                   header='t,x,y,cte,cte_ref0,vx,cmd_l,cmd_r,solve_ms,recovery', comments='')
        print(f'{o["name"]:22s} {o["length"]:5.1f} {o["time"]:5.1f} {o["v"]:5.2f} {o["rms"] * 100:7.2f} '
              f'{o["maxd"] * 100:7.2f} {o["mean"] * 100:+7.2f} {o["rms_ref0"] * 100:6.2f}/{o["mean_ref0"] * 100:+6.2f} {o["sat"]:6.1f} {o["fails"]:5.1f} {o["fence"]:4d} '
              f'{o["recovery"]:4d} {o["dstep"]:8.2f} {o["solve"]:8.1f}  {o["verdict"]}'
              f'{"" if o["finished"] else " (沒開完)"}', flush=True)
    return 0 if all(o['verdict'] != '不合格' for o in rows) else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
