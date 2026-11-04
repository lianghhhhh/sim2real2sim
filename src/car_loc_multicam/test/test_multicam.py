#!/usr/bin/env python3
"""方法六的離線 A/B —— 不需要 ROS, 不需要 Isaac, 不需要 YOLO。

    cd src/car_loc_multicam && python3 test/test_multicam.py

**README 裡的每一個數字都是這支印出來的。**

它做的事: 在 9.87 x 6.78 m 的房間裡跑一條 8 字形, 由同一條真值長出

    IMU     60 Hz  零偏 + 雜訊 (+ 可選的 yaw 偏差)
    輪速    60 Hz  尺度誤差 + 雜訊
    8 台相機 15 Hz  config/env_cameras.yaml 的真實內外參 + config/car_model.yaml 的
                   真實車型 -> 外接框; 延遲 80 ms; 每個邊有白雜訊 + **會慢慢飄的系統
                   誤差** (模擬 YOLO 框得忽鬆忽緊, 這才是真正難的部分); 車出了畫面就
                   裁框; 太小 (< 12 px) 就偵測不到

然後比三種做法 (遞推那一半完全一樣, 只差相機怎麼進濾波器):

    [C]    MultiCamEkf: 框直接當量測, 量測模型 = 視線 (這個 package)
    [B]    每台相機先用方法 B 反推 (x, y) (yaw 用濾波器當下的), 再當成圓形 sigma 的
           AbsMeas 丟進 car_loc_fusion 原本的 FusionEkf —— 「只做 B 不做 C」
    [舊]   框的中心那條視線打到車身中段高度 —— 「沿用 car_loc_camera 的做法」

量測照「到達時刻」餵 (t + 延遲), 所以倒帶重放是真的被測到的。
"""
from __future__ import annotations

import math
import os
import sys

for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')   # 小矩陣多執行緒只會更慢 (實測 user time 15 倍)

import numpy as np  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.join(_HERE, '..')
sys.path.insert(0, _PKG)
sys.path.insert(0, os.path.abspath(os.path.join(_PKG, '..', 'car_loc_fusion')))
sys.path.insert(0, os.path.abspath(os.path.join(_PKG, '..', 'car_loc_wheel')))

from car_loc_fusion.fusion_ekf import AbsMeas, FusionEkf, Step       # noqa: E402
from car_loc_wheel.wheel_ins import WheelStillDetector               # noqa: E402
from car_loc_multicam.camera_model import load_cameras               # noqa: E402
from car_loc_multicam.car_model import CarModel, associate, initial_pose  # noqa: E402
from car_loc_multicam.multicam_ekf import BboxMeas, BboxObs, MultiCamEkf  # noqa: E402

CAMS = load_cameras(os.path.join(_PKG, 'config', 'env_cameras.yaml'))
CAR = CarModel.from_yaml(os.path.join(_PKG, 'config', 'car_model.yaml'))

DT = 1.0 / 60.0
RADIUS = 0.05035
TRUE_SCALE, CFG_SCALE = 1.02, 1.0
GYRO_BIAS, ACC_BIAS = 0.004, 0.02
GYRO_NOISE, ACC_NOISE, WHEEL_NOISE = 0.004, 0.02, 0.05
CAM_HZ, CAM_DELAY = 15.0, 0.08
EDGE_NOISE = 1.5          # px, 逐幀白雜訊
EDGE_BIAS = 0.03          # 系統誤差 = 框邊長的 3% (1-sigma), 慢慢飄 (rho 0.95 @ 15 Hz)
P_DETECT = 0.95
MIN_PX = 12.0


# ---------------------------------------------------------------- 真值與感測器
def make_truth(seed=0, dur=60.0, period=40.0, yaw_bias_deg=0.0, cams_on=None,
               p_outlier=0.0, dropout=0.0):
    rng = np.random.default_rng(seed)
    n = int(dur / DT)
    t = np.arange(n) * DT
    # 8 字形 x = 3.0 sin s, y = 1.8 sin 2s; 前 2.5 秒停著 (靜止校正)
    ramp = 0.5 * (1 + np.tanh((t - 3.0) / 0.5))
    s = np.cumsum(2 * np.pi / period * ramp) * DT
    cx, cy = 3.0 * np.sin(s), 1.8 * np.sin(2 * s)
    vx, vy = np.gradient(cx, DT), np.gradient(cy, DT)
    v_prof = np.hypot(vx, vy)
    head = np.unwrap(np.arctan2(vy, vx))
    head[v_prof < 1e-4] = head[np.argmax(v_prof > 1e-4)]
    wz = np.gradient(head, DT)
    a_fwd = np.gradient(v_prof, DT)

    # 真值用**跟濾波器一模一樣的離散模型**積出來 (見 car_loc_fusion/test/test_fusion.py)
    yaw = np.zeros(n)
    px = np.zeros(n)
    py = np.zeros(n)
    vtrue = np.zeros(n)
    yaw[0] = head[0]
    vv = 0.0
    for i in range(1, n):
        ds = vv * DT + 0.5 * a_fwd[i] * DT * DT
        px[i] = px[i - 1] + ds * math.cos(yaw[i - 1])
        py[i] = py[i - 1] + ds * math.sin(yaw[i - 1])
        vv += a_fwd[i] * DT
        vtrue[i] = vv
        yaw[i] = yaw[i - 1] + wz[i] * DT

    gyro = wz + GYRO_BIAS + GYRO_NOISE * rng.standard_normal(n)
    acc = a_fwd + ACC_BIAS + ACC_NOISE * rng.standard_normal(n)
    w_true = vtrue / (TRUE_SCALE * RADIUS)
    W = np.tile(w_true, (4, 1)) + WHEEL_NOISE * rng.standard_normal((4, n))
    yaw_meas = yaw + math.radians(yaw_bias_deg)

    # --- 8 台相機的外接框 ---
    stride = int(round(1.0 / CAM_HZ / DT))
    bias = {c.name: np.zeros(4) for c in CAMS}
    rho = 0.95
    groups = []
    for i in range(0, n, stride):
        obs = []
        for c in CAMS:
            if cams_on is not None and c.name not in cams_on:
                continue
            b, _ = CAR.predict_bbox(c, px[i], py[i], yaw[i], jac=False)
            if b is None:
                continue
            size = max(b[2] - b[0], b[3] - b[1])
            bias[c.name] = (rho * bias[c.name]
                            + math.sqrt(1 - rho * rho) * EDGE_BIAS * size * rng.standard_normal(4))
            b = b + bias[c.name] + EDGE_NOISE * rng.standard_normal(4)
            # 出了畫面就裁 (YOLO 只框得到看得見的部分)
            b = np.array([max(b[0], 0.0), max(b[1], 0.0), min(b[2], c.width), min(b[3], c.height)])
            if b[2] - b[0] < MIN_PX or b[3] - b[1] < MIN_PX:
                continue
            if rng.random() > P_DETECT or rng.random() < dropout:
                continue
            cands = [(b, 0.8)]
            if rng.random() < p_outlier:
                # 認錯: 畫面裡別的地方多一個框 (影子、反光), 而且這一次真的框沒偵測到
                w, h = b[2] - b[0], b[3] - b[1]
                u, v = rng.uniform(0, c.width - w), rng.uniform(c.height * 0.4, c.height - h)
                cands = [(np.array([u, v, u + w, v + h]), 0.9)]
            obs.append((c, cands))
        if obs:
            groups.append(dict(t=float(t[i]), arrive=float(t[i]) + CAM_DELAY, obs=obs))
    return dict(t=t, yaw=yaw, px=px, py=py, v=vtrue, gyro=gyro, acc=acc, W=W,
                yaw_meas=yaw_meas, groups=groups)


# ---------------------------------------------------------------------- 跑一輪
EKF_KW = dict(rewind=True, rewind_horizon=0.4, abs_reject_time=3.0,
              forward_deg=0.0, wheel_scale=CFG_SCALE, sigma_acc=0.05, sigma_k=0.0)


def run(d, method='C', init='truth', **opts):
    if method == 'C':
        f = MultiCamEkf(CAR, **opts, **EKF_KW)
    else:
        f = FusionEkf(**EKF_KW)
    det = WheelStillDetector()
    m = d['t'] < 2.0
    started = False

    def start(i):
        if init == 'truth':
            f.set_pose(d['px'][i], d['py'][i], d['yaw'][i], d['t'][i])
        f.ins.x[4] = float(d['gyro'][m].mean())
        f.ins.x[5] = float(d['acc'][m].mean())

    gi = 0
    groups = d['groups']
    errs, nees, init_err = [], [], None
    for i in range(1, len(d['t'])):
        ti = float(d['t'][i])
        if not started and ti >= 2.0:
            start(i)
            started = True
        if not started:
            continue
        det.add(ti, d['W'][:, i], d['gyro'][i])
        v_wheel = float(np.median(d['W'][:, i])) * RADIUS
        spread = max(abs(d['W'][0, i] - d['W'][2, i]), abs(d['W'][1, i] - d['W'][3, i]))
        f.step(Step(ti, float(d['acc'][i]), float(d['gyro'][i]), v_wheel=v_wheel,
                    spread=spread, still=det.is_still(),
                    yaw_meas=float(d['yaw_meas'][i]), yaw_sigma=0.02, dt=DT))

        while gi < len(groups) and groups[gi]['arrive'] <= ti:
            g = groups[gi]
            gi += 1
            if g['t'] < 2.0:
                continue
            ie = []
            feed(f, method, g, d, ie)
            if ie and init_err is None:
                init_err = ie[0]
        if not f.initialized:
            continue
        r = np.array([f.pos[0] - d['px'][i], f.pos[1] - d['py'][i]])
        errs.append(float(np.linalg.norm(r)))
        try:
            nees.append(float(r @ np.linalg.inv(f.ins.P[0:2, 0:2]) @ r))
        except np.linalg.LinAlgError:
            pass
    return np.array(errs), np.array(nees), f, init_err


def feed(f, method, g, d, init_errs):
    """一組框進濾波器, 走跟節點一樣的路: 挑框 -> (起點) -> 更新。"""
    state = None
    if f.initialized and not f.recovering:
        state = (float(f.pos[0]), float(f.pos[1]), float(f.yaw))
    picked = []
    for c, cands in g['obs']:
        k = associate(CAR, c, cands, state)
        if k is not None:
            picked.append((c, cands[k][0]))
    if not picked:
        return
    if not f.initialized:
        # 起點: 方法 B 反推, yaw 用 IMU 的 (跟節點的 _try_init 一樣)
        i = int(round(g['t'] / DT))
        r = initial_pose(CAR, picked, float(d['yaw_meas'][i]))
        if r is not None:
            f.set_pose(r['x'], r['y'], r['yaw'], t=g['t'], sigma_p=r['sigma_p'],
                       sigma_yaw=r['sigma_yaw'])
            init_errs.append(math.hypot(r['x'] - d['px'][i], r['y'] - d['py'][i]))
        return
    if method == 'C':
        f.absolute(BboxMeas(g['t'], [BboxObs(c, b) for c, b in picked]))
        return
    for c, b in picked:
        if method == 'B':
            # 方法 B: yaw 當已知 (用濾波器當下的), 每台相機各自反推
            r = CAR.solve_position([(c, b)], float(f.yaw), x0=(f.pos[0], f.pos[1]))
            if r is None:
                continue
            x, y = r['x'], r['y']
            sig = float(np.sqrt(max(np.linalg.eigvalsh(r['cov']).max(), 0.0)))
            sig = max(sig * 1.5, 0.02)
        else:
            # 舊做法: 框中心的視線打到車身中段高度
            p = CAR.initial_guess(c, b)
            if p is None:
                continue
            x, y = p
            sig = 0.05
        f.absolute(AbsMeas(g['t'], c.name, x, y, sig))


def summary(errs, nees):
    if len(errs) == 0:
        return '   (沒有輸出)'
    e = errs * 100
    return (f'RMS {np.sqrt(np.mean(e * e)):6.2f}  中位 {np.median(e):5.2f}  '
            f'p95 {np.percentile(e, 95):6.2f}  最大 {e.max():6.2f} cm  NEES {np.mean(nees):6.2f}')


def section(title):
    print(f'\n=== {title} ' + '=' * max(0, 70 - len(title)))


def main():
    print(f'相機 {len(CAMS)} 台, 車型 {len(CAR.points)} 點, base_z {CAR.base_z:.3f} m')

    section('[1] 三種做法, 0.7 m/s 的 8 字形 (60 s, 種子 0/1/2)')
    for method, name in (('C', '[C]  框當量測 (視線)'), ('B', '[B]  先反推再融合'),
                         ('N', '[舊] 框中心打地面')):
        rows = []
        for seed in range(3):
            d = make_truth(seed)
            e, n_, f, _ = run(d, method)
            rows.append((e, n_))
        e = np.concatenate([r[0] for r in rows])
        n_ = np.concatenate([r[1] for r in rows])
        print(f'  {name:18s} {summary(e, n_)}')

    section('[2] 快一倍 (1.5 m/s) —— 延遲 80 ms = 12 cm, 倒帶重放要補回來')
    d = make_truth(0, period=20.0)
    for method, name in (('C', '[C]'), ('B', '[B]')):
        e, n_, f, _ = run(d, method)
        print(f'  {name:5s} {summary(e, n_)}')
    e, n_, f, _ = run(d, 'C')
    print(f"  (C 的倒帶次數 {f.counts['rewind']}, 太舊 {f.counts['too_old']})")

    section('[3] IMU 的 yaw 偏 5 度 (yaw 有誤差時, 位置跟著錯多少)')
    d = make_truth(0, yaw_bias_deg=5.0)
    for method, name in (('C', '[C]'), ('B', '[B]')):
        e, n_, f, _ = run(d, method)
        print(f'  {name:5s} {summary(e, n_)}')
    e, n_, f, _ = run(d, 'C', update_yaw=False)
    print(f'  [C 不修 yaw] {summary(e, n_)}')

    section('[4] 被擋住: 只剩幾台相機 / 每台隨機掉一半')
    for label, kw in (('只剩 cam 0 (+X 牆)', dict(cams_on={'env_cam_0'})),
                      ('只剩 cam 0 + 2 (互相垂直)', dict(cams_on={'env_cam_0', 'env_cam_2'})),
                      ('每台掉 50%', dict(dropout=0.5)),
                      ('每台掉 80%', dict(dropout=0.8))):
        d = make_truth(0, **kw)
        e_c, n_c, _, _ = run(d, 'C')
        e_b, n_b, _, _ = run(d, 'B')
        print(f'  {label:22s} [C] {summary(e_c, n_c)}')
        print(f'  {"":22s} [B] {summary(e_b, n_b)}')

    section('[5] 認錯: 一定比例的偵測是畫面別處的假框')
    for p in (0.1, 0.3):
        d = make_truth(0, p_outlier=p)
        e, n_, f, _ = run(d, 'C')
        c = f.bbox_counts
        print(f'  假框 {p * 100:3.0f}%  {summary(e, n_)}  '
              f"(框 採用 {c['used']} / 閘門擋 {c['gate']} / 強制 {c['forced']})")

    section('[6] 沒有給起點: 第一組框用方法 B 反推 (yaw 用 IMU)')
    for seed in range(3):
        d = make_truth(seed)
        e, n_, f, ie = run(d, 'C', init='none')
        print(f'  種子 {seed}: 起點誤差 {ie * 100:5.2f} cm | 之後 {summary(e, n_)}')

    section('[7] 時鐘偏了幾千秒的框不能進濾波器')
    # 2026-10-30: 時鐘鎖定前的框帶著原始時戳 (未來 2089 s) 進了 abs_log, 每次倒帶都用開機時的
    # 框重新套用一次 -> 估計被拉回起點, 前 66 秒最大 34 cm (同一批資料沒有這些框: 3 cm)
    d = make_truth(0)
    _, _, f, _ = run(d, 'C')
    c, cands = d['groups'][-1]['obs'][0]
    n_log = len(f.abs_log)
    ok = f.absolute(BboxMeas(f.t + 2089.3, [BboxObs(c, cands[0][0])]))
    print(f'  未來 2089 s 的框: 套用 {ok}, abs_log {n_log} -> {len(f.abs_log)}')
    assert not ok and len(f.abs_log) == n_log


if __name__ == '__main__':
    main()
