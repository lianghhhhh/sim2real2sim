#!/usr/bin/env python3
"""拿 `car_run_data/sim_data.csv` (真的 Isaac 資料) 離線重跑濾波器, 並量參數。

    python3 test/replay_csv.py ../../car_run_data/sim_data.csv
    python3 test/replay_csv.py ../../car_run_data/sim_data.csv --measure
    python3 test/replay_csv.py ../../car_run_data/sim_data.csv --no-slip-gate

**不需要 ROS, 不需要 Isaac。** 參數不要用猜的 —— 這份 CSV 有四輪轉速、ground
truth 位姿與角速度, 該給多少全部量得出來, 而且每次餵的是完全一樣的資料。

這支能驗證什麼、不能驗證什麼
----------------------------
CSV 裡**沒有原始的 IMU 加速度** (只有 ground truth 位姿、四輪轉速、角速度),
所以這裡跑的是「輪速 + 陀螺儀」那一半: 遞推時 `a_fwd = 0`, 過程雜訊放大到蓋得住
真實的加速度。這正好也是**加速度計整個壞掉時**的下界 —— 實務上很有參考價值。

加速度那一路 (扣重力、b_a 估計、打滑時靠 IMU 撐) 由 `test/test_wheel_ins.py`
的合成資料驗證。兩支要一起看。

`sim_data.csv` 是什麼資料
-------------------------
228 秒、4557 筆、20 Hz。這份資料是為了量**打滑與滑行**錄的, 所以裡面有大量
蓄意製造的極端情況: 12 次高速自旋 (w0 = 2/5/8/11 rad/s)、急煞、B4 slip_launch。
一般行駛 (`Reposition` 那些) 只占一部分。

也就是說: **下面跑出來的漂移率是悲觀的上界, 不是日常表現。**
拿它做 A/B 很好 (兩邊吃的是同一份資料), 拿它當「這個方法有多準」會低估。
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from car_loc_wheel.wheel_ins import WheelIns, WheelStillDetector   # noqa: E402

WHEEL_COLS = ('front_left_velocity', 'front_right_velocity',
              'rear_left_velocity', 'rear_right_velocity')


def load(path):
    rows = list(csv.DictReader(open(path)))

    def col(n):
        return np.array([float(r[n]) if r.get(n) not in (None, '', 'nan') else np.nan
                         for r in rows])

    have = set(rows[0]) if rows else set()
    if 'gt_yaw' in have:
        yaw = col('gt_yaw')
    elif 'car_orientation_w' in have:
        # 舊格式的 CSV 沒有 gt_yaw 那一欄, 只有四元數。自己轉。
        qx, qy, qz, qw = (col('car_orientation_' + k) for k in 'xyzw')
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    else:
        raise SystemExit('CSV 裡既沒有 gt_yaw 也沒有 car_orientation_* —— '
                         '沒有 ground truth 的朝向就沒辦法量參數')
    d = {
        't': col('timestamp'),
        'px': col('car_position_x'),
        'py': col('car_position_y'),
        'yaw': yaw,
        'wz': col('car_angular_velocity_z'),
        'W': np.stack([col(c) for c in WHEEL_COLS]),
        'scenario': np.array([r.get('scenario_name', '') for r in rows]),
    }
    # 有 NaN 的列直接丟掉。CSV 是逐列快照, 某個來源那一幀還沒來就是 NaN ——
    # 留著的話 np.gradient 會把 NaN 傳染給前後鄰居, 而 np.var 一遇到 NaN 整個
    # 統計量就變成 NaN (實測 sim_data.backup 那份就是這樣直接炸掉)。
    need = np.stack([d['t'], d['px'], d['py'], d['yaw'], d['wz']] + list(d['W']))
    keep = np.all(np.isfinite(need), axis=0)
    if not keep.all():
        print(f'  (丟掉 {int((~keep).sum())} 列有 NaN 的資料, 剩 {int(keep.sum())} 列)')
    if keep.sum() < 50:
        raise SystemExit('可用的列太少 —— 檢查 CSV 是不是缺了輪速或 ground truth 欄位')
    for k in ('t', 'px', 'py', 'yaw', 'wz'):
        d[k] = d[k][keep]
    d['W'] = d['W'][:, keep]
    d['scenario'] = d['scenario'][keep]

    # ground truth 的速度**用位置微分**, 不要用 car_linear_velocity_* 那幾欄 ——
    # 實測那幾欄跟位置微分的相關係數只有 0.06, 量級也差十倍 (中位數 0.06 vs
    # 0.59 m/s), 拿來當真值會把所有結論帶歪。
    dt = np.gradient(d['t'])
    vx, vy = np.gradient(d['px']) / dt, np.gradient(d['py']) / dt
    # 車頭方向 = yaw + forward_deg。car.usd 的車頭是 -Y, 所以是 yaw - 90°。
    fwd = d['yaw'] - math.pi / 2
    d['vf'] = vx * np.cos(fwd) + vy * np.sin(fwd)
    d['vlat'] = -vx * np.sin(fwd) + vy * np.cos(fwd)
    d['wz_fd'] = np.gradient(np.unwrap(d['yaw'])) / dt
    d['spread'] = np.maximum(np.abs(d['W'][0] - d['W'][2]),
                             np.abs(d['W'][1] - d['W'][3]))
    d['med'] = np.median(d['W'], axis=0)
    d['mean'] = d['W'].mean(axis=0)
    return d


# ---------------------------------------------------------------- 量參數
def measure(d, radius=0.075):
    print('=' * 74)
    print(f'  量參數  ({len(d["t"])} 筆, {d["t"][-1] - d["t"][0]:.1f} 秒, '
          f'{np.median(np.diff(d["t"])) * 1e3:.0f} ms/筆)')
    print('=' * 74)

    mov = np.abs(d['vf']) > 0.05
    # 「乾淨行駛」= 有在動、四輪沒有明顯互相矛盾、沒有在急轉
    clean = (np.abs(d['vf']) > 0.1) & (d['spread'] < 1.0) & (np.abs(d['wz_fd']) < 0.5)

    print('\n[1] 輪速尺度 (wheel_scale)')
    kmed = None
    for est, name in ((d['med'], 'median'), (d['mean'], 'mean')):
        k = float(np.sum(est[clean] * d['vf'][clean]) / np.sum(est[clean] ** 2))
        e = k * est[clean] - d['vf'][clean]
        if kmed is None:
            kmed, emed = k, e
        print(f'    {name:6s} n={clean.sum():4d}  有效輪半徑 {k:.4f} m '
              f'(幾何 {radius}, 差 {100 * (k / radius - 1):+.1f}%) '
              f'-> wheel_scale {k / radius:.3f}  |  殘差 std {e.std():.4f} '
              f'p95 {np.percentile(np.abs(e), 95):.4f} m/s')
    print('    尺度誤差是**系統性**的: 走 100 m 就是幾公尺, 停車不會讓它消失。')
    print()
    print('    **這個數字不要照抄, 它每一輪都不一樣。** 同一台車的四份資料:')
    print('      spin12.csv                  n= 312  scale 1.008  殘差 std 0.0097')
    print('      sim_data.csv (2026-09-09)   n=  95  scale 0.928  殘差 std 0.0494')
    print('      sim_data.backup (09-03)     n=1966  scale 0.927  殘差 std 0.0948')
    print('      sim_data.csv (較早的一份)     n=1129  scale 0.976  殘差 std 0.0646')
    print('    差到 8%。而且**殘差越小的那一份越接近幾何值** (spin12 殘差小一個')
    print('    數量級, 量出來就是 1.008)。這不是巧合:')
    print('      「乾淨段」的篩選 (spread<1) 擋不掉整側一起滑, 而殘留的打滑一定是')
    print('      **輪子比車快** -> v/w 偏小 -> 量出來的有效半徑被系統性地拉低。')
    print('    所以: 殘差大的那一份不是「這台車今天的尺度不一樣」, 是那一份資料')
    print('    的打滑沒篩乾淨。要量 scale 就錄一段**平順的直線**, 不要拿自旋/')
    print('    急煞的資料去量。')
    q = float(emed.std())
    if clean.sum() < 200 or q > 0.03:
        print(f'    !! 這一份的樣本 {clean.sum()} 筆 / 殘差 std {q:.4f} —— '
              '品質不足以定 scale。')
        print('       要 n > 200 而且殘差 std < 0.03 才算數。先錄一段平順的直線。')
    else:
        print(f'    這一份: n={clean.sum()} 殘差 std {q:.4f} -> '
              f'可以用 (wheel_scale {kmed / radius:.3f})。')
    print('\n    四輪取 median vs 取 mean (用上面量到的有效半徑):')
    k = float(np.sum(d['med'][clean] * d['vf'][clean]) / np.sum(d['med'][clean] ** 2))
    sets = (('乾淨行駛 (spread<1)', mov & (d['spread'] < 1)),
            ('一般行駛 (Reposition)', mov & np.array(['Reposition' in s
                                                      for s in d['scenario']])),
            ('蓄意打滑 (spread>2)', mov & (d['spread'] > 2)))
    for name, m in sets:
        if m.sum() < 20:
            continue
        em, ea = k * d['med'][m] - d['vf'][m], k * d['mean'][m] - d['vf'][m]
        print(f'      {name:22s} n={m.sum():5d}  '
              f'median std {em.std():.4f} p95 {np.percentile(np.abs(em), 95):.4f} | '
              f'mean std {ea.std():.4f} p95 {np.percentile(np.abs(ea), 95):.4f}')
    print('    median 的價值在**一般行駛**時 (單輪偶爾空轉, 中位數不受影響);')
    print('    蓄意打滑那一段兩者一樣, 因為那裡是整側一起滑 —— 任何統計量都救不了,')
    print('    只能靠 slip_spread 與卡方閘門把那些量測降權。')

    print('\n[2] 靜止門檻 (still_wheel) —— 輪速讓 ZUPT 不用猜')
    rest = (np.abs(d['vf']) < 0.005) & (np.abs(d['wz_fd']) < 0.01)
    print(f'    靜止 n={rest.sum()}: 各輪 |w| p95 {np.percentile(np.abs(d["W"][:, rest]), 95):.4f} '
          f'max {np.abs(d["W"][:, rest]).max():.4f} rad/s '
          f'(= {radius * np.percentile(np.abs(d["W"][:, rest]), 95):.4f} m/s)')
    for th in (0.2, 0.5, 1.0, 2.0):
        quiet = np.max(np.abs(d['W']), axis=0) < th
        bad = quiet & (np.abs(d['vf']) > 0.1)
        print(f'    still_wheel={th:.1f}: 判為靜止 {100 * quiet.mean():4.1f}%, '
              f'其中車子其實在動 (>0.1 m/s) 的只有 {100 * bad.mean():.2f}%')
    print('    對照: 純慣性的靜止偵測分不出「靜止」與「等速直線」, 要靠加速度')
    print('    變異數去猜 (car_loc_imu 的 still_var)。輪子在轉就是在走, 沒這問題。')

    print('\n[3] 打滑指標 (slip_spread) —— 同側前後輪的轉速差')
    print('    同一側前後輪沒有差速器, 幾何上必須同速; 不同速就是有一顆在滑。')
    print('    這是輪速資料**自己內部的矛盾**, 不需要 ground truth 也不需要濾波器狀態。')
    scale = 0.0732 / radius
    err = scale * radius * d['med'] - d['vf']
    err_mean = scale * radius * d['mean'] - d['vf']
    for lo, hi in ((0, 0.5), (0.5, 2), (2, 5), (5, 1e9)):
        m = mov & (d['spread'] >= lo) & (d['spread'] < hi)
        if m.sum() < 10:
            continue
        print(f'    同側差 {lo:4.1f}-{hi if hi < 1e8 else float("inf"):>4.1f} rad/s: '
              f'n={m.sum():4d}  |輪速誤差| med {np.median(np.abs(err[m])):.3f} '
              f'p95 {np.percentile(np.abs(err[m]), 95):.3f} '
              f'max {np.abs(err[m]).max():.2f} m/s'
              f'   (改用平均: p95 {np.percentile(np.abs(err_mean[m]), 95):.3f})')

    print('\n[4] yaw 要從哪裡來 —— 這台車不能用輪速差')
    print(f'    r*(w_R - w_L)/track 跟真實角速度的相關係數: '
          f'{np.corrcoef((radius * ((d["W"][1] + d["W"][3]) / 2 - (d["W"][0] + d["W"][2]) / 2))[mov], d["wz_fd"][mov])[0, 1]:.3f}')
    m3 = mov & (np.abs(d['wz_fd']) < 3)
    kk = float(np.sum((radius * ((d['W'][1] + d['W'][3]) / 2
                                 - (d['W'][0] + d['W'][2]) / 2))[m3] * d['wz_fd'][m3])
               / np.sum(d['wz_fd'][m3] ** 2))
    print(f'    最小平方法反推的「有效輪距」: {kk:.3f} m (幾何輪距 0.25 m)')
    print(f'    /odom 的角速度 vs ground truth yaw 微分: '
          f'{np.corrcoef(d["wz"], d["wz_fd"])[0, 1]:.4f}  <- 陀螺儀這條路才是對的')

    print('\n[5] 側向速度 —— NHC 在這台車上幾乎是恆等式')
    print(f'    乾淨段 (n={clean.sum()}): 中位 {np.median(np.abs(d["vlat"][clean])):.4f} '
          f'p95 {np.percentile(np.abs(d["vlat"][clean]), 95):.4f} '
          f'max {np.abs(d["vlat"][clean]).max():.4f} m/s')
    sp = mov & (np.abs(d['wz_fd']) > 3)
    if sp.sum() > 10:
        print(f'    高速自旋 |wz|>3 (n={sp.sum()}): p95 '
              f'{np.percentile(np.abs(d["vlat"][sp]), 95):.3f} m/s '
              '<- sigma_cross 要蓋得住這個')
    print('    所以這裡把「側向速度 = 0」寫進狀態 (少一個自由度), 不是當偽量測。')

    print('\n[6] 真實加速度的量級 —— 沒有加速度計時 sigma_acc 要給多大')
    af = np.gradient(d['vf']) / np.gradient(d['t'])
    print(f'    |a_fwd| 中位 {np.median(np.abs(af[mov])):.2f} '
          f'p95 {np.percentile(np.abs(af[mov]), 95):.2f} '
          f'max {np.abs(af[mov]).max():.1f} m/s^2')
    print('=' * 74)


# ---------------------------------------------------------------- 重跑
def replay(d, *, wheel=True, slip_gate=True, zupt=True, zaru=True,
           yaw_from='orientation', wheel_scale=0.93, radius=0.075,
           sigma_acc=1.5, gyro_bias=0.0, seed=0, track=0.25, **kw):
    """把 CSV 一筆一筆餵進 WheelIns。

    `yaw_from` —— yaw 從哪裡來, 這是整張表差異最大的一項:

        orientation  用 ground truth 的 yaw 當**絕對量測** (`ins.update_yaw`)。
                     這是在模擬 Isaac 的 `IsaacReadIMU`: 它的 `orientation` 給的
                     就是模擬器的精確姿態, 節點預設 (`yaw_source:=imu_orientation`)
                     吃的正是這個。**真車的 6 軸 IMU 沒有這個東西。**
        gyro         只積分角速度 (真車的 6 軸 IMU 走這條)。
        wheels       用左右輪速差算角速度 (**故意做錯給你看**)。
        truth        直接把狀態的 yaw 設成真值 —— 完全沒有 yaw 誤差的下界。

    陀螺儀讀數用 `/odom` 的角速度 (跟 ground truth yaw 微分的相關係數 0.997)。

    > **20 Hz 的 CSV 對 `gyro` 那一列不公平。** 這份資料 50 ms 一筆, 而裡面有
    > 12 次高速自旋 (最高 14.5 rad/s = **每筆轉 42 度**)。一階保持的離散化誤差
    > 是 `|w|*dt/2`, 光是這一項 228 秒就累積 131 度 —— 那是取樣率造成的, 不是
    > 演算法。真的節點吃 60 Hz 的 `/imu`, 而且預設用絕對 yaw, 不會這樣。
    """
    rng = np.random.default_rng(seed)
    ins = WheelIns(forward_deg=-90.0, wheel_scale=wheel_scale,
                   sigma_acc=sigma_acc, sigma_k=0.0, **kw)
    det = WheelStillDetector()
    t, W = d['t'], d['W']
    ins.set_pose(d['px'][0], d['py'][0], d['yaw'][0], t[0])

    errs, yerrs = [], []
    gt_dist = 0.0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 0.5:
            continue
        gt_dist += math.hypot(d['px'][i] - d['px'][i - 1], d['py'][i] - d['py'][i - 1])
        if yaw_from == 'wheels':
            w = radius * ((W[1, i] + W[3, i]) / 2 - (W[0, i] + W[2, i]) / 2) / track
        else:
            w = d['wz'][i] + gyro_bias

        # CSV 沒有原始加速度 -> a_fwd = 0, 靠 sigma_acc 蓋住 (見模組說明)
        ins.predict(t[i], 0.0, w)
        if yaw_from == 'orientation':
            ins.update_yaw(d['yaw'][i], 0.02, dt)
        elif yaw_from == 'truth':
            ins.x[3] = d['yaw'][i]

        det.add(t[i], W[:, i], w)
        still = det.is_still()
        if still and zupt:
            if ins.still_since is None:
                ins.still_since = t[i]
            ins.zupt()
            if zaru:
                ins.zaru(w)
            if t[i] - ins.still_since >= ins.anchor_after:
                ins.anchor_position()
        else:
            ins.still_since = None
            ins.anchor = None

        if wheel:
            ins.update_wheel(radius * float(np.median(W[:, i])),
                             d['spread'][i] if slip_gate else 0.0)

        errs.append(math.hypot(ins.x[0] - d['px'][i], ins.x[1] - d['py'][i]))
        yerrs.append(abs(math.degrees(math.atan2(
            math.sin(ins.yaw - d['yaw'][i]), math.cos(ins.yaw - d['yaw'][i])))))
    e, ye = np.array(errs), np.array(yerrs)
    return {'final': e[-1], 'rms': float(np.sqrt((e ** 2).mean())), 'max': e.max(),
            'curve': e, 'gt_dist': gt_dist, 'yaw': ye[-1], 'yaw_max': ye.max(),
            'counts': dict(ins.counts), 'drift': 100 * e.max() / max(gt_dist, 1e-6)}


def row(name, r):
    c = r['counts']
    print(f'  {name:28s} {r["rms"]:7.2f} {r["max"]:7.2f} {r["final"]:7.2f} '
          f'{r["drift"]:6.1f}% {r["yaw"]:7.1f} {r["yaw_max"]:7.1f}   '
          f'輪速{c["wheel"]:5d} 放寬{c["slip"]:4d} 擋{c["rejected"]:4d} '
          f'ZUPT{c["zupt"]:5d}')


def header():
    print(f'  {"":28s} {"RMS":>7} {"最大":>7} {"終點":>7} {"漂移率":>7} '
          f'{"yaw°":>7} {"yawmax":>7}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', '..', 'car_run_data', 'sim_data.csv'))
    ap.add_argument('--measure', action='store_true', help='只量參數, 不重跑')
    ap.add_argument('--radius', type=float, default=0.075)
    ap.add_argument('--scale', type=float, default=0.93)
    ap.add_argument('--sigma-acc', type=float, default=1.5)
    ap.add_argument('--no-slip-gate', action='store_true')
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        raise SystemExit(f'找不到 {args.csv}')
    d = load(args.csv)
    if args.measure:
        measure(d, args.radius)
        return

    base = dict(radius=args.radius, wheel_scale=args.scale,
                sigma_acc=args.sigma_acc)
    gt = float(np.sum(np.hypot(np.diff(d['px']), np.diff(d['py']))))
    print('=' * 108)
    print(f'  {os.path.basename(args.csv)}: {len(d["t"])} 筆, '
          f'{d["t"][-1] - d["t"][0]:.1f} 秒, ground truth 走了 {gt:.1f} m')
    print('  這份資料是為了量**打滑與滑行**錄的 —— 12 次高速自旋 (最高 14.5 rad/s)、')
    print('  急煞、蓄意打滑, 一般行駛只占一部分。所以絕對數字是悲觀的上界,')
    print('  **A/B 的差距才是重點** (兩邊吃的是同一份資料)。')
    print('  漂移率 = 最大誤差 / ground truth 走過的距離。')
    print('=' * 108)

    print('\n[A] 預設設定 (yaw 用 IMU 的絕對姿態 —— Isaac / 9 軸 IMU)')
    header()
    full = replay(d, **base, slip_gate=not args.no_slip_gate)
    row('全開 (預設)', full)
    row('不擋打滑 (slip_gate 關)', replay(d, **base, slip_gate=False))
    row('沒有 ZUPT', replay(d, **base, zupt=False))
    row('scale = 1.0 (幾何值/滑行工作點)', replay(d, **{**base, 'wheel_scale': 1.0}))
    row('yaw 直接給真值 (下界)', replay(d, **base, yaw_from='truth'))

    print('\n[B] yaw 只能靠積分 (真車的 6 軸 IMU)')
    print('    注意: 20 Hz 的 CSV 對這一組不公平 —— 12 次高速自旋每筆轉到 42 度,')
    print('    光是離散化誤差 228 秒就累積 131 度。真的節點吃 60 Hz。')
    header()
    row('yaw 用陀螺儀積分', replay(d, **base, yaw_from='gyro'))
    row('  + 人工零偏 2 mrad/s', replay(d, **base, yaw_from='gyro', gyro_bias=0.002))
    row('  + 零偏但關掉 ZARU', replay(d, **base, yaw_from='gyro',
                                      gyro_bias=0.002, zaru=False))
    row('yaw 用左右輪速差', replay(d, **base, yaw_from='wheels'))

    print('\n[C] 誤差怎麼長 (預設設定) —— 階梯狀代表 ZUPT 真的有在動')
    e = full['curve']
    for f in (0.1, 0.25, 0.5, 0.75, 1.0):
        i = min(int(len(e) * f), len(e) - 1)
        print(f'    第 {d["t"][i] - d["t"][0]:6.1f} 秒: 誤差 {e[i]:6.3f} m')

    print('\n  怎麼讀這張表:')
    print('   * 「不擋打滑」與「全開」的差距 = 同側前後輪轉速差那個指標值多少。')
    print('   * 「yaw 用左右輪速差」是這個 package 最重要的一個否定結論:')
    print('     skid-steer 不能用輪速算 yaw, 要用陀螺儀。')
    print('   * 「yaw 直接給真值」是把 yaw 誤差整個拿掉的下界 —— 它跟「全開」的')
    print('     差距就是 yaw 貢獻了多少誤差。長期而言 yaw 是主導項, 不是輪速。')
    print('   * 「+ 零偏但關掉 ZARU」是開機靜止校正 + ZARU 的價值。')
    print('   * CSV 沒有原始加速度, 所以這裡量不到「加速度計在打滑時撐住估計」')
    print('     那一路 —— 那個由 test/test_wheel_ins.py 的合成資料驗證。')
    print('=' * 108)


if __name__ == '__main__':
    main()
