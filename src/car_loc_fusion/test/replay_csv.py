#!/usr/bin/env python3
"""拿 `collect_data_node` 錄的 CSV 重放融合 —— 不需要 ROS / Isaac。

    python3 test/replay_csv.py ../../car_run_data/sim_data.csv

`test_fusion.py` 跑的是合成資料 (誤差模型是我們自己放的), 這支跑的是**真的
Isaac 資料**: 相機與 LiDAR 的 pose 就是那一輪真的發出來的那些, 連時戳都原封不動
—— 時鐘偏移、追丟、鎖 180 度全部都在裡面。

**這支能回答的問題只有一個: 同一份輸入, 換一組設定會差多少。**

它跟真的節點有三個差別, 所以**絕對數字是悲觀的上界**:

| | 真的節點 | 這裡 |
| --- | --- | --- |
| 遞推頻率 | /imu 60~200 Hz | CSV 20 Hz |
| 前進加速度 | IMU 扣完重力 | **沒有** (CSV 沒錄原始加速度) -> `a_fwd = 0`, 靠 `sigma_acc` 蓋住 |
| 角速度 | 陀螺儀 | ground truth 的角速度 (CSV 只有這個) |

`a_fwd = 0` 表示遞推完全靠輪速撐, 加速的那一瞬間會落後 —— 真的節點有加速度計。
`sigma_acc` 因此要給大 (預設 1.5), 這是 `car_loc_wheel/test/replay_csv.py` 同一個
取捨, 理由那邊寫得更詳細。
"""
from __future__ import annotations

import csv
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, '..', '..', 'car_loc_wheel')))

from car_loc_fusion.fusion_ekf import AbsMeas, FusionEkf, Step      # noqa: E402
from car_loc_fusion.sources import AbsSource                        # noqa: E402
from car_loc_wheel.wheel_ins import WheelStillDetector              # noqa: E402

WHEEL_COLS = ('front_left_velocity', 'front_right_velocity',
              'rear_left_velocity', 'rear_right_velocity')
RADIUS = 0.075
FORWARD_DEG = -90.0


def load(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise SystemExit('CSV 是空的')

    def col(n):
        return np.array([float(r[n]) if r.get(n) not in (None, '', 'nan') else np.nan
                         for r in rows])

    have = set(rows[0])
    for need in ('odom_stamp', 'car_position_x', 'gt_yaw'):
        if need not in have:
            raise SystemExit(f'CSV 缺少 {need} —— 這不是 collect_data_node 錄的格式')

    d = {'t': col('odom_stamp'), 'px': col('car_position_x'),
         'py': col('car_position_y'), 'yaw': col('gt_yaw'),
         'wz': col('car_angular_velocity_z'),
         'W': np.stack([col(c) for c in WHEEL_COLS])}
    keep = np.all(np.isfinite(np.stack(
        [d['t'], d['px'], d['py'], d['yaw'], d['wz']] + list(d['W']))), axis=0)
    for k in ('t', 'px', 'py', 'yaw', 'wz'):
        d[k] = d[k][keep]
    d['W'] = d['W'][:, keep]
    d['spread'] = np.maximum(np.abs(d['W'][0] - d['W'][2]),
                             np.abs(d['W'][1] - d['W'][3]))

    # --- 絕對量測: 一個時戳 = 一則訊息。CSV 是 20 Hz 的快照, 同一則會重複好幾列,
    #     所以要**依時戳去重**; 那一列的時刻就是「已經收到了」的時刻 (到達時間)。
    d['meas'] = []
    for pre, src in (('cam', 'camera'), ('lid', 'lidar')):
        if pre + '_stamp' not in have:
            continue
        st = col(pre + '_stamp'); x = col(pre + '_x'); y = col(pre + '_y')
        yw = col(pre + '_yaw'); sg = col(pre + '_sigma'); ag = col(pre + '_age')
        tt = col('odom_stamp')
        seen = set()
        for i in range(len(rows)):
            s = st[i]
            if not np.isfinite(s) or s in seen or not np.isfinite(x[i]):
                continue
            seen.add(s)
            d['meas'].append(dict(t=float(s), arrive=float(tt[i]), src=src,
                                  x=float(x[i]), y=float(y[i]),
                                  yaw=float(yw[i]) if np.isfinite(yw[i]) else None,
                                  reported=float(sg[i]) if np.isfinite(sg[i]) else 0.0,
                                  age=float(ag[i]) if np.isfinite(ag[i]) else 0.0))
    d['meas'].sort(key=lambda m: m['arrive'])
    n_cam = sum(1 for m in d['meas'] if m['src'] == 'camera')
    n_lid = len(d['meas']) - n_cam
    print(f'  {len(d["t"])} 列, {d["t"][-1] - d["t"][0]:.0f} 秒; '
          f'絕對量測 相機 {n_cam} 則, LiDAR {n_lid} 則')
    for pre, src in (('cam', 'camera'), ('lid', 'lidar')):
        o = [m['t'] - m['arrive'] for m in d['meas'] if m['src'] == src]
        if o:
            print(f'    {src:7s} 時戳 - odom_stamp: 中位 {np.median(o):+10.3f} s'
                  + ('   <-- **時鐘基準不一樣**' if abs(np.median(o)) > 1.0 else ''))
    return d


def replay(d, *, camera=True, lidar=True, rewind=True, clock_fix=True,
           lidar_sigma_max=0.0025, cam_extra_delay=0.0, sigma_acc=1.5,
           wheel_scale=0.93):
    cmo, cft = (1.0, 0.1) if clock_fix else (1e9, 1e9)
    src = {
        'camera': AbsSource('camera', enabled=camera, sigma_floor=0.03,
                            r_inflate=1.5, clock_max_offset=cmo,
                            clock_future_tol=cft, extra_delay=cam_extra_delay,
                            use_yaw=False),
        'lidar': AbsSource('lidar', enabled=lidar, sigma_floor=0.04,
                           r_inflate=1.5, sigma_max=lidar_sigma_max,
                           clock_max_offset=cmo, clock_future_tol=cft,
                           use_yaw=True, yaw_sigma=0.05),
    }
    f = FusionEkf(rewind=rewind, rewind_horizon=0.4, forward_deg=FORWARD_DEG,
                  wheel_scale=wheel_scale, sigma_acc=sigma_acc, sigma_k=0.0)
    det = WheelStillDetector()
    f.set_pose(d['px'][0], d['py'][0], d['yaw'][0], d['t'][0])

    t, W = d['t'], d['W']
    meas, mi = d['meas'], 0
    errs, yerrs, sig = [], [], []
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 0.5:
            continue
        w = float(d['wz'][i])
        det.add(t[i], W[:, i], w)
        # a_fwd = 0: CSV 沒有原始加速度, 遞推完全靠輪速 (見模組說明)
        f.step(Step(t[i], 0.0, w, v_wheel=RADIUS * float(np.median(W[:, i])),
                    spread=float(d['spread'][i]), still=det.is_still(),
                    yaw_meas=float(d['yaw'][i]), yaw_sigma=0.02, dt=dt))

        while mi < len(meas) and meas[mi]['arrive'] <= t[i]:
            mm = meas[mi]; mi += 1
            s = src[mm['src']]
            if not s.enabled:
                continue
            mt = s.to_filter_clock(mm['t'], f.ins.t)
            ok, _ = s.accept(mt, mm['reported'], t[i])
            if not ok:
                continue
            am = AbsMeas(mt, mm['src'], mm['x'], mm['y'], s.sigma_for(mm['reported']),
                         yaw=mm['yaw'], yaw_sigma=s.yaw_sigma)
            if f.absolute(am, use_yaw=s.use_yaw):
                s.mark_used(mt)
            else:
                s.counts['gate'] += 1

        errs.append(math.hypot(f.pos[0] - d['px'][i], f.pos[1] - d['py'][i]))
        yerrs.append(abs(math.degrees(math.atan2(
            math.sin(f.yaw - d['yaw'][i]), math.cos(f.yaw - d['yaw'][i])))))
        sig.append(f.sigma_pos())
    e = np.array(errs)
    return {'rms': float(np.sqrt((e ** 2).mean())), 'med': float(np.median(e)),
            'p95': float(np.percentile(e, 95)), 'max': float(e.max()),
            'yaw': float(np.median(yerrs)), 'sigma': float(np.median(sig)),
            'f': f, 'src': src, 'curve': e}


def baseline(d, which):
    """「只用這一條」: 20 Hz 去問它, 拿最新一則 (含延遲與空窗)。"""
    ms = [m for m in d['meas'] if m['src'] == which]
    errs, cur, j = [], None, 0
    for i in range(1, len(d['t'])):
        while j < len(ms) and ms[j]['arrive'] <= d['t'][i]:
            cur = ms[j]; j += 1
        if cur is None:
            continue
        errs.append(math.hypot(cur['x'] - d['px'][i], cur['y'] - d['py'][i]))
    e = np.array(errs) if errs else np.array([np.nan])
    return {'rms': float(np.sqrt((e ** 2).mean())), 'med': float(np.median(e)),
            'p95': float(np.percentile(e, 95)), 'max': float(e.max())}


def row(name, r, extra=''):
    print(f'  {name:34s} RMS {r["rms"] * 100:7.2f} cm  中位 {r["med"] * 100:6.2f}  '
          f'p95 {r["p95"] * 100:7.2f}  最大 {r["max"] * 100:7.2f}  {extra}')


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    path = argv[1]
    line = '=' * 108
    print(line)
    print(f'  {os.path.basename(path)}')
    d = load(path)
    print(line)

    print('\n[0] 只用單一來源 (20 Hz 拿最新一則, 含延遲與空窗)')
    for src, tag in (('camera', '只有相機'), ('lidar', '只有 LiDAR')):
        if any(m['src'] == src for m in d['meas']):
            row(tag, baseline(d, src))

    print('\n[1] 融合 —— 時鐘偵測開/關 (這是 2026-09-10 修掉的那個 bug)')
    on = replay(d, clock_fix=True)
    off = replay(d, clock_fix=False)
    row('有時鐘偵測 (現在的預設)', on,
        f'sigma {on["sigma"] * 1000:5.2f} mm  {on["f"].report()}')
    row('關掉 (= 修之前的行為)', off,
        f'sigma {off["sigma"] * 1000:5.2f} mm  {off["f"].report()}')
    if off['rms'] > 3 * on['rms']:
        print('     ** 時戳落在未來 -> 緩衝區清不掉 -> 整段歷史被重複套用 -> P 被壓垮')
        print('        -> 估計凍結。sigma 那一欄就是證據: 它「非常確定」自己停在原地。')

    print('\n[2] 少一個來源會怎樣')
    row('只有相機當絕對量測', replay(d, lidar=False))
    row('只有 LiDAR 當絕對量測', replay(d, camera=False))

    print('\n[3] LiDAR 的 sigma 閘門 (擋「鎖到 180 度」用的)')
    g_on = replay(d, lidar_sigma_max=0.0025)
    g_off = replay(d, lidar_sigma_max=0.0)
    row('有 (門檻 0.0025)', g_on,
        f'sigma擋 {g_on["src"]["lidar"].counts["sigma"]}')
    row('沒有', g_off, f'sigma擋 {g_off["src"]["lidar"].counts["sigma"]}')

    print('\n[4] 延遲補償')
    row('有倒帶重放', replay(d, rewind=True))
    row('沒有', replay(d, rewind=False))
    print('     時鐘偏掉的時候這兩列會**一樣** —— 因為真正的延遲已經沒辦法從資料裡')
    print('     分離出來了。要看得出差別, 得先去源頭把時鐘修好。')

    print('\n[5] 手動把量到的常數延遲加回去 (時鐘沒修好時的暫時解)')
    for ed in (0.0, 0.03, 0.05, 0.08):
        row(f'camera_extra_delay = {ed:.2f} s', replay(d, cam_extra_delay=ed))
    print('     掃出來的最小值就是該填的。**時鐘修好之後要設回 0** (不然扣兩次)。')

    print('\n' + line)
    print('  提醒: 這支的遞推是 20 Hz 而且沒有加速度計 (CSV 沒錄), 所以絕對數字')
    print('  是悲觀的上界。**要看的是每一組的差距, 不是單一個數字。**')
    print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
