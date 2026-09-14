#!/usr/bin/env python3
"""一次開所有感測器的那一輪 CSV -> 各來源的誤差 / 誤差隨時間變化 + 圖。

    ./scripts/report_all_loc.py [car_run_data/sim_data.csv] [--out car_run_data/report_all]

時間對齊: 每一個來源的值都拿**它自己的 _stamp** 去內插 ground truth, 量到的是
「那一刻估得準不準」(本質誤差)。另外也算「記錄當下直接比」(= 下游拿來用時真正
看到的誤差, 含延遲), 兩者差就是延遲造成的部分。相機的時戳跟 /odom 差一個常數
時鐘偏移 (~1432 s), 先扣掉中位數再內插。
"""
import argparse
import json
import os

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = {'cam': 'Camera (YOLO)', 'lid': 'LiDAR', 'whl': 'IMU + wheel', 'imu': 'IMU only'}
COL = {'cam': '#2a78d6', 'lid': '#eb6834', 'whl': '#eda100', 'imu': '#1baf7a'}
# 畫圖的順序與單位 (IMU 會漂到公尺等級, 其他都是公分)
PLOT_ORDER = ('cam', 'lid', 'whl', 'imu')
UNIT = {'cam': ('cm', 100), 'lid': ('cm', 100), 'whl': ('cm', 100), 'imu': ('m', 1)}
INK, INK2, MUTED, GRID = '#0b0b0b', '#52514e', '#898781', '#e1e0d9'
SEG_COL = {'B1': '#f0efec', 'B2': '#e4e3de', 'B3': '#f0efec', 'B4': '#e4e3de'}


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def style(ax):
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color('#c3c2b7')
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)


def block(name):
    for b in ('B1', 'B2', 'B3', 'B4'):
        if b in name:
            return b
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?',
                    default=os.path.join(REPO, 'car_run_data', 'sim_data.csv'))
    ap.add_argument('--out', default=os.path.join(REPO, 'car_run_data', 'report_all'))
    ap.add_argument('--sigma', type=float, default=0.0025)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    df = pd.read_csv(a.csv)
    t0 = df.odom_stamp.iloc[0]
    df['t'] = df.odom_stamp - t0
    df['spd'] = np.hypot(df.car_linear_velocity_x, df.car_linear_velocity_y)
    df['w'] = df.car_angular_velocity_z.abs()
    df['block'] = df.scenario_name.map(block)
    # 區段: B1..B4 measure/brake 算在區塊裡, reposition 單獨一類
    df['seg'] = np.where(df.phase == 'reposition', 'reposition',
                         df.block.fillna('rest/idle'))

    # ground truth 時間序列 (odom_stamp 有重複列 -> 取唯一)
    gt = df.drop_duplicates('odom_stamp').sort_values('odom_stamp')
    gts = gt.odom_stamp.values
    gx, gy = gt.car_position_x.values, gt.car_position_y.values
    gyaw = np.unwrap(gt.gt_yaw.values)

    def gt_at(s):
        return (np.interp(s, gts, gx), np.interp(s, gts, gy),
                np.interp(s, gts, gyaw))

    # 走過的距離 (GT)
    d = np.r_[0, np.cumsum(np.hypot(np.diff(df.car_position_x),
                                    np.diff(df.car_position_y)))]
    df['dist'] = d

    res = {}
    live = [p for p in ('cam', 'lid', 'imu', 'whl', 'fus')
            if p + '_x' in df and df[p + '_x'].notna().any()]
    offline = [p for p in ('whl', 'fus') if p not in live]

    for p in live:
        off = float((df[p + '_stamp'] - df.odom_stamp).median())
        clk = off if abs(off) > 1.0 else 0.0          # 時鐘基準偏移, 不是延遲
        st = df[p + '_stamp'] - clk
        x, y, yw = gt_at(st.values)
        # 本質誤差 (用來源自己的時戳對齊)
        df[p + '_e'] = np.hypot(df[p + '_x'] - x, df[p + '_y'] - y)
        df[p + '_ey'] = np.degrees(wrap(df[p + '_yaw'] - yw))
        # 下游看到的誤差 (記錄當下直接比)
        df[p + '_eraw'] = np.hypot(df[p + '_x'] - df.car_position_x,
                                   df[p + '_y'] - df.car_position_y)
        df[p + '_ex'] = df[p + '_x'] - x
        df[p + '_eyy'] = df[p + '_y'] - y

        # 延遲掃描: 位置誤差最小的時間位移 (相機時鐘偏移扣掉之後殘下的延遲)
        lags = np.arange(-0.30, 0.301, 0.005)
        best = []
        mv = df.spd > 0.5
        for L in lags:
            xl, yl, _ = gt_at(st.values - L)
            e = np.hypot(df[p + '_x'] - xl, df[p + '_y'] - yl)
            best.append(np.median(e[mv]) if mv.any() else np.nan)
        best = np.array(best)
        lag = float(lags[np.nanargmin(best)])

        res[p] = dict(clock_offset=clk, stamp_minus_odom=off,
                      lag_best=lag, lag_curve=(lags.tolist(), best.tolist()))

    def summ(e):
        e = e.dropna()
        if not len(e):
            return None
        return dict(n=int(len(e)), med=100 * e.median(),
                    rms=100 * np.sqrt((e ** 2).mean()),
                    p90=100 * e.quantile(.9), p95=100 * e.quantile(.95),
                    max=100 * e.max())

    def ysumm(e):
        e = e.dropna()
        if not len(e):
            return None
        return dict(n=int(len(e)), med_abs=float(e.abs().median()),
                    rms=float(np.sqrt((e ** 2).mean())),
                    p90=float(e.abs().quantile(.9)), max=float(e.abs().max()))

    # 只開某幾條的時候 (例如只跑 LiDAR), 沒開的那幾條不能拿來算 —— 用全 False 佔位
    no = pd.Series(False, index=df.index)
    lid_lost = (df.lid_ey.abs() > 90) if 'lid' in live else no
    lid_ok = (df.lid_sigma < a.sigma) if 'lid' in live else no
    cam_yaw_valid = df.spd > 0.5
    panels = [p for p in PLOT_ORDER if p in live]

    out = {'n_rows': len(df), 'duration_s': float(df.t.max()),
           'gt_distance_m': float(d[-1]), 'offline': offline,
           'speed_max': float(df.spd.max()), 'w_max': float(df.w.max())}
    for p in live:
        r = res[p]
        r['pos'] = summ(df[p + '_e'])
        r['pos_raw'] = summ(df[p + '_eraw'])
        r['yaw'] = ysumm(df[p + '_ey'])
        r['bias_x_cm'] = 100 * float(df[p + '_ex'].median())
        r['bias_y_cm'] = 100 * float(df[p + '_eyy'].median())
        r['by_seg'] = {}
        for sg, s in df.groupby('seg'):
            r['by_seg'][sg] = dict(pos=summ(s[p + '_e']), yaw=ysumm(s[p + '_ey']),
                                   t=[float(s.t.min()), float(s.t.max())])
        r['by_speed'] = {}
        for lo, hi in ((0, .15), (.15, .5), (.5, 1.5), (1.5, 2.5), (2.5, 9)):
            s = df[(df.spd >= lo) & (df.spd < hi)]
            if len(s) >= 20:
                r['by_speed'][f'{lo}-{hi}'] = dict(pos=summ(s[p + '_e']),
                                                   yaw=ysumm(s[p + '_ey']))
        r['by_w'] = {}
        for lo, hi in ((0, 1), (1, 3), (3, 5), (5, 8), (8, 99)):
            s = df[(df.w >= lo) & (df.w < hi)]
            if len(s) >= 5:
                r['by_w'][f'{lo}-{hi}'] = dict(pos=summ(s[p + '_e']),
                                               yaw=ysumm(s[p + '_ey']),
                                               lost=100 * float((s[p + '_ey'].abs() > 90).mean()))
        # 每 10 s 一格的誤差演變
        bins = np.arange(0, df.t.max() + 10, 10)
        r['by_time'] = []
        for lo in bins[:-1]:
            s = df[(df.t >= lo) & (df.t < lo + 10)]
            r['by_time'].append(dict(t=float(lo), med=100 * float(s[p + '_e'].median()),
                                     p90=100 * float(s[p + '_e'].quantile(.9)),
                                     yaw_med=float(s[p + '_ey'].abs().median())))
        out[p] = r

    if 'cam' in live:
        out['cam']['yaw_moving'] = ysumm(df.cam_ey[cam_yaw_valid])
    if 'lid' in live:
        out['lid']['lost_pct'] = 100 * float(lid_lost.mean())
        out['lid']['gated'] = dict(keep_pct=100 * float(lid_ok.mean()),
                                   pos=summ(df.lid_e[lid_ok]), yaw=ysumm(df.lid_ey[lid_ok]),
                                   leak=int((lid_ok & lid_lost).sum()))
        out['lid']['not_lost'] = dict(pos=summ(df.lid_e[~lid_lost]),
                                      yaw=ysumm(df.lid_ey[~lid_lost]))
        # LiDAR 追丟事件: 連續 |yaw 誤差|>90 的片段, 記起訖與是否回復
        ev = []
        g = (lid_lost != lid_lost.shift()).cumsum()
        for _, s in df[lid_lost].groupby(g[lid_lost]):
            ev.append(dict(t0=float(s.t.iloc[0]), t1=float(s.t.iloc[-1]),
                           dur=float(s.t.iloc[-1] - s.t.iloc[0]), n=len(s),
                           seg=s.seg.mode().iloc[0], wmax=float(s.w.max()),
                           pos_med=100 * float(s.lid_e.median())))
        out['lid']['lost_events'] = ev
    if 'imu' in live:
        # IMU 漂移: 誤差 vs 時間 / 距離
        im = out['imu']
        for T in (10, 30, 60, 90, 120, 150, float(df.t.max())):
            i = int(np.argmin(np.abs(df.t.values - T)))
            im.setdefault('at', []).append(dict(t=float(df.t.iloc[i]), dist=float(d[i]),
                                                e=float(df.imu_e.iloc[i]),
                                                yaw=float(df.imu_ey.iloc[i])))
        # 每個區段內 IMU 誤差的**增量** (哪裡在長)
        im['growth'] = []
        g2 = (df.seg != df.seg.shift()).cumsum()
        for _, s in df.groupby(g2):
            im['growth'].append(dict(seg=s.seg.iloc[0], name=s.scenario_name.iloc[0],
                                     t0=float(s.t.iloc[0]), t1=float(s.t.iloc[-1]),
                                     de=float(s.imu_e.iloc[-1] - s.imu_e.iloc[0]),
                                     dd=float(s.dist.iloc[-1] - s.dist.iloc[0])))

    with open(os.path.join(a.out, 'summary.json'), 'w') as f:
        json.dump(out, f, indent=1, ensure_ascii=False, default=float)

    # ------------------------------------------------------------------ plots
    plt.rcParams.update({'font.size': 9, 'axes.titlesize': 10,
                         'axes.titleweight': 'bold', 'text.color': INK,
                         'axes.labelcolor': INK2})

    def shade(ax, label=False):
        g = (df.block != df.block.shift()).cumsum()
        for _, s in df[df.block.notna()].groupby(g[df.block.notna()]):
            b = s.block.iloc[0]
            ax.axvspan(s.t.iloc[0], s.t.iloc[-1], color=SEG_COL[b], lw=0, zorder=0)
        if label:
            for b, (lo, hi) in {'B1 spin in place': (0, 55), 'B2 creep+spin': (55, 73),
                                'B3 straight sprints': (73, 145),
                                'B4 slip launch': (145, 179)}.items():
                ax.text((lo + hi) / 2, 1.02, b, transform=ax.get_xaxis_transform(),
                        ha='center', va='bottom', fontsize=8, color=INK2)

    def roll(s, n=21):
        return s.rolling(n, center=True, min_periods=3).median()

    # 1) position error vs time, one panel per source (scales differ by 100x)
    fig, axs = plt.subplots(len(panels), 1, figsize=(11, 0.6 + 2.55 * len(panels)),
                            sharex=True, squeeze=False)
    axs = axs[:, 0]
    for ax, p in zip(axs, panels):
        unit, k = UNIT[p]
        first = p == panels[0]
        shade(ax, label=first)
        ax.plot(df.t, df[p + '_e'] * k, color=COL[p], lw=0.6, alpha=0.35)
        ax.plot(df.t, roll(df[p + '_e']) * k, color=COL[p], lw=1.8,
                label='1 s rolling median')
        if p == 'lid':
            ax.scatter(df.t[lid_lost], (df.lid_e * k)[lid_lost], s=6,
                       color='#d03b3b', zorder=3, label='yaw flipped (>90°)')
        ax.set_ylabel(f'position error ({unit})')
        ax.set_title(SRC[p], loc='left', color=INK, pad=14 if first else 4)
        style(ax)
        ax.legend(loc='upper left', fontsize=7.5, frameon=False)
    axs[-1].set_xlabel('time (s, sim)')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'err_pos_vs_time.png'), dpi=140)
    plt.close(fig)

    # 2) yaw error vs time
    fig, axs = plt.subplots(len(panels), 1, figsize=(11, 0.6 + 2.2 * len(panels)),
                            sharex=True, squeeze=False)
    axs = axs[:, 0]
    for ax, p in zip(axs, panels):
        first = p == panels[0]
        shade(ax, label=first)
        ey = df[p + '_ey']
        if p == 'cam':
            ax.scatter(df.t[~cam_yaw_valid], ey[~cam_yaw_valid], s=3, color=MUTED,
                       alpha=0.4, label='speed < 0.5 m/s (yaw from heading, undefined)')
            ax.scatter(df.t[cam_yaw_valid], ey[cam_yaw_valid], s=4, color=COL[p],
                       label='speed > 0.5 m/s')
        else:
            ax.plot(df.t, ey, color=COL[p], lw=0.9)
        ax.set_ylabel('yaw error (deg)')
        ax.set_ylim(-190, 190) if p in ('cam', 'lid') else None
        ax.set_title(SRC[p], loc='left', pad=14 if first else 4)
        style(ax)
        if p == 'cam':
            ax.legend(loc='lower left', fontsize=7.5, frameon=False)
    axs[-1].set_xlabel('time (s, sim)')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'err_yaw_vs_time.png'), dpi=140)
    plt.close(fig)

    # 3) IMU drift vs distance travelled & vs time
    if 'imu' in live:
        fig, axs = plt.subplots(1, 2, figsize=(11, 3.8))
        ax = axs[0]
        shade(ax)
        ax.plot(df.t, df.imu_e, color=COL['imu'], lw=1.6, label='IMU position error')
        ax.plot(df.t, df.dist * 0.1, color=MUTED, lw=1, ls='--',
                label='10% of distance travelled')
        ax.set_xlabel('time (s)'); ax.set_ylabel('m'); style(ax)
        ax.set_title('IMU error vs time', loc='left')
        ax.legend(frameon=False, fontsize=8)
        ax = axs[1]
        ax.plot(df.dist, df.imu_e, color=COL['imu'], lw=1.6)
        ax.set_xlabel('distance travelled by GT (m)'); ax.set_ylabel('position error (m)')
        ax.set_title('IMU error vs distance', loc='left'); style(ax)
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, 'imu_drift.png'), dpi=140)
        plt.close(fig)

    # 4) error vs speed / angular rate (公分等級的那幾條)
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.8))
    for p in [q for q in panels if UNIT[q][0] == 'cm']:
        axs[0].scatter(df.spd, df[p + '_e'] * 100, s=4, alpha=0.35,
                       color=COL[p], label=SRC[p])
        axs[1].scatter(df.w, df[p + '_e'] * 100, s=4, alpha=0.35,
                       color=COL[p], label=SRC[p])
    axs[0].set_xlabel('GT speed (m/s)'); axs[1].set_xlabel('|yaw rate| (rad/s)')
    for ax in axs:
        ax.set_ylabel('position error (cm)'); ax.set_ylim(0, 60); style(ax)
        ax.legend(frameon=False, fontsize=8, markerscale=3)
    axs[0].set_title('Position error vs speed', loc='left')
    axs[1].set_title('Position error vs spin rate', loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'err_vs_motion.png'), dpi=140)
    plt.close(fig)

    # 5) trajectories
    fig, ax = plt.subplots(figsize=(11, 3.9))
    ax.plot(df.car_position_x, df.car_position_y, color=INK, lw=2, label='ground truth')
    for p in panels:
        ax.plot(df[p + '_x'], df[p + '_y'], color=COL[p], lw=0.9, label=SRC[p])
    ax.set_aspect('equal'); style(ax); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.legend(frameon=False, fontsize=8, ncol=4, loc='lower right',
              bbox_to_anchor=(1, 1.0))
    ax.set_title('Trajectories (world frame)', loc='left', pad=8)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'traj.png'), dpi=140)
    plt.close(fig)

    # 6) CDF (cam, lidar all / lidar gated)
    fig, ax = plt.subplots(figsize=(7, 3.8))
    curves = []
    if 'cam' in live:
        curves.append(('Camera', df.cam_e, COL['cam'], '-'))
    if 'lid' in live:
        curves += [('LiDAR (all)', df.lid_e, COL['lid'], '-'),
                   (f'LiDAR (sigma<{a.sigma})', df.lid_e[lid_ok], COL['lid'], '--')]
    if 'whl' in live:
        curves.append(('IMU + wheel', df.whl_e, COL['whl'], '-'))
    for lab, e, c, ls in curves:
        v = np.sort(e.dropna().values) * 100
        ax.plot(v, np.arange(1, len(v) + 1) / len(v), color=c, ls=ls, lw=1.8, label=lab)
    ax.set_xscale('log'); ax.set_xlabel('position error (cm, log)'); ax.set_ylabel('fraction of frames')
    style(ax); ax.legend(frameon=False, fontsize=8)
    ax.set_title('Error CDF', loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'err_cdf.png'), dpi=140)
    plt.close(fig)

    print(json.dumps({k: (v if not isinstance(v, dict) else
                          {kk: vv for kk, vv in v.items() if kk != 'lag_curve'})
                      for k, v in out.items()}, indent=1, ensure_ascii=False, default=float))


if __name__ == '__main__':
    main()
