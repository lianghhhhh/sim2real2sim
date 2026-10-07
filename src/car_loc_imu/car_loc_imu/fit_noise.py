#!/usr/bin/env python3
"""從一段**靜止**的 IMU 資料擬合出雜訊模型, 寫成濾波器的參數檔。

    # 1. 車子完全不動, 錄 IMU (越久越好, 建議 2 小時以上)
    ros2 bag record -s sqlite3 -o imu_static /imu

    # 2. 擬合 (不需要 ROS)
    ros2 run car_loc_imu imu_fit_noise imu_static --plot allan.png
    # 或
    python3 -m car_loc_imu.fit_noise imu_static --topic /imu/data --plot allan.png

    # 3. 用擬合出來的參數跑定位
    ros2 launch car_loc_imu imu_loc.launch.py noise_fit:=$PWD/imu_noise_fit.yaml

擬合的模型 (每一軸):

    讀數 = b0 + b_dyn(t) + n_w(t),   db_dyn/dt = -b_dyn/tau + n_b

    b0         整段的平均 (只是印出來看; 每次開機都不一樣, 濾波器用開機靜止
               校正自己量, 不用這裡的值)
    N          白雜訊密度
    sigma_gm   b_dyn 的穩態標準差   -> sigma_gm_bg / sigma_gm_ba
    tau        b_dyn 的相關時間     -> tau_bg / tau_ba

濾波器只用陀螺儀 z 軸與加速度計 x/y 軸, 所以擬合的也是這三軸 (x/y 的 Allan
variance 先平均再擬合)。

**要錄多久:** tau 要分辨得出來, 資料至少要有幾十個 tau。tau 通常是幾百秒,
所以要以小時計。錄太短的話只看得到零偏「正在漂」, 看不到它會飽和 —— 那時
這支程式會說 tau 分辨不出來, 並改給等效的隨機遊走參數。
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

if __package__ in (None, ''):            # 直接 python3 fit_noise.py 也能跑
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from car_loc_imu.allan import (allan_variance, fit_white_gm,     # noqa: E402
                               gm_shape, model_avar)
from car_loc_imu.imu_io import read_imu                           # noqa: E402


def _fit(series, fs, duration):
    """series: 一個或多個軸。多個軸的話 Allan variance 先平均。"""
    curves = [allan_variance(x, fs) for x in series]
    T = curves[0][0]
    avar = np.mean([c[1] for c in curves], axis=0)
    relerr = curves[0][2] / math.sqrt(len(curves))
    fit = fit_white_gm(T, avar, relerr, duration)
    return fit, (T, avar, relerr)


def _describe(name, unit, fit, duration):
    print(f'\n[{name}]')
    print(f'  白雜訊密度 N      {fit["n_white"]:.3e} {unit}/sqrt(Hz)')
    if not fit['has_gm']:
        print('  Gauss-Markov 項   沒有偵測到 —— Allan 曲線就是一條白雜訊的斜線。')
        print('                    (Isaac 原始的 IMU 就是這樣: 它沒有零偏模型。)')
        return
    print(f'  GM 穩態標準差     {fit["sigma_gm"]:.3e} {unit}')
    print(f'  GM 相關時間 tau   {fit["tau"]:.1f} s')
    print(f'  等效隨機遊走      {fit["rw_equiv"]:.3e} {unit}/sqrt(s)')
    if not fit['tau_resolved']:
        print(f'  ** tau 分辨不出來: 資料只有 {duration:.0f} 秒, 是 tau 的 '
              f'{duration / fit["tau"]:.0f} 倍 (至少要 20 倍)。')
        print('     這段資料只看得到零偏在漂, 看不到它飽和。上面的 tau / sigma '
              '只能當下限, 錄久一點再擬合。')


def _plot(path, panels):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('\n沒有 matplotlib, 跳過畫圖 (pip install matplotlib)')
        return
    ink, ink2, muted, grid = '#0b0b0b', '#52514e', '#898781', '#e1e0d9'
    c_data, c_fit = '#2a78d6', '#eb6834'
    fig, axes = plt.subplots(1, len(panels), figsize=(6.4 * len(panels), 4.6),
                             facecolor='#fcfcfb')
    for ax, (title, unit, fit, (T, avar, _)) in zip(np.atleast_1d(axes), panels):
        ax.set_facecolor('#fcfcfb')
        ax.loglog(T, np.sqrt(avar), 'o', ms=4, color=c_data, label='measured')
        Tf = np.logspace(math.log10(T[0]), math.log10(T[-1]), 300)
        ax.loglog(Tf, fit['n_white'] / np.sqrt(Tf), '--', lw=1, color=muted)
        ax.annotate('white noise', (Tf[0], fit['n_white'] / math.sqrt(Tf[0])),
                    xytext=(6, -12), textcoords='offset points', color=ink2, fontsize=9)
        if fit['has_gm']:
            gm = fit['sigma_gm'] * np.sqrt(gm_shape(Tf, fit['tau']))
            ax.loglog(Tf, gm, ':', lw=1.2, color=muted)
            k = int(np.argmax(gm))
            ax.annotate('Gauss-Markov', (Tf[k], gm[k]), xytext=(0, -14),
                        textcoords='offset points', ha='center', color=ink2, fontsize=9)
        ax.loglog(Tf, np.sqrt(model_avar(Tf, fit['n_white'], fit['sigma_gm'],
                                         fit['tau'])),
                  '-', lw=2, color=c_fit, label='fit: white + GM')
        sub = f'N = {fit["n_white"]:.2e} {unit}/√Hz'
        if fit['has_gm']:
            sub += f',  σ_GM = {fit["sigma_gm"]:.2e} {unit},  τ = {fit["tau"]:.0f} s'
        else:
            sub += ',  no GM term detected'
        ax.set_title(f'{title}\n{sub}', loc='left', color=ink, fontsize=10)
        ax.set_xlabel('averaging time T (s)', color=ink2)
        ax.set_ylabel(f'Allan deviation ({unit})', color=ink2)
        ax.grid(True, which='major', color=grid, lw=0.6)
        ax.tick_params(colors=muted, which='both')
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        for s in ('left', 'bottom'):
            ax.spines[s].set_color('#c3c2b7')
        ax.legend(frameon=False, labelcolor=ink2, loc='upper right')
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f'\nAllan 曲線圖: {path}')


def _still_thresholds(gyro, acc):
    """靜止偵測的門檻至少要多大, 這顆 IMU 靜止時才過得了。

    靜止偵測看的是**原始**讀數 (還沒扣零偏), 所以門檻要蓋得住開機零偏加
    每一筆的白雜訊。只會把門檻往上調 (不低於模擬用的預設值) —— 上限是行駛
    時的震動有多大, 那個從靜止資料看不出來, 要實際開車確認 status 印的
    「靜止 xx%」跟實際停車的比例對得上。
    """
    # 每一筆的白雜訊標準差: 用相鄰兩筆的差來算, 不會被慢慢漂的零偏影響
    sg = np.diff(gyro, axis=0).std(axis=0) / math.sqrt(2.0)
    sa = np.diff(acc, axis=0).std(axis=0) / math.sqrt(2.0)
    g_mean = float(np.linalg.norm(gyro.mean(axis=0)))
    a_off = abs(float(np.linalg.norm(acc, axis=1).mean()) - 9.80665)
    return dict(
        still_gyro=max(0.03, 1.5 * g_mean + 5.0 * float(np.linalg.norm(sg))),
        still_var=max(0.05, 1.6 * float(np.linalg.norm(sa))),
        still_acc=max(0.25, a_off + 0.15))


def _yaml(path, src, duration, fs, fg, fa, still):
    any_gm = fg['has_gm'] or fa['has_gm']
    lines = [
        '# IMU 零偏模型的參數 —— car_loc_imu/fit_noise.py 擬合出來的, 不要手改。',
        f'# 來源: {src}  ({duration:.0f} 秒, {fs:.1f} Hz)',
        '#',
        '# 量到的白雜訊密度 (感測器本身的底線, 僅供對照):',
        f'#   陀螺儀 z      {fg["n_white"]:.3e} rad/s/sqrt(Hz)',
        f'#   加速度計 x/y  {fa["n_white"]:.3e} m/s^2/sqrt(Hz)',
        '# imu_loc.yaml 的 sigma_gyro / sigma_acc 是過程雜訊, 除了感測器雜訊還要',
        '# 蓋住車體震動與模型誤差, 所以只能比上面的值大, 不能比它小。',
        'imu_localizer:',
        '  ros__parameters:',
        f'    bias_model: {"gm" if any_gm else "rw"}',
    ]
    for tag, name, fit in (('bg', '陀螺儀 z', fg), ('ba', '加速度計 x/y', fa)):
        if fit['has_gm']:
            if not fit['tau_resolved']:
                lines.append(f'    # {name}: tau 分辨不出來 (資料太短), 下面兩個值只是下限')
            lines.append(f'    tau_{tag}: {fit["tau"]:.1f}')
            lines.append(f'    sigma_gm_{tag}: {fit["sigma_gm"]:.4e}')
        else:
            lines.append(f'    # {name}: 沒有偵測到 Gauss-Markov 項, 沿用節點的預設值')
    lines += [
        '    # 靜止偵測的門檻: 這顆 IMU 靜止時的零偏與雜訊至少需要這麼寬。',
        '    # 上限要開車確認 (status 的「靜止 xx%」要跟實際停車比例對得上)。',
        f'    still_gyro: {still["still_gyro"]:.4f}',
        f'    still_var: {still["still_var"]:.4f}',
        f'    still_acc: {still["still_acc"]:.4f}',
    ]
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print(f'參數檔: {path}')


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='從靜止的 IMU 資料擬合白雜訊 + Gauss-Markov 零偏模型')
    ap.add_argument('data', help='rosbag2 目錄 (sqlite3) 或 CSV')
    ap.add_argument('--topic', default='/imu', help='bag 裡的 IMU topic')
    ap.add_argument('--out', default='imu_noise_fit.yaml', help='輸出的參數檔')
    ap.add_argument('--plot', default='', help='把 Allan 曲線存成圖 (例如 allan.png)')
    ap.add_argument('--force', action='store_true',
                    help='資料看起來不是靜止的也照算')
    ap.add_argument('--skip', type=float, default=0.0,
                    help='開頭丟掉幾秒 (剛放下車子、還在晃的那一段)')
    args = ap.parse_args(argv)

    t, _, gyro, acc = read_imu(args.data, args.topic)
    keep = t >= t[0] + args.skip
    t, gyro, acc = t[keep], gyro[keep], acc[keep]
    dts = np.diff(t)
    dts = dts[dts > 0]
    fs = 1.0 / float(np.median(dts))
    duration = len(t) / fs
    print(f'{len(t)} 筆, {duration:.0f} 秒 ({duration / 3600.0:.2f} 小時), {fs:.1f} Hz')
    gap = float(dts.max())
    if gap > 5.0 / fs:
        print(f'** 時戳有 {gap:.2f} 秒的斷點 —— Allan variance 假設等間隔取樣, '
              '掉包嚴重的話結果會偏。')

    # 這支程式假設車子完全沒動。車子有動的話, 動作本身會被當成零偏漂移。
    dev = np.abs(gyro - np.median(gyro, axis=0)).max(axis=1)
    moving = float((dev > 0.05).mean())
    if moving > 0.01:
        msg = (f'{100 * moving:.0f}% 的資料角速度偏離中位數超過 0.05 rad/s —— 這不是'
               '靜止的資料。車子的動作會被當成零偏漂移, 擬合結果不能用。')
        if not args.force:
            raise SystemExit(f'** {msg}\n   (確定要照算的話加 --force)')
        print(f'** {msg}')

    b0g = float(gyro[:, 2].mean())
    b0a = acc[:, :2].mean(axis=0)
    print(f'\n平均值 (這次開機的 b0): 陀螺儀 z {b0g:+.3e} rad/s, '
          f'加速度計 x/y ({b0a[0]:+.4f}, {b0a[1]:+.4f}) m/s^2')
    print('  (加速度的平均值包含車身傾斜漏進來的重力, 不全是零偏。)')

    fg, cg = _fit([gyro[:, 2]], fs, duration)
    fa, ca = _fit([acc[:, 0], acc[:, 1]], fs, duration)
    _describe('陀螺儀 z', 'rad/s', fg, duration)
    _describe('加速度計 x/y', 'm/s^2', fa, duration)

    still = _still_thresholds(gyro, acc)
    print()
    _yaml(args.out, args.data, duration, fs, fg, fa, still)
    if args.plot:
        _plot(args.plot, [('Gyro z', 'rad/s', fg, cg),
                          ('Accel x/y (mean)', 'm/s²', fa, ca)])


if __name__ == '__main__':
    main()
