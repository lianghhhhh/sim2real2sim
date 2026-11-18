#!/usr/bin/env python3
"""重算 sensor_mpc_node CSV 的循跡誤差 (到路徑折線的真正距離)。🖥 host 上跑, 只要 numpy。

    python3 src/sensor_mpc_pkg/scripts/eval_tracking.py car_run_data/mpc_xxx.csv [更多 CSV ...] \\
        [--path src/sensor_mpc_pkg/config/path_kros_car_mpc_sensor.csv] [--open] [--markdown]

新舊 CSV 都可以: 2026-11-18 以前的 cte 是對參考軌跡第 0 列算的 (彎道有 0.1*0.2/(2R) 的偏差),
這裡一律用 x/y (有 GT 就用 gt_x/gt_y) 對 --path 重算 (sensor_mpc_pkg/path_error.py), 舊的 cte 只印出來對照。

直線 / 左轉 / 右轉用投影點的路徑曲率分: |kappa| > 0.3 rad/m 算彎道 (以前用量到的 yaw rate 分)。
輸出欄位:
    v        平均車速 (vx, m/s)
    RMS/最大/平均   橫向誤差 (cm, 左正)
    直線RMS / 彎道RMS / 左轉平均 / 右轉平均   (cm)
    航向RMS  車頭和路徑切線的差 (°)
    飽和     任一輪 |指令| >= max_wheel_speed - 0.06 的週期 (%)
    指令變化 相鄰週期輪速指令差的 RMS x sqrt(2) (rad/s)
    定位 (有 GT 時): 位置誤差平均 / 最大, 車頭方向 / 左右的平均 (cm), yaw 誤差 p95 (°)
    舊cte    CSV 裡原本的 gt_cte (或 cte) RMS / 平均, 對照用
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from sensor_mpc_pkg.path_error import PathError          # noqa: E402
from sensor_mpc_pkg.path_tracker import load_path_csv     # noqa: E402

DEFAULT_PATH = os.path.join(HERE, '..', 'config', 'path_kros_car_mpc_sensor.csv')
K_TURN = 0.3


def col(rows, name):
    out = []
    for r in rows:
        v = r.get(name, '')
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            out.append(float('nan'))
    return np.array(out)


def rms(v):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    return float(np.sqrt(np.mean(v ** 2))) if len(v) else float('nan')


def mean(v):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    return float(np.mean(v)) if len(v) else float('nan')


def evaluate(csv_path, pe, wmax):
    with open(csv_path, newline='') as f:
        rows = [r for r in csv.DictReader(f) if r.get('ok') == '1']
    if not rows:
        return None
    gx, gy, gyaw = col(rows, 'gt_x'), col(rows, 'gt_y'), col(rows, 'gt_yaw')
    has_gt = np.isfinite(gx).mean() > 0.5
    x, y, yaw = col(rows, 'x'), col(rows, 'y'), col(rows, 'yaw')
    X, Y, YAW = (gx, gy, gyaw) if has_gt else (x, y, yaw)
    e = pe.query_many(X, Y, YAW)
    c, k = e['cross'], e['kappa']
    turn = np.abs(k) > K_TURN
    t = col(rows, 't')
    cmd = np.c_[col(rows, 'cmd_l'), col(rows, 'cmd_r')]
    old = col(rows, 'gt_cte' if has_gt else 'cte')
    o = dict(name=os.path.basename(csv_path), gt=has_gt, n=len(rows), dur=t[-1] - t[0],
             v=mean(col(rows, 'vx')), rms=rms(c), max=np.nanmax(np.abs(c)), mean=mean(c),
             rms_s=rms(c[~turn]), rms_t=rms(c[turn]), left=mean(c[k > K_TURN]), right=mean(c[k < -K_TURN]),
             head=math.degrees(rms(e['head_err'])),
             sat=float((np.abs(cmd) >= wmax - 0.06).any(axis=1).mean() * 100),
             dstep=rms(np.diff(cmd, axis=0)) * math.sqrt(2),
             solve=mean(col(rows, 'solve_ms')), old_rms=rms(old), old_mean=mean(old),
             src=sorted({r.get('err_src') or 'ref0' for r in rows}))
    if has_gt and not np.allclose(np.nan_to_num(x), np.nan_to_num(gx)):
        dx, dy = x - gx, y - gy
        cs, sn = np.cos(gyaw), np.sin(gyaw)
        ep = np.hypot(dx, dy)
        ey = np.degrees(np.abs((yaw - gyaw + np.pi) % (2 * np.pi) - np.pi))
        o.update(loc_mean=mean(ep), loc_max=np.nanmax(ep), loc_fwd=mean(cs * dx + sn * dy),
                 loc_lat=mean(-sn * dx + cs * dy), yaw_p95=float(np.nanpercentile(ey, 95)))
    return o


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--path', default=DEFAULT_PATH, help='路徑 CSV (跑的時候 path_file 給的那條)')
    ap.add_argument('--open', action='store_true', help='路徑不封閉 (loop:=false 跑的)')
    ap.add_argument('--max-wheel-speed', type=float, default=10.0)
    ap.add_argument('--markdown', action='store_true', help='印成 markdown 表格')
    a = ap.parse_args()
    pe = PathError(load_path_csv(a.path), loop=not a.open)
    res = [r for r in (evaluate(p, pe, a.max_wheel_speed) for p in a.csv) if r]
    if a.markdown:
        print('| CSV | v | RMS | 最大 | 平均 | 直線 RMS | 彎道 RMS | 左轉平均 | 右轉平均 | 航向 RMS | 飽和 | 指令變化 | 舊 cte RMS |')
        print('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
        for o in res:
            print(f'| {o["name"]} | {o["v"]:.2f} | {o["rms"] * 100:.2f} | {o["max"] * 100:.2f} | {o["mean"] * 100:+.2f} | '
                  f'{o["rms_s"] * 100:.2f} | {o["rms_t"] * 100:.2f} | {o["left"] * 100:+.2f} | {o["right"] * 100:+.2f} | '
                  f'{o["head"]:.2f}° | {o["sat"]:.0f}% | {o["dstep"]:.2f} | {o["old_rms"] * 100:.2f} |')
        loc = [o for o in res if 'loc_mean' in o]
        if loc:
            print('\n| CSV | 位置誤差平均 | 最大 | 車頭方向 | 左右 | yaw p95 |')
            print('| --- | --- | --- | --- | --- | --- |')
            for o in loc:
                print(f'| {o["name"]} | {o["loc_mean"] * 100:.2f} | {o["loc_max"] * 100:.2f} | {o["loc_fwd"] * 100:+.2f} | '
                      f'{o["loc_lat"] * 100:+.2f} | {o["yaw_p95"]:.2f}° |')
        return
    print(f'路徑 {os.path.basename(a.path)} ({pe.total:.1f} m, {"封閉" if pe.loop else "不封閉"}); 橫向誤差 cm, 左正')
    for o in res:
        print(f'{o["name"]}  ({"gt_x/gt_y" if o["gt"] else "x/y (沒有 gt 欄; 位姿來源是 GT 的那幾輪就是 GT)"}, {o["n"]} 週期, {o["dur"]:.0f} s, '
              f'CSV 的 cte 算法 {"/".join(o["src"])})')
        print(f'  v {o["v"]:.2f} m/s | RMS {o["rms"] * 100:.2f}  最大 {o["max"] * 100:.2f}  平均 {o["mean"] * 100:+.2f} | '
              f'直線 {o["rms_s"] * 100:.2f}  彎道 {o["rms_t"] * 100:.2f} | 左轉平均 {o["left"] * 100:+.2f}  '
              f'右轉平均 {o["right"] * 100:+.2f} | 航向 RMS {o["head"]:.2f}°')
        print(f'  飽和 {o["sat"]:.0f}%  指令變化 {o["dstep"]:.2f} rad/s  solve {o["solve"]:.1f} ms | '
              f'舊 cte RMS {o["old_rms"] * 100:.2f} 平均 {o["old_mean"] * 100:+.2f}')
        if 'loc_mean' in o:
            print(f'  定位: 平均 {o["loc_mean"] * 100:.2f}  最大 {o["loc_max"] * 100:.2f}  車頭方向 {o["loc_fwd"] * 100:+.2f}  '
                  f'左右 {o["loc_lat"] * 100:+.2f} cm  yaw p95 {o["yaw_p95"]:.2f}°')


if __name__ == '__main__':
    main()
