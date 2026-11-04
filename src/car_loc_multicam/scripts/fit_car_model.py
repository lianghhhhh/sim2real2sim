#!/usr/bin/env python3
"""用錄到的 YOLO 框 + ground truth, 檢查車型 (car_model.yaml) 跟 YOLO 對不對得上。

    python3 src/car_loc_multicam/scripts/fit_car_model.py /tmp/rec.jsonl config/car_model.yaml [其他車型.yaml ...]
    python3 src/car_loc_multicam/scripts/fit_car_model.py car_run_data/all_loc.csv config/car_model.yaml

(錄法: scripts/record_bbox.py 的 jsonl, 或 calibrate_env_pkg collect_data_node 的 CSV +
 `_bbox.csv` —— 給哪一個都可以。)

(只需要 numpy + PyYAML, 不需要 ROS。)

對每個車型印三件事:

  1. **每個邊的偏差** (YOLO - 預測, px), 全部與每台相機分開 —— 某台某個邊特別大,
     通常是那個方向看得到的細長零件 (夾爪、LiDAR) 該排除 (export_from_usd.py --exclude)
  2. **建議的 edge_bias_px** = 前半段的平均偏差 (每個邊)
  3. **它值不值得**: 後半段每一刻用多台相機反推位置 (yaw 用 GT), 跟 GT 比 ——
     加 edge_bias 前後的常數偏移與 RMS。前半段量、後半段驗, 所以不是自己驗自己。
  4. **依距離分**: 車離相機多遠時 YOLO 開始框不準 (2026-10-30 就是這張表抓到
     「車太近時 ROI 沒有縮小」的問題 —— 2~3 m 內 83% 的框差超過 10 px)。

注意: 這裡的「真值」是 /odom (Isaac 的 IsaacComputeOdometry, 相對起點)。車子不是從
世界原點、yaw 0 出發的話, /odom 跟相機的世界座標就差一個剛體變換, 這支的數字會錯。
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from car_loc_multicam.camera_model import load_cameras, share_file  # noqa: E402
from car_loc_multicam.car_model import CarModel, associate, edge_mask  # noqa: E402


def load_collect_csv(path):
    """calibrate_env_pkg collect_data_node 記的檔 -> 跟 jsonl 一樣的紀錄。

    給主 CSV 或 `_bbox.csv` 都可以: GT 從主 CSV (odom_stamp, car_position_x/y, gt_yaw),
    框從 `<主檔名>_bbox.csv` (一列一個框, n_boxes = 0 的列是沒看到)。
    """
    import csv
    stem = path[:-len('_bbox.csv')] if path.endswith('_bbox.csv') else os.path.splitext(path)[0]
    main_csv, bbox_csv = stem + '.csv', stem + '_bbox.csv'
    for p in (main_csv, bbox_csv):
        if not os.path.exists(p):
            sys.exit(f'找不到 {p} (collect_data_node 要開 bbox_csv)')
    rec, seen = [], set()
    with open(main_csv, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            try:
                t, x, y, w = (float(r[k]) for k in ('odom_stamp', 'car_position_x',
                                                    'car_position_y', 'gt_yaw'))
            except (KeyError, ValueError):
                continue
            if t == t and t not in seen:          # 同一則 /odom 會被記好幾列
                seen.add(t)
                rec.append(dict(k='gt', t=t, x=x, y=y, yaw=w))
    frames = {}
    with open(bbox_csv, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            b = frames.setdefault((r['cam'], r['stamp']), [])
            if int(r['n_boxes']) > 0:
                b.append([float(r[k]) for k in ('u1', 'v1', 'u2', 'v2', 'conf')])
    rec += [dict(k='bbox', t=float(t), cam=c, boxes=b) for (c, t), b in frames.items()]
    return rec


def load(path):
    if path.endswith('.csv'):
        rec = load_collect_csv(path)
    else:
        rec = [json.loads(line) for line in open(path, encoding='utf-8')]
    gt = sorted([r for r in rec if r['k'] == 'gt'], key=lambda r: r['t'])
    if not gt:
        sys.exit('錄到的檔案裡沒有 /odom (ground truth)')
    T = np.array([r['t'] for r in gt])
    X = np.array([r['x'] for r in gt])
    Y = np.array([r['y'] for r in gt])
    W = np.unwrap([r['yaw'] for r in gt])

    def at(t):
        return float(np.interp(t, T, X)), float(np.interp(t, T, Y)), float(np.interp(t, T, W))
    bbs = sorted([r for r in rec if r['k'] == 'bbox' and r['boxes'] and T[0] <= r['t'] <= T[-1]],
                 key=lambda r: r['t'])
    return bbs, at


def residuals(car, cams, bbs, at):
    """每一則: (時刻, 相機, 量到的框, YOLO - 預測 (不能用的邊是 nan))"""
    out = []
    for r in bbs:
        c = cams.get(r['cam'])
        if c is None:
            continue
        st = at(r['t'])
        k = associate(car, c, [(np.array(b[:4]), b[4]) for b in r['boxes']], st)
        if k is None:
            continue
        box = np.array(r['boxes'][k][:4])
        pred, _ = car.predict_bbox(c, *st, jac=False)
        if pred is None:
            continue
        m = edge_mask(c, box, pred, 4.0)
        out.append((r['t'], c, box, np.where(m, box - pred, np.nan)))
    return out


def by_distance(res, at):
    for lo, hi in ((0, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 8), (8, 99)):
        ds = []
        for t, c, _, d in res:
            x, y, _ = at(t)
            if lo <= np.hypot(x - c.center[0], y - c.center[1]) < hi:
                ds.append(d)
        if ds:
            D = np.vstack(ds)
            bad = np.mean(np.nanmax(np.abs(np.nan_to_num(D)), 1) > 10)
            print(f'   距離 {lo}~{hi} m: n={len(D):4d} 平均 {np.round(np.nanmean(D, 0), 1).tolist()} '
                  f'std {np.round(np.nanstd(D, 0), 1).tolist()}  差 >10 px 的比例 {bad:.0%}')


def position_check(car, res, at):
    """同一時刻的框 (多台相機) 一起反推位置, yaw 用 GT -> 跟 GT 的差 (cm)。"""
    groups = {}
    for t, c, box, _ in res:
        groups.setdefault(round(t, 3), []).append((c, box))
    err = []
    for t, obs in groups.items():
        x, y, yaw = at(t)
        r = car.solve_position(obs, yaw, x0=(x, y))
        if r is not None:
            err.append((r['x'] - x, r['y'] - y))
    e = np.array(err) * 100
    if not len(e):
        return 'n=0'
    return (f'n={len(e)} 常數偏移 ({e[:, 0].mean():+.2f}, {e[:, 1].mean():+.2f}) cm, '
            f'RMS {np.sqrt((e ** 2).sum(1).mean()):.2f} cm')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('rec')
    ap.add_argument('models', nargs='+')
    ap.add_argument('--cameras', default=share_file('env_cameras.yaml'))
    a = ap.parse_args()
    cams = {c.name: c for c in load_cameras(a.cameras)}
    bbs, at = load(a.rec)
    print(f'{len(bbs)} 則有框的偵測')
    for path in a.models:
        car = CarModel.from_yaml(path)
        car.edge_bias = np.zeros(4)           # 先看原始外形
        res = residuals(car, cams, bbs, at)
        if not res:
            print(f'\n== {path}: 沒有可以比的框')
            continue
        D = np.vstack([r[3] for r in res])
        print(f'\n== {os.path.basename(path)} ({len(car.points)} 點): {len(D)} 框, '
              f'每邊 RMS {np.sqrt(np.nanmean(D ** 2)):.2f} px')
        print(f'   [u1 v1 u2 v2] 平均 {np.round(np.nanmean(D, 0), 1).tolist()}  '
              f'std {np.round(np.nanstd(D, 0), 1).tolist()}')
        for name in sorted({r[1].name for r in res}):
            d = np.vstack([r[3] for r in res if r[1].name == name])
            print(f'   {name}: n={len(d):4d} 平均 {np.round(np.nanmean(d, 0), 1).tolist()}  '
                  f'std {np.round(np.nanstd(d, 0), 1).tolist()}')

        by_distance(res, at)
        half = len(res) // 2
        bias = np.nanmean(np.vstack([r[3] for r in res[:half]]), 0)
        bias = np.where(np.isfinite(bias), bias, 0.0)
        test = res[half:]
        print(f'   建議 edge_bias_px (前半段量): [{", ".join(f"{b:.1f}" for b in bias)}]')
        print(f'   後半段反推位置, 不加 bias: {position_check(car, test, at)}')
        car.edge_bias = bias
        print(f'   後半段反推位置, 加 bias  : {position_check(car, test, at)}')
        D2 = np.vstack([r[3] for r in test]) - bias
        print(f'   加 bias 後後半段每邊 std {np.round(np.nanstd(D2, 0), 1).tolist()} px '
              '(-> multicam_loc.yaml 的 bbox_sigma_px / bbox_size_frac 參考)')


if __name__ == '__main__':
    main()
