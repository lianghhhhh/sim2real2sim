#!/usr/bin/env python3
"""從實跑資料擬合「天花板相機像素 -> Isaac 世界座標」的校正參數。

用法:
    python3 scripts/calibrate_camera_ground.py car_run_data/sim_data.csv
    python3 scripts/calibrate_camera_ground.py run.csv -o src/car_inference/config/camera_ground.yaml
    python3 scripts/calibrate_camera_ground.py run.csv --model homography+radial

CSV 需要的欄位:
    car_position_x, car_position_y      -- Isaac odom ground truth
    以及底下二選一:
      yolo_px, yolo_py                  -- bbox 中心的原始像素 (建議, 新版節點會記)
      yolo_x, yolo_y                    -- 舊版節點算出的世界座標
                                           (腳本會用 --legacy-* 參數反推回像素)

為什麼要有這個腳本:
    舊版的轉換是手量的比例常數 (1860 px = 10 m, 1120 px = 6 m), 誤差 RMSE
    0.136 m。裡面有一半是「視差」造成的系統性放大: 相機在 z=2.7 m, 但
    YOLO bbox 中心看到的是車身 (z ≈ 0.10 m) 而不是接地點, 半徑被放大
    2.7/(2.7-0.10) = 3.7%。單應性直接擬合到「車身高度那個平面」, 這一項
    連同相機傾斜、主點偏移、安裝旋轉一起被吸收掉。
    只要相機、解析度、車高、YOLO 模型任何一個變了, 就該重跑這支腳本。
"""
import argparse
import sys

import numpy as np
import pandas as pd

CENTER = np.array([960.0, 768.0])
NORM = 1000.0


def fit_homography(px_norm, world):
    """DLT: 解 H 使得 H @ [dx, dy, 1] ~ [X, Y, 1] (齊次)。"""
    x, y = px_norm[:, 0], px_norm[:, 1]
    X, Y = world[:, 0], world[:, 1]
    z, o = np.zeros_like(x), np.ones_like(x)
    A = np.vstack([
        np.stack([x, y, o, z, z, z, -X * x, -X * y, -X], axis=1),
        np.stack([z, z, z, x, y, o, -Y * x, -Y * y, -Y], axis=1),
    ])
    h = np.linalg.svd(A)[2][-1].reshape(3, 3)
    return h / h[2, 2]


def fit_affine(px_norm, world):
    """6 DOF 仿射, 寫成單應性的形式 (最後一列固定 [0, 0, 1])。"""
    A = np.c_[px_norm, np.ones(len(px_norm))]
    M = np.linalg.lstsq(A, world, rcond=None)[0]      # (3, 2)
    return np.vstack([M.T, [0.0, 0.0, 1.0]])


def apply_h(H, px_norm):
    v = np.c_[px_norm, np.ones(len(px_norm))] @ H.T
    return v[:, :2] / v[:, 2:3]


def undistort(px_norm, k):
    r2 = (px_norm ** 2).sum(axis=1, keepdims=True)
    return px_norm * (1.0 + k[0] * r2 + k[1] * r2 * r2)


def cross_val(px_norm, world, fit, folds=5, seed=0):
    """K-fold 交叉驗證。自由度越多擬合誤差一定越小, 只有交叉驗證能看出
    到底是真的學到鏡頭的幾何, 還是在背這一批資料的噪聲。"""
    idx = np.arange(len(px_norm))
    np.random.RandomState(seed).shuffle(idx)
    errs = []
    for k in range(folds):
        te = idx[k::folds]
        tr = np.setdiff1d(idx, te)
        pred = apply_h(fit(px_norm[tr], world[tr]), px_norm[te])
        errs.append(np.hypot(*(pred - world[te]).T))
    return np.concatenate(errs)


def report(name, err):
    print(f'  {name:<26} RMSE {np.sqrt((err ** 2).mean()):.4f} m   '
          f'mean {err.mean():.4f}   p95 {np.percentile(err, 95):.4f}   '
          f'max {err.max():.4f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('-o', '--output', default='src/car_inference/config/camera_ground.yaml')
    ap.add_argument('--model', default='homography',
                    choices=['affine', 'homography', 'homography+radial'],
                    help='預設 homography。實測這台相機幾乎是純針孔投影, '
                         '加徑向項只從 0.0729 降到 0.0717, 不值得多兩個自由度。')
    ap.add_argument('--width', type=int, default=1920, help='擬合時的影像寬')
    ap.add_argument('--height', type=int, default=1536, help='擬合時的影像高')
    ap.add_argument('--dry-run', action='store_true', help='只比較各模型, 不寫檔')
    # 只有 CSV 裡沒有 yolo_px/yolo_py 時才用得到: 反推舊版節點的線性公式
    ap.add_argument('--legacy-x-per-px', type=float, default=-5.0 / 930.0)
    ap.add_argument('--legacy-y-per-px', type=float, default=3.0 / 560.0)
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    if {'yolo_px', 'yolo_py'} <= set(df.columns):
        df = df.dropna(subset=['yolo_px', 'yolo_py', 'car_position_x', 'car_position_y'])
        px = df.yolo_px.to_numpy(float)
        py = df.yolo_py.to_numpy(float)
        source = 'yolo_px / yolo_py (原始像素)'
    elif {'yolo_x', 'yolo_y'} <= set(df.columns):
        df = df.dropna(subset=['yolo_x', 'yolo_y', 'car_position_x', 'car_position_y'])
        px = df.yolo_x.to_numpy(float) / args.legacy_x_per_px + CENTER[0]
        py = df.yolo_y.to_numpy(float) / args.legacy_y_per_px + CENTER[1]
        source = 'yolo_x / yolo_y 反推 (舊版線性公式)'
    else:
        sys.exit('CSV 需要 yolo_px/yolo_py 或 yolo_x/yolo_y 欄位')

    world = df[['car_position_x', 'car_position_y']].to_numpy(float)
    if len(world) < 20:
        sys.exit(f'只有 {len(world)} 筆資料, 太少; 請讓車子跑遍整個場地再錄一次')

    raw = (np.stack([px, py], axis=1) - CENTER) / NORM
    print(f'資料: {args.csv}  n={len(world)}  來源: {source}')
    print(f'涵蓋範圍: x[{world[:, 0].min():+.2f},{world[:, 0].max():+.2f}] '
          f'y[{world[:, 1].min():+.2f},{world[:, 1].max():+.2f}] m')
    # car.usd 的房間內牆是 x[-5,5] y[-3,3]。校正只在資料涵蓋到的地方有效,
    # 外推出去誤差會迅速放大 (單應性的透視項在外推時最不穩)。
    room = ((-5.0, 5.0), (-3.0, 3.0))
    for ax, name in enumerate('xy'):
        lo, hi = world[:, ax].min(), world[:, ax].max()
        span = (hi - lo) / (room[ax][1] - room[ax][0])
        if span < 0.7:
            print(f'  警告: {name} 只涵蓋房間的 {span:.0%} '
                  f'({lo:+.2f}~{hi:+.2f}, 房間 {room[ax][0]:+.1f}~{room[ax][1]:+.1f})。'
                  f'請讓車子跑遍四個角落再錄一次, 尤其是角落 —— '
                  f'投影模型的非線性項全靠邊緣的點才定得住。')

    print('\n各模型的 5-fold 交叉驗證誤差:')
    if {'yolo_x', 'yolo_y'} <= set(df.columns):
        report('現況 (舊版線性公式)',
               np.hypot(df.yolo_x.to_numpy(float) - world[:, 0],
                        df.yolo_y.to_numpy(float) - world[:, 1]))
    report('仿射 6 DOF', cross_val(raw, world, fit_affine))
    report('單應性 8 DOF', cross_val(raw, world, fit_homography))

    k = np.array([0.0, 0.0])
    if args.model == 'homography+radial':
        from scipy.optimize import least_squares

        def resid(kk):
            u = undistort(raw, kk)
            return np.hypot(*(apply_h(fit_homography(u, world), u) - world).T)

        k = least_squares(resid, [0.0, 0.0]).x
        report('單應性 + 徑向 k1,k2', cross_val(undistort(raw, k), world, fit_homography))
        print(f'    k1={k[0]:.5f} k2={k[1]:.5f}')

    fit = fit_affine if args.model == 'affine' else fit_homography
    used = undistort(raw, k)
    H = fit(used, world)
    final = np.hypot(*(apply_h(H, used) - world).T)
    print(f'\n選用模型: {args.model}')
    report('全資料擬合殘差', final)

    if args.dry_run:
        print('\n(dry-run, 沒有寫檔)')
        return

    rows = '\n'.join('  - [' + ', '.join(f'{v: .6f}' for v in r) + ']' for r in H)
    with open(args.output, 'w') as f:
        f.write(f"""# 天花板相機 -> Isaac 世界座標 的地面投影校正
#
# 由 scripts/calibrate_camera_ground.py 自動產生, 不要手改。
#   資料: {args.csv}  n={len(world)}  模型: {args.model}
#   全資料殘差 RMSE {np.sqrt((final ** 2).mean()):.4f} m (p95 {np.percentile(final, 95):.4f})
#
# 模型: 像素 -> (徑向去畸變) -> 單應性 -> 世界 (x, y)
#   d  = ([px, py] - center_px) / norm_scale
#   d' = d * (1 + k1*|d|^2 + k2*|d|^4)
#   [Xw, Yw, w] = H @ [d'x, d'y, 1];  (X, Y) = (Xw/w, Yw/w)
#
# 單應性是針孔相機看一個平面的精確模型。把它擬合到「車身高度那個平面」,
# 會一併吸收掉視差 (相機 z=2.7, bbox 中心是車身 z≈0.10)、相機傾斜、
# 主點偏移與安裝旋轉。
#
# 相機位置/朝向、render 解析度、車高、YOLO 模型任一改變 -> 重跑校正腳本。

image_width: {args.width}
image_height: {args.height}

center_px: [{CENTER[0]}, {CENTER[1]}]
norm_scale: {NORM}

distortion: [{k[0]:.6f}, {k[1]:.6f}]

homography:
{rows}
""")
    print(f'\n已寫入 {args.output}')


if __name__ == '__main__':
    main()
