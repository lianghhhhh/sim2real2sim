#!/usr/bin/env python3
"""用 Isaac 的 ground truth 量出 LiDAR 地圖的 origin (平移 + 旋轉), 寫回地圖 .yaml。不需要 ROS。

    ./scripts/calibrate_map_origin.py                               # 只印結果, 不改檔
    ./scripts/calibrate_map_origin.py car_run_data/run.csv --write  # 寫進 room.yaml
    ./scripts/calibrate_map_origin.py --rotation yaw                # 旋轉改用 yaw 誤差估

寫完要在容器裡 `r` 一次, 地圖才會裝進 share (lidar_loc 預設讀 share 裡那份)。

為什麼需要
----------
slam_toolbox 建出來的地圖, `map` 座標系是**開始建圖那一刻的車子位姿** —— 起點不在
世界原點就差一個平移, 車頭沒對齊世界座標軸就差一個旋轉。兩種都不是定位在漂:

* 平移: 整段誤差是一個固定向量, 到哪裡都一樣
* 旋轉: yaw 誤差是一個固定角度; 位置誤差**跟離原點的距離成正比**, 房間兩端方向
  相反。1° 在 3 m 外 = 5.2 cm。房間中央量起來很準、開到兩端就不準, 就是這個

2026-09-14 那輪就是這樣: 原地自旋 (在中央) 誤差 1 cm, 衝刺到兩端 4~6 cm, 看起來
像「後面不準」, 其實是地圖轉了約 -1°。

怎麼算
------
1. 每一幀 LiDAR 估計用它**自己的 lid_stamp** 內插 GT (直接同列相減會把時間差當誤差)
2. 過濾: lid_age < 0.3、lid_sigma < 0.0025 (擋掉 180° 對稱解)、|yaw 誤差| < 30°、
   |角速度| < 0.5 rad/s (自旋時運動補償的誤差不是地圖的錯)
3. 擬合剛體變換  估計 ≈ R(θ)·真值 + b  (2D Kabsch, 迭代剔除殘差 > 3 倍中位數的幀)
4. 地圖座標跟世界座標差的就是同一個 (R, b), 所以新的 origin:
       yaw_new = yaw_old − θ
       xy_new  = R(−θ) · (xy_old − b)
   (推導: 地圖上的點 m = xy_old + R(yaw_old)·p, 世界座標 w = R(−θ)(m − b)
    = R(−θ)(xy_old − b) + R(yaw_old − θ)·p, 正好是 nav2 origin 的形式)

需要 `car_loc_lidar` 的 gridmap 支援 origin 的 yaw (2026-09 加的); 舊版讀到 yaw ≠ 0
會直接報錯。

注意
----
* **CSV 一定要是用「目前這份 yaml」錄的。** 拿舊資料對新 yaml 再算一次會把修正
  套兩遍。所以 CSV 比 yaml 舊的時候 --write 會拒絕 (確定沒問題再加 --force)。
* 車子要**開到房間各處**。只在中央原地轉的資料量不出旋轉 (槓桿臂是 0), 會警告。
* 位置擬合與 yaw 誤差兩個角度差很多 (> 0.3°) 代表地圖不只轉了, 還有扭曲 (SLAM
  建圖誤差), 那部分改 origin 修不掉 —— 要重建地圖或提高解析度。
* 實體車沒有 GT, 這支不能用。要改用已知座標的地標量 (見 src/car_loc_lidar/maps/README.md)。
"""
import argparse
import math
import os
import re
import sys

import numpy as np

try:
    import pandas as pd
except ImportError:
    sys.exit('需要 pandas: pip install pandas')

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CSV = os.path.join(REPO, 'car_run_data', 'sim_data.csv')
DEFAULT_YAML = os.path.join(REPO, 'src', 'car_loc_lidar', 'maps', 'room.yaml')


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def read_origin(yaml_path):
    import yaml
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    o = cfg.get('origin', [0.0, 0.0, 0.0])
    return float(o[0]), float(o[1]), float(o[2]) if len(o) > 2 else 0.0


def fit_rigid(P, Q, iters=10, k=3.0):
    """找 θ, b 使 Q ≈ R(θ)·P + b。P = 真值, Q = 估計, 都是 (N, 2)。"""
    keep = np.ones(len(P), dtype=bool)
    for _ in range(iters):
        pc, qc = P[keep].mean(axis=0), Q[keep].mean(axis=0)
        H = (P[keep] - pc).T @ (Q[keep] - qc)
        th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        b = qc - rot(th) @ pc
        r = np.linalg.norm(Q - (P @ rot(th).T + b), axis=1)
        new = r < max(k * float(np.median(r[keep])), 0.02)
        if (new == keep).all():
            break
        keep = new
    return th, b, keep


def err_stats(e_m):
    e = np.asarray(e_m) * 100
    return f'中位 {np.median(e):5.2f} cm, p90 {np.quantile(e, .9):5.2f} cm, 最大 {e.max():6.2f} cm'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('csv', nargs='?', default=DEFAULT_CSV)
    ap.add_argument('--yaml', default=DEFAULT_YAML, help='要校正的地圖 yaml (預設 room.yaml)')
    ap.add_argument('--rotation', choices=('position', 'yaw', 'none'), default='position',
                    help='旋轉怎麼估: position = 用位置擬合 (預設, 直接最小化位置誤差); '
                         'yaw = 用 yaw 誤差中位數; none = 只修平移')
    ap.add_argument('--sigma', type=float, default=0.0025, help='lid_sigma 門檻')
    ap.add_argument('--max-omega', type=float, default=0.5, help='|角速度| 上限 (rad/s)')
    ap.add_argument('--max-speed', type=float, default=float('inf'),
                    help='車速上限 (m/s)。只想用靜止幀就給 0.05')
    ap.add_argument('--write', action='store_true', help='把新的 origin 寫回 yaml')
    ap.add_argument('--force', action='store_true', help='CSV 比 yaml 舊也照寫')
    a = ap.parse_args()

    for path in (a.csv, a.yaml):
        if not os.path.isfile(path):
            sys.exit(f'找不到 {path}')
    df = pd.read_csv(a.csv)
    if 'lid_x' not in df or not df.lid_x.notna().any():
        sys.exit('CSV 裡沒有 LiDAR 的資料 (lid_x 整欄 NaN) —— 錄的時候 lidar 沒開?')

    gt = df.drop_duplicates('odom_stamp').sort_values('odom_stamp')
    T = gt.odom_stamp.values
    off = float((df.lid_stamp - df.odom_stamp).median())
    st = df.lid_stamp.values - (off if abs(off) > 1.0 else 0.0)   # 時鐘基準不同就先扣掉
    P = np.c_[np.interp(st, T, gt.car_position_x), np.interp(st, T, gt.car_position_y)]
    Q = np.c_[df.lid_x.values, df.lid_y.values]
    gyaw = np.interp(st, T, np.unwrap(gt.gt_yaw.values))
    eyaw = wrap(df.lid_yaw.values - gyaw)
    omega = np.abs(np.interp(st, T, np.gradient(np.unwrap(gt.gt_yaw.values), T)))
    speed = np.hypot(np.interp(st, T, np.gradient(gt.car_position_x.values, T)),
                     np.interp(st, T, np.gradient(gt.car_position_y.values, T)))

    ok = (np.isfinite(Q).all(axis=1) & (df.lid_age.values < 0.3)
          & (df.lid_sigma.values < a.sigma) & (np.abs(eyaw) < math.radians(30))
          & (omega < a.max_omega) & (speed <= a.max_speed))
    n = int(ok.sum())
    print(f'{a.csv}\n  {len(df)} 列, 用 {n} 幀 (lid_sigma<{a.sigma}, |w|<{a.max_omega}, '
          f'|yaw 誤差|<30°' + (f', 車速<={a.max_speed}' if np.isfinite(a.max_speed) else '') + ')')
    if n < 100:
        sys.exit('  可用的幀太少 (< 100), 不算')

    # 旋轉靠的是槓桿臂: 只要在**某個方向**離中心夠遠就量得到 (沿 x 來回開, y 的
    # 誤差就會跟 x 成正比)。所以看離質心的總距離, 不是 x、y 各自都要散開。
    spread = P[ok].std(axis=0)
    rotation = a.rotation
    if rotation != 'none' and float(np.hypot(*spread)) < 0.5:
        print(f'  !! 車子幾乎沒離開原地 (離質心 RMS {np.hypot(*spread):.2f} m), '
              '量不出旋轉 —— 改成只修平移。要量旋轉就開到房間各處再錄一次')
        rotation = 'none'

    th_pos, b_pos, keep = fit_rigid(P[ok], Q[ok])
    th_yaw = float(np.median(eyaw[ok]))
    if rotation == 'position':
        th, b = th_pos, b_pos
    else:
        th = th_yaw if rotation == 'yaw' else 0.0
        b = np.median(Q[ok] - P[ok] @ rot(th).T, axis=0)

    print(f'\n  旋轉: 位置擬合 {math.degrees(th_pos):+.3f}°, yaw 誤差中位 '
          f'{math.degrees(th_yaw):+.3f}°  -> 採用 {math.degrees(th):+.3f}° ({rotation})')
    if abs(th_pos - th_yaw) > math.radians(0.3):
        print('  !! 兩個角度差超過 0.3° —— 地圖除了轉歪還有扭曲 (SLAM 建圖誤差), '
              '改 origin 只能修掉一部分')
    print(f'  平移: ({b[0] * 100:+.2f}, {b[1] * 100:+.2f}) cm'
          f'   (擬合時剔除 {int((~keep).sum())} 幀離群)')

    corr = (Q[ok] - b) @ rot(th)             # R(-θ)(Q - b), 每一列乘 R = 套用 R^T
    print(f'\n  預期效果 (用這份資料離線套修正):')
    print(f'    現在    {err_stats(np.linalg.norm(Q[ok] - P[ok], axis=1))}')
    print(f'    修正後  {err_stats(np.linalg.norm(corr - P[ok], axis=1))}')
    print(f'    yaw     中位 {math.degrees(th_yaw):+.2f}° -> {math.degrees(th_yaw - th):+.2f}°')

    ox, oy, oyaw = read_origin(a.yaml)
    nx, ny = rot(-th) @ (np.array([ox, oy]) - b)
    nyaw = oyaw - th
    new_line = f'origin: [{nx:.4f}, {ny:.4f}, {nyaw:.6f}]'
    print(f'\n  {a.yaml}')
    print(f'    現在  origin: [{ox}, {oy}, {oyaw}]')
    print(f'    改成  {new_line}      (yaw {math.degrees(nyaw):+.3f}°)')

    if not a.write:
        print('\n  (沒有改檔。確認數字合理之後加 --write)')
        return 0
    if os.path.getmtime(a.csv) < os.path.getmtime(a.yaml) and not a.force:
        sys.exit('\n  !! CSV 比 yaml 舊 —— 這份資料很可能是用「改之前的 origin」錄的, '
                 '再套一次會修兩遍。\n     用目前的 yaml 重錄一輪再算; 確定沒問題就加 --force')
    with open(a.yaml) as f:
        text = f.read()
    text, cnt = re.subn(r'^origin:.*$', new_line, text, count=1, flags=re.M)
    if cnt != 1:
        sys.exit(f'  !! {a.yaml} 裡找不到 origin: 那一行, 沒有改')
    with open(a.yaml, 'w') as f:
        f.write(text)
    print(f'\n  已寫入。舊的是 origin: [{ox}, {oy}, {oyaw}] (要還原就改回去)')
    print('  記得在容器裡 `r`, 讓地圖裝進 share; 然後重錄一輪, 再跑一次這支確認偏移 < 1 cm')
    return 0


if __name__ == '__main__':
    sys.exit(main())
