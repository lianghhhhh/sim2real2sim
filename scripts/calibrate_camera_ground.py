#!/usr/bin/env python3
"""從實跑資料擬合「天花板相機像素 -> Isaac 世界座標」的校正參數。

用法:
    python3 scripts/calibrate_camera_ground.py car_run_data/sim_data.csv            # 只看結果
    python3 scripts/calibrate_camera_ground.py run1.csv run2.csv --write            # 寫回校正檔
    python3 scripts/calibrate_camera_ground.py tour.csv --model homography --write  # 整個重擬

CSV 是 collect_data_node 錄的 (需要 yolo_px/yolo_py/yolo_conf、cam_stamp、odom_stamp、
car_position_x/y、gt_yaw)。可以給好幾份, 會合在一起擬合。

兩件舊版沒做、而換成 kros_car 之後一定要做的事
----------------------------------------------
1. **依影像時刻對齊 ground truth。** CSV 每一列是「收到過的最新像素」配當下的真值,
   逐列配對會混進 50~100 ms 的年齡; 車子在動的時候那就是好幾公分, 而且方向跟著
   車頭走。這裡用 cam_stamp (影像時戳) 去內插真值。

2. **bbox 中心不是 base_link 原點。** kros_car 車頭有手臂, YOLO 框的中心在原點
   前方 ~1.5 cm, 而且這個偏移**跟著車頭轉** —— 它不是常數, 單應性吸收不掉;
   硬擬合只會被平均成一個「看資料裡車頭多半朝哪」的假平移。所以模型多兩個參數
   body_offset = (bx, by):

       投影(像素) = p_base + R(yaw) @ body_offset

   相機節點自己**不會**扣這一項 (它的 yaw 是速度方向推的, 原地轉時是舊的值, 拿去
   扣只會更差 —— 離線實測 RMS 2.55 -> 2.40 / 2.34 -> 2.97 cm, 沒有穩定變好);
   有可信 yaw 的下游要扣: 融合節點的 `camera_offset` (--write 會一起改)。

模型 (--model)
--------------
    adjust (預設)      沿用現有的單應性與畸變, 只修「車換了」會動到的三件事:
                       整體比例 (bbox 中心的等效高度變了 -> 視差變了)、平移、
                       body_offset。5 個參數, **資料只涵蓋房間一部分也安全**。
    homography         整個重擬 (8 + 2 個參數)。
    homography+radial  再加徑向畸變 k1, k2。

    後兩個只有在資料**跑遍整個房間**時才能用 —— 單應性的透視項與畸變全靠邊緣的
    點定住。2026-10-13 實測: 拿只涵蓋 x±0.8 m 的資料整個重擬, 訓練殘差 1.50 cm
    很漂亮, 拿到另一份資料上是 **19 cm**。涵蓋不到 70% 這裡會直接拒絕
    (--force 可以硬來)。要整個重擬請先用 scripts/tour_drive.py 繞場錄一份。
"""
import argparse
import datetime
import os
import re
import sys

import numpy as np
import pandas as pd
import yaml
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CAL_DEFAULT = os.path.join(REPO, 'src/car_loc_camera/config/camera_ground.yaml')
FUSION_DEFAULT = os.path.join(REPO, 'src/car_loc_fusion/config/fusion_loc.yaml')
ROOM = ((-5.0, 5.0), (-3.0, 3.0))          # 房間內牆


def load(path, min_conf):
    """一則偵測一列: 影像時刻的像素 + 那一刻的 ground truth (x, y, yaw)。"""
    raw = pd.read_csv(path, low_memory=False)
    need = {'yolo_px', 'yolo_py', 'cam_stamp', 'odom_stamp',
            'car_position_x', 'car_position_y', 'gt_yaw'}
    if not need <= set(raw.columns):
        sys.exit(f'{path}: 缺少欄位 {sorted(need - set(raw.columns))} '
                 '(要 collect_data_node 錄的 CSV, 而且相機定位節點要開著)')
    raw = raw.sort_values('odom_stamp')
    g = (raw[['odom_stamp', 'car_position_x', 'car_position_y', 'gt_yaw']]
         .dropna().drop_duplicates('odom_stamp'))
    T = g.odom_stamp.to_numpy()
    yaw_u = np.unwrap(g.gt_yaw.to_numpy())
    u = raw[raw.yolo_px.notna() & raw.cam_stamp.notna()].drop_duplicates('cam_stamp')
    t = u.cam_stamp.to_numpy()
    # /rgb 與 /odom 不一定同一個時鐘基準 (實測有一份差 2372 s)。差超過 1 秒才當成
    # 時鐘不同、扣掉; 否則那個差就是影像真的比較舊, 不能扣 (見 replay_camera_csv.py)。
    off = float(np.median(t - u.odom_stamp.to_numpy()))
    if abs(off) > 1.0:
        t = t - off
    conf = (u.yolo_conf.to_numpy() if 'yolo_conf' in u else np.ones(len(u)))
    ok = (t > T[0] + 0.2) & (t < T[-1] - 0.2) & (conf >= min_conf)
    u, t = u[ok], t[ok]
    # 車子不動的時候同一個像素會重複幾千次, 留一筆就好 (不然靜止點主導擬合)
    keep = ~pd.DataFrame({'a': u.yolo_px.to_numpy(), 'b': u.yolo_py.to_numpy()}).duplicated().to_numpy()
    u, t = u[keep], t[keep]
    return dict(px=u.yolo_px.to_numpy(float), py=u.yolo_py.to_numpy(float),
                x=np.interp(t, T, g.car_position_x), y=np.interp(t, T, g.car_position_y),
                yaw=np.interp(t, T, yaw_u))


class Model:
    def __init__(self, cal):
        self.c = np.asarray(cal['center_px'], float)
        self.ns = float(cal['norm_scale'])
        self.H0 = np.asarray(cal['homography'], float).reshape(3, 3)
        self.H0 = self.H0 / self.H0[2, 2]
        self.k0 = np.asarray(list(cal.get('distortion', [0, 0])) + [0, 0], float)[:2]

    def project(self, H, k, px, py):
        dx, dy = (px - self.c[0]) / self.ns, (py - self.c[1]) / self.ns
        r2 = dx * dx + dy * dy
        f = 1.0 + k[0] * r2 + k[1] * r2 * r2
        dx, dy = dx * f, dy * f
        w = H[2, 0] * dx + H[2, 1] * dy + H[2, 2]
        return ((H[0, 0] * dx + H[0, 1] * dy + H[0, 2]) / w,
                (H[1, 0] * dx + H[1, 1] * dy + H[1, 2]) / w)

    def unpack(self, kind, p):
        """參數向量 -> (H, k, body_offset)。"""
        b = np.asarray(p[:2], float)
        if kind == 'current':
            return self.H0, self.k0, np.zeros(2)
        if kind == 'offset':
            return self.H0, self.k0, b
        if kind == 'adjust':
            H = self.H0.copy()
            H[:2] *= p[2]              # 以世界原點 (相機正下方) 為中心的整體比例
            H[0, 2] += p[3]
            H[1, 2] += p[4]
            return H, self.k0, b
        H = np.r_[p[2:10], 1.0].reshape(3, 3)
        return H, (np.asarray(p[10:12], float) if kind == 'homography+radial' else self.k0), b

    def x0(self, kind):
        h = self.H0.ravel()[:8]
        return {'current': np.zeros(0), 'offset': np.zeros(2),
                'adjust': np.array([0, 0, 1.0, 0, 0]),
                'homography': np.r_[0, 0, h],
                'homography+radial': np.r_[0, 0, h, self.k0]}[kind]

    def resid(self, kind, p, d, use_offset=True):
        H, k, b = self.unpack(kind, p)
        if not use_offset:
            b = np.zeros(2)
        zx, zy = self.project(H, k, d['px'], d['py'])
        c, s = np.cos(d['yaw']), np.sin(d['yaw'])
        return zx - (d['x'] + c * b[0] - s * b[1]), zy - (d['y'] + s * b[0] + c * b[1])

    def fit(self, kind, d):
        if kind == 'current':
            return np.zeros(0)
        # soft_l1: 偶爾框到影子 / 只框到半台車的那幾幀不要主導
        return least_squares(lambda p: np.concatenate(self.resid(kind, p, d)),
                             self.x0(kind), loss='soft_l1', f_scale=0.02).x


def sub(d, idx):
    return {k: v[idx] for k, v in d.items()}


def stat(ex, ey):
    e = np.hypot(ex, ey) * 100
    return (f'RMS {np.sqrt((e ** 2).mean()):5.2f}  中位 {np.median(e):5.2f}  '
            f'p95 {np.percentile(e, 95):5.2f} cm   平均 ({ex.mean() * 100:+.2f}, {ey.mean() * 100:+.2f})')


def cross_val(m, kind, d, folds=5):
    """依**時間**切段 (不是打散): 相鄰的幀幾乎一樣, 打散等於拿訓練資料考自己。"""
    n = len(d['px'])
    edges = np.linspace(0, n, folds + 1).astype(int)
    ex, ey = [], []
    for k in range(folds):
        te = np.arange(edges[k], edges[k + 1])
        tr = np.r_[np.arange(0, edges[k]), np.arange(edges[k + 1], n)]
        p = m.fit(kind, sub(d, tr))
        a, b = m.resid(kind, p, sub(d, te))
        ex.append(a)
        ey.append(b)
    return np.concatenate(ex), np.concatenate(ey)


def rewrite_yaml(path, H, k, b, sigma, note):
    """只換數字, 註解全部留著 (那些註解是踩過的坑的紀錄)。"""
    s = open(path).read()
    rows = '\n'.join('  - [' + ', '.join(repr(float(v)) for v in r) + ']' for r in H)
    s, n1 = re.subn(r'(?m)^homography:\n(?:[ \t]+-[^\n]*\n){3}', f'homography:\n{rows}\n', s)
    s, n2 = re.subn(r'(?m)^distortion:.*$', f'distortion: [{float(k[0])!r}, {float(k[1])!r}]', s)
    s, n3 = re.subn(r'(?m)^residual_rms:.*$', f'residual_rms: {sigma:.4f}', s)
    if not (n1 == 1 and n2 == 1 and n3 == 1):
        sys.exit(f'{path}: 找不到 homography / distortion / residual_rms 欄位, 沒有改檔')
    line = f'body_offset: [{b[0]:.4f}, {b[1]:.4f}]'
    if re.search(r'(?m)^body_offset:', s):
        s = re.sub(r'(?m)^body_offset:.*$', line, s)
    else:
        s = s.rstrip('\n') + f"""

# bbox 中心在 base_link 裡的位置 [x, y] (m): 投影出來的點 = p_base + R(yaw) @ body_offset。
# kros_car 車頭有手臂, 框的中心在原點前方; 這個偏移跟著車頭轉, 上面的單應性吸收不掉。
# **相機節點不會扣它** (它的 yaw 是速度方向推的, 原地轉時是舊值, 扣了沒有比較好);
# 有可信 yaw 的下游要扣 —— 融合節點的 camera_offset (fusion_loc.yaml) 要跟這裡一樣。
# 由 scripts/calibrate_camera_ground.py 量, 兩邊會一起寫。
{line}
"""
    s = re.sub(r'(?m)^(image_width:)', note + r'\n\1', s, count=1)
    open(path, 'w').write(s)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--cal', default=CAL_DEFAULT, help='現有的校正檔 (adjust 的起點, 也是 --write 的對象)')
    ap.add_argument('--model', default='adjust',
                    choices=['adjust', 'homography', 'homography+radial'])
    ap.add_argument('--min-conf', type=float, default=0.5)
    ap.add_argument('--write', action='store_true', help='寫回 --cal 與融合的 camera_offset')
    ap.add_argument('--fusion-yaml', default=FUSION_DEFAULT)
    ap.add_argument('--force', action='store_true', help='涵蓋範圍不夠也照樣整個重擬')
    args = ap.parse_args()

    cal = yaml.safe_load(open(args.cal))
    m = Model(cal)
    runs = [load(p, args.min_conf) for p in args.csv]
    d = {k: np.concatenate([r[k] for r in runs]) for k in runs[0]}
    print(f'校正檔: {args.cal}')
    cover = []
    for ax, name in enumerate('xy'):
        v = d['xy'[ax]]
        cover.append((v.max() - v.min()) / (ROOM[ax][1] - ROOM[ax][0]))
    print(f'資料: {len(d["px"])} 則偵測 ({len(args.csv)} 份 CSV), '
          f'x[{d["x"].min():+.2f},{d["x"].max():+.2f}] y[{d["y"].min():+.2f},{d["y"].max():+.2f}] m '
          f'= 房間的 {cover[0]:.0%} x {cover[1]:.0%}, 車頭轉過 {np.degrees(np.ptp(d["yaw"])):.0f}°')
    if len(d['px']) < 200:
        sys.exit('偵測太少。')
    if np.ptp(d['yaw']) < np.pi:
        print('  警告: 車頭轉不到半圈, body_offset 跟平移分不開 (兩個都是「往某個方向偏」)。'
              '讓車子原地轉一圈再錄。')
    if args.model != 'adjust' and min(cover) < 0.7 and not args.force:
        sys.exit(f'資料只涵蓋房間的 {cover[0]:.0%} x {cover[1]:.0%}, 不能整個重擬單應性 '
                 '(外推出去會差到幾十公分, 見檔頭)。用預設的 --model adjust, '
                 '或先用 scripts/tour_drive.py 繞場錄一份; 真的要硬來加 --force。')

    print('\n對 ground truth 的誤差 (cm):')
    print(f'  現在的校正檔, 當成 base_link    {stat(*m.resid("current", [], d))}')
    p_off = m.fit('offset', d)
    print(f'  現在的校正檔 + body_offset      {stat(*m.resid("offset", p_off, d))}')
    p = m.fit(args.model, d)
    H, k, b = m.unpack(args.model, p)
    print(f'  {args.model:<18s} (全資料)   {stat(*m.resid(args.model, p, d))}')
    cv = cross_val(m, args.model, d)
    print(f'  {args.model:<18s} (5 段交叉) {stat(*cv)}')
    print(f'  同上, 但下游不扣 body_offset    {stat(*m.resid(args.model, p, d, use_offset=False))}')
    if len(runs) > 1:
        print('  每一份 CSV 各自看 (用全部資料擬合的那一組):')
        for path, r in zip(args.csv, runs):
            print(f'    {os.path.basename(path):28s} {stat(*m.resid(args.model, p, r))}')

    sigma = float(np.sqrt((cv[0] ** 2 + cv[1] ** 2).mean()) / np.sqrt(2.0))
    print(f'\nbody_offset = ({b[0] * 100:+.2f}, {b[1] * 100:+.2f}) cm   '
          f'(bbox 中心在 base_link 的{"前" if b[0] > 0 else "後"}方)')
    if args.model == 'adjust':
        print(f'比例 {p[2]:.5f}, 平移 ({p[3] * 100:+.2f}, {p[4] * 100:+.2f}) cm')
    print(f'residual_rms (單軸 1-sigma = 交叉驗證的二維 RMS / sqrt(2)) = {sigma * 100:.2f} cm')

    if not args.write:
        print('\n(沒有改檔。確認數字合理之後加 --write)')
        return
    today = datetime.date.today().isoformat()
    note = (f'# {today} scripts/calibrate_camera_ground.py --model {args.model} '
            f'(n={len(d["px"])}, {", ".join(os.path.basename(c) for c in args.csv)}):\n'
            f'#   body_offset ({b[0] * 100:+.2f}, {b[1] * 100:+.2f}) cm'
            + (f', 比例 {p[2]:.5f}, 平移 ({p[3] * 100:+.2f}, {p[4] * 100:+.2f}) cm'
               if args.model == 'adjust' else '')
            + f'; 交叉驗證 {stat(*cv).split("   ")[0]}。\n'
            '#   下面 homography / distortion / residual_rms / body_offset 是這次的值;\n'
            '#   更上面的註解是舊車 (car.usd) 那一次擬合的紀錄, 數字已經不是現值。')
    rewrite_yaml(args.cal, H, k, b, sigma, note)
    print(f'\n已寫入 {args.cal}')
    if os.path.exists(args.fusion_yaml):
        s = open(args.fusion_yaml).read()
        s, n = re.subn(r'(?m)^(\s*)camera_offset:.*$',
                       rf'\g<1>camera_offset: [{b[0]:.4f}, {b[1]:.4f}]', s)
        if n == 1:
            open(args.fusion_yaml, 'w').write(s)
            print(f'已把 camera_offset 寫入 {args.fusion_yaml}')
        else:
            print(f'!! {args.fusion_yaml} 裡找不到 camera_offset, 請手動填 [{b[0]:.4f}, {b[1]:.4f}]')
    print('相機節點要重開才會讀到新的校正檔。')


if __name__ == '__main__':
    main()
