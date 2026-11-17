#!/usr/bin/env python3
"""產生 kros_car_mpc_sensor.usd 房間用的循跡路線 (CSV + 預覽圖)。🖥 host 上跑, 只要 numpy / matplotlib。

    python3 src/sensor_mpc_pkg/scripts/make_path_kros_car_mpc_sensor.py
    -> src/sensor_mpc_pkg/config/path_kros_car_mpc_sensor.csv
       src/sensor_mpc_pkg/config/path_kros_car_mpc_sensor.png

房間 (kros_car_mpc_sensor.usd, 2026-11-17 離線讀出來的):
    牆內側  x [-4.93, +4.93]  y [-3.39, +3.39]
    圓柱    中心 (+2.10, -2.04) 半徑 0.50
    車子    出生在 (0, 0), 車頭朝 +x
舊路線 (kros_car_mpc.usd 搬過來的) 最低到 y = -4.81, 在南牆外面, 所以會撞牆。

新路線: 逆時針一圈 20.9 m, 從 (0, 0) 朝 +x 出發, 由直線與半徑 R 的圓弧接成 (曲率連續的地方是
直線-圓弧, 跟舊路線的彎差不多緊):
    A  (0,0) -> (2.4,0)            直線 +x
    B  半圓, 圓心 (2.4, 1.2)        右端最遠到 x = +3.6, 轉到朝 -x
    C  (2.4,2.4) -> (-2.4,2.4)      直線 -x (北邊)
    D  1/4 圓, 圓心 (-2.4, 1.2)     轉到朝 -y, 左端 x = -3.6
    E  (-3.6,1.2) -> (-3.6,-1.2)    直線 -y (西邊)
    F  半圓, 圓心 (-2.4, -1.2)      最低到 y = -2.4, 轉到朝 +y
    G  1/4 圓往右, 圓心 (0, -1.2)   S 形接回 (0, 0) 朝 +x
離牆最近 0.99 m (北 / 南), 離圓柱表面最近 1.54 m。點每 0.2 m 一個 (圓弧上相鄰兩段轉 9.5°)。
"""
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'config', 'path_kros_car_mpc_sensor')

WALL_X, WALL_Y = 4.93, 3.39                 # 牆內側
PILLAR, PILLAR_R = (2.10, -2.04), 0.50
R = 1.2
STEP = 0.2


def arc(center, radius, a0, a1, n):
    """從角度 a0 到 a1 (rad) 的圓弧, n 個點 (不含起點)。"""
    a = np.linspace(a0, a1, n + 1)[1:]
    return np.c_[center[0] + radius * np.cos(a), center[1] + radius * np.sin(a)]


def line(p0, p1, n):
    t = np.linspace(0.0, 1.0, n + 1)[1:, None]
    return np.asarray(p0) + t * (np.asarray(p1) - np.asarray(p0))


def n_for(length):
    return max(1, int(round(length / STEP)))


def build():
    pts = [np.array([[0.0, 0.0]])]
    pts.append(line((0, 0), (2.4, 0), n_for(2.4)))                                  # A
    pts.append(arc((2.4, 1.2), R, -math.pi / 2, math.pi / 2, n_for(math.pi * R)))   # B
    pts.append(line((2.4, 2.4), (-2.4, 2.4), n_for(4.8)))                           # C
    pts.append(arc((-2.4, 1.2), R, math.pi / 2, math.pi, n_for(math.pi / 2 * R)))   # D
    pts.append(line((-3.6, 1.2), (-3.6, -1.2), n_for(2.4)))                         # E
    pts.append(arc((-2.4, -1.2), R, math.pi, 2 * math.pi, n_for(math.pi * R)))      # F
    pts.append(arc((0.0, -1.2), R, math.pi, math.pi / 2, n_for(math.pi / 2 * R)))   # G (往右轉)
    p = np.vstack(pts)
    if np.linalg.norm(p[0] - p[-1]) < 1e-6:
        p = p[:-1]                  # 封閉: 最後一點就是起點, 不重複
    return p


def check(p):
    seg = np.linalg.norm(np.diff(np.r_[p, p[:1]], axis=0), axis=1)
    ang = np.arctan2(*np.diff(np.r_[p, p[:1]], axis=0).T[::-1])
    corner = np.abs(np.angle(np.exp(1j * np.diff(np.r_[ang, ang[:1]]))))
    d_wall = np.min([WALL_X - p[:, 0].max(), p[:, 0].min() + WALL_X,
                     WALL_Y - p[:, 1].max(), p[:, 1].min() + WALL_Y])
    d_pillar = np.linalg.norm(p - PILLAR, axis=1).min() - PILLAR_R
    print(f'{len(p)} 點, 總長 {seg.sum():.2f} m, 間距 {seg.min():.3f}~{seg.max():.3f} m, '
          f'最尖轉角 {math.degrees(corner.max()):.1f}°')
    print(f'x [{p[:, 0].min():+.2f}, {p[:, 0].max():+.2f}]  y [{p[:, 1].min():+.2f}, {p[:, 1].max():+.2f}]')
    print(f'離牆最近 {d_wall:.2f} m, 離圓柱表面最近 {d_pillar:.2f} m')
    assert d_wall > 0.8 and d_pillar > 0.8


def write_csv(p, path):
    with open(path, 'w') as f:
        f.write(f'# kros_car_mpc_sensor.usd 的循跡路線 ({len(p)} points, world frame, metres, closed), '
                f'made by scripts/make_path_kros_car_mpc_sensor.py\n')
        f.write('x,y,z\n')
        for x, y in p:
            f.write(f'{x:.4f},{y:.4f},0.0000\n')


def plot(p, path, old=None):
    import matplotlib
    matplotlib.use('Agg')
    matplotlib.rcParams['font.family'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'Noto Serif CJK TC',
                                          'Noto Serif CJK JP', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    fig, ax = plt.subplots(figsize=(9, 6.6), dpi=130)
    ax.set_facecolor('#f6f6f6')
    ax.add_patch(Rectangle((-WALL_X, -WALL_Y), 2 * WALL_X, 2 * WALL_Y, fill=False, lw=3, ec='#444444'))
    ax.add_patch(Circle(PILLAR, PILLAR_R, fc='#999999', ec='#444444'))
    ax.text(PILLAR[0], PILLAR[1] + 0.65, '圓柱', ha='center', fontsize=9, color='#444444')
    if old is not None:
        o = np.r_[old, old[:1]]
        ax.plot(o[:, 0], o[:, 1], '--', lw=1.5, color='#c0504d', label='舊路線 (kros_car_mpc.usd)')
    q = np.r_[p, p[:1]]
    ax.plot(q[:, 0], q[:, 1], '-', lw=2, color='#2f6fb3', label='新路線')
    ax.plot(p[:, 0], p[:, 1], '.', ms=4, color='#2f6fb3')
    ax.annotate('', xy=(0.9, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='->', lw=2, color='#2f6fb3'))
    ax.plot(0, 0, 's', ms=9, color='#222222', label='車子出生點 (0, 0), 車頭 +x')
    ax.set_aspect('equal')
    ax.set_xlim(-5.6, 5.6)
    ax.set_ylim(-5.2, 4.0)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(True, color='#dddddd', lw=0.6)
    ax.legend(loc='lower right', fontsize=9, framealpha=0.95)
    ax.set_title('kros_car_mpc_sensor.usd: 房間 (牆內側) 與循跡路線', fontsize=11)
    fig.tight_layout()
    fig.savefig(path)
    print('預覽圖', path)


if __name__ == '__main__':
    p = build()
    check(p)
    write_csv(p, OUT + '.csv')
    print('CSV', OUT + '.csv')
    old_csv = os.path.join(HERE, '..', 'config', 'path_kros_car_mpc.csv')
    old = None
    if os.path.exists(old_csv):
        old = np.loadtxt(old_csv, delimiter=',', skiprows=2)[:, :2]
    try:
        plot(p, OUT + '.png', old)
    except ImportError:
        print('沒有 matplotlib, 跳過預覽圖')
