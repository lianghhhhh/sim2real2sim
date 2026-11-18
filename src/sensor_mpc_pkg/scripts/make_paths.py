#!/usr/bin/env python3
"""產生 kros_car_mpc_sensor.usd 房間用的多條循跡路線 (CSV + 總覽圖)。🖥 host 上跑, 只要 numpy / matplotlib。

    python3 src/sensor_mpc_pkg/scripts/make_paths.py            # 全部重產
    python3 src/sensor_mpc_pkg/scripts/make_paths.py e1 m2      # 只產這幾條
    -> src/sensor_mpc_pkg/config/paths/<名字>.csv
       src/sensor_mpc_pkg/config/paths/overview.png

每條路線都從車子出生點 (0, 0)、車頭 +x 出發, 用「直線 + 圓弧」接成 (跟預設路線同一種做法),
所以曲率是分段固定的, 每一段的半徑就是難度。用法 (open = 不封閉, 要 loop:=false):

    ros2 launch sensor_mpc_pkg sensor_mpc.launch.py \\
        path_file:=/workspaces/src/sensor_mpc_pkg/config/paths/m1_figure8_r10.csv loop:=true \\
        output_csv:=/workspaces/car_run_data/mpc_m1_figure8.csv

每條都檢查 (不過就不寫檔):
    離牆內側 >= 0.95 m           (牆 x ±4.93, y ±3.39; 圍籬 ±4.5 x ±3.0)
    離圓柱中心 >= 1.30 m         (圓柱 (2.10, -2.04) 半徑 0.5; 圍籬圓半徑 0.9)
    自己跟自己: 沿路徑相隔 > 3 m 的兩點距離 >= min_gap (除了刻意交叉的 8 字)
    封閉路線首尾位置與航向接得起來

點的間距: 直線 0.2 m, 圓弧每段轉 <= 8° (半徑 0.4 m 時約 0.056 m)。path_tracker 只在上一個最近點
±search_radius (20) 個點裡找, 點越密這個視窗 (m) 越小, 但 0.45 m/s、50 Hz 每次只走 1 cm, 夠用。
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, '..', 'config', 'paths')

WALL_X, WALL_Y = 4.93, 3.39
PILLAR, PILLAR_R = np.array([2.10, -2.04]), 0.50
WALL_MIN, PILLAR_MIN = 0.95, 1.30
STEP_LINE, STEP_ANG = 0.2, math.radians(8.0)
V_MAX, K_CURV = 0.45, 0.5                 # 只拿來估一圈幾秒 (mpc_config.json 的 v_max / curvature_slowdown)


class Turtle:
    """從 (0, 0) 朝 +x 出發, S(長度) 直線, L / R(角度°, 半徑) 左 / 右轉圓弧。"""

    def __init__(self):
        self.x, self.y, self.h = 0.0, 0.0, 0.0
        self.pts = [(0.0, 0.0)]
        self.kappa = []                  # 每一段 (pts[i] -> pts[i+1]) 的曲率, 左正
        self.turn = {'L': 0.0, 'R': 0.0}
        self.radii = []

    def S(self, length):
        n = max(1, int(math.ceil(length / STEP_LINE - 1e-9)))
        for i in range(1, n + 1):
            t = length * i / n
            self.pts.append((self.x + t * math.cos(self.h), self.y + t * math.sin(self.h)))
            self.kappa.append(0.0)
        self.x, self.y = self.pts[-1]
        return self

    def _arc(self, deg, radius, side):
        sgn = 1.0 if side == 'L' else -1.0
        cx = self.x - sgn * radius * math.sin(self.h)
        cy = self.y + sgn * radius * math.cos(self.h)
        a0 = math.atan2(self.y - cy, self.x - cx)
        sweep = math.radians(deg)
        n = max(2, int(math.ceil(sweep / STEP_ANG - 1e-9)))
        for i in range(1, n + 1):
            a = a0 + sgn * sweep * i / n
            self.pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
            self.kappa.append(sgn / radius)
        self.x, self.y = self.pts[-1]
        self.h += sgn * sweep
        self.turn[side] += deg
        self.radii.append(radius)
        return self

    def L(self, deg, radius):
        return self._arc(deg, radius, 'L')

    def R(self, deg, radius):
        return self._arc(deg, radius, 'R')


# ---------------------------------------------------------------------------- 路線
# (名字, 難度, 封閉, 說明, 自己跟自己的最小距離, 建構函式)
def e1(t):   # 逆時針跑道, 只有左轉
    t.S(1.8).L(180, 1.2).S(3.6).L(180, 1.2).S(1.8)


def e2(t):   # 順時針跑道, 只有右轉 (往西放, 避開圓柱)
    t.R(180, 1.0).S(2.9).R(180, 1.0).S(2.9)


def e3(t):   # 逆時針定半徑圓
    t.L(360, 1.2)


def e4(t):   # 順時針定半徑圓
    t.R(360, 1.0)


def e5(t):   # 不封閉: 直線 + 180° 迴轉 + 直線
    t.S(2.0).L(180, 1.0).S(4.0)


def m1(t):   # 8 字: 左圓接右圓, 在原點曲率直接反號
    t.L(360, 1.0).R(360, 1.0)


def m2(t):   # 逆時針跑道, 北邊直線換成 3 個 S 彎 (R0.7)
    r = 0.45 / math.sin(math.radians(40))          # 0.70 m, 讓 8 個 40° 剛好 3.6 m
    t.S(1.8).L(180, 1.2)
    t.L(40, r).R(80, r).L(80, r).R(80, r).L(40, r)
    t.L(180, 1.2).S(1.8)


def m3(t):   # L 形, R0.8 圓角, 5 個左轉 + 1 個右轉 (凹角)
    t.S(2.6).L(90, 0.8).S(0.6).L(90, 0.8).S(5.2).L(90, 0.8).S(2.8).L(90, 0.8)
    t.S(0.8).L(90, 0.8).S(0.6).R(90, 0.8).S(0.2)


def h1(t):   # 蛇行: 4 條車道 (間距 0.8 m), R0.4 的 180° 髮夾彎, 左右都有
    t.S(2.4).L(180, 0.4).S(4.8).R(180, 0.4).S(4.8).L(180, 0.4).S(4.8)
    t.L(90, 1.0).S(0.4).L(90, 1.0).S(2.4)


def h2(t):   # 跟 m3 同形狀, 圓角縮到 R0.4 (接近直角)
    t.S(3.0).L(90, 0.4).S(1.4).L(90, 0.4).S(6.0).L(90, 0.4).S(3.6).L(90, 0.4)
    t.S(1.6).L(90, 0.4).S(1.4).R(90, 0.4).S(0.6)


def h3(t):   # 不封閉: 向內螺旋, 每 90° 半徑縮小 (1.2 -> 0.4)
    for r in (1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4):
        t.L(90, r)


def h4(t):   # 混合賽道: R1.4 大彎、R0.5 右髮夾、西邊 R0.5 S 彎、最後 R1.2 右轉接回起點
    r = 2.8 / (8 * math.sin(math.radians(45)))     # 0.495 m, 讓 S 彎剛好 2.8 m
    t.S(1.4).L(90, 1.4).L(90, 1.0).S(1.2)
    t.L(90, 0.5).R(180, 0.5).L(90, 0.5).S(1.0)
    t.L(90, 1.0)
    t.L(45, r).R(90, r).L(90, r).R(90, r).L(45, r)
    t.L(90, 1.0).S(0.6).L(90, 0.6).S(0.6).R(90, 1.2)


PATHS = {
    'e1_oval_ccw_r12': ('簡單', True, '逆時針跑道, R1.2 左轉', 0.8, e1),
    'e2_oval_cw_r10': ('簡單', True, '順時針跑道, R1.0 右轉', 0.8, e2),
    'e3_circle_ccw_r12': ('簡單', True, '逆時針定圓 R1.2 (穩態左轉)', 0.8, e3),
    'e4_circle_cw_r10': ('簡單', True, '順時針定圓 R1.0 (穩態右轉)', 0.8, e4),
    'e5_uturn_open': ('簡單', False, '不封閉: 直線 + R1.0 迴轉 + 直線', 0.8, e5),
    'm1_figure8_r10': ('中等', True, '8 字, R1.0, 原點左右轉直接切換', 0.0, m1),
    'm2_slalom_r07': ('中等', True, '跑道 + 3 個 R0.7 S 彎', 0.8, m2),
    'm3_lshape_r08': ('中等', True, 'L 形, R0.8, 含一個右轉凹角', 0.8, m3),
    'h1_serpentine_r04': ('困難', True, '蛇行 4 車道, R0.4 髮夾彎', 0.6, h1),
    'h2_lshape_r04': ('困難', True, 'L 形, R0.4 近直角', 0.8, h2),
    'h3_spiral_open': ('困難', False, '不封閉: 向內螺旋 R1.2 -> 0.4', 0.35, h3),
    'h4_mixed_track': ('困難', True, '混合: R1.4 大彎、R0.5 右髮夾、R0.5 S 彎', 0.8, h4),
}


def build(name):
    level, loop, desc, gap, fn = PATHS[name]
    t = Turtle()
    fn(t)
    p = np.array(t.pts)
    k = np.array(t.kappa)
    if loop:
        err = np.linalg.norm(p[-1] - p[0])
        herr = abs(math.remainder(t.h, 2 * math.pi))
        assert err < 1e-6 and herr < 1e-6, f'{name}: 沒接起來 (位置差 {err:.4f} m, 航向差 {math.degrees(herr):.2f}°)'
        p = p[:-1]
    return p, k, t


def check(name, p, k, t):
    level, loop, desc, gap, _ = PATHS[name]
    q = np.r_[p, p[:1]] if loop else p
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)][:len(p)]
    total = seg.sum()
    d_wall = min(WALL_X - p[:, 0].max(), p[:, 0].min() + WALL_X, WALL_Y - p[:, 1].max(), p[:, 1].min() + WALL_Y)
    d_pillar = np.linalg.norm(p - PILLAR, axis=1).min()
    # 自己跟自己 (沿路徑相隔 > 3 m)
    D = np.linalg.norm(p[:, None] - p[None], axis=2)
    ds = np.abs(s[:, None] - s[None])
    if loop:
        ds = np.minimum(ds, total - ds)
    far = ds > 3.0
    self_gap = D[far].min() if far.any() else np.inf
    # 一圈 (或走完) 大概幾秒: v_ref = v_max / (1 + k * |曲率|)
    kk = np.abs(k[:len(seg)])
    lap = float(np.sum(seg * (1.0 + K_CURV * kk) / V_MAX))
    info = dict(n=len(p), length=total, d_wall=d_wall, d_pillar=d_pillar, self_gap=self_gap,
                r_min=min(t.radii), left=t.turn['L'], right=t.turn['R'], lap=lap,
                x=(p[:, 0].min(), p[:, 0].max()), y=(p[:, 1].min(), p[:, 1].max()))
    ok = d_wall >= WALL_MIN and d_pillar >= PILLAR_MIN and self_gap >= gap
    return ok, info


def write_csv(name, p, info):
    level, loop, desc, _, _ = PATHS[name]
    path = os.path.join(OUT_DIR, name + '.csv')
    with open(path, 'w') as f:
        f.write(f'# {name}: {desc} [{level}] | {"closed, loop:=true" if loop else "open, loop:=false"} | '
                f'{info["length"]:.1f} m, R_min {info["r_min"]:.1f} m, 左轉 {info["left"]:.0f}° 右轉 {info["right"]:.0f}° | '
                f'start (0,0) heading +x | kros_car_mpc_sensor.usd | made by scripts/make_paths.py\n')
        f.write('x,y,z\n')
        for x, y in p:
            f.write(f'{x:.4f},{y:.4f},0.0000\n')
    return path


def plot(results, path):
    import matplotlib
    matplotlib.use('Agg')
    matplotlib.rcParams['font.family'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    n = len(results)
    cols = 3
    rows = int(math.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5.2 * cols, 3.9 * rows), dpi=110)
    colors = {'簡單': '#2f6fb3', '中等': '#d08a1c', '困難': '#c0392b'}
    for ax, (name, p, info) in zip(axes.flat, results):
        level, loop, desc, _, _ = PATHS[name]
        ax.set_facecolor('#f6f6f6')
        ax.add_patch(Rectangle((-WALL_X, -WALL_Y), 2 * WALL_X, 2 * WALL_Y, fill=False, lw=2, ec='#444444'))
        ax.add_patch(Rectangle((-4.5, -3.0), 9.0, 6.0, fill=False, lw=0.8, ls=':', ec='#888888'))
        ax.add_patch(Circle(PILLAR, PILLAR_R, fc='#999999', ec='#444444'))
        ax.add_patch(Circle(PILLAR, 0.9, fill=False, lw=0.8, ls=':', ec='#888888'))
        q = np.r_[p, p[:1]] if loop else p
        ax.plot(q[:, 0], q[:, 1], '-', lw=1.8, color=colors[level])
        m = len(p) // 6
        for i in range(m, len(p) - 1, max(m, 1)):
            d = p[i + 1] - p[i]
            ax.annotate('', xy=p[i] + d / np.linalg.norm(d) * 0.25, xytext=p[i],
                        arrowprops=dict(arrowstyle='->', lw=1.2, color=colors[level]))
        ax.annotate('', xy=(0.7, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='->', lw=2, color='#222222'))
        ax.plot(0, 0, 's', ms=6, color='#222222')
        if not loop:
            ax.plot(p[-1, 0], p[-1, 1], 'o', ms=6, mfc='white', mec='#222222')
        ax.set_aspect('equal')
        ax.set_xlim(-5.2, 5.2)
        ax.set_ylim(-3.7, 3.7)
        ax.tick_params(labelsize=7)
        ax.grid(True, color='#dddddd', lw=0.5)
        ax.set_title(f'{name} [{level}]\n{desc} | {info["length"]:.1f} m, 約 {info["lap"]:.0f} s',
                     fontsize=9)
    for ax in list(axes.flat)[n:]:
        ax.axis('off')
    fig.suptitle('kros_car_mpc_sensor.usd 循跡路線 (黑方塊 = 出生點 (0,0) 朝 +x, 空心圓 = 不封閉路線終點, 點線 = 圍籬)',
                 fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(path)


def main(argv):
    names = [n for n in PATHS if not argv or any(n.startswith(a) for a in argv)]
    os.makedirs(OUT_DIR, exist_ok=True)
    results, bad = [], []
    print(f'{"名字":22s} {"難度":4s} {"長度":>6s} {"R_min":>5s} {"左轉°":>6s} {"右轉°":>6s} '
          f'{"離牆":>5s} {"離柱心":>6s} {"自距":>5s} {"約秒":>5s}')
    for name in names:
        p, k, t = build(name)
        ok, info = check(name, p, k, t)
        print(f'{name:22s} {PATHS[name][0]:4s} {info["length"]:6.1f} {info["r_min"]:5.1f} {info["left"]:6.0f} '
              f'{info["right"]:6.0f} {info["d_wall"]:5.2f} {info["d_pillar"]:6.2f} {info["self_gap"]:5.2f} '
              f'{info["lap"]:5.0f} {"" if ok else "  <-- 不合格"}')
        if ok:
            write_csv(name, p, info)
            results.append((name, p, info))
        else:
            bad.append(name)
    if results and not argv:
        try:
            plot(results, os.path.join(OUT_DIR, 'overview.png'))
            print('總覽圖', os.path.join(OUT_DIR, 'overview.png'))
        except ImportError:
            print('沒有 matplotlib, 跳過總覽圖')
    if bad:
        print('不合格 (沒寫檔):', ', '.join(bad))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
