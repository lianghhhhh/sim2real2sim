#!/usr/bin/env python3
"""畫 nav_mpc_manager 規劃失敗時存的快照 (nav_fail_<t>.npz, 在 events_csv 的資料夾)。

    python3 src/nav_mpc_pkg/scripts/show_nav_fail.py car_run_data/nav_fail_186.7.npz          # 存成同名 .png
    python3 src/nav_mpc_pkg/scripts/show_nav_fail.py car_run_data/nav_fail_186.7.npz --show   # 開視窗

圖上 (圖內文字用英文, matplotlib 預設字型沒有中文): costmap (灰階; 紅 = 規劃器眼中的禁區, 也就是 2x2 降採樣取最大後 >= 99)、雷射點 (藍)、
車子 (綠箭頭; 有移過起點的話虛線箭頭)、目標 (紫箭頭)、原本的路線 (橘), 以及文字列出試過的起點 / 目標方向。
"""
import argparse
import math
import os

import numpy as np


def planner_cost(grid, f):
    """跟 SmacHybrid 的 downsample_costmap 一樣: f x f 格取最大, 再放大回原解析度方便疊圖。"""
    if f <= 1:
        return grid
    h, w = grid.shape
    H, W = -(-h // f), -(-w // f)
    pad = np.full((H * f, W * f), -1, dtype=grid.dtype)
    pad[:h, :w] = grid
    ds = pad.reshape(H, f, W, f).max(axis=(1, 3))
    return np.kron(ds, np.ones((f, f), dtype=grid.dtype))[:h, :w]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz')
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--radius', type=float, default=2.5, help='畫車子 / 目標附近多大 (m)')
    args = ap.parse_args()

    import matplotlib
    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    d = np.load(args.npz, allow_pickle=False)
    grid = d['grid'].astype(int)
    res = float(d['res'])
    ox, oy = d['origin']
    pose, goal, start = d['pose'], d['goal'], d['plan_start']
    f = int(d['planner_downsample'])
    pc = planner_cost(grid, f)
    h, w = grid.shape
    ext = [ox, ox + w * res, oy, oy + h * res]

    fig, ax = plt.subplots(figsize=(9, 9))
    img = np.where(grid < 0, 0.6, 1.0 - np.clip(grid, 0, 100) / 130.0)
    ax.imshow(img, origin='lower', extent=ext, cmap='gray', vmin=0, vmax=1)
    lethal = np.ma.masked_where(pc < 99, np.ones_like(pc))
    ax.imshow(lethal, origin='lower', extent=ext, cmap='autumn', alpha=0.35)
    if len(d['scan']):
        ax.plot(d['scan'][:, 0], d['scan'][:, 1], '.', ms=2, color='tab:blue', label='scan')
    if len(d['old_path']):
        ax.plot(d['old_path'][:, 0], d['old_path'][:, 1], '-', color='tab:orange', lw=1.5, label='old path')

    def arrow(p, color, ls='-', label=None):
        ax.annotate('', xy=(p[0] + 0.35 * math.cos(p[2]), p[1] + 0.35 * math.sin(p[2])), xytext=(p[0], p[1]),
                    arrowprops=dict(arrowstyle='->', color=color, lw=2, ls=ls))
        ax.plot(p[0], p[1], 'o', color=color, ms=6, label=label)

    arrow(pose, 'tab:green', label='car')
    if np.hypot(*(start[:2] - pose[:2])) > 0.01:
        arrow(start, 'tab:green', ls='--', label='plan start (shifted)')
    arrow(goal, 'tab:purple', label='goal')

    cx, cy = (pose[0] + goal[0]) / 2, (pose[1] + goal[1]) / 2
    r = max(args.radius, 0.6 * np.hypot(goal[0] - pose[0], goal[1] - pose[1]) + 0.8)
    ax.set_xlim(cx - r, cx + r)
    ax.set_ylim(cy - r, cy + r)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)

    def cost_at(g, p):
        i, j = int((p[0] - ox) / res), int((p[1] - oy) / res)
        return int(g[j, i]) if 0 <= i < w and 0 <= j < h else None

    tried = d['tried']
    ok = [f'({math.degrees(a):+.0f}°, {math.degrees(b):+.0f}°)' for a, b, s in tried if s]
    lines = [os.path.basename(args.npz),
             f"car ({pose[0]:+.2f}, {pose[1]:+.2f}, {math.degrees(pose[2]):+.0f}°) cost {cost_at(grid, pose)} "
             f"/ planner sees {cost_at(pc, start)}",
             f"goal ({goal[0]:+.2f}, {goal[1]:+.2f}, {math.degrees(goal[2]):+.0f}°) cost {cost_at(grid, goal)} "
             f"/ planner sees {cost_at(pc, goal)}",
             f'tried {len(tried)} (start, goal) heading offsets, ok {len(ok)}' + (f': {", ".join(ok[:6])}' if ok else ''),
             ]
    ax.set_title('\n'.join(lines), fontsize=8, loc='left')
    ax.legend(loc='lower right', fontsize=8)
    if args.show:
        plt.show()
    else:
        out = os.path.splitext(args.npz)[0] + '.png'
        fig.savefig(out, dpi=130, bbox_inches='tight')
        print(out)


if __name__ == '__main__':
    main()
