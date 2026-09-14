#!/usr/bin/env python3
"""地圖 yaml 的 origin 帶 yaw 時, 讀進來的牆是不是在對的位置。不需要 ROS。

    cd src/car_loc_lidar && python3 test/test_gridmap_yaw.py

檢查三件事 (合成一個 10 x 6 m 的房間 + 柱子, 存成 nav2 格式, 再改 yaml 的 yaw 讀回來):
  1. 位置: 讀回來的每個佔據格, 離「理論上轉過去的牆」多遠 —— 要在一格以內
  2. 沒有破洞: 理論上的牆點每一個附近都要有佔據格 (反向查表的重點)
  3. 沒有變粗: 佔據格數量跟原圖差不多 (變粗 = 牆面往空地移 = 房間變小)
另外 yaw=0 要跟原本的讀法一模一樣; 有裝 scipy 的話再看距離場在牆上是不是 ~0。
"""
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from car_loc_lidar.gridmap import GridMap  # noqa: E402

RES = 0.05


def room_points():
    pts = []
    for x in np.arange(-5.0, 5.0 + 1e-9, RES / 2):
        pts += [(x, -3.0), (x, 3.0)]
    for y in np.arange(-3.0, 3.0 + 1e-9, RES / 2):
        pts += [(-5.0, y), (5.0, y)]
    for a in np.linspace(0, 2 * np.pi, 80, endpoint=False):     # 柱子, 非對稱才有意義
        pts.append((1.5 + 0.2 * np.cos(a), 0.8 + 0.2 * np.sin(a)))
    return np.array(pts)


def nearest(a, b, chunk=2000):
    """a 的每一點到 b 最近的距離 (不依賴 scipy)。"""
    out = np.empty(len(a))
    for i in range(0, len(a), chunk):
        d = a[i:i + chunk, None, :] - b[None, :, :]
        out[i:i + chunk] = np.sqrt((d ** 2).sum(-1)).min(axis=1)
    return out


def write_yaml(stem, origin):
    with open(stem + '.yaml') as f:
        lines = f.read().splitlines()
    lines = [f'origin: [{origin[0]}, {origin[1]}, {origin[2]}]' if l.startswith('origin:')
             else l for l in lines]
    with open(stem + '.yaml', 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    base = GridMap.from_points(room_points(), RES, margin=0.5)
    ok = True
    with tempfile.TemporaryDirectory() as tmp:
        stem = os.path.join(tmp, 'room')
        base.save_nav2(stem)
        o = base.origin

        g0 = GridMap.load(stem + '.yaml')
        same = (g0.occ.shape == base.occ.shape and (g0.occ == base.occ).all()
                and np.allclose(g0.origin, base.origin))
        print(f'yaw=0 跟原本一模一樣: {same}')
        ok &= same

        for yaw_deg in (1.0, -1.3, 30.0, 90.0):
            yaw = np.radians(yaw_deg)
            write_yaml(stem, (o[0], o[1], yaw))
            g = GridMap.load(stem + '.yaml')
            R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            expect = (base.occupied_points() - o) @ R.T + o     # nav2: 繞左下角轉
            got = g.occupied_points()
            d_pos = nearest(got, expect)            # 讀回來的格 -> 理論牆
            d_hole = nearest(expect, got)           # 理論牆 -> 讀回來的格
            ratio = g.n_occupied / base.n_occupied
            line = (f'yaw {yaw_deg:+6.1f}°: 位置 中位 {np.median(d_pos) * 100:4.2f} / 最大 '
                    f'{d_pos.max() * 100:4.2f} cm | 破洞 最大 {d_hole.max() * 100:4.2f} cm | '
                    f'格數比 {ratio:.3f}')
            passed = d_pos.max() <= RES * 1.0 and d_hole.max() <= RES * 1.0 and 0.9 <= ratio <= 1.15
            try:
                d, _, _, valid = g.sample(expect)
                line += f' | 距離場在牆上 中位 {np.median(d[valid]) * 100:4.2f} cm'
                passed &= bool(valid.all()) and np.median(d[valid]) <= RES / 2
            except RuntimeError:
                pass                                  # 沒有 scipy 就跳過距離場那一項
            print(line + ('  PASS' if passed else '  FAIL'))
            ok &= passed

    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
