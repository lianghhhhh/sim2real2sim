"""2D 點地圖的存檔格式與檢視工具。

為什麼要有這個檔:
    之前的做法是每次啟動用前 2 秒即時建圖, 建出來的地圖沒有人看過, 好壞完全
    憑運氣 —— 起跑位置不同、前 2 秒的配準品質不同, 每次的地圖都不一樣, 而
    「地圖是壞的」和「定位演算法是壞的」在結果上看起來一模一樣, 無從分辨。
    把地圖變成一個「存得下來、看得到、可以驗證」的東西, 這兩件事才分得開。

存檔格式 (.npz):
    points   (N, 2) float64   地圖點, 單位公尺, 在 frame_id 座標系下
    meta     JSON 字串        frame_id / grid / 建立時間 / 來源說明

用法:
    python3 -m car_navigation.gridmap show maps/room.npz      # 印出 ASCII 圖與統計
    python3 -m car_navigation.gridmap png  maps/room.npz out.png
"""
import json
import time

import numpy as np


def save(path, points, frame_id='map', grid=0.05, source='', extra=None):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    meta = {
        'frame_id': frame_id,
        'grid': float(grid),
        'created': time.strftime('%Y-%m-%d %H:%M:%S'),
        'source': source,
        'n_points': int(points.shape[0]),
    }
    if extra:
        meta.update(extra)
    np.savez_compressed(path, points=points, meta=json.dumps(meta, ensure_ascii=False))
    return meta


def load(path):
    """回傳 (points (N,2), meta dict)。"""
    with np.load(path, allow_pickle=False) as z:
        points = z['points'].astype(np.float64).reshape(-1, 2)
        meta = json.loads(str(z['meta']))
    if points.shape[0] == 0:
        raise ValueError(f'{path} 裡沒有任何地圖點')
    return points, meta


def stats(points, grid=0.05):
    """地圖的基本體檢數字。"""
    cells = np.unique(np.floor(points / grid).astype(np.int64), axis=0)
    lo, hi = points.min(axis=0), points.max(axis=0)
    # 「厚度」: 每個點到最近鄰的距離。真實牆面很薄, 鬼牆會讓這個值變大
    from scipy.spatial import cKDTree
    d = cKDTree(points).query(points, k=2)[0][:, 1]
    return {
        'n_points': int(points.shape[0]),
        'n_cells': int(cells.shape[0]),
        'extent': (float(hi[0] - lo[0]), float(hi[1] - lo[1])),
        'bounds': (float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])),
        'nn_median': float(np.median(d)),
        'nn_p95': float(np.percentile(d, 95)),
    }


def ascii_art(points, width=78):
    """把地圖畫成文字圖。看一眼就知道是不是像一個房間。"""
    lo, hi = points.min(axis=0), points.max(axis=0)
    span = np.maximum(hi - lo, 1e-6)
    # 終端機的字元高度大約是寬度的兩倍, 所以 y 方向要除以 2 才不會被拉長
    height = max(3, int(round(width * span[1] / span[0] / 2.0)))
    idx = ((points - lo) / span * [width - 1, height - 1]).astype(int)
    canvas = np.zeros((height, width), dtype=int)
    np.add.at(canvas, (idx[:, 1], idx[:, 0]), 1)
    ramp = ' .:-=+*#%@'
    out = []
    for row in canvas[::-1]:                      # y 往上為正
        out.append(''.join(ramp[min(int(v), len(ramp) - 1)] for v in row))
    return '\n'.join(out)


def _main():
    import sys
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    cmd, path = sys.argv[1], sys.argv[2]
    points, meta = load(path)
    if cmd == 'show':
        st = stats(points, meta.get('grid', 0.05))
        print(f"檔案: {path}")
        print(f"座標系: {meta['frame_id']}   建立於 {meta.get('created', '?')}")
        print(f"來源: {meta.get('source', '?')}")
        print(f"點數 {st['n_points']}  格數 {st['n_cells']}  "
              f"範圍 {st['extent'][0]:.2f} x {st['extent'][1]:.2f} m")
        print(f"邊界 x[{st['bounds'][0]:+.2f},{st['bounds'][2]:+.2f}] "
              f"y[{st['bounds'][1]:+.2f},{st['bounds'][3]:+.2f}]")
        print(f"點間距 中位數 {st['nn_median']:.3f} m  p95 {st['nn_p95']:.3f} m")
        print()
        print(ascii_art(points))
        print()
        print('檢查重點: 圖形應該像房間的俯視輪廓 (牆是細線, 不是一團霧)。')
        print('如果同一面牆出現兩條平行線, 就是建圖時位姿漂了, 那張地圖不能用。')
    elif cmd == 'png':
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        out = sys.argv[3] if len(sys.argv) > 3 else 'map.png'
        plt.figure(figsize=(10, 7))
        plt.scatter(points[:, 0], points[:, 1], s=1)
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.title(f"{path}  ({points.shape[0]} pts, frame={meta['frame_id']})")
        plt.savefig(out, dpi=130, bbox_inches='tight')
        print(f'已存到 {out}')
    else:
        print(f'不認得的指令 {cmd}; 可用: show / png')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(_main())
