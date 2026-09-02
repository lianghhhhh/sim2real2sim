#!/usr/bin/env python3
"""2D 佔據格點地圖 + 距離場 (EDT)。

這個模組只有「地圖」這件事, 跟 ROS 完全無關, 所以可以在容器外面直接
`python3 -m car_localization.gridmap show <map.npz>` 拿來檢查地圖。

座標約定 (整份 package 都一樣):
    世界座標 (x, y) 對應格點 (col, row)
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
    occ[row, col] = True 表示那一格上有東西 (牆 / 柱子)。

距離場 dist[row, col] 是「這一格中心離最近的被佔格中心有幾公尺」。掃描比對
就是在最小化每個雷射點落點的 dist —— 點落在牆上 dist=0, 落在空中 dist 就是
它離牆多遠。用 EDT 而不是 likelihood field 的好處是殘差本身就有公尺這個單位,
可以直接看「平均差幾公分」, 調參跟除錯都不用猜。
"""
from __future__ import annotations

import os
import sys

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:  # pragma: no cover - 容器裡一定有 scipy, 這只是給外面跑的保險
    distance_transform_edt = None


def _read_pnm(path: str) -> np.ndarray:
    """讀 PGM (P2/P5) 或 PBM, 回傳 (H, W) 的 uint8。不依賴 PIL/OpenCV。"""
    with open(path, 'rb') as f:
        data = f.read()
    if data[:2] not in (b'P5', b'P2'):
        raise ValueError(f'{path} 不是 PGM (開頭是 {data[:2]!r})')
    binary = data[:2] == b'P5'
    # 解析 header: magic, width, height, maxval, 中間可以夾註解與任意空白
    fields, i = [], 2
    while len(fields) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] not in (b'\n', b'\r'):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j]))
        i = j
    w, h, maxval = fields
    i += 1                                   # header 之後固定一個空白字元
    if binary:
        dt = np.dtype('>u2') if maxval > 255 else np.uint8
        px = np.frombuffer(data, dtype=dt, count=w * h, offset=i).reshape(h, w)
    else:
        px = np.array(data[i:].split()[:w * h], dtype=np.int64).reshape(h, w)
    if maxval != 255:
        px = (px.astype(np.float64) * 255.0 / maxval)
    return px.astype(np.uint8)


def voxel_downsample(pts_xy: np.ndarray, size: float) -> np.ndarray:
    """每個格子只留一個點。滾動子圖要重建距離場, 點數直接決定重建要多久。"""
    pts = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
    if pts.size == 0 or size <= 0:
        return pts
    q = np.floor(pts / size).astype(np.int64)
    _, idx = np.unique(q, axis=0, return_index=True)
    return pts[np.sort(idx)]


class GridMap:

    def __init__(self, occ: np.ndarray, resolution: float, origin,
                 meta: dict | None = None, dist: np.ndarray | None = None):
        self.occ = np.ascontiguousarray(occ.astype(bool))
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=np.float64).reshape(2)
        self.meta = dict(meta or {})
        self._dist = None if dist is None else np.ascontiguousarray(
            dist.astype(np.float32))
        self._exact = dist is not None
        self._grad = None

    # ------------------------------------------------------------------ 基本資訊
    @property
    def shape(self):
        return self.occ.shape

    @property
    def n_occupied(self) -> int:
        return int(self.occ.sum())

    @property
    def bounds(self):
        """(xmin, ymin, xmax, ymax) —— 地圖涵蓋的世界範圍。"""
        h, w = self.occ.shape
        return (float(self.origin[0]), float(self.origin[1]),
                float(self.origin[0] + w * self.resolution),
                float(self.origin[1] + h * self.resolution))

    def __repr__(self):
        h, w = self.occ.shape
        x0, y0, x1, y1 = self.bounds
        return (f'GridMap({w}x{h} @ {self.resolution:.3f} m, '
                f'x[{x0:+.2f},{x1:+.2f}] y[{y0:+.2f},{y1:+.2f}], '
                f'{self.n_occupied} occupied)')

    # ------------------------------------------------------------------ 建立
    @classmethod
    def from_points(cls, pts_xy: np.ndarray, resolution: float = 0.05,
                    margin: float = 1.0, meta: dict | None = None,
                    exact: bool = False) -> 'GridMap':
        """從一堆世界座標的障礙點建地圖。margin 是四周多留的空白 (m)。

        exact=True 會直接用這些點算距離場 (見 set_exact_distance), 精度好但慢;
        滾動子圖那種每幾秒重建一次的場合值得, 每幀重建就不值得。
        """
        pts = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
        if pts.size == 0:
            raise ValueError('沒有任何障礙點, 建不出地圖')
        lo = pts.min(axis=0) - margin
        hi = pts.max(axis=0) + margin
        origin = np.floor(lo / resolution) * resolution
        n = np.ceil((hi - origin) / resolution).astype(int) + 1
        occ = np.zeros((int(n[1]), int(n[0])), dtype=bool)
        g = cls(occ, resolution, origin, meta)
        g.insert(pts)
        if exact:
            g.set_exact_distance(pts)
        return g

    @classmethod
    def empty(cls, bounds, resolution: float = 0.05, meta: dict | None = None) -> 'GridMap':
        x0, y0, x1, y1 = bounds
        origin = np.array([x0, y0], dtype=np.float64)
        w = int(np.ceil((x1 - x0) / resolution)) + 1
        h = int(np.ceil((y1 - y0) / resolution)) + 1
        return cls(np.zeros((h, w), dtype=bool), resolution, origin, meta)

    def insert(self, pts_xy: np.ndarray) -> int:
        """把障礙點寫進地圖 (超出邊界的會被丟掉)。回傳新增了幾格。"""
        pts = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
        if pts.size == 0:
            return 0
        col, row, ok = self._to_cell(pts)
        col, row = col[ok], row[ok]
        before = self.n_occupied
        self.occ[row, col] = True
        self._dist = None
        self._exact = False
        self._grad = None
        return self.n_occupied - before

    def ensure_bounds(self, pts_xy: np.ndarray, margin: float = 1.0) -> bool:
        """地圖不夠大就長大。建圖時不必事先知道房間多大。"""
        pts = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
        if pts.size == 0:
            return False
        h, w = self.occ.shape
        lo = self.origin
        hi = self.origin + np.array([w, h]) * self.resolution
        need_lo = np.minimum(lo, pts.min(axis=0) - margin)
        need_hi = np.maximum(hi, pts.max(axis=0) + margin)
        if np.allclose(need_lo, lo) and np.allclose(need_hi, hi):
            return False
        new_origin = np.floor(need_lo / self.resolution) * self.resolution
        n = np.ceil((need_hi - new_origin) / self.resolution).astype(int) + 1
        occ = np.zeros((int(n[1]), int(n[0])), dtype=bool)
        off = np.round((self.origin - new_origin) / self.resolution).astype(int)
        occ[off[1]:off[1] + h, off[0]:off[0] + w] = self.occ
        self.occ = occ
        self.origin = new_origin
        self._dist = None
        self._exact = False
        self._grad = None
        return True

    def _to_cell(self, pts):
        idx = (pts - self.origin) / self.resolution
        col = np.floor(idx[:, 0]).astype(np.int64)
        row = np.floor(idx[:, 1]).astype(np.int64)
        h, w = self.occ.shape
        ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        return col, row, ok

    # ------------------------------------------------------------------ 距離場
    @property
    def exact(self) -> bool:
        """距離場是不是直接用表面點算的 (而不是格點化之後的 EDT)。"""
        return bool(self._exact)

    def set_exact_distance(self, surface_xy: np.ndarray, workers: int = -1) -> None:
        """用真正的表面取樣點 (不是格點中心) 算距離場。

        這一步是「差 3 公分」跟「差 3 毫米」的分界。EDT 量的是「離最近的被佔格
        中心多遠」, 而格點中心跟真正的牆面最多差半格 (2.5 cm)。更糟的是這個偏差
        不會互相抵銷 —— 整張地圖的格點是同一組, 對面的兩道牆會往同一個方向偏,
        所以誤差不是抖動而是定值偏移, 平均再多幀也消不掉。
        直接對表面點做最近鄰查詢就沒有這個問題, 地圖的幾何誤差降到取樣間距等級。
        """
        from scipy.spatial import cKDTree
        pts = np.asarray(surface_xy, dtype=np.float64).reshape(-1, 2)
        if pts.size == 0:
            raise ValueError('沒有表面點')
        h, w = self.occ.shape
        cx = self.origin[0] + (np.arange(w) + 0.5) * self.resolution
        cy = self.origin[1] + (np.arange(h) + 0.5) * self.resolution
        gx, gy = np.meshgrid(cx, cy)
        q = np.stack([gx.ravel(), gy.ravel()], axis=1)
        d, _ = cKDTree(pts).query(q, k=1, workers=workers)
        self._dist = d.reshape(h, w).astype(np.float32)
        self._exact = True
        self._grad = None

    @property
    def dist(self) -> np.ndarray:
        """每一格離最近障礙的距離 (公尺)。"""
        if self._dist is None:
            if distance_transform_edt is None:
                raise RuntimeError('需要 scipy 才能算距離場')
            if not self.occ.any():
                self._dist = np.full(self.occ.shape, 1e3, dtype=np.float32)
            else:
                d = distance_transform_edt(~self.occ, sampling=self.resolution)
                self._dist = d.astype(np.float32)
        return self._dist

    @property
    def grad(self):
        """距離場的梯度 (dd/dx, dd/dy), 單位是 m/m。"""
        if self._grad is None:
            gy, gx = np.gradient(self.dist, self.resolution)
            self._grad = (gx.astype(np.float32), gy.astype(np.float32))
        return self._grad

    def sample(self, pts_xy: np.ndarray, d_far: float = 5.0):
        """雙線性取樣距離場與其梯度。

        回傳 (d, gx, gy, valid)。落在地圖外的點 d = d_far、梯度 0、valid=False ——
        這樣它們在最小平方裡不會產生任何拉力, 不會把位姿往地圖外拖。
        """
        pts = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
        n = pts.shape[0]
        d = np.full(n, float(d_far), dtype=np.float64)
        gx = np.zeros(n, dtype=np.float64)
        gy = np.zeros(n, dtype=np.float64)
        if n == 0:
            return d, gx, gy, np.zeros(0, dtype=bool)

        h, w = self.occ.shape
        # 格點中心在 (col+0.5), 所以連續座標要先扣掉 0.5
        fx = (pts[:, 0] - self.origin[0]) / self.resolution - 0.5
        fy = (pts[:, 1] - self.origin[1]) / self.resolution - 0.5
        valid = (fx >= 0) & (fx <= w - 1.001) & (fy >= 0) & (fy <= h - 1.001)
        if not valid.any():
            return d, gx, gy, valid

        fxv, fyv = fx[valid], fy[valid]
        x0 = np.floor(fxv).astype(np.int64)
        y0 = np.floor(fyv).astype(np.int64)
        tx = fxv - x0
        ty = fyv - y0
        x1, y1 = x0 + 1, y0 + 1

        D = self.dist
        d00 = D[y0, x0]; d10 = D[y0, x1]
        d01 = D[y1, x0]; d11 = D[y1, x1]

        top = d00 * (1 - tx) + d10 * tx
        bot = d01 * (1 - tx) + d11 * tx
        d[valid] = top * (1 - ty) + bot * ty

        # 雙線性的解析梯度 —— 用這個而不是另外存一張梯度圖, 才會跟上面內插出來的
        # 值完全一致, Gauss-Newton 才收斂得乾淨。
        inv = 1.0 / self.resolution
        gx[valid] = ((d10 - d00) * (1 - ty) + (d11 - d01) * ty) * inv
        gy[valid] = (bot - top) * inv
        return d, gx, gy, valid

    def score(self, pts_xy: np.ndarray, sigma: float = 0.20) -> float:
        """likelihood field 分數, 給粗搜尋用 (越大越好, 已對點數正規化)。"""
        d, _, _, valid = self.sample(pts_xy)
        if not valid.any():
            return 0.0
        return float(np.exp(-0.5 * (d[valid] / sigma) ** 2).sum() / len(d))

    def occupied_points(self) -> np.ndarray:
        """所有被佔格的中心座標 (N, 2)。"""
        row, col = np.nonzero(self.occ)
        return np.stack([self.origin[0] + (col + 0.5) * self.resolution,
                         self.origin[1] + (row + 0.5) * self.resolution], axis=1)

    def free_mask(self, clearance: float) -> np.ndarray:
        """離障礙至少 clearance 公尺的格 —— 全域初始化時只在這些格裡找。"""
        return self.dist >= clearance

    # ------------------------------------------------------------------ 存 / 讀
    def save(self, path: str) -> str:
        path = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        extra = {}
        if self._exact and self._dist is not None:
            extra['dist'] = self._dist          # 精確距離場, 讀回來就不用重算
        np.savez_compressed(path, occ=np.packbits(self.occ, axis=-1),
                            occ_width=np.int64(self.occ.shape[1]),
                            resolution=np.float64(self.resolution),
                            origin=self.origin,
                            meta=np.array(repr(self.meta)), **extra)
        return path

    @classmethod
    def load(cls, path: str) -> 'GridMap':
        """.npz (本 package 的格式) 或 .yaml (nav2 / slam_toolbox 的格式)。"""
        path = os.path.abspath(os.path.expanduser(path))
        if path.endswith(('.yaml', '.yml')):
            return cls.load_nav2(path)
        z = np.load(path, allow_pickle=False)
        occ = np.unpackbits(z['occ'], axis=-1, count=int(z['occ_width'])).astype(bool)
        meta = {}
        if 'meta' in z:
            try:
                meta = eval(str(z['meta']), {'__builtins__': {}})  # noqa: S307 - 自己寫的 repr
            except Exception:
                meta = {}
        dist = np.asarray(z['dist']) if 'dist' in z.files else None
        return cls(occ, float(z['resolution']), z['origin'], meta, dist=dist)

    @classmethod
    def load_nav2(cls, yaml_path: str) -> 'GridMap':
        """讀 nav2 / slam_toolbox / map_saver_cli 存出來的 .yaml + 影像。

        這條路是給「地圖不是從 USD 幾何算出來」的場合用的 —— 實體環境跑 SLAM
        建完圖之後, 直接把 map_saver_cli 的產物餵進來就能定位。

        注意精度上限: 這種地圖的牆只精確到半格 (預設 2.5 cm), 而且本來就帶著
        建圖時的位姿誤差。它不會比 make_map_from_usd.py 產生的地圖準, 但在真實
        環境裡你沒有 USD 檔可以切, 這是唯一的選擇。
        """
        import yaml as _yaml

        yaml_path = os.path.abspath(os.path.expanduser(yaml_path))
        with open(yaml_path) as f:
            cfg = _yaml.safe_load(f)
        img_path = cfg['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        img = _read_pnm(img_path)

        res = float(cfg['resolution'])
        origin = cfg.get('origin', [0.0, 0.0, 0.0])
        if len(origin) > 2 and abs(float(origin[2])) > 1e-6:
            raise ValueError(
                f'{yaml_path} 的 origin 帶了 yaw={origin[2]}, 這裡沒有支援 '
                '(整張圖要先旋轉)。請用 yaw=0 的地圖。')
        negate = int(cfg.get('negate', 0))
        occ_th = float(cfg.get('occupied_thresh', 0.65))

        p = img.astype(np.float64) / 255.0
        occupancy = p if negate else (1.0 - p)
        occ = np.flipud(occupancy > occ_th)      # 影像 row0 是 y 最大, 地圖 row0 是 y 最小
        return cls(occ, res, [float(origin[0]), float(origin[1])],
                   meta={'source': os.path.basename(yaml_path), 'format': 'nav2'})

    def occupancy_data(self, seed_xy=None) -> np.ndarray:
        """轉成 nav_msgs/OccupancyGrid 的 data (int8, row-major, row0 = y 最小)。

        佔據 = 100, 可走 = 0, 其餘 = -1 (未知)。給了 seed_xy 就從那一點 flood fill
        找出真正走得到的空間, 其他地方標成未知 —— 這樣在 rviz / Foxglove 看到的
        才是「一個房間」, 而不是一整片白色矩形中間畫幾條線。
        """
        data = np.full(self.occ.shape, -1, dtype=np.int8)
        if seed_xy is not None:
            try:
                from scipy import ndimage
                lab, _ = ndimage.label(~self.occ)
                col, row, ok = self._to_cell(np.asarray(seed_xy).reshape(1, 2))
                if ok[0] and lab[row[0], col[0]] > 0:
                    data[lab == lab[row[0], col[0]]] = 0
                else:
                    data[~self.occ] = 0
            except ImportError:
                data[~self.occ] = 0
        else:
            data[~self.occ] = 0
        data[self.occ] = 100
        return data

    def save_nav2(self, stem: str) -> str:
        """順便存一份 nav2 / rviz 看得懂的 .pgm + .yaml。

        只是為了方便肉眼檢查與之後接 nav2; 本 package 自己定位是讀 .npz。
        """
        stem = os.path.abspath(os.path.expanduser(stem))
        os.makedirs(os.path.dirname(stem) or '.', exist_ok=True)
        h, w = self.occ.shape
        img = np.where(self.occ, 0, 254).astype(np.uint8)
        img = np.flipud(img)                    # PGM 由上往下, 地圖 row0 是 y 最小
        with open(stem + '.pgm', 'wb') as f:
            f.write(b'P5\n# car_localization ground-truth map\n')
            f.write(f'{w} {h}\n255\n'.encode())
            f.write(img.tobytes())
        with open(stem + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(stem)}.pgm\n'
                    f'resolution: {self.resolution}\n'
                    f'origin: [{self.origin[0]}, {self.origin[1]}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        return stem + '.pgm'

    # ------------------------------------------------------------------ 檢查
    def ascii_view(self, cols: int = 100) -> str:
        h, w = self.occ.shape
        step = max(1, int(np.ceil(w / cols)))
        rows = []
        for r in range(h - 1, -1, -step):       # y 由大到小, 印出來才跟俯視圖同向
            block = self.occ[max(0, r - step + 1):r + 1]
            line = ''.join('#' if block[:, c:c + step].any() else '.'
                           for c in range(0, w, step))
            rows.append(line)
        return '\n'.join(rows)


def _cli(argv):
    if len(argv) < 3:
        print('用法:\n'
              '  python3 -m car_localization.gridmap show <map.npz>\n'
              '  python3 -m car_localization.gridmap nav2 <map.npz> <輸出檔名(不含副檔名)>')
        return 1
    cmd, path = argv[1], argv[2]
    g = GridMap.load(path)
    if cmd == 'show':
        print(g)
        print(f'  距離場: {"表面點精確值" if g.exact else "格點 EDT (有 ~半格的量化偏差)"}')
        for k, v in sorted(g.meta.items()):
            print(f'  {k}: {v}')
        print(g.ascii_view())
        return 0
    if cmd == 'nav2':
        print('已寫出', g.save_nav2(argv[3]))
        return 0
    print('不認得的指令', cmd)
    return 1


if __name__ == '__main__':
    sys.exit(_cli(sys.argv))
