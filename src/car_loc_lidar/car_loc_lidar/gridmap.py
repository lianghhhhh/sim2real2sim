#!/usr/bin/env python3
"""2D 佔據格點地圖 + 距離場 (EDT)。

這個模組跟 ROS 完全無關, 所以可以在容器外面直接檢查地圖:

    python3 -m car_loc_lidar.gridmap show maps/room.yaml

座標約定 (整個 package 都一樣):
    世界座標 (x, y) 對應格點 (col, row)
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
    occ[row, col] = True 表示那一格上有東西 (牆 / 柱子)。

距離場 dist[row, col] = 「這一格中心離最近的被佔格中心有幾公尺」。掃描比對就是
在最小化每個雷射點落點的 dist —— 點落在牆上 dist=0, 落在空中 dist 就是它離牆
多遠。用 EDT 而不是 likelihood field 的好處是殘差本身就有公尺這個單位, 可以
直接看「平均差幾公分」, 調參跟除錯都不用猜。

精度上限要講清楚: 這個 package 的地圖是**用 slam_toolbox 建出來的**, 牆只精確到
半格 (預設 2.5 cm), 而且本來就帶著建圖時的位姿誤差。定位再怎麼準也不會超過
地圖本身的準度。
"""
from __future__ import annotations

import os
import sys

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:      # pragma: no cover - 容器裡一定有 scipy
    distance_transform_edt = None


def _read_pnm(path: str) -> np.ndarray:
    """讀 PGM (P2/P5), 回傳 (H, W) 的 uint8。不依賴 PIL / OpenCV。"""
    with open(path, 'rb') as f:
        data = f.read()
    if data[:2] not in (b'P5', b'P2'):
        raise ValueError(f'{path} 不是 PGM (開頭是 {data[:2]!r})')
    binary = data[:2] == b'P5'
    fields, i = [], 2
    while len(fields) < 3:                       # magic, width, height, maxval
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':                # 註解行整行跳掉
            while i < len(data) and data[i:i + 1] not in (b'\n', b'\r'):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j]))
        i = j
    w, h, maxval = fields
    i += 1                                       # header 之後固定一個空白字元
    if binary:
        dt = np.dtype('>u2') if maxval > 255 else np.uint8
        px = np.frombuffer(data, dtype=dt, count=w * h, offset=i).reshape(h, w)
    else:
        px = np.array(data[i:].split()[:w * h], dtype=np.int64).reshape(h, w)
    if maxval != 255:
        px = px.astype(np.float64) * 255.0 / maxval
    return px.astype(np.uint8)


def voxel_downsample(pts_xy: np.ndarray, size: float) -> np.ndarray:
    """每個格子只留一個點。滾動子圖每幾秒要重建一次距離場, 點數直接決定要多久。"""
    pts = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
    if pts.size == 0 or size <= 0:
        return pts
    q = np.floor(pts / size).astype(np.int64)
    _, idx = np.unique(q, axis=0, return_index=True)
    return pts[np.sort(idx)]


class GridMap:

    def __init__(self, occ: np.ndarray, resolution: float, origin,
                 meta: dict | None = None):
        self.occ = np.ascontiguousarray(np.asarray(occ).astype(bool))
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=np.float64).reshape(2)
        self.meta = dict(meta or {})
        self._dist = None

    # ------------------------------------------------------------------ 基本
    @property
    def shape(self):
        return self.occ.shape

    @property
    def n_occupied(self) -> int:
        return int(self.occ.sum())

    @property
    def bounds(self):
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
                    margin: float = 1.0, meta: dict | None = None) -> 'GridMap':
        pts = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
        if pts.size == 0:
            raise ValueError('沒有任何障礙點, 建不出地圖')
        lo = pts.min(axis=0) - margin
        hi = pts.max(axis=0) + margin
        origin = np.floor(lo / resolution) * resolution
        n = np.ceil((hi - origin) / resolution).astype(int) + 1
        g = cls(np.zeros((int(n[1]), int(n[0])), dtype=bool), resolution, origin, meta)
        g.insert(pts)
        return g

    @classmethod
    def empty(cls, bounds, resolution: float = 0.05,
              meta: dict | None = None) -> 'GridMap':
        x0, y0, x1, y1 = bounds
        w = int(np.ceil((x1 - x0) / resolution)) + 1
        h = int(np.ceil((y1 - y0) / resolution)) + 1
        return cls(np.zeros((h, w), dtype=bool), resolution,
                   [float(x0), float(y0)], meta)

    def insert(self, pts_xy: np.ndarray) -> int:
        """把障礙點寫進地圖 (超出邊界的丟掉)。回傳新增了幾格。"""
        pts = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
        if pts.size == 0:
            return 0
        col, row, ok = self._to_cell(pts)
        before = self.n_occupied
        self.occ[row[ok], col[ok]] = True
        self._dist = None
        return self.n_occupied - before

    def ensure_bounds(self, pts_xy: np.ndarray, margin: float = 1.0) -> bool:
        """地圖不夠大就長大 —— 滾動子圖不必事先知道場地多大。"""
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
    def dist(self) -> np.ndarray:
        if self._dist is None:
            if distance_transform_edt is None:
                raise RuntimeError('需要 scipy 才能算距離場')
            if not self.occ.any():
                self._dist = np.full(self.occ.shape, 1e3, dtype=np.float32)
            else:
                self._dist = distance_transform_edt(
                    ~self.occ, sampling=self.resolution).astype(np.float32)
        return self._dist

    def sample(self, pts_xy: np.ndarray, d_far: float = 5.0):
        """雙線性取樣距離場與梯度, 回傳 (d, gx, gy, valid)。

        落在地圖外的點 d = d_far、梯度 0、valid=False —— 這樣它們在最小平方裡
        不會產生任何拉力, 不會把位姿往地圖外拖。
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
        tx, ty = fxv - x0, fyv - y0
        x1, y1 = x0 + 1, y0 + 1

        D = self.dist
        d00, d10 = D[y0, x0], D[y0, x1]
        d01, d11 = D[y1, x0], D[y1, x1]
        top = d00 * (1 - tx) + d10 * tx
        bot = d01 * (1 - tx) + d11 * tx
        d[valid] = top * (1 - ty) + bot * ty

        # 雙線性的解析梯度 —— 用這個而不是另外存一張梯度圖, 值才會跟上面內插
        # 出來的完全一致, Gauss-Newton 才收斂得乾淨。
        inv = 1.0 / self.resolution
        gx[valid] = ((d10 - d00) * (1 - ty) + (d11 - d01) * ty) * inv
        gy[valid] = (bot - top) * inv
        return d, gx, gy, valid

    def free_mask(self, clearance: float) -> np.ndarray:
        """離障礙至少 clearance 公尺的格 —— 全域初始化只在這些格裡找。"""
        return self.dist >= clearance

    def occupied_points(self) -> np.ndarray:
        row, col = np.nonzero(self.occ)
        return np.stack([self.origin[0] + (col + 0.5) * self.resolution,
                         self.origin[1] + (row + 0.5) * self.resolution], axis=1)

    # ------------------------------------------------------------------ 存 / 讀
    def save(self, path: str) -> str:
        path = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        np.savez_compressed(path, occ=np.packbits(self.occ, axis=-1),
                            occ_width=np.int64(self.occ.shape[1]),
                            resolution=np.float64(self.resolution),
                            origin=self.origin,
                            meta=np.array(repr(self.meta)))
        return path

    @classmethod
    def load(cls, path: str) -> 'GridMap':
        """.npz (本 package 的格式) 或 .yaml (nav2 / map_saver_cli 的格式)。"""
        path = os.path.abspath(os.path.expanduser(path))
        if path.endswith(('.yaml', '.yml')):
            return cls.load_nav2(path)
        z = np.load(path, allow_pickle=False)
        occ = np.unpackbits(z['occ'], axis=-1, count=int(z['occ_width'])).astype(bool)
        meta = {}
        if 'meta' in z.files:
            try:
                meta = eval(str(z['meta']), {'__builtins__': {}})  # noqa: S307
            except Exception:
                meta = {}
        return cls(occ, float(z['resolution']), z['origin'], meta)

    @classmethod
    def load_nav2(cls, yaml_path: str) -> 'GridMap':
        """讀 map_saver_cli / slam_toolbox 存出來的 .yaml + .pgm。

        這是本 package 的**正常路徑** —— 地圖就是手動開車用 slam_toolbox 建的。
        """
        import yaml as _yaml
        yaml_path = os.path.abspath(os.path.expanduser(yaml_path))
        with open(yaml_path) as f:
            cfg = _yaml.safe_load(f)
        img_path = cfg['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        img = _read_pnm(img_path)

        origin = cfg.get('origin', [0.0, 0.0, 0.0])
        yaw = float(origin[2]) if len(origin) > 2 else 0.0
        p = img.astype(np.float64) / 255.0
        occupancy = p if int(cfg.get('negate', 0)) else (1.0 - p)
        # 影像 row0 是 y 最大, 地圖 row0 是 y 最小
        occ = np.flipud(occupancy > float(cfg.get('occupied_thresh', 0.65)))
        res = float(cfg['resolution'])
        meta = {'source': os.path.basename(yaml_path), 'format': 'nav2'}
        if abs(yaw) < 1e-9:
            return cls(occ, res, [float(origin[0]), float(origin[1])], meta=meta)
        meta['yaw'] = yaw
        return cls._resample_rotated(occ, res, (float(origin[0]), float(origin[1])),
                                     yaw, meta)

    @classmethod
    def _resample_rotated(cls, occ: np.ndarray, resolution: float, origin_xy,
                          yaw: float, meta: dict) -> 'GridMap':
        """origin 帶 yaw 的 nav2 地圖 -> 跟世界座標軸對齊的 GridMap。

        nav2 的定義: 影像格點 (col, row) 的中心在世界座標是
            origin_xy + R(yaw) @ ((col + 0.5) * res, (row + 0.5) * res)
        也就是整張圖繞左下角轉 yaw。本 package 的 GridMap 永遠軸對齊 (距離場、
        雙線性取樣都靠這個), 所以讀進來時重新取樣成一張新的軸對齊格點。

        用**反向查表**: 新格點的每一格中心轉回舊影像去查它落在哪一格。反過來
        「把每個佔據格轉過去」的話, 斜線上會漏格 (牆出現破洞) 或為了補洞把牆
        加粗 —— 加粗會讓牆面往空地移, 等於把房間縮小, 直接變成定位誤差。

        SLAM 地圖轉歪 (建圖起點的車頭沒對齊世界座標軸) 時, 就是靠這個在 yaml
        裡填 yaw 修正, 不用重建地圖。量法見 scripts/calibrate_map_origin.py。
        """
        h, w = occ.shape
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        o = np.asarray(origin_xy, dtype=np.float64)
        corners = np.array([[0, 0], [w, 0], [0, h], [w, h]], dtype=np.float64) * resolution
        wc = corners @ R.T + o
        lo, hi = wc.min(axis=0), wc.max(axis=0)
        n = np.ceil((hi - lo) / resolution).astype(int)
        cols, rows = np.meshgrid(np.arange(n[0]), np.arange(n[1]))
        centers = np.stack([lo[0] + (cols + 0.5) * resolution,
                            lo[1] + (rows + 0.5) * resolution], axis=-1).reshape(-1, 2)
        src = (centers - o) @ R                  # 每一列乘 R = 套用 R^T (轉回影像座標)
        sc = np.floor(src[:, 0] / resolution).astype(np.int64)
        sr = np.floor(src[:, 1] / resolution).astype(np.int64)
        ok = (sc >= 0) & (sc < w) & (sr >= 0) & (sr < h)
        new = np.zeros(centers.shape[0], dtype=bool)
        new[ok] = occ[sr[ok], sc[ok]]
        return cls(new.reshape(int(n[1]), int(n[0])), resolution, lo, meta=meta)

    def save_nav2(self, stem: str) -> str:
        """存成 .pgm + .yaml, rviz / nav2 / 肉眼都看得懂。"""
        stem = os.path.abspath(os.path.expanduser(stem))
        os.makedirs(os.path.dirname(stem) or '.', exist_ok=True)
        h, w = self.occ.shape
        img = np.flipud(np.where(self.occ, 0, 254).astype(np.uint8))
        with open(stem + '.pgm', 'wb') as f:
            f.write(b'P5\n# car_loc_lidar map\n')
            f.write(f'{w} {h}\n255\n'.encode())
            f.write(img.tobytes())
        with open(stem + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(stem)}.pgm\n'
                    f'resolution: {self.resolution}\n'
                    f'origin: [{self.origin[0]}, {self.origin[1]}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        return stem + '.pgm'

    def occupancy_data(self, seed_xy=None) -> np.ndarray:
        """nav_msgs/OccupancyGrid 的 data (int8, row-major, row0 = y 最小)。

        佔據 = 100, 可走 = 0, 其餘 = -1。給了 seed_xy 就從那一點 flood fill 找出
        真正走得到的空間, 其他標成未知 —— 這樣在 rviz 看到的才是「一個房間」,
        而不是一整片白色矩形中間畫幾條線。
        """
        data = np.full(self.occ.shape, -1, dtype=np.int8)
        done = False
        if seed_xy is not None:
            try:
                from scipy import ndimage
                lab, _ = ndimage.label(~self.occ)
                col, row, ok = self._to_cell(np.asarray(seed_xy, dtype=np.float64)
                                             .reshape(1, 2))
                if ok[0] and lab[row[0], col[0]] > 0:
                    data[lab == lab[row[0], col[0]]] = 0
                    done = True
            except ImportError:
                pass
        if not done:
            data[~self.occ] = 0
        data[self.occ] = 100
        return data

    # ------------------------------------------------------------------ 檢查
    def ascii_view(self, cols: int = 100) -> str:
        h, w = self.occ.shape
        step = max(1, int(np.ceil(w / cols)))
        rows = []
        for r in range(h - 1, -1, -step):        # y 由大到小, 印出來才跟俯視同向
            block = self.occ[max(0, r - step + 1):r + 1]
            rows.append(''.join('#' if block[:, c:c + step].any() else '.'
                                for c in range(0, w, step)))
        return '\n'.join(rows)


def _cli(argv):
    if len(argv) < 3:
        print('用法:\n'
              '  python3 -m car_loc_lidar.gridmap show <map.npz|map.yaml>\n'
              '  python3 -m car_loc_lidar.gridmap nav2 <map.npz> <輸出檔名(不含副檔名)>')
        return 1
    cmd, path = argv[1], argv[2]
    g = GridMap.load(path)
    if cmd == 'show':
        print(g)
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
