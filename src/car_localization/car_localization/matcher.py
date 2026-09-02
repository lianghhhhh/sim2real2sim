#!/usr/bin/env python3
"""掃描對地圖 (scan-to-map) 的配準。

輸入的點已經是「以車體原點為中心、但已經轉到世界座標軸向」的 2D 點集
(見 localizer.py 怎麼算出來的), 所以這裡要解的只剩下:

    world_xy_i = t + Rz(delta) @ base_xy_i

其中 t 是車子在地圖裡的位置, delta 是 yaw 的殘餘修正量。
在 Isaac 裡 IMU 的 orientation 是精確的, delta 直接鎖成 0 (lock_yaw=True),
問題退化成兩個自由度 —— 這也是為什麼這個做法可以做到公分級。

代價函數是「每個點到最近障礙的距離」的 Huber 加權平方和, 距離直接從地圖的
EDT 雙線性內插出來, 所以不需要每次迭代重找對應點 (ICP 最慢也最容易錯的一步)。
"""
from __future__ import annotations

import math

import numpy as np


def rot2(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


class MatchResult:
    __slots__ = ('t', 'delta', 'residual', 'inlier_ratio', 'n_points',
                 'iterations', 'converged', 'cov')

    def __init__(self, t, delta, residual, inlier_ratio, n_points,
                 iterations, converged, cov):
        self.t = np.asarray(t, dtype=np.float64)
        self.delta = float(delta)
        self.residual = float(residual)          # inlier 的平均距離 (m)
        self.inlier_ratio = float(inlier_ratio)
        self.n_points = int(n_points)
        self.iterations = int(iterations)
        self.converged = bool(converged)
        self.cov = cov                           # 3x3 (x, y, yaw), 可能是 None

    def __repr__(self):
        return (f'MatchResult(t=[{self.t[0]:+.4f},{self.t[1]:+.4f}], '
                f'delta={math.degrees(self.delta):+.3f} deg, '
                f'residual={self.residual * 100:.2f} cm, '
                f'inlier={self.inlier_ratio:.2f}, n={self.n_points})')


class ScanMatcher:

    def __init__(self, gmap, huber: float = 0.10, max_iter: int = 30,
                 tol: float = 1e-5, inlier_dist: float = 0.30,
                 d_far: float = 3.0, lm_lambda: float = 1e-6):
        self.map = gmap
        self.huber = float(huber)          # 超過這個距離的點改用線性代價 (擋離群點)
        self.max_iter = int(max_iter)
        self.tol = float(tol)
        self.inlier_dist = float(inlier_dist)
        self.d_far = float(d_far)
        self.lm_lambda = float(lm_lambda)

    # ------------------------------------------------------------------ 精配準
    def _cost(self, base, t, delta, ndim):
        """回傳 (cost, r, J, valid_count)。cost 是 Huber 代價的總和。"""
        R = rot2(delta)
        rot = base @ R.T
        d, gx, gy, valid = self.map.sample(rot + t, d_far=self.d_far)
        nv = int(valid.sum())
        if nv < 10:
            return float('inf'), None, None, nv
        r = d[valid]
        J = np.empty((r.size, ndim), dtype=np.float64)
        J[:, 0] = gx[valid]
        J[:, 1] = gy[valid]
        if ndim == 3:
            pr = rot[valid]
            # d/d(delta) 造成的位移是 Rz'(delta) @ base, 也就是 rot 轉 90 度
            J[:, 2] = gx[valid] * (-pr[:, 1]) + gy[valid] * pr[:, 0]
        a = np.abs(r)
        cost = float(np.sum(np.where(a <= self.huber, 0.5 * r ** 2,
                                     self.huber * (a - 0.5 * self.huber))))
        return cost, r, J, nv

    def refine(self, base_xy: np.ndarray, t0, delta0: float = 0.0,
               lock_yaw: bool = True) -> MatchResult:
        base = np.asarray(base_xy, dtype=np.float64).reshape(-1, 2)
        t = np.asarray(t0, dtype=np.float64).reshape(2).copy()
        delta = float(delta0)
        n = base.shape[0]
        if n < 10:
            return MatchResult(t, delta, float('inf'), 0.0, n, 0, False, None)

        ndim = 2 if lock_yaw else 3
        lam = self.lm_lambda
        cost, r, J, nv = self._cost(base, t, delta, ndim)
        if not np.isfinite(cost):
            return MatchResult(t, delta, float('inf'), 0.0, n, 0, False, None)

        H = None
        it = 0
        converged = False
        for it in range(1, self.max_iter + 1):
            # Huber 的權重: 近的點用平方 (拉得準), 遠的離群點只給常數拉力,
            # 不會被少數打到別的東西的點綁架。
            a = np.abs(r)
            w = np.where(a <= self.huber, 1.0, self.huber / np.maximum(a, 1e-9))
            Jw = J * w[:, None]
            H = J.T @ Jw
            g = Jw.T @ r
            scale = max(np.trace(H) / ndim, 1e-9)

            accepted = False
            for _ in range(8):        # Levenberg-Marquardt: 試到這一步真的變好為止
                try:
                    step = -np.linalg.solve(H + lam * scale * np.eye(ndim), g)
                except np.linalg.LinAlgError:
                    break
                # 距離場只在障礙附近才有意義, 一次跳太遠會跳到別的牆上
                sn = float(np.linalg.norm(step[:2]))
                if sn > 0.5:
                    step = step * (0.5 / sn)
                    sn = 0.5
                nt = t + step[:2]
                nd = delta + (float(np.clip(step[2], -0.2, 0.2)) if ndim == 3 else 0.0)
                nc, nr, nJ, nnv = self._cost(base, nt, nd, ndim)
                if nc < cost:
                    t, delta, cost, r, J = nt, nd, nc, nr, nJ
                    lam = max(lam * 0.3, 1e-9)
                    accepted = True
                    break
                lam = min(lam * 10.0, 1e4)
            if not accepted:
                converged = True      # 已經沒有更好的方向了
                break
            if sn < self.tol and (ndim == 2 or abs(step[2]) < self.tol):
                converged = True
                break

        pts = base @ rot2(delta).T + t
        d, _, _, valid = self.map.sample(pts, d_far=self.d_far)
        inl = valid & (d < self.inlier_dist)
        residual = float(d[inl].mean()) if inl.any() else float('inf')
        ratio = float(inl.sum()) / max(n, 1)

        cov = None
        if H is not None and inl.sum() > 20:
            # sigma^2 * H^-1 —— 只當作「這次配準有多可信」的粗略指標, 不是嚴謹的後驗
            try:
                s2 = max(residual, 0.005) ** 2
                Hi = np.linalg.inv(H + 1e-9 * np.eye(H.shape[0]))
                cov = np.zeros((3, 3))
                cov[:ndim, :ndim] = Hi * s2
                if ndim == 2:
                    cov[2, 2] = 1e-6
            except np.linalg.LinAlgError:
                cov = None
        return MatchResult(t, delta, residual, ratio, n, it, converged, cov)

    # ------------------------------------------------------------------ 粗搜尋
    def coarse_search(self, base_xy: np.ndarray, centers: np.ndarray,
                      deltas: np.ndarray, sigma: float = 0.25,
                      max_points: int = 400, chunk: int = 4096):
        """在 (centers x deltas) 這組候選上暴力打分, 回傳分數最高的 (t, delta)。

        分數用 likelihood field 的最近格查表 (不內插), 因為粗搜尋只需要挑出
        「大概對的地方」, 之後交給 refine 去磨到公分級。
        """
        base = np.asarray(base_xy, dtype=np.float64).reshape(-1, 2)
        if base.shape[0] > max_points:
            sel = np.linspace(0, base.shape[0] - 1, max_points).astype(int)
            base = base[sel]
        centers = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
        deltas = np.asarray(deltas, dtype=np.float64).reshape(-1)

        lik = np.exp(-0.5 * (self.map.dist / sigma) ** 2).astype(np.float32)
        h, w = lik.shape
        res = self.map.resolution
        ox, oy = self.map.origin

        best = (-1.0, centers[0], float(deltas[0]))
        for delta in deltas:
            rot = base @ rot2(float(delta)).T
            for s in range(0, centers.shape[0], chunk):
                c = centers[s:s + chunk]                     # (M,2)
                px = c[:, 0, None] + rot[None, :, 0]
                py = c[:, 1, None] + rot[None, :, 1]
                col = ((px - ox) / res).astype(np.int32)
                row = ((py - oy) / res).astype(np.int32)
                ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
                np.clip(col, 0, w - 1, out=col)
                np.clip(row, 0, h - 1, out=row)
                sc = np.where(ok, lik[row, col], 0.0).sum(axis=1)
                i = int(np.argmax(sc))
                if sc[i] > best[0]:
                    best = (float(sc[i]), c[i].copy(), float(delta))
        return best[1], best[2], best[0] / base.shape[0]

    def global_localize(self, base_xy: np.ndarray, step: float = 0.20,
                        clearance: float = 0.25, yaw_search: bool = False,
                        yaw_bins: int = 72, lock_yaw: bool = True):
        """整張地圖找一次車子在哪 —— 不需要使用者填初始位姿。

        yaw 已知 (IMU) 時只搜平移, 這個房間只有 ~1000 個候選點, 瞬間就跑完。
        """
        free = self.map.free_mask(clearance)
        row, col = np.nonzero(free)
        xs = self.map.origin[0] + (col + 0.5) * self.map.resolution
        ys = self.map.origin[1] + (row + 0.5) * self.map.resolution
        keep = np.ones(xs.shape, dtype=bool)
        if step > self.map.resolution:
            q = np.round(np.stack([xs, ys], axis=1) / step).astype(np.int64)
            _, idx = np.unique(q, axis=0, return_index=True)
            keep = np.zeros(xs.shape, dtype=bool)
            keep[idx] = True
        centers = np.stack([xs[keep], ys[keep]], axis=1)
        deltas = (np.linspace(0, 2 * np.pi, yaw_bins, endpoint=False)
                  if yaw_search else np.array([0.0]))

        t0, d0, _ = self.coarse_search(base_xy, centers, deltas)
        # 粗搜尋只保證找到對的房間角落, 再用一次細搜尋把角度磨掉格點誤差
        if yaw_search:
            fine = d0 + np.linspace(-np.pi / yaw_bins, np.pi / yaw_bins, 9)
            local = t0 + np.stack(np.meshgrid(
                np.linspace(-step, step, 5), np.linspace(-step, step, 5),
                indexing='ij'), axis=-1).reshape(-1, 2)
            t0, d0, _ = self.coarse_search(base_xy, local, fine)
        return self.refine(base_xy, t0, d0, lock_yaw=lock_yaw)
