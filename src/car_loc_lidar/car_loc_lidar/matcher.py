#!/usr/bin/env python3
"""掃描對地圖 (scan-to-map) 的 3 自由度配準。

要解的是

    world_xy_i = t + Rz(theta) @ sensor_xy_i

其中 (t, theta) 就是車子在地圖裡的位姿。

**這裡三個自由度都要解。** 跟「有 IMU」的做法差別就在這裡: IMU 給得出絕對
yaw 的時候可以把 theta 鎖死, 問題退化成超定到不能再超定的兩自由度最小平方,
公分級很輕鬆; 只有雷射的時候 yaw 必須由幾何自己撐出來, 而 yaw 誤差會透過
「距離 x 角度」放大成位置誤差 —— 10 m 外的牆, 0.5 度就是 8.7 cm。
這是方法二相對於融合做法真正的代價, 不是參數沒調好。

代價函數是「每個點到最近障礙的距離」的 Huber 加權平方和, 距離直接從地圖的
EDT 雙線性內插, 所以不需要每次迭代重找對應點 (ICP 最慢也最容易錯的一步)。
"""
from __future__ import annotations

import math

import numpy as np


def rot2(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class MatchResult:
    __slots__ = ('t', 'theta', 'residual', 'inlier_ratio', 'n_points',
                 'iterations', 'converged', 'cov', 'n_occluded', 'see_through')

    def __init__(self, t, theta, residual, inlier_ratio, n_points,
                 iterations, converged, cov, n_occluded=0):
        self.t = np.asarray(t, dtype=np.float64)
        self.theta = float(theta)
        self.residual = float(residual)        # inlier 的平均距離 (m)
        self.inlier_ratio = float(inlier_ratio)
        self.n_points = int(n_points)
        self.iterations = int(iterations)
        self.converged = bool(converged)
        self.cov = cov                         # 3x3 (x, y, yaw), 可能是 None
        self.n_occluded = int(n_occluded)      # 當成地圖上沒有的東西拿掉的點 (不算在 n_points 裡)
        self.see_through = float('nan')        # 穿牆射線的比例 (ScanMatcher.see_through_ratio), 有算才有

    def __repr__(self):
        return (f'MatchResult(t=[{self.t[0]:+.4f},{self.t[1]:+.4f}], '
                f'theta={math.degrees(self.theta):+.2f}°, '
                f'residual={self.residual * 100:.2f} cm, '
                f'inlier={self.inlier_ratio:.2f}, n={self.n_points}'
                + (f', occluded={self.n_occluded}' if self.n_occluded else '') + ')')


class ScanMatcher:

    def __init__(self, gmap, huber: float = 0.10, max_iter: int = 40,
                 tol: float = 1e-5, inlier_dist: float = 0.30,
                 d_far: float = 3.0, lm_lambda: float = 1e-6,
                 occluder_dist: float = 0.0, occluder_max_ratio: float = 0.5):
        self.map = gmap
        # 地圖上沒有的東西 (箱子、人): 第一次配準後, 落在房間內部 (interior_mask) 而且離牆
        # > occluder_dist 的點拿掉, 再配準一次, 品質指標 (殘差 / inlier / cov) 只用剩下的點。
        # 不拿掉的話每個點都有 Huber 的常數拉力, 一個佔掃描 15% 的箱子會把位姿拉偏幾公分,
        # 殘差變大 -> 融合的 sigma 閘門擋掉、鎖死偵測誤判 (2026-11-18 Isaac, 見 README)。
        # 牆外 / 柱子裡的點不拿 —— 位姿錯了的時候點就是落在那裡, 那是要留著的證據。
        # 拿掉的比例 > occluder_max_ratio 就不拿 (位姿大概錯了, 照舊報爛的指標)。0 = 關。
        self.occluder_dist = float(occluder_dist)
        self.occluder_max_ratio = float(occluder_max_ratio)
        self.interior = gmap.interior_mask() if self.occluder_dist > 0 else None
        self.huber = float(huber)        # 超過這個距離的點改用線性代價 (擋離群點)
        self.max_iter = int(max_iter)
        self.tol = float(tol)
        self.inlier_dist = float(inlier_dist)
        self.d_far = float(d_far)
        self.lm_lambda = float(lm_lambda)

    # ------------------------------------------------------------------ 精配準
    def _cost(self, base, t, theta):
        """回傳 (cost, r, J, valid_count)。"""
        rot = base @ rot2(theta).T
        d, gx, gy, valid = self.map.sample(rot + t, d_far=self.d_far)
        nv = int(valid.sum())
        if nv < 10:
            return float('inf'), None, None, nv
        r = d[valid]
        pr = rot[valid]
        J = np.empty((r.size, 3), dtype=np.float64)
        J[:, 0] = gx[valid]
        J[:, 1] = gy[valid]
        # d/d(theta) 造成的位移是 Rz'(theta) @ base, 也就是 rot 轉 90 度
        J[:, 2] = gx[valid] * (-pr[:, 1]) + gy[valid] * pr[:, 0]
        a = np.abs(r)
        cost = float(np.sum(np.where(a <= self.huber, 0.5 * r ** 2,
                                     self.huber * (a - 0.5 * self.huber))))
        return cost, r, J, nv

    def _occluders(self, base, t, theta, dist):
        """在位姿 (t, theta) 下, 落在房間內部而且離牆 > dist 的點 (bool mask)。"""
        pts = base @ rot2(theta).T + np.asarray(t, dtype=np.float64).reshape(2)
        d, _, _, valid = self.map.sample(pts, d_far=self.d_far)
        return valid & (d > dist) & self.map.in_mask(self.interior, pts)

    def refine(self, sensor_xy: np.ndarray, t0, theta0: float = 0.0,
               occluders: bool = True) -> MatchResult:
        """配準。occluder_dist > 0 而且 occluders=True 時, 地圖上沒有的東西分兩步拿掉:

        1. 在**起點** (追蹤時 = 等速預測) 先拿掉離牆 > max(2 x occluder_dist, 0.3) m 的點 ——
           門檻放寬是因為預測本身有誤差, 不能把牆的點當成箱子。一定要在第一次配準**之前**拿:
           2026-11-18 Isaac 四個箱子佔掃描 21% (Isaac 的 RTX 雷射一半射線是無效的), 從真值起配
           都會被箱子的 Huber 拉力拖走 26 cm, 之後才分類就來不及了。
        2. 配完再用 occluder_dist 在新位姿重新分類一次 (全部的點重新判, 先前拿錯的會回來), 再配一次。
        任何一步拿掉的比例 > occluder_max_ratio 就那一步不拿 (位姿大概錯了)。
        結果 inlier < 50% (起點本身就錯很多, 1 拿掉的其實是牆) 時再用舊的方法 (全部的點) 配一次,
        取 inlier 高的 —— 不然起點偏 0.5 m / 20° 時就救不回來 (test_occluders E)。
        """
        base = np.asarray(sensor_xy, dtype=np.float64).reshape(-1, 2)
        n = len(base)
        if self.interior is None or not occluders or n < 10:
            return self._refine(base, t0, theta0)
        r = self._refine_occluded(base, t0, theta0)
        if r.inlier_ratio < 0.5:
            alt = self._refine(base, t0, theta0)
            if alt.inlier_ratio > r.inlier_ratio:
                return alt
        return r

    def _refine_occluded(self, base, t0, theta0):
        n = len(base)
        cap = self.occluder_max_ratio * n
        pre = self._occluders(base, t0, theta0, max(2.0 * self.occluder_dist, 0.3))
        if 0 < pre.sum() <= cap and n - pre.sum() >= 10:
            r1 = self._refine(base[~pre], t0, theta0)
        else:
            r1 = self._refine(base, t0, theta0)
        if not np.isfinite(r1.residual):
            return r1
        occ = self._occluders(base, r1.t, r1.theta, self.occluder_dist)
        k = int(occ.sum())
        if k > cap or n - k < 10:
            return self._refine(base, r1.t, r1.theta)       # 報全部點的 (爛) 指標
        if k == 0:
            return self._refine(base, r1.t, r1.theta) if pre.any() else r1
        r2 = self._refine(base[~occ], r1.t, r1.theta)
        r2.n_occluded = k
        return r2

    def _refine(self, sensor_xy: np.ndarray, t0, theta0: float = 0.0) -> MatchResult:
        base = np.asarray(sensor_xy, dtype=np.float64).reshape(-1, 2)
        t = np.asarray(t0, dtype=np.float64).reshape(2).copy()
        theta = float(theta0)
        n = base.shape[0]
        if n < 10:
            return MatchResult(t, theta, float('inf'), 0.0, n, 0, False, None)

        lam = self.lm_lambda
        cost, r, J, _ = self._cost(base, t, theta)
        if not np.isfinite(cost):
            return MatchResult(t, theta, float('inf'), 0.0, n, 0, False, None)

        H = None
        it = 0
        converged = False
        sn = 0.0
        step = np.zeros(3)
        for it in range(1, self.max_iter + 1):
            # Huber 權重: 近的點用平方 (拉得準), 遠的離群點只給常數拉力,
            # 不會被少數打到動態障礙 (人、椅子) 的點綁架。
            a = np.abs(r)
            w = np.where(a <= self.huber, 1.0, self.huber / np.maximum(a, 1e-9))
            Jw = J * w[:, None]
            H = J.T @ Jw
            g = Jw.T @ r
            scale = max(np.trace(H) / 3.0, 1e-9)

            accepted = False
            for _ in range(8):    # Levenberg-Marquardt: 試到這一步真的變好為止
                try:
                    step = -np.linalg.solve(H + lam * scale * np.eye(3), g)
                except np.linalg.LinAlgError:
                    break
                # 距離場只在障礙附近才有意義, 一次跳太遠會跳到別的牆上
                sn = float(np.linalg.norm(step[:2]))
                if sn > 0.5:
                    step = step * (0.5 / sn)
                    sn = 0.5
                nt = t + step[:2]
                nth = theta + float(np.clip(step[2], -0.2, 0.2))
                nc, nr, nJ, _ = self._cost(base, nt, nth)
                if nc < cost:
                    t, theta, cost, r, J = nt, nth, nc, nr, nJ
                    lam = max(lam * 0.3, 1e-9)
                    accepted = True
                    break
                lam = min(lam * 10.0, 1e4)
            if not accepted:
                converged = True                 # 已經沒有更好的方向了
                break
            if sn < self.tol and abs(step[2]) < self.tol:
                converged = True
                break

        pts = base @ rot2(theta).T + t
        d, _, _, valid = self.map.sample(pts, d_far=self.d_far)
        inl = valid & (d < self.inlier_dist)
        residual = float(d[inl].mean()) if inl.any() else float('inf')
        ratio = float(inl.sum()) / max(n, 1)

        cov = None
        if H is not None and inl.sum() > 20:
            # sigma^2 * H^-1 —— 當作「這次配準有多可信」的粗略指標, 不是嚴謹後驗。
            # 幾何退化 (只看得到一面牆) 時 H 會接近奇異, 沿牆方向的變異數會爆掉,
            # 那正是我們想看到的訊號。
            try:
                s2 = max(residual, 0.005) ** 2
                cov = np.linalg.inv(H + 1e-9 * np.eye(3)) * s2
            except np.linalg.LinAlgError:
                cov = None
        return MatchResult(t, theta, residual, ratio, n, it, converged, cov)

    # ------------------------------------------------------------------ 穿牆
    def see_through_ratio(self, sensor_xy, t, theta, margin: float = 0.3,
                          r_max: float = 8.0) -> float:
        """從車子中心射到每個點的射線, 在離終點 margin 以前就穿過地圖佔據格的比例。

        地圖上沒有的東西 (箱子、人) 只會讓量到的距離比地圖預期的**短**, 不可能長;
        量到的比預期長 = 雷射「穿過」地圖上的牆 / 柱子 = 位姿錯了 (例如鎖到 180 度對稱解,
        柱子的位置對不上)。所以這個指標不會被遮擋騙, 拿來補 occluder 拿掉點之後變弱的鎖死偵測。
        margin 擋掉斜斜掃過牆面的射線 (貼著牆走一段才打到)。只看 r_max 以內的點。"""
        base = np.asarray(sensor_xy, dtype=np.float64).reshape(-1, 2)
        rng = np.linalg.norm(base, axis=1)
        keep = (rng > margin + 0.05) & (rng < r_max)
        if keep.sum() < 10:
            return float('nan')
        base, rng = base[keep], rng[keep]
        pts = base @ rot2(theta).T + np.asarray(t, dtype=np.float64).reshape(2)
        o = np.asarray(t, dtype=np.float64).reshape(2)
        step = self.map.resolution
        fr = np.arange(step, r_max, step)                     # 沿射線的距離
        u = (pts - o) / rng[:, None]                           # 單位方向
        L = rng - margin                                       # 每條射線檢查到哪
        hit = np.zeros(len(pts), dtype=bool)
        for k in range(0, len(fr), 64):
            f = fr[k:k + 64]
            q = o[None, None, :] + u[:, None, :] * f[None, :, None]       # (M, F, 2)
            col, row, ok = self.map._to_cell(q.reshape(-1, 2))
            occ = np.zeros(len(col), dtype=bool)
            occ[ok] = self.map.occ[row[ok], col[ok]]
            occ = occ.reshape(len(pts), len(f)) & (f[None, :] < L[:, None])
            hit |= occ.any(axis=1)
            if f[0] > L.max():
                break
        return float(hit.mean())

    # ------------------------------------------------------------------ 掃角度重試
    def refine_sweep(self, sensor_xy: np.ndarray, t0, theta0: float,
                     yaw_span: float = 0.7, yaw_bins: int = 15,
                     xy_span: float = 0.3, xy_bins: int = 5) -> MatchResult:
        """在預測位姿附近掃一圈角度再 refine —— 等速預測跟不上時的救援。

        為什麼需要它: 沒有 IMU 的時候, 「下一幀朝哪」是用上一段的角速度外推的。
        車子原地自旋到 20 rad/s 時一幀 (50 ms) 就轉過 57 度, 角加速度只要一變,
        預測的 yaw 就差好幾十度 —— 那已經超出 refine 的收斂半徑, 它會掉進隔壁
        的局部極小值, 然後整條軌跡就毀了。

        先在 (位置 x 角度) 的小網格上打分挑起點, 再交給 refine。範圍是「預測
        可能錯多少」而不是「車可能在哪」, 所以很小, 成本遠低於全域定位。

        **結果超出搜尋範圍就整個丟掉。** coarse_search 的候選被限制在 span 之內,
        但它之後的 refine 是沒有限制的, 可以一路走到別的地方去。長方形的房間對
        180 度旋轉幾乎是對稱的 (柱子那種小特徵撐不住), 所以「連續兩幀各被拉走
        90 度」是真的會發生的事 —— 離線實測遇過一次, 鎖在 180 度之後殘差看起來
        還很漂亮, 就再也回不來了。
        寧可回報失敗讓呼叫端去做全域定位, 也不要接受一個物理上不可能的修正量。
        """
        t0 = np.asarray(t0, dtype=np.float64).reshape(2)
        thetas = theta0 + np.linspace(-yaw_span, yaw_span, yaw_bins)
        d = np.linspace(-xy_span, xy_span, xy_bins)
        centers = t0 + np.stack(np.meshgrid(d, d, indexing='ij'),
                                axis=-1).reshape(-1, 2)
        ts, th, _ = self.coarse_search(sensor_xy, centers, thetas)
        r = self.refine(sensor_xy, ts, th)
        if (abs(wrap_pi(r.theta - theta0)) > 1.5 * yaw_span
                or float(np.linalg.norm(r.t - t0)) > 2.0 * xy_span):
            return MatchResult(t0, theta0, float('inf'), 0.0,
                               len(sensor_xy), r.iterations, False, None)
        return r

    # ------------------------------------------------------------------ 粗搜尋
    def coarse_search(self, sensor_xy: np.ndarray, centers: np.ndarray,
                      thetas: np.ndarray, sigma: float = 0.25,
                      max_points: int = 300, chunk: int = 2048):
        """在 (centers x thetas) 這組候選上暴力打分, 回傳分數最高的 (t, theta)。

        分數用 likelihood field 的最近格查表 (不內插) —— 粗搜尋只需要挑出
        「大概對的地方」, 之後交給 refine 去磨到公分級。
        """
        base = np.asarray(sensor_xy, dtype=np.float64).reshape(-1, 2)
        if base.shape[0] > max_points:
            base = base[np.linspace(0, base.shape[0] - 1, max_points).astype(int)]
        centers = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
        thetas = np.asarray(thetas, dtype=np.float64).reshape(-1)

        lik = np.exp(-0.5 * (self.map.dist / sigma) ** 2).astype(np.float32)
        h, w = lik.shape
        res = self.map.resolution
        ox, oy = self.map.origin

        best = (-1.0, centers[0], float(thetas[0]))
        for th in thetas:
            rot = base @ rot2(float(th)).T
            for s in range(0, centers.shape[0], chunk):
                c = centers[s:s + chunk]                       # (M, 2)
                col = ((c[:, 0, None] + rot[None, :, 0] - ox) / res).astype(np.int32)
                row = ((c[:, 1, None] + rot[None, :, 1] - oy) / res).astype(np.int32)
                ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
                np.clip(col, 0, w - 1, out=col)
                np.clip(row, 0, h - 1, out=row)
                sc = np.where(ok, lik[row, col], 0.0).sum(axis=1)
                i = int(np.argmax(sc))
                if sc[i] > best[0]:
                    best = (float(sc[i]), c[i].copy(), float(th))
        return best[1], best[2], best[0] / base.shape[0]

    def global_localize(self, sensor_xy: np.ndarray, step: float = 0.30,
                        clearance: float = 0.25, yaw_bins: int = 72,
                        center=None, radius: float = 0.0) -> MatchResult:
        """整張地圖找一次車子在哪 —— 不需要使用者填初始位姿。

        只有雷射的時候 yaw 也是未知的, 所以候選是 (位置 x 角度) 的乘積, 比
        「IMU 給 yaw」的版本貴 yaw_bins 倍。實務上這個房間 step=0.30 大約
        1000 個位置 x 72 個角度, 幾秒內跑得完, 而且只在冷開機與追丟時才做。

        center / radius
        ---------------
        給了就只在 center 周圍 radius 公尺內找。**冷開機不要給** (那時候本來就
        不知道車在哪), 但「追丟了要救回來」的情況一定要給: 對稱的走廊或格局
        重複的房間, 不設限的全域搜尋很容易在別的地方找到一個殘差同樣漂亮的解,
        於是估計瞬間跳到幾公尺外。

        半徑該給多少有物理上限: 車子最快 v_max, 追丟了 dt 秒, 就不可能離開上一個
        可信位置 v_max*dt 以外。呼叫端照這個算。
        """
        free = self.map.free_mask(clearance)
        row, col = np.nonzero(free)
        if row.size == 0:
            return MatchResult([0.0, 0.0], 0.0, float('inf'), 0.0,
                               len(sensor_xy), 0, False, None)
        xs = self.map.origin[0] + (col + 0.5) * self.map.resolution
        ys = self.map.origin[1] + (row + 0.5) * self.map.resolution
        if center is not None and radius > 0.0:
            c = np.asarray(center, dtype=np.float64).reshape(2)
            near = (xs - c[0]) ** 2 + (ys - c[1]) ** 2 <= radius * radius
            if near.sum() >= 4:            # 太少就不設限, 免得完全沒有候選點
                xs, ys = xs[near], ys[near]

        keep = np.ones(xs.shape, dtype=bool)
        if step > self.map.resolution:
            q = np.round(np.stack([xs, ys], axis=1) / step).astype(np.int64)
            _, idx = np.unique(q, axis=0, return_index=True)
            keep = np.zeros(xs.shape, dtype=bool)
            keep[idx] = True
        centers = np.stack([xs[keep], ys[keep]], axis=1)
        thetas = np.linspace(0, 2 * np.pi, yaw_bins, endpoint=False)

        t0, th0, _ = self.coarse_search(sensor_xy, centers, thetas)
        # 粗搜尋只保證找到「對的角落」, 再用一次細搜尋把角度磨掉格點誤差,
        # 不然 refine 的起點可能差 2.5 度, 那已經足夠讓它收斂到隔壁的牆。
        fine = th0 + np.linspace(-np.pi / yaw_bins, np.pi / yaw_bins, 9)
        local = t0 + np.stack(np.meshgrid(
            np.linspace(-step, step, 5), np.linspace(-step, step, 5),
            indexing='ij'), axis=-1).reshape(-1, 2)
        t0, th0, _ = self.coarse_search(sensor_xy, local, fine)
        # 全域定位**不拿**地圖上沒有的點: 拿掉 30% 的點之後, 錯的候選也會看起來很乾淨
        # (2026-11-18 Isaac: 四個箱子時接受了一個錯 2.4 m 的解, inlier 100% / 拿掉 59 點)。
        # 呼叫端另外用 see_through_ratio 擋鎖到對稱解的候選。
        return self.refine(sensor_xy, t0, th0, occluders=False)
