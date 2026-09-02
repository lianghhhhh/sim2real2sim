#!/usr/bin/env python3
"""直接從 car.usd 的幾何算出 2D 佔據地圖 —— 不用開 Isaac, 也不用邊開車邊建圖。

為什麼要這樣做:
    「建圖」跟「定位」的誤差長得一模一樣。用 SLAM 邊開邊建出來的地圖, 牆的位置
    本身就帶著建圖當下的位姿誤差 (實測會出現 10~30 cm 的鬼牆), 之後拿它來定位,
    你永遠分不清是地圖歪了還是定位歪了。
    但這是模擬環境 —— 牆在哪裡是 USD 檔裡寫死的已知事實。直接把那些幾何切一刀
    投影成 2D, 得到的地圖誤差只剩格點解析度, 定位精度的上限就只剩雷射本身的
    量測雜訊 (SICK multiScan136 的 rangeAccuracy 是 0.02 m)。

做法:
    把場景裡所有幾何三角化 -> 取世界座標 z 落在 [z_min, z_max] 的部分 -> 投影到
    xy 平面 -> 打進格點。牆的上下底面 (水平面) 不在這個高度帶裡, 所以切出來的
    自然只有「牆面的輪廓線」, 不是實心的牆。

用法 (在 host 上跑, 不是在 docker 裡):
    ./scripts/make_map_from_usd.py
    ./scripts/make_map_from_usd.py --resolution 0.02 --out src/car_localization/maps/room.npz
    ./scripts/make_map_from_usd.py car_sim.usd --out src/car_localization/maps/room_sim.npz

輸出:
    <out>.npz        car_localization 定位用的地圖
    <out>.pgm/.yaml  nav2 / rviz 看得懂的同一張圖 (只是為了肉眼檢查)
"""
import argparse
import glob
import os
import sys

DEFAULT_ISAAC = os.environ.get('ISAAC_SIM_PATH', os.path.expanduser('~/isaac-sim'))
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ─────────────────────────────────────────────────────────────────────────────
# 以下這段只有在 Isaac 的 python 底下才跑得動 (需要 pxr)。
# 這個檔案會自己用正確的環境重新執行自己, 所以使用者不用管 PYTHONPATH。
# ─────────────────────────────────────────────────────────────────────────────
def run_worker(args):
    import numpy as np
    from pxr import Usd, UsdGeom

    sys.path.insert(0, os.path.join(REPO, 'src', 'car_localization'))
    from car_localization.gridmap import GridMap

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        sys.exit(f'開不起來: {args.usd}')
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    if abs(mpu - 1.0) > 1e-9:
        print(f'注意: stage 的 metersPerUnit = {mpu}, 會把座標乘上這個值換成公尺')

    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'], useExtentsHint=False)
    excludes = tuple(args.exclude)

    def world_triangles(prim):
        """回傳這個 prim 在世界座標下的三角形 (T, 3, 3); 不認得的型別回 None。"""
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        M = np.array([[m[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)

        def to_world(v):
            v = np.asarray(v, dtype=np.float64).reshape(-1, 3)
            h = np.concatenate([v, np.ones((v.shape[0], 1))], axis=1)
            return (h @ M)[:, :3] * mpu           # USD 是 row-vector 慣例

        t = prim.GetTypeName()
        if t == 'Mesh':
            g = UsdGeom.Mesh(prim)
            pts = g.GetPointsAttr().Get()
            counts = g.GetFaceVertexCountsAttr().Get()
            idx = g.GetFaceVertexIndicesAttr().Get()
            if not pts or not counts or not idx:
                return None
            pts = to_world(np.array(pts, dtype=np.float64))
            idx = np.asarray(idx, dtype=np.int64)
            tris, k = [], 0
            for c in counts:
                for i in range(1, c - 1):       # 多邊形用扇形三角化
                    tris.append((idx[k], idx[k + i], idx[k + i + 1]))
                k += c
            if not tris:
                return None
            return pts[np.asarray(tris, dtype=np.int64)]

        if t == 'Cube':
            s = UsdGeom.Cube(prim).GetSizeAttr().Get()
            s = 1.0 if s is None else float(s)
            h = s * 0.5
            v = np.array([[x, y, z] for x in (-h, h) for y in (-h, h) for z in (-h, h)])
            # 8 個角: index = 4*ix + 2*iy + iz
            faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
                     (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
            tris = []
            for a, b, c, d in faces:
                tris += [(a, b, c), (a, c, d)]
            return to_world(v)[np.asarray(tris)]

        if t in ('Cylinder', 'Cone', 'Capsule'):
            g = getattr(UsdGeom, t)(prim)
            r = float(g.GetRadiusAttr().Get() or 1.0)
            hgt = float(g.GetHeightAttr().Get() or 2.0)
            axis = str(g.GetAxisAttr().Get() or 'Z')
            n = args.circle_segments
            th = np.linspace(0, 2 * np.pi, n, endpoint=False)
            # 先在 Z 軸版本上建, 最後再轉到指定的軸
            r_top = 0.0 if t == 'Cone' else r
            lo, hi = -hgt / 2.0, hgt / 2.0
            ring_b = np.stack([r * np.cos(th), r * np.sin(th), np.full(n, lo)], axis=1)
            ring_t = np.stack([r_top * np.cos(th), r_top * np.sin(th), np.full(n, hi)], axis=1)
            v = np.concatenate([ring_b, ring_t], axis=0)
            tris = []
            for i in range(n):
                j = (i + 1) % n
                tris += [(i, j, n + i), (j, n + j, n + i)]
            v = v[:, {'X': [2, 0, 1], 'Y': [1, 2, 0], 'Z': [0, 1, 2]}[axis]]
            return to_world(v)[np.asarray(tris)]

        if t == 'Sphere':
            r = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get() or 1.0)
            n, k = args.circle_segments, args.circle_segments // 2
            th = np.linspace(0, 2 * np.pi, n, endpoint=False)
            ph = np.linspace(-np.pi / 2, np.pi / 2, k)
            v = np.stack([(r * np.cos(p) * np.cos(th), r * np.cos(p) * np.sin(th),
                           np.full(n, r * np.sin(p))) for p in ph], axis=0)
            v = np.transpose(v, (0, 2, 1)).reshape(-1, 3)
            tris = []
            for a in range(k - 1):
                for i in range(n):
                    j = (i + 1) % n
                    p0, p1 = a * n + i, a * n + j
                    q0, q1 = (a + 1) * n + i, (a + 1) * n + j
                    tris += [(p0, p1, q0), (p1, q1, q0)]
            return to_world(v)[np.asarray(tris)]

        return None

    # 法線離鉛直方向多近就算「水平面」。水平面 (地板、牆的頂面/底面) 一定要丟掉:
    # 雷射打在水平面上得到的回波, 投影到 2D 之後會把整個面的footprint 填成實心,
    # 牆就從「兩條 1 m 相距的細線」變成一塊 1 m 厚的實心磚。那樣的地圖在牆內部
    # 距離場全是 0, 位姿往牆裡陷 30 cm 也不會被罰到, 定位精度直接毀掉。
    cos_flat = np.cos(np.deg2rad(90.0 - args.min_surface_tilt))

    def sample_triangles(tris, step, z_lo, z_hi):
        """在三角形上取樣, 只留 z 在高度帶裡的點, 回傳 (N, 2) 的 xy。"""
        out = []
        for tri in tris:
            zmin, zmax = tri[:, 2].min(), tri[:, 2].max()
            if zmax < z_lo or zmin > z_hi:
                continue
            a, b, c = tri
            nrm = np.cross(b - a, c - a)
            ln = np.linalg.norm(nrm)
            if ln < 1e-12 or abs(nrm[2]) / ln > cos_flat:
                continue                        # 水平面 -> 不是雷射看得到的「牆」
            e = max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b))
            m = int(min(max(1, np.ceil(e / step)), 4000))
            i, j = np.meshgrid(np.arange(m + 1), np.arange(m + 1), indexing='ij')
            keep = (i + j) <= m
            u = (i[keep] / m)[:, None]
            v = (j[keep] / m)[:, None]
            p = a[None, :] + u * (b - a)[None, :] + v * (c - a)[None, :]
            p = p[(p[:, 2] >= z_lo) & (p[:, 2] <= z_hi)]
            if p.size:
                out.append(p[:, :2])
        return np.concatenate(out, axis=0) if out else np.empty((0, 2))

    print(f'讀取 {args.usd}')
    print(f'高度帶 (世界座標 z): [{args.z_min:.2f}, {args.z_max:.2f}] m')
    print(f'排除: {", ".join(excludes)}\n')

    all_xy, rows, unhandled = [], [], []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if path.startswith(excludes):
            continue
        if not prim.IsA(UsdGeom.Gprim):
            continue
        img = UsdGeom.Imageable(prim)
        if img and img.ComputeVisibility(Usd.TimeCode.Default()) == UsdGeom.Tokens.invisible:
            continue
        b = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
        if b.IsEmpty():
            continue
        if b.GetMax()[2] * mpu < args.z_min or b.GetMin()[2] * mpu > args.z_max:
            rows.append((path, prim.GetTypeName(), 0, '高度帶之外'))
            continue

        tris = world_triangles(prim)
        if tris is None:
            unhandled.append((path, str(prim.GetTypeName())))
            continue
        xy = sample_triangles(tris, args.sample_step, args.z_min, args.z_max)
        rows.append((path, prim.GetTypeName(), len(xy), ''))
        if len(xy):
            all_xy.append(xy)

    for path, tp, n, note in rows:
        print(f'  {path:<46} {str(tp):<10} {n:>8} 點  {note}')
    if unhandled:
        print('\n以下幾何型別沒有處理 (它們不會出現在地圖上):')
        for path, tp in unhandled:
            print(f'  {path}  ({tp})')
        print('  -> 如果它們是雷射掃得到的障礙物, 這張地圖就是錯的, 不要用。')

    if not all_xy:
        sys.exit('\n高度帶裡沒有任何幾何, 地圖是空的。檢查 --z-min / --z-max。')

    pts = np.concatenate(all_xy, axis=0)
    grid = GridMap.from_points(
        pts, resolution=args.resolution, margin=args.margin,
        meta={'source': os.path.basename(args.usd), 'z_band': [args.z_min, args.z_max],
              'resolution': args.resolution, 'generator': 'make_map_from_usd.py'})

    kept_pts = pts
    if not args.keep_hidden:
        removed = _keep_visible(grid, args.seed)
        print(f'\n只保留車子看得到的表面: 移除 {removed} 格 '
              f'(牆的外側面、房間外面的東西)。')
        print('  為什麼一定要做這件事: 牆是有厚度的實體, 內外兩面在地圖上會變成'
              '\n  兩條相距 1 m 的平行線。掃描比對分不出自己對上的是哪一條, 位姿'
              '\n  就會整個滑到外牆上, 而且殘差看起來還很漂亮 —— 這種錯最難查。')
        col, row, ok = grid._to_cell(pts)
        keep = np.zeros(len(pts), dtype=bool)
        keep[ok] = grid.occ[row[ok], col[ok]]
        kept_pts = pts[keep]

    # 距離場直接用表面取樣點算, 不經過格點 -> 地圖幾何誤差 ~取樣間距, 不是半格
    grid.set_exact_distance(kept_pts)

    out = os.path.abspath(os.path.expanduser(args.out))
    if not out.endswith('.npz'):
        out += '.npz'
    grid.save(out)
    stem = out[:-4]
    grid.save_nav2(stem)

    x0, y0, x1, y1 = grid.bounds
    print(f'\n{grid}')
    print(f'  取樣點 {len(pts)} -> 佔據格 {grid.n_occupied}')
    print(f'  地圖範圍 x[{x0:+.2f}, {x1:+.2f}]  y[{y0:+.2f}, {y1:+.2f}] m')
    print(f'\n已寫出:\n  {out}\n  {stem}.pgm + {stem}.yaml')
    print('\n俯視圖 (# = 障礙):')
    print(grid.ascii_view(cols=110))
    print('\n請肉眼確認這張圖就是你在 Isaac 裡看到的房間。牆應該是「一格寬的細線」,'
          '\n如果出現兩條平行線或形狀不對, 就是幾何抓錯了, 不要拿去定位。')


def _keep_visible(grid, seed):
    """從房間裡面 flood fill, 只留下「跟可走空間相鄰」的障礙格。

    這等於一次視線檢查: 車子只能在房間內部移動, 雷射也只能看到從房間內部
    碰得到的表面。牆的外側面、房間外的地板, 雷射永遠看不到, 留在地圖上只會
    製造假的配準解。
    """
    import numpy as np
    from scipy import ndimage

    free = ~grid.occ
    lab, _ = ndimage.label(free)
    col = int((seed[0] - grid.origin[0]) / grid.resolution)
    row = int((seed[1] - grid.origin[1]) / grid.resolution)
    h, w = grid.occ.shape
    if not (0 <= col < w and 0 <= row < h) or lab[row, col] == 0:
        print(f'  警告: 種子點 {seed} 不在可走空間裡, 略過視線過濾。')
        return 0
    inside = lab == lab[row, col]
    reachable = ndimage.binary_dilation(inside, structure=np.ones((3, 3), bool))
    visible = grid.occ & reachable
    removed = int(grid.occ.sum() - visible.sum())
    grid.occ = visible
    grid._dist = None
    grid._grad = None
    return removed


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('usd', nargs='?', default=os.path.join(REPO, 'car.usd'))
    ap.add_argument('--out', default=os.path.join(REPO, 'src', 'car_localization',
                                                  'maps', 'car_usd.npz'),
                    help='輸出的 .npz (同名的 .pgm/.yaml 也會一起產生)')
    ap.add_argument('--resolution', type=float, default=0.05, help='格點大小 (m)')
    ap.add_argument('--z-min', type=float, default=0.05,
                    help='高度帶下界 (m, 地面在 0)。要高過地面。')
    ap.add_argument('--z-max', type=float, default=0.95,
                    help='高度帶上界 (m)。要低於牆的頂端 (car.usd 的牆頂在 1.00)。')
    ap.add_argument('--min-surface-tilt', type=float, default=15.0,
                    help='表面要比水平面陡多少度才算牆 (度)。低於這個角度的面視為'
                         '地板/頂面, 不進地圖。')
    ap.add_argument('--margin', type=float, default=0.5, help='地圖四周多留的空白 (m)')
    ap.add_argument('--sample-step', type=float, default=0.02, help='三角形上的取樣間距 (m)')
    ap.add_argument('--circle-segments', type=int, default=256,
                    help='圓柱/球體切成幾段 (影響柱子的圓有多圓)')
    ap.add_argument('--exclude', nargs='*',
                    default=['/World/small_car', '/Environment', '/Render', '/OmniverseKit'],
                    help='要排除的 prim 路徑前綴 (車子本身一定要排除)')
    ap.add_argument('--seed', type=float, nargs=2, default=[0.0, 0.0],
                    metavar=('X', 'Y'),
                    help='房間內部的一個點 (m), 用來 flood fill 找出可走空間。'
                         '車子的起始位置就可以。')
    ap.add_argument('--keep-hidden', action='store_true',
                    help='保留雷射看不到的表面 (牆的外側面)。除錯用, 平常不要開。')
    ap.add_argument('--isaac', default=DEFAULT_ISAAC, help='Isaac Sim 安裝路徑')
    args = ap.parse_args()

    try:
        import pxr  # noqa: F401
        import numpy  # noqa: F401
    except ImportError:
        _reexec_under_isaac(args)
        return
    run_worker(args)


def _reexec_under_isaac(args):
    """沒有 pxr -> 用 Isaac 自帶的 python 把自己重跑一次。"""
    isaac = os.path.expanduser(args.isaac)
    py = os.path.join(isaac, 'kit', 'python', 'bin', 'python3')
    usdlib = glob.glob(os.path.join(isaac, 'extscache', 'omni.usd.libs-*'))
    piplib = glob.glob(os.path.join(isaac, 'extscache', 'omni.kit.pip_archive-*', 'pip_prebundle'))
    complib = glob.glob(os.path.join(isaac, 'exts', 'omni.pip.compute', 'pip_prebundle'))
    if not os.path.exists(py) or not usdlib or not piplib:
        sys.exit(f'在 {isaac} 底下找不到 Isaac Sim 的 python / USD 函式庫。\n'
                 f'請用 --isaac 指定路徑, 或設環境變數 ISAAC_SIM_PATH。')

    env = {k: v for k, v in os.environ.items()
           if k not in ('CONDA_PREFIX', 'CONDA_DEFAULT_ENV', 'PYTHONHOME')}
    env['PYTHONPATH'] = ':'.join(usdlib + piplib + complib)
    env['LD_LIBRARY_PATH'] = os.path.join(usdlib[0], 'bin') + ':' + env.get('LD_LIBRARY_PATH', '')
    env['_CAR_LOC_REEXEC'] = '1'
    if os.environ.get('_CAR_LOC_REEXEC'):
        sys.exit('用 Isaac 的 python 重跑之後還是缺套件, 放棄。')
    os.execve(py, [py, os.path.abspath(__file__)] + sys.argv[1:], env)


if __name__ == '__main__':
    main()
