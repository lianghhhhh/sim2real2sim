#!/usr/bin/env python3
"""從 USD 匯出 car_loc_multicam 要的兩個設定檔 (**只讀 USD, 不會改它**):

    config/env_cameras.yaml   每台環境相機的內參 + 外參 (/World/EnvCameras/env_cam_*)
    config/car_model.yaml     車的 3D 外形 (visual mesh 的凸包頂點, base_link 座標)

需要 pxr (USD 的 Python 綁定)。最簡單是借 Isaac 的:

    ISAAC=~/isaac-sim
    USDLIB=$(ls -d $ISAAC/extscache/omni.usd.libs-*)
    NP=$(ls -d $ISAAC/extscache/omni.kit.pip_archive-*/pip_prebundle)
    PYTHONPATH=$USDLIB:$NP LD_LIBRARY_PATH=$USDLIB/bin \\
        $ISAAC/kit/python/bin/python3 src/car_loc_multicam/scripts/export_from_usd.py \\
        kros_car_new.usd --out src/car_loc_multicam/config

(`~/isaac-sim/python.sh` 預設的路徑裡沒有 pxr, 所以要自己指。)
也可以在 Isaac 的 Script Editor 裡跑 (讀目前開著的 stage, 參數用下面的 SCRIPT_EDITOR_ARGS)。
**車型要這樣匯出**: 手臂在按下 Play 之後才會被關節驅動到實際姿勢, 而 USD 裡的姿勢要
Physics 設定的 "Update to USD" (/physics/updateToUsd) 打開才會跟著更新 ——
打開、Play 幾秒、再跑這支。

什麼時候要重跑
--------------
* 相機搬了 / 換鏡頭 / 改 spawn_env_camera_circle.py 的 RIG 或 LENS
* 車的外形變了 (加零件、手臂換預設姿勢)
render 解析度不在 USD 裡 (在 render product 上), 用 --width / --height 給;
預設是 IMX296 的 1456 x 1088, 跟 spawn_env_camera_circle.py 一樣。
"""
import argparse
import math
import os
import sys

import numpy as np
from pxr import Gf, Usd, UsdGeom

FISHEYE = 'omni:lensdistortion:opencvFisheye:'

# 在 Script Editor 裡跑的時候用這組參數 (那裡沒辦法給命令列參數)。
# --exclude grap,mesh_3: YOLO 不框夾爪與車頂 LiDAR (見 config/car_model.yaml 開頭)
SCRIPT_EDITOR_ARGS = ['--out', '/tmp/car_loc_multicam_export', '--exclude', 'grap,mesh_3']


def _quat_wxyz(R):
    """3x3 旋轉矩陣 (列向量慣例) -> 四元數 [w, x, y, z]。"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1)


def _world_rot(m):
    """Gf 矩陣 (列向量在左的慣例) -> 世界旋轉 (行向量慣例), 每一行是一根本地軸。"""
    cols = [m.TransformDir(Gf.Vec3d(*e)) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    R = np.array([[c[0], c[1], c[2]] for c in cols]).T
    # 去掉縮放
    return R / np.linalg.norm(R, axis=0, keepdims=True)


def export_cameras(stage, root, width, height):
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    cams = []
    rootp = stage.GetPrimAtPath(root)
    if not rootp:
        sys.exit(f'stage 裡沒有 {root} —— 先跑 scripts/spawn_env_camera_circle.py 生相機')
    for p in Usd.PrimRange(rootp):
        if not p.IsA(UsdGeom.Camera):
            continue
        c = UsdGeom.Camera(p)
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        f = c.GetFocalLengthAttr().Get()
        ha = c.GetHorizontalApertureAttr().Get()
        ho = c.GetHorizontalApertureOffsetAttr().Get() or 0.0
        vo = c.GetVerticalApertureOffsetAttr().Get() or 0.0
        # Isaac 的 render 是方形像素: 垂直視角由水平光圈 + 解析度長寬比決定,
        # verticalAperture 不影響內參, 所以 fy = fx
        fx = f / ha * width
        d = dict(name=p.GetName(), fx=fx, fy=fx,
                 cx=width / 2.0 + ho / ha * width, cy=height / 2.0 - vo / ha * width,
                 position=list(np.array(m.ExtractTranslation()) * mpu),
                 quat=_quat_wxyz(_world_rot(m)), fisheye=None)
        a = p.GetAttribute(FISHEYE + 'k1')
        if a and a.Get() is not None:
            d['fisheye'] = [float(p.GetAttribute(FISHEYE + f'k{i}').Get()) for i in range(1, 5)]
            for k in ('fx', 'fy', 'cx', 'cy'):
                v = p.GetAttribute(FISHEYE + k)
                if v and v.Get() is not None:
                    d[k] = float(v.Get())
        cams.append(d)
    return cams


def _fib_sphere(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], axis=1)


def support_points(P, n_dirs=3000):
    """凸包頂點的近似: 在 n_dirs 個方向上各取最外面那一點。

    外接框的每個邊都是「某個方向上最外面的那一點」決定的, 所以這正是需要的東西。
    3000 個方向的角間距約 3.7 度, 對輪子這種圓的外形誤差 < 0.1 mm。
    """
    P = np.unique(np.round(P, 4), axis=0)          # 先去掉 0.1 mm 以內的重複點
    D = _fib_sphere(n_dirs)
    keep = set()
    for k in range(0, n_dirs, 250):
        keep.update(np.argmax(P @ D[k:k + 250].T, axis=0).tolist())
    return P[sorted(keep)]


def _axis_frame(axis):
    """把 (x, y, 沿軸) 的點換成 USD 的 axis 慣例 (X / Y / Z)。"""
    return {'X': [2, 0, 1], 'Y': [1, 2, 0]}.get(axis, [0, 1, 2])


def _gprim_points(p):
    """一個幾何 prim 在它自己座標系裡的表面點 (N, 3)。

    不只 Mesh: URDF 匯入時簡單的零件會變成 Cube / Cylinder / Sphere (kros_car 的
    LiDAR 就是一個 Cylinder, 在車頂最高的地方), 漏掉它的話斜拍的外接框上緣會差一截。
    """
    if p.IsA(UsdGeom.Mesh):
        v = UsdGeom.Mesh(p).GetPointsAttr().Get()
        return None if not v else np.asarray(v, dtype=np.float64)
    if p.IsA(UsdGeom.Cube):
        h = 0.5 * (UsdGeom.Cube(p).GetSizeAttr().Get() or 2.0)
        return np.array([[x, y, z] for x in (-h, h) for y in (-h, h) for z in (-h, h)])
    if p.IsA(UsdGeom.Sphere):
        return _fib_sphere(200) * (UsdGeom.Sphere(p).GetRadiusAttr().Get() or 1.0)
    if p.IsA(UsdGeom.Cylinder) or p.IsA(UsdGeom.Capsule):
        g = UsdGeom.Cylinder(p) if p.IsA(UsdGeom.Cylinder) else UsdGeom.Capsule(p)
        r = g.GetRadiusAttr().Get() or 1.0
        h = 0.5 * (g.GetHeightAttr().Get() or 2.0)
        a = np.linspace(0, 2 * math.pi, 64, endpoint=False)
        ring = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
        P = np.vstack([np.c_[ring, np.full(64, -h)], np.c_[ring, np.full(64, h)]])
        if p.IsA(UsdGeom.Capsule):
            s = _fib_sphere(200) * r
            P = np.vstack([P, s + [0, 0, h], s - [0, 0, h]])
        return P[:, _axis_frame(g.GetAxisAttr().Get())]
    return None


def export_hull(stage, car, base, exclude=()):
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    basep = stage.GetPrimAtPath(base)
    if not basep:
        sys.exit(f'stage 裡沒有 {base}')
    inv = UsdGeom.Xformable(basep).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
    pts = []
    # visuals 通常是 instanceable 的 (URDF 匯入的慣例), 不走 instance proxy 就看不到 mesh
    for p in Usd.PrimRange(stage.GetPrimAtPath(car), Usd.TraverseInstanceProxies()):
        path = p.GetPath().pathString
        if not p.IsA(UsdGeom.Gprim) or '/visuals' not in path:
            continue
        if any(e and e in path for e in exclude):
            continue
        if UsdGeom.Imageable(p).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        V = _gprim_points(p)
        if V is None or not len(V):
            continue
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()) * inv
        M = np.array(m)                                   # 列向量在左: p' = p @ M
        pts.append((np.c_[V, np.ones(len(V))] @ M)[:, :3] * mpu)
    if not pts:
        sys.exit(f'{car} 底下找不到 visuals 幾何')
    P = np.vstack(pts)
    return P, support_points(P)


def write_cameras(path, cams, width, height, src):
    L = [f'# 由 scripts/export_from_usd.py 從 {os.path.basename(src)} 匯出, 別手改 —— 相機動了就重跑。',
         '# 內參是 render 解析度 width x height 下的值 (Isaac 的 USD 相機是方形像素, fy = fx)。',
         '# 外參 usd_quat_wxyz = USD 相機 (-Z 朝前, +Y 朝上) 的世界姿態; camera_model.py 會轉成 OpenCV。',
         '# 實體相機改用 rvec / tvec (solvePnP 的輸出), 見 README「實體相機」。',
         f'width: {width}', f'height: {height}', 'cameras:']
    for c in cams:
        q = ', '.join(f'{v:.9f}' for v in c['quat'])
        pos = ', '.join(f'{v:.4f}' for v in c['position'])
        L += [f"  - name: {c['name']}",
              f"    image_topic: /{c['name']}/rgb",
              f"    detection_topic: /{c['name']}/car_bbox",
              f"    fx: {c['fx']:.4f}", f"    fy: {c['fy']:.4f}",
              f"    cx: {c['cx']:.4f}", f"    cy: {c['cy']:.4f}",
              f'    position: [{pos}]', f'    usd_quat_wxyz: [{q}]']
        if c['fisheye']:
            L.append('    fisheye_k: [' + ', '.join(f'{v:.9g}' for v in c['fisheye']) + ']')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')


def write_hull(path, P_all, H, src, exclude=()):
    lo, hi = P_all.min(axis=0), P_all.max(axis=0)
    base_z = -float(lo[2])
    L = [f'# 由 scripts/export_from_usd.py 從 {os.path.basename(src)} 匯出 (visual mesh 的凸包頂點)。',
         '# 座標是 base_link (m), x = 車頭, z = 上。手臂姿勢 / 外形改了就重跑。',
         f'# 外形範圍 x {lo[0]:+.3f}~{hi[0]:+.3f}, y {lo[1]:+.3f}~{hi[1]:+.3f}, '
         f'z {lo[2]:+.3f}~{hi[2]:+.3f} m ({len(P_all)} 個 mesh 頂點 -> {len(H)} 個凸包點)',
         '# 排除的零件 (prim 路徑含這些字): ' + (', '.join(exclude) if exclude else '無'),
         '#',
         '# base_z = base_link 原點離地面多高。這裡用「最低的頂點貼地」算 (= 輪子底);',
         '# 車子停在平地上時就是這個值。地板不在 z=0 的話要自己加上地板高度。',
         f'base_z: {base_z:.4f}',
         '# YOLO 的框 - 外形的外接框, 每個邊 [u1, v1, u2, v2] 的平均差 (px)。',
         '# 量法: scripts/record_bbox.py 錄一段, 再 scripts/fit_car_model.py。量到之前先 0。',
         'edge_bias_px: [0.0, 0.0, 0.0, 0.0]',
         '# 讀進來之後整個外形平移多少 (base_link, m)。YOLO 的框跟凸包差一個固定量時用;',
         '# 量法: 停車時多相機定位 vs GT 沿車頭方向的平均誤差 (見 car_model.yaml 的說明)。',
         '# 重新匯出之後要重量, 先 0。',
         'points_shift: [0.0, 0.0, 0.0]',
         '# 沒有 points 時的備案: 一個長方體 (min / max 兩個角)',
         'box:',
         f'  min: [{lo[0]:.4f}, {lo[1]:.4f}, {lo[2]:.4f}]',
         f'  max: [{hi[0]:.4f}, {hi[1]:.4f}, {hi[2]:.4f}]',
         'points:']
    L += [f'  - [{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}]' for p in H]
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('usd', nargs='?', default='', help='空 = Isaac Script Editor 裡目前的 stage')
    try:
        default_out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
    except NameError:                        # Script Editor 裡沒有 __file__
        default_out = '/tmp/car_loc_multicam_export'
    ap.add_argument('--out', default=default_out)
    ap.add_argument('--cameras-root', default='/World/EnvCameras')
    ap.add_argument('--car', default='/kros_car')
    ap.add_argument('--base', default='/kros_car/base_link')
    ap.add_argument('--width', type=int, default=1456)
    ap.add_argument('--height', type=int, default=1088)
    ap.add_argument('--only', choices=['cameras', 'car', 'both'], default='both')
    # YOLO 不框的細長零件。2026-10-30 在 Isaac 斜拍畫面上看到: YOLO 的框不含夾爪尖端,
    # 也不含車頂的 LiDAR 圓盤 —— 外形把它們算進去的話, 那個邊就固定差 10~20 px。
    ap.add_argument('--exclude', default='',
                    help='逗號分隔; prim 路徑含其中任一字串的零件不算進外形 (例: grap,mesh_3)')
    ap.add_argument('--name', default='car_model.yaml', help='輸出檔名')
    a = ap.parse_args(argv)

    if a.usd:
        stage = Usd.Stage.Open(os.path.abspath(a.usd))
        src = a.usd
    else:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        src = stage.GetRootLayer().identifier
    os.makedirs(a.out, exist_ok=True)

    if a.only in ('cameras', 'both'):
        cams = export_cameras(stage, a.cameras_root, a.width, a.height)
        p = os.path.join(a.out, 'env_cameras.yaml')
        write_cameras(p, cams, a.width, a.height, src)
        print(f'{len(cams)} 台相機 -> {p}')
    if a.only in ('car', 'both'):
        ex = [e.strip() for e in a.exclude.split(',') if e.strip()]
        P, H = export_hull(stage, a.car, a.base, ex)
        p = os.path.join(a.out, a.name)
        write_hull(p, P, H, src, ex)
        lo, hi = P.min(axis=0), P.max(axis=0)
        print(f'車的外形 {len(P)} 點 -> 凸包 {len(H)} 點, 範圍 {lo.round(3)} ~ {hi.round(3)} -> {p}')


if __name__ == '__main__':
    main(SCRIPT_EDITOR_ARGS if 'omni.kit' in sys.modules else None)
