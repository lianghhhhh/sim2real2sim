#!/usr/bin/env python3
"""把 USD 裡的 BasisCurves (MPC 的循跡路線) 匯出成 CSV (x, y, z, 世界座標), 給 path_tracker 用。

不需要 Isaac 在跑, 用 Isaac 自帶的 python 跟 pxr (沒有 numpy, 所以這裡全是純 python):

    E=$(ls -d ~/isaac-sim/extscache/omni.usd.libs-*cp311 | head -1)
    PYTHONPATH=$E LD_LIBRARY_PATH=$E/bin:$HOME/isaac-sim/kit/python/lib \\
        ~/isaac-sim/kit/python/bin/python3 \\
        src/sensor_mpc_pkg/scripts/export_path_from_usd.py kros_car_mpc.usd \\
        -o src/sensor_mpc_pkg/config/path_kros_car_mpc.csv

也可以在 Isaac 的 Script Editor 裡 exec(open(...).read()) 後呼叫 export(stage, out_path)。
路線在 GUI 裡被搬過的話要重匯 (CSV 是當下的快照)。

Isaac 的 OmniGraph 版 (mlp_control_pkg/temp_curve.py) 是用 ComputeLocalToWorldTransform
把 points 轉到世界座標再找最近點, 這裡做一樣的事, 所以兩邊的路線是同一條。
"""
import argparse
import sys


def find_curves(stage):
    from pxr import UsdGeom
    found = []
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.BasisCurves):
            found.append(prim)
    return found


def curve_points_world(prim, time_code=None):
    from pxr import Gf, Usd, UsdGeom
    tc = Usd.TimeCode.Default() if time_code is None else time_code
    geom = UsdGeom.BasisCurves(prim)
    pts = geom.GetPointsAttr().Get(tc)
    counts = geom.GetCurveVertexCountsAttr().Get(tc) or [len(pts)]
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(tc)
    out = []
    for p in pts:
        w = xf.Transform(Gf.Vec3d(p[0], p[1], p[2]))
        out.append((float(w[0]), float(w[1]), float(w[2])))
    return out, list(counts)


def export(stage, out_path, prim_path=None, curve_index=0):
    curves = find_curves(stage)
    if prim_path:
        curves = [c for c in curves if str(c.GetPath()) == prim_path]
    else:
        # kros_car_mpc.usd 裡環境相機的視野框 (env_cam_i_footprint) 也是 BasisCurves, 跳過
        curves = [c for c in curves if not str(c.GetPath()).endswith('_footprint')] or curves
    if not curves:
        raise SystemExit('stage 裡找不到 BasisCurves' + (f' {prim_path}' if prim_path else ''))
    if len(curves) > 1:
        print('找到多條 BasisCurves, 用第一條 (--prim 指定別的):', file=sys.stderr)
        for c in curves:
            print('   ', c.GetPath(), file=sys.stderr)
    prim = curves[0]
    pts, counts = curve_points_world(prim)
    # 一個 prim 可能有好幾段 curve; 取第 curve_index 段
    start = sum(counts[:curve_index])
    seg = pts[start:start + counts[curve_index]]
    with open(out_path, 'w') as f:
        f.write(f'# source: {prim.GetPath()} ({len(seg)} points, world frame, metres)\n')
        f.write('x,y,z\n')
        for x, y, z in seg:
            f.write(f'{x:.6f},{y:.6f},{z:.6f}\n')
    print(f'{prim.GetPath()}: {len(seg)} 點 -> {out_path}')
    xs = [p[0] for p in seg]
    ys = [p[1] for p in seg]
    print(f'  x [{min(xs):+.3f}, {max(xs):+.3f}]  y [{min(ys):+.3f}, {max(ys):+.3f}]')
    return seg


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('usd')
    ap.add_argument('-o', '--out', default='path.csv')
    ap.add_argument('--prim', default=None, help='BasisCurves 的 prim path (預設: 找到的第一條)')
    ap.add_argument('--curve', type=int, default=0, help='同一個 prim 裡的第幾段 curve')
    a = ap.parse_args()
    from pxr import Usd
    stage = Usd.Stage.Open(a.usd)
    if stage is None:
        raise SystemExit(f'開不了 {a.usd}')
    export(stage, a.out, a.prim, a.curve)


if __name__ == '__main__':
    main()
