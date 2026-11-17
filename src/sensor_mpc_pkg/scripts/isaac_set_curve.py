"""Isaac Script Editor 片段: 把路徑 CSV 寫進目前開著的 stage 的 BasisCurves (只是為了在 GUI 裡看得到路線,
sensor_mpc_pkg 的 path_tracker 讀的是 CSV, 不讀 USD)。

用法 (Window -> Script Editor, 改 CSV 路徑後 Run):

    CSV = '/home/liang/sim2real2sim/src/sensor_mpc_pkg/config/path_kros_car_mpc_sensor.csv'
    exec(open('/home/liang/sim2real2sim/src/sensor_mpc_pkg/scripts/isaac_set_curve.py').read())
    set_curve(CSV)                       # 預設寫到 /World/BasisCurves/BasisCurves
    set_curve(CSV, '/World/MyCurve')     # 不存在的話會新建一條

CSV 是世界座標; 這裡會扣掉 prim 的 xform 再寫 points, 所以 prim 本身有被搬過也沒關係。
改完要自己 File -> Save (或不存, 只看)。原本的 ScriptNode (temp_curve) 若還在, 它跟 path_tracker
算的是同一條線, 互不影響 —— MPC 節點吃的是 /sensor_mpc/nearest_points。
"""
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, Vt


def _read_csv(path):
    pts = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            parts = s.split(',')
            try:
                pts.append((float(parts[0]), float(parts[1]),
                            float(parts[2]) if len(parts) > 2 else 0.0))
            except ValueError:
                continue
    return pts


def set_curve(csv_path, prim_path='/World/BasisCurves/BasisCurves', z=0.0, width=0.03, closed=True):
    stage = omni.usd.get_context().get_stage()
    world = _read_csv(csv_path)
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        curve = UsdGeom.BasisCurves.Define(stage, Sdf.Path(prim_path))
        curve.CreateTypeAttr().Set('linear')
        curve.CreateBasisAttr().Set('bezier')
        prim = curve.GetPrim()
        print(f'新建 {prim_path}')
    curve = UsdGeom.BasisCurves(prim)
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    inv = xf.GetInverse()
    local = [inv.Transform(Gf.Vec3d(x, y, z)) for x, y, _ in world]
    curve.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]))
                                             for p in local]))
    curve.GetCurveVertexCountsAttr().Set(Vt.IntArray([len(local)]))
    curve.CreateWrapAttr().Set('periodic' if closed else 'nonperiodic')
    curve.CreateWidthsAttr().Set(Vt.FloatArray([width]))
    curve.GetExtentAttr().Set(UsdGeom.PointBased(prim).ComputeExtent(curve.GetPointsAttr().Get()))
    xs = [p[0] for p in world]
    ys = [p[1] for p in world]
    print(f'{prim_path}: {len(local)} 點, x [{min(xs):+.2f}, {max(xs):+.2f}] y [{min(ys):+.2f}, {max(ys):+.2f}]')
