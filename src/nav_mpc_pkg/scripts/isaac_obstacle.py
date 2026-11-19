"""Isaac Script Editor 片段: 模擬跑到一半在場景裡放 / 拿掉障礙物 (測 nav_mpc 的停車與重新規劃)。

用法 (Window -> Script Editor; Play 中也可以執行):

    exec(open('/home/liang/sim2real2sim/src/nav_mpc_pkg/scripts/isaac_obstacle.py').read())
    add_obstacle_ahead(1.6)              # 車子正前方 1.6 m 放一個 0.4 m 方塊 (高 0.6 m)
    add_obstacle(2.0, 0.5)               # 放在世界座標 (2.0, 0.5)
    add_obstacle(2.0, 0.5, size=0.3, height=0.4)
    move_obstacle('Box_0', 2.5, -0.5)    # 搬走 (雷射掃過舊位置, costmap 會自己清掉)
    clear_obstacles()                    # 全部拿掉

方塊放在 /World/NavObstacles 底下, 是靜態碰撞體 (沒有剛體, 車子撞上去推不動)。RTX 雷射看得到它,
所以 /scan 會有、Nav2 的 costmap 會畫上去; 它在 LiDAR 定位的地圖裡沒有, 那幾個點對配準來說是外點
(inlier 比例會掉一點, 正常)。**只改記憶體裡的 stage, 不要存檔** (存了下次開檔就一直在)。

高度要蓋到雷射的掃描平面 (laser_frame 離地 ~0.24 m + 車身), 預設 0.6 m。
"""
import math

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

ROOT = '/World/NavObstacles'
CAR = '/kros_car/base_link'


def _stage():
    return omni.usd.get_context().get_stage()


def car_pose():
    """(x, y, yaw) of the car. Play 中 USD 的 xform 不一定跟著物理更新, 先問 Isaac 的 prim API。"""
    try:
        from isaacsim.core.prims import XFormPrim
        pos, quat = XFormPrim(CAR).get_world_poses()
        x, y = float(pos[0][0]), float(pos[0][1])
        w, qx, qy, qz = [float(v) for v in quat[0]]
    except Exception as e:                               # noqa: BLE001
        print(f'[isaac_obstacle] XFormPrim 讀不到 ({e}), 改讀 USD (Play 中可能是舊的位置)')
        m = omni.usd.get_world_transform_matrix(_stage().GetPrimAtPath(CAR))
        t, q = m.ExtractTranslation(), m.ExtractRotationQuat()
        x, y = t[0], t[1]
        w, (qx, qy, qz) = q.GetReal(), q.GetImaginary()
    yaw = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return x, y, yaw


def add_obstacle(x, y, size=0.4, height=0.6, name=None):
    stage = _stage()
    UsdGeom.Xform.Define(stage, Sdf.Path(ROOT))
    if name is None:
        k = 0
        while stage.GetPrimAtPath(f'{ROOT}/Box_{k}').IsValid():
            k += 1
        name = f'Box_{k}'
    path = f'{ROOT}/{name}'
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.35, 0.1)])
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, height / 2.0))
    xf.AddScaleOp().Set(Gf.Vec3f(size, size, height))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    print(f'[isaac_obstacle] {path}: ({x:+.2f}, {y:+.2f}) 邊長 {size} m 高 {height} m')
    return path


def add_obstacle_ahead(dist=1.6, size=0.4, height=0.6, lateral=0.0):
    """車子正前方 dist 公尺 (車頭方向, 不是沿路線), 往左偏 lateral 公尺。"""
    x, y, yaw = car_pose()
    ox = x + dist * math.cos(yaw) - lateral * math.sin(yaw)
    oy = y + dist * math.sin(yaw) + lateral * math.cos(yaw)
    print(f'[isaac_obstacle] 車子 ({x:+.2f}, {y:+.2f}) yaw {math.degrees(yaw):+.0f}°')
    return add_obstacle(ox, oy, size, height)


def move_obstacle(name, x, y):
    prim = _stage().GetPrimAtPath(f'{ROOT}/{name}')
    if not prim.IsValid():
        print(f'[isaac_obstacle] 沒有 {ROOT}/{name}')
        return
    op = UsdGeom.Xformable(prim).GetOrderedXformOps()[0]
    z = op.Get()[2]
    op.Set(Gf.Vec3d(x, y, z))
    print(f'[isaac_obstacle] {name} -> ({x:+.2f}, {y:+.2f})')


def clear_obstacles():
    stage = _stage()
    if stage.GetPrimAtPath(ROOT).IsValid():
        stage.RemovePrim(ROOT)
        print(f'[isaac_obstacle] 拿掉 {ROOT}')
