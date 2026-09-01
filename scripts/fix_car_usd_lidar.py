#!/usr/bin/env python3
"""修 car.usd 裡兩個讓 LiDAR 定位無法運作的設定。

1) LiDAR 被埋在車殼裡
   /World/small_car/Cube 尺寸 0.200 x 0.300 x 0.075, 世界 z [0.037, 0.112];
   四個輪子高 0.150 (頂端 z=0.15)。LiDAR 卻放在 world z=0.075 —— 正好是車身
   內部正中央, 也是輪子的半高。實測點雲最低仰角 +26.6 度 (= atan(0.075/0.15),
   剛好是輪子擋住的角度), 地面與下半部的牆完全看不到, 方位角只覆蓋 52.8%。
   -> 把感測器抬到車身與輪子上方。

2) ROS2RtxLidarHelper 的 fullScan 沒有設定, 預設是 False
   Isaac 因此「每個 render frame 發一小片」而不是一整圈: 實測每則訊息只有
   26~292 點 (完整一圈應該是 675x16=10800), 60Hz。任何掃描比對演算法都無法
   用這種資料工作。
   -> 設成 True, 改成累積滿一圈才發布 (20 Hz)。

用法 (先在 Isaac GUI 存檔, 再跑這個腳本, 然後重新載入場景):
    ./scripts/fix_car_usd_lidar.py                     # 就地修改 car.usd
    ./scripts/fix_car_usd_lidar.py --lidar-z 0.30      # 自訂掛載高度
    ./scripts/fix_car_usd_lidar.py --dry-run           # 只看會改什麼
"""
import argparse
import glob
import os
import subprocess
import sys

ISAAC = os.environ.get('ISAAC_SIM_PATH', os.path.expanduser('~/isaac-sim'))
LIDAR_PRIM = '/World/small_car/Cube/World/multiScan136'
HELPER_PRIM = '/World/small_car/Cube/ActionGraph/ros2_rtx_lidar_helper'
CAR_PRIM = '/World/small_car'

WORKER = r'''
import sys, math
from pxr import Usd, UsdGeom, Gf, Sdf

usd_path, margin, dry = sys.argv[1], float(sys.argv[2]), sys.argv[3] == '1'
LIDAR_PRIM, HELPER_PRIM, CAR_PRIM = sys.argv[4], sys.argv[5], sys.argv[6]
override = float(sys.argv[7]) if sys.argv[7] != 'auto' else None

stage = Usd.Stage.Open(usd_path)
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'], useExtentsHint=False)

lidar = stage.GetPrimAtPath(LIDAR_PRIM)
if not lidar.IsValid():
    print(f'找不到 {LIDAR_PRIM}'); sys.exit(1)

cur = UsdGeom.Xformable(lidar).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default()).ExtractTranslation()

# 掃過車上所有幾何 (排除 LiDAR 自己的子樹), 找出最高點與各自的遮蔽角
lidar_path = str(lidar.GetPath())
top, occluders = 0.0, []
for p in stage.Traverse():
    if not p.IsA(UsdGeom.Gprim):
        continue
    path = str(p.GetPath())
    if not path.startswith(CAR_PRIM) or path.startswith(lidar_path):
        continue
    b = cache.ComputeWorldBound(p).ComputeAlignedRange()
    if b.IsEmpty():
        continue
    mn, mx = b.GetMin(), b.GetMax()
    half = max(abs(mn[0] - cur[0]), abs(mx[0] - cur[0]),
               abs(mn[1] - cur[1]), abs(mx[1] - cur[1]))
    occluders.append((path.split('/')[-1], mx[2], half))
    top = max(top, mx[2])

print('車上幾何 (排除 LiDAR 本身):')
for name, z, half in sorted(occluders, key=lambda x: -x[1]):
    print(f'    {name:<22} 頂端 z={z:+.3f}  水平半徑 {half:.3f} m')

target = override if override is not None else top + margin

# 祖先節點可能有非單位縮放, 所以不能直接把世界高度差加到 local translate 上;
# 要用父節點的 local-to-world 矩陣做逆變換, 才能算出正確的 local 值。
parent_m = UsdGeom.Xformable(lidar.GetParent()).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default())
scale_z = parent_m.TransformDir(Gf.Vec3d(0, 0, 1))[2]
new_local = parent_m.GetInverse().Transform(Gf.Vec3d(cur[0], cur[1], target))
new_local_z = new_local[2]

print(f'\nLiDAR 世界高度 {cur[2]:.3f} -> {target:.3f}'
      + ('' if override is not None else f'  (車上最高點 {top:.3f} + 餘裕 {margin:.3f})'))
print(f'  父節點 z 縮放 {scale_z:.4f} -> local translate z = {new_local_z:.4f}')
worst = None
for name, z, half in occluders:
    up = z - target
    if up <= 0:
        continue
    ang = math.degrees(math.atan2(up, half)) if half > 1e-6 else 90.0
    worst = (name, ang) if worst is None or ang > worst[1] else worst
if worst:
    print(f'  移動後仍會被 {worst[0]} 擋到仰角 {worst[1]:+.1f} 度 -- 餘裕不夠, 請加大 --margin')
else:
    print(f'  移動後車上已無物件高過 LiDAR, 可以看到水平線以下')

op = lidar.GetAttribute('xformOp:translate')
old = op.Get() if op and op.HasAuthoredValue() else Gf.Vec3d(0, 0, 0)
if not dry:
    if not op:
        op = UsdGeom.Xformable(lidar).AddTranslateOp().GetAttr()
    op.Set(Gf.Vec3d(old[0], old[1], new_local_z))

helper = stage.GetPrimAtPath(HELPER_PRIM)
if not helper.IsValid():
    print(f'找不到 {HELPER_PRIM}'); sys.exit(1)
a = helper.GetAttribute('inputs:fullScan') or helper.CreateAttribute(
    'inputs:fullScan', Sdf.ValueTypeNames.Bool)
print(f'\nfullScan {a.Get()} -> True')
if not dry:
    a.Set(True)

# --- 時間源一致性 ---
# /clock 來自 on_playback_tick.time (按 Play 歸零), IMU 也跟著 playback;
# 但 RTX LiDAR 的 resetSimulationTimeOnStop 與 IsaacReadSimulationTime 的
# resetOnStop 預設都是 False (跨 Stop/Play 單調累加)。反覆 Stop/Play 之後
# 兩個時鐘會差開幾百甚至幾千秒, robot_localization 就會把 LiDAR odometry
# 當成未來資料整批丟掉 -> EKF 只剩 IMU, 位置永遠卡在原點。
print('\n時間源 (全部設成跟著 playback 歸零, 才會跟 /clock 一致):')
time_flags = [
    (HELPER_PRIM, 'inputs:resetSimulationTimeOnStop'),
    ('/World/ActionGraph_camera/ros2_camera_helper', 'inputs:resetSimulationTimeOnStop'),
]
for pr in stage.Traverse():
    if pr.GetAttribute('node:type') and \
            pr.GetAttribute('node:type').Get() == 'isaacsim.core.nodes.IsaacReadSimulationTime':
        time_flags.append((str(pr.GetPath()), 'inputs:resetOnStop'))
for prim_path, attr_name in time_flags:
    pr = stage.GetPrimAtPath(prim_path)
    if not pr.IsValid():
        continue
    at = pr.GetAttribute(attr_name) or pr.CreateAttribute(attr_name, Sdf.ValueTypeNames.Bool)
    cur_v = at.Get()
    print(f'    {prim_path.split("/")[-1]:<26} {attr_name.split(":")[-1]} '
          f'{cur_v if cur_v is not None else "(預設 False)"} -> True')
    if not dry:
        at.Set(True)

room = stage.GetPrimAtPath('/World/Room')
if room.IsValid():
    print('\n房間牆面 (地面在 z=0, 只有地面以上的部分掃得到):')
    for c in room.GetChildren():
        b = cache.ComputeWorldBound(c).ComputeAlignedRange()
        if b.IsEmpty():
            continue
        mn, mx = b.GetMin(), b.GetMax()
        note = '  <- 有一半埋在地面下' if mn[2] < -0.05 else ''
        print(f'    {c.GetName():<14} z[{mn[2]:+.2f},{mx[2]:+.2f}] '
              f'地面以上 {max(mx[2], 0):.2f} m{note}')

print(f'\n建議的 launch 參數: lidar_z:={target:.3f}')
if dry:
    print('(dry-run, 沒有寫入)')
else:
    check = UsdGeom.Xformable(lidar).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()[2]
    if abs(check - target) > 1e-4:
        print(f'錯誤: 寫入後 LiDAR 世界高度是 {check:.4f}, 不是預期的 {target:.4f}; '
              f'沒有存檔。')
        sys.exit(2)
    stage.GetRootLayer().Save()
    print(f'已寫回 {usd_path} (驗證世界高度 = {check:.4f})')
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('usd', nargs='?',
                    default=os.path.join(os.path.dirname(os.path.dirname(
                        os.path.abspath(__file__))), 'car.usd'))
    ap.add_argument('--margin', type=float, default=0.05,
                    help='LiDAR 要高過車上最高點多少 (m), 預設 0.05')
    ap.add_argument('--lidar-z', type=float, default=None,
                    help='直接指定 LiDAR 的目標世界高度 (m); 不給就自動算')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    usdlib = glob.glob(os.path.join(ISAAC, 'extscache', 'omni.usd.libs-*'))
    if not usdlib:
        sys.exit(f'在 {ISAAC} 底下找不到 omni.usd.libs, 請設 ISAAC_SIM_PATH')
    env = {k: v for k, v in os.environ.items()
           if k not in ('CONDA_PREFIX', 'CONDA_DEFAULT_ENV', 'PYTHONHOME')}
    env['PYTHONPATH'] = usdlib[0]
    env['LD_LIBRARY_PATH'] = (os.path.join(usdlib[0], 'bin') + ':'
                              + env.get('LD_LIBRARY_PATH', ''))

    worker = '/tmp/_fix_car_usd_worker.py'
    with open(worker, 'w') as f:
        f.write(WORKER)

    print(f'目標檔案: {args.usd}')
    r = subprocess.run(
        [os.path.join(ISAAC, 'python.sh'), worker, args.usd, str(args.margin),
         '1' if args.dry_run else '0', LIDAR_PRIM, HELPER_PRIM, CAR_PRIM,
         'auto' if args.lidar_z is None else str(args.lidar_z)],
        cwd=ISAAC, env=env, capture_output=True, text=True)
    for line in r.stdout.splitlines():
        if not line.startswith('[') and 'conda' not in line:
            print(line)
    if r.returncode != 0:
        print(r.stderr[-2000:], file=sys.stderr)
        sys.exit(r.returncode)
    if r.stderr and 'Traceback' in r.stderr:
        print(r.stderr[-2000:], file=sys.stderr)


if __name__ == '__main__':
    main()
