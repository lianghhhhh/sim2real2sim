#!/usr/bin/env python3
"""把 car.usd 裡的 OmniLidar 設成 Oradar MS200 (2D 單線), 並改好 ActionGraph。

為什麼需要這個腳本
------------------
`IsaacSensorCreateRtxLidar` 的 `config=` 參數在 Isaac Sim 5.x **只認註冊過的
名字** (`SUPPORTED_LIDAR_CONFIGS`, 對應 assets server 上的 .usd), 給一個 JSON
的絕對路徑不會生效 —— 它只會印一行 warning, 然後退回預設的 Generic/LidarCore:
128 線、仰角 ±15 度、量程 200 m。那不是 2D 雷射, 掃描比對會拿到一坨立體點雲。

而 `data/lidar_configs/` 那個 JSON 資料夾, 在 extension.toml 裡註明是給
「(deprecated) camera-based Lidar」用的。**新的 OmniLidar prim 是直接讀 USD
屬性 `omni:sensor:Core:*`**, 所以正確做法就是把 profile 寫進屬性 —— 這個腳本
做的就是這件事, 數值全部來自 config/oradar_ms200.json。

順便修好 ActionGraph 的三件事:
  1. isaac_create_render_product 的 675x16 -> 1x1
     (RTX 感測器的射線圖樣由 profile 決定, render product 只是給它一塊畫布;
      Isaac 自己的 standalone_examples/api/isaacsim.ros2.bridge/rtx_lidar.py
      用的就是 [1, 1]。675x16 是上一顆 SICK multiScan136 留下來的。)
  2. ros2_rtx_lidar_helper 的 type: point_cloud -> laser_scan
     真的 MS200 驅動發的就是 LaserScan, 這樣模擬跟實體車完全一致。
     laser_scan 走的是 IsaacComputeRTXLidarFlatScan。
  3. fullScan: False -> True (**laser_scan 也吃這一項**)
     文件說 "Output execution triggers when lidar sensor has accumulated a full
     scan", 曾經據此判斷 laser_scan 不需要 fullScan —— **那是錯的, 已經量過**:
     fullScan=False 時每則 /scan 只有「當下那個 render 批次」掃過的方位角有值,
     其餘 bin 一律填 **-1**。60 FPS 算繪 + 10 Hz 轉速 = 一批 60 度, 實測 450 個
     bin 只有 150~200 個是真的, 而且空洞的位置每幀移動 60 度 (30 幀的有效 bin
     聯集 67%, 交集只有 1%)。
     症狀: 掃描在 rviz/Foxglove 上是幾段斷開的弧; 配準的 yaw 只剩 1/3 的點在撐,
     每幀 yaw 雜訊 1.3 度, 建圖時隨機遊走累積 -> 房間被抹成同心圓。
     (-1 這個填充值還會被下游當成半徑 1 m 的點, 見 car_loc_lidar/scan.py 的
      laserscan_to_xyz —— 那邊也修過了。)
  4. topicName: lidar/point_cloud -> scan, 並把 frameId 寫死

用法 (先在 Isaac GUI 存檔, 再跑這個腳本, 然後重新載入場景):
    ./scripts/setup_oradar_lidar.py --dry-run      # 只看會改什麼
    ./scripts/setup_oradar_lidar.py
    ./scripts/setup_oradar_lidar.py --publish both # 另外再發一份 point_cloud
    ./scripts/setup_oradar_lidar.py --lidar-z 0.20 # 改掛載高度
"""
import argparse
import glob
import os
import subprocess
import sys

ISAAC = os.environ.get('ISAAC_SIM_PATH', os.path.expanduser('~/isaac-sim'))
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

LIDAR_PRIM = '/World/small_car/Cube/oradar_ms200'
GRAPH_PRIM = '/World/small_car/Cube/ActionGraph_lidar'
PROFILE = os.path.join(REPO, 'src/car_loc_lidar/config/oradar_ms200.json')

WORKER = r'''
import json, sys
from pxr import Usd, UsdGeom, Gf, Sdf, Vt

usd, profile_path, dry = sys.argv[1], sys.argv[2], sys.argv[3] == '1'
LIDAR, GRAPH = sys.argv[4], sys.argv[5]
publish, frame_id, lidar_z = sys.argv[6], sys.argv[7], sys.argv[8]

stage = Usd.Stage.Open(usd)
lidar = stage.GetPrimAtPath(LIDAR)
if not lidar.IsValid():
    print(f'找不到 {LIDAR}'); sys.exit(1)
prof = json.load(open(profile_path))['profile']

changes = []

def same(a, b):
    """USD 把 float 存成 float32, 讀回來的 0.05 是 0.05000000074505806 ——
    直接用 == 比會永遠不相等, 腳本就不冪等了 (每次跑都說「有 28 項要改」)。"""
    if isinstance(a, float) and isinstance(b, float):
        return abs(a - b) <= 1e-6 * max(1.0, abs(b))
    if hasattr(a, '__len__') and hasattr(b, '__len__') and not isinstance(a, str):
        return len(a) == len(b) and all(same(float(x), float(y)) if
                                        isinstance(y, float) else x == y
                                        for x, y in zip(a, b))
    return a == b


def put(prim, name, value, sdftype=None):
    a = prim.GetAttribute(name)
    if not a:
        if sdftype is None:
            print(f'  ! 沒有屬性 {name}, 跳過'); return
        a = prim.CreateAttribute(name, sdftype, False)
    old = a.Get()
    if same(old, value):
        return
    changes.append((str(prim.GetPath()).split("/")[-1], name, old, value))
    if not dry:
        a.Set(value)

# ---------------------------------------------------------------- 感測器 profile
C = 'omni:sensor:Core:'
n_emit = int(prof['numberOfEmitters'])
es = prof['emitterStates'][0]

put(lidar, C + 'scanType', prof['scanType'].upper())
put(lidar, C + 'rayType', prof['rayType'].upper())
put(lidar, C + 'rotationDirection', prof['rotationDirection'].upper())
put(lidar, C + 'intensityProcessing', prof['intensityProcessing'].upper())
put(lidar, C + 'intensityMappingType', prof['intensityMappingType'].upper())

put(lidar, C + 'nearRangeM', float(prof['nearRangeM']))
put(lidar, C + 'farRangeM', float(prof['farRangeM']))
put(lidar, C + 'minDistBetweenEchosM', float(prof['minDistBetweenEchos']))
put(lidar, C + 'rangeResolutionM', float(prof['rangeResolutionM']))
put(lidar, C + 'rangeAccuracyM', float(prof['rangeAccuracyM']))
put(lidar, C + 'avgPowerW', float(prof['avgPowerW']))
put(lidar, C + 'minReflectance', float(prof['minReflectance']))
put(lidar, C + 'minReflectionRangeM', float(prof['minReflectanceRange']))
put(lidar, C + 'waveLengthNm', float(prof['wavelengthNm']))
put(lidar, C + 'pulseTimeNs', int(prof['pulseTimeNs']))
put(lidar, C + 'azimuthErrorMean', float(prof['azimuthErrorMean']))
put(lidar, C + 'azimuthErrorStd', float(prof['azimuthErrorStd']))
put(lidar, C + 'elevationErrorMean', float(prof['elevationErrorMean']))
put(lidar, C + 'elevationErrorStd', float(prof['elevationErrorStd']))
put(lidar, C + 'maxReturns', int(prof['maxReturns']))
put(lidar, C + 'scanRateBaseHz', int(prof['scanRateBaseHz']))
put(lidar, C + 'reportRateBaseHz', int(prof['reportRateBaseHz']))

# 這四個是「變成單線 2D」的關鍵。預設是 128 線、仰角 -15~+15 度。
put(lidar, C + 'numberOfEmitters', n_emit)
put(lidar, C + 'numberOfChannels', n_emit)
put(lidar, C + 'emitterState:s001:azimuthDeg',
    Vt.FloatArray([float(v) for v in es['azimuthDeg']]))
put(lidar, C + 'emitterState:s001:elevationDeg',
    Vt.FloatArray([float(v) for v in es['elevationDeg']]))
put(lidar, C + 'emitterState:s001:fireTimeNs',
    Vt.UIntArray([int(v) for v in es['fireTimeNs']]))
put(lidar, C + 'emitterState:s001:channelId',
    Vt.UIntArray(list(range(1, n_emit + 1))))

put(lidar, C + 'validStartAzimuthDeg', 0.0)
put(lidar, C + 'validEndAzimuthDeg', 360.0)
# 索引 = 發射順序 = 時間, 運動補償靠這個。無效回波要留在陣列裡當佔位。
put(lidar, C + 'skipDroppingInvalidPoints', True)
put(lidar, 'omni:sensor:tickRate', float(prof['scanRateBaseHz']))
put(lidar, 'omni:sensor:marketName', 'MS200')
put(lidar, 'omni:sensor:modelName', 'Oradar MS200')
put(lidar, 'omni:sensor:modelVendor', 'Oradar')

# ---------------------------------------------------------------- 掛載高度
def local_matrix(prim):
    order = prim.GetAttribute('xformOpOrder').Get() or []
    M = Gf.Matrix4d(1.0)
    for name in order:
        v = prim.GetAttribute(name).Get()
        if v is None:
            continue
        m = Gf.Matrix4d(1.0)
        if name.endswith('translate'):
            m.SetTranslate(Gf.Vec3d(v))
        elif name.endswith('scale'):
            m.SetScale(Gf.Vec3d(v))
        elif name.endswith('orient'):
            m.SetRotate(Gf.Quatd(v))
        elif name.endswith('rotateXYZ'):
            m = (Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1,0,0), v[0])) *
                 Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,1,0), v[1])) *
                 Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1), v[2])))
        M = m * M
    return M

def world_matrix(prim):
    M, p = Gf.Matrix4d(1.0), prim
    while p and str(p.GetPath()) != '/':
        M = M * local_matrix(p)
        p = p.GetParent()
    return M

# OmniLidar 的 schema 在純 USD 環境沒註冊, UsdGeom 的內建計算會回父層的矩陣,
# 所以自己從 authored 的 xformOp 組。
W = world_matrix(lidar)
t = W.ExtractTranslation()
sc = tuple(Gf.Vec3d(W[i][0], W[i][1], W[i][2]).GetLength() for i in range(3))
print(f'掛載位置: world ({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}), scale '
      f'({sc[0]:.4f}, {sc[1]:.4f}, {sc[2]:.4f})')
if max(abs(s - 1.0) for s in sc) > 1e-3:
    print('  ! 世界 scale 不是 1 —— 感測器被父層縮放了, 量到的距離會不對。')
    print('    父層 Cube 有 scale (0.2, 0.3, 0.075), 子層要用倒數補回來。')

if lidar_z != 'keep':
    target = float(lidar_z)
    parent = lidar.GetParent()
    pw = world_matrix(parent)
    pz = pw.ExtractTranslation()[2]
    pscale = Gf.Vec3d(pw[2][0], pw[2][1], pw[2][2]).GetLength()
    local_z = (target - pz) / pscale
    a = lidar.GetAttribute('xformOp:translate')
    old = a.Get()
    new = Gf.Vec3d(old[0], old[1], local_z)
    if abs(old[2] - local_z) > 1e-9:
        changes.append(('oradar_ms200', 'xformOp:translate', old, new))
        if not dry:
            a.Set(new)
        print(f'  掛載高度 -> world z = {target:.4f} (local z = {local_z:.6f})')

# ---------------------------------------------------------------- ActionGraph
rp = stage.GetPrimAtPath(GRAPH + '/isaac_create_render_product')
if rp.IsValid():
    # RTX 感測器的射線圖樣由 profile 決定, render product 只是一塊畫布。
    put(rp, 'inputs:width', 1)
    put(rp, 'inputs:height', 1)
    rel = rp.GetRelationship('inputs:cameraPrim')
    tg = [str(x) for x in rel.GetTargets()] if rel else []
    if tg != [LIDAR]:
        print(f'  ! cameraPrim 指向 {tg}, 不是 {LIDAR}')
        if not dry:
            rel.SetTargets([Sdf.Path(LIDAR)])
        changes.append(('isaac_create_render_product', 'inputs:cameraPrim', tg, [LIDAR]))
else:
    print(f'找不到 {GRAPH}/isaac_create_render_product'); sys.exit(1)

helper = stage.GetPrimAtPath(GRAPH + '/ros2_rtx_lidar_helper')
if not helper.IsValid():
    print(f'找不到 {GRAPH}/ros2_rtx_lidar_helper'); sys.exit(1)

primary = 'laser_scan' if publish in ('laser_scan', 'both') else 'point_cloud'
put(helper, 'inputs:type', primary)
put(helper, 'inputs:topicName', 'scan' if primary == 'laser_scan' else 'lidar/point_cloud')
put(helper, 'inputs:frameId', frame_id, Sdf.ValueTypeNames.String)
# 兩種 type 都要開。關掉的話 laser_scan 每則只送當下 render 批次的那 60 度,
# 其餘 bin 填 -1 —— 見檔頭第 3 點, 這是量出來的, 不要憑文件字面再關回去。
put(helper, 'inputs:fullScan', True)
put(helper, 'inputs:resetSimulationTimeOnStop', True)

if publish == 'both':
    second = GRAPH + '/ros2_rtx_lidar_helper_pc'
    sp = stage.GetPrimAtPath(second)
    if not sp.IsValid():
        print(f'  另外建一個 point_cloud 發布節點: {second}')
        if not dry:
            sp = stage.DefinePrim(second, 'OmniGraphNode')
            sp.CreateAttribute('node:type', Sdf.ValueTypeNames.Token, False).Set(
                'isaacsim.ros2.bridge.ROS2RtxLidarHelper')
            sp.CreateAttribute('node:typeVersion', Sdf.ValueTypeNames.Int, False).Set(1)
            for name, tn in (('inputs:execIn', Sdf.ValueTypeNames.Token),
                             ('inputs:context', Sdf.ValueTypeNames.UInt64),
                             ('inputs:renderProductPath', Sdf.ValueTypeNames.Token)):
                sp.CreateAttribute(name, tn, False)
            # 跟主要那個 helper 接同樣的來源
            for name in ('inputs:execIn', 'inputs:context', 'inputs:renderProductPath'):
                src = helper.GetAttribute(name).GetConnections()
                if src:
                    sp.GetAttribute(name).SetConnections(src)
            sp.CreateAttribute('inputs:type', Sdf.ValueTypeNames.Token, False).Set('point_cloud')
            sp.CreateAttribute('inputs:topicName', Sdf.ValueTypeNames.String, False).Set('lidar/point_cloud')
            sp.CreateAttribute('inputs:frameId', Sdf.ValueTypeNames.String, False).Set(frame_id)
            sp.CreateAttribute('inputs:fullScan', Sdf.ValueTypeNames.Bool, False).Set(True)
            sp.CreateAttribute('inputs:resetSimulationTimeOnStop', Sdf.ValueTypeNames.Bool, False).Set(True)
        changes.append(('ros2_rtx_lidar_helper_pc', '(新節點)', None, 'point_cloud -> /lidar/point_cloud'))

# ---------------------------------------------------------------- 報告
pts = int(prof['reportRateBaseHz']) / float(prof['scanRateBaseHz'])
print()
print(f'每圈 {pts:.0f} 點, 角解析度 {360.0 / pts:.3f} 度, '
      f'{float(prof["scanRateBaseHz"]):.0f} Hz, 量程 '
      f'{prof["nearRangeM"]}~{prof["farRangeM"]} m, 單線 (仰角 0)')
print()
if not changes:
    print('沒有需要改的東西 —— 已經是設定好的狀態。')
else:
    print(f'{"(dry-run) " if dry else ""}共 {len(changes)} 項:')
    for who, name, old, new in changes:
        o = repr(old)
        if len(o) > 60:
            o = o[:57] + '...'
        n = repr(new)
        if len(n) > 60:
            n = n[:57] + '...'
        print(f'  [{who}] {name}: {o} -> {n}')
    if not dry:
        stage.GetRootLayer().Save()
        print('\n已存檔。回 Isaac 重新載入場景 (File -> Open) 再按 Play。')
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--usd', default=os.path.join(REPO, 'car.usd'))
    ap.add_argument('--profile', default=PROFILE)
    ap.add_argument('--publish', default='laser_scan',
                    choices=['laser_scan', 'point_cloud', 'both'],
                    help='laser_scan (預設, 跟實體 MS200 一致) / point_cloud / both')
    ap.add_argument('--frame-id', default='laser_frame')
    ap.add_argument('--lidar-z', default='keep',
                    help='感測器的目標世界高度 (m); keep = 不動')
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

    worker = '/tmp/_setup_oradar_worker.py'
    with open(worker, 'w') as f:
        f.write(WORKER)

    print(f'目標檔案: {args.usd}')
    print(f'profile : {args.profile}')
    r = subprocess.run(
        [os.path.join(ISAAC, 'python.sh'), worker, args.usd, args.profile,
         '1' if args.dry_run else '0', LIDAR_PRIM, GRAPH_PRIM,
         args.publish, args.frame_id, args.lidar_z],
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
