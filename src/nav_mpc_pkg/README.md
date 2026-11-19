# nav_mpc_pkg — 點目標導航: Nav2 規劃 + 感測器 MPC 循跡 + 遇到障礙物停車重新規劃

`sensor_mpc_pkg` 沿一條**事先畫好**的路徑 (CSV) 走。這個 package 把路徑換成「使用者給目標點, Nav2 算一條
不會撞到東西的路線」, MPC (完全沿用 `sensor_mpc_pkg`) 沿這條路線走; 途中前方出現地圖上沒有的障礙物就停車,
等 Nav2 把它畫進 costmap、算出繞過去的新路線, 再沿新路線走到目標。

```
 Foxglove 3D 面板                    Nav2 (只開規劃, 控制器是 MPC)
 "Publish pose" ──/goal_pose──┐      ┌─ nav_map_server ── /nav_mpc/map (= LiDAR 定位的同一張地圖)
 "Publish pose estimate"      │      ├─ planner_server: 全域 costmap (地圖 + /scan 障礙物 + 膨脹)
   └─/initialpose─┐           │      │                  + Smac Hybrid-A* (只往前, R >= 0.5 m)
                  │           v      └─ lifecycle_manager
   lidar_localizer│     nav_mpc_manager ──ComputePathToPose──> planner_server
   (從給的位姿配準) │      ├─ 路線重取樣 0.1 m ──/nav_mpc/path──> path_tracker ──> sensor_mpc_node ──> /joint_command
                  │      ├─ /scan 落在「前方路線走廊」裡 -> 停車, 重新規劃          ^
                  └──────├─ /nav_mpc/hold (true = 停, 20 Hz) ────────────────────────┘
                         └─ /nav_mpc/state, /nav_mpc/markers (走廊 + 擋路的點)
 /fusion_loc/odom (融合定位, 跟 sensor_mpc 一樣) ─> 全部
```

## 狀態機 (`/nav_mpc/state`)

| 狀態 | 車子 | 什麼時候離開 |
| --- | --- | --- |
| `IDLE` | 停 | 收到目標 |
| `PLANNING` | 停 | 有路線 -> `DRIVING`; 沒路線 -> `WAIT_REPLAN` |
| `DRIVING` | MPC 循跡 | 走到 -> `ALIGNING` (車頭差 > 5°) 或 `ARRIVED`; 前方走廊有障礙物、或 costmap 說剩下的路線太靠近障礙物 -> `WAIT_REPLAN` |
| `WAIT_REPLAN` | 停 | 車停穩 + `replan_wait` (1 s, 讓 costmap 畫上去) -> `PLANNING`; 規劃失敗則每 `replan_period` (2 s) 再試 |
| `ALIGNING` | 原地旋轉 | 車頭差 < `align_tolerance_deg` (5°) 連續 0.5 s -> `ARRIVED`; 超過 20 s、或 0.30 m 內有東西 -> 放棄, 也是 `ARRIVED` |
| `ARRIVED` | 停 | 新目標 |

**終點對準**: path_tracker 在離終點 0.15 m 就算到了, Hybrid-A* 的路線又常在最後一小段才轉向目標方向,
所以到的時候車頭可能差很多 (2026-11-18 Isaac: +53°)。`ALIGNING` 用融合的 yaw 做 P 控制: yaw rate 目標
`clip(1.5 x 誤差, ±0.6 rad/s)` (至少 0.2 rad/s, 克服靜摩擦), 乘 `align_wheel_per_yaw` (5.7 = T_eff / 2r)
換成輪速發到 `/nav_mpc/spin`; `sensor_mpc_node` 在 hold 中送 左 -u / 右 +u (一樣量化成整數 rpm, 定位 / 圍籬
檢查照舊)。**目標點在車子 0.2 m 內就不規劃, 直接原地轉** —— 在 Foxglove 車子上點一個方向 = 只轉車頭。
`align_heading:=false` 關掉。

任何時候給新目標都會直接重新規劃。`ros2 service call /nav_mpc_manager/cancel std_srvs/srv/Trigger` 停車放棄目標;
`.../replan` 用目前的目標重新規劃。

**障礙物怎麼判斷**: 每一幀 `/scan` 轉到 map 座標 (位姿 = 融合, 雷射外參 = TF), 丟掉車身框 (`footprint_box`)
裡的點, 看「從車子在路線上的投影點沿路線往前 `check_distance` (1.5 m)、兩側各 `corridor_half_width` (0.25 m)」
的走廊裡有沒有 >= `min_points` (3) 個點, 連續 `confirm_scans` (2) 幀 (10 Hz 雷射 = 0.2 s) 就停。走廊是沿著路線彎的,
所以轉彎時正前方 (但不在路線上) 的牆不會讓車停。地圖上的牆不用另外排除: 規劃時路線已經離牆 >= `robot_radius`
(0.45 m), 不會落在 0.25 m 的走廊裡。

**路線驗證 (costmap)**: 雷射一次只看得到障礙物朝向車子的那一面, 繞過去的新路線會貼著還沒看到的背面走。
假世界實測 (2026-11-18, 只有走廊偵測): 車子繞到箱子側面才發現, 停在離箱子 7 cm 的地方 —— 已經在膨脹區裡,
Hybrid-A* 拒絕從那裡起步 ("Starting point in lethal space"), 車子就卡死了。所以 DRIVING 時每收到一張
`/global_costmap/costmap` (2 Hz) 就檢查車子前方 `validate_distance` (4 m) 的路線離障礙物格子有沒有
< `validate_clearance` (0.35 m); 有就跟走廊偵測一樣停車重新規劃。門檻刻意比規劃用的 `robot_radius` (0.45)
鬆 0.1 m: Hybrid-A* 的路線會貼著膨脹區的邊走, 用同一個門檻的話雷射每多看到一格, 剛規劃好的路線就又不合格
(實測: 同一個場景連續重新規劃 5 次)。

**為什麼不用 Nav2 的 controller_server / collision_monitor**: 控制器是我們的 MPC (直接出四輪輪速), Nav2 的
collision_monitor 只會攔 `cmd_vel` (Twist), 接不上; 而且「停下來 -> 等 costmap -> 重新規劃 -> 換路線」這個流程
要跟 path_tracker / MPC 的狀態配合, 自己寫一個小節點比套 behavior tree 簡單。

## 用法 (模擬)

**第一次要裝 Nav2 規劃器**: 推上去的 `sim2real2sim:v3` 只有 `nav2_map_server` / `nav2_msgs`。根目錄的
`Dockerfile` 已經加了 `ros-humble-nav2-{planner,smac-planner,navfn-planner,costmap-2d,lifecycle-manager}`,
重 build (`docker build -t sim2real2sim:combined .`) 或在正在跑的容器裡先手動裝:

```bash
apt-get update && apt-get install -y ros-humble-nav2-planner ros-humble-nav2-smac-planner \
    ros-humble-nav2-navfn-planner ros-humble-nav2-costmap-2d ros-humble-nav2-lifecycle-manager
```

Isaac 開 **`kros_car_mpc_sensor.usd`** (有房間才有 `/scan`, 規劃跟障礙物偵測都要它), 按 Play, 然後:

```bash
r                                                  # 第一次要 build
ros2 launch nav_mpc_pkg nav_mpc.launch.py          # 融合定位 + Nav2 + path_tracker + MPC + 管理節點
ros2 launch car_viz viz.launch.py                  # 另一個 terminal: Foxglove bridge
```

Foxglove: Open connection -> 印出來的網址, Layout -> Import `src/nav_mpc_pkg/config/foxglove_nav_layout.json`
(3D 面板的 publish 工具已經設好 topic)。

1. **初始位姿** (通常不用給): LiDAR 定位自己會在整張地圖上做全域定位, 融合也是用第一則絕對量測當起點。
   位置不對 (例如長方形房間鎖到 180 度反方向) 才用 3D 面板左上角的 **Publish pose estimate**
   (按住拖出方向): `lidar_localizer` 從那裡開始局部配準, 管理節點 0.5 s 後看融合位姿跟它差太多
   (> 0.3 m 或 15°) 就叫 `/fusion_localizer/reset` (車子要停著 ~1 s, 融合要做靜止校正)。
   給初始位姿時正在走的話會先停車, 要重新給目標。
2. **目標**: **Publish pose** (按住拖出車頭方向)。Display frame 要是 `map`。Foxglove 預設發到 `/move_base_simple/goal`,
   要在 3D 面板設定 -> Publish 把 Pose topic 改成 `/goal_pose`、Pose estimate topic 改成 `/initialpose` (匯入 layout 也可以)。
3. 看 `/nav_mpc/state`, 3D 面板裡藍線 = Nav2 路線、橘線 = MPC 預測、綠 / 紅色寬帶 = 檢查中的走廊 (紅 = 有東西)。

不想用 Foxglove 時 (或要重複同一個實驗) 直接發:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 3.5, y: 1.0}, orientation: {z: 0.0, w: 1.0}}}"
```

**放障礙物測試** (Isaac Script Editor, Play 中執行, 不要存檔):

```python
exec(open('/home/liang/sim2real2sim/src/nav_mpc_pkg/scripts/isaac_obstacle.py').read())
add_obstacle_ahead(1.6)       # 車子正前方 1.6 m 放 0.4 m 方塊
clear_obstacles()
```

常用參數 (都可以在 launch 那行給):

```bash
# 跟實體車一樣的定位組合 (沒有環境相機)
ros2 launch nav_mpc_pkg nav_mpc.launch.py multicam:=false
# 定位已經在別的 terminal 跑
ros2 launch nav_mpc_pkg nav_mpc.launch.py localization:=false
# Hybrid-A* 找不到 (例如停得太近, 往前繞不過去) 時改用 2D A* (不管車頭方向, 路線開頭可能要原地大轉)
ros2 launch nav_mpc_pkg nav_mpc.launch.py fallback_planner_id:=Grid
# 偵測距離 / 走廊寬度 / 最小轉彎半徑 / 規劃用的車子半徑
ros2 launch nav_mpc_pkg nav_mpc.launch.py check_distance:=2.0 corridor_half_width:=0.3 \
    minimum_turning_radius:=0.4 robot_radius:=0.4
```

`sensor_mpc.launch.py` 的參數 (`multicam`、`lidar`、`imu_topic`、`wheel_feedback_topic`、`gt_topic`、`geofence_*`、
`disturbance` ...) 直接加在這行就會傳下去。輸出:

* `output_csv` (預設 `car_run_data/nav_mpc_tracking.csv`): sensor_mpc_node 的逐週期 CSV, 跟 sensor_mpc 一樣;
  停車的週期 `reason` 欄是「上層要求停車 (/nav_mpc/hold)」。每次換路線 cte 都改量到新路線
* `events_csv` (預設 `car_run_data/nav_mpc_events.csv`): 每個事件一列 —— `goal`、`path` (路線長、規劃毫秒)、
  `obstacle` (沿路線多遠、幾個點)、`path_invalid` (costmap 驗證, 沿路線多遠)、`arrived` / `aligned` / `align_abort`
  (detail 有車頭差 `heading_err_deg`)、`plan_failed`、`arrived`、`initialpose`、`fusion_reset`, 含當時位姿

**每一輪給不同的 `output_csv:=` / `events_csv:=`**, 不然會互相覆寫。

## 參數的取捨

| 參數 | 值 | 為什麼 |
| --- | --- | --- |
| `robot_radius` (nav2_params) | 0.45 m | 車子半對角 ~0.26 m; 但 sensor_mpc_node 的**圍籬**是牆往內 0.4 m、圓柱外擴 0.4 m, 路線比這個靠近牆 MPC 就會鎖住 |
| `minimum_turning_radius` | 0.5 m | MPC 離線在 R0.4 還跟得上 (sensor_mpc_pkg README); 留餘量 |
| 規劃器 | Smac Hybrid-A*, Dubins (只往前) | 會考慮起點車頭方向和轉彎半徑, 路線 MPC 跟得上。path_tracker / MPC 不會倒車, 所以不用 Reeds-Shepp |
| `check_distance` | 1.5 m | 停下來之後要有空間往前繞: 偵測到時離障礙物 ~1.3 m, 障礙物膨脹出來的禁區 (0.45 m) 前面還剩 ~0.7 m 可以轉。太短 -> 停太近, Hybrid-A* 找不到路 (只會一直 `WAIT_REPLAN`) |
| `corridor_half_width` | 0.25 m | 車寬一半 ~0.17 + 8 cm。太寬 -> 路線旁邊的東西也會讓車停 |
| `validate_clearance` | 0.35 m | 比 robot_radius 鬆 0.1 m (遲滯, 見上面「路線驗證」); 仍大於車子半對角 0.26 m |
| `replan_wait` | 1 s | costmap 5 Hz 更新, 雷射 10 Hz; 太短 -> 新路線還穿過障礙物 (管理節點會檢查新路線前方的走廊, 擋住就算失敗再等) |

## 已知限制

* **終點對準是原地旋轉, 位置會偏一點**: 滑移轉向原地轉的中心不一定是 base_link, 假世界 180° 轉完中心偏 ~11 cm
  (只對準車頭, 不再修位置)。Isaac / 實體車還沒量。
* **只會往前**: path_tracker / MPC 不倒車, 規劃器用 Dubins。停得離障礙物太近 (< robot_radius) 就規劃不出來,
  log 會寫「車子自己在障礙物的膨脹區裡」, 要把障礙物或車子移開 (或 `fallback_planner_id:=Grid` 試 2D A*)。
* **箱子會干擾 LiDAR 定位** (2026-11-18 Isaac 第一次放箱子: 融合一直擋 LiDAR -> MPC「絕對量測斷了」停車,
  LiDAR 還一度鎖到 180 度反方向)。已在 car_loc_lidar 修 (地圖上沒有的點不拿來配準 + 穿牆射線鎖死偵測,
  見 car_loc_lidar README 第 10 節); Isaac 上還沒重測。
* **低於雷射平面的東西看不到**: 障礙物偵測和 costmap 都只用 2D `/scan` (laser_frame 離地 ~0.24 m)。
* Isaac 跟實體車都**還沒跑過**; 下面的數字是假世界 (受控體 = 動態模型) 的。

## 實體車

`kros-car-new` (Jazzy) 已經有完整 Nav2, launch 會依 `ROS_DISTRO` 自動換 plugin 名稱 (Humble 用
`nav2_smac_planner/SmacPlannerHybrid`, Jazzy 用 `::`)。MPC 需要的 acados / l4casadi / CPU torch 由
`real/Dockerfile` 裝 (2026-11-19 起, 改完要 `real/scripts/build_image.sh` 重 build), `real/scripts/sim2real2sim.sh`
預設會 build mlp_control_pkg / sensor_mpc_pkg / nav_mpc_pkg, 並把 `model/`、`car_run_data/` 掛到
`/workspaces/` 底下 (launch 的預設路徑)。2026-11-19 在這個 image 裡用假世界 (`test/fake_world.py`) 跑過整條流程:
Jazzy Nav2 規劃、遇箱子停車重新規劃、到達與對準都正常, MPC 每週期 9–10 ms (開發機, 不是車上電腦)。
Jazzy 的 planner_server 啟動時會印一行 Smac「Inflation layer ... not set sufficiently」的 ERROR, 規劃照常, 可忽略。
其他要改的: `map_path` (場地地圖, 規劃也用它)、`geofence_*`、實體車的定位參數 (見 sensor_mpc_pkg README 的實體車那一行),
`use_sim_time:=false`。雷射外參從 TF 拿 (base_link -> scan 的 frame), 查不到就用 `laser_offset`。

## 節點 `nav_mpc_manager`

| | topic | 型別 | |
| --- | --- | --- | --- |
| in | `/goal_pose` | `geometry_msgs/PoseStamped` | 目標 (frame 要是 map) |
| in | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初始位姿 (LiDAR 定位也訂閱) |
| in | `/fusion_loc/odom` | `nav_msgs/Odometry` | 位姿 |
| in | `/scan` | `sensor_msgs/LaserScan` | 障礙物 |
| in | `/sensor_mpc/finished` | `std_msgs/Bool` | path_tracker 說走到了 |
| out | `/nav_mpc/path` | `nav_msgs/Path` (latched) | 給 path_tracker (重取樣 0.1 m) |
| out | `/nav_mpc/hold` | `std_msgs/Bool` | true = 停, 20 Hz; 斷 0.5 s MPC 也停 (`hold_timeout`) |
| out | `/nav_mpc/spin` | `std_msgs/Float32` | `ALIGNING` 時的原地旋轉輪速 (rad/s, 正 = 逆時針) |
| out | `/nav_mpc/state` | `std_msgs/String` (latched) | 狀態 + 原因 |
| out | `/nav_mpc/markers` | `visualization_msgs/MarkerArray` | 走廊 + 擋路的點 |
| out | `/nav_mpc/goal` | `geometry_msgs/PoseStamped` (latched) | 目前的目標 |
| action | `/compute_path_to_pose` | `nav2_msgs/ComputePathToPose` | Nav2 planner_server |
| srv | `~/cancel`, `~/replan` | `std_srvs/Trigger` | |

參數在 `config/nav_mpc.yaml`; Nav2 的在 `config/nav2_params.yaml`。

為了這個 package 改了別人的地方 (預設行為都不變):

* `sensor_mpc_pkg`: `sensor_mpc_node` 多了 `hold_topic` / `hold_timeout` / `spin_topic` (空 = 不訂閱);
  `sensor_mpc.launch.py` 多了 `path_topic`、`hold_topic`、`hold_timeout`、`spin_topic` 四個參數
* `car_loc_lidar`: `lidar_localizer` 訂閱 `/initialpose` (`initialpose_topic`, 空 = 不訂閱), 收到就從那裡局部配準; 另外 `occluder_dist` / `lock_see_through`
  (**預設開**, 會改變所有用 lidar_localizer 的流程; 沒有箱子時結果一樣)

## 測試

```bash
cd src/nav_mpc_pkg && python3 test/test_geometry.py      # host 就能跑: 重取樣、沿路線進度、走廊判斷、雷射轉換
```

`test/fake_world.py` 是不開 Isaac 的假世界 (車子 = 動態模型, 假 LiDAR 對 room.pgm 做射線, 車子離起點 0.6 m 時
在路線前方 1.8 m 放一個 0.4 m 方塊), 跟 `nav_mpc.launch.py use_sim_time:=false localization:=false` 一起跑,
整條流程 (Nav2 規劃 -> MPC -> 偵測 -> 停車 -> 重新規劃 -> 到達) 都是真的節點:

假世界結果 (2026-11-18, `sim2real2sim:v3` + Nav2, v_max 0.45, 一個 0.4 m 方塊在車子前方 1.88 m 出現):

| 場景 | 經過 | 車身離方塊最近 | 循跡 RMS / 最大 |
| --- | --- | --- | --- |
| (0,0) -> (3.5, 1.0, 0°) | 走廊偵測 1.50 m 停 (煞停 7 cm) -> 1.5 s 後新路線 3.84 m -> 到達 (3.51, 0.98) | 0.31 m | 0.81 / 1.64 cm |
| (0,0) -> (-3, -1.5, 180°), 要迴轉 | 走廊偵測停 -> 新路線 -> 看到箱子另一面, costmap 驗證停 1 次 -> 到達 (-3.00, -1.49) | 0.31 m | 0.73 / 1.58 cm |
| 同上, 沒有方塊 | 一次規劃直接到 (-3.03, -1.47), yaw -179°; 牆沒有造成任何停車 | - | 0.61 / 1.82 cm |
| 終點對準 (加在上面三個場景) | 到達時車頭差 +15~21° -> 1.4 s 轉到 +3.3~3.9° | - | - |
| 目標就在車子上, 只轉 180° | 不規劃, 原地轉 5.5 s, 剩 -3.3° (中心偏 11 cm) | - | - |
| 開到一半把管理節點 kill -9 | MPC 0.5 s 後停車 (`/nav_mpc/hold 超過 0.5 s 沒訊息`) | - | - |

規劃 (Smac Hybrid) 每次 2~4 ms; MPC 每週期中位數 ~13 ms。Nav2 參數在 Jazzy (`kros-car-new`) 也載得起來,
兩個規劃器都規劃得出路線 (只測了 planner_server, 沒有整套)。

```bash
python3 src/nav_mpc_pkg/test/fake_world.py --ros-args -p map_yaml:=$(ros2 pkg prefix car_loc_lidar)/share/car_loc_lidar/maps/room.yaml &
ros2 launch nav_mpc_pkg nav_mpc.launch.py use_sim_time:=false localization:=false gt_topic:=/none \
    output_csv:=/tmp/nav_track.csv events_csv:=/tmp/nav_events.csv
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 3.5, y: 1.0}, orientation: {w: 1.0}}}"
```
