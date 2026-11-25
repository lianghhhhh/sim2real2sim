# sensor_mpc_pkg — 感測器定位 + MPC 循跡 (給實體車, 模擬也用同一套)

`mlp_control_pkg` 的 MPC 吃的是 Isaac 的 ground truth (`/odom`) 與 Isaac 裡 ScriptNode 算的目標點
(`/nearest_curve_point`), 實體車兩個都沒有。這個 package 把那兩個輸入換成 ROS 端的東西,
**MPC 本身 (模型、acados solver、參考點的建法) 直接 import `mlp_control_pkg` 的, 一行都沒抄**:

```
/imu /joint_states(/wheel_feedback) /env_cam_i/car_bbox /lidar_loc/odom
        │
        v
  car_loc_fusion (方法五) ──> /fusion_loc/odom, /fusion_loc/abs_age
        │                           │
        │                           v
        │                  path_tracker ──> /sensor_mpc/nearest_points (10 個目標點 + 絕對航向)
        │                 (路徑 CSV)        /sensor_mpc/path, /sensor_mpc/targets (給 Foxglove)
        v                           │
  sensor_mpc_node  <────────────────┘
  ├─ sensor_state_node: odom -> 6 維狀態 (舊版 + 輪速回授 -> 10 維); 判斷定位能不能信
  └─ sensor_mpc_node:   acados MPC (mlp_control_pkg.utils) -> /joint_command (四輪 rad/s)
                                                          -> /sensor_mpc/predicted_path_viz
```

| 原本 (mlp_control_pkg) | 這裡 | 差在哪 |
| --- | --- | --- |
| `/odom` (Isaac GT) 的 pose / twist | `/fusion_loc/odom` 的 pose / twist | 融合不估側向速度 vy, 預設給 0 (`vy_source:=diff` 用位姿差分) |
| `/joint_states` | `joint_states_topic` (實體車 `/wheel_feedback`) | 用關節名字對, 順序無所謂 |
| Isaac ScriptNode `temp_curve.py` 的 `/nearest_curve_point` | `path_tracker` 節點 + 路徑 CSV | 同一套演算法移植成 numpy; 航向改成絕對值; 最近點改投影到最近的**線段** (原版在 0.5 m 折線的轉角會把目標點算到路徑外) |
| 不檢查定位 | 位姿 / 輪速斷、sigma 太大、絕對量測斷太久 -> 送 0 停車 | 規則跟 `calibrate_env_pkg` 的 `control_car_node` 一樣 |
| 循跡誤差只對 GT | 對感測器位姿 + (模擬時) 對 GT, 逐週期存 CSV | 拿來量「定位誤差吃掉多少循跡精度」 |

## 用法

### 模擬 (Isaac 開 `kros_car_mpc_sensor.usd`, 按 Play)

**不用**在 Stage 裡選車和路線 —— 路線由 CSV 給, Isaac 的 ScriptNode 有沒有在跑都無所謂
(開檔時那個 ScriptNode Warning 還是要按 Yes, 不然整個場景的 OmniGraph 都不會跑, 一個 topic 都沒有)。

```bash
r                                                 # 第一次要 build
ros2 launch sensor_mpc_pkg sensor_mpc.launch.py   # 定位 (方法五) + actuator_model + path_tracker + MPC
```

launch 會開: `collect_all.launch.py` 的 fusion 那一組 (多相機偵測 + LiDAR + 融合; `maps/room.yaml`
跟這個房間同尺寸)、`actuator_model` (kros_car*.usd 吃 `/joint_command_isaac`, 不開車子不動)、
`path_tracker` (預設路線 `config/path_kros_car_mpc_sensor.csv`, 下面「路徑 CSV」)、`sensor_mpc_node`。
用沒有房間的 `kros_car_mpc.usd` 時要 `lidar:=false path_file:=.../path_kros_car_mpc.csv` (沒有牆就沒有 `/scan`)。
偵測節點第一次要暖機 ~30 s, 期間 MPC 會印「停車: 還沒收到位姿」, 正常。

開始前讓車停一秒 (融合要做靜止校正); 車子要在路線附近 (離路徑 > 1.5 m 會走回歸曲線接回去)。
Ctrl-C 會先送輪速 0, 印誤差統計, 逐週期資料存到 `output_csv` (沒給 = `<工作區>/car_run_data/sensor_mpc_{time}.csv`, `{time}` = 啟動時的台灣時間, 例 `sensor_mpc_20261119_153012.csv`; 給的檔名已經存在就自動在後面加時間, 不覆寫; 啟動時會印出這一輪的檔名)。

常用開關:

```bash
# 定位已經在別的 terminal 跑 (collect_all / fusion_loc.launch.py), 這裡只開 tracker + MPC
ros2 launch sensor_mpc_pkg sensor_mpc.launch.py localization:=false

# A/B: 位姿改吃 Isaac 的 GT, 其餘 (path_tracker、MPC) 完全一樣 —— 跟 mlp_control_node 的差別只剩目標點的來源
ros2 launch sensor_mpc_pkg sensor_mpc.launch.py localization:=false odom_topic:=/odom

# 不要 LiDAR (只有多相機 + 遞推)
ros2 launch sensor_mpc_pkg sensor_mpc.launch.py lidar:=false

# 換一條路線 / 不封閉的路線 (走到終點停車)
ros2 launch sensor_mpc_pkg sensor_mpc.launch.py path_file:=/workspaces/src/sensor_mpc_pkg/config/xxx.csv loop:=false
```

MPC 的參數 (N、dt、v_max、權重、模型檔) 還是 `model/mpc_config.json` (`MPC_CONFIG_PATH`), 跟
`mlp_control_pkg` 共用 —— **模擬跟實體車要用同一份**, 比較才公平。節點本身的參數在
`config/sensor_mpc.yaml`。

### 實體車

topic、IMU 模式、地圖、路徑檔都不一樣 (定位的部分跟 `friction_test.launch.py` 實體車那一行同一組參數,
完整步驟見 `real/REAL_CAR_TELEOP_LIDAR_GUIDE.md`):

```bash
ros2 launch sensor_mpc_pkg sensor_mpc.launch.py use_sim_time:=false multicam:=false lidar:=true \
    wheel_feedback_topic:=/wheel_feedback imu_topic:=/imu/data \
    gravity_mode:=complementary yaw_source:=gyro fusion_sigma_acc:=0.35 \
    map_path:=/s2r/src/car_loc_lidar/maps/lab.yaml \
    path_file:=/s2r/src/sensor_mpc_pkg/config/path_lab.csv gt_topic:='' \
    output_csv:=/s2r/car_run_data/real_mpc.csv
```

* `use_sim_time:=false` 時不會開 `actuator_model` (那是模擬輪子用的); `/joint_command` 直接給
  `wheel_velocity_controller` (記得先切換控制器, 見 guide 第 4 節)。輪速一律量化成整數 rpm。
* 實體車沒有環境相機 (`multicam:=false`), 融合 = LiDAR + IMU + 輪速, 所以 **LiDAR 要有那個場地的地圖**
  (`map_path`), 而且路徑 CSV 的座標要在**同一個 map frame** 裡 (在 RViz / Foxglove 看
  `/sensor_mpc/path` 有沒有疊在地圖上)。
* 實體車的 `/imu/data` 是 20 Hz, 融合的輸出也是 20 Hz, 跟 MPC 的 dt 0.05 s 一樣 —— 位姿每個週期
  只有一則新的。`pose_timeout` 0.3 s 夠用; 如果 IMU 更慢要放寬。
* MPC 一個週期要在 50 ms 內解完 (log 會印 `MPC cycle over budget`)。車上電腦不夠快的話降 `N_steps`。
* `/fusion_loc/abs_age` 超過 `abs_age_max` (1 s, LiDAR 完全沒進來) 就停車; 不想要這個保護給 `abs_age_max:=0`。

### 路徑 CSV 怎麼來

**`config/path_kros_car_mpc_sensor.csv` (預設)** 是給 `kros_car_mpc_sensor.usd` 的房間畫的
(`scripts/make_path_kros_car_mpc_sensor.py` 產生, 預覽圖 `config/path_kros_car_mpc_sensor.png`):
逆時針一圈 20.9 m, 從車子出生點 (0, 0) 朝 +x 出發, 直線 + 半徑 1.2 m 的圓弧, 104 點 (0.2 m 間距),
離牆最近 0.99 m、離 (2.1, −2.04) 的圓柱表面 1.54 m。舊的 `kros_car_mpc.usd` 路線搬進這個房間會穿過南牆
(最低到 y = −4.81), 不能用。想在 Isaac 裡看到這條線, Script Editor 跑 `scripts/isaac_set_curve.py`
(檔頭有用法, 它會把 CSV 寫進 `/World/BasisCurves/BasisCurves`; USD 要不要存自己決定 —— path_tracker
只讀 CSV, 不讀 USD)。

**`config/paths/` 多條測試路線** (2026-11-18, `scripts/make_paths.py` 產生, 總覽圖 `config/paths/overview.png`)。
全部給 `kros_car_mpc_sensor.usd` 的房間, 都從出生點 (0, 0) 朝 +x 出發, 直線 + 圓弧接成; 離牆 >= 0.95 m、
離圓柱中心 >= 1.3 m (圍籬圓 0.9 m)。要加新路線就在 `make_paths.py` 的 `PATHS` 加一個函式 (Turtle: `S` 直線、
`L` / `R` 左右轉), 重跑會自己檢查接不接得起來、會不會碰牆。

| 檔名 | 難度 | 內容 | 長度 | 最小半徑 | 封閉 |
| --- | --- | --- | --- | --- | --- |
| `e1_oval_ccw_r12` | 簡單 | 逆時針跑道, 只有左轉 | 14.7 m | 1.2 | 是 |
| `e2_oval_cw_r10` | 簡單 | 順時針跑道, 只有右轉 | 12.1 m | 1.0 | 是 |
| `e3_circle_ccw_r12` | 簡單 | 逆時針定圓 (穩態左轉) | 7.5 m | 1.2 | 是 |
| `e4_circle_cw_r10` | 簡單 | 順時針定圓 (穩態右轉) | 6.3 m | 1.0 | 是 |
| `e5_uturn_open` | 簡單 | 直線 + 180° 迴轉 + 直線 | 9.1 m | 1.0 | 否 |
| `m1_figure8_r10` | 中等 | 8 字, 原點左右轉直接切換 | 12.6 m | 1.0 | 是 |
| `m2_slalom_r07` | 中等 | 跑道 + 3 個 S 彎 | 15.0 m | 0.7 | 是 |
| `m3_lshape_r08` | 中等 | L 形, 含一個右轉凹角 | 20.3 m | 0.8 | 是 |
| `h1_serpentine_r04` | 困難 | 蛇行 4 車道 (間距 0.8 m), 髮夾彎 | 26.5 m | 0.4 | 是 |
| `h2_lshape_r04` | 困難 | 跟 m3 同形狀, 圓角 R0.4 (接近直角) | 21.4 m | 0.4 | 是 |
| `h3_spiral_open` | 困難 | 向內螺旋, 半徑 1.2 -> 0.4 | 11.3 m | 0.4 | 否 |
| `h4_mixed_track` | 困難 | R1.4 大彎、R0.5 右髮夾、R0.5 S 彎 | 20.8 m | 0.5 | 是 |

```bash
ros2 launch sensor_mpc_pkg sensor_mpc.launch.py multicam:=false \
    path_file:=/workspaces/src/sensor_mpc_pkg/config/paths/m1_figure8_r10.csv loop:=true \
    output_csv:=/workspaces/car_run_data/mpc_m1_figure8.csv
# 不封閉的 (e5, h3) 要 loop:=false, 開到終點會自己停
```

在 Isaac 裡看路線: `scripts/isaac_set_curve.py` 的 `set_curve(CSV)`; 不封閉的給 `closed=False`。
車子每次都要放回 (0, 0) 朝 +x (Isaac Stop -> Play 就會回去)。

離線試開 (`test/drive_paths_offline.py`, 受控體 = 動態模型自己, v_max 0.45, 2026-11-18): 12 條全部開完,
沒碰圍籬、acados 沒失敗。到路徑的真正距離 RMS 0.09 到 0.29 cm、最大 1.08 cm (R0.4 的髮夾彎);
輪速頂到上限的時間 `h2` 15%、`h1` 6%, 其他 <= 2.4%。這只證明幾何、速度、輪速上限和視界沒問題,
模型在 R0.4 這麼小的半徑學得準不準要在 Isaac 上才看得出來。

`config/path_kros_car_mpc.csv` 是從 `kros_car_mpc.usd` 的 `/World/BasisCurves/BasisCurves` 匯出的
(39 個控制點, 間距 0.5 m, 封閉, 總長 19 m; 用的是**控制點連成的折線**, 跟 Isaac 的 ScriptNode 一樣)。
路線在 GUI 裡被搬過就要重匯 (🖥 host 上, 不需要 Isaac 在跑):

```bash
E=$(ls -d ~/isaac-sim/extscache/omni.usd.libs-*cp311 | head -1)
PYTHONPATH=$E LD_LIBRARY_PATH=$E/bin:$HOME/isaac-sim/kit/python/lib \
    ~/isaac-sim/kit/python/bin/python3 \
    src/sensor_mpc_pkg/scripts/export_path_from_usd.py kros_car_mpc.usd \
    -o src/sensor_mpc_pkg/config/path_kros_car_mpc.csv
```

實體場地的路線自己寫 CSV (`x,y` 每列一點, map 座標, m; `#` 開頭是註解) 或發 `nav_msgs/Path` 到
`path_topic`。點不用密 (0.3~0.5 m 就好, tracker 會沿線段內插), 但轉角越尖 MPC 越難跟。

## 節點與 topic

### `path_tracker`

| | topic | 型別 | 內容 |
| --- | --- | --- | --- |
| in | `odom_topic` (`/fusion_loc/odom`) | `nav_msgs/Odometry` | 車子位姿 |
| in | `path_topic` (預設不訂閱) | `nav_msgs/Path` | 整條換掉路徑並 reset |
| out | `/sensor_mpc/nearest_points` | `std_msgs/Float32MultiArray` | `[idx, x1, y1, psi1, ..., x10, y10, psi10]`, psi = 絕對路徑航向 (度)。排列跟 Isaac 的 `/nearest_curve_point` 一樣, 只差 psi 是絕對值 |
| out | `/sensor_mpc/path` | `nav_msgs/Path` (latched) | 整條路徑 |
| out | `/sensor_mpc/targets` | `nav_msgs/Path` | 目前的 10 個目標點 |
| out | `/sensor_mpc/finished` | `std_msgs/Bool` | 不封閉路徑走到終點 |
| srv | `/path_tracker/reset` | `std_srvs/Trigger` | 重新決定方向與最近點 (車子被搬走) |

參數 (`config/sensor_mpc.yaml`): `loop`、`direction` (auto / forward / backward, auto = 車頭跟哪個方向近)、
`spacing` (0.1 m, **要跟 `mpc_config.json` 的 `path_point_spacing` 一樣**)、`far_threshold` (1.5 m)、
`goal_tolerance`、`rate` (50 Hz)。

### `sensor_mpc_node` (一個 process 兩個節點)

| | topic | 型別 | 內容 |
| --- | --- | --- | --- |
| in | `odom_topic` | `nav_msgs/Odometry` | pose -> x, y, θ; twist -> vx, wz |
| in | `joint_states_topic` | `sensor_msgs/JointState` | 四輪輪速 (rad/s) |
| in | `abs_age_topic` (空 = `/fusion_loc/abs_age`) | `std_msgs/Float32` | 絕對量測多久沒進來 |
| in | `gt_topic` (模擬 `/odom`; 實體車 `''`) | `nav_msgs/Odometry` | 只拿來記誤差, 不控制 |
| in | `path_topic` | `Float32MultiArray` | 目標點; `path_heading_relative:=true` 可以直接吃 Isaac 的 `/nearest_curve_point` |
| in | `hold_topic` (預設不訂閱) | `std_msgs/Bool` | true = 停車 (`nav_mpc_pkg` 的 `/nav_mpc/hold`); `hold_timeout` > 0 時這條斷太久也停 |
| in | `spin_topic` (預設不訂閱) | `std_msgs/Float32` | hold 中收到新鮮的 (< 0.3 s) 輪速 u 就送 左 -u / 右 +u (原地旋轉, 正 = 逆時針; nav_mpc 終點對準用) |
| out | `/joint_command` | `sensor_msgs/JointState` | 四輪輪速指令 |
| out | `/sensor_mpc/predicted_path_viz` | `nav_msgs/Path` | MPC 預測的整條 horizon |
| out | `/sensor_mpc/predicted_path` | `Float64MultiArray` | 前 3 個預測位姿 (跟舊節點一樣) |

**什麼時候會停車 (送 0)**: 還沒收到位姿、位姿超過 `pose_timeout` (0.3 s) 沒更新、
(~~輪速超過 `wheel_timeout`~~ 模型不含輪速後已註解掉)、位姿 1-sigma > `pose_sigma_max` (0.3 m)、`abs_age` > `abs_age_max` (1 s)、
目標點超過 `path_timeout` 沒更新、路徑走完、**出圍籬**、`hold_topic` 說停 (或斷了超過 `hold_timeout`)。恢復時停超過一個 window (10 x 0.05 s) 就把 MPC 的歷史
重新用目前狀態填, 不到的話補「停著沒動」。

**圍籬 (geofence)**: 車子 (感測器位姿) 出了 `geofence_x/y_min/max` 的矩形、進了 `geofence_circles`
的圓 (柱子, `[x, y, r, ...]`)、或 MPC 預測的前 `geofence_horizon` (0.5 s) 會出去, 就送 0 並**鎖住**,
之後要 `ros2 service call /sensor_mpc_node/release_geofence std_srvs/srv/Trigger` 才會再動。預設是
`kros_car_mpc_sensor.usd` 的房間往內縮 0.4 m、圓柱外擴 0.4 m; **實體場地一定要用 launch 參數改**
(`geofence_x_min:=... geofence_circles:="[0.0]"`)。沒設圍籬節點會印警告。

**位姿延遲補償** 有兩層, 先量再補:
* CSV 的 `pose_age` 欄 = MPC 拿到位姿時那則訊息已經幾秒前了 (定位鏈的處理延遲), 結束的統計也會印。
* **融合那邊** (`bbox_extra_delay` / `lidar_extra_delay`, 透過 launch 給): 量測時戳跟 `/clock` 不同基準時,
  融合的時鐘偏移鎖定會把真正的延遲一起吃掉 (見 `car_loc_fusion/sources.py`), 只能把量到的常數延遲
  加回去, 倒帶重放就會補對。2026-11-17 在 `kros_car_mpc_sensor.usd` 上 8 台相機的融合位置落後 GT
  約 0.25 s (6 cm), LiDAR 單獨那條沒有落後 (0.8 cm, 無延遲), 所以是相機時戳的問題:
  `bbox_extra_delay:=0.25` 試, 看 CSV 裡 sensor 對 GT 的前後誤差是不是歸零。
  **更正 (同一天, 用 0.24 與 0.43 m/s 兩輪分開): 相機那條的落後大部分是固定偏移, 不是延遲** ——
  直線上前後誤差 = −5.25 cm − 0.045 s × 車速。等速 0.25 m/s 時 6 cm 的固定偏移跟 0.25 s 延遲分不出來,
  所以 `bbox_extra_delay:=0.25` 在慢速剛好有效, 到 0.5 m/s 會反過來超前約 4.6 cm。比較可能的原因是
  `car_loc_multicam/config/car_model.yaml` 的車型跟這份 USD 的車 (手臂姿勢) 不一樣, 要重匯車型
  (`car_loc_multicam/scripts/export_from_usd.py`, Play 之後) 而不是補延遲。只影響模擬的環境相機。
* **MPC 這邊** (`pose_lead_mode:=stamp|fixed`, `pose_lead_extra`): 用 vx、wz 把位姿往前外插 lead 秒
  (stamp = 用 `pose_age` + extra; fixed = 只有 extra)。這是給實體車 LiDAR 節點處理延遲用的通用補法,
  預設關。

### 輸出 CSV

每個 MPC 週期一列: `t, x, y, yaw, vx, vy, wz` (餵給 MPC 的狀態)、`cte, ate, head_err` (對感測器位姿)、
`gt_x, gt_y, gt_yaw, gt_cte, gt_ate, gt_head_err` (對 GT; 實體車空白)、`path_s, path_kappa` (投影點的弧長與
路徑曲率, 左轉正; 分直線 / 左轉 / 右轉用)、`err_src`。
`cte` / `head_err` 是**到整條路徑折線的距離** (左正) 與跟路徑切線的角度差 (`sensor_mpc_pkg/path_error.py`,
訂閱 path_tracker 的 `/sensor_mpc/path`, `err_src = path`)。2026-11-18 以前是對參考軌跡第 0 列算的: 那一點
是從前方 0.1、0.2 m 兩個目標點往後延伸的, 在半徑 R 的彎道落在圓弧外側約 0.1 x 0.2 / (2R) (R1.2 時 0.83 cm),
所以舊 CSV 的 cte 左轉偏正、右轉偏負 (`err_src` 空白或 `ref0`)。新舊 CSV 都可以用
`scripts/eval_tracking.py` 重算 (`--path` 給跑的那條路線、`--markdown` 印表格)。`ate` 還是對第 0 個參考點。
`v_ref, cmd_l, cmd_r, solve_ms, ok` (0 = 這一週期在停車或 solver 失敗, `reason` 寫原因)、
`pose_age` (位姿訊息幾秒前的)、`lead` (實際外插了幾秒)。`t` 是 NaN 的列是 Isaac Stop / Play 的分段標記。
不會互相覆寫: 沒給 `output_csv:=` 用 `sensor_mpc_{時間}.csv`; 給了但同名檔已存在, 自動存成 `<原檔名>_{時間}.csv`。檔名裡也可以直接寫 `{time}`, 例 `output_csv:=/workspaces/car_run_data/mpc_h4_{time}.csv`。

## 離線測試 (不需要 ROS / Isaac)

```bash
cd src/sensor_mpc_pkg && python3 test/test_path_tracker.py     # 秒級, host 就能跑
python3 test/test_path_error.py                                 # 橫向誤差的定義 (path_error.py)
# 容器裡 (要 torch / acados), 約 2 分鐘:
cd /tmp && python3 /workspaces/src/sensor_mpc_pkg/test/test_offset_free.py
```

`test_path_tracker`: 理想車沿路徑走兩圈 (偏離 0 m)、方向判斷、回歸曲線、不封閉路徑的終點。
`test_offset_free`: 受控體 = 模型 + 固定擾動, 比較擾動估計開 / 關。改了 `mpc_config.json` 的權重先跑它。
`drive_paths_offline` (容器裡): `config/paths/` 每條路線離線開一圈, 看開不開得完。

## Isaac 實測 (2026-11-17, kros_car_mpc_sensor.usd, 預設路線)

完整的 17 輪參數與結果在 repo 根目錄的 `report_sensor_mpc_2026-11-17.md`。下面的橫向誤差都是到路徑的
真正距離 (2026-11-18 用 `scripts/eval_tracking.py` 重算; 舊算法在彎道有 0.83 cm 的偏差, 見「輸出 CSV」)。

| 位姿來源 | 輪胎摩擦 | head | v_max | 橫向 RMS | 最大 | 定位誤差 |
| --- | --- | --- | --- | --- | --- | --- |
| 8 相機 + LiDAR 融合 | 沒開 (PhysX 預設) | 8 | 0.30 | 2.77 cm | 6.89 cm | 車頭方向落後 5.4 cm |
| GT `/odom` | μ0.35 (= 模型的訓練條件) | 8 | 0.30 | 2.04 cm | 3.93 cm | - |
| LiDAR + IMU + 輪速 (`multicam:=false`, = 實體車的組合) | μ0.35 | 8 | 0.30 | 2.17 cm | 4.57 cm | 0.8 cm, 沒有落後 |
| GT `/odom` | μ0.35 | **0.5** | 0.30 | **0.17 cm** | 0.56 cm | - |
| LiDAR + 實體車 IMU 設定 + MEMS 雜訊 | μ0.35 | 0.5 | 0.30 | **0.48 cm** | 1.34 cm | 0.7 cm |
| 同上 | μ0.35 | 0.5 | **0.45** | **0.48 cm** | 1.33 cm | 1.3 cm |

* 輪胎摩擦要跟動態模型的訓練條件一樣 (`scripts/isaac_tire_friction.py`, μ0.35)。
* head 8 時 GT 位姿、LiDAR、相機三種位姿來源都是 RMS 2.0~2.3 cm, 而且左轉偏左、右轉偏右 (都往彎道內側)
  —— 是 MPC 本身的問題, 不是定位。

### 切彎的原因與修正 (2026-11-17)

離線 (`受控體 = 模型自己`, 沒有任何擾動) head 8 也一樣切內側 (左轉平均 +2.5 cm), 所以是成本設計。滑移轉向車在
彎道裡車身會側滑, 車頭要偏離切線 2~3° (模型: 半徑 0.8 m 約 3°); `mpc_config.json` 的航向權重 8
太重, MPC 硬把車頭轉回切線, 代價是整台車往彎道內側偏。逐項掃權重 (每一種設定一個程序跑, v_max 0.3,
到路徑的真正距離):

| `weights.head` | 橫向 RMS | 最大 | 指令變化量 |
| --- | --- | --- | --- |
| 8 (舊) | 1.72 cm | 3.57 cm | 0.76 rad/s/步 |
| 2 | 0.77 cm | 1.61 cm | 0.38 |
| 1 | 0.44 cm | 0.99 cm | 0.28 |
| **0.5 (現在)** | **0.18 cm** | **0.52 cm** | 0.20 |
| 0.25 | 0.09 cm | 0.30 cm | 0.15 |
| 0.5, v_max 0.45 | 0.11 cm | 0.37 cm | 0.16 |

試過沒用的: 參考點前視 (變差)、航向改成對行進方向 (變差)、降低 du 權重 (變差)。
**`model/mpc_config.json` 的 `head` 已經從 8 改成 0.5** (`mlp_control_pkg` 共用這一份)。

GT 位姿關掉擾動估計是 0.22 cm (開 0.17 cm) —— 模擬裡模型跟車子幾乎一樣, 估計器沒事做 (估到 wz −0.0125 rad/s,
前進 +0.5 cm/s)。上實體車時這三個值就是「模型跟車差多少」的直接量測。

另外加了 **offset-free MPC** (`disturbance.py`, 預設開, `disturbance:=false` 關掉): 每個週期比較模型
預測的下一步與實際量到的, 低通 (tau 1 s) 後當成擾動加進 MPC 的預測。車子與模型不合 (實體車一定會)
時把穩態偏差吃掉, 沒有不合時不影響。離線測試 (`test/test_offset_free.py`, 受控體多一個每秒往右轉
0.03 rad、往左滑 0.5 cm/s 的擾動, v_max 0.45): 估計關 RMS 0.34 cm, 估計開 0.12 cm, 沒擾動 0.11 cm;
估到的擾動跟設定值一樣。CSV 多三欄 `dist_vfwd, dist_vlat, dist_wz` (估到的擾動, 每秒)。多一次模型計算, 每週期約 +5 ms。

**h4_mixed_track 的 A/B (2026-11-24, 無輪速模型, Isaac + 融合定位, GT 量誤差)**:
* 蛇行 (約 0.45 Hz 左右擺) 是 `du` 權重太小造成的, 不是擾動估計: 擾動估計開著時, `du` 0.5 / 1.5 的圓弧中段差速擺動
  2.4~2.7 rad/s、航向 RMS 5.5~6.0°; `du` 5.0 降到 1.25 rad/s、3.8°。(0.5 -> 1.5 沒有效果, 要調到 5.0 才有。)
* `du` 5.0 下擾動估計開 / 關 (各 4 圈): 循跡 RMS 1.38 / 1.44 cm、p95 2.67 / 2.91 cm、直線偏移 +0.08 / +0.61 cm
  (開著把固定偏差吃掉); 代價是指令較抖: 每步差速變化 0.33 / 0.23 rad/s、航向 3.8 / 3.4°。所以維持預設開。
* 這個模型在閉迴路估到的 yaw rate 差穩定在約 +0.043 rad/s (融合位姿 + vy 給 0), 偶爾 (1.4%) 碰到上限 0.1。
* vy 給 0: 動態模型的輸入有 vy, GT 微分出來的側向速度 RMS 只有 0.9 cm/s, 影響很小。
* 實體車的場地地圖與路徑 CSV 都還沒有。
