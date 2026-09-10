# sim2real2sim

Isaac Sim `car.usd` 場景的車輛定位。

## 五種定位方式

**相機、LiDAR、IMU、IMU+輪速 各自獨立成一條路線。** 這四個 package 之間不互相
import, 每一條也不訂閱別人的 topic —— 這樣才能拿同一趟資料把它們放在一起比較。
**第五條 (`car_loc_fusion`) 是把它們結合起來的那一個**, 所以那條規則對它不適用。

| package | 感測器 | 怎麼定位 | 細節 |
| --- | --- | --- | --- |
| `car_loc_camera` | 只有 `/rgb` | YOLO 找出車在畫面中的位置 -> 單應性投影成世界座標 -> 等速卡爾曼濾波 | [README](src/car_loc_camera/README.md) |
| `car_loc_lidar` | 只有 `/scan` (Oradar MS200) | 手動開車用 slam_toolbox 建圖 -> 掃描對地圖 3 自由度配準 | [README](src/car_loc_lidar/README.md) |
| `car_loc_imu` | 只有 `/imu` | EKF 慣性推算 + ZUPT / ZARU / NHC / 零偏估計 抗漂移 | [README](src/car_loc_imu/README.md) |
| `car_loc_wheel` | `/imu` + `/joint_states` | 7 維 EKF 航位推算: 輪速給前進速度、陀螺儀給 yaw、打滑三道防線 | [README](src/car_loc_wheel/README.md) |
| `car_loc_fusion` | **全部** | 方法五: 遞推 (IMU+輪速) + 絕對量測 (相機/LiDAR) + 倒帶重放補延遲 | [README](src/car_loc_fusion/README.md) |
| `car_teleop` | — | 手動開車 (建圖時要用) | [README](src/car_teleop/README.md) |

> 五種方法的實作細節、誤差量測與橫向比較: **[report_5methods.md](report_5methods.md)**

前四條**互不相干** (才能公平比較); `car_loc_fusion` 是把它們結合起來的那一條 ——
它訂閱前兩條的輸出當絕對量測, 自己吃 `/imu` + `/joint_states` 做高頻遞推。
離線 bench 上融合 **1.35 cm** vs 只有相機 5.42 / 只有 LiDAR 6.97 / 只有航位推算
7.55 cm; 但它真正的價值在**相機被擋住的那 10 秒** (174 cm -> 1.5 cm)。

```bash
# 方法一
ros2 launch car_loc_camera camera_loc.launch.py evaluate:=true

# 方法二 —— 先建圖 (手動開一圈), 再定位
# (第一次要先在 host 上跑 ./scripts/setup_oradar_lidar.py 把 car.usd 的雷射
#  設成 MS200, 見 src/car_loc_lidar/README.md 第 0 節)
ros2 launch car_loc_lidar mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f /workspaces/src/car_loc_lidar/maps/room
ros2 launch car_loc_lidar lidar_loc.launch.py evaluate:=true

# 方法三 (開始前先讓車停幾秒, 要做靜止校正)
ros2 launch car_loc_imu imu_loc.launch.py evaluate:=true

# 方法四 (同上, 但只要停 1 秒)
ros2 launch car_loc_wheel wheel_loc.launch.py evaluate:=true

# 方法五 —— 相機 + LiDAR + 融合一起開 (要先有地圖與 YOLO 模型)
ros2 launch car_loc_fusion fusion_loc.launch.py evaluate:=true
```

每一條都有 `evaluate:=true`, 會拿 Isaac 的 ground truth `/odom` 當尺即時報誤差,
Ctrl-C 印總結。多條同時跑的時候**只能留一條發 `map -> base_link`**, 其他的
加 `publish_tf:=false`。

### 五條一起跑, 跟 GT 記成同一份 CSV

```bash
# 車子生在哪裡: ros2 topic echo /odom --once --field pose.pose.position
ros2 launch bringup_pkg collect_all.launch.py \
    imu_initial_pose:="[2.0, -0.3, 0.0]" csv_filename:=all_loc.csv
ros2 run car_teleop teleop_key          # 另開 terminal 手動開車
```

一列 = 一個時刻的六個答案 (`car_run_data/all_loc.csv`):

| 欄位 | 來源 |
| --- | --- |
| `car_position_x/y`, `gt_yaw` | `/odom` — Isaac ground truth |
| `cam_x/y/yaw` | `/camera_loc/odom` |
| `lid_x/y/yaw` | `/lidar_loc/odom` |
| `imu_x/y/yaw` | `/imu_loc/odom` |
| `whl_x/y/yaw` | `/wheel_loc/odom` |
| `fus_x/y/yaw` | `/fusion_loc/odom` (方法五, 吃 cam_ 與 lid_ 的輸出) |

每一組還配 `_stamp` 跟 `_age`, **兩個都要用**:

- `_stamp` 是那個來源自己的時戳。同一列的五個值**不是同一個時刻**的
  (相機 ~30 Hz、LiDAR 10 Hz、IMU 200 Hz、記錄器 20 Hz), 算誤差前要先用它內插
  對齊。0.8 m/s 時 50 ms 的錯位就是 4 cm —— 跟 LiDAR 的真實誤差同一個量級。
- `_age` 是「這個值放多久了」。節點掛掉時最後一個值會被一路複製到檔尾, 看起來
  像車子停著不動, 統計出來的誤差是假的。先 `df[df.lid_age < 0.3]` 再算 RMS。

`tf_source` 決定誰發 `map -> base_link` (預設 `fusion`), 只蒐 CSV 的話設
`tf_source:=none`。不想跑 YOLO 就加 `camera:=false` (那時 `fus_` 也跟著少一個
來源)。**IMU 與輪速那兩條一定要給 `imu_initial_pose`**, 它們從那裡開始積分,
沒對到出生點的話整段差一個常數平移, 看起來像超大飄移 —— **融合那條不用給**,
它拿第一則絕對量測當起點。

`fus_` 跟 `cam_`/`lid_` **不獨立** (融合吃的就是那兩條的輸出), 拿它們比是公平的
但不要當成獨立樣本。`fus_sigma` 也不同: 它不是單調長大的 (那是 `imu_`/`whl_`),
也不是「這一幀追丟了」(那是 `lid_`), 而是「所有來源合起來還剩多少不確定」——
用它切出「那幾秒只有遞推在撐」的片段。

每一條路線都有**不需要 ROS / Isaac 的離線測試**, README 裡的每一個數字都是
它印出來的 —— 改了參數想知道值不值得就重跑它:

```bash
cd src/car_loc_camera && python3 test/test_tracker.py    # 秒級
cd src/car_loc_lidar  && python3 test/test_matcher.py    # 約 20 秒
cd src/car_loc_imu    && python3 test/test_ins.py        # 秒級
```

各自的精度上限 (每一條的 README 都有實測數字與推導):

| 方法 | 誤差主要來自 | 大概的量級 |
| --- | --- | --- |
| 相機 | **校正模型**, 不是 YOLO (校正殘差 7.26 cm ≈ 實測誤差 7.4 cm) | 濾波後 5~6.5 cm, yaw 只有 ~20°; 看不到車就完全沒有輸出 |
| LiDAR | **地圖解析度** (誤差 ≈ 0.7 x 格點大小) | 5 cm 的圖 -> 一般行駛 5 cm; 轉速 >8 rad/s 或走廊等幾何退化處會追丟 |
| IMU | **姿態誤差** (傾斜 1 度 = 0.17 m/s² 的假加速度) | 隨時間長大; 撐多久取決於多久停一次車 |
| IMU+輪速 | **yaw 誤差 x 走過的距離** (輪速把 t² 變成距離的一次式) | 走 100 m 而 yaw 差 1 度 = 1.7 m |
| 融合 | **絕對量測的延遲**沒補的話 = v x 80 ms (不是精度問題, 是時間軸) | 補了 1.4 cm; 沒補 3 m/s 時 27 cm |

---

## 感測器 (2026-09 換過)

車上的雷射已經從模擬用的 SICK multiScan136 (3D, 16 線, 40 m) 換成實體車那顆
**Oradar MS200** (2D 單線, 12 m, 10 Hz, 一圈 450 點)。`car.usd` 裡的 prim 是
`/World/small_car/Cube/oradar_ms200`, 掛在 world z = 0.200。

* 設定的方法與踩過的坑: [src/car_loc_lidar/README.md](src/car_loc_lidar/README.md) 第 0 節
* topic 從 `/lidar/point_cloud` (PointCloud2) 換成 **`/scan`** (LaserScan),
  跟實體車的 MS200 驅動完全一致
* `car.usd` 的原始備份在 `car.usd.bak`

---

## 舊的做法 (LiDAR + IMU 融合, 保留參考)

下面這一整段是把 LiDAR 與 IMU **融合**在一起的舊路線。它精度最好, 但三種感測器
綁在一起, 沒辦法單獨評估任何一種。

> **這一段的指令現在不會直接跑起來** —— 它們吃的是 `/lidar/point_cloud`, 而雷射
> 換成 MS200 之後那個 topic 不存在了。`car_localization` 本來就有
> `input_type:=scan` 這條路, 要跑的話加上
> `input_type:=scan scan_topic:=/scan range_max:=12.5`。
> `scripts/fix_car_usd_lidar.py` 也已經過時 (它找的是 multiScan136 那顆 prim)。

| package | 做什麼 | 細節 |
| --- | --- | --- |
| `car_localization` | 定位、建圖、Foxglove 橋接 | [README](src/car_localization/README.md) |
| `car_inference` | 相機 + YOLO (只發位置, 沒有濾波) | |
| `car_loc_eskf` / `car_loc_mcl` / `car_loc_graph` / `car_loc_robust` | 各種融合濾波器 | |
| `car_navigation` | 更舊的做法 (rf2o / ICP + EKF) | |

---

## 標記說明

* 🖥 = 在 **host** 上跑
* 📦 = 在 **容器裡**跑 (`run_isaac_gui.sh` 開出來的那個 shell, 或 `docker exec -it ros2_node bash`)

---

## 0. 一次性準備

🖥 從 `car.usd` 的幾何直接切出地圖 (不用開 Isaac, 大約 20 秒):

```bash
cd ~/sim2real2sim
./scripts/make_map_from_usd.py
```

它會印出房間的俯視 ASCII 圖 —— **看一眼**, 牆要是細線、形狀要像那個房間。
輸出在 `src/car_localization/maps/car_usd.npz` (同名的 `.pgm`/`.yaml` 給 rviz/Foxglove)。

改過 `car.usd` 的幾何 (搬牆、加柱子) 就要重跑一次。

---

## 1. 每次都要做的三件事

**🖥 終端 A —— 開容器 (先做)**

```bash
cd ~/sim2real2sim
bash run_isaac_gui.sh
```

容器叫 `ros2_node`, `ROS_DOMAIN_ID=82` (跟 `car.usd` 裡的 ROS2Context 一致)。
`--rm`: 離開這個 shell 容器就沒了。

**🖥 終端 B —— 開 Isaac Sim**

```bash
~/isaac-sim/isaac-sim.streaming.sh
```

在 GUI 裡 File -> Open 選 `~/sim2real2sim/car.usd`, 然後按 **Play** (▶)。
沒按 Play 就不會有任何 ROS topic。

**📦 build**

```bash
r          # = colcon build --symlink-install && source install/setup.bash
```

之後要再開幾個容器 shell 就用:

```bash
tmux  # 再 ctrl+b, shift+' -> 多開幾個終端
```

**先確認 Isaac 真的在發資料**:

```bash
ros2 topic hz /scan                # 應該 ~10 Hz (Oradar MS200)
ros2 topic hz /imu                 # 應該 ~60 Hz
ros2 topic echo /scan --field ranges --once | head -3   # 一圈應該是 450 筆
```

---

## 2. 情境 A：直接定位 (car.usd, 用 USD 切出來的地圖)

**這是平常在模擬裡要用的。** 不用建圖、不用給初始位姿。

📦
```bash
ros2 launch car_localization localization.launch.py evaluate:=true
```

`evaluate:=true` 會拿 Isaac 的 ground truth `/odom` 當尺, 每 5 秒印一行誤差:

```
[live ] 1298 筆, GT 走了 28.27 m | 位置誤差 RMS 0.80 cm, 平均 0.57 cm, p95 1.02 cm
```

Ctrl-C 會印總結。要逐點存檔加 `csv:=/workspaces/car_run_data/loc_eval.csv`。

想邊定位邊開車: 加 `teleop:=true`, 然後另開一個 shell 跑 `ros2 run car_teleop teleop_key`。

---

## 3. 情境 B：手動開車建圖 (換到實體環境走這條)

**📦 終端 1 —— 建圖**

```bash
ros2 launch car_localization slam.launch.py
```

裡面同時起了三個東西: 雷射里程計 (發 `odom -> base_link` 和運動補償過的 `/scan`)、
slam_toolbox (發 `map -> odom` 和 `/map`)、遙控的速度控制層。

**📦 終端 2 —— 鍵盤遙控** (`-it` 是必要的, 鍵盤需要真的 TTY)

```bash
docker exec -it ros2_node bash -lc 'r && ros2 run car_teleop teleop_key'
```

```
  w/s 前進後退   a/d 左右轉   空白 停
  +/- 速度上限   [/] 轉向上限   t 切換按住/持續   q 離開
```

開的時候三件事:

* **慢慢開** (預設上限 0.6 m/s 就是為了這個)。開太快掃描比對跟不上, 位姿一漂地圖就歪。
* **柱子後面、四個角落都要繞到**, 沒繞到的地方地圖上就是空的。
* **要繞回起點**, 回環偵測才有東西可以閉。

**📦 終端 3 —— 存圖** (跟 wildbot 的 `docker-compose_store_map.yml` 同一個指令)

```bash
ros2 run nav2_map_server map_saver_cli -f /workspaces/src/car_localization/maps/room
```

**📦 用這張圖定位** (直接吃 nav2 的 `.yaml`, 不用轉檔)

```bash
ros2 launch car_localization localization.launch.py \
    map_path:=/workspaces/src/car_localization/maps/room.yaml evaluate:=true
```

> SLAM 地圖的 `map` 原點是**車子按 Play 那一刻的位置**, 不是 USD 世界原點, 所以
> 位置會跟 Isaac 的 `/odom` 差一個固定平移 —— 那不是定位在漂。`evaluate` 會把常數
> 偏移單獨報出來, 並直接印出修正指令 (把 `.yaml` 的 `origin` 減掉它)。

---

## 4. 用 Foxglove 看

📦
```bash
ros2 launch car_localization viz.launch.py
```

它會把要貼進 Foxglove 的網址印出來 (`ws://<容器IP>:9090`)。
Foxglove Studio -> Open connection -> **Rosbridge** -> 貼上。

| topic | 看什麼 |
| --- | --- |
| `/map` | 地圖 (OccupancyGrid) |
| `/scan` | 運動補償後的一圈掃描 |
| `/localization/pose` | 車子現在在哪 |
| `/localization/scan_matched` | 配準後的點雲, 疊在地圖上看貼不貼 (`publish_debug_cloud:=true`) |
| `/tf` | `map -> base_link -> sim_lidar / sim_imu` |
| `/cmd_vel` | Foxglove 的 **Teleop 面板**往這裡發, 就能用滑鼠開車 |

`src/car_localization/config/foxglove_layout.json` 是現成版面 (Layout -> Import from file)。
匯不進去就照上表自己拉面板。

目前映像檔只有 `rosbridge_server`; `Dockerfile` 已經加了 `foxglove-bridge`,
重 build 之後 `viz.launch.py` 會自動改用它 (port 8765, 點雲效能好很多)。

連不上就在 `run_isaac_gui.sh` 的 `docker run` 加 `-p 9090:9090`。

---

## 5. 不開 Isaac 先跑一遍

`fake_isaac` 照 `car.usd` 的規格合成 `/clock`, `/imu`, `/lidar/point_cloud`, `/joint_states`
和 ground truth `/odom`。整條流程都能先驗過。

📦
```bash
# 照腳本走 (驗定位精度)
ros2 run car_localization fake_isaac --ros-args -p motion:=figure8

# 可以手動開 (驗遙控 + 建圖流程)
ros2 run car_localization fake_isaac --ros-args -p motion:=drive
```

然後照情境 A 或 B 的指令跑就好。

📦 也可以只驗地圖與配準 (不需要 ROS):

```bash
cd /workspaces/src/car_localization && python3 test/test_matcher.py
```

---

## 6. 常用檢查指令

📦
```bash
# 地圖長什麼樣 (ASCII 俯視圖)
python3 -m car_localization.gridmap show /workspaces/src/car_localization/maps/car_usd.npz

# 重新做一次全域定位 (車子被搬走了之類)
ros2 service call /car_localizer/relocalize std_srvs/srv/Trigger

# 建圖模式中途存檔
ros2 service call /car_localizer/save_map std_srvs/srv/Trigger

# 看定位輸出
ros2 topic echo /localization/odom --field pose.pose.position
```

🖥
```bash
# 檢查 car.usd 的 LiDAR 設定 (掛載高度 / fullScan / 時鐘 reset)
./scripts/fix_car_usd_lidar.py --dry-run
```

---

## 7. 換到實體車

```bash
ros2 launch car_localization slam.launch.py \
    use_sim_time:=false input_type:=scan imu_topic:=/imu/data yaw_source:=gyro
```

| 參數 | 為什麼 |
| --- | --- |
| `input_type:=scan` | 實體車是 2D 雷射 (wildbot 用 oradar), 發 `LaserScan`。這個模式會自動關掉高度過濾 |
| `yaw_source:=gyro` | 真車的 6 軸 IMU 沒有絕對 yaw, 改成陀螺儀積分 + 掃描比對修正 |
| `use_sim_time:=false` | 沒有 `/clock` |
| `lidar_translation` | 改成你車上實際的雷射掛載位置 |

`car_teleop` 的輪半徑/輪距/關節名稱也要改成實體車的 (在 `cmd_vel_bridge` 的參數裡)。

---

## 出事的時候

| 症狀 | 通常是 |
| --- | --- |
| 沒有任何 topic | Isaac 沒按 Play, 或 `ROS_DOMAIN_ID` 不是 82 |
| 「找不到地圖檔」 | 沒跑過 `./scripts/make_map_from_usd.py`, 或跑完沒重新 `r` |
| 「LiDAR 與 IMU 的時間源對不上」 | Isaac 反覆 Stop/Play 後時鐘分家。🖥 跑 `./scripts/fix_car_usd_lidar.py` 再重載場景 |
| 「高度帶裡只剩 N 點」 | `lidar_translation` 跟 USD 的實際掛載高度對不上 (應該是 0.200) |
| 誤差是一個不會變的常數 | 地圖原點跟 `/odom` 原點差一個平移, 不是在漂。看 `evaluate` 的「常數偏移」那行 |
| 車子按前進鍵一直加速 | `car_teleop` 的 bridge 沒起來, 或收不到 `/joint_states`+`/imu` 回授 |

更深的說明在 [src/car_localization/README.md](src/car_localization/README.md)。
