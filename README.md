# sim2real2sim

Isaac Sim `car.usd` 場景的車輛定位。五種方法各自一個 package, 可以單獨跑,
也可以一次全開、跟 ground truth 記進同一份 CSV 比較。

## Package 一覽

| package | 做什麼 | 細節 |
| --- | --- | --- |
| `car_loc_camera` | 方法一: 只有 `/rgb`。YOLO 找車 -> 單應性投影成世界座標 -> 等速卡爾曼濾波 | [README](src/car_loc_camera/README.md) |
| `car_loc_lidar` | 方法二: 只有 `/scan` (Oradar MS200)。slam_toolbox 建圖 -> 掃描對地圖 3 自由度配準 | [README](src/car_loc_lidar/README.md) |
| `car_loc_imu` | 方法三: 只有 `/imu`。EKF 慣性推算 + ZUPT / ZARU / NHC / 零偏估計 抗漂移 | [README](src/car_loc_imu/README.md) |
| `car_loc_wheel` | 方法四: `/imu` + `/joint_states`。7 維 EKF 航位推算: 輪速給前進速度、陀螺儀給 yaw | [README](src/car_loc_wheel/README.md) |
| `car_loc_fusion` | 方法五: **全部**。遞推 (IMU+輪速) + 絕對量測 (相機/LiDAR) + 倒帶重放補延遲 | [README](src/car_loc_fusion/README.md) |
| `car_teleop` | 手動開車: `/cmd_vel` -> 閉迴路扭矩控制, 加鍵盤遙控 | [README](src/car_teleop/README.md) |
| `car_viz` | 開 WebSocket 給 Foxglove 連, 附現成版面 | [README](src/car_viz/README.md) |
| `calibrate_env_pkg` | `collect_data_node` (把 GT + 五條定位記成 CSV) 與摩擦力測試腳本 | |
| `bringup_pkg` | `collect_all.launch.py` (五條一起開) 與 `my_launch.launch.py` (摩擦力測試) | |

前四條**互不相干** —— 彼此不 import、不訂閱別人的 topic, 才能拿同一趟資料公平比較。
`car_loc_fusion` 是把它們結合起來的那一條: 訂閱相機與 LiDAR 的輸出當絕對量測,
自己吃 `/imu` + `/joint_states` 做高頻遞推。

> 報告:
> * 五種方法的實作細節與離線 bench 比較: **[report_5methods.md](report_5methods.md)**
> * Isaac 上一次全開的實測 (誤差大小、隨時間的變化): **[report_all_sensors.md](report_all_sensors.md)**

---

## 標記說明

* 🖥 = 在 **host** 上跑
* 📦 = 在 **容器裡**跑 (`run_isaac_gui.sh` 開出來的那個 shell, 或 `docker exec -it ros2_node bash`)

---

## 0. 一次性準備

🖥 **把 `car.usd` 裡的雷射設成 MS200** (做過就不用再做), 見
[src/car_loc_lidar/README.md](src/car_loc_lidar/README.md) 第 0 節:

```bash
./scripts/setup_oradar_lidar.py
```

📦 **LiDAR 那條要一張地圖。** repo 裡已經有一張 `src/car_loc_lidar/maps/room.yaml`
(5 cm 格點)。改過 `car.usd` 的幾何 (搬牆、加柱子) 就要照第 3 節重建。

📦 **相機那條要 YOLO 模型**, 放在 `src/car_loc_camera/resource/best.onnx`
(已經放了)。

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
沒按 Play 就不會有任何 ROS topic。沒有 X display (SSH) 的話用
`scripts/isaac_headless.py` 或 `scripts/run_isaac_docker.sh` 無頭跑。

**📦 build**

```bash
r          # = colcon build --symlink-install && source install/setup.bash
```

之後要再開幾個容器 shell 就用 `tmux` (ctrl+b, shift+' 多開幾個終端)。

**先確認 Isaac 真的在發資料**:

```bash
ros2 topic hz /scan                # 應該 ~10 Hz (Oradar MS200)
ros2 topic hz /imu                 # 應該 ~60 Hz
ros2 topic echo /scan --field ranges --once | head -3   # 一圈應該是 450 筆
python3 /workspaces/scripts/check_scan.py               # 掃描是不是完整的一圈
```

---

## 2. 定位

### 單獨跑某一條

📦
```bash
# 方法一 —— 相機
ros2 launch car_loc_camera camera_loc.launch.py evaluate:=true

# 方法二 —— LiDAR (要先有地圖, 見第 0 / 3 節)
ros2 launch car_loc_lidar lidar_loc.launch.py evaluate:=true

# 方法三 —— 純 IMU (開始前先讓車停幾秒, 要做靜止校正)
ros2 launch car_loc_imu imu_loc.launch.py evaluate:=true

# 方法四 —— IMU + 輪速 (同上, 停 1 秒就夠)
ros2 launch car_loc_wheel wheel_loc.launch.py evaluate:=true

# 方法五 —— 相機 + LiDAR + 融合一起開
ros2 launch car_loc_fusion fusion_loc.launch.py evaluate:=true
```

`evaluate:=true` 會拿 Isaac 的 ground truth `/odom` 當尺即時報誤差, Ctrl-C 印總結;
逐點存檔加 `csv:=/workspaces/car_run_data/xxx.csv`。想邊定位邊開車就加
`teleop:=true`, 另開一個 shell 跑 `ros2 run car_teleop teleop_key`。

多條同時跑的時候**只能留一條發 `map -> base_link`**, 其他的加 `publish_tf:=false`。

### 五條一起跑, 跟 GT 記成同一份 CSV

📦
```bash
# 車子生在哪裡: ros2 topic echo /odom --once --field pose.pose.position
ros2 launch bringup_pkg collect_all.launch.py \
    imu_initial_pose:="[2.0, -0.3, 0.0]" csv_filename:=all_loc_0910.csv
ros2 run car_teleop teleop_key          # 另開 terminal 手動開車
```

**每一輪給不同的 `csv_filename`** —— 同名會直接覆蓋上一輪。

一列 = 一個時刻的六個答案 (`car_run_data/<csv_filename>`):

| 欄位 | 來源 |
| --- | --- |
| `car_position_x/y`, `gt_yaw` | `/odom` — Isaac ground truth |
| `cam_x/y/yaw` | `/camera_loc/odom` |
| `lid_x/y/yaw` | `/lidar_loc/odom` |
| `imu_x/y/yaw` | `/imu_loc/odom` |
| `whl_x/y/yaw` | `/wheel_loc/odom` |
| `fus_x/y/yaw` | `/fusion_loc/odom` (方法五, 吃 cam_ 與 lid_ 的輸出) |

每一組還配 `_stamp`、`_age`、`_sigma`:

- `_stamp` 是那個來源自己的時戳。同一列的五個值**不是同一個時刻**的
  (相機 ~30 Hz、LiDAR 10 Hz、IMU 200 Hz、記錄器 20 Hz), 算誤差前要先用它內插
  對齊。0.8 m/s 時 50 ms 的錯位就是 4 cm —— 跟 LiDAR 的真實誤差同一個量級。
- `_age` 是「這個值放多久了」。節點掛掉時最後一個值會被一路複製到檔尾, 看起來
  像車子停著不動, 統計出來的誤差是假的。
- `_sigma` 是節點自己回報的 1-sigma, 每一條的意義不一樣 (見 `collect_data_node.py`
  的說明)。LiDAR 用 `lid_sigma < 0.0025` 就能把 180° 對稱解的追丟幀全部擋掉。

`tf_source` 決定誰發 `map -> base_link` (預設 `fusion`), 只蒐 CSV 的話設
`tf_source:=none`。不想跑 YOLO 就加 `camera:=false` (那時 `fus_` 也跟著少一個
來源)。**IMU 與輪速那兩條一定要給 `imu_initial_pose`**, 它們從那裡開始積分,
沒對到出生點的話整段差一個常數平移, 看起來像超大飄移 —— **融合那條不用給**,
它拿第一則絕對量測當起點。

`fus_` 跟 `cam_`/`lid_` **不獨立** (融合吃的就是那兩條的輸出), 拿它們比是公平的
但不要當成獨立樣本。

### 分析 CSV (🖥, 不需要 ROS)

```bash
./scripts/eval_loc_csv.py car_run_data/all_loc_0910.csv --sweep     # 各條的誤差 + 過濾門檻掃描
./scripts/report_all_loc.py car_run_data/all_loc_0910.csv \
    --out car_run_data/report_0910                                  # 時間對齊誤差、隨時間變化、圖
```

### 精度 (Isaac 實測, 摩擦力測試腳本, 2026-09-11 全開那一輪)

| 方法 | 位置誤差中位 | 誤差主要來自 |
| --- | --- | --- |
| 相機 | 0.6–0.7 cm | **延遲** (~40–55 ms): 誤差跟車速成正比, 3 m/s 時 10–15 cm。補償後 < 1 cm。看不到車就沒有輸出 |
| LiDAR | 7.3 cm (σ 過濾後 7.1 cm, 靜止 6.8 cm) | 自旋 **> 8 rad/s** (10 Hz 雷射的物理上限) 會追丟、可能鎖到 180° 對稱解; 鎖死偵測 (`lock_*`) 停下來後自己整張地圖重定位, 最長 2.7 s 回來。整輪追丟 2.9% |
| 純 IMU | 0.7–1.2 m, 最後 3–7 m | 隨**走過的距離**發散, 大約距離的 5–13%。yaw 很準是因為模擬直接給了姿態 |
| IMU+輪速 | 15.3 cm | 走 59.9 m 漂移率 0.76% (最大誤差 / 距離)。yaw 誤差 x 走過的距離 (走 100 m 而 yaw 差 1° = 1.7 m) |
| 融合 | 還沒在 Isaac 上量 | 離線 bench 1.35 cm; 價值在**相機被擋住的那 10 秒** (174 cm -> 1.5 cm) |

細節與圖見 [report_all_sensors.md](report_all_sensors.md)。

---

## 3. 手動開車建圖 (換場景或到實體環境時)

**📦 終端 1 —— 建圖** (裡面已經包含 `car_teleop` 的速度控制層)

```bash
ros2 launch car_loc_lidar mapping.launch.py
```

**📦 終端 2 —— 鍵盤遙控** (`-it` 是必要的, 鍵盤需要真的 TTY)

```bash
docker exec -it ros2_node bash -lc 'r && ros2 run car_teleop teleop_key'
```

```
  w/s 前進後退   a/d 左右轉   空白 停
  +/- 速度上限   [/] 轉向上限   t 切換按住/持續   q 離開
```

不想用鍵盤的話, 開 `car_viz` (第 4 節) 用 Foxglove 的 Teleop 面板開車也可以。
要重複同一條路徑就用 `python3 /workspaces/scripts/tour_drive.py --loops 2`。

開的時候三件事:

* **慢慢開** (預設上限 0.6 m/s 就是為了這個)。開太快掃描比對跟不上, 位姿一漂地圖就歪。
* **柱子後面、四個角落都要繞到**, 沒繞到的地方地圖上就是空的。
* **要繞回起點**, 回環偵測才有東西可以閉。

**📦 終端 3 —— 存圖**, 然後 `r` 讓新地圖裝進 share:

```bash
ros2 run nav2_map_server map_saver_cli -f /workspaces/src/car_loc_lidar/maps/room
r
```

> SLAM 地圖的原點由建圖時的起點決定, 位置可能會跟 Isaac 的 `/odom` 差一個固定
> 平移 —— 那不是定位在漂。誤差是一個不會變的常數時, 去改地圖 `.yaml` 的 `origin`。

---

## 4. 用 Foxglove 看

📦
```bash
ros2 launch car_viz viz.launch.py
```

它會把網址印出來 (`ws://<容器IP>:8765`, 沒裝 foxglove_bridge 時是 rosbridge 的 9090)。
Foxglove Studio -> Open connection -> 貼上。現成版面在
`src/car_viz/config/foxglove_layout.json` (Layout -> Import from file):
地圖 + 掃描 + GT + 五條位姿、YOLO 畫面、Teleop、五條線的 x 對時間。

`car_viz` 只開門, 不起任何定位節點 —— 要看的自己另外開 (例如第 2 節的
`collect_all.launch.py`)。可以看的 topic 與連線問題見 [src/car_viz/README.md](src/car_viz/README.md)。

---

## 5. 不開 Isaac 先驗一遍

每一條都有**不需要 ROS / Isaac 的離線測試**, 各自 README 裡的數字就是它印出來的 ——
改了參數想知道值不值得就重跑它:

```bash
cd src/car_loc_camera && python3 test/test_tracker.py      # 秒級
cd src/car_loc_imu    && python3 test/test_ins.py          # 約 5 秒
cd src/car_loc_wheel  && python3 test/test_wheel_ins.py    # 約 5 秒
cd src/car_loc_lidar  && python3 test/test_matcher.py      # 約 25 秒
cd src/car_loc_fusion && python3 test/test_fusion.py       # 約 30 秒
```

---

## 6. 常用檢查指令

📦
```bash
# 地圖長什麼樣 (ASCII 俯視圖, 不需要 ROS)
python3 -m car_loc_lidar.gridmap show /workspaces/src/car_loc_lidar/maps/room.yaml

# LiDAR 重新做一次全域定位 (車子被搬走、或鎖到 180 度對稱解了)
ros2 service call /lidar_localizer/relocalize std_srvs/srv/Trigger

# 其他幾條重設 (camera / imu / wheel / fusion)
ros2 service call /fusion_localizer/reset std_srvs/srv/Trigger

# 看某一條的輸出
ros2 topic echo /fusion_loc/odom --field pose.pose.position
```

---

## 7. 換到實體車

雷射是同一顆規格 (MS200)、同一個 topic (`/scan`)、同一份參數, 只要關掉模擬時鐘:

```bash
ros2 launch car_loc_lidar lidar_loc.launch.py use_sim_time:=false
ros2 launch car_loc_imu imu_loc.launch.py use_sim_time:=false \
    imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro
```

| 參數 | 為什麼 |
| --- | --- |
| `use_sim_time:=false` | 沒有 `/clock` |
| `gravity_mode:=complementary` | 真車的 6 軸 IMU 沒有 orientation, 改成陀螺儀積分 + 靜止時用加速度校正 |
| `yaw_source:=gyro` | 同上, 沒有絕對 yaw |

`car_teleop` 的輪半徑 / 輪距 / 關節名稱也要改成實體車的 (在 `cmd_vel_bridge` 的參數裡)。

---

## 感測器 (2026-09 換過)

車上的雷射已經從模擬用的 SICK multiScan136 (3D, 16 線, 40 m) 換成實體車那顆
**Oradar MS200** (2D 單線, 12 m, 10 Hz, 一圈 450 點)。`car.usd` 裡的 prim 是
`/World/small_car/Cube/oradar_ms200`, 掛在 world z = 0.200。

* 設定的方法與踩過的坑: [src/car_loc_lidar/README.md](src/car_loc_lidar/README.md) 第 0 節
* topic 從 `/lidar/point_cloud` (PointCloud2) 換成 **`/scan`** (LaserScan),
  跟實體車的 MS200 驅動完全一致
* `car.usd` 沒有另外留 `.bak`, 改之前的版本在 git 歷史裡: `git log -- car.usd`
  列出來, 例如 `git show 7c984df:car.usd > /tmp/car_old.usd` (跟現在的不一樣)

舊的定位 package (`car_localization`、`car_inference`、`car_navigation` 等) 已經在
commit `b029732` 刪掉, 要翻舊程式就 `git show 7c984df:src/<package>/...`。

---

## 出事的時候

| 症狀 | 通常是 |
| --- | --- |
| 沒有任何 topic | Isaac 沒按 Play, 或 `ROS_DOMAIN_ID` 不是 82 |
| LiDAR「找不到地圖」 | 還沒建圖, 或建完沒 `r`。也可以直接給 `map_path:=...` |
| LiDAR 位置還算合理但 yaw 差 180° | 自旋太快 (> 8 rad/s) 鎖到對稱解。看 `lid_sigma` (> 0.0025 就是)。車子停下來約 1.5 s 鎖死偵測會自己整張地圖重定位 (記錄印「疑似鎖在對稱解」); 一直沒回來才手動呼叫 `relocalize` |
| LiDAR 一轉彎就大量「配準失敗」 | `lidar_loc.yaml` 的 `auto_scan_stamp` 被打開、投錯了時序。保持 `false` + `reverse/end` |
| 掃描在 Foxglove 上是幾段斷開的弧 | `car.usd` 的 `fullScan` 沒開, 重跑 `./scripts/setup_oradar_lidar.py` |
| 誤差是一個不會變的常數 | 地圖原點跟 `/odom` 原點差一個平移, 不是在漂。改地圖 `.yaml` 的 `origin` |
| IMU / 輪速一開始就差很遠 | `imu_initial_pose` 沒對到出生點 |
| CSV 裡 `cam_stamp` 比 `/odom` 多了上千秒 | 時鐘基準不一樣 (`eval_loc_csv.py` 會警告)。分析時要先扣掉; **開融合之前要先查** |
| 車子按前進鍵一直加速 | `car_teleop` 的 bridge 沒起來, 或收不到 `/joint_states`+`/imu` 回授 |
| Foxglove 連不上 | `run_isaac_gui.sh` 沒做 port mapping, 用容器 IP 直連或加 `-p 8765:8765` |
