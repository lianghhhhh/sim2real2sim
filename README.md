# sim2real2sim

Isaac Sim `car.usd` 場景的 LiDAR + IMU 定位 / 建圖。

| package | 做什麼 | 細節 |
| --- | --- | --- |
| `car_localization` | 定位、建圖、Foxglove 橋接 | [README](src/car_localization/README.md) |
| `car_teleop` | 手動開車 (建圖時要用) | [README](src/car_teleop/README.md) |
| `car_navigation` | 舊的做法 (rf2o / ICP + EKF), 保留參考 | |

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
ros2 topic hz /lidar/point_cloud   # 應該 ~20 Hz
ros2 topic hz /imu                 # 應該 ~60 Hz
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
