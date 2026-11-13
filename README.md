# sim2real2sim

Isaac Sim 裡 KROS car (`kros_car_new.usd`) 的車輛定位。六種方法各自一個 package, 可以單獨跑,
也可以一次全開、跟 ground truth 記進同一份 CSV 比較。

## Package 一覽

**定位**

| package | 做什麼 | 細節 |
| --- | --- | --- |
| `car_loc_camera` | 方法一: 只有天花板那台 `/rgb`。YOLO 找車 -> 單應性投影成世界座標 -> 等速卡爾曼濾波 | [README](src/car_loc_camera/README.md) |
| `car_loc_lidar` | 方法二: 只有 `/scan` (Oradar MS200)。slam_toolbox 建圖 -> 掃描對地圖 3 自由度配準 | [README](src/car_loc_lidar/README.md) |
| `car_loc_imu` | 方法三: 只有 `/imu`。EKF 慣性推算 + ZUPT / ZARU / NHC / 零偏估計 抗漂移 | [README](src/car_loc_imu/README.md) |
| `car_loc_wheel` | 方法四: `/imu` + `/joint_states`。7 維 EKF 航位推算: 輪速給前進速度、陀螺儀給 yaw | [README](src/car_loc_wheel/README.md) |
| `car_loc_fusion` | 方法五: **全部**。遞推 (IMU+輪速) + 絕對量測 (8 台相機的車框 + LiDAR) + 倒帶重放補延遲 | [README](src/car_loc_fusion/README.md) |
| `car_loc_multicam` | 方法六: 8 台斜拍環境相機 (`/env_cam_0..7/rgb`) + IMU + 輪速。YOLO 框**直接當 EKF 量測** (量測模型 = 視線), 用車的 3D 外形反推起點 | [README](src/car_loc_multicam/README.md) |

**控制、記錄、工具**

| package | 做什麼 | 細節 |
| --- | --- | --- |
| `car_teleop` | 手動開車: `/cmd_vel` -> 四輪輪速, 加鍵盤遙控; `actuator_model` 把實體車馬達的延遲與加速斜坡加到模擬的輪子上 | [README](src/car_teleop/README.md) |
| `car_viz` | 開 WebSocket 給 Foxglove 連, 附現成版面 | [README](src/car_viz/README.md) |
| `calibrate_env_pkg` | `collect_data_node` (把 GT + 六條定位記成 CSV) 與摩擦力測試腳本 (`control_car_node`) | |
| `bringup_pkg` | `collect_all.launch.py` (六條一起開)、`friction_test.launch.py` (摩擦力測試 + 定位)、`my_launch.launch.py` (只跑摩擦力腳本) | |
| `car_imu_noise` | **只給模擬用**: 把真實 MEMS 的零偏與雜訊加到 Isaac 的 `/imu` 上 | [README](src/car_imu_noise/README.md) |
| `collect_car_pkg` | **只給模擬用**: 隨機輪速指令開車, 收車輛動態模型的訓練資料 (給 `dynamic_predictor/`) | [README](src/collect_car_pkg/README.md) |
| `mlp_control_pkg` | 用學到的動態模型做 MPC 循跡 (acados)。位姿吃 Isaac 的 `/odom`, 目前只能在模擬跑 | |

方法一到四**互不相干** —— 彼此不 import、不訂閱別人的 topic, 才能拿同一趟資料公平比較。
方法五和方法六是**同一個節點類別** (`car_loc_multicam` 的 `multicam_fusion`): 都吃 8 台相機的
車框當絕對量測、自己吃 `/imu` + `/joint_states` 做高頻遞推, 差別只在方法五**多吃 LiDAR**。
所以拿 `fus_` 跟 `mc_` 比, 比的就是「LiDAR 值多少」。天花板相機 (方法一) 已經**不是**融合的輸入了。

> 報告:
> * 五種方法的實作細節與離線 bench 比較 (方法六之前寫的): **[report_5methods.md](report_5methods.md)**
> * Isaac 上一次全開的實測 (2026-09-10 那一輪的快照, 已經過時, 見它開頭的說明): **[report_all_sensors.md](report_all_sensors.md)**
> * IMU 雜訊模型與校正 (含 Isaac 的 IMU 跟真車差多少): **[report_imu_noise.md](report_imu_noise.md)**
> * 多相機與融合的最新數字: [car_loc_multicam](src/car_loc_multicam/README.md)、[car_loc_fusion](src/car_loc_fusion/README.md) 的開頭

---

## 標記說明

* 🖥 = 在 **host** 上跑
* 📦 = 在 **容器裡**跑 (`run_isaac_gui.sh` 開出來的那個 shell, 或 `docker exec -it ros2_node bash`)

---

## 0. 一次性準備

🖥 **雷射要是 MS200。** `kros_car*.usd` 都已經設好了; 換一個新的 stage 才要跑
(腳本預設改的是 `kros_car.usd`, 別的檔用 `--usd` 指定), 見
[src/car_loc_lidar/README.md](src/car_loc_lidar/README.md) 第 0 節:

```bash
./scripts/setup_oradar_lidar.py --usd <stage>.usd
```

📦 **LiDAR 那條要一張地圖。** repo 裡已經有一張 `src/car_loc_lidar/maps/room.yaml`
(2.5 cm 格點)。改過房間的幾何 (搬牆、加柱子) 就要照第 3 節重建, 並重新校正 origin。

📦 **相機那兩條要 YOLO 模型**, 放在 `src/car_loc_camera/resource/best.onnx`
(已經放了; 方法一和多相機共用這一個)。

📦 **多相機那條要 8 台相機的內外參與車的外形**: `src/car_loc_multicam/config/env_cameras.yaml`、
`car_model.yaml`, 已經從 `kros_car_new.usd` 匯出了。相機搬了、車的外形或手臂姿勢改了才要重匯出,
見 [car_loc_multicam 的「校正 / 換場景時要重做的事」](src/car_loc_multicam/README.md)。

---

## 1. 每次都要做的三件事

**🖥 終端 A —— 開容器 (先做)**

```bash
cd ~/sim2real2sim
bash run_isaac_gui.sh
```

容器叫 `ros2_node`, `ROS_DOMAIN_ID=82` (跟 USD 裡的 ROS2Context 一致)。
`--rm`: 離開這個 shell 容器就沒了。

**🖥 終端 B —— 開 Isaac Sim**

```bash
~/isaac-sim/isaac-sim.streaming.sh
```

在 GUI 裡 File -> Open 選 `~/sim2real2sim/kros_car_new.usd`, 然後按 **Play** (▶)。
沒按 Play 就不會有任何 ROS topic。沒有 X display (SSH) 的話用
`scripts/isaac_headless.py` 或 `scripts/run_isaac_docker.sh` 無頭跑。

要用哪一份 USD:

| USD | 8 台環境相機 | 輪子吃哪個 topic | 用途 |
| --- | --- | --- | --- |
| `kros_car_new.usd` | **有** | `/joint_command_isaac` | 日常用。方法五、六只有在這份上才有相機 |
| `kros_car.usd` | 沒有 | `/joint_command_isaac` | 車的原始模型 (下面「kros_car.usd」那一節); `collect_car_pkg` 收動態資料用 |
| `kros_car_sim.usd` / `kros_car_real.usd` | **有** | `/joint_command_isaac` | `run_car.sh` 摩擦力自動校正的預設輸入 (2026-11-11 起, 跟 `kros_car.usd` 同一份)。μ 由自訂輪胎摩擦設定, 不在 USD 裡 |
| `kros_car_05_sim.usd` / `kros_car_10_real.usd` / `kros_car_sim_calibrated.usd` | 沒有 | `/joint_command` | 舊的 `run_car.sh --friction-model physx` 輸入 (地面 mu 不一樣)。融合在這幾份上只剩 LiDAR + 遞推 |
| `car.usd` | 沒有 | `/joint_command` (effort) | 舊車, 現在的控制節點開不動它 (見下面「kros_car.usd」那一節) |

**📦 輪子吃 `/joint_command_isaac` 的 USD, 一定要開致動器模型, 不然車子完全不動:**

```bash
ros2 launch car_teleop actuator_model.launch.py
```

它把 `/joint_command` 加上實體車馬達的延遲、30 rad/s² 加速斜坡與停車衰減, 轉發到
`/joint_command_isaac`。下面所有會讓車動的指令 (teleop、建圖、摩擦力腳本) 都要它在跑。

**📦 build**

```bash
r          # = colcon build --symlink-install && source install/setup.bash
```

之後要再開幾個容器 shell 就用 `tmux` (ctrl+b, shift+' 多開幾個終端)。

**先確認 Isaac 真的在發資料**:

```bash
ros2 topic hz /scan                # 應該 ~10 Hz (Oradar MS200)
ros2 topic hz /imu                 # 應該 ~60 Hz
ros2 topic hz /env_cam_0/rgb       # 8 台環境相機 (env_cam_0..7), 只有 kros_car_new.usd 有
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

# 方法五 —— 多相機偵測 + LiDAR + 融合一起開 (stage 要有環境相機: kros_car_new.usd)
ros2 launch car_loc_fusion fusion_loc.launch.py evaluate:=true

# 方法六 —— 多相機偵測 + 融合 (不吃 LiDAR, 不需要地圖)
ros2 launch car_loc_multicam multicam_loc.launch.py evaluate:=true
```

`evaluate:=true` 會拿 Isaac 的 ground truth `/odom` 當尺即時報誤差, Ctrl-C 印總結;
逐點存檔加 `csv:=/workspaces/car_run_data/xxx.csv`。想邊定位邊開車就加
`teleop:=true`, 另開一個 shell 跑 `ros2 run car_teleop teleop_key` (`actuator_model` 也要開, 見第 1 節)。

多條同時跑的時候**只能留一條發 `map -> base_link`**, 其他的加 `publish_tf:=false`。

**方法五、六要注意的**:

* **8 台相機只能有一個偵測節點。** 兩個 launch 都會自己開一個 `multicam_detector`; 要同時開的話
  其中一個加 `detector:=false`。兩個 YOLO 搶同樣 8 台影像時, 2026-11-03 那兩輪 Isaac 的相機與
  LiDAR 一起斷了 (偵測節點發現有兩個時會一直印紅字)。
* **新容器第一次開, 偵測節點要先暖機 ~30 s** (log 印「YOLO 暖機中」, CUDA 在編 kernel)。這段時間
  沒有框、Ctrl-C 也收不掉, 不是當掉。
* **不需要給起點**: 第一組車框用車的 3D 外形反推位置, yaw 用 IMU 的。開始前讓車停一秒 (靜止校正)。
* 偵測疊圖加 `debug:=true` (`/env_cam_i/car_bbox_debug`); 每台相機的視線在 `/fusion_loc/markers` /
  `/multicam_loc/markers` (綠 = 採用, 紅 = 被閘門擋掉)。

### 六條一起跑, 跟 GT 記成同一份 CSV

📦
```bash
# 車子生在哪裡: ros2 topic echo /odom --once --field pose.pose.position
ros2 launch bringup_pkg collect_all.launch.py \
    imu_initial_pose:="[2.0, -0.3, 0.0]" csv_filename:=all_loc_0910.csv
ros2 launch car_teleop actuator_model.launch.py   # 另開 terminal, 輪子才會動
ros2 run car_teleop teleop_key                    # 另開 terminal 手動開車
```

`collect_all` 已經包含方法六 (`multicam_only`, 預設開, 跟方法五共用同一個偵測節點) ——
**不要再另外開 `multicam_loc.launch.py`**。

同名的 CSV 已經存在時, 舊的會先改名成 `<檔名>_<時間>.csv` 留著 (`_imu` / `_bbox` 一起改),
新的照樣寫在原本的名字。還是建議每一輪給不同的 `csv_filename`, 比較好找。

一列 = 一個時刻的七個答案 (`car_run_data/<csv_filename>`):

| 欄位 | 來源 |
| --- | --- |
| `car_position_x/y`, `gt_yaw` | `/odom` — Isaac ground truth |
| `cam_x/y/yaw` | `/camera_loc/odom` (方法一, 天花板相機) |
| `lid_x/y/yaw` | `/lidar_loc/odom` |
| `imu_x/y/yaw` | `/imu_loc/odom` |
| `whl_x/y/yaw` | `/wheel_loc/odom` |
| `fus_x/y/yaw` | `/fusion_loc/odom` (方法五, 吃 8 台相機的車框與 LiDAR) |
| `mc_x/y/yaw`, `mc_ncam` | `/multicam_loc/odom` (方法六, 同一份車框、不吃 LiDAR); `mc_ncam` = 最近 0.3 s 內有幾台相機框到車 |

每一組還配 `_stamp`、`_age`、`_sigma`:

- `_stamp` 是那個來源自己的時戳。同一列的各個值**不是同一個時刻**的
  (相機 ~15–30 Hz、LiDAR 10 Hz、IMU 200 Hz、記錄器 50 Hz), 算誤差前要先用它內插
  對齊。0.8 m/s 時 50 ms 的錯位就是 4 cm —— 跟 LiDAR 的真實誤差同一個量級。
- `_age` 是「這個值放多久了」。節點掛掉時最後一個值會被一路複製到檔尾, 看起來
  像車子停著不動, 統計出來的誤差是假的。
- `_sigma` 是節點自己回報的 1-sigma, 每一條的意義不一樣 (見 `collect_data_node.py`
  的說明)。LiDAR 用 `lid_sigma < 0.0025` 就能把 180° 對稱解的追丟幀全部擋掉。

另外兩個檔: `<檔名>_imu.csv` (IMU 全速率) 與 `<檔名>_bbox.csv` (8 台相機每一個原始框,
`car_loc_multicam/scripts/fit_car_model.py` 可以直接吃)。這兩個是 `collect_data_node` 的參數
(`imu_csv` / `bbox_csv`), `collect_all` 沒有開放成 launch 參數, 要關就直接 `ros2 run` 那個節點加 `-p`。

開關:

| 參數 | 意思 |
| --- | --- |
| `tf_source` | 誰發 `map -> base_link`: `fusion` (預設) \| `multicam` \| `lidar` \| `camera` \| `imu` \| `wheel` \| `none`。只蒐 CSV 就 `none` |
| `camera:=false` | 關掉天花板相機 (`cam_`)。**不影響融合** |
| `multicam:=false` | 關掉偵測節點, `fus_` 只剩 LiDAR + 遞推、`mc_` 沒有。stage 沒有環境相機時用它省 GPU |
| `multicam_only:=false` | 不開方法六 (`mc_`) |
| `lidar:=false` / `imu:=false` / `wheel:=false` / `fusion:=false` | 關掉那一條 |

**IMU 與輪速那兩條一定要給 `imu_initial_pose`**, 它們從那裡開始積分,
沒對到出生點的話整段差一個常數平移, 看起來像超大飄移 —— **方法五、六不用給**,
它們拿第一組車框 (沒有相機時是 LiDAR) 當起點。

`fus_` 跟 `lid_` / `mc_` **不獨立** (融合吃的就是同一份 LiDAR 與車框), 拿它們比是公平的
但不要當成獨立樣本。

### 分析 CSV (🖥, 不需要 ROS)

```bash
./scripts/eval_loc_csv.py car_run_data/all_loc_0910.csv --sweep     # 各條的誤差 + 過濾門檻掃描
./scripts/report_all_loc.py car_run_data/all_loc_0910.csv \
    --out car_run_data/report_0910                                  # 時間對齊誤差、隨時間變化、圖
```

### 精度 (Isaac 實測)

各列**不是同一輪量的**, 車也不一定一樣 (2026-10 之前是舊車 `car.usd`), 看量級就好:

| 方法 | 位置誤差 | 量的那一輪 | 誤差主要來自 |
| --- | --- | --- | --- |
| 方法一 天花板相機 | 中位 0.6–0.7 cm | 2026-09-11, 舊車, 摩擦力腳本 | **延遲** (~40–55 ms): 誤差跟車速成正比, 3 m/s 時 10–15 cm。補償後 < 1 cm。看不到車就沒有輸出 |
| 方法二 LiDAR | 中位 7.3 cm (σ 過濾後 7.1 cm, 靜止 6.8 cm) | 同上 | 自旋 **> 8 rad/s** (10 Hz 雷射的物理上限) 會追丟、可能鎖到 180° 對稱解; 鎖死偵測 (`lock_*`) 停下來後自己整張地圖重定位, 最長 2.7 s 回來。整輪追丟 2.9% |
| 方法三 純 IMU | 中位 0.7–1.2 m, 最後 3–7 m; kros_car 上中位 0.5 m | 同上; kros_car 是 2026-10-13 | 隨**走過的距離**發散, 大約距離的 5–13%。yaw 很準是因為模擬直接給了姿態 |
| 方法四 IMU+輪速 | 中位 15.3 cm; kros_car 上 37–45 cm | 同上 | 原地轉時車體會側滑, 輪速看不到 (左右輪速相反, 平均是 0); yaw 誤差 x 走過的距離 |
| 方法六 多相機 | RMS 2.11 cm, 中位 0.76 cm, 最大 6.4 cm | 2026-10-30, `kros_car_new.usd`, 繞房間 101 s, 冷啟動 | 剩下的誤差幾乎都在**慢速轉彎** (3–4 cm), 原因還沒找到 |
| 方法五 融合 | RMS 1.94 cm, 中位 1.40 cm, 最大 7.5 cm; 摩擦力腳本全程用它控制 RMS 1.77 cm | 2026-11-02, `kros_car_new.usd` | 平均誤差多相機已經很好, LiDAR 買到的是**中斷時的下限**: 重放時相機全斷 30 s, 那段 RMS 只有多相機 10.45 cm, 加 LiDAR 2.97 cm |

方法五、六那兩列是在 docker 裡另一份 Isaac 複本 (ROS domain 83) 上量的, 不是你平常開的那台;
自己跑一次 `evaluate:=true` 對一下。細節: [car_loc_multicam](src/car_loc_multicam/README.md)、
[car_loc_fusion](src/car_loc_fusion/README.md)。前四列的圖見 [report_all_sensors.md](report_all_sensors.md)
(更早的 09-10 那一輪)。

---

## 3. 手動開車建圖 (換場景或到實體環境時)

**📦 終端 1 —— 建圖** (裡面已經包含 `car_teleop` 的速度控制層; 致動器模型要另外開, 見第 1 節)

```bash
ros2 launch car_loc_lidar mapping.launch.py
ros2 launch car_teleop actuator_model.launch.py    # 另一個 shell
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

**🖥 校正 origin (建完圖一定要做)。** SLAM 地圖的原點與方向由建圖時的起點決定, 會跟
Isaac 的 `/odom` 差一個平移加一個小旋轉 —— 那不是定位在漂。用新地圖錄一輪 (車子開到
房間各處), 然後:

```bash
./scripts/calibrate_map_origin.py car_run_data/<run>.csv --write    # 擬合平移 + 旋轉, 寫進 room.yaml
```

再 `r` 一次。細節見 [src/car_loc_lidar/maps/README.md](src/car_loc_lidar/maps/README.md)。

---

## 4. 用 Foxglove 看

📦
```bash
ros2 launch car_viz viz.launch.py
```

它會把網址印出來 (`ws://<容器IP>:8765`, 沒裝 foxglove_bridge 時是 rosbridge 的 9090)。
Foxglove Studio -> Open connection -> 貼上。現成版面在
`src/car_viz/config/foxglove_layout.json` (Layout -> Import from file):
地圖 + 掃描 + GT + 五條位姿、天花板相機的 YOLO 畫面、Teleop、五條線的 x 對時間。
**版面還沒有方法六**: `/multicam_loc/pose`、`/multicam_loc/markers` / `/fusion_loc/markers`
(每台相機的視線) 與 `/env_cam_i/car_bbox_debug` (要 `debug:=true`) 要自己加面板。

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
cd src/car_loc_fusion && python3 test/test_all_sensors.py  # 約 1 分鐘
cd src/car_loc_multicam && python3 test/test_multicam.py   # 約 1 分鐘
```

拿錄好的資料重放真正的融合節點 (不需要 ROS / Isaac / YOLO, 要有那一輪的主 CSV、`_bbox.csv`、`_imu.csv`):
`src/car_loc_fusion/test/replay_all_sensors.py`, 用法見它的檔頭。改了融合的參數想知道效果,
用它比連續兩輪線上跑可靠 —— 兩輪之間偵測率可能差三倍, 那樣的 A/B 不公平。

---

## 6. 常用檢查指令

📦
```bash
# 地圖長什麼樣 (ASCII 俯視圖, 不需要 ROS)
python3 -m car_loc_lidar.gridmap show /workspaces/src/car_loc_lidar/maps/room.yaml

# LiDAR 重新做一次全域定位 (車子被搬走、或鎖到 180 度對稱解了)
ros2 service call /lidar_localizer/relocalize std_srvs/srv/Trigger

# 其他幾條重設 (camera / imu / wheel / fusion / multicam_fusion)
ros2 service call /fusion_localizer/reset std_srvs/srv/Trigger

# 看某一條的輸出
ros2 topic echo /fusion_loc/odom --field pose.pose.position

# 融合上一則被採用的絕對量測是幾秒前 (> 1 s = 相機和 LiDAR 都斷了, 只剩遞推)
ros2 topic echo /fusion_loc/abs_age

# 8 台相機各自有沒有在出框
ros2 topic hz /env_cam_0/car_bbox
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

融合 (方法五) 一樣要這三個, 再加 `sigma_acc:=0.35`:

```bash
ros2 launch car_loc_fusion fusion_loc.launch.py use_sim_time:=false \
    imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro sigma_acc:=0.35
```

**模擬專用、真車不要開的**: `car_teleop` 的 `actuator_model` (真車的馬達自己就有那些延遲)、
`car_imu_noise` (真車的 IMU 自己就有雜訊)、`collect_car_pkg`、`mlp_control_pkg` (吃 Isaac 的 `/odom`)。

**多相機 (方法五、六) 搬到實體相機要重做的**, 細節見
[car_loc_multicam 的「實體相機」](src/car_loc_multicam/README.md):

* `env_cameras.yaml` 現在是從 USD 匯出的 Isaac 針孔相機。實體相機要用棋盤格校正內參、魚眼鏡頭加
  `fisheye_k`, 外參用地上量過座標的標記點跑 `solvePnP` (`rvec` / `tvec`)。
* `best.onnx` 是用 Isaac 的畫面訓練的, 實景要重新標注訓練。
* 每台 Pi 自己跑 YOLO 的話照 `bbox_msg.py` 的格式發 `/env_cam_i/car_bbox` (`header.stamp` = 曝光時刻),
  這邊 `detector:=false` 只開融合。
* 真車沒有絕對 yaw, 單靠框反推起點可能顛倒 180 度 —— 讓 LiDAR 給起點 (方法五預設就有), 或給 `initial_pose`。

**地圖**: `room.pgm` 是在 Isaac 裡建的, 真的房間要照第 3 節重建 (`calibrate_map_origin.py` 要有 GT,
真車上要另外想辦法對齊原點)。**`evaluate:=true` 與 `eval_loc_csv.py` 都要 ground truth**, 真車沒有 `/odom`,
要量誤差得另外準備 (動作捕捉、地上量好的標記點)。

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

## kros_car.usd (實體車的模型, 2026-10 新增)

`kros_car.usd` 是實體車 (KROS car) 的模型, 由 `real/workspaces/src/kros_car_description/urdf/kros_car.urdf`
匯入 (sim 與實體車共用這一份 URDF)。房間、俯視相機跟 `car.usd` 完全一樣, 所以 `src/car_loc_lidar/maps/room.yaml` 可以沿用。
實體車那邊的程式與介面在 [`real/`](real/) (見它的 README)。
日常用的 `kros_car_new.usd` 是同一台車, 多了 8 台環境相機 (方法五、六的輸入)。

**程式的預設值已經是這台車的 (2026-10-12)**: 車頭軸、輪半徑、雷射外參、IMU 位置、
輪速控制。各 package README 裡如果還看到 effort / 扭矩控制、`car.usd` 的
數字 (車頭 -Y、輪半徑 0.075、自旋 8 rad/s ...), 那些都是舊車的。要回去跑舊的 `car.usd`
就照下面「參數對照」那張表把值給回去 —— 但 `control_car_node` / `cmd_vel_bridge`
現在只會送輪速, 舊車 (只收 effort) 開不動。

### 結構

| prim | 內容 |
| --- | --- |
| `/kros_car/base_link` | articulation root, 車體。**+X 是車頭 (手臂那側)、+Y 朝左**, 照 REP-103 |
| `/kros_car/base_link/laser_frame/kros_car_oradar_ms200` | MS200, 在 base_link 的 (+0.045, 0, 0.236) |
| `/kros_car/base_link/imu_link/Imu_Sensor_02` | IMU, 在 base_link 的 (+0.125, -0.0286, 0.110) |
| `/kros_car/base_link/ActionGraph_lidar` | `/scan` + `/clock` |
| `/kros_car/base_link/ActionGraph_imu` | `/imu` |
| `/kros_car/ActionGraph_odom` | `/odom` (ground truth)、`/joint_states`、`/joint_command` |
| `/kros_car/joints/{front,rear}_{left,right}_joint` | 輪子。正值往前; 速度驅動, 最大力矩 2.0 N·m、最高 200 rpm |

車體在 base_link 原點時輪子最低點是 z = -0.0416, 所以 `/kros_car` 要放在 z = 0.0416 才剛好貼地。

`scripts/setup_oradar_lidar.py` 的預設已經指到這份 USD 與上面的 prim。

### 參數對照 (預設值已經是右邊那一欄)

| 項目 | `car.usd` (舊) | `kros_car.usd` (現在的預設) | 在哪裡 |
| --- | --- | --- | --- |
| 車頭相對 base_link +X | -90 度 (車頭是 -Y) | 0 | `forward_deg`: `imu_loc.yaml`、`wheel_loc.yaml`、`fusion_loc.yaml` 與各自的 launch; `forward_axis_deg`: `control_car_node`、`tour_drive.py`、`estimate_friction.py` |
| 相機那條的 yaw 補償 | 90 度 | 0 | `yaw_offset_deg`: `car_loc_camera/config/camera_loc.yaml` |
| 輪半徑 | 0.075 | 0.05035 | `wheel_radius`: `wheel_loc.yaml`、`fusion_loc.yaml`、`cmd_vel_bridge`、`control_car_node` |
| 有效輪半徑比例 | 0.93 (effort 控制下量的) | 1.0 (2026-10-12 實跑量到 1.0005) | `wheel_scale`: `wheel_loc.yaml`、`fusion_loc.yaml` |
| 輪距 | 0.25 | 幾何 0.27, 轉向用的等效輪距 0.95 (實跑 0.945 / 1.055) | `effective_track`: `cmd_vel_bridge`、`control_car_node` |
| 雷射外參 | (0, 0, 0.20) | (0.045, 0, 0.236) | `lidar_translation`: `lidar_loc.yaml`、`lidar_odom.yaml`; static TF: 四個 launch 的 `lidar_x` / `lidar_z` |
| IMU 位置 | 旋轉中心 (0.6 mm) | (0.125, -0.0286), 離中心 12.8 cm | `imu_lever_arm`: `imu_loc.yaml`、`wheel_loc.yaml`、`fusion_loc.yaml`、`control_car_node` (節點會把加速度換算回 base_link 原點) |
| 控制介面 | `/joint_command` 的 `effort` | `/joint_command` 的 `velocity` (rad/s) | `control_car_node`、`cmd_vel_bridge` |
| 輪速上限 | — | 10 rad/s (= 實體車 max_rpm 100; USD 是 20.94) | `max_wheel_speed`: `control_car_node`、`cmd_vel_bridge`、`friction_test.launch.py` |
| 摩擦力測試的自旋輪速 | 8 rad/s | 6 rad/s, 指令量化成整數 rpm | `spin_wheel_speed`、`command_rpm_step`: `control_car_node`、`friction_test.launch.py` |
| 相機量到的點 | 車體中心 | bbox 中心在 base_link **前方 1.5 cm** (車頭有手臂) | `body_offset`: `camera_ground.yaml`; `camera_offset`: `fusion_loc.yaml` (兩個要一樣, 校正腳本會一起寫; 融合只在 `camera_enabled: true` 時用到) |
| 雷射一圈的時間模型 | 線性 reverse/end | Isaac 是 6 個算繪幀拼的階梯; 哪個扇區最新 (接縫) **每次按 Play 都可能不同**, 定位節點開機後在轉彎的掃描上自己校正 | `scan_time_model: auto`、`auto_scan_seam`、`scan_frames`、`scan_seam_deg`、`scan_lead_frames`: `lidar_loc.yaml`、`lidar_odom.yaml` (`use_sim_time:=false` 時自動用線性) |
| IMU 槓桿臂補償 | 不需要 (IMU 在中心) | 實體車要補 (0.125, -0.0286); **Isaac 的 IMU 不模擬安裝位置, 模擬裡自動不補** | `imu_lever_arm`、`imu_lever_arm_in_sim`: `imu_loc.yaml`、`wheel_loc.yaml`、`fusion_loc.yaml`、`control_car_node` |

**還沒處理的**:

* **車重。** `base_link` 的 `physics:mass` 目前是 5.0 kg (整車 12.5 kg), 那是為了讓車子轉得動
  先填的假設值, 要秤實車: 質量 = 整車重 - 7.51 kg, 慣量 = (0.1556, 0.2975, 0.3868) x 質量 / 37.56。
  自旋扭矩正比於「mu x 車重」, 車重錯多少, 校正出來的 mu 就反向錯多少。
* **`run_car.sh --friction-model physx` 用的舊 USD (`kros_car_05_sim` / `kros_car_10_real` /
  `kros_car_sim_calibrated`) 沒有 8 台環境相機**, 而且輪子吃 `/joint_command` (理想輪子)。預設的
  tire 模式用 `kros_car_sim.usd` / `kros_car_real.usd`, 沒有這個問題 (`friction_test.launch.py` 會開致動器模型)。
* **天花板相機 (方法一) 的 yaw 在原地轉時是錯的** (速度方向推的, 車體沒位移就停在舊值; 低速時誤差中位
  53~105 度)。要換 OBB 定向框模型才解得了; 融合已經不吃天花板相機, 不受影響。
* **多相機在慢速轉彎時誤差 3~4 cm**, 是方法五、六最大的誤差來源, 原因還沒找到 (側向滑動、相機台數
  都排除了)。另外車停著時多相機會固定偏車頭方向 ~1 cm, 現在用 `car_model.yaml` 的 `points_shift`
  (+1 cm) 補。
* **相機的單應性只做了局部修正** (比例 + 平移 + body_offset)。要整個重擬得先繞場錄一份
  涵蓋整個房間的資料 (`scripts/tour_drive.py`), 再跑
  `scripts/calibrate_camera_ground.py <CSV> --model homography --write`。
* **純 IMU / IMU+輪速這兩條在摩擦力測試裡本來就不準** (2026-10-13 全開那一輪, 走 47 m):
  IMU+輪速 中位 37~45 cm —— 原地轉時車體會平移 10~15 cm, 輪速看不到 (左右輪速相反,
  平均是 0); 純 IMU 中位 0.5 m、最後 25 m —— 輪速控制下等速直行時加速度與角速度都是 0,
  IMU 分不出「等速」跟「靜止」, 靜止偵測會誤判並把速度歸零 (實體車行進時有振動, 不會
  這麼嚴重)。這兩條只是對照組, 控制與分析用的是融合。
* 原因不明、先擱著的: 原地自旋時車體轉速忽快忽慢並且會平移 (4 秒 10~15 cm); 靜止時以
  約 0.75 mm/s 爬行。兩個都不影響摩擦力的主特徵。(爬行曾經讓融合的估計凍在原地, 2026-11-02 修掉了:
  靜止錨點現在會跟著絕對量測一起移。)

2026-10-13 在這台車上量過的三件事, 換硬體 / 換設定時要重量:

```bash
# 雷射一圈裡每個方位角是什麼時候打的 (只有模擬能量; 量到的是這一次 Play 的值,
# 拿來對照定位節點 log 裡「掃描接縫校正完成 ... 用 X°」選得對不對)
docker exec ros2_node bash -lc 'r && python3 /workspaces/scripts/check_scan_timing.py --spin'
# 相機的 bbox 中心偏移 + 比例 (先不加 --write 看數字)
python3 scripts/calibrate_camera_ground.py car_run_data/sim_data.csv --write
# 地圖原點對 ground truth
./scripts/calibrate_map_origin.py car_run_data/sim_data.csv --rotation yaw --write
```

### 控制方式與摩擦力校正

`/joint_command` 的 `velocity` 放四輪的 rad/s, 跟實體車的 `wheel_velocity_controller` 同一種訊息。
**指令要持續送**: 實體車 0.3 秒沒收到就停, Isaac 則會一直維持最後一筆 (所以節點結束前會補送 0)。

輪速控制下沒有「斷油滑行」, 舊的摩擦力特徵 (coast-down 的減速度、起轉 effort) 不存在。
現在的主特徵是**等速原地轉時的輪子扭矩** (正比於 mu), 細節見
[control_car_node.py](src/calibrate_env_pkg/calibrate_env_pkg/control_car_node.py) 與
[scripts/estimate_friction.py](scripts/estimate_friction.py) 的檔頭。

`run_car.sh` (2026-11-11 起) 預設用**自訂輪胎摩擦** ([scripts/isaac_tire_friction.py](scripts/isaac_tire_friction.py)):
sim = `kros_car_sim.usd` 從 μ 0.5 開始迭代, real = `kros_car_real.usd` 設 μ 1.0 (sim-to-sim 的真值; 這台車量得到的上限約 1.2, 2.0 會馬達飽和)。
Isaac 每次載入 USD 之後、Play 之前由 `load_isaac_usd.py` 執行那支腳本再設 μ; 收斂的 μ 寫到
`car_run_data/<run>/calibrated_tire.json` (自訂摩擦存不進 USD)。`--sim-tire-mu` / `--real-tire-mu` 換值,
`--friction-model physx` 回到舊的改地面材質流程。

**前提是馬達扭矩沒有飽和**, 而馬達出得了的扭矩隨轉速下降 (USD 的輪子關節現在有
DDSM115 的扭矩-轉速曲線: 堵轉 2.0 N·m, 115 rpm 剩 0.96, 200 rpm 是 0)。所以自旋用比較低的
輪速 (`spin_wheel_speed`, 預設 6 rad/s), 分析時飽和的那幾檔輪速會被拿掉 (兩邊拿掉同樣的)。

**B1 有時會掉進「低轉速狀態」**: 同樣的輪速指令與實際輪速, 車體只轉正常的 1/3~2/3, 扭矩低 ~9%
(2026-11-02/03 在 6 rad/s 那一檔 6 趟有 3~4 趟, 另一輪 0 趟, USD 沒改, 原因未知)。混進來比值會
偏低 2~3%, 所以 `estimate_friction.py` 預設排除這幾趟並在報告裡列出 (`--keep-low-spin` 留著)。
同一套設定跑兩次的重現性因此只有 ~1~3%。
無頭 PhysX 直接跑同一套指令的結果 (2026-10-13, 整車 12.5 kg, 輪子材質 0.5):

| 地面 mu | 自旋輪速 (rad/s) | 自旋的輪子扭矩 | 結果 |
| --- | --- | --- | --- |
| 0.5 | 2.4 / 4.2 / 6.0 | 0.566 / 0.587 / 0.593 N·m | 都沒飽和 |
| 1.0 | 2.4 / 4.2 / 6.0 | 0.853 / 0.889 / 0.943 | 最高檔飽和 -> 用前兩檔, 比值 1.50 (真值 1.50), 估出地面 1.00 |
| 1.0 | 4 / 7 / 10 | 0.881 / 0.923 / 0.969 | 後兩檔飽和 -> 只剩一檔, 比值 1.51 |

(舊的 45 kg 車重在地面 0.5 就轉不動 —— 自旋扭矩正比於車重, 車重要用實際值。)

**實體車的扭矩回授含馬達自己的損耗** (電流 x 扭矩常數; 規格書額定點 1.5 A x 0.75 = 1.13 N·m,
軸輸出 0.96)。B3 的等速直行就是量它的, 分析時會擬合 `loss(w) = c0 + c1*w` 從自旋扭矩裡
扣掉。用假資料驗過: 在模擬資料上加 0.12 + 0.008*w 的損耗, 擬合出 0.116 + 0.0083*w,
比值 1.01 (不扣是 1.28)。模擬自己的損耗是 0, 扣不扣沒差。

實體車收資料時 topic 不一樣 (`spin_wheel_speed` / `max_wheel_speed` 要跟模擬那一輪相同):

```bash
ros2 launch bringup_pkg friction_test.launch.py use_sim_time:=false \
    wheel_feedback_topic:=/wheel_feedback imu_topic:=/imu/data csv_filename:=real.csv
```

---

## 出事的時候

| 症狀 | 通常是 |
| --- | --- |
| 沒有任何 topic | Isaac 沒按 Play, 或 `ROS_DOMAIN_ID` 不是 82 |
| 有指令但車子完全不動 | `kros_car_new.usd` / `kros_car.usd` 的輪子吃 `/joint_command_isaac`, `actuator_model.launch.py` 沒開 |
| 融合 log 說只有 LiDAR, `mc_` 整欄 NaN | stage 沒有 8 台環境相機 (只有 `kros_car_new.usd` 有), 或偵測節點還在暖機 (新容器第一次 ~30 s) |
| 偵測節點一直印紅字說有兩個偵測節點 | 開了 `collect_all` / `friction_test` 又另外開 `multicam_loc.launch.py`。關掉後者, 或給它 `detector:=false` |
| 每台相機每秒才幾個框 | 影像訂閱被改成 BEST_EFFORT (`image_reliable: false`), 大張影像少一片就整張作廢。改回 RELIABLE; 跑久了掉到 ~3 Hz 的話加大 host 的 `net.core.rmem_max` |
| 車停著, 多相機的位置固定偏車頭方向 ~1 cm (LiDAR 沒有) | 車的外形對不上 YOLO 的框。調 `car_model.yaml` 的 `points_shift`, 量法寫在那個 yaml 裡 |
| 融合發 TF, 但明明給了 `tf_source:=none` | 舊版 launch 的參數疊法有問題 (yaml 蓋掉 launch 參數), 2026-11-03 修掉了。rebuild |
| LiDAR「找不到地圖」 | 還沒建圖, 或建完沒 `r`。也可以直接給 `map_path:=...` |
| LiDAR 位置還算合理但 yaw 差 180° | 自旋太快 (> 8 rad/s) 鎖到對稱解。看 `lid_sigma` (> 0.0025 就是)。車子停下來約 1.5 s 鎖死偵測會自己整張地圖重定位 (記錄印「疑似鎖在對稱解」); 一直沒回來才手動呼叫 `relocalize` |
| LiDAR 一轉彎就大量「配準失敗」 | `lidar_loc.yaml` 的 `auto_scan_stamp` 被打開、投錯了時序。保持 `false` + `reverse/end` |
| 掃描在 Foxglove 上是幾段斷開的弧 | USD 的 `fullScan` 沒開, 重跑 `./scripts/setup_oradar_lidar.py --usd <stage>.usd` |
| 誤差是一個不會變的常數 | 地圖原點跟 `/odom` 原點差一個平移, 不是在漂。跑 `./scripts/calibrate_map_origin.py` |
| LiDAR 在房間中央很準、開到兩端就不準 (yaw 誤差是固定角度) | 地圖轉歪了。同上, 腳本會連旋轉一起量, 寫成 `origin` 的 yaw |
| IMU / 輪速一開始就差很遠 | `imu_initial_pose` 沒對到出生點 |
| CSV 裡 `cam_stamp` 比 `/odom` 多了上千秒 | 時鐘基準不一樣 (`eval_loc_csv.py` 會警告)。分析時要先扣掉; **開融合之前要先查** |
| 車子按前進鍵一直加速 | `car_teleop` 的 bridge 沒起來, 或收不到 `/joint_states`+`/imu` 回授 |
| Foxglove 連不上 | `run_isaac_gui.sh` 沒做 port mapping, 用容器 IP 直連或加 `-p 8765:8765` |
