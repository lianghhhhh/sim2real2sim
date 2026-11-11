# 實車操作手冊：遙控、建圖、定位與摩擦力測試

這份手冊帶你在實體 KROS car 上跑 sim2real2sim 的程式：

- `car_teleop`：用鍵盤或 Foxglove 遙控車子
- `car_loc_lidar`：先用光達建地圖，再拿地圖定位
- `car_loc_imu`、`car_loc_wheel`、`car_loc_fusion`：IMU、IMU + 輪速、融合定位（第 8 節）
- `calibrate_env_pkg`、`bringup_pkg`：多條定位一起記錄、摩擦力測試（第 9、10 節）

**前提**：[REAL_CAR_TEST_REPORT.md](REAL_CAR_TEST_REPORT.md) 的測試都已經通過。車上現在有**輪子、光達和 IMU**，只缺**環境相機**，所以 `car_loc_camera`、`car_loc_multicam` 的相機那一路在實車上都不開。

---

## 1. 整體架構

全部的容器都在**車上電腦**跑，都用 host 網路和 `ROS_DOMAIN_ID=67`（設定在 `docker/compose/.env`）。

```
終端 A  start_all.sh         輪子驅動 (ros2_control)      收 /joint_command，發 /wheel_feedback、/joint_states、TF
                             MS200 驅動 + 濾波            發 /scan
                             IMU 驅動 (wildbot_imu)       發 /imu/data
                             foxglove_bridge              ws://<車上電腦 IP>:8765
終端 B  sim2real2sim.sh      跑 ../src 的程式             遙控、定位、摩擦力測試
```

指令從遙控端一路傳到輪子：

```
teleop_key 或 Foxglove ──/cmd_vel (Twist)──> cmd_vel_bridge ──/joint_command (四輪 rad/s)──> wheel_velocity_controller ──> 輪子
```

這裡有幾個跟模擬不一樣的地方：

1. **`src` 的程式改跑在車子的映像檔（`kros-car-new`，ROS 2 Jazzy）。** 模擬用的 `sim2real2sim:v1`（Humble）不在車上，而且它在 bridge 網路上，看不到車子的 topic。
2. **輪子預設的控制器是 `base_controller`，它不收 `/joint_command`。** 每次啟動之後，都要先切到 `wheel_velocity_controller`（見第 4 節）。
3. **topic 名稱不一樣**：IMU 是 `/imu/data`（模擬是 `/imu`），輪速和扭矩回授用 `/wheel_feedback`（模擬是 `/joint_states`）。
4. **實車沒有 `/clock`，也沒有 ground truth `/odom`。** 所有 launch 都要加 `use_sim_time:=false`，也都不要加 `evaluate:=true`。

---

## 2. 第一次的準備（做過就不用再做）

### 2-1 重建映像檔

改了 `real/workspaces/src` 或 `real/Dockerfile` 之後要重建一次（例如 `wildbot_calibration` 改成從 controller 讀參數之後）。重建時需要網路：

```bash
cd ~/sim2real2sim/real
./scripts/build_image.sh
```

### 2-2 決定地圖的名字

`src/car_loc_lidar/maps/room.yaml` 是 **Isaac 模擬房間**的地圖，實體場地不能用。實車的地圖請另外取名（下面都用 `lab`），不要蓋掉 `room`。

### 2-3 IMU 和輪子的校正

跑一次 `./scripts/calibrate.sh`（見測試報告第 9-3 節）。2026-11-09 的結果：

| 項目 | 結果 | 在這份手冊裡怎麼用 |
| --- | --- | --- |
| IMU 軸向 | 前後、左轉、抬頭四項都 PASS | x 朝車頭，`forward_deg` 維持 0 |
| `wheel_radius` | 0.0506 → 0.0511（11-04）→ **0.051356** m（已寫進 `docker/compose/configs/controllers.yaml`） | `src` 的節點（`cmd_vel_bridge`、定位、`control_car_node`）和 `kros_car.usd` 都改成 0.051356，`wheel_scale` 用預設的 1.0 |
| `wheel_separation_multiplier` | 2.254 → 2.197（11-04）→ **2.153867** | 只影響 `base_controller`。等效輪距 = 0.274 × 2.153867 = 0.590 m，拿來驗收模擬的轉彎（`scripts/measure_track.py`） |

重新校正、輪徑變了的話，要嘛把 `src` 節點的 `wheel_radius` 和 `kros_car.usd` 一起改，要嘛給 `wheel_scale:=新輪徑 / 0.051356`。

---

## 3. 每次都要做：開啟終端

都在 `~/sim2real2sim/real` 底下執行。

```bash
# 終端 A —— 輪子 + 光達 + IMU + Foxglove bridge，Ctrl+C 一起關掉
./scripts/start_all.sh

# 終端 B —— sim2real2sim 的 shell（第一次開會先 build）
./scripts/sim2real2sim.sh
```

只想開其中幾個，也可以分開用 `kros_car.sh`、`lidar.sh`、`imu.sh`、`foxglove_bridge.sh`。

**`sim2real2sim.sh` 的用法：**

- 第一次執行會開一個叫 `s2r_real` 的容器，build 實車用得到的所有 package，然後進入 shell。提示字元是 `(s2r_real) /s2r$`。
  - 預設 build 的是 `car_teleop`、`car_loc_lidar`、`car_loc_imu`、`car_loc_wheel`、`car_loc_fusion`、`car_loc_multicam`、`car_loc_camera`、`calibrate_env_pkg`、`bringup_pkg`。
  - 融合節點的程式寫在 `car_loc_multicam` 裡，`bringup_pkg` 又依賴 `car_loc_camera`，所以這兩個也要 build，但它們的相機節點在實車上不會開。
  - 只想 build 其中幾個的話，用 `PACKAGES="car_teleop car_loc_lidar" ./scripts/sim2real2sim.sh`。
- **之後再執行一次，會在同一個容器開一個新的 shell。** 下面需要多個 shell 時都這樣開。
- `../src` 掛在容器裡的 `/s2r/src`。改了程式碼之後打 `r` 重新 build。
- 第一個 shell 一離開，容器和 build 結果就沒了。`src` 裡的檔案（例如存好的地圖）會留下來。

**確認接上了**（在終端 B）：

```bash
ros2 topic hz /scan              # 約 14 Hz（實車驅動設 motor_speed 14，模擬是 10 Hz）
ros2 topic hz /wheel_feedback    # 約 50 Hz
ros2 topic hz /imu/data          # 測試報告第 9-2 節預期 20 Hz
```

三個都有數字才繼續。沒有的話見第 13 節。

---

## 4. 切換輪子控制器（每次重開 kros_car 後都要做）

在終端 B 執行：

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: [wheel_velocity_controller], deactivate_controllers: [base_controller], strictness: 2}"
```

確認：

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers \
  | grep -oE "name='[a-z_]+'|state='[a-z]+'" | paste - -
```

**預期**：`wheel_velocity_controller` 是 `active`，`base_controller` 是 `inactive`。

沒有切換的話，`/joint_command` 會被忽略，輪子不會動，log 也不會報錯。

> 跑 `calibrate.sh` 的輪子校正時要反過來，用 `base_controller`（它送的是 `/base_controller/cmd_vel`）。

---

## 5. 第一步：架高試遙控

**目的**：在輪子離地的狀態下，確認整條指令路徑和四輪方向都對。

**車子先架高、四輪離地。**

```bash
# 終端 B —— 速度換算層
ros2 launch car_teleop teleop.launch.py use_sim_time:=false

# 終端 C（再執行一次 ./scripts/sim2real2sim.sh）—— 鍵盤遙控
ros2 run car_teleop teleop_key
```

```
  w/s 前進後退   a/d 原地左右轉   空白 停
  +/- 速度上限   [/] 轉向上限   t 切換按住/持續   q 離開
```

| 動作 | 預期 | 實際 |
| --- | --- | --- |
| 按住 `w` | 四輪都往車頭方向轉，約 6.95 rad/s（0.35 m/s） | |
| 放開 `w` | 0.4 秒內停 | |
| 按住 `a` | 左輪往後、右輪往前 | |
| 關掉 `teleop_key` 或 `cmd_vel_bridge` | 0.3 到 0.5 秒內停 | |

用 `ros2 topic echo /wheel_feedback --field velocity` 可以看到實際輪速。

通過後再放到地上，在空曠的地方用低速開一小段。

- `cmd_vel_bridge` 的輪徑跟實體車一樣是 0.051356（2026-11-09），車速和指令一致。
- 預設 `yaw_trim_gain:=0`，不會用到 IMU。想讓它用陀螺儀修正轉速（給 nav2 這類程式用），就加 `imu_topic:=/imu/data yaw_trim_gain:=1.0`。

**注意**：`cmd_vel_bridge` 同時只能有一個在跑，不然兩個會搶著發 `/joint_command`。後面的 `mapping.launch.py`，以及加了 `teleop:=true` 的 launch，都已經自帶一個。跑它們之前要先把這裡的 `teleop.launch.py` 關掉。

---

## 6. 第二步：建地圖

### 6-1 開始建圖

把車放在場地裡**之後要當原點的位置**。地圖原點就是建圖開始時車子所在的位置。

```bash
# 終端 B —— 建圖（裡面已經帶 cmd_vel_bridge，不要再另外開 teleop.launch.py）
ros2 launch car_loc_lidar mapping.launch.py use_sim_time:=false

# 終端 C —— 鍵盤遙控
ros2 run car_teleop teleop_key
```

**預期 log**：

- `slam_toolbox`：依序出現 `Configuring`、`Activating`、`Registering sensor`
- `lidar_odometry`：每 3 秒一行，例如 `成功 100% ... 殘差 2.2 cm`

**開車的要點：**

- **慢慢開。** `teleop_key` 預設 0.35 m/s，建圖時建議按 `-` 降到 0.2 m/s 左右，轉向也放慢。開太快，掃描比對會跟不上。
- **每個角落、柱子後面都要繞到。** 沒繞到的地方在地圖上就是空的。
- **最後要繞回起點**，回環偵測才能把地圖閉合。
- MS200 實車的濾波設定只保留 **8 m 以內**的點（`real/docker/compose/launch/config/scan_filters.yaml`）。大場地要多繞幾趟。

### 6-2 在 Foxglove 上看建圖進度

在遠端電腦連 `ws://<車上電腦 IP>:8765`（見第 11 節），開一個 3D 面板：

- Fixed frame 選 `map`
- 勾選 `/map`、`/scan`、TF

地圖會一邊開一邊長出來。牆面出現重影，就表示掃描比對跟不上：放慢速度，或回到熟悉的地方讓它重新對上。

### 6-3 存地圖

**車子停下來之後**，開另一個 shell 執行：

```bash
ros2 run nav2_map_server map_saver_cli -f /s2r/src/car_loc_lidar/maps/lab
```

會產生 `src/car_loc_lidar/maps/lab.yaml` 和 `lab.pgm`。存完再按 Ctrl+C 關掉建圖。

用 ASCII 俯視圖檢查一下，不需要 ROS：

```bash
python3 -m car_loc_lidar.gridmap show /s2r/src/car_loc_lidar/maps/lab.yaml
```

> 容器是用 root 寫檔的。如果在 host 上要改這些檔案，先執行
> `sudo chown $USER: ~/sim2real2sim/src/car_loc_lidar/maps/lab.*`。

---

## 7. 第三步：用地圖做光達定位

```bash
# 終端 B —— 定位，加 teleop:=true 順便開速度換算層
ros2 launch car_loc_lidar lidar_loc.launch.py use_sim_time:=false teleop:=true \
    map_path:=/s2r/src/car_loc_lidar/maps/lab.yaml

# 終端 C —— 鍵盤遙控
ros2 run car_teleop teleop_key
```

### 7-1 怎麼判斷定位正常

看 `lidar_localizer` 每 2 秒印的那一行：

```
166 幀 (成功 93%, 掃角度重試 0), 配準 15.7 ms | x=... y=... yaw=... | 殘差 8.20 cm, inlier 71% | 定位中
```

| 指標 | 正常 | 不正常時代表什麼 |
| --- | --- | --- |
| 殘差 | < 2.5 cm | 掃描和地圖對不上：地圖品質不好，或環境跟建圖時不一樣 |
| inlier | > 85% | 同上 |
| `疑似鎖在對稱解` 警告 | 偶爾出現 | 一直重複出現，就是定位不穩 |

輸出的 topic 是 `/lidar_loc/odom`，TF 是 `map -> base_link`。

> **目前的實測**（2026-10-19，車子**架高、靜止**，用一張只建了 20 秒的地圖）：
> 殘差 7 到 8 cm、inlier 約 70%，每秒觸發一次「疑似鎖在對稱解」。
> 當時周圍物體大多在 1 m 以內，地圖只有 58 個被佔據的格子，所以這組數字**沒有參考價值**。
> 要在地上用第 6 節建好的完整地圖重新看一次。如果還是一樣，就要調整 `lidar_loc.yaml` 的門檻：那些門檻是在模擬裡調的，模擬的雷射沒有實體雷射的雜訊。

### 7-2 常用指令

```bash
# 車子被搬走、或定位跑掉了：整張地圖重新全域定位
ros2 service call /lidar_localizer/relocalize std_srvs/srv/Trigger

# 看目前位置
ros2 topic echo /lidar_loc/odom --field pose.pose.position

# 把這一趟錄下來（之後可以離線重播）
cd /s2r/car_run_data && ros2 bag record -o lidar_0101 /scan /wheel_feedback /imu/data /lidar_loc/odom /tf /tf_static
```

錄好的資料會存在 `~/sim2real2sim/car_run_data/`。

---

## 8. IMU、輪速與融合定位

### 8-1 先檢查 IMU

車子放在地上、靜止：

1. `ros2 topic echo /imu/data --once`：`linear_acceleration.z` 約 9.8，x 和 y 接近 0。
2. 用手把車頭往前推一下，`linear_acceleration.x` 應該先變正；原地左轉時，`angular_velocity.z` 應該是正的。這就是 `calibrate.sh` 的 `imu_axis_sanity` 那一步，已經 PASS 了。
3. IMU 的安裝位置要跟 URDF 的 `imu_link` 一致，也就是 base_link 的 (0.125, -0.0286, 0.11)。定位節點的 `imu_lever_arm` 用的就是這組數字，在 `use_sim_time:=false` 時會自動補償，不用另外設定。

### 8-2 實車要給的參數

實車 IMU 是 6 軸模式：roll 和 pitch 有加速度計可以校正，yaw 只是陀螺儀積分出來的，會漂。所以 IMU、輪速、融合這三條都要給：

| 參數 | 值 | 為什麼 |
| --- | --- | --- |
| `imu_topic` | `/imu/data` | 模擬是 `/imu` |
| `gravity_mode` | `complementary` | 改用陀螺儀積分，再在靜止時用加速度校正 |
| `yaw_source` | `gyro` | 沒有絕對 yaw |
| `joint_states_topic` | `/wheel_feedback` | `/wheel_feedback` 的時戳是 ESP32 送出的時刻，`/joint_states` 的時戳被控制迴圈量化成 20 ms |
| `wheel_scale` | 不用給（預設 1.0） | 節點的 `wheel_radius` 已經是實體輪徑 0.051356（見 2-3） |
| `sigma_acc`（只有融合） | `0.35` | complementary 模式下的加速度過程雜訊（`fusion_loc.yaml` 的註解） |

**開始之前先讓車子停幾秒**，節點要在這段時間量陀螺儀零偏。一啟動就開走，漂移會大很多。

### 8-3 三種定位

```bash
# 方法三 —— 純 IMU
ros2 launch car_loc_imu imu_loc.launch.py use_sim_time:=false \
    imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro

# 方法四 —— IMU + 輪速
ros2 launch car_loc_wheel wheel_loc.launch.py use_sim_time:=false \
    imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro \
    joint_states_topic:=/wheel_feedback

# 方法五 —— 融合：LiDAR + IMU + 輪速（沒有環境相機，所以 multicam:=false）
ros2 launch car_loc_fusion fusion_loc.launch.py use_sim_time:=false multicam:=false teleop:=true \
    imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro sigma_acc:=0.35 \
    joint_states_topic:=/wheel_feedback \
    map_path:=/s2r/src/car_loc_lidar/maps/lab.yaml
```

- 方法三、四是航位推算，從 `initial_pose`（預設 (0, 0, 0)）開始積分。車子要從地圖原點出發，不然整段會差一個固定的平移。
- 方法五用第一筆 LiDAR 定位當起點，不需要給起點。它自己會開一個 `lidar_localizer`，**不要再另外開第 7 節的 `lidar_loc.launch.py`**。
- 輸出的 topic 分別是 `/imu_loc/odom`、`/wheel_loc/odom`、`/fusion_loc/odom`。
- **同一時間只能有一個節點發 `map -> base_link`**：同時跑兩條以上時，其他條要加 `publish_tf:=false`。方法五已經自己把 LiDAR 那條的 TF 關掉了。

**建議的順序**：

1. 先單獨跑方法四，開一圈回到原點，看漂移了多少（模擬是走過距離的 0.76%）。
2. 再跑方法五，看 LiDAR 被擋住的時候，融合還能不能靠 IMU + 輪速撐住。

---

## 9. 多條定位一起記錄（`collect_all.launch.py`）

把 LiDAR、IMU、IMU + 輪速、融合四條同時跑，記進同一個 CSV，拿來比較每一條的定位品質。實車上要關掉所有相機那一路：

```bash
ros2 launch bringup_pkg collect_all.launch.py use_sim_time:=false \
    camera:=false multicam:=false multicam_only:=false \
    imu_topic:=/imu/data joint_states_topic:=/wheel_feedback \
    gravity_mode:=complementary yaw_source:=gyro fusion_sigma_acc:=0.35 \
    map_path:=/s2r/src/car_loc_lidar/maps/lab.yaml \
    output_dir:=/s2r/car_run_data csv_filename:=real_all_0101.csv

# 另一個 shell —— 鍵盤遙控（collect_all 已經帶 cmd_vel_bridge）
ros2 run car_teleop teleop_key
```

- 車子要從地圖原點出發，或者用 `imu_initial_pose:="[x, y, yaw度]"` 指定起點，給 `imu_` 和 `whl_` 那兩條用。
- 預設由融合發 TF（`tf_source:=fusion`）。只要記 CSV 的話，用 `tf_source:=none` 最乾淨。
- CSV 會存在 `~/sim2real2sim/car_run_data/`。`car_position_*`（GT）、`cam_*`、`mc_*` 這幾欄在實車上都是空的。

---

## 10. 摩擦力測試（`friction_test.launch.py`）

這個測試會**自動開車**，包括原地自旋、直線、急煞，靠融合定位和圍籬（geofence）防止車子撞牆。

### 10-1 跑之前

1. **先在模擬確認同樣的參數跑得完**。`spin_wheel_speed`、`max_wheel_speed` 兩邊要一樣。
2. **輪子健康檢查**：把車架高、停掉 `start_all.sh` 的輪子服務，跑 `python3 tools/wheel_noise_check.py --rpm 40 --seconds 30`。
   - 2026-11-05 的基準：其他三顆輪子的 `|tor|_mean` 約 405 到 470。第 2 顆（rear_left）有一個穩定的額外阻力，約 645，角度漣波約 550。
   - 第 2 顆的數值變化超過 10% 的話，就代表狀態變了，先檢查再收資料。
3. **方法五的融合要先在這個場地驗證過**（第 8 節），而且地圖要涵蓋整個場地。
4. **量出圍籬的範圍**：用 map 座標表示車子到得了的內牆範圍（m）。可以在 Foxglove 的 `/map` 上量，也可以用 `gridmap show` 看。
   - 預設是**模擬房間**（x ±4.58、y ±2.58），實體場地一定要改。
   - 節點會在這個範圍外再自動留 `wall_margin`（0.6 m）加上煞車距離的餘裕，所以填的是牆的位置，不用自己再扣。
5. 場地要比模擬寬敞，而且有人守在電源旁邊。

### 10-2 執行

先只跑第一段（`blocks:="[1]"`），確認沒問題再跑全部：

```bash
ros2 launch bringup_pkg friction_test.launch.py use_sim_time:=false multicam:=false \
    wheel_feedback_topic:=/wheel_feedback imu_topic:=/imu/data \
    gravity_mode:=complementary yaw_source:=gyro fusion_sigma_acc:=0.35 \
    map_path:=/s2r/src/car_loc_lidar/maps/lab.yaml \
    room_x_min:=-2.0 room_x_max:=2.0 room_y_min:=-1.5 room_y_max:=1.5 \
    blocks:="[1]" output_dir:=/s2r/car_run_data csv_filename:=real_0101.csv
```

（`room_*` 的數字只是範例，要換成 10-1 量到的值。）

- 輸出兩個檔：`real_0101.csv`（50 Hz）和 `real_0101_imu.csv`（IMU 全速率）。
- 跑完會自動結束。
- **這個 launch 會自己開 `cmd_vel_bridge` 以外的所有東西**。不要同時開 `teleop_key`，也不要開其他定位的 launch。

### 10-3 分析

不需要 ROS，在 host 上就可以跑：

```bash
cd ~/sim2real2sim
./scripts/estimate_friction.py car_run_data/sim_ref.csv car_run_data/real_0101.csv
```

- 實車一定要有 B3（直線段）：分析時會用它擬合馬達自己的損耗，再從自旋扭矩裡扣掉。**不要加 `--no-loss-correction`**，不然第 2 顆輪子的額外阻力會直接算進 mu。
- 細節見 [scripts/estimate_friction.py](../scripts/estimate_friction.py) 的檔頭。

> **還沒在實車上驗證過。** 第一次跑之前，先對照 `control_car_node.py` 開頭的說明再確認一次參數。

---

## 11. 從遠端電腦用 Foxglove 看

`start_all.sh` 裡的 `foxglove_bridge` 用 host 網路，port 8765 直接開在車上電腦。

1. 查車上電腦的 IP：`ip -4 addr`。2026-10-19 是 `192.168.75.9`（Wi-Fi），換網路就會變。
2. 遠端電腦開 Foxglove → **Open connection** → **Foxglove WebSocket** → 輸入 `ws://192.168.75.9:8765`。
3. 連不上（不在同一個網段，或被防火牆擋）就改走 SSH 轉發，再連 `ws://localhost:8765`：

   ```bash
   ssh -L 8765:localhost:8765 scream@192.168.75.9
   ```

**值得看的東西：**

| 面板 | 設定 | 看什麼 |
| --- | --- | --- |
| 3D | Fixed frame `map`；勾選 `/map`、`/scan`、TF | 建圖進度；定位時掃描有沒有貼齊牆 |
| Plot | `/wheel_feedback.velocity[0..3]` | 四輪實際輪速 |
| Plot | `/imu/data.angular_velocity.z` | 轉向時的角速度 |
| Raw Messages | `/lidar_loc/odom`、`/fusion_loc/odom` | 目前位置 |
| Teleop | topic 填 `/cmd_vel` | 可以取代 `teleop_key`，但一樣要有 `cmd_vel_bridge` 在跑 |

`src/car_viz/config/foxglove_layout.json` 是模擬用的版面，裡面的 GT 和相機在實車上都是空的。可以匯入之後只留上面這幾個面板。

---

## 12. 結束

1. `teleop_key` 按 `q` 離開，它會補發一次停車。
2. 終端 B 按 Ctrl+C 關掉 launch。
3. 終端 A 按 Ctrl+C，所有服務就會關掉。

---

## 13. 疑難排解

| 狀況 | 原因 | 對策 |
| --- | --- | --- |
| 終端 B 的 `ros2 topic list` 看不到 `/scan`、`/wheel_feedback`、`/imu/data` | 不是用 `sim2real2sim.sh` 開的容器（domain 或網路不對） | 用 `./scripts/sim2real2sim.sh`。自己開的話要加 `--network host --ipc host -e ROS_DOMAIN_ID=67` |
| 沒有 `/imu/data` | IMU 沒接好，或沒有 `/dev/imu` | `ls -l /dev/imu`；沒有的話見測試報告第 1 關的 udev 設定。USB 線要能傳資料 |
| 有 `/joint_command`，但輪子不動 | 控制器還是 `base_controller` | 做第 4 節 |
| 輪子一下動一下停 | 有兩個 `cmd_vel_bridge` 在跑 | `ros2 node list \| grep cmd_vel_bridge` 只能有一個 |
| 節點一直卡在「等 /scan」或「等 IMU」 | 忘了加 `use_sim_time:=false`，節點在等不存在的 `/clock`；或 `imu_topic` 還是 `/imu` | 每個 launch 都加 `use_sim_time:=false` 和第 8-2 節的參數 |
| IMU / 輪速定位一開始就漂很快 | 啟動後馬上開車，零偏沒量到 | 開始前讓車子停幾秒 |
| 融合只有遞推、沒有絕對位置 | 地圖路徑錯了，`lidar_localizer` 自己退出 | 確認 `map_path` 指到 `lab.yaml` |
| 建圖時 `/map` 一直沒出來 | `slam_toolbox` 卡在 unconfigured | `ros2 lifecycle get /slam_toolbox` 應該是 `active`。`mapping.launch.py` 已經會自動切換，見第 14 節 |
| `teleop_key` 按鍵沒反應 | 沒有 TTY | 一定要在 `sim2real2sim.sh` 開出來的 shell 裡跑（它有 `-it`） |
| 摩擦力測試一開始就 abort「離牆太近」 | 圍籬還是模擬房間的範圍 | 給 `room_x_min` 等四個參數（10-1） |
| `start_all.sh` 報 `no such file or directory` | 某個 USB 裝置沒接，或沒有固定名稱 | 照測試報告第 1 關設定 udev 名稱 |

---

## 14. 為了實車改了什麼

### 2026-11-05：IMU 裝上之後

| 檔案 | 改了什麼 | 為什麼 |
| --- | --- | --- |
| `real/scripts/sim2real2sim.sh` | 預設 build 的 package 從 `car_teleop car_loc_lidar`，擴大成 IMU、輪速、融合、摩擦力測試全部（`car_viz` 除外） | 有 IMU 之後這些都能在實車上跑 |
| `src/bringup_pkg/launch/collect_all.launch.py` | 新增 `imu_topic`、`joint_states_topic`、`gravity_mode`、`yaw_source`、`fusion_sigma_acc`、`wheel_scale`，傳給 `imu_`、`whl_`、`fus_`、`mc_` 和 `collect_data_node` | 原本這些參數只能改 yaml，launch 改不到，實車的 `/imu/data`、`/wheel_feedback` 傳不進去。預設值和 yaml 一樣，模擬不受影響 |
| `src/bringup_pkg/launch/friction_test.launch.py` | 把上面這些參數和 `map_path` 轉傳給 `collect_all`；新增 `room_x_min` 等四個圍籬參數 | 原本融合節點在實車上會去等 `/imu`，用的是模擬房間的地圖和圍籬 |

**已經驗證的**（2026-11-05，在 `kros-car-new` 映像檔裡）：

- 9 個 package 都 build 得過。
- 用實車參數展開 `friction_test.launch.py`：融合節點拿到 `/imu/data`、`/wheel_feedback`、`complementary`、`gyro`、`sigma_acc` 0.35、`wheel_scale` 1.015 和 `lab` 地圖；偵測節點和方法六沒有開；`calibrate_env_node` 拿到 IMU topic 和圍籬範圍。
- 用預設參數展開 `collect_all.launch.py`：每個節點的值都跟原本 yaml 裡的一樣。

**還沒驗證的**：第 8 到 10 節的指令都還沒接上實車的感測器跑過。

### 2026-10-19：光達與遙控

| 檔案 | 改了什麼 | 為什麼 |
| --- | --- | --- |
| `src/car_loc_lidar/launch/mapping.launch.py` | ROS 不是 Humble 時，`slam_toolbox` 改成以 lifecycle 節點啟動，並自動送 configure 和 activate | Jazzy 的 `slam_toolbox` 2.8 是 lifecycle 節點，不切換就停在 unconfigured、不建圖。Humble（模擬）照舊 |
| `src/car_loc_lidar/package.xml` | 加 `lifecycle_msgs` | 同上 |
| `real/Dockerfile` | 裝 `python3-scipy` | `lidar_localizer` 用它算距離場 |
| `real/scripts/sim2real2sim.sh` | 新增 | 用車子的映像檔、host 網路、domain 67 開 shell，並 build `../src` |

**已經驗證的**（2026-10-19，車子架高、靜止）：

- 容器裡收得到 `/scan`（14 Hz，每圈約 324 點）和 `/wheel_feedback`。
- `mapping.launch.py`：`slam_toolbox` 會進到 `active` 並發出 `/map`，光達里程計 100% 配準成功。
- `map_saver_cli` 存得了地圖，`lidar_loc.launch.py` 載得進地圖並發出 `/lidar_loc/odom`。
- `cmd_vel_bridge` 以 50 Hz 發 `/joint_command`。

**還沒驗證的**：輪子實際跟著 `/cmd_vel` 轉（第 5 節）、在地上建一張完整的地圖（第 6 節）、地上的定位精度（第 7 節）。
