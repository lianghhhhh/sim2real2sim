# 實車操作手冊：遙控（car_teleop）與光達定位（car_loc_lidar）

這份手冊帶你在實體 KROS car 上跑 sim2real2sim 的兩個 package：

- `car_teleop`：用鍵盤或 Foxglove 遙控車子
- `car_loc_lidar`：先用光達建地圖，再拿地圖定位

最後一節是其他 package 在實車上的用法和限制。

**前提**：[REAL_CAR_TEST_REPORT.md](REAL_CAR_TEST_REPORT.md) 的測試都已經通過。車上目前**只有輪子和光達，沒有 IMU**。

---

## 1. 整體架構

全部的容器都在**車上電腦**跑，都用 host 網路和 `ROS_DOMAIN_ID=67`（設定在 `docker/compose/.env`）。

```
終端 A  kros_car.sh          輪子驅動 (ros2_control)      收 /joint_command，發 /wheel_feedback、/joint_states、TF
終端 B  lidar.sh             MS200 驅動 + 濾波            發 /scan
終端 C  foxglove_bridge.sh   給遠端 Foxglove 連           ws://<車上電腦 IP>:8765
終端 D  sim2real2sim.sh      跑 ../src 的程式             car_teleop、car_loc_lidar
```

指令從遙控端一路傳到輪子：

```
teleop_key 或 Foxglove ──/cmd_vel (Twist)──> cmd_vel_bridge ──/joint_command (四輪 rad/s)──> wheel_velocity_controller ──> 輪子
```

這裡有兩個跟模擬不一樣的地方：

1. **`src` 的程式改跑在車子的映像檔（`kros-car-new`，ROS 2 Jazzy）。** 模擬用的 `sim2real2sim:v1`（Humble）不在車上，而且它在 bridge 網路上，看不到車子的 topic。
2. **輪子預設的控制器是 `base_controller`，它不收 `/joint_command`。** 每次啟動 `kros_car` 之後，都要先切到 `wheel_velocity_controller`（見第 4 節）。

---

## 2. 第一次的準備（做過就不用再做）

### 2-1 重建映像檔

`real/Dockerfile` 這次加裝了 `python3-scipy`，光達定位算距離場會用到。重建時需要網路：

```bash
cd ~/sim2real2sim/real
./scripts/build_image.sh
```

> 2026-10-19 已經在這台電腦上重建過了。改了 `real/workspaces/src` 或 `real/Dockerfile` 之後要再重建一次。

### 2-2 決定地圖的名字

`src/car_loc_lidar/maps/room.yaml` 是 **Isaac 模擬房間**的地圖，實體場地不能用。實車的地圖請另外取名（下面都用 `lab`），不要蓋掉 `room`。

---

## 3. 每次都要做：開啟四個終端

都在 `~/sim2real2sim/real` 底下執行。A、B、C 三個會佔住前景，按 Ctrl+C 就會關掉。

```bash
# 終端 A —— 輪子
./scripts/kros_car.sh

# 終端 B —— 光達
./scripts/lidar.sh

# 終端 C —— Foxglove bridge（要從遠端看才需要開）
./scripts/foxglove_bridge.sh

# 終端 D —— sim2real2sim 的 shell（第一次開會先 build，大約 10 秒）
./scripts/sim2real2sim.sh
```

不要用 `start_all.sh`：它會一起開 IMU，沒有 `/dev/imu` 會啟動失敗。

**`sim2real2sim.sh` 的用法：**

- 第一次執行會開一個叫 `s2r_real` 的容器，build `car_teleop` 和 `car_loc_lidar`，然後進入 shell。提示字元是 `(s2r_real) /s2r$`。
- **之後再執行一次，會在同一個容器開一個新的 shell。** 下面需要多個 shell 時都這樣開。
- `../src` 掛在容器裡的 `/s2r/src`。改了程式碼之後打 `r` 重新 build。
- 第一個 shell 一離開，容器和 build 結果就沒了。`src` 裡的檔案（例如存好的地圖）會留下來。

**確認接上了**（在終端 D）：

```bash
ros2 topic hz /scan              # 約 14 Hz（實車驅動設 motor_speed 14，模擬是 10 Hz）
ros2 topic hz /wheel_feedback    # 約 50 Hz
```

兩個都有數字才繼續。沒有的話見第 10 節。

---

## 4. 切換輪子控制器（每次重開 kros_car 後都要做）

在終端 D 執行：

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

---

## 5. 第一步：架高試遙控

**目的**：在輪子離地的狀態下，確認整條指令路徑和四輪方向都對。

**車子先架高、四輪離地。**

```bash
# 終端 D —— 速度換算層
ros2 launch car_teleop teleop.launch.py use_sim_time:=false

# 終端 E（再執行一次 ./scripts/sim2real2sim.sh）—— 鍵盤遙控
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

**注意**：`cmd_vel_bridge` 同時只能有一個在跑，不然兩個會搶著發 `/joint_command`。第 6 節的 `mapping.launch.py` 已經自帶一個，跑它之前要先把這裡的 `teleop.launch.py` 關掉。

---

## 6. 第二步：建地圖

### 6-1 開始建圖

把車放在場地裡**之後要當原點的位置**。地圖原點就是建圖開始時車子所在的位置。

```bash
# 終端 D —— 建圖（裡面已經帶 cmd_vel_bridge，不要再另外開 teleop.launch.py）
ros2 launch car_loc_lidar mapping.launch.py use_sim_time:=false

# 終端 E —— 鍵盤遙控
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

在遠端電腦連 `ws://<車上電腦 IP>:8765`（見第 8 節），開一個 3D 面板：

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

## 7. 第三步：用地圖定位

```bash
# 終端 D —— 定位，加 teleop:=true 順便開速度換算層
ros2 launch car_loc_lidar lidar_loc.launch.py use_sim_time:=false teleop:=true \
    map_path:=/s2r/src/car_loc_lidar/maps/lab.yaml

# 終端 E —— 鍵盤遙控
ros2 run car_teleop teleop_key
```

**不要加 `evaluate:=true`。** 它拿 Isaac 的 ground truth `/odom` 當答案，實車上沒有這個 topic。

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
cd /s2r/car_run_data && ros2 bag record -o lidar_0101 /scan /wheel_feedback /lidar_loc/odom /tf /tf_static
```

錄好的資料會存在 `~/sim2real2sim/car_run_data/`。launch 的 `csv:=` 參數不要用：它只有 `evaluate:=true` 的評估節點會寫，而評估需要 GT。

---

## 8. 從遠端電腦用 Foxglove 看

終端 C 的 `foxglove_bridge` 用 host 網路，port 8765 直接開在車上電腦。

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
| Raw Messages | `/lidar_loc/odom` | 目前位置 |
| Teleop | topic 填 `/cmd_vel` | 可以取代 `teleop_key`，但一樣要有 `cmd_vel_bridge` 在跑 |

`src/car_viz/config/foxglove_layout.json` 是模擬用的版面，裡面的 GT、相機、其他四條定位在實車上都是空的。可以匯入之後只留上面這幾個面板。

---

## 9. 結束

1. `teleop_key` 按 `q` 離開，它會補發一次停車。
2. 終端 D 按 Ctrl+C 關掉 launch。
3. 終端 A、B、C 按 Ctrl+C，對應的服務就會關掉。

---

## 10. 疑難排解

| 狀況 | 原因 | 對策 |
| --- | --- | --- |
| 終端 D 的 `ros2 topic list` 看不到 `/scan`、`/wheel_feedback` | 不是用 `sim2real2sim.sh` 開的容器（domain 或網路不對） | 用 `./scripts/sim2real2sim.sh`。自己開的話要加 `--network host --ipc host -e ROS_DOMAIN_ID=67` |
| 有 `/joint_command`，但輪子不動 | 控制器還是 `base_controller` | 做第 4 節 |
| 輪子一下動一下停 | 有兩個 `cmd_vel_bridge` 在跑 | `ros2 node list \| grep cmd_vel_bridge` 只能有一個 |
| 節點一直卡在「等 /scan」 | 忘了加 `use_sim_time:=false`，節點在等不存在的 `/clock` | 每個 launch 都加 `use_sim_time:=false` |
| 建圖時 `/map` 一直沒出來 | `slam_toolbox` 卡在 unconfigured | `ros2 lifecycle get /slam_toolbox` 應該是 `active`。`mapping.launch.py` 已經會自動切換，見第 12 節 |
| `teleop_key` 按鍵沒反應 | 沒有 TTY | 一定要在 `sim2real2sim.sh` 開出來的 shell 裡跑（它有 `-it`） |
| `sim2real2sim.sh` 報 `scipy` 錯誤 | 映像檔還是舊的 | 做 2-1 |
| `kros_car.sh` 報 `no such file or directory` | 某個 USB 裝置沒接，或沒有固定名稱 | 照測試報告第 1 關設定 udev 名稱 |

---

## 11. 其他程式碼在真車上的用法

都在 `sim2real2sim.sh` 的 shell 裡跑。要先把 package 加進 build 清單，例如：

```bash
PACKAGES="car_teleop car_loc_lidar car_loc_wheel" ./scripts/sim2real2sim.sh
```

這個設定只在第一次開容器時有效。容器已經在跑的話，先把第一個 shell 關掉再重開。

**所有 launch 都要加 `use_sim_time:=false`，也都不要加 `evaluate:=true`**：實車沒有 `/clock`，也沒有 ground truth `/odom`。

### 11-1 總表

| package | 現在能跑嗎 | 缺什麼 |
| --- | --- | --- |
| `car_teleop` | ✅ 可以 | — |
| `car_loc_lidar` | ✅ 可以 | 實體場地的地圖（第 6 節） |
| `car_viz` | ⚠️ 不需要 | 車上的 `foxglove_bridge.sh` 已經提供同樣的功能。`car_viz` 的影像壓縮是給 Isaac 的 `/rgb` 用的 |
| `car_loc_imu` | ❌ | IMU |
| `car_loc_wheel` | ❌ | IMU（yaw 從陀螺儀來） |
| `car_loc_fusion` | ❌ | IMU；相機那一路還缺俯視相機 |
| `car_loc_camera` | ❌ | 實體的俯視相機和它的 `/rgb`、地面單應性校正。映像檔也還沒裝 `onnxruntime` |
| `calibrate_env_pkg`（`control_car_node`，摩擦力測試） | ❌ | `/imu` 的陀螺儀，以及 `/fusion_loc/odom` 給的絕對位置 |
| `calibrate_env_pkg`（`collect_data_node`） | ⚠️ 部分可以 | 沒有 GT，`car_position_*` 那幾欄會是空的 |
| `bringup_pkg`（`collect_all`、`friction_test`） | ❌ | 同上 |

### 11-2 IMU 裝上之後

先在 `real/` 底下用 `./scripts/imu.sh` 開 IMU，或改回 `./scripts/start_all.sh`。實車 IMU 的 topic 是 **`/imu/data`**，而且是 6 軸、**沒有 orientation**，所以下面三個參數都要給：

| 參數 | 值 | 為什麼 |
| --- | --- | --- |
| `imu_topic` | `/imu/data` | 模擬是 `/imu` |
| `gravity_mode` | `complementary` | 沒有 orientation，改用陀螺儀積分，再在靜止時用加速度校正 |
| `yaw_source` | `gyro` | 沒有絕對 yaw |

```bash
# 方法三 —— 純 IMU（開始前讓車停幾秒，做靜止校正）
ros2 launch car_loc_imu imu_loc.launch.py use_sim_time:=false \
    imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro

# 方法四 —— IMU + 輪速（停 1 秒就夠）
ros2 launch car_loc_wheel wheel_loc.launch.py use_sim_time:=false \
    imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro \
    joint_states_topic:=/wheel_feedback

# 方法五 —— 融合（沒有俯視相機就關掉 camera）
ros2 launch car_loc_fusion fusion_loc.launch.py use_sim_time:=false camera:=false \
    imu_topic:=/imu/data gravity_mode:=complementary yaw_source:=gyro \
    joint_states_topic:=/wheel_feedback map_path:=/s2r/src/car_loc_lidar/maps/lab.yaml
```

- `joint_states_topic:=/wheel_feedback`：`/wheel_feedback` 的時戳是 ESP32 送出的時刻，`/joint_states` 的時戳被控制迴圈量化成 20 ms。訊息格式一樣，所以前者比較準。
- IMU 的安裝位置 `imu_lever_arm`（0.125, -0.0286）在 `use_sim_time:=false` 時會自動補償，不用另外設定。
- **同一時間只能有一條發 `map -> base_link`**：同時跑兩條以上時，其他條要加 `publish_tf:=false`。

IMU 裝上後要做的檢查：

1. 車子靜止時，`ros2 topic echo /imu/data` 的加速度大小約 9.8，z 軸朝上。
2. IMU 的安裝座標和 URDF 的 `imu_link`（base_link 的 (0.125, -0.0286, 0.11)）一致。
3. 先單獨跑方法四，開一圈回到原點，看漂移多少（模擬是走過距離的 0.76%）。

### 11-3 摩擦力測試（`friction_test.launch.py`）

要等 IMU 裝上、融合定位穩定之後才能跑。它會**自動開車**，包括自旋和直線，而且有 geofence。實車第一次跑前請務必：

1. 先在模擬確認同樣的參數跑得完。
2. 實車場地要比模擬寬敞，並且有人守在電源旁邊。
3. 先用 `blocks:="[1]"` 只跑第一段。

```bash
ros2 launch bringup_pkg friction_test.launch.py use_sim_time:=false state_source:=sensor \
    imu_topic:=/imu/data camera:=false output_dir:=/s2r/car_run_data csv_filename:=real_0101.csv
```

這條指令還沒在實車上驗證過。實際跑之前，先對照 `control_car_node.py` 開頭的說明確認一次參數。

---

## 12. 這次為了實車改了什麼

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
