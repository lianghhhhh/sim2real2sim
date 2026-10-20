# kros_car_new

實體車（KROS car）給 sim2real2sim 收資料、跑 MPC 用的全部東西：韌體、ROS 套件、啟動設定。
車輪用馬達內建的**速度環**控制，並把馬達回報的輪速、位置、電流讀回來。

## 資料夾內容

| 路徑 | 內容 |
| --- | --- |
| `wildbot_wheelctrl/` | ESP32 韌體（輪子的 RS-485 控制器）。協定見裡面的 README |
| `workspaces/src/kros_car/` | ros2_control 硬體介面 + 輪速控制器。細節見裡面的 README |
| `workspaces/src/kros_car_description/` | URDF（`front_left_joint` 命名，四輪加了 effort 回授；`base_link` 的 +x 朝車頭、+y 朝左） |
| `workspaces/src/wildbot_imu/` | IMU 驅動，發 `/imu/data` |
| `workspaces/src/oradar_ros/` | MS200 光達驅動 |
| `workspaces/src/wildbot_calibration/` | 輪徑、輪距、IMU 校正精靈 |
| `docker/compose/` | 各服務的 compose 檔、`configs/`（控制器與硬體參數）、光達 launch |
| `scripts/` | 建映像檔、啟動、切換控制器、校正 |
| `tools/` | USB 裝置固定名稱（udev）工具。git submodule，來自 [screamlab/tools](https://github.com/screamlab/tools)，clone 後要 `git submodule update --init` 才會有內容 |
| `Dockerfile` | 以車上原本的 driver 映像檔為底，重建 `workspaces/src` 的套件 |

`wildbot_imu`、`oradar_ros`、`wildbot_calibration` 是從車上的映像檔
`kros-wildbot-driver-image:latest` 取出來的原始碼，內容沒有改。
`laser_filters`、`foxglove_bridge`、ros2_control 等 apt 套件仍然來自那個映像檔。

## 上車步驟

**韌體和映像檔要一起換。** 新舊之間的 baud rate（460800 對 115200）和指令格式都不同，
只換一邊輪子不會動。

1. 指定usb名稱\
      USB 設定名稱:
      - LiDAR: oradar
      - 車輪: usb_wheel

   ```bash
   git submodule update --init tools   # 第一次才需要
   cd tools
   sudo ./create_usb_rule.sh </dev/ttyUSB0> <usb_custom_name>
   ```
2. 燒韌體：

   ```bash
   cd wildbot_wheelctrl && ./flash_esp32.sh        # 預設 PORT=/dev/usb_wheel
   ```

3. 建映像檔（改了 `workspaces/src` 之後也要重建）：

   ```bash
   ./scripts/build_image.sh                        # 產生 kros-car-new:latest
   ```

4. **第一次啟動先把車架高、輪子離地**，照下面「還沒在實車上確認的事」逐項檢查。
5. 啟動：

   ```bash
   ./scripts/start_all.sh          # 輪子、光達、IMU、Foxglove
   ./scripts/kros_car.sh           # 或只開輪子
   ```

`start_all.sh` 不開 EKF：它會佔用 `/odom`，而 sim2real2sim 把 `/odom` 當 ground truth。

## 給 sim2real2sim 的介面

`ROS_DOMAIN_ID=67`（`docker/compose/.env`）。模擬用 82，**刻意不一樣**：兩邊的 topic 同名
（`/joint_command`、`/scan`…），同一個 domain 的話，給模擬車的指令會直接驅動實體車。
要連實體車時，把 sim2real2sim 容器的 `ROS_DOMAIN_ID` 設成 67。

| Topic | 型別 | 說明 |
| --- | --- | --- |
| `/joint_command` | `sensor_msgs/JointState` | **輪速指令**，`velocity` 放 rad/s，用關節名稱對應。要持續送，停 0.3 秒輪子就停 |
| `/base_controller/cmd_vel` | `geometry_msgs/TwistStamped` | 速度指令（遙控、建圖、校正用） |
| `/wheel_feedback` | `sensor_msgs/JointState` | 四輪的位置 rad、輪速 rad/s、扭矩 N·m，50 Hz，時戳是 ESP32 送出的時刻。**記錄資料用這個** |
| `/joint_states` | `sensor_msgs/JointState` | 同樣的值，但時戳被控制迴圈量化成 20 ms |
| `/base_controller/odom` | `nav_msgs/Odometry` | 由量到的輪子位置算的里程計 |
| `/scan` | `sensor_msgs/LaserScan` | MS200，經過 `scan_filters.yaml` |
| `/imu/data` | `sensor_msgs/Imu` | 6 軸 |

關節名稱：`front_left_joint`、`rear_left_joint`、`front_right_joint`、`rear_right_joint`，
四輪都是正值往前。

兩個輪子控制器同時只能有一個在用，預設是 `cmd_vel`：

```bash
./scripts/wheel_controller.sh joint_command     # 收資料、MPC
./scripts/wheel_controller.sh cmd_vel           # 遙控、校正
```

想一開機就用輪速指令，把 `docker-compose_kros_car.yml` 的 command 改成
`ros2 launch kros_car bringup.launch.py wheel_controller:=wheel_velocity_controller`。

換算：

- 馬達電流 A = `/wheel_feedback` 的 effort ÷ `wheel_torque_constant`（預設 0.75）
- 輪速解析度 1 rpm = 0.105 rad/s ≈ 5 mm/s。編碼器是 4096 格，50 Hz 下用位置差分並不會更細，
  要更細只能對較長的時間窗差分

## 車體參數（模擬端要用同一組）

這些值在 URDF 和 `kros_car.usd` 裡是一致的。sim2real2sim 各 package 的預設值目前還是舊的
`car.usd` 那台小車的，用這台車時要改，對照表在 sim2real2sim 的 README。

| 項目 | 值 | 出處 |
| --- | --- | --- |
| `base_link` 方向 | +x 車頭（手臂側）、+y 左、+z 上 | URDF |
| 輪半徑 | 0.05035 m | URDF 碰撞圓柱（`controllers.yaml` 校正值是 0.0506） |
| 左右輪距（輪胎中心） | 0.27 m | URDF；`controllers.yaml` 用 0.274，轉向時再乘 2.254 修正打滑 |
| 前後輪距 | 0.289 m | URDF |
| 輪子最大扭矩 / 轉速 | 2.0 N·m / 200 rpm（20.94 rad/s） | DDSM115 規格；實體車另外被 `max_rpm: 100` 限制 |
| 光達 `laser_frame` | (+0.045, 0, 0.236) | URDF，朝向同 `base_link` |
| IMU `imu_link` | (+0.125, −0.0286, 0.110) | URDF，朝向同 `base_link` |
| `base_link` 離地 | 0.0416 m | 輪心高 0.0087 m − 輪半徑 |

## 和車上原本的版本差在哪

| 項目 | 原本 | 現在 |
| --- | --- | --- |
| `/joint_states` 的輪子 | 把指令當成量測值 | 馬達回報的位置、輪速、電流 |
| 輪子指令 | 主機一顆一顆送，每輪 25 Hz | 主機一次送四輪，ESP32 以 50 Hz 輪詢馬達並一次回報 |
| heartbeat | 關閉，主機當掉輪子繼續轉 | 預設 500 ms，逾時輪子停 |
| 輪速指令介面 | 只有 `cmd_vel` | 多一個 `/joint_command`，和模擬同一種訊息 |
| 回授遺失、馬達模式不對 | 不知道 | 四輪停下並記錄原因 |
| USB baud | 115200 | 460800 |
| 控制迴圈 | 20 Hz | 50 Hz |
| 手臂和夾爪 | 由 `/arm_controller` 控制 | 移除，URDF 裡固定不動 |
| 關節名稱 | `wheel_1_joint`… | `front_left_joint`… |
| `calibrate.sh` | 要求一個 compose 沒建立的 docker network | 改用 host network，和其他服務一致 |

沒有帶過來的：比賽任務程式、YOLO 模型、導航 / SLAM / EKF、RealSense 與 Orbbec 相機。
原本的資料夾都沒有動。

## 驗證做到哪裡

沒有接實體車。做過的是：

- 硬體介面和控制器在車上的映像檔裡編譯通過，單元測試 13 項通過。
- 韌體以 ESP32 為目標用 arduino-cli 編譯通過。
- 把韌體的原始碼原封不動接上模擬的 RS-485 匯流排和四顆模擬馬達，和硬體介面、控制器在容器裡對跑：
  前進、原地轉、兩種指令介面、指令逾時、馬達不回應、馬達模式被改掉、主機凍結觸發 heartbeat、
  手臂軌跡，20 項檢查通過，匯流排沒有碰撞或 CRC 錯誤。

模擬馬達是我寫的模型，只能證明兩邊的程式邏輯和協定對得上，不能證明對真的 DDSM115 也成立。

## 還沒在實車上確認的事

架高、輪子離地時逐項看：

1. **回授的正負號。** 位置和電流的正負號我假設和輪速一致。往前轉時，`/wheel_feedback` 四輪的
   velocity 和 position 都應該往正的方向走；加速時 effort 為正。不對的話要改
   `kros_car_system_threads.cpp` 的 `wheel_handle_line()`。
2. **速度環的 mode 值。** 我假設馬達在速度環時回報 `mode` = 2。如果不是，log 會一直出現
   `is in mode N, switching it to speed loop` 而且輪子不動；先在 `hardware.yaml` 設
   `wheel_require_feedback: false` 確認實際的值。
3. **460800 baud。** 這塊 ESP32 板子的 USB 晶片要能跑這個速度。log 出現
   `Wheel feedback problem (no fresh feedback ...)` 且一直不恢復，就把韌體的
   `SERIAL_BAUDRATE` 和 `hardware.yaml` 的 `wheel_baudrate` 一起降回 115200（流量約 7 kB/s，仍然夠）。
4. **匯流排時序。** 每 4 ms 問一顆馬達，沿用 `stop_all_wheels()` 原本的間隔。用
   `ros2 topic hz /wheel_feedback` 看是不是 50 Hz、訊息裡是不是都有四個關節。
5. **heartbeat。** 行駛中 `docker stop` 掉 `kros_car` 容器，輪子應該在 0.5 秒內停。

之後要量的：

- **轉矩常數。** 0.75 是規格書的值，規格書自己的數字換算出來是 0.64–0.74。量完改
  `wheel_torque_constant`。
- **輪徑和輪距修正。** `controllers.yaml` 的 `wheel_radius` 和 `wheel_separation_multiplier`
  是在里程計還是開迴路的時候校正的，現在里程計用的是量到的輪子位置，要用 `calibrate.sh` 重做。

## 待你決定

- **整車質量。** URDF 的 `base_link` 還是 37.6 kg，要實際秤。它不影響實體車，但模擬會用到。
- **感測器座標請對照實車量一次。** `base_link` 是 +x 朝車頭（手臂那側）、+y 朝左；CAD 是朝 −x 畫的，
  所以 `base_link` 自己的網格、慣量和鎖在它上面的零件都繞 z 轉了 180°。三個感測器直接掛在
  `base_link`、朝向和它一致：
  光達 (+0.045, 0, 0.236)、IMU (+0.125, −0.0286, 0.11)、D455 (+0.110, 0, 0.209)。
  光達和 IMU 是把舊座標的 x、y 變號換算過來的，D455 沿用原值。
- **光達轉速。** 實體車設 14 Hz（`oradarlidar.launch.py` 的 `motor_speed`），模擬和
  `car_loc_lidar` 的註解都是 10 Hz。這裡沒有改。
- **`max_rpm: 100`。** 等於 0.53 m/s。馬達空載可到 200 rpm，要不要放寬看你資料收集的速度範圍。
