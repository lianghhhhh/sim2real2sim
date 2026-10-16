# wildbot_calibration — build & run

互動式 CLI 校正 package。所有結果寫入 `~/.ros/calibration_results.yaml`。

## Build

```bash
# 在你的 ROS 2 Jazzy Docker 內
cd ~/ros2_ws            # 或你 workspace 的路徑
colcon build --packages-select wildbot_calibration
source install/setup.bash
```

如果 build / runtime 報 `ruamel.yaml` 找不到:

```bash
sudo apt install python3-ruamel.yaml      # 推薦
# 或
pip3 install ruamel.yaml
```

`numpy` 通常 ROS 2 已內建,若沒有:`sudo apt install python3-numpy`。

## Run

### 整套 wizard

```bash
ros2 launch wildbot_calibration run_all.launch.py
```

依序跑:
1. `imu_axis_sanity` — 用手推/轉車驗證 IMU 軸向
2. `imu_static_calibration` — 30 秒靜態,算 gyro/accel bias + covariance
3. `wheel_linear_calibration` — 直線 20 秒,捲尺實測,算 wheel_radius
4. `wheel_angular_calibration` — 原地左轉 5 圈,IMU yaw 自動量,算 wheel_separation_multiplier
5. `imu_accel_two_face` — 水平 + 朝天兩姿態驗證加速度計

每步驟之間可以選擇 ENTER 執行、`s` 跳過、`q` 結束 wizard。

### 單獨跑某一項(重校時)

```bash
ros2 launch wildbot_calibration imu_axis_sanity.launch.py
ros2 launch wildbot_calibration imu_static_calibration.launch.py
ros2 launch wildbot_calibration wheel_linear_calibration.launch.py
ros2 launch wildbot_calibration wheel_angular_calibration.launch.py
ros2 launch wildbot_calibration imu_accel_two_face.launch.py
```

### 前置條件

跑 wheel_* 校正前,kros_car 跟 wildbot_imu 都要先啟動:

```bash
# Terminal 1
ros2 launch kros_car bringup.launch.py

# Terminal 2
ros2 launch wildbot_imu imu.launch.py

# Terminal 3
ros2 launch wildbot_calibration run_all.launch.py
```

跑純 IMU 的腳本(axis_sanity / static / accel_two_face)時 kros_car 不一定需要,但 wildbot_imu 必須跑。

## Topics / frames / nominal values

下列值已 **寫死** 在各腳本檔案頂端的「discovered constants」區塊(註解標明來源檔案/行號):

| 常數 | 值 | 來源 |
|---|---|---|
| `CMD_VEL_TOPIC` | `/base_controller/cmd_vel` | `kros_car/config/controllers.yaml:8` + `README.md:45` |
| `CMD_VEL_MSG_TYPE` | `geometry_msgs/TwistStamped` | `kros_car/README.md:45` |
| `ODOM_TOPIC` | `/base_controller/odom` | `kros_car/README.md:46` |
| `IMU_TOPIC` | `/imu/data` | `wildbot_imu/src/wildbot_imu_node.cpp:70` |
| `BASE_FRAME` | `base_link` | `kros_car/config/controllers.yaml:27` |
| `IMU_FRAME` | `imu_link` | `wildbot_imu/launch/imu.launch.py:29` |
| `WHEEL_RADIUS_NOMINAL` | `0.05035` m | `kros_car/config/controllers.yaml:23` |
| `WHEEL_SEPARATION_MULTIPLIER_NOMINAL` | `2.21` | `kros_car/config/controllers.yaml:20` |
| `NUM_REVOLUTIONS` | `5` | `wildbot_calibration` 自訂(可改) |

**如果你日後改了 controller 設定 / 換了 IMU driver,請去對應腳本檔案頂端常數區塊更新,然後重新 `colcon build`。**

### 為什麼校正 multiplier 而不是 wheel_separation 本身?

`kros_car/config/controllers.yaml` 同時設了:
- `wheel_separation: 0.274` (URDF 物理值)
- `wheel_separation_multiplier: 2.21` (校正用 trim)

`diff_drive_controller` 真正使用的 effective separation 是 `0.274 × 2.21 = 0.60554 m`。本 package 把 multiplier 視為要校正的旋鈕,base 值保持不動。

## 結果輸出

```
~/.ros/calibration_results.yaml
```

(用 `ROS2_CALIBRATION_YAML` 環境變數可以改路徑。)

Docker 跑的話請確認這個路徑有 volume mount 到 host,否則 container 砍掉就會跟著消失。

## 校正後手動更新

`wheel_linear_calibration` 跟 `wheel_angular_calibration` **不會自動修改你的 ros2_control yaml**。每個腳本的 summary 會告訴你新舊值:

- `wheel_radius`:`kros_car/config/controllers.yaml` line 23
- `wheel_separation_multiplier`:`kros_car/config/controllers.yaml` line 20

改完之後重新 `colcon build kros_car`。

IMU bias / covariance 則供你的 IMU driver 讀取使用(driver 整合不在本 package 範圍內)。可從 `~/.ros/calibration_results.yaml` 讀以下欄位:

- `imu.gyro_bias` — 從 raw `angular_velocity` 扣除
- `imu.gyro_covariance_diag` — 寫入 `sensor_msgs/Imu` 的 `angular_velocity_covariance` 對角線
- `imu.accel_bias_xy` — 從 raw `linear_acceleration.x/y` 扣除
- `imu.accel_covariance_diag` — 寫入 `linear_acceleration_covariance` 對角線

## 安全機制

任何送 cmd_vel 的腳本都裝了:

- 動作前要 confirm 兩次(其中一次「場地夠嗎?」)
- Ctrl+C / SIGTERM / atexit / 例外退出時,publish 零 Twist ≥ 10 次
- 結束後再 publish 零 Twist 確保停車

如果腳本卡住,Ctrl+C 仍然會嘗試停車,但**請額外打 e-stop**。

## 不會做的事

- 不會自動偵測 IMU 裝反(由 `imu_axis_sanity` 互動處理)
- 不會校正夾爪(已另行處理)
- 不會校正磁力計(IMU 為 6 軸模式)
- 不會校正相機外參、不會 tune EKF
- 不會修改 `kros_car` / `wildbot_imu` 任何檔案

## 重跑與不變量

腳本可獨立重跑(只更新 YAML 中該 section 的欄位 + `calibrated_at` 時間戳),其他欄位保留。建議的順序(wizard 自動依此順序):

1. `imu_axis_sanity` 永遠最先 — 軸向錯,後面全錯
2. `imu_static_calibration` — bias 校正後 yaw 短時間積分才可信
3. `wheel_linear_calibration` — 跟 IMU 無關,可隨時做
4. `wheel_angular_calibration` — 用 IMU yaw,需要 step 2 完成
5. `imu_accel_two_face` — 順序自由
