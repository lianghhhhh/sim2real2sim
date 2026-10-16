# wildbot_calibration

互動式 CLI 校正 package(ROS 2 Jazzy,Python)。一次跑一個腳本,結果全部寫入單一 YAML(`~/.ros/calibration_results.yaml`)。為 wildbot 競賽機器人(4WD skid-steer + Yahboom CMP10A IMU)而寫。

跑完即退出,不是 spin 常駐 node。繁體中文 prompt。

## 校正腳本

| # | 腳本 | 內容 |
|---|---|---|
| 1 | `imu_axis_sanity` | 推/轉/抬車,被動觀察 IMU,驗證軸向 sign。**不送 cmd_vel**。 |
| 2 | `imu_static_calibration` | 靜態 30 秒,算 `gyro_bias`、`gyro_covariance_diag`、`accel_bias_xy`、`accel_covariance_diag`、`|g|`。 |
| 3 | `wheel_linear_calibration` | 直線 20 秒 @ 0.1 m/s,人工捲尺量,算 `wheel_radius` 校正值。 |
| 4 | `wheel_angular_calibration` | 原地左轉 5 圈 @ 0.5 rad/s,**IMU yaw 自動量**,算 `wheel_separation_multiplier` 校正值。 |
| 5 | `imu_accel_two_face` | 水平 + 朝天兩姿態,加速度計 sanity(簡化版六面)。 |

Wizard:`run_all_calibrations` 依序跑 1→5,可單步跳過。

夾爪不在範圍內(已另行校正)。磁力計也不校(IMU 為 6 軸模式)。

## 校正順序的理由

1. **`imu_axis_sanity` 必須最先** — IMU 軸向裝反是 silent killer,先抓出來避免後面全錯
2. **`imu_static_calibration` 第二** — bias 校正後,短時間 yaw 積分才可信,步驟 4 需要
3. **`wheel_linear`** — 跟 IMU 無關,可獨立
4. **`wheel_angular`** — 用 IMU yaw 累積值自動量,需要步驟 2
5. **`imu_accel_two_face`** — 順序自由

每個腳本也可以單獨重跑,只更新該 YAML section + 時間戳。

## YAML 結構(`~/.ros/calibration_results.yaml`)

```yaml
imu:
  gyro_bias: [x, y, z]                # rad/s, imu_link frame
  gyro_covariance_diag: [x, y, z]     # std^2
  accel_bias_xy: [x, y]               # m/s^2
  accel_covariance_diag: [x, y, z]
  accel_gravity_magnitude: 9.80665
  axis_sanity_passed: bool
  axis_sanity_notes: str
  yaw_trust: false                    # 6 軸模式恆為 false
  calibrated_at, calibrated_by

base:
  linear_scale                        # measured / commanded
  wheel_radius_nominal, wheel_radius_corrected
  angular_scale
  wheel_separation_multiplier_nominal, wheel_separation_multiplier_corrected
  calibrated_at, calibrated_by

gripper:                              # 留欄位但本 package 不寫
meta:
  schema_version, robot_name
```

優先序:`$ROS2_CALIBRATION_YAML` > `~/.ros/calibration_results.yaml`。使用 `ruamel.yaml`(保留註解/排版),partial update。

## Build & Run

請見 [`BUILD.md`](BUILD.md)。簡要:

```bash
colcon build --packages-select wildbot_calibration
source install/setup.bash
ros2 launch wildbot_calibration run_all.launch.py
```

前置:`kros_car` 跟 `wildbot_imu` 要先啟動(wheel_* 校正需要 cmd_vel + IMU)。

## 設計原則

- **Topic / nominal value 寫死在源碼**,不做 launch argument、不做 ROS parameter、不做 runtime controller config 解析。每個腳本檔案頂端有「discovered constants」區塊,**註解標明來源檔案/行號**,改 controller 設定要回頭改常數重新 build。
- **單一 YAML source of truth**。`wheel_radius` / `wheel_separation_multiplier` 不會自動修改 `kros_car/config/controllers.yaml`,由使用者依 summary 提示手動更新後重 build。IMU bias / covariance 供 IMU driver 自行讀取。
- **安全機制**:任何送 cmd_vel 的腳本都裝 atexit + SIGINT/SIGTERM brake,Ctrl+C 後仍會 publish 零 Twist ≥10 次。動作前要 confirm 兩次(含「場地夠嗎」)。
- **SI 單位儲存**(rad/s、m/s²、m、s)。Summary 換算 deg / mm 給人看。
- **跑完即退出**,不 spin 常駐,不 GUI。

## 來源常數(出處)

| 常數 | 值 | 來源 |
|---|---|---|
| `CMD_VEL_TOPIC` | `/base_controller/cmd_vel` | `kros_car/config/controllers.yaml:8` + `README.md:45` |
| `CMD_VEL_MSG_TYPE` | `geometry_msgs/TwistStamped` | `kros_car/README.md:45,165` |
| `ODOM_TOPIC` | `/base_controller/odom` | `kros_car/README.md:46` |
| `IMU_TOPIC` | `/imu/data` | `wildbot_imu/src/wildbot_imu_node.cpp:70` |
| `BASE_FRAME` | `base_link` | `kros_car/config/controllers.yaml:27` |
| `IMU_FRAME` | `imu_link` | `wildbot_imu/launch/imu.launch.py:29` |
| `WHEEL_RADIUS_NOMINAL` | `0.05035` m | `kros_car/config/controllers.yaml:23` |
| `WHEEL_SEPARATION_MULTIPLIER_NOMINAL` | `2.21` | `kros_car/config/controllers.yaml:20` |
| `NUM_REVOLUTIONS` | `5` | 自訂(`wheel_angular_calibration.py`) |

角度校正調整的是 `wheel_separation_multiplier`,不是 base `wheel_separation`,讓 URDF 物理量保持乾淨。

## 套件結構

```
wildbot_calibration/
├── package.xml
├── setup.py, setup.cfg
├── README.md, BUILD.md
├── resource/wildbot_calibration
├── config/calibration_results.yaml         # 預設範本
├── wildbot_calibration/
│   ├── common/
│   │   ├── yaml_store.py                   # ruamel.yaml partial update
│   │   ├── prompts.py                      # CLI helpers (countdown/confirm/...)
│   │   └── motion.py                       # cmd_vel publisher + safety brake
│   ├── imu_axis_sanity.py
│   ├── imu_static_calibration.py
│   ├── wheel_linear_calibration.py
│   ├── wheel_angular_calibration.py
│   ├── imu_accel_two_face.py
│   └── run_all.py
└── launch/
    ├── imu_axis_sanity.launch.py
    ├── imu_static_calibration.launch.py
    ├── wheel_linear_calibration.launch.py
    ├── wheel_angular_calibration.launch.py
    ├── imu_accel_two_face.launch.py
    └── run_all.launch.py
```

## License

MIT.
