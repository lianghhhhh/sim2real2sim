# collect_car_pkg

收**車輛動態模型**的訓練資料: 在 Isaac 裡讓 kros_car 照隨機的輪速指令一直開, 把指令與車子的反應
(輪速、輪子扭矩、IMU、GT 位姿與速度) 記成 CSV, 給 `dynamic_predictor` 訓練。

**只給模擬用**, 而且**沒有撞牆保護** —— 要在沒有牆的開放場地跑 (見「注意」)。

```bash
# Isaac 先打開 kros_car.usd (開放場地版) 並按 Play; 然後在容器裡:
r                                                     # 第一次要 build
ros2 launch collect_car_pkg collect_car.launch.py csv_filename:=dyn_mu05_ou_s0.csv
# 收夠了按 Ctrl-C (會先送輪速 0 讓車停下)
```

資料寫到 `car_run_data/<csv_filename>` 與 `car_run_data/<檔名>_imu.csv`。

## 架構

```
control_car_node ──/joint_command (四輪 rad/s)──> actuator_model ──/joint_command_isaac──> Isaac
      │                     │                    (延遲 + 30 rad/s² 斜坡)
      │ /test_phase         │
      │ /test_scenario      v
      └──────────> collect_data_node (calibrate_env_pkg) <── /joint_states /odom /imu
                          │
                          v
                car_run_data/<csv_filename>, <檔名>_imu.csv
```

`collect_car.launch.py` 一次開三個節點:

| 節點 | 套件 | 做什麼 |
| --- | --- | --- |
| `control_car_node` | collect_car_pkg | 產生隨機輪速指令 (下一節), 20 Hz 送到 `/joint_command`。它結束 (Ctrl-C) 整個 launch 跟著關 |
| `collect_data_node` | calibrate_env_pkg | 記錄; 跟摩擦力測試用的是同一個 |
| `actuator_model` | car_teleop | 把實體車 DDSM115 的延遲、加速斜坡、停車衰減加到指令上 (`actuator:=false` 不開) |

**一定要開 actuator_model**: `kros_car.usd` 訂閱的是 `joint_command_isaac`, 不開的話車子完全不動。
(`kros_car_05_sim.usd` / `kros_car_10_real.usd` 還是訂閱 `joint_command`, 那時 actuator_model 沒作用, 等於理想輪子。)

CSV 裡的 `velocity_command_*` 是 actuator_model **之前**的指令 = 實體車會收到的指令, 動態模型的輸入用這個。

## 激發方式 (`mode`)

指令先在正規化的 throttle / steer (範圍 [-1, 1]) 上產生, 再換成輪速:

```
左輪 = (throttle - steer) × max_wheel_speed      右輪 = (throttle + steer) × max_wheel_speed
[FL, FR, RL, RR] = [左, 右, 左, 右]              steer > 0 = 往左轉 (wz > 0)
```

* 超過 `max_wheel_speed` (預設 10 rad/s = 實體車 max_rpm 100) 時左右**等比縮小**, 保住轉彎半徑。
* 送出去的輪速量化成**整數 rpm** (實體車的馬達只收整數 rpm)。
* throttle 負的是**倒車**; 煞車 = 指令 0 (速度環主動煞停)。

| | `ou` (預設) | `hold_ramp` |
| --- | --- | --- |
| 怎麼產生 | 兩個 Ornstein-Uhlenbeck 過程 (throttle 偏前進 mu=+0.5、steer 左右對稱), 再過兩層 0.25 s 低通 (`ou_smooth_tau`) | 維持 0.5 ~ 2.5 s → 換新目標值; 30% 直接跳 (像按鍵盤), 其餘 0.3 ~ 1.5 s 線性 ramp |
| 開起來像 | 一直微調油門與方向 | 一段一段: 巡航、慢速、停、低速、倒車; 直行、左轉、右轉 |
| 可行的 (車速, 轉速) 範圍覆蓋 | 97% | 85% |
| 近乎直行 (yaw < 0.05 rad/s) | 6% | 22% |
| 倒車 | 21% | 13% |
| 停下來 ≥ 0.5 s | 幾乎沒有 | 每小時約 5 次 |
| 指令方向來回抖 | 7% | 6% |

(數字是 50 分鐘指令的統計, 2026-10-21。) 兩種互補: `ou` 範圍廣, `hold_ramp` 有直行、階躍與停車。
**建議兩種都收**, 例如 `ou` seed 0~3 各 10 分鐘 + `hold_ramp` seed 0~1 各 10 分鐘, 總共 30 ~ 60 分鐘。

預設不對指令做斜率限制 (`rate_limit` 0): 模型要從資料裡學到實體車的延遲 (~0.06 s) 與 30 rad/s² 的
加速斜坡 (actuator_model 加上的), 指令先被抹平就學不到。

### 同一個 seed = 同一串指令

產生器只看自己的亂數與狀態 (不看車子), 所以 seed、mode、`ou_smooth_tau`、`max_wheel_speed` 一樣,
指令序列就完全一樣 —— 換一個地面摩擦係數的環境重跑, 車子收到的是同一串指令, 才能比較兩個環境的反應。
跑的時間長短不同沒關係, 短的那份就是長的那份的開頭。

## 輸出的資料

主 CSV (`collect_data_node`, 50 Hz) 會用到的欄位:

| 欄位 | 說明 |
| --- | --- |
| `timestamp` | 記錄當下的 ROS 時間 (模擬時間) |
| `phase` / `scenario_name` | `measure` = 激發中 (**訓練只用這段**), `idle` = 開始前的 1 秒; scenario 是 mode 名稱 |
| `velocity_command_{front_left,front_right,rear_left,rear_right}` | 輪速指令 (rad/s) |
| `{front_left,...}_velocity` / `_angle` / `_effort` + `wheel_stamp` | 輪速 (rad/s)、輪子角度、輪子扭矩 (N·m), 與 `/joint_states` 的時戳 |
| `car_position_x/y/z`, `car_orientation_x/y/z/w`, `gt_yaw` + `odom_stamp` | Isaac GT 位姿, 與 `/odom` 的時戳 |
| `car_linear_velocity_x/y/z`, `car_angular_velocity_x/y/z` | Isaac GT 速度, **車體座標**: x = 往前、y = 往左、wz 逆時針為正 (kros_car 車頭朝 +x, 不用翻號) |
| `gyro_z`, `acc_x`, `acc_y` + `imu_msg_stamp` | `/imu` |
| `cam_*`, `lid_*`, `fus_*` ... | 定位節點的輸出; 這個 launch 沒開定位, 全是 NaN |

`<檔名>_imu.csv` 是 `/imu` 每一則訊息一列 (全速率), 帶著當下最新的指令、輪速、扭矩。

**時間軸要用各來源的 stamp, 不要直接拿列當時間步。** 記錄是 50 Hz, 但 Isaac 的 ROS 時鐘 60 Hz 才跳一次,
列與列的間隔在 1/60 與 2/60 s 之間跳, 而且約 20% 的列跟上一列是同一則訊息。
`dynamic_predictor/carDynamic_mlp_new_2_body/utils.py` 的 `load_segments()` 會處理: 只取 `measure`,
用 `odom_stamp` / `wheel_stamp` 重取樣到固定 0.05 s, 指令用「沿用前一筆」, 遇到時間空檔就切段。

## 參數

`collect_car.launch.py`:

| 參數 | 預設 | 說明 |
| --- | --- | --- |
| `csv_filename` | `dyn_data.csv` | 輸出檔名 |
| `output_dir` | `/workspaces/car_run_data` | 容器裡的路徑 = host 的 `car_run_data/` |
| `mode` | `ou` | `ou` \| `hold_ramp` |
| `seed` | 0 | 同一個 seed = 同一串指令 |
| `ou_smooth_tau` | 0.25 | s, OU 兩層低通的時間常數; 0 = 不濾 (每 0.05 s 都在亂跳) |
| `max_wheel_speed` | 10.0 | rad/s |
| `rate` | 20.0 | Hz, 送指令的頻率 (模擬時鐘 60 Hz, 20 Hz 剛好每 3 跳一次) |
| `actuator` | true | 開 actuator_model |
| `actuator_delay` | 0.05 | s, actuator_model 的延遲 (在使用者的 Isaac 校的, 2026-11-06; 換環境見對齊報告 B1) |
| `use_sim_time` | true | |

`control_car_node` 另外還有 (用 `ros2 run collect_car_pkg control_car_node --ros-args -p ...` 才改得到):

| 參數 | 預設 | 說明 |
| --- | --- | --- |
| `joint_command_topic` | `/joint_command` | |
| `rate_limit` | 0.0 | rad/s², 指令斜率上限; 0 = 不限制 |
| `command_rpm_step` | 1.0 | 指令量化成幾 rpm 的整數倍; 0 = 不量化 |
| `status_period` | 10.0 | s, 多久印一次狀態 |

`ros2 run` 只會開駕駛節點, 不會記錄 —— 收資料一律用 launch。

## 注意

* **沒有撞牆保護。** `ou` 的 throttle 偏前進, 10 分鐘車子大約離開起點幾十公尺。地面要夠大 (無限大的平面,
  或至少 ±100 m), 不然車子會開出地面掉下去。原本有牆的房間 (`kros_car.usd` 預設場景) 不要用。
* **Ctrl-C 會先送 0 再關。** 駕駛節點與 actuator_model 都自己處理 SIGINT / SIGTERM (rclpy 預設的處理會先
  關掉 ROS context, 0 就送不出去, Isaac 會照最後一筆指令一直開)。`collect_data_node` 按 Ctrl-C 時會印一段
  traceback、結束碼 1, 那是它原本的行為, 資料有寫完。
* Isaac 中途按 Stop / Play (模擬時間倒退) 時, 駕駛節點會停 1 秒再接著送 (指令序列接續, 不重來);
  記錄裡會出現時間倒退, `load_segments()` 會在那裡切段。
* 換地面摩擦係數收另一份資料時, 檔名帶上摩擦係數與 seed (例如 `dyn_mu10_ou_s0.csv`), 其他參數保持一樣。

## 跟舊版 (扭矩控制) 的差別

舊版是給車頭朝 -y、扭矩控制的 `car.usd` 用的。主要差別: 送輪速 (rad/s) 而不是 effort; 左右等比縮放而不是各自
截斷; 負 throttle 是倒車而不是煞車; 有固定 seed; 記錄改用 `calibrate_env_pkg/collect_data_node`
(多了輪子扭矩、IMU、各來源 stamp、段落標籤, 存到有掛出容器的 `car_run_data`); 拿掉 chirp; OU 加了低通;
經過 actuator_model。
