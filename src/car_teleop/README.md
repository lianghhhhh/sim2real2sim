# car_teleop

手動開 kros_car (模擬 `kros_car*.usd` 與實體 KROS car 都適用)。**建圖的時候需要它** ——
要人開著車把房間每個角落都掃到, 地圖才會完整。

```bash
# 終端 1: 速度換算層 (mapping.launch.py 與各定位 launch 會自己帶起來, 通常不用另外跑)
ros2 launch car_teleop teleop.launch.py

# 終端 2: 鍵盤遙控 (要真的 TTY, 所以要 docker exec -it)
ros2 run car_teleop teleop_key
```

## 架構

```
teleop_key ──/cmd_vel (Twist: m/s, rad/s)──> cmd_vel_bridge ──/joint_command (四輪 rad/s)──> 車
                                                    ^
                                       /imu (只有 yaw_trim_gain > 0 才用)
```

任何會發 `geometry_msgs/Twist` 到 `/cmd_vel` 的東西都可以取代 `teleop_key` ——
Foxglove 的 Teleop 面板、搖桿、`teleop_twist_keyboard`、nav2。

### teleop_key (鍵盤)

```
  w/s 直線前進後退   a/d 原地左右轉   空白 停
  +/- 速度上限   [/] 轉向上限   t 切換按住/持續   q 離開
```

* 預設速度 0.35 m/s、轉速 0.6 rad/s (`linear_speed` / `angular_speed`)。
* 四個方向鍵互斥: `w`/`s` 會把轉向歸零, `a`/`d` 會把車速歸零 (原地轉)。
* 預設「按住才走」: 放開超過 0.4 秒 (`hold_time`) 就停, 靠終端機的按鍵重複。
  終端機沒有按鍵重複或 ssh 很慢的話按 `t` 切成「按一下持續走」, 空白鍵停。
* 每秒發 20 次; 離開前補發停車。

### cmd_vel_bridge (換算)

每秒 50 次:

1. **斜率限制**: 目標車速每秒最多變 0.8 m/s (`accel_limit`), 轉速 2.5 rad/s² (`angular_accel_limit`)。
2. **運動學換算**:

   ```
   左輪 = (v - w × T_eff / 2) / r        右輪 = (v + w × T_eff / 2) / r
   [FL, FR, RL, RR] = [左, 右, 左, 右]
   ```

   r = 0.05035 m。T_eff = 0.95 m 是滑移轉向的**等效輪距**, 不是幾何輪距 0.27 m:
   原地轉時四個輪子都在橫向滑, 車體轉得比「輪速差 / 輪距」慢很多。0.95 是模擬
   (地面 0.5) 實測的值, 換地面會變。
3. **輪速上限**: 超過 10 rad/s (`max_wheel_speed`, 實體車 max_rpm 100 = 10.47) 時左右等比縮小,
   保住車速與轉速的比例。
4. **安全**: 0.5 秒沒收到 `/cmd_vel` 就把目標歸零; 不管目標是不是 0 都持續發
   (實體車 0.3 秒沒收到指令輪子會停, Isaac 則會一直維持最後一筆)。關掉前補送 0。

鍵盤預設值對應的輪速:

| 按鍵 | 四輪輪速 (rad/s) |
| --- | --- |
| `w` | 全部 +6.95 (= 0.35 m/s) |
| `s` | 全部 -6.95 |
| `a` | 左 -5.66、右 +5.66 (要求 0.6 rad/s) |
| `d` | 左 +5.66、右 -5.66 |

## 兩種模式: 固定輪速 (預設) 與 gyro 修正

| | `yaw_trim_gain: 0.0` (預設) | `yaw_trim_gain: 1.0` |
| --- | --- | --- |
| 輪速指令 | 同一個 `/cmd_vel` 永遠是同一組輪速 | 加上「要求轉速 - gyro 轉速」的積分, 最多 ±0.6 rad/s (`yaw_trim_limit`) |
| 實際轉速 | 看地面: 同樣 0.6 rad/s, 換地板可能差兩三成 | 跟要求的一樣 |
| 模擬 vs 實體 | 收到的指令完全一樣 | 指令會各自被 gyro 帶著變 |
| 適合 | 人用鍵盤開 (看著畫面自己補); 建圖 | nav2 這類假設車子照指令動的程式 |

切換:

```bash
ros2 launch car_teleop teleop.launch.py yaw_trim_gain:=1.0
ros2 launch car_loc_lidar mapping.launch.py yaw_trim_gain:=1.0
```

**為什麼預設關掉** (2026-10-14 模擬實測): 原地轉時四個輪子的載重會在兩組對角之間跳,
車體轉速一直抖, 修正會帶著輪速指令在 5 ~ 10 rad/s 之間起伏; 車子卡住、轉不動時
修正會一路加到輪速上限 10。對人手動開車來說, 這兩件事都比「轉速跟要求差一點」糟。

摩擦力測試 (`calibrate_env_pkg/control_car_node`) 不經過這個節點, 直接送 `/joint_command`,
不受這個設定影響。

## 參數

| 參數 | 預設 | 說明 |
| --- | --- | --- |
| `cmd_vel_topic` / `joint_command_topic` / `imu_topic` | `/cmd_vel` / `/joint_command` / `/imu` | 實體車的 IMU 是 `/imu/data` |
| `wheel_radius` | 0.05035 | m |
| `effective_track` | 0.95 | m, 等效輪距 |
| `max_wheel_speed` | 10.0 | rad/s |
| `max_linear` / `max_angular` | 0.5 / 1.2 | m/s / rad/s; 車速超過 0.50 m/s 會被輪速上限縮掉 |
| `accel_limit` / `angular_accel_limit` | 0.8 / 2.5 | m/s² / rad/s² |
| `yaw_trim_gain` / `yaw_trim_limit` | 0.0 / 0.6 | 見上一節 |
| `cmd_timeout` | 0.5 | s |
| `control_rate` | 50 | Hz |

舊車 `car.usd` (扭矩控制、PI 速度環) 的版本見 git 歷史。
