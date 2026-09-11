# car_teleop

手動開 `car.usd` 的車子。**建圖的時候需要它** —— 要人開著車把房間每個角落都掃到,
地圖才會完整。

```bash
# 終端 1: 速度控制層 (通常由 slam.launch.py 自動帶起來, 不用另外跑)
ros2 launch car_teleop teleop.launch.py

# 終端 2: 鍵盤遙控 (要真的 TTY, 所以要 docker exec -it)
ros2 run car_teleop teleop_key
```

```
  w/s 前進後退   a/d 左右轉   空白 停
  +/- 速度上限   [/] 轉向上限   t 切換按住/持續   q 離開
```

不想用鍵盤的話, 任何會發 `geometry_msgs/Twist` 到 `/cmd_vel` 的東西都可以 ——
Foxglove 的 Teleop 面板、搖桿、`teleop_twist_keyboard`、nav2 都行。

---

## 為什麼不能直接發 effort

`car.usd` 的車子是**扭矩控制**: `/joint_command` 收的是 `JointState.effort`,
不是速度。而這台車幾乎沒有滾動阻力 —— 從你自己的 `car_run_data/sim_data.csv`
量到:

| 固定 effort 3.7, 從 t+1.0s 到 t+3.0s | 車速 |
| --- | --- |
| t+1.0s | 0.87 m/s |
| t+2.0s | 1.82 m/s |
| t+3.0s | 2.24 m/s |

**完全沒有收斂**。也就是說開迴路下:

* 按著前進鍵 = 一路加速到撞牆 (10x6 的房間, effort 4 兩秒就到 3.6 m/s)
* 放開按鍵 ≠ 停車, 因為沒有阻力讓它慢下來

所以 `cmd_vel_bridge` 這一層是必要的: 把「我要 0.3 m/s」用 PI 翻譯成扭矩, 而且
**指令逾時是把目標速度設成 0, 不是把 effort 設成 0** —— 控制器會主動給反向扭矩
把車煞停。節點結束前也會先煞車再退出。

## 回授用什麼

| 量 | 來源 | 為什麼 |
| --- | --- | --- |
| 線速度 | `/joint_states` 的輪速 × 0.075 m | 實測沒打滑時跟真值差 0.002 m/s。撞牆/打滑時輪速會飆高, 控制器因此自動收油 —— 這是想要的行為 |
| 角速度 | `/imu` 的 gyro z | 直接量測。skid-steer 轉彎時輪子一定在滑, 用輪速差推算不可靠 |

兩個都是真車上也有的感測器, 沒有用到任何 ground truth。

## 從 car.usd / 你的資料量到的常數

| 項目 | 值 | 來源 |
| --- | --- | --- |
| 輪半徑 | 0.075 m | cylinder radius 0.5 × scale 0.15 |
| 左右輪距 | 0.25 m | 輪心 x = ±0.125 |
| 關節名稱 | `front_left_joint`, `front_right_joint`, `rear_left_joint`, `rear_right_joint` | USD |
| 混控 | `[FL,FR,RL,RR] = [thr-steer, thr+steer, thr-steer, thr+steer]` | 跟 `calibrate_env_pkg/control_car_node.py` 一致 |
| steer 正負 | steer > 0 → `wz > 0` | 實測: 左 -10 / 右 +10 → wz = +16.4 rad/s |
| 前進方向 | 車體 **-Y** | 實測: 正 throttle → `vy` 為負 |
| 驅動增益 | `a ≈ 0.34 × throttle`, `α ≈ 0.57 × steer` | 由 `sim_data.csv` 回歸 (R² 只有 0.17/0.02, 那份資料有大量撞牆與打滑, 所以只當標稱值) |

PI 增益 (`kp_v` 3.0 / `ki_v` 3.0 / `kp_w` 2.0 / `ki_w` 2.0) 是從上面那組增益推出來的
量級, **不是在 Isaac 裡精調過的**。覺得軟或會晃就調 `config` 或用 `--ros-args -p`。

## `cmd_vel.linear.x` 的方向

`car.usd` 的 `base_link` 是 x 朝左、車頭 -Y (不是 ROS 慣例)。但所有現成的遙控工具
都把前進放在 `linear.x`, 所以這個 bridge 一律把 `linear.x` 當「車頭方向的速度」。
要接 nav2 的時候這件事要一起處理。

## 不開 Isaac 也能試

**目前沒有。** 以前的 `fake_isaac` (`motion:=drive` 會訂 `/joint_command`、用上面
那組回歸出來的模型跑物理、發 `/joint_states`) 放在 `car_localization` 裡, 已經
跟著那個 package 刪掉了。而且它合成的是舊的 `/lidar/point_cloud`, 不是現在
`car_loc_lidar` 吃的 `/scan`, 就算救回來也接不上建圖流程。

要翻舊程式: `git show 7c984df:src/car_localization/car_localization/fake_isaac.py`。
注意它的輪速是**理想無滑移**的, 打滑情境下測出來的手感會比實際樂觀。

現在要驗遙控就直接開 Isaac; 各條定位線則各自有不需要 ROS 的離線測試
(見根目錄 README 第 5 節)。
