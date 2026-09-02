# car_localization

只用 **LiDAR + IMU** 在 Isaac Sim 的 `car.usd` 場景裡估車子的位置, 目標是做到跟
直接訂閱 Isaac 的 ground truth `/odom` 一樣準。

---

## 60 秒版本

```bash
# 1) 在 host 上產生地圖 (只要做一次, 不用開 Isaac)
./scripts/make_map_from_usd.py

# 2) 進容器 build
./run_isaac_gui.sh
colcon build --packages-select car_localization --symlink-install && source install/setup.bash

# 3) Isaac 按 Play, 然後
ros2 launch car_localization localization.launch.py evaluate:=true
```

手邊沒開 Isaac 也可以先驗一遍 (見下面 fake_isaac 那節):

```bash
ros2 run car_localization fake_isaac &
ros2 launch car_localization localization.launch.py evaluate:=true
```

`evaluate:=true` 會同時開一個節點, 拿 Isaac 的 `/odom` 當尺, 每 5 秒告訴你
「現在差幾公分」。不用自己看 rviz 猜。

想用 Foxglove 看畫面, 或是要自己開車建圖:

```bash
ros2 launch car_localization viz.launch.py    # 開 WebSocket 給 Foxglove 連
ros2 launch car_localization slam.launch.py   # 手動開車建圖 (換到實體環境走這條)
ros2 run car_teleop teleop_key                # 鍵盤遙控 (要 docker exec -it)
```

輸出:

| topic / TF | 內容 |
| --- | --- |
| `/localization/odom` | `nav_msgs/Odometry`, map -> base_link, 跟 IMU 同頻 (~60 Hz) |
| `/localization/pose` | `PoseWithCovarianceStamped`, 給 rviz |
| `/localization/map_cloud` | 地圖的點雲 (latched), 給 rviz 疊圖看 |
| TF `map -> base_link` | 可用 `tf_mode:=map_to_odom` 改成 nav2 慣例 |

---

## 跟舊做法差在哪

| | 地圖 | odom -> base_link | map -> odom | 這台車能不能用 |
| --- | --- | --- | --- | --- |
| **wildbot (實體車)** | slam_toolbox 建, map_saver 存 | EKF: 輪速計 + IMU | AMCL 粒子濾波 | 不能, 沒有輪速計 |
| **car_navigation (舊)** | slam_toolbox 即時建 / 自建 npz | rf2o 或自寫 ICP 對關鍵幀 | EKF 的 world_frame=odom, 沒有全域層 | 會漂 |
| **car_localization (新)** | 直接從 USD 幾何切 (模擬) / slam_toolbox (實體) | 不需要這一層 | 每幀直接對地圖配準 | 0.5~1.0 cm |

`wildbot` 那一套 (AMCL + robot_localization EKF) 在實體車上是對的, 因為真車有
**輪速計**: EKF 有一個連續、不跳動的 odom -> base_link 可以吃, AMCL 只負責修
map -> odom 的慢漂移。

這台模擬車沒有輪速計可用 (你要的是純 LiDAR+IMU), 所以那個架構缺了最底下那一層。
`sim2real2sim` 之前的兩次嘗試都是在補這一層:

* `rf2o_laser_odometry` + EKF —— 掃描比對只跟**上一幀**比, 本質上是里程計, 誤差
  一幀一幀往下傳, 一定會漂。
* 自己寫的 ICP 對關鍵幀 —— 同上, 而且關鍵幀模式在原地打轉時特別容易失效。

`car_navigation` 的 `ekf_lidar_imu.yaml` 註解裡其實已經寫對了診斷 ——
「odom0 提供絕對的 x/y/yaw (由 ICP 累積)」—— 問題就在「累積」兩個字:
沒有絕對參考的東西, 疊再多層濾波器也只是把漂移變平滑, 不會變準。

這個 package 換掉的是**問題的形狀**, 不是參數:

> 房間是靜態的、封閉的, 而且 360 度 LiDAR 從房間裡任何一點都看得到四面牆。
> 既然「整張地圖隨時都在視野裡」, 就不需要里程計 —— 每一幀直接對**地圖**配準,
> 得到的是絕對位姿, 誤差不累積, 也不需要回環偵測。

---

## 三種模式

| `mode` | 在做什麼 | 需要地圖嗎 | 發什麼 TF | 什麼時候用 |
| --- | --- | --- | --- | --- |
| `localize` (預設) | 每幀對既有地圖配準 | 要 | `map -> base_link` | 平常跑 |
| `odometry` | 只對最近幾個關鍵幀組成的**滾動子圖**配準 | 不用 | `odom -> base_link` | 餵給 slam_toolbox / nav2 的底層里程計 |
| `mapping` | 一邊配準一邊把地圖長出來 (hector 式) | 不用 | `map -> base_link` | 小房間快速拿一張堪用的圖 |

`odometry` 是這次補的關鍵一塊。實體車那一層是輪速計 + EKF 給的 (wildbot 就是這樣),
這台模擬車沒有輪速計, 所以用雷射自己生一個。它刻意**只**對滾動子圖配準, 不去做
全域一致 —— 那件事交給 slam_toolbox 的位姿圖, 兩邊各司其職。

---

## 換到實體環境: 用 slam_toolbox 建圖

`car.usd` 這個場景不需要 SLAM (幾何是已知的)。但實體環境沒有 USD 檔可以切,
就要真的建圖。這條路用的是 wildbot 實體車已經驗證過的那一套。

**建圖一定要手動把車開一圈** —— 雷射只看得到走過的地方, 沒繞到的角落地圖上就是空的,
之後定位開進那些地方就會飄。所以 `slam.launch.py` 預設會把 `car_teleop` 的速度控制層
一起帶起來 (`teleop:=false` 可以關掉), 你只要另開一個 terminal 跑鍵盤遙控:

```bash
# 終端 1
ros2 launch car_localization slam.launch.py

# 終端 2 (要 -it, 鍵盤遙控需要真的 TTY)
docker exec -it <container> bash -lc 'r && ros2 run car_teleop teleop_key'
#   w/s 前進後退   a/d 左右轉   空白 停   +/- 調速度   q 離開
```

也可以完全不用鍵盤, 從 Foxglove 的 Teleop 面板往 `/cmd_vel` 發 (見下面「用 Foxglove 看」)。

開的時候注意:
* **慢慢開** (預設上限 0.6 m/s 就是為了這個)。開太快掃描比對跟不上, 位姿一漂地圖就歪。
* **柱子後面、四個角落都要繞到**, 不然那裡沒有地圖。
* **要繞回起點**, 回環偵測才有東西可以閉, 全域誤差才分攤得掉。

```bash
# 1) 建圖 (照上面那樣手動開一圈)
ros2 launch car_localization slam.launch.py

# 2) 存圖 (跟 wildbot 的 docker-compose_store_map.yml 同一個指令)
ros2 run nav2_map_server map_saver_cli -f /workspaces/src/car_localization/maps/room

# 3) 用這張圖定位 —— 直接吃 nav2 的 .yaml, 不用轉檔
ros2 launch car_localization localization.launch.py \
    map_path:=/workspaces/src/car_localization/maps/room.yaml
```

`slam.launch.py` 裡面是:

```
/lidar/point_cloud ─┐
/imu ───────────────┴─> car_localizer (odometry) ─┬─> TF odom->base_link
                                                  └─> /scan (運動補償過的)
                                                               │
                                                   slam_toolbox ┴─> TF map->odom + /map
```

兩個細節:

* **為什麼不是內建的 `mapping` 模式**: 內建的是 hector 式的, **沒有回環偵測**。
  走遠再繞回來時累積誤差沒有任何機制可以分攤, 地圖會在接縫處錯開。slam_toolbox
  有位姿圖 + 回環偵測 + 全域最佳化, 而地圖是要長期使用的資產。
* **`/scan` 是我們自己發的, 不是 pointcloud_to_laserscan**: 定位節點本來就要做
  運動補償, 順手把補償後的一圈打成 LaserScan 就好。車子邊轉邊掃的那一圈, 沒補償
  的版本是歪的, 餵給 slam_toolbox 只會讓它建出歪的圖。

### 建出來的地圖會差一個常數平移

slam_toolbox 的 `map` 原點是**車子按下 Play 那一刻的位置**, 不是 USD 的世界原點。
所以拿 SLAM 地圖定位時, 位置會跟 Isaac 的 `/odom` 差一個固定平移 (實測 2.5 m 左右) ——
那不是定位在漂。`evaluate` 節點會把常數偏移單獨報出來, 並直接印出修正指令:

```
常數偏移: dx -251.53 cm, dy -162.14 cm, dyaw +0.01 deg
扣掉常數偏移後的位置誤差: RMS 2.40 cm
要讓地圖座標跟 /odom 對齊, 把地圖 .yaml 的 origin 減掉這個偏移:
    origin_new = [origin_x - (-2.5153), origin_y - (-1.6214), 0]
```

改完重跑, 實測殘餘偏移掉到 0.4 cm 以內。

### 實體車上要改的參數

```bash
ros2 launch car_localization slam.launch.py \
    use_sim_time:=false input_type:=scan imu_topic:=/imu/data yaw_source:=gyro
```

| 參數 | 為什麼 |
| --- | --- |
| `input_type:=scan` | 實體車是 2D 雷射 (wildbot 用 oradar), 發 `LaserScan` 不是 `PointCloud2`。這個模式會自動關掉高度過濾 —— 2D 雷射本來就只有一個水平切片。 |
| `yaw_source:=gyro` | 真車的 6 軸 IMU 沒有絕對 yaw。改成陀螺儀積分 + 掃描比對修正。 |
| `lidar_translation` | 換成你車上實際的雷射掛載位置 |
| `use_sim_time:=false` | 沒有 `/clock` |

---

## 用 Foxglove 看地圖 / 雷射 / 車子在哪

跟 wildbot 一樣, 開一個 WebSocket 讓 Foxglove Studio 連進來:

```bash
ros2 launch car_localization viz.launch.py
```

它啟動後會把要貼進 Foxglove 的網址直接印出來 (`ws://<容器IP>:9090`)。
Foxglove Studio -> Open connection -> Rosbridge -> 貼上。

| topic | 型別 | 看什麼 |
| --- | --- | --- |
| `/map` | `OccupancyGrid` | 地圖。定位模式是載入的那張, 建圖模式是正在長的那張 |
| `/scan` | `LaserScan` | **運動補償後**的一圈掃描 |
| `/localization/pose` | `PoseWithCovarianceStamped` | 車子現在在哪 (含共變異數) |
| `/localization/odom` | `Odometry` | 同上, 高頻版 |
| `/localization/scan_matched` | `PointCloud2` | 配準後的點雲, 疊在地圖上看貼不貼 (`publish_debug_cloud:=true`) |
| `/tf`, `/tf_static` | | `map -> base_link -> sim_lidar / sim_imu` |
| `/cmd_vel` | `Twist` | Foxglove 的 **Teleop 面板**往這裡發, 就能用滑鼠開車 |

`config/foxglove_layout.json` 是一份現成的版面 (3D + Teleop + 圖表), Foxglove 裡
Layout -> Import from file 匯入。**匯不進去也沒關係** —— Foxglove 的版面格式會隨版本變,
照上面那張表自己拉面板就好, 花不到一分鐘。

### 關於容器連線

`run_isaac_gui.sh` 沒有做 port mapping。Linux 上直接用 launch 印出來的容器 IP 連就行;
連不上就在 `docker run` 加 `-p 9090:9090` (foxglove_bridge 則是 `-p 8765:8765`)。

目前這個映像檔**只有 `rosbridge_server`, 沒有 `foxglove_bridge`**, 所以預設走 rosbridge。
`Dockerfile` 已經加了 `ros-humble-foxglove-bridge` 那一行, 重 build 之後
`viz.launch.py` 會自動改用它 (效能好很多, 尤其是點雲)。

---

## 三個關鍵決定

### 1. 地圖直接從 `car.usd` 的幾何算出來, 不用 SLAM 建

`./scripts/make_map_from_usd.py` 把場景裡所有幾何三角化, 取世界高度
`[0.05, 0.95]` 的切片投影成 2D。

這一步把「建圖誤差」直接歸零。用 SLAM 邊開邊建的地圖, 牆的位置本身就帶著建圖
當下的位姿誤差 (實測會出現 10~30 cm 的鬼牆), 之後你永遠分不清是地圖歪了還是
定位歪了。這是模擬環境 —— 牆在哪裡是 USD 檔裡寫死的已知事實, 沒有理由用猜的。

腳本裡有三個看起來很小、但少一個地圖就是壞的處理:

* **水平面不進地圖。** 牆的頂面 (z=1.0) 投影下來會把整條牆的 footprint 填成實心,
  牆就從「兩條細線」變成「1 m 厚的實心磚」。那樣的地圖在牆內部距離場全是 0,
  位姿往牆裡陷 30 cm 也不會被罰到。
* **只留車子看得到的表面。** 牆有厚度, 內外兩面在地圖上是相距 1 m 的兩條平行線,
  配準會分不出自己對上哪一條, 位姿整個滑到外牆上 —— 而且殘差看起來還很漂亮。
  腳本從房間內部 flood fill, 只留跟可走空間相鄰的格。(實測: 沒做這件事之前,
  60 次配準有 1/3 鎖到外牆, 誤差 90+ cm。)
* **距離場用表面取樣點算, 不用格點 EDT。** EDT 量的是「離最近的被佔格中心多遠」,
  格中心跟真牆面最多差半格 (2.5 cm), 而且整張圖的格點是同一組, 對面兩道牆會往
  同一個方向偏 —— 那是**定值偏移**, 不是抖動, 平均再多幀也消不掉。
  (實測: 換成表面點最近鄰之後, 平均誤差從 3.03 cm 掉到 0.20 cm。)

### 2. yaw 直接吃 IMU 的 orientation, 掃描比對只解 x/y

`car.usd` 的 `IsaacReadIMU` 有輸出 `orientation`, 那是模擬器直接給的車體世界姿態,
不是積分出來的, 沒有漂移。IMU sensor 掛在 `/World/small_car/Cube` 底下且旋轉是
單位矩陣, 所以**它就等於 `/odom` 發的那個姿態**。

拿它把 yaw 鎖死之後, 掃描比對從三個自由度變成兩個。在一個四面牆都看得到的房間裡,
兩個自由度的最小平方是超定到不能再超定的問題 —— 這是能做到公分級的主因。

真車沒有這種東西 (6 軸 IMU 的 yaw 一定漂)。把 `yaw_source` 換成 `gyro` 就會改成
「陀螺儀積分 + 掃描比對修正 yaw」, 那條路真車能用, 但精度會差一截。這個差距是
真的, 不是參數調得不夠好。

### 3. 每個雷射點各自用它發射瞬間的姿態去轉 (deskew)

SICK multiScan136 是 20 Hz 全掃描 —— 一整圈是 50 ms 累積出來的, 但整則
PointCloud2 只有一個時間戳。這台車原地可以轉到 20 rad/s 以上, 那 50 ms 裡車子
會轉超過 50 度, 掃描圖形整個被抹開, 不補償的話配準必錯。

`car.usd` 裡 sensor 設了 `skipDroppingInvalidPoints=1`, 所以沒打到東西的射線也
會留在陣列裡 —— 點的**索引**因此對得上發射順序, 也就對得上時間。節點用索引比例
把每個點的發射時刻還原出來, 再分時間桶各查一次 IMU 姿態。桶數跟著轉速走
(每桶最多讓車子轉 `deskew_bin_angle`, 預設 0.02 rad): 停著時 1 個桶就夠, 打轉
20 rad/s 時會用到 50 個。固定 16 桶在 20 rad/s 下每桶要轉 3.6 度, 實測那個殘留
的抹除量會讓誤差從 0.55 cm 變成 1.41 cm。

Isaac 沒有說那個時間戳是掃描的開始還是結束。與其猜, 節點開頭趁車子**快速旋轉**
的時候把 `start` / `mid` / `end` 三種假設各跑一次配準, 看誰殘差小:

```
掃描時戳校正完成 (end=2.95 cm, mid=13.75 cm, start=13.53 cm) -> 差距不夠明顯, 維持 scan_stamp="end"
```

兩個門檻是踩過坑才加的: 只在 `|gyro_z| > 2 rad/s` 時取樣 (轉得慢的時候三者殘差
會完全打平, 選出來的等於擲骰子 —— 實測選錯會讓誤差從 0.9 cm 變成 7.6 cm), 而且
贏的那個要**明顯**贏過現用值才換。車子一直沒轉那麼快就放棄校正並照實說。

---

## 從 `car.usd` 量到的事實

這些數字寫在 `config/localization.yaml` 裡, **不要憑感覺改**:

| 項目 | 值 | 怎麼來的 |
| --- | --- | --- |
| LiDAR 掛載 | `base_link` 往上 0.200 m, 旋轉是單位矩陣 | `/World/small_car/Cube/World/multiScan136` 的 world transform |
| IMU 掛載 | `base_link` 往上 0.075 m, 旋轉是單位矩陣 | `/World/small_car/Cube/Imu_Sensor` |
| LiDAR 型號 | SICK multiScan136: 20 Hz, 10800 點/圈, 0.05~60 m, 仰角 -22.5°~+42.5°, 測距精度 0.02 m | `SICK_multiScan136.json` |
| 房間 | 內部 x∈[-5,5], y∈[-3,3], 牆高 1 m, 三根 r=0.5 的柱子 | `/World/Room` 的 world bounds |
| topics | `/lidar/point_cloud`, `/imu`, `/clock`, `/odom` (ground truth) | OmniGraph 節點 |
| `/odom` 的原點 | 車子按 Play 當下的位姿; `car.usd` 裡就是世界原點 | `IsaacComputeOdometry` |

**注意 `base_link` 的軸向**: 四個輪子的位置顯示這台車的**車頭朝 -Y、左邊是 +X**,
不是 ROS 慣例的 x 朝前。`/odom` 發的就是這個軸向的姿態, 所以這個 package 也照用,
兩邊才對得起來。之後要接 nav2 的話這件事要處理 (在 USD 裡把車轉 90 度, 或多插一層
`base_link -> base_footprint` 的 static TF)。

`Cube` 上有非等比縮放 `(0.2, 0.3, 0.075)`, 感測器 prim 繼承了它。實測 Isaac 的
RTX LiDAR 忽略這個縮放 (點雲是公制的、沒有變形), 所以不影響 —— 但如果哪天你發現
點雲的尺度不對, 這裡是第一個要看的地方。

---

## 精度

### 端到端 (整條管線, 跟 ground truth 逐點比對)

用 `fake_isaac` (照 car.usd 規格合成的資料源, 見下節) 跑, `localization_eval`
逐點跟真值比對, 丟掉開頭 60 筆初始化過渡:

| 情境 | 位置 RMS | 平均 | p95 | 最大 | yaw RMS |
| --- | --- | --- | --- | --- | --- |
| 8 字形行駛, 最高 ~2 m/s (走了 225 m) | **1.02 cm** | 0.90 | 1.77 | 3.11 | 0 (直接用 IMU) |
| 原地打轉 20 rad/s | **0.55 cm** | 0.48 | 0.99 | 2.01 | 0 |
| `yaw_source:=gyro` (真車路線), 8 字形 | **0.89 cm** | 0.79 | 1.50 | 2.23 | 0.28° |
| 改用 slam_toolbox 建的地圖 (對齊原點後) | 2.25 cm | — | — | 6.46 | 0 |

最後一列是「地圖品質」的代價: 同一套定位程式, 只是把地圖從「USD 幾何切出來的」
換成「SLAM 建出來的」, 誤差就從 1.0 cm 變成 2.25 cm。實體環境沒有第一種選項,
但在模擬裡沒有理由不用。

另外兩個模式 (沒有既有地圖, 所以位置本來就在自己的座標系裡, 下面是**扣掉常數
偏移之後**的值 —— 那才是「有沒有在漂」):

| 模式 | 開了 ~50 m 之後 |
| --- | --- |
| `odometry` (滾動子圖) | RMS 3.49 cm, 最大 19.6 cm |
| `mapping` (地圖一直長大) | RMS 2.23 cm, 最大 6.4 cm |

`odometry` 比 `mapping` 差是預期的 —— 它刻意只記得最近 15 個關鍵幀, 換來的是
「不會被遠處的舊地圖拉歪、輸出連續不跳」, 那正是 `odom -> base_link` 這一層要的
性質。全域一致交給 slam_toolbox。

運動補償的效果 (原地打轉, 其他條件相同):

| 轉速 | 有補償 | 關掉補償 |
| --- | --- | --- |
| 8 rad/s | 0.37 cm | 10.99 cm |
| 20 rad/s | 0.55 cm | **294 cm** (完全失效) |

殘差 (每個雷射點到地圖的平均距離) 穩定在 2.4~3.0 cm, 那就是 2 cm 測距雜訊的
量級 —— 也就是說配準已經貼到雜訊底了, 再調參數也不會更好。

### 離線 (只驗地圖與配準, 不需要 ROS)

`python3 test/test_matcher.py` —— 用房間真實幾何合成 675 點的掃描, 每個測距加
2 cm 高斯雜訊 (對應 multiScan136 規格書的 `rangeAccuracyM`):

| 情境 | 結果 |
| --- | --- |
| 精配準, yaw 由 IMU 給定 | 平均 0.17 cm, 最大 0.36 cm |
| 精配準, yaw 也要解 | 位置平均 0.21 cm, yaw 平均 0.040° |
| 全域定位, 不給初始位姿 | 12/12 次成功, 平均 0.20 cm, 18 ms |
| 全域定位, 連 yaw 都不知道 | 5/5 次成功, < 0.1° |

### 在真的 Isaac 上會不會一樣

上面的數字是用合成資料量的, 它涵蓋了地圖、配準、運動補償、時間對齊這幾件事,
但涵蓋不到 Isaac 那邊的實際行為 (RTX LiDAR 的雜訊模型、fullScan 的實際時戳語意、
各 publisher 的時鐘)。**要知道真正的數字, 開 `evaluate:=true` 自己量。**
節點每 5 秒會印一行, Ctrl-C 會印總結。

---

## fake_isaac —— 不開 Isaac 也能驗

```bash
ros2 run car_localization fake_isaac                                    # 8 字形行駛 (照腳本)
ros2 run car_localization fake_isaac --ros-args -p motion:=spin -p spin_rate:=20.0
ros2 run car_localization fake_isaac --ros-args -p motion:=drive        # 可以手動開
```

`motion:=drive` 會訂 `/joint_command`、發 `/joint_states`, 用從
`car_run_data/sim_data.csv` 回歸出來的模型跑物理 (`a ≈ 0.34×throttle`,
`α ≈ 0.57×steer`)。整條「遙控 -> 建圖 -> 存圖 -> 定位」的流程可以完全不開 Isaac
先跑過一遍。它的輪速是理想無滑移的, 所以打滑情境下會比實際樂觀。

它照 `car.usd` 的規格合成 `/clock`, `/imu`, `/lidar/point_cloud` (10800 點) 與
ground truth `/odom`, 而且**一整圈掃描是在 50 ms 內用每個點各自那一刻的車體位姿
產生的** —— 所以運動抹除是真的存在的, 拿來驗運動補償有沒有做對。

用它的理由: 開著 Isaac 除錯很痛苦, 點雲不對可能是外參錯、可能是 z 濾波錯、
可能是時戳對不上, 而你沒有真值可以逐項比對。這個節點每一項都是已知的, 可以把
「定位演算法有問題」跟「Isaac 那邊有問題」分開。

## 常用參數

全部參數與說明在 `config/localization.yaml`。最常動的幾個:

```bash
# 換一張地圖
ros2 launch car_localization localization.launch.py \
    map_path:=/workspaces/src/car_localization/maps/room.npz

# 真車路線: 不吃 IMU 的絕對姿態, yaw 由陀螺儀積分 + 掃描比對修
ros2 launch car_localization localization.launch.py yaw_source:=gyro

# 在 rviz 裡看配準結果疊在地圖上
ros2 launch car_localization localization.launch.py publish_debug_cloud:=true

# 誤差逐點存檔
ros2 launch car_localization localization.launch.py evaluate:=true \
    csv:=/workspaces/car_run_data/loc_eval.csv
```

服務:

```bash
ros2 service call /car_localizer/relocalize std_srvs/srv/Trigger   # 重新全域定位
ros2 service call /car_localizer/save_map   std_srvs/srv/Trigger   # 建圖模式存檔
```

---

## 內建的 mapping 模式

`launch/mapping.launch.py` —— hector 式建圖, 不需要 slam_toolbox。地圖會自己長大,
不用事先知道房間多大。

```bash
ros2 launch car_localization mapping.launch.py \
    map_save_path:=/workspaces/src/car_localization/maps/room.npz
```

**先確認你要的是這個**:

| 想做的事 | 用哪個 |
| --- | --- |
| `car.usd` 這個模擬場景 | 都不要用, 跑 `./scripts/make_map_from_usd.py` (零建圖誤差) |
| 實體環境, 要一張長期用的地圖 | `slam.launch.py` (有回環偵測) |
| 小房間, 只想快速拿一張堪用的圖 | 這個 |

它沒有回環偵測。存完一定要親眼看過:

```bash
python3 -m car_localization.gridmap show maps/room.npz
```

牆要是細線。同一面牆出現兩條平行線 = 建圖時位姿漂了, 重來。

**建圖要從靜止的起點開始。** 第一幀是拿來定義地圖原點的, 它沒有東西可以對,
所以車子當下的速度估計還是 0 —— 邊開邊開始建圖的話, 第一幀會被運動抹開幾公分,
而之後所有幀都會對齊到那道歪掉的牆上。`map_insert_max_residual` 會擋掉後續殘差
過大的幀, 但擋不住第一幀。

---

## 疑難排解

**「找不到地圖檔」** — 先在 host 上跑 `./scripts/make_map_from_usd.py`, 再重新
`colcon build` (地圖是跟著 package 裝到 share 底下的)。

**「LiDAR 與 IMU 的時間源對不上」** — Isaac 各 publisher 用的時間源不一定同一個,
反覆 Stop/Play 之後會差開幾百秒。節點會直接報錯並印出估計的偏移量。
修法: 在 host 上跑 `./scripts/fix_car_usd_lidar.py` (它會把所有
`resetOnStop` / `resetSimulationTimeOnStop` 打開), 然後重新載入場景。

**「高度帶裡只剩 N 點」** — `lidar_translation` 跟 USD 裡的實際掛載高度對不上,
或 `z_min`/`z_max` 設錯。車子的世界高度應該是 0.200 m。

**「配準不合格」一直出現** — 通常是地圖跟現在的場景不一樣 (你改了 `car.usd` 但
沒重新產生地圖), 或者車子被撞到房間外面去了。連續失敗 10 次會自動重新全域定位。

**位置誤差是一個不會變的常數** — 那不是定位在漂, 是地圖原點跟 `/odom` 原點差一個
平移 (你把車子的起始位置從世界原點挪開了)。`evaluate` 節點會另外報「扣掉常數偏移
之後」的誤差, 用來分辨這兩件事。

---

## 檔案

```
car_localization/
├── gridmap.py     佔據格點地圖 + 距離場 (存 / 讀 / 檢查, 跟 ROS 無關)
├── pointcloud.py  PointCloud2 解析
├── imu_track.py   IMU 時間序列緩衝 (姿態內插、陀螺儀零偏)
├── matcher.py     掃描對地圖配準 (粗搜尋 + Huber Gauss-Newton/LM)
├── localizer.py   主節點 (localize / odometry / mapping 三種模式)
├── evaluate.py    跟 ground truth 比對誤差 (ros2 run car_localization localization_eval)
└── fake_isaac.py  照 car.usd 規格合成感測器資料, 不開 Isaac 也能驗
launch/localization.launch.py        定位
launch/slam.launch.py                slam_toolbox 建圖 (實體環境用這個)
launch/viz.launch.py                 開 WebSocket 給 Foxglove 連
launch/mapping.launch.py             內建的 hector 式建圖
config/localization.yaml             定位參數
config/slam_toolbox.yaml             建圖參數 (從 wildbot 驗證過的那份改的)
config/foxglove_layout.json          Foxglove 版面 (可匯入)
maps/car_usd.npz                     從 car.usd 切出來的地圖 (.pgm/.yaml 同名)
test/test_matcher.py                 離線精度驗證 (不需要 ROS)
../../scripts/make_map_from_usd.py   從 USD 幾何產生地圖 (在 host 上跑)
../car_teleop/                       手動開車 (建圖時要用)
```
