# car_loc_lidar — 方法二: 只用 LiDAR (手動建圖 + 對圖定位)

感測器: **Oradar MS200** —— 2D 單線, 360°, 0.03~12 m, 10 Hz, `/scan` 一圈 450
個 bin (角解析度 0.8°)。**450 是規格書的名目值, Isaac 實際一圈只打到約 300 點**
—— 剩下的 bin 是 `-1`, 下游會濾掉, 對定位沒有影響 (見第 3 節)。
用 `scripts/check_scan.py` 隨時可以量。

**模擬與實體車是同一顆規格、同一個 topic (`/scan`)、同一份參數**,
所以除了 `use_sim_time` 之外不需要為了搬到實體車改任何東西。

分兩個階段, **兩個階段都不吃 IMU、不吃相機**, 也不 import 任何其他 package。

```
階段一 建圖 (手動開一圈)
  /scan (MS200) ─> lidar_odometry (純雷射) ─┬─> TF odom->base_link
                                            └─> /scan_deskewed (運動補償過)
                                                        │
                                            slam_toolbox ┴─> /map + TF map->odom
                                                        │
                                          map_saver_cli ─┴─> maps/room.yaml + .pgm

階段二 定位
  /scan ─> 去畸變 -> scan-to-map 3 自由度配準
  maps/room.yaml ──────────┘
                           └─> /lidar_loc/odom + TF map->base_link
```

> `/scan` 一直是**感測器自己的** topic (模擬與實體車都是), 沒有任何節點會蓋掉它。
> 去畸變過的版本另外發 `/scan_deskewed` 給 slam_toolbox 吃。

## 0. 一次性: 把 car.usd 裡的雷射設成 MS200

```bash
./scripts/setup_oradar_lidar.py --dry-run    # 先看會改什麼
./scripts/setup_oradar_lidar.py
```

改完回 Isaac **重新載入場景** (File -> Open) 再按 Play。

**為什麼需要這一步 (這是最容易做錯的地方):**
`IsaacSensorCreateRtxLidar` 的 `config=` 在 Isaac Sim 5.x **只認註冊過的名字**
(`SUPPORTED_LIDAR_CONFIGS`, 對應 assets server 上的 `.usd`)。給一個 JSON 的
絕對路徑**不會生效** —— 它只在 log 印一行 `Config '...' not found`, 然後退回
預設的 Generic/LidarCore: **128 線、仰角 ±15°、量程 200 m**。prim 一樣叫
`OmniLidar`, 從外觀完全看不出來, 但那不是 2D 雷射。

而 `data/lidar_configs/` 那個 JSON 資料夾, 在 `extension.toml` 裡註明是給
「(deprecated) camera-based Lidar」用的。**新的 `OmniLidar` prim 是直接讀 USD
屬性 `omni:sensor:Core:*`**, 所以腳本做的就是把 `config/oradar_ms200.json` 的
profile 寫進屬性, 順便把 ActionGraph 也改對:

| 改了什麼 | 從 | 到 | 為什麼 |
| --- | --- | --- | --- |
| 發射器 | 128 顆, 仰角 -15~+15° | 1 顆, 仰角 0 | **少了這個就不是 2D 雷射** |
| 量程 | 0.3~200 m | 0.03~12 m | MS200 規格 |
| 取樣率 | 36000 Hz | 4500 Hz | = 450 點/圈 x 10 Hz |
| render product | 675x16 | 1x1 | 射線圖樣由 profile 決定, render product 只是一塊畫布 (Isaac 自己的 `rtx_lidar.py` 範例就是 `[1, 1]`); 675x16 是上一顆 SICK multiScan136 留下來的 |
| helper type | `point_cloud` | `laser_scan` | 真的 MS200 驅動發的就是 LaserScan |
| topic | `lidar/point_cloud` | `scan` | 跟實體車一致 |
| `fullScan` | False | **True** | **`laser_scan` 也吃這一項**。關掉的話每則 `/scan` 只有當下 render 批次掃過的 60° 有值, 其餘 bin 填 `-1` —— 見下面 |

**`laser_scan` 一樣會「每個 render frame 發一小片」—— `fullScan` 一定要開。**
`IsaacComputeRTXLidarFlatScan` 的文件寫著 *"Output execution triggers when lidar
sensor has accumulated a full scan"*, 曾經據此判斷 laser_scan 不需要 `fullScan`。
**那是錯的, 已經量過**: 關掉時 60 FPS 算繪配 10 Hz 轉速 = 一批 60°, 450 個 bin
只有 150~200 個是真的, 空洞位置每幀移動 60° (30 幀的有效 bin 聯集 67%, 交集 1%)。
訊息長度、角度欄位、`scan_time`、發布頻率全都是對的, **只有內容是破的**。

跑完腳本一定要在 Isaac **File → Open 重新載入場景** —— USD 屬性是載入 stage
時才讀的, `Stop → Play` 只是重播時間軸, 不會重讀。改完用這個確認:

```bash
docker exec ros2_node bash -lc 'r && python3 /workspaces/scripts/check_scan.py'
```

規格對不上你手上那顆的話, 改 `config/oradar_ms200.json` 再重跑腳本就好
(腳本是冪等的, 已經對的東西不會動)。

## 1. 用法

**階段一 —— 手動開車建圖**

```bash
ros2 launch car_loc_lidar mapping.launch.py
```

另開一個 terminal 用鍵盤開車 (`-it` 是必要的, 鍵盤需要真的 TTY):

```bash
docker exec -it ros2_node bash -lc 'r && ros2 run car_teleop teleop_key'
```

開的時候四件事:

* **慢慢開。** MS200 只有 10 Hz、一圈約 300 點, 比之前的雷射稀疏又慢。
* **柱子後面、每個角落都要繞到**, 沒繞到的地方地圖上就是空的。
* **要繞回起點**, 回環偵測才有東西可以閉。
* **MS200 只看得到 12 m** —— 大場地要多繞幾趟, 不能站在中間轉一圈就算數。

存檔:

```bash
ros2 run nav2_map_server map_saver_cli -f /workspaces/src/car_loc_lidar/maps/room
```

**階段二 —— 定位**

```bash
colcon build --symlink-install && source install/setup.bash   # 地圖跟著 package 裝
ros2 launch car_loc_lidar lidar_loc.launch.py
ros2 launch car_loc_lidar lidar_loc.launch.py evaluate:=true
```

不用給初始位姿 —— 節點會自己在整張地圖裡找 (位置 x 角度)。追丟了也會自己重找。
想手動給 (例如長方形房間鎖到 180 度反方向): 發 `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`,
Foxglove 3D 面板的 "Publish pose estimate" / RViz 的 2D Pose Estimate), 節點就從那裡開始局部配準
(參數 `initialpose_topic`, 空字串 = 不訂閱; 2026-11-18 為 nav_mpc_pkg 加的)。融合 (`fusion_localizer`) 不會跟著跳,
要另外叫 `/fusion_localizer/reset` —— nav_mpc_pkg 的管理節點會自動做。

**實體車**: 同一顆 MS200 規格, 只要關掉模擬時鐘。

```bash
ros2 launch car_loc_lidar lidar_loc.launch.py use_sim_time:=false
```

檢查地圖 (不需要 ROS):

```bash
python3 -m car_loc_lidar.gridmap show src/car_loc_lidar/maps/room.yaml
```

會印出俯視 ASCII 圖。**看一眼** —— 牆要是細線、形狀要像那個場地。

## 2. 離線驗證 (不需要 ROS, 不需要 Isaac)

```bash
cd src/car_loc_lidar && python3 test/test_matcher.py     # 約 25 秒
```

造一個房間 -> 對真實表面做 sphere tracing 模擬帶畸變的掃描 -> 跑整條追蹤迴圈
(含掃角度重試與全域重定位) -> 跟真值比。**下面每一張表都是這個腳本印出來的**,
改了參數想知道值不值得就重跑它。

## 3. 換成 MS200 之後的變化

點數少一半、慢一倍、量程只剩三分之一。8 字形行駛 200 幀:

| 感測器 | 有補償 | 沒補償 |
| --- | --- | --- |
| 舊的 3D (720 點, 20 Hz, 40 m) | 3.69 cm / 0.83° | 5.86 cm / 2.82° |
| **MS200 (450 點, 10 Hz, 12 m)** | **4.99 cm / 2.91°** | 10.53 cm / 5.43° |

位置差了約 1.3 cm, yaw 差了 2 度 —— 這是換感測器的代價, 不是設定沒調好。

**實際點數是 300 不是 450, 但在正常操作範圍內沒有差別。** 上面那張表跟第 4 節
都是 bench 用名目的 450 條射線跑的 (`test/test_matcher.py` 的 `N_BEAM`)。拿
300 重跑一次對照:

| | 8 字形 300 幀 | 自旋 8 rad/s | 自旋 10 rad/s |
| --- | --- | --- | --- |
| 450 點 | 5.03 cm / 2.96° | 3.51 cm / 0.05° | 4.90 cm / 3.66° |
| **300 點 (實際)** | **5.02 cm / 2.96°** | **3.51 cm / 0.05°** | **168.74 cm / 176.72°** 追丟 |

**8 rad/s 以內兩者一模一樣**, 只有超過追蹤上限之後 300 點比較容易鎖到 180 度
—— 而那個區間本來就是「不可預測」的。所以**不需要**為了補滿 450 去動
`config/oradar_ms200.json` 的 `reportRateBaseHz`。

**運動補償變得更重要。** MS200 一圈是 **100 ms**, 比之前的 3D 雷射長一倍, 也就是
車子在一圈裡轉過的角度是以前的兩倍。實測 (原地自旋):

| 角速度 | 有補償 | 沒補償 |
| --- | --- | --- |
| 5 rad/s | 3.51 cm / 0.05° | 11.28 cm / 14.44° |
| 8 字形行駛 | 5.04 cm / 2.96° | 10.23 cm / 5.43° |

## 4. 轉太快會追丟 —— 上限在哪

純雷射沒有角速度感測器,「下一幀轉到哪」只能用上一段外推。原地自旋 150 幀:

| 角速度 | 一圈轉過 | 位置 RMS | yaw RMS | |
| --- | --- | --- | --- | --- |
| 1 rad/s | 5.7° | 3.49 cm | 0.04° | |
| 3 rad/s | 17.2° | 3.50 cm | 0.04° | |
| 5 rad/s | 28.6° | 3.51 cm | 0.05° | |
| 8 rad/s | 45.8° | 3.51 cm | 0.05° | |
| 9 rad/s | 51.6° | 175.37 cm | 178.37° | **追丟** |
| 10 rad/s | 57.3° | 4.90 cm | 3.66° | |
| 12 rad/s | 68.8° | 156.55 cm | 150.89° | **追丟** |

**~8 rad/s 以內穩定; 再上去就不可預測** —— 9 rad/s 會鎖到 180 度反方向,
10 rad/s 又正常。長方形房間對 180 度旋轉幾乎是對稱的 (柱子那種小特徵撐不住),
鎖住之後殘差看起來還很漂亮, **回不來**。

這是 10 Hz 雷射的物理極限, 不是參數問題。`car_teleop` 的轉向上限預設
**1.2 rad/s** —— 離這裡還有 7 倍餘裕, 正常操作碰不到。

因為這件事, MS200 的設定檔把 **`sweep_on_fail` 預設關掉**:

| 突然跳到 | 不重試 | 掃角度重試 |
| --- | --- | --- |
| 4 rad/s | 3.51 cm / 1.38° (0 次失敗) | 一樣 (從沒觸發) |
| 8 rad/s | 4.59 cm / 4.10° (0 次失敗) | 一樣 (從沒觸發) |
| 9 rad/s | 85.57 cm / 100.29° | **160.32 cm / 167.87°** |

可追蹤範圍內配準根本不會失敗, 重試永遠不會被觸發; 超出範圍時它用一個蓋不住
真值的範圍去掃, 反而更容易鎖到 180 度。
**換回 20 Hz 的 3D 雷射 (`input_type:=pointcloud`) 時請打開** —— 那邊是大勝:
20 rad/s 自旋 200 幀, 不重試 114.60 cm / 122 幀失敗, 有重試 3.53 cm / 0 幀失敗。

## 5. 掃描的時間順序 —— 一個很賊的坑

Isaac 的 `laser_scan` 走 `IsaacComputeRTXLidarFlatScan`, 它的輸出文件寫著:

> *"Linear depth measurements from full scan, **ordered by increasing azimuth**"*

**排序是按方位角, 不是按時間。** 而 MS200 是 **CW** 旋轉 —— 方位角隨時間遞減,
所以陣列的索引順序跟發射順序是**相反的**。拿索引直接當時間, 運動補償會補到
反邊去, 而且症狀很賊: **直線走完全正常, 一轉彎殘差就變兩倍。**

所以 `time_order` 是 `reverse`、`scan_stamp` 是 `end`。

### ⚠ `auto_scan_stamp` 的投票結果不可以直接採信

> **2026-09-11 起預設關掉** (`lidar_loc.yaml` 與節點的預設值都是 `false`), 直接用
> 建圖驗收過的 `reverse/end`。那天跟 `collect_all` + 摩擦力腳本一起跑又投錯一次
> (w0=11 自旋那 15 幀投給 `forward/end`), 之後每次轉彎補償都補反, 慢速移位也一直
> 配準失敗, 追丟 67.6%; 關掉之後同一個腳本 17.7%, 再加鎖死偵測 2.9%。
> 下面是當初為什麼不能信它的紀錄 —— 要重新打開校正之前先讀完。

`auto_scan_stamp` 開著的時候, 節點會在車子轉得夠快的前 15 幀把
(頭/中/尾) x (順/逆) 六種組合試一遍, 用配準殘差投票並印出來:

```
掃描時序校正完成 (15 幀, 殘差由小到大):
  forward/mid 3.79 cm, forward/start 4.00 cm, forward/end 4.12 cm,
  reverse/mid 5.03 cm, reverse/start 5.08 cm, reverse/end 5.12 cm
  -> 用 forward/mid
```

2026-09-08 它就是這樣投的 —— 三個 `forward` 全部低於三個 `reverse`, 兩群不重疊,
看起來毫無懸念。**照它改成 `forward/mid` 之後重新建圖, 地圖直接爛掉:**

| | `reverse/end` | `forward/mid` |
| --- | --- | --- |
| 佔據範圍 | **10.15 x 6.30 m** | 11.75 x 11.15 m (房間被剪切成菱形) |
| 到真實牆面 中位數 | **3.7 cm** | 45.4 cm |
| < 10 cm | **75.5%** | 20.0% |
| > 50 cm | **5.7%** | 46.6% |

(真實房間是 10.00 x 6.00 m, 來自 `car.usd` 的牆面幾何。當時用來比對的線段檔
`car_usd_refined_segments.yaml` 放在已經刪掉的 `car_localization/maps/`, 而且從來
沒進過 git, 已經找不回來了。)

**為什麼會投錯**: 那次校正是跟摩擦力測試腳本 (`bringup_pkg my_launch`) 一起跑的,
車子自旋到 **14 rad/s**, 遠超過第 4 節量出來的 8 rad/s 追蹤上限。超出範圍的幀
配準本來就是壞的, 拿它們的殘差投票沒有意義。`stamp_calib_omega` 只有下限沒有
上限, 擋不掉這件事。

**結論 —— 這一節唯一該記住的事:**

* 要校正就用 **teleop 慢慢開**, 不要跟摩擦力腳本一起跑
* 投票結果只是**線索**, 不是答案。**唯一的驗收是重建一次地圖再量佔據範圍**
  (應該接近 10.00 x 6.00 m), 殘差小不代表地圖對
* `lidar_odometry` (建圖端) 沒有 `auto_scan_stamp`, 那兩個值是寫死的。
  **不要照投票結果去動它** —— 這一端錯了是把整張地圖建歪, 比定位錯貴得多

驗收地圖的量法:

```bash
python3 -m car_loc_lidar.gridmap show src/car_loc_lidar/maps/room.yaml
```

## 6. 沒有 IMU 之後改變的兩件事

**yaw 要自己解 —— 配準是 3 自由度。**
有 IMU 的做法可以把 yaw 鎖死, 問題退化成超定的兩自由度最小平方。只有雷射的時候
yaw 必須由幾何自己撐出來, 而 yaw 誤差會透過「距離 x 角度」放大成位置誤差 ——
**MS200 只看得到 12 m, 12 m 外的牆差 0.5 度就是 10.5 cm**。
`evaluate` 會直接把「yaw 誤差換算成位置誤差」印出來。

**預測只剩等速模型。** 上一節那張表就是這件事的代價。

## 7. 精度上限由地圖解析度決定

| 地圖解析度 | 位置誤差 |
| --- | --- |
| 10 cm | 6.91 cm |
| **5 cm (預設)** | **3.45 cm** |
| 2.5 cm | 1.73 cm |
| 1.25 cm | 0.91 cm |

誤差 ≈ 0.7 x 解析度, 而且它是**系統性偏移**不是抖動 —— 整張地圖的格點是同一組,
對面的兩道牆會往同一個方向偏, 平均再多幀也消不掉。想更準就把
`slam_toolbox.yaml` 跟 `submap_resolution` 的 `resolution` 一起改小。

除此之外, SLAM 建的地圖本來就帶著建圖時的位姿誤差 —— 定位再怎麼準也不會超過
地圖本身的準度。

## 8. 為什麼建圖是 slam_toolbox

自己長地圖 (hector 式) 在小房間裡很好用, 但它**沒有回環偵測**: 走遠再繞回來時
累積的誤差沒有任何機制分攤掉, 地圖會在接縫處錯開。slam_toolbox 有位姿圖 +
回環偵測 + 全域最佳化, 而且那份參數是 wildbot 實體車 (也是 oradar) 已經驗證
過的。地圖是要長期使用的資產, 值得用那一套。

它缺的那一塊 (`odom -> base_link`) 由本 package 的 `lidar_odometry` 補上 ——
對最近 15 個關鍵幀疊出來的**滾動子圖**配準, 產生局部準確、連續不跳的里程計。
掃描對掃描的誤差是純隨機遊走, 走幾公尺就明顯歪。

## 9. 出事的時候

| 症狀 | 通常是 |
| --- | --- |
| `/scan` 沒有東西 | Isaac 沒按 Play; 或還沒跑 `scripts/setup_oradar_lidar.py` (那時候 topic 還叫 `/lidar/point_cloud`) |
| 掃描在 Foxglove 上是**幾段斷開的弧**, 每幀位置還會滾動 | `fullScan` 沒開。每則 `/scan` 只有當下 render 批次的那 60° 有值, 其餘填 `-1`。訊息長度/角度/頻率全都是對的, 只有內容是破的。跑 `scripts/check_scan.py`, 它會直接判給你看。修法: `setup_oradar_lidar.py` 之後在 Isaac **File → Open 重載場景** (Stop/Play 不夠, USD 屬性是載入時才讀的) |
| 地圖中央有一個**半徑 1 m 的圓環**, 而且跟著車走 | `-1` 沒被濾掉, 被算成 `(-1·cosθ, -1·sinθ)` = 半徑 1 m、方向反轉 180° 的點。`valid_mask` 用 `r²` 判斷, 平方之後正負號就沒了直接放行。已在 `scan.py` 的 `laserscan_to_xyz` 修掉 (`isfinite(r) & (r > 0)`); 症狀是每幀 2/3 的點變成一個完美的圓, 對 yaw 零約束卻佔壓倒性權重 —— 每幀 yaw 雜訊 1.32°, 建圖時累積成同心圓 |
| 啟動時**看到**「掃描時序校正完成」 | `auto_scan_stamp` 被打開了 (預設 `false`)。它投出來的結論不能直接用, 見第 5 節; 手動校正時 `stamp_calib_omega` 要低於 `car_teleop` 的轉向上限 (`max_angular`, 預設 1.2), 而且只用 teleop 慢慢開 |
| 一圈只有 26~292 點, 60 Hz | 雷射還是舊的 3D 設定。跑 `./scripts/setup_oradar_lidar.py --dry-run` 看它說什麼 |
| 點雲是立體的 / 有仰角 | `config=` 給了 JSON 路徑, 沒生效, 現在是 128 線的 Generic。同上 |
| `找不到地圖` | 還沒建圖, 或建完沒 `colcon build`。給 `-p map_path:=...` 也行 |
| 直線走正常, 一轉彎殘差變兩倍 | `time_order` / `scan_stamp` 不對。確認 `lidar_loc.yaml` 是 `reverse` / `end` 而且 `auto_scan_stamp: false` |
| yaw 突然差 180 度而且回不來 | 轉太快 (>8 rad/s), 鎖到了對稱解。開慢一點; 確認 `sweep_on_fail: false` |
| 狀態寫「定位中」, 但停著時殘差 3~7 cm / inlier 50~77% (正常是 0.2 cm / 100%) | 鎖在 180 度對稱解, 配準不會失敗所以 `max_failures` 永遠不觸發。`lock_*` 參數 (鎖死偵測) 會在幾乎沒在轉時連續 15 幀偏高就**整張地圖**重新定位, 記錄會印「疑似鎖在對稱解」。2026-09-11 沒有這個時, 自旋 12 rad/s 之後鎖了 131 s 到結束 |
| log 說「掃描斷了 X s, 重新全域定位」 | `/scan` 停了超過 `scan_gap_reset` (1 s)。等速外推的預測已經不知道錯到哪裡, 拿它配準會收斂到錯的局部解而且 sigma 很小 (2026-11-03: 斷 16 秒後那一幀錯 4.8 m, sigma 1.3 mm), 所以直接重新全域定位。全域定位剛成功的 `settle_scans` (10) 幀, 回報的 sigma 至少 `settle_sigma` (5 cm) —— 融合與 `eval_loc_csv.py` 都會把它們當成「追丟」 |
| 誤差是一個不會變的常數 | slam_toolbox 的地圖原點是車子按 Play 那一刻的位置, 跟 `/odom` 差一個平移是**正常的**。看 `evaluate` 的「常數偏移」那行, 它會印出修正指令 |
| 沿著長走廊會滑 | 幾何退化: 只看得到平行的兩面牆時, 沿牆方向不可觀測。12 m 的量程讓這件事比以前更容易發生 |
| 大場地定位不穩 | MS200 只看得到 12 m。空曠處看不到足夠的牆就撐不住 —— 這是感測器的定義域 |

## 10. 地圖上沒有的東西 (箱子、人) —— 2026-11-18

**症狀** (Isaac, nav_mpc 放一個 0.4 m 箱子在車子前方 1.4 m): inlier 92% -> 82%, 殘差 1.2 -> 2.5 cm。
箱子的點離牆很遠, 本來就是外點, 但 Huber 對外點還是有**常數拉力**, 佔掃描 15% 的箱子把位姿拉偏幾公分
(殘差變大就是這個偏移)。接著:
* 融合的 `lidar_sigma_max` (2.5 mm) 一直擋 -> 1 s 沒有絕對量測 -> MPC 停車 (「絕對量測斷了」)
* 鎖死偵測 (inlier < 85%) 每 3 秒誤判一次 -> 整張地圖全域定位 -> 第 4 次**真的鎖到 180 度反方向**

**修法** (兩件事, `matcher.py`):
1. `occluder_dist` (0.15 m): 第一次配準後, 落在**房間內部** (`GridMap.interior_mask`: 牆圍起來的最大空區)
   而且離牆 > 0.15 m 的點拿掉, 再配一次; 殘差 / inlier / sigma 只算剩下的點。牆外、柱子裡面的點不拿
   (位姿錯了點就落在那裡)。拿掉超過 `occluder_max_ratio` (30%) 就不拿 —— 位姿大概錯了, 照舊回報爛指標。
2. 鎖死偵測多一個條件 `lock_see_through` (3%): **穿牆射線**的比例。箱子只會讓量到的比地圖預期**短**;
   量到比預期長 (射線穿過地圖上的牆 / 柱子) 只會是位姿錯了。這個要加, 因為 (1) 把 180 度對稱解裡柱子對不上的
   那幾個點也拿掉了, 殘差 / inlier 就判不出鎖死。

`test/test_occluders.py` (真的 room.yaml, 模擬 MS200, 6 個位姿):

| 情境 | 關 (舊) | 開 |
| --- | --- | --- |
| 沒箱子 | 位置誤差 ≤ 0.42 cm | 一樣 (沒有點被拿掉) |
| 前方 1.4 m 一個箱子 | ≤ 3.94 cm | ≤ 0.54 cm |
| 三個箱子 (一個離牆 0.13 m) | ≤ 11.7 cm, 鎖死誤判 4/6, 融合擋 3/6 | ≤ 1.68 cm, 0/6, 0/6 |
| 鎖到 180 度 | 抓到 **0/6** (這個房間太對稱, 殘差 1.2~1.8 cm / inlier 89~97% 都在門檻內) | 抓到 5/6 (穿牆 3.6~5.8%) |
| 正確位姿的穿牆比例 | - | ≤ 0.7% |

漏的那一個是從 (3.8, -0.5) 看不到柱子, 兩個方向的掃描一樣, 那一刻本來就分不出來。
ROS 實測 (fake_world 的掃描, 車子停著、前方 1.4 m 箱子): 關 inlier 94% / 殘差 1.0 cm; 開 100% / 0.55 cm
(拿掉 ~22 點), 穿牆 0%, 沒有誤判; 每幀多 ~1 ms。**穿牆比例只在模擬的理想掃描量過**, Isaac / 實體車的
基準值看 status 那行的「穿牆 x%」, 正常應該遠低於 3%。

**第二輪 (同一天, Isaac 四個箱子, LiDAR 整個跑掉)**: Isaac 的 RTX 雷射一圈只有 ~210~300 個有效點
(一半射線回 -1), 四個箱子就佔 21~39%。第一版「先配準、再分類」從真值起配都會被箱子拖走 26 cm, 而且超過
30% 上限的幀整個退回舊行為 -> 配準失敗 -> 全域定位收了一個錯 2.4 m 的解 (拿掉 59 點後 inlier 100%)。改成:
* 在**起點 (預測位姿)** 先用 0.3 m 門檻拿掉, 配完再用 0.15 m 重新分類、再配一次; `occluder_max_ratio` 0.3 -> 0.5
* 結果 inlier < 50% 時再用全部的點配一次, 取 inlier 高的 (起點已經錯很多時救得回來)
* 全域定位不拿點, 而且穿牆 > `lock_see_through` 的解不收

錄下來的 Isaac 掃描 (57 幀, 4 個箱子, 對 GT): 預測偏 0~15 cm / 0~5° 時新的 57/57 配準成功、誤差中位數 0.7~0.9 cm、
最大 2.2 cm、0 次鎖死誤判、穿牆 0%; 舊的一半失敗, 「成功」的中位數錯 1.9 m。從那個錯解起配: 拿掉點後 inlier 可以
到 100%, 但穿牆 4.7~20%, 6/6 抓得到。**還沒解決**: 箱子在視野裡時全域定位找到的還是錯解 (現在會被擋掉, 但就一直
找不到); 追丟之後要先把箱子拿掉或用 Foxglove 給初始位姿。

限制: 貼著牆 (< 0.15 m) 的東西還是會被當成牆; 地圖本身跟場地不合 (家具搬過) 的地方會被當成箱子拿掉
(只要不超過 30% 就沒差, 超過就要重建圖)。`occluder_dist:=0.0 lock_see_through:=0.0` = 舊的行為。
