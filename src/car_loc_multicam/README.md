# car_loc_multicam — 方法六: 8 台斜拍環境相機 + 3D 車型 + 視線量測

```
/env_cam_0..7/rgb ─> multicam_detector ─> /env_cam_i/car_bbox (YOLO 框, 每台各自)
   (Isaac / Pi)        YOLO + ROI 縮放            │
                          ^                       v
                          │      multicam_fusion = car_loc_fusion 的 FusionLocalizer
     /multicam_loc/odom ──┘        + 每台相機的框各自當 EKF 量測 (量測模型 = 視線)
     (預測車在畫面哪裡)             /imu + /joint_states 遞推, 倒帶重放補延遲
                                          │
                                          v
                 /multicam_loc/odom, /multicam_loc/pose, TF map->base_link, /multicam_loc/markers
```

對應那張方法表的兩條:

| | 做法 | 在哪 |
| --- | --- | --- |
| **B** 用車的 3D 模型反推 | 車的凸包 (從 USD 匯出) 投影到相機, 找讓「預測框」最吻合 YOLO 框的 (x, y) | `car_model.py` (`predict_bbox`, `solve_position`, `initial_pose`) |
| **C** 多相機融合 | 每台相機的框**各自**進 EKF; 量測是框的四個邊 (px), H = d框/d(x, y, yaw) 就是「視線」 | `multicam_ekf.py` (`MultiCamEkf`, 繼承 `FusionEkf`) |

遞推、倒帶重放、逃生口、時鐘偵測、IMU/輪速、發布全部是 import `car_loc_fusion` /
`car_loc_wheel` 來的; YOLO 包裝與 `best.onnx` 用 `car_loc_camera` 的。launch 會先讀
`car_loc_fusion/config/fusion_loc.yaml` 再蓋上本 package 的設定, 所以遞推參數兩條路線
永遠同一組。

**方法五 (`car_loc_fusion`) 跑的就是這裡的節點**, 只差多吃 LiDAR、輸出在 `/fusion_loc/*`:
日常用那一條, 這一條留著回答「LiDAR 值多少」。

為什麼是「框當量測」而不是「先算 (x, y) 再融合」: 一台斜拍相機給的資訊是**沿視線拉長的
橢圓** (左右邊 = 方位角, 很準; 上下邊 = 俯角換成距離, 差很多), 壓成 car_loc_fusion 那種
圓形 sigma 不是過度自信就是丟資訊。框直接進 EKF 的話 H 自己就是那個橢圓, 8 台從不同方向
看疊起來就是一個小圓 —— 「不同方向的偏差互相抵銷」; yaw 也在狀態裡, yaw 的誤差會正確地
傳到位置上 (方法 B 單獨用時 yaw 錯位置就跟著錯)。細節見 `multicam_ekf.py` 開頭。

## 用法

**1. Isaac 要發 8 台相機的影像。** `kros_car_new.usd` 已經有了 (`/World/Camera_ActionGraph/camera_0*`,
發 `/env_cam_i/rgb` + `camera_info`, 1456x1088), 開 stage 按 Play 就好。
**不要再跑 `scripts/publish_env_cameras.py`** —— 那是 USD 還沒有發布時的替代品, 現在再跑會變成
每個 topic 兩個 publisher (影像重複)。換一個沒有發布的 stage 時才用得到它。

**2. 跑**

```bash
colcon build --symlink-install --packages-select car_loc_multicam
ros2 launch car_loc_multicam multicam_loc.launch.py                  # 偵測 + 融合
ros2 launch car_loc_multicam multicam_loc.launch.py evaluate:=true   # 加上跟 GT /odom 比
ros2 launch car_loc_multicam multicam_loc.launch.py debug:=true      # /env_cam_i/car_bbox_debug 疊圖
ros2 launch car_loc_multicam multicam_loc.launch.py detector:=false  # 框由別處發 (例如每台 Pi)
```

**新的容器第一次啟動, detector 要先暖機 ~30 s** (log 會印「YOLO 暖機中」; CUDA 在幫這張
Blackwell 卡編 kernel, 快取在容器裡的 `~/.nv/ComputeCache`, 同一個容器之後 < 1 s)。
這段時間沒有任何框、Ctrl-C 也收不掉, 不是當掉。容器重建就要再等一次。

開始前讓車停一秒 (靜止校正)。**不需要給起點** —— 第一組框用方法 B 反推, yaw 用 IMU 的。
跟方法五 (`fusion_loc.launch.py`) 同時開的話, 其中一個要 `publish_tf:=false`, 而且 8 台
相機只要一個偵測節點 (另一邊 `detector:=false`)。`bringup_pkg` 的 `collect_all` /
`friction_test` **已經包含方法六** (`multicam_only`, 共用偵測節點), 用它們的時候不要再開這個。

## 輸出

| topic | 型別 | 內容 |
| --- | --- | --- |
| `/multicam_loc/odom` | `nav_msgs/Odometry` | 主要輸出 (IMU 頻率) |
| `/multicam_loc/pose` | `PoseWithCovarianceStamped` | 同上 |
| `/multicam_loc/markers` | `MarkerArray` | 每台相機一條視線 (綠 = 採用, 紅 = 閘門擋掉) + 那台自己用方法 B 反推的位置 (黃點) |
| `/env_cam_i/car_bbox` | `PolygonStamped` | 偵測結果, 格式見 `bbox_msg.py` (容器裡沒有 vision_msgs) |
| TF | `map -> base_link` | `publish_tf:=false` 可關 |

## Isaac 實測 (2026-10-30)

在一份**我自己的** Isaac 複本 (docker, ROS domain 83, `kros_car_new.usd` 只讀) 上跑整條
產線: Isaac 發 8 台相機 -> `multicam_detector` (YOLO, GPU) -> `multicam_fusion`, 車用
航點繞整個房間 (0.45 m/s, 含原地轉彎), 冷啟動 (沒給起點), 跟 `/odom` 比:

| | 數值 |
| --- | --- |
| 位置誤差 (101 s 模擬時間, n=6049) | **RMS 2.11 cm, 中位 0.76 cm**, p95 5.08, 最大 6.41 cm |
| 常數偏移 | (+0.09, +0.11) cm |
| yaw 誤差中位 | 0.02~0.06° (IMU 的絕對姿態) |
| 起點 (方法 B, 3 台相機) | 誤差約 2 cm, 殘差 1.6 px |
| 框被卡方閘門擋掉 | 102 / 8891 (1.1%) |
| 每台的偵測率 | 60~100% (低的是預測說車不在那台畫面裡而略過, 不是漏偵測) |
| YOLO 推論 | 16 ms / 張 (onnxruntime CUDA) |

依速度分 —— **剩下的誤差幾乎都在「慢速轉彎」, 而且不是相機的問題**:

| 速度 | 只用相機 (同一刻多台反推, yaw 用 GT) | 融合輸出 |
| --- | --- | --- |
| < 0.05 m/s (停著) | 1.07 cm | 0.94 cm |
| 0.05~0.3 m/s (轉彎) | 1.65 cm | **3.40 cm** |
| > 0.3 m/s (直行) | 1.74 cm | 0.78 cm |

直行時融合比相機本身好一倍 (IMU + 輪速在中間補); 慢速轉彎時融合反而比它吃進去的相機
量測差一倍 —— 是**遞推那一半** (car_loc_fusion / car_loc_wheel) 在原地轉的時候把它拉走了,
很可能是 skid-steer 轉彎時輪子打滑、輪速不等於車速 (沒驗證)。這個 package 沒有動遞推,
方法五應該有一樣的現象。可以試的旋鈕: `bbox_r_inflate` 調小 (更信相機)。

> 這是「在我的複本上成立」, 不是在你那台 Isaac 上量的。你那邊跑一次
> `evaluate:=true`, 數字對得上再說它好。

### 在 Isaac 上踩到、已經修掉的四件事

每一件都會讓結果差好幾公分, 而且在離線測試裡完全看不到:

1. **手臂的姿勢。** 按 Play 之後手臂被關節驅動到 `arm_1 1.05 / arm_2 3.14 rad`, 往前伸到
   x = +0.367 m; 編輯姿勢只到 +0.195。凸包要在 **Play 之後**匯出。
2. **YOLO 不框夾爪尖端和車頂的 LiDAR 圓盤。** 把它們算進外形, 對應的邊就固定差 10~20 px。
   用 4192 個 YOLO 框 + GT 比 (`fit_car_model.py`, 後半段多相機反推):
   整台車 常數偏移 (-4.60, -1.15) cm / RMS 4.96 cm; 排除夾爪 + LiDAR (現在的設定)
   (-0.15, -0.90) cm / RMS 1.38 cm。
3. **YOLO 只認得「訓練時那個大小」的車** (512 輸入裡 ~22 px, 天花板正下方拍的)。
   裁小塊再放大 -> 車 ~100 px -> 8 台幾乎全認不得。所以 ROI 的工作是把車**縮放**到固定大小
   (預測框的 18 倍視窗縮進 512 畫布), 不是放大。
4. **車太近也一樣。** 離相機 2~3 m 時車在原圖 120~160 px, 舊的 ROI 被畫面大小卡住沒辦法
   再縮, 83% 的框差 >10 px。改成視窗超出畫面就補灰之後: 48%, 整條 RMS 2.65 -> 2.11 cm,
   最大 10.5 -> 6.4 cm, 被閘門擋掉 8.5% -> 1.1%。

## 離線測試 (不需要 ROS / Isaac / YOLO)

```bash
cd src/car_loc_multicam && python3 test/test_multicam.py      # 約 1 分鐘
```

真實的 8 台相機內外參 + 真實車型, 合成 IMU / 輪速 / 框 (延遲 80 ms, 框的邊有白雜訊 +
會慢慢飄的 3% 系統誤差, 出畫面就裁)。三種做法遞推完全一樣, 只差相機怎麼進濾波器:

| 情境 | [C] 框當量測 (這個) | [B] 先反推再融合 | [舊] 框中心打地面 |
| --- | --- | --- | --- |
| 0.7 m/s, 3 個種子 | **0.76 cm** (NEES 1.39) | 0.95 cm | 3.53 cm (NEES 14.4) |
| 1.5 m/s | **1.18 cm** | 1.59 cm | |
| IMU yaw 偏 5 度 | **1.06 cm** | 1.40 cm | |
| 只剩 cam 0 + 2 | **1.23 cm** | 1.58 cm | |
| 每台隨機掉 50% | **0.90 cm** | 1.28 cm | |
| 30% 的偵測是別處的假框 | 0.82 cm (擋掉 169 個) | | |
| 沒給起點 | 起點誤差 0.2~1.2 cm, 之後 0.73~0.79 cm | | |

**合成的框是用同一個車型生的** (只加了雜訊與飄移), 所以這張表量不到「YOLO 框到哪裡」跟
外形對不上的問題 —— 上面四件事全都是這張表看不到、Isaac 上才抓到的。

## 校正 / 換場景時要重做的事

全部只讀 USD, 不會改它。

| 什麼變了 | 做什麼 |
| --- | --- |
| 相機搬了 / 換鏡頭 | `scripts/export_from_usd.py <usd> --only cameras` -> `config/env_cameras.yaml` |
| 車的外形 / 手臂姿勢 | Isaac 裡打開 Physics 的 "Update to USD"、**按 Play 幾秒之後**在 Script Editor 跑 `export_from_usd.py` (預設參數 `SCRIPT_EDITOR_ARGS` 已經帶 `--exclude grap,mesh_3`, 輸出到 `/tmp/car_loc_multicam_export/`, 再把 `car_model.yaml` 複製到 `config/`)。不打開 Update to USD 的話 USD 裡的手臂還是編輯姿勢 |
| 車停著時多相機的位置固定偏車頭某個方向 (LiDAR 沒有) | `config/car_model.yaml` 的 `points_shift` (整個外形平移)。2026-11-02/03 三輪都偏前方 ~1 cm, 現在給 +1 cm; 量法與 A/B 寫在 yaml 裡。重新匯出外形之後會變回 0, 要重量 |
| 換 YOLO 模型 / 想確認車型對不對 | `scripts/record_bbox.py` 錄一段 (車要開遍房間), 再 `scripts/fit_car_model.py <錄的檔> config/car_model.yaml` —— 印每台每個邊的偏差、依距離分的表、建議的 `edge_bias_px`, 以及前半段量後半段驗的位置誤差 |

`export_from_usd.py` 在 Isaac 外面跑要借 Isaac 的 pxr, 指令寫在它的開頭。

## 實體相機

* `env_cameras.yaml` 的外參改用 `rvec` / `tvec` (OpenCV `solvePnP` 的輸出, 用地上量過
  座標的標記點), 內參用棋盤格校正; 魚眼鏡頭加 `fisheye_k` (Kannala-Brandt k1..k4)。
* 每台 Pi 自己跑 YOLO 的話照 `bbox_msg.py` 的格式發 `/env_cam_i/car_bbox`,
  `header.stamp` = **曝光時刻** (PTP 時間), 這裡 `detector:=false` 只開融合。
  只同步時鐘、不同步快門也可以 —— 每一則就是一台, 倒帶重放照樣處理。
* `best.onnx` 是用 Isaac 的天花板視角訓練的, 實景一定要重新標注訓練; 標注時框要不要含夾爪 /
  LiDAR 會直接決定 `--exclude` 該填什麼 (用 `fit_car_model.py` 量)。
* 真車沒有絕對 yaw (`yaw_source:=gyro`) 時, 起點的 yaw 只能從框的形狀掃 —— 車身前後差不多
  對稱, **可能顛倒 180 度**。建議 `lidar:=true` 讓 LiDAR 給起點, 或給 initial_pose。

## 已知的限制

| 症狀 | 原因 |
| --- | --- |
| 慢速轉彎時 3~4 cm | 原因還沒找到 (側向滑動、相機台數都排除了)。`points_shift` +1 cm 之後它是最大的誤差: 整體 p95 3.47 -> 3.91 cm 全部來自這裡 |
| 只有一台相機看得到車時狀態錯了, 收斂很慢 | 重定位要 `reloc_min_cameras` (2) 台; 一台只能走強制接受, 車子停著時 P 不長, 走不動 |
| 車離某台相機 < 3 m 時那台的框比較不準 | 還有 48% 差 >10 px; 卡方閘門會擋掉大部分, 其他台補上 |
| 手臂動了位置就偏 | 凸包是固定姿勢; 手臂姿勢改了要重新匯出 |
| 地板不平 / 車子傾斜 | 假設 base_link 永遠在 `base_z` 高度 |
| 偵測率很低、每台每秒才幾張 | 影像訂閱被改成 BEST_EFFORT 了 (`image_reliable: false`)。Isaac(host) -> 容器 的 UDP 緩衝只有 208 KB, 4.75 MB 的影像少一片整張作廢; 2026-10-30 實測 8 台 0~0.4 張/秒, 改回 RELIABLE 就是完整的 15 Hz |
| 跑一陣子之後每台從 15 Hz 掉到 ~3 Hz, 框的間隔 0.5~0.7 s | RELIABLE 在補掉的片段 (推測, 沒驗證): host 的 socket 緩衝只有 208 KB, 一開始掉片就一直在補。精度影響不大 (2026-10-30 那一輪 3 Hz 時 RMS 1.3~2.2 cm), 要救的話加大 host 的 `net.core.rmem_max` / `rmem_default` |
| 某幾台相機的時戳跟 `/clock` 差幾百秒 | Isaac 的問題 (Stop/Play 沒讓那幾台歸零, 8 個 graph 設定一樣)。融合與偵測都會自己量、自己扣 (log 有警告), 但扣掉之後那幾台的延遲補償沒了 —— 重開 Isaac 才是根本解法 |
