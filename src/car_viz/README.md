# car_viz — 用 Foxglove 看

開一個 WebSocket 讓 Foxglove Studio 連進來。只開門, 不起任何定位節點 ——
要看的東西自己另外開, 開了什麼就看得到什麼。

```bash
ros2 launch car_viz viz.launch.py                     # 自動挑 bridge
ros2 launch car_viz viz.launch.py bridge:=rosbridge   # 強制用 rosbridge
ros2 launch car_viz viz.launch.py port:=9999          # 換 port
ros2 launch car_viz viz.launch.py image_rate:=5.0 image_width:=640   # 網路慢就再壓
ros2 launch car_viz viz.launch.py image_topics:=/rgb                  # 只壓 /rgb
```

| `bridge:=` | 用哪個 | port |
| --- | --- | --- |
| `auto` (預設) | 有 `foxglove_bridge` 就用它, 沒有就退回 `rosbridge_server` (+ `rosapi`) | 8765 / 9090 |
| `foxglove` | `foxglove_bridge` (點雲效能好很多) | 8765 |
| `rosbridge` | `rosbridge_server` | 9090 |

啟動後會把網址印出來 (`ws://<容器IP>:<port>`)。
Foxglove Studio -> Open connection -> Foxglove WebSocket (或 Rosbridge) -> 貼上。

映像檔 `sim2real2sim:v1` 本身只有 rosbridge; `Dockerfile` 已經加了
`ros-humble-foxglove-bridge`, 重 build 之後 `auto` 就會改用它。

## 影像會自動壓縮

有兩個影像 topic 都大到會把 bridge 塞爆:

| topic | 內容 | 大小 |
| --- | --- | --- |
| `/rgb` | Isaac 相機原圖 | 1920x1536 rgb8 x 50 Hz |
| `/camera_loc/detections` | YOLO 標註圖 (`car_loc_camera`, 每處理一張 `/rgb` 發一張) | 1920x1536 bgr8 x 最多 50 Hz |

一張 8.85 MB, 每秒 **440 MB**。foxglove_bridge 的送出緩衝預設才 10 MB, 一張圖就快塞滿,
Foxglove 裡點一下 bridge 就卡死, 地圖和位姿跟著全部停住。

所以 `viz.launch.py` 預設會做兩件事:

1. 每個 topic 起一個 `image_compressor`, 壓成 **`<topic>/compressed`**
   (`CompressedImage`, JPEG): `/rgb/compressed`、`/camera_loc/detections/compressed`
2. 用 foxglove_bridge 的 `topic_whitelist` 把**原始影像擋在 bridge 外面**,
   Foxglove 的 topic 清單裡根本不會出現它們, 想點錯都點不到

Foxglove 的 Image 面板選 `.../compressed` 那個就好 (現成版面已經改好了)。

只壓縮是不夠的 —— 全解析度 JPEG 一張要編 18 ms (50 Hz 編不完), 還是每秒 18 MB。
所以同時限頻、縮圖:

| 參數 | 預設 | 效果 |
| --- | --- | --- |
| `image_rate` | `10.0` Hz | 上限。多的幀直接丟, 連解碼都不做 (實際會是 ~9 Hz)。`<= 0` 不限頻 |
| `image_width` | `960` px | 只縮不放大, 保持長寬比。編碼 18 ms -> 4.4 ms。`<= 0` 不縮 |
| `jpeg_quality` | `70` | 1~100。Isaac 畫面實測一張 ~18 KB |
| `image_topics` | `/rgb,/camera_loc/detections` | 要壓哪些, 逗號分隔。每個一個節點 |
| `compress` | `true` | `false` = 不壓也不擋 (bridge 很可能會卡死) |

四個參數對所有 topic 一體適用。

預設值下 `/rgb` 實測 (Foxglove 端收到的) ~9 Hz x 18 KB = **每秒 0.16 MB**, 原本的 1/2700。

沒有人訂 `<topic>/compressed` 的時候, 它**連原始影像都不訂** —— 光是 DDS 把 440 MB/s
送進來就是一筆開銷。Foxglove 打開 Image 面板的一秒內它才開始訂, 關掉就退訂。
所以 `car_loc_camera` 沒在跑也沒關係, 那個節點就只是閒著。
有人在看的時候每 10 秒印一行實際的收發頻率、每張大小、編碼時間, 看那行就知道參數
要不要調。

**走 rosbridge 的話擋不掉原始影像** (rosbridge 沒有黑名單), 壓縮照樣會起,
但只能靠自己別去點它們。

也可以單獨跑:

```bash
ros2 run car_viz image_compressor --ros-args -p image_topic:=/rgb -p max_rate:=5.0
```

## 看什麼

| topic | 型別 | 從哪來 |
| --- | --- | --- |
| `/map` | `OccupancyGrid` | `car_loc_lidar` 載入的地圖 (建圖時是 slam_toolbox 正在長的那張) |
| `/scan` | `LaserScan` | 感測器原始的一圈 (MS200) |
| `/lidar_loc/scan_matched` | `PointCloud2` | 配準後的點雲, 疊在地圖上看貼不貼 (`publish_debug_cloud:=true` 才有) |
| `/odom` | `Odometry` | Isaac ground truth |
| `/camera_loc/pose` `/lidar_loc/pose` `/imu_loc/pose` `/wheel_loc/pose` `/fusion_loc/pose` | `PoseWithCovarianceStamped` | 五條定位線 |
| `/rgb/compressed` | `CompressedImage` | 相機畫面, 壓過的 (見上面)。**原始 `/rgb` 不經過 bridge** |
| `/camera_loc/detections/compressed` | `CompressedImage` | YOLO 標註過的畫面, 壓過的 (`car_loc_camera` 的 `publish_annotated:=true`, 預設開)。**原始的不經過 bridge** |
| `/tf`, `/tf_static` | | `map -> base_link -> laser_frame`。**只能有一條定位線發 `map -> base_link`** |
| `/cmd_vel` | `Twist` | Foxglove 的 **Teleop 面板**往這裡發, 就能用滑鼠開車 (要有 `car_teleop` 的 `cmd_vel_bridge`) |

## 現成版面

`config/foxglove_layout.json` (build 之後在 `install/car_viz/share/car_viz/config/`):
3D (地圖 + 掃描 + GT + 五條位姿) · YOLO 畫面 · Teleop · 五條線的 x 對時間 · 融合的共變異數。
Foxglove 裡 Layout -> Import from file 匯入。

**匯不進去也沒關係** —— Foxglove 的版面格式會隨版本變, 照上面那張表自己拉面板就好。
IMU 那條預設隱藏 (它會漂出房間, 把畫面拉得很遠), 要看再在 3D 面板勾起來。

## 連不上

`run_isaac_gui.sh` 沒有做 port mapping。Linux 上直接用 launch 印出來的容器 IP 連;
連不上就在 `docker run` 加 `-p 8765:8765` (rosbridge 則是 `-p 9090:9090`)。
