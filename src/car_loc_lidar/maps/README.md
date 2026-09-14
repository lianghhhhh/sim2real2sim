# maps

手動開車建出來的地圖放這裡。

```bash
ros2 launch car_loc_lidar mapping.launch.py
# 開一圈 (慢慢開, 每個角落都繞到, 要回到起點), 然後:
ros2 run nav2_map_server map_saver_cli -f /workspaces/src/car_loc_lidar/maps/room
```

會產生 `room.pgm` + `room.yaml`。`colcon build` 之後這兩個檔會跟著 package 裝進
share, `lidar_loc.launch.py` 預設就去那裡找 `room.yaml`。

檢查地圖 (不需要 ROS):

```bash
python3 -m car_loc_lidar.gridmap show src/car_loc_lidar/maps/room.yaml
```

## 校正 origin (建完圖一定要做)

SLAM 地圖的 `map` 座標系是**開始建圖那一刻的車子位姿**, 所以一定會跟 Isaac 的世界
座標差一個平移, 車頭沒對齊的話還差一個旋轉。症狀:

| 症狀 | 原因 |
| --- | --- |
| 整段誤差是同一個向量 | 平移 |
| yaw 誤差是固定角度; 房間中央很準, 開到兩端誤差變大而且兩端方向相反 | 旋轉 (1° 在 3 m 外 = 5.2 cm) |

**模擬 (有 GT)**: 用**目前這份 yaml** 錄一輪 (LiDAR 有開、車子要開到房間各處), 然後

```bash
./scripts/calibrate_map_origin.py car_run_data/<run>.csv            # 先看數字
./scripts/calibrate_map_origin.py car_run_data/<run>.csv --write    # 寫進 room.yaml
r                                                                    # 容器裡, 裝進 share
```

它會擬合平移 + 旋轉, 印出修正前後的預期誤差, 並把結果寫成
`origin: [x, y, yaw]`。yaw 是 nav2 的定義 (整張圖繞左下角轉), `gridmap.load_nav2`
讀的時候會重新取樣成軸對齊的格點。寫完再錄一輪、再跑一次確認平移 < 1 cm、旋轉 < 0.1°。

* CSV 比 yaml 舊時 `--write` 會拒絕 —— 那份資料是用改之前的 origin 錄的, 再套會修兩遍
* 腳本說「位置擬合與 yaw 誤差兩個角度差很多」= 地圖還有扭曲, 改 origin 只修得掉一部分,
  要重建地圖
* **每次重建地圖都要重新量**

2026-09-14 那張圖: 旋轉 -1.34°, 預期誤差中位 2.8 -> 1.2 cm、p90 6.6 -> 2.9 cm。

**實體車 (沒有 GT)**: 在房間裡量幾個點的世界座標 (貼膠帶), 把車停上去、記
`/lidar_loc/odom` 幾秒取中位數。至少兩個離得遠的點才量得出旋轉。
