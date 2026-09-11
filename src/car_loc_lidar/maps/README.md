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
