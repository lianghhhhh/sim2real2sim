# KROS Car 實車測試報告

| 項目 | 內容 |
| --- | --- |
| 測試日期 | |
| 測試人員 | |
| 韌體 | `wildbot_wheelctrl`（`SERIAL_BAUDRATE` = 460800） |
| 映像檔 | `kros-car-new:latest` |
| 結論 | □ 全部通過　□ 部分通過　□ 未通過 |

## 1. 概要

這份報告用來驗證 `kros_car_new` 在實體車上能正常運作。它記錄測試步驟、預期結果和實際結果。

在這之前，程式只在沒有實車的環境驗證過。驗證內容包括：硬體介面的單元測試、韌體編譯，以及接模擬 RS-485 馬達的整合測試。模擬馬達只能證明協定兩端對得上，不能保證真的 DDSM115 也一樣。所以下面這些項目要上車確認：

| # | 待確認項目 | 驗證關卡 |
| --- | --- | --- |
| A | ESP32 的 USB 能跑 460800 baud | 第 3、4 關 |
| B | 馬達在速度環時回報的 `mode` 是 2 | 第 3、4 關 |
| C | 每 4 ms 問一顆馬達的匯流排時序沒有碰撞，回授是 50 Hz | 第 3、5 關 |
| D | 位置和電流回授的正負號跟輪速一致 | 第 6 關 |
| E | 主機卡死時，心跳逾時會在 0.5 秒內停輪 | 第 7 關 |

測試順序是從底層往上：先只測韌體和馬達，再加上 ROS，最後才落地。每一關都通過了才進下一關。這樣出問題時，範圍只會落在新加上的那一層。

## 2. 測試環境與安全準備

- [ ] **車子架高，四輪離地。** 第 3 到第 8 關都在離地狀態下做。
- [ ] 伸手就能切掉馬達電源。
- [ ] 電池電量足夠。電壓不足時，馬達回授可能不正常。
- [ ] 車上電腦已經安裝 Docker，可以連網（燒韌體時要下載 arduino-cli 和 ESP32 core）。
- [ ] 已經取得最新的 `kros_car_new` 資料夾。

以下指令都在**車上電腦**執行。除非另外說明，路徑都相對於 `kros_car_new/`。

---

## 3. 第 1 關：USB 固定名稱

**目的**：讓每個裝置的名稱固定，不受插入順序或 USB 孔位影響。

**步驟**

```bash
cd tools
sudo ./create_usb_rule.sh /dev/ttyUSB? usb_wheel          # 車輪 ESP32
sudo ./create_usb_rule.sh /dev/ttyUSB? oradar             # 光達
sudo ./create_usb_rule.sh /dev/ttyUSB? imu                # IMU
ls -l /dev/usb_wheel /dev/oradar /dev/imu
```

如果車上原本就設定過，只要確認名稱對得上。

**預期**：三個名稱都存在，各自指向正確的 `ttyUSB*` 或 `ttyACM*`。

**實際結果**：

**通過**：□

---

## 4. 第 2 關：燒錄韌體

**目的**：把新版韌體寫進 ESP32。

**步驟**

```bash
cd wildbot_wheelctrl
./flash_esp32.sh                       # 預設 PORT=/dev/usb_wheel
```

**預期**：編譯和上傳都成功，最後沒有出現錯誤。

**實際結果**：

**通過**：□

---

## 5. 第 3 關：只測 ESP32 和馬達，不開 ROS

**目的**：不經過 ROS，直接用序列埠對韌體下指令，確認韌體和馬達本身正常。這一關驗證待確認項目 A、B、C。

**前提**：`kros_car` 容器沒有在跑，因為它會佔用 `/dev/usb_wheel`。

### 3-0 開啟序列埠

**終端機 A**：設定 port，然後持續顯示 ESP32 的輸出。

```bash
stty -F /dev/usb_wheel 460800 raw -echo
cat /dev/usb_wheel
```

**終端機 B**：送指令。打開 port 時 ESP32 可能會重開機，等 1 到 2 秒再送。

```bash
W=/dev/usb_wheel
echo '{"T":11001,"time":-1}' > $W      # 關閉心跳，避免手動打字太慢觸發停輪
```

### 3-1 查詢每顆馬達的狀態

```bash
echo '{"T":10032,"id":1}' > $W
echo '{"T":10032,"id":2}' > $W
echo '{"T":10032,"id":3}' > $W
echo '{"T":10032,"id":4}' > $W
```

**預期**：每一行指令都得到一行回應，例如 `{"T":20010,"id":1,"mode":2,"tor":0,"spd":0,"temp":35,"u8":...,"err":0}`。

| 馬達 ID | 有回應 | mode | temp (°C) | err |
| --- | --- | --- | --- | --- |
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |
| 4 | | | | |

判讀方式：
- 完全沒有輸出，或出現亂碼：baud 不對（項目 A）。見第 13 節的對策。
- `mode` 不是 2：記下實際值（項目 B），第 4 關會用到。
- 只有某一顆沒回應：檢查那顆馬達的 ID、RS-485 接線和電源。
- `err` 不是 0：見第 13 節的錯誤碼對照。

### 3-2 單顆馬達慢轉

一次只轉一顆，確認 ID 和實際位置對得上，並記錄轉動方向。

```bash
echo '{"T":10010,"id":1,"cmd":20,"act":3}' > $W      # 20 rpm
echo '{"T":10000,"id":1}' > $W                       # 停止
# ID 2、3、4 照做
```

**預期**：ID 1 是左前、2 是左後、3 是右前、4 是右後。送正的 rpm 時，左輪往車頭方向轉，右輪往車尾方向轉。這對應 `hardware.yaml` 的 `wheel_motor_directions: 1,1,-1,-1`。

| 馬達 ID | 預期位置 | 實際位置 | +20 rpm 的轉向（往前 / 往後） |
| --- | --- | --- | --- |
| 1 | 左前 | | front |
| 2 | 左後 | | front |
| 3 | 右前 | | back |
| 4 | 右後 | | back |

### 3-3 輪詢模式

```bash
echo '{"T":10011,"cmd":[20,20,-20,-20],"act":3}' > $W
```

**預期**：
- 終端機 A 快速捲出 `{"T":20012,...}`，每秒大約 50 行。
- `ok` 幾乎都是 15，代表四顆馬達都有回應（項目 C）。偶爾掉一次可以接受，常常掉就是時序有問題。
- `err` 全部是 0。
- 四輪都往前轉。

觀察完之後停止：

```bash
echo '{"T":10013,"en":0}' > $W         # 離開輪詢模式，輪子會停
echo '{"T":11001,"time":500}' > $W     # 恢復心跳（ROS 啟動後也會每 2 秒自動重設）
```

最後在終端機 A 按 Ctrl+C。

**實際結果**（`ok` 的狀況、有沒有 `err`、四輪方向）：

**通過**：□ 3-1　□ 3-2　□ 3-3

---

## 6. 第 4 關：啟動 ROS（只開輪子）

**目的**：確認硬體介面能開啟序列埠、收到回授，並且沒有持續的錯誤。

**步驟**

```bash
./scripts/build_image.sh
./scripts/kros_car.sh          # log 會在前景顯示，按 Ctrl+C 會關掉全部服務
```

**預期**：log 中出現下面這兩行：

- `Wheel serial opened: /dev/usb_wheel @ 460800`
- `KrosCarSystem activated`

下列訊息**不應該**持續出現：

| 訊息 | 代表 | 對策 |
| --- | --- | --- |
| `is in mode N, switching it to speed loop` | 速度環的 mode 值不是 2（項目 B） | 見第 13 節 |
| `Wheel feedback problem (no fresh feedback ...)` | 收不到回授，通常是 baud 或接線（項目 A） | 見第 13 節 |
| `Wheel motor N (...) reports error 0x..` | 馬達自己回報錯誤 | 見第 13 節錯誤碼 |
| `Wheel controller stopped the wheels: no command within its heartbeat timeout` | 主機送指令中斷超過 500 ms | 檢查主機負載和 USB 連線 |

**實際結果**（貼上關鍵 log）：

```
```

**通過**：□

---

## 7. 第 5 關：觀察回授 topic

**目的**：確認 `/wheel_feedback` 的頻率和內容（項目 C）。

**步驟**：另外開一個終端機，啟動一個工具容器。它的參數跟 `scripts/wheel_controller.sh` 一樣，進去時 ROS 環境和 `ROS_DOMAIN_ID=67` 都已經設好：

```bash
docker run --rm -it --env-file docker/compose/.env --network host --ipc host kros-car-new:latest bash
```

在容器裡執行：

```bash
ros2 topic hz /wheel_feedback
ros2 topic echo /wheel_feedback
```

**預期**：
- 頻率約 50 Hz。
- 每則訊息都有四個關節：`front_left_joint`、`rear_left_joint`、`front_right_joint`、`rear_right_joint`。
- 靜止時 `velocity` 是 0，`position` 不會一直跳動。

**實際結果**（頻率：____ Hz）：

**通過**：□

---

## 8. 第 6 關：正負號與兩種輪子控制器

**目的**：確認回授的正負號（項目 D），以及 `base_controller` 和 `wheel_velocity_controller` 都能驅動輪子，並且會逾時停車。輪子仍然離地。

### 6-1 cmd_vel 慢速前進

預設啟用的就是 `base_controller`。在工具容器裡執行：

```bash
ros2 topic pub -r 20 /base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.1}}}"
```

另一個工具容器同時執行 `ros2 topic echo /wheel_feedback`。

| 檢查項目 | 預期 | 實際 |
| --- | --- | --- |
| 四輪實際轉向 | 都往車頭方向 | |
| 四個 `velocity` | 都是正值，約 2 rad/s | |
| 四個 `position` | 都持續增加 | |
| 起步瞬間的 `effort` | 正值 | |
| Ctrl+C 之後 | 0.6 秒內停止（`cmd_vel_timeout`） | |

正負號不對時：
- 輪子**實際轉向**錯了：改 `docker/compose/configs/hardware.yaml` 的 `wheel_motor_directions`。
- 轉向正確，但 `position` 或 `effort` 的正負號錯了：改 [kros_car_system_threads.cpp](workspaces/src/kros_car/src/kros_car_system_threads.cpp) 中 `wheel_handle_line()` 的換算，然後重建映像檔。

### 6-2 cmd_vel 原地左轉

```bash
ros2 topic pub -r 20 /base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {angular: {z: 0.5}}}"
```

**預期**：左輪往後轉，`velocity` 為負值。右輪往前轉，`velocity` 為正值。

**實際結果**：

### 6-3 joint_command 逐輪速度

在容器外，`kros_car_new/` 底下切換控制器：

```bash
./scripts/wheel_controller.sh joint_command
```

在工具容器裡執行：

```bash
ros2 topic pub -r 50 /joint_command sensor_msgs/msg/JointState \
  "{name: [front_left_joint, rear_left_joint, front_right_joint, rear_right_joint], velocity: [2.0, 2.0, 2.0, 2.0]}"
```

**預期**：四輪都往前轉，`/wheel_feedback` 的 `velocity` 約 2.0 rad/s。按 Ctrl+C 後，0.3 秒內停止（`command_timeout`）。

**實際結果**：

測完切回預設控制器：

```bash
./scripts/wheel_controller.sh cmd_vel
```

**通過**：□ 6-1　□ 6-2　□ 6-3

---

## 9. 第 7 關：心跳保護

**目的**：確認主機卡死時，ESP32 會自己停輪（項目 E）。

**步驟**：
1. 照 6-3 的方式，用 `joint_command` 讓輪子持續轉動。
2. 在容器外凍結 `kros_car` 容器：

```bash
docker ps                               # 找出 kros_car 容器的名稱
docker pause <kros_car 容器名稱>
```

3. 觀察輪子。
4. 測完恢復容器：

```bash
docker unpause <kros_car 容器名稱>
```

這裡要用 `docker pause`，不要用 `docker stop`。`stop` 會讓程式正常關閉，而 `on_deactivate()` 本身就會送停輪指令，這樣測不到心跳。`pause` 會凍結程序，才是真的模擬主機卡死。

**預期**：輪子在 0.5 秒內停下。`unpause` 之後，log 出現 `Wheel controller stopped the wheels: no command within its heartbeat timeout`。

**實際結果**（停輪大約花了 ____ 秒）：

**通過**：□

---

## 10. 第 8 關：手臂（已移除）

車上不再接手臂，ROS 程式也已經拿掉手臂的控制，這一關不用做。

---

## 11. 第 9 關：落地、光達與 IMU、校正

### 9-1 落地低速行駛

把車放回地面，在空曠的地方測試。確認目前是 `cmd_vel` 控制器：

```bash
./scripts/wheel_controller.sh cmd_vel
```

用 0.1 m/s 測前進、後退、左轉、右轉，每個動作各跑幾秒。

**預期**：車子行進方向和指令一致，沒有異常聲音或抖動。停止發布後，車子會停下。

**實際結果**：

### 9-2 全部服務

先按 Ctrl+C 關掉 `kros_car.sh`，再全部啟動：

```bash
./scripts/start_all.sh
```

在工具容器裡執行：

```bash
ros2 topic hz /scan            # 光達
ros2 topic hz /imu/data        # IMU
ros2 topic hz /wheel_feedback  # 確認輪子回授仍是 50 Hz
```

也可以用 Foxglove 連 `ws://<車子 IP>:8765` 檢視資料。

| Topic | 有資料 | 頻率 (Hz) |
| --- | --- | --- |
| `/scan` | | 13 |
| `/imu/data` | | x |
| `/wheel_feedback` | | 50 |

### 9-3 校正

里程計現在用的是量到的輪子位置，所以 `controllers.yaml` 的輪徑和輪距修正值要重新校正。

```bash
./scripts/calibrate.sh
```

依照精靈的指示跑完五個步驟。結果會存在 `kros_car_new/.ros/calibration_results.yaml`。把結果更新到 `docker/compose/configs/controllers.yaml`，再重啟 `kros_car`。

| 參數 | 原本的值 | 校正結果 |
| --- | --- | --- |
| `wheel_radius` (m) | 0.0506 | |
| `wheel_separation_multiplier` | 2.254 | |

**通過**：□ 9-1　□ 9-2　□ 9-3

---

## 12. 結果總表

| 關卡 | 內容 | 驗證項目 | 結果 | 備註 |
| --- | --- | --- | --- | --- |
| 1 | USB 固定名稱 | | | |
| 2 | 燒錄韌體 | | | |
| 3 | ESP32 和馬達單獨測試 | A、B、C | | |
| 4 | 啟動 ROS | A、B | | |
| 5 | 回授 topic | C | | |
| 6 | 正負號與控制器 | D | | |
| 7 | 心跳保護 | E | | |
| 8 | 手臂（已移除） | | 不適用 | |
| 9 | 落地、感測器、校正 | | | |

**實測得到的參數**

| 參數 | 假設值 | 實測值 |
| --- | --- | --- |
| 速度環的 `mode` | 2 | |
| USB baud | 460800 | |
| `/wheel_feedback` 頻率 | 50 Hz | |
| 心跳停輪時間 | < 0.5 s | |

---

## 13. 問題排除

| 症狀 | 可能原因 | 處理方式 |
| --- | --- | --- |
| 第 3 關完全沒輸出或出現亂碼 | ESP32 板子的 USB 晶片跑不到 460800 | 把 [wildbot_wheelctrl.ino](wildbot_wheelctrl/wildbot_wheelctrl.ino) 的 `SERIAL_BAUDRATE` 和 `docker/compose/configs/hardware.yaml` 的 `wheel_baudrate` **一起**改成 115200，重新燒錄，第 3 關的 `stty` 也改成 115200 |
| 一直出現 `is in mode N, switching it to speed loop`，輪子不動 | 速度環的 mode 值不是 2 | 先在 `hardware.yaml` 設 `wheel_require_feedback: false`，對照第 3-1 關記下的 mode 值，再修改程式中的判斷 |
| 一直出現 `Wheel feedback problem`，而且不會恢復 | baud 不符、USB 斷線、某顆馬達沒回應 | 回到第 3 關逐顆檢查 |
| 輪詢模式的 `ok` 常常不是 15 | 匯流排時序碰撞、接線不良、終端電阻 | 檢查 RS-485 接線，記錄是哪一顆掉（`ok` 的 bit i 代表 ID i+1） |
| 輪子轉向和指令相反 | `wheel_motor_directions` 不對 | 修改 `hardware.yaml` |
| 轉向正確，但 position 或 effort 正負號錯 | 回授換算的正負號假設錯誤 | 修改 `kros_car_system_threads.cpp` 的 `wheel_handle_line()`，然後重建映像檔 |
| `docker pause` 後輪子沒停 | 心跳被關掉，或 `wheel_heartbeat_ms` 是 -1 | 檢查 `hardware.yaml` 的 `wheel_heartbeat_ms` |

**DDSM115 錯誤碼**（`err` 欄位，依 bit 判讀）

| Bit | 意義 |
| --- | --- |
| 0 | 感測器故障 |
| 1 | 過電流 |
| 2 | 相電流過大 |
| 3 | 堵轉 |
| 4 | 故障排除中 |

---

## 14. 後續待辦

- [ ] 實際量測轉矩常數，更新 `hardware.yaml` 的 `wheel_torque_constant`（目前是規格書的 0.75 N·m/A）。
- [ ] 秤整車質量，更新 URDF 的 `base_link` 質量（目前是 37.6 kg，模擬會用到）。
- [ ] 對照實車量測光達、IMU、D455 的安裝座標。
- [ ] 決定光達轉速（實車 14 Hz，模擬 10 Hz）。
- [ ] 決定是否放寬 `max_rpm: 100`（約 0.53 m/s）。
