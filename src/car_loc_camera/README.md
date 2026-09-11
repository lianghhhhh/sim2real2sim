# car_loc_camera — 方法一: 只用天花板相機 + YOLO

```
/rgb ──> YOLO ──> bbox 中心 (px) ──> 地面投影 (單應性) ──> (x, y) 世界座標
                                                            │
                                  等速卡爾曼濾波 (NIS 閘門) ─┘
                                                            v
              /camera_loc/odom + /camera_loc/pose + TF map->base_link
```

**這個 package 只訂閱 `/rgb`。** 不吃 IMU, 不吃 LiDAR, 不 import 任何其他 package。
三條定位路線要能互相當對照, 就不能偷用彼此的資料。

## 用法

```bash
ros2 launch car_loc_camera camera_loc.launch.py
ros2 launch car_loc_camera camera_loc.launch.py evaluate:=true   # 跟 GT /odom 比
ros2 launch car_loc_camera camera_loc.launch.py teleop:=true     # 邊開邊看
```

鍵盤遙控要另開一個 terminal (需要真的 TTY):

```bash
docker exec -it ros2_node bash -lc 'r && ros2 run car_teleop teleop_key'
```

## 模型放哪

YOLO 模型 (9.7 MB) 放在本 package 的 `resource/best.onnx`, `colcon build` 時跟著
裝進 share。節點**依序去找**:

1. `-p model_path:=/path/to/best.onnx`
2. 環境變數 `CAR_YOLO_MODEL`
3. 本 package 的 `resource/best.onnx` (build 後的 share 或原始碼目錄)

(以前還會退回去 `car_inference` 那裡找; 那個 package 已經刪掉, 模型已經搬進來了。)

> `imgsz` 必須是 **512** —— `best.onnx` 是用固定 512x512 匯出的 (沒有 dynamic axes),
> 填別的值會直接在推論時報 `Got invalid dimensions for input: images`。
> 要換解析度得重新匯出模型 (`yolo export model=best.pt format=onnx imgsz=960`),
> 但不見得值得, 理由見下面「誤差在哪裡」。

## 輸出

| topic | 型別 | 內容 |
| --- | --- | --- |
| `/camera_loc/odom` | `nav_msgs/Odometry` | 位姿 + 速度 + 共變異數 (主要輸出) |
| `/camera_loc/pose` | `PoseWithCovarianceStamped` | 同上, 給 nav2 / rviz |
| `/camera_loc/detections` | `Image` | YOLO 標註圖 |
| `/camera_loc/detection_px` | `Float32MultiArray` | `[px, py, conf]` 原始像素, 重新校正時直接餵給擬合腳本 |
| TF | `map -> base_link` | `publish_tf:=false` 可關 (三條路線同時跑時只能留一條) |

## 誤差在哪裡 —— 先看這一段再調參

`config/camera_ground.yaml` 的校正殘差是 **RMSE 7.26 cm**, 而實測 YOLO 定位的
誤差中位數是 **7.4 cm**。兩個數字幾乎一樣, 也就是說:

> **誤差幾乎全部來自校正模型本身, 不是來自偵測。**

bbox 中心的逐幀抖動只有 1.5 px (≈ 0.8 cm), 比校正殘差小一個數量級。所以換更大的
YOLO、提高輸入解析度**都不會動到誤差的主要來源**。要往下壓只有三條路, 按效果排:

1. **量出 `delay` 並填進校正檔。** 曝光 + 傳輸 + YOLO 推論加起來幾十到上百毫秒;
   車速 0.8 m/s 時 100 ms 就是 8 cm 的系統性偏差, 而且偏差方向跟著車頭轉 ——
   單應性是一個只跟位置有關的模型, 吸收不掉它。
   量法: 開一段直線, 算 `/camera_loc/pose` 與 ground truth `/odom` 的互相關峰值。
2. **重新擬合 `k1`/`k2`。** 現在是 `[0, 0]` —— 不是「量出來是 0」, 是根本沒擬合。
   repo 裡有 `fish_eye.py` 跟魚眼原圖, 代表這顆相機是廣角的。
3. **打開 `use_aabb`。** YOLO 給的是軸對齊框, 而車子是有高度的長方體, 框的中心
   落在頂面與底面投影之間, 偏移量隨 yaw 變。量級 0.5~1 cm。
   **要打開必須先重新擬合單應性** —— 現在的 `H` 已經把這一項平均吸收進去了,
   直接打開等於扣兩次。

## 離線驗證 (不需要 ROS, 不需要 YOLO, 不需要 Isaac)

```bash
cd src/car_loc_camera && python3 test/test_tracker.py
```

下面每一個數字都是它印出來的。

## 三個實作上的決定

**挑框用「離預測位置最近」, 不是「信心最高」。**
畫面裡只要多一個誤判 (影子、反光), 用排序或信心挑都會讓軌跡整段跳掉; 而濾波器
本來就知道車子大概在哪。`gate_radius` 是閘門半徑; 連續 `gate_giveup` 幀都沒有框
落在閘門內就放棄閘門 (車真的被搬走時預測位置本來就是錯的)。

**輸出是濾波後的, 不是逐幀量測。**
逐幀的 7.26 cm 白雜訊被等速模型平均掉, 而且 YOLO 漏幀時輸出不會斷, 只是共變異數
長大。離線實測 (20 Hz, 量測雜訊 7.26 cm):

| 機動程度 (峰值加速度) | `accel_sigma`=1 | **=2 (預設)** | =3 | =5 |
| --- | --- | --- | --- | --- |
| 慢速 0.29 m/s² | 4.63 cm | **5.19 cm** | 5.59 cm | 6.13 cm |
| 中速 0.94 m/s² | 6.13 cm | **5.37 cm** | 5.58 cm | 6.10 cm |
| 高機動 1.66 m/s² | 12.35 cm | **6.50 cm** | 6.10 cm | 6.29 cm |

`accel_sigma` 是唯一真的需要調的參數, 規則很簡單: **設成車子的峰值加速度**。
**設太小比設太大危險得多** —— `accel_sigma=1` 在高機動下不只誤差翻倍, 800 幀裡
還有 91 幀**正確的**量測被卡方閘門擋掉 (濾波器過度自信的典型症狀, `accel_sigma=3`
只擋 6 幀)。預設 2.0 是三種情境下都不會出事的折衷。

也就是說: 濾波把 7.26 cm 壓到 5~6.5 cm。**沒有壓到 3 cm** —— 那需要先把校正
做好 (見上一節), 濾波只能處理隨機的那一半, 處理不了系統性的偏差。

離群值閘門的效果 (高機動, 一部分幀誤判到 1.5 m 外):

| 誤判率 | 位置 RMS | 擋掉 / 總數 |
| --- | --- | --- |
| 0% | 6.38 cm | 17 / 800 |
| 10% | 7.16 cm | 96 / 800 |
| 25% | 8.02 cm | 211 / 800 |

車子被瞬間搬走 3.6 m 時, 逃生門 (`force_accept_after`) 讓它 **7 幀 (0.35 s)**
就回到 30 cm 以內 —— 沒有這個機制的話, 閘門會永遠擋著正確的量測不放。

**朝向 (yaw) 來自速度方向。**
單一個 bbox 中心**沒有任何資訊**可以分辨車頭朝向, 只看得出移動方向。所以:

* 低於 `yaw_min_speed` (0.15 m/s) 就維持上一個 yaw —— 車速低的時候「速度方向」
  只是量測雜訊除以 dt。
* 第一次認定 yaw 的門檻更高 (`yaw_init_speed` 0.30 m/s)。
* **倒車時移動方向跟車頭差 180 度。** 預設 (`allow_reverse: false`) 直接把移動
  方向當車頭 —— 誠實, 但倒車時會差 180 度。開 `allow_reverse` 是改用「車身不會
  瞬間翻面」的連續性去猜, 倒車就對了, **但萬一第一次定案定反了會一路錯下去**。
  離線實測三個亂數種子: `false` 全部是 22° 左右, `true` 有**兩個**鎖到 164°。
  要真的解決得換 YOLO 的 OBB 定向框模型, 那需要重新標注資料。

> **yaw 只能算粗略朝向。** 誤差大約是 `atan(速度估計誤差 / 車速)`, 實測
> 0.4~1.0 m/s 車速下 yaw RMS 是 **15~22 度**, 而且**車越慢越差**。
> 這是「用移動方向當車頭」的物理上限, 不是參數問題。要拿它去做需要精確朝向的
> 事 (例如貼牆停車) 是不行的。

## 這條路的死穴

| 症狀 | 原因 |
| --- | --- |
| 誤差是一個不會變的平移 | 校正檔的單應性原點沒對準, 不是定位在漂。`evaluate` 會把常數偏移單獨報出來 |
| 輸出整段中斷 | 車開到柱子後面 / YOLO 漏偵測。`evaluate` 會報中斷次數與最長中斷 —— **這條路的可用性由它決定, 不是由平均誤差** |
| 位置突然跳幾公尺 | 誤判 (影子、反光) 被當成車。把 `gate_radius` 縮小, 或把 `conf` 調高 |
| 車在畫面邊緣時特別不準 | 單應性在校正資料涵蓋不到的區域是外插。校正資料要涵蓋整個活動範圍 |
| 天花板相機看不到的地方 | 沒救 —— 這是這個方法的定義域, 不是參數問題 |
