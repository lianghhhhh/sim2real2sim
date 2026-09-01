#!/usr/bin/env python3
"""
lidar_imu_odometry.py

只用 LiDAR + IMU 推算車子位置的 2D 里程計 (純 Python / numpy / scipy)。

演算法:
    1. PointCloud2 (或 LaserScan) -> 轉到 base_link -> 濾地面/天花板 -> 依角度降採樣成 2D 點集
    2. IMU 的 gyro-z 積分出 delta-yaw, 當成 ICP 的初始猜測 (讓 ICP 不會在快速轉彎時發散)
    3. ICP (trimmed point-to-point, scipy cKDTree) 把當前掃描對齊到「關鍵幀」
       -> 用關鍵幀而不是上一幀, 可以大幅減少靜止/慢速時的累積漂移
    4. ZUPT: IMU 判定靜止時直接凍結位姿, 不做 ICP

輸出:
    nav_msgs/Odometry  (預設 /odom_lidar, odom -> base_link)
    可選 TF odom -> base_link  (publish_tf; 若後面接 robot_localization EKF 請設 false)

注意: 訂閱端一律用 BEST_EFFORT。RELIABLE 的 publisher 可以餵給 BEST_EFFORT 的
subscriber (相容), 反過來則不行 — 所以這樣寫不管 Isaac 用哪種 QoS 都收得到。
"""
import math
import os

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_srvs.srv import Trigger

# PointField.datatype -> numpy dtype
_PF_DTYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
             5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """把 PointCloud2 的 x/y/z 欄位拆成 (N,3) float64, 不依賴 ros2_numpy。"""
    fields = {f.name: f for f in msg.fields if f.name in ('x', 'y', 'z')}
    if len(fields) < 3:
        return np.empty((0, 3))

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    rows = raw.size // msg.point_step
    n = min(rows, msg.width * msg.height) if msg.width * msg.height else rows
    if n == 0:
        return np.empty((0, 3))
    raw = raw[:rows * msg.point_step].reshape(rows, msg.point_step)[:n]

    out = np.empty((n, 3), dtype=np.float64)
    for i, name in enumerate(('x', 'y', 'z')):
        f = fields[name]
        dt = np.dtype(_PF_DTYPE[f.datatype]).newbyteorder('>' if msg.is_bigendian else '<')
        col = raw[:, f.offset:f.offset + dt.itemsize].copy().view(dt).reshape(-1)
        out[:, i] = col
    return out


def laserscan_to_xyz(msg: LaserScan) -> np.ndarray:
    r = np.asarray(msg.ranges, dtype=np.float64)
    ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
    ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
    r, ang = r[ok], ang[ok]
    return np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros_like(r)], axis=1)


class LidarImuOdometry(Node):

    def __init__(self):
        super().__init__('lidar_imu_odometry')

        p = self.declare_parameter
        # --- topics / frames ---
        p('input_type', 'pointcloud')          # 'pointcloud' 或 'scan'
        p('cloud_topic', '/lidar/point_cloud')
        p('scan_topic', '/scan')
        p('imu_topic', '/imu')
        p('odom_topic', '/odom_lidar')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('publish_tf', False)                 # 接 EKF 時保持 False, 由 EKF 發 TF
        # --- 點雲前處理 (皆在 base_link 座標下) ---
        p('range_min', 0.20)
        p('range_max', 30.0)
        # 地面回波必須濾掉: 地面點永遠比牆近, 會贏走每一個角度格, 而它們每幀
        # 的落點都不同 -> ICP 會靜默失效 (位姿看起來就是不會動)。
        #
        # 'auto' (預設): 直接從點雲量出地面在哪裡。平坦的地面在感測器座標下
        #   z 是一個定值, 所以會在 z 直方圖上形成一根很尖的峰; 找到那根峰就
        #   知道感測器離地多高, 完全不需要人工填 static TF 的高度。
        #   這樣即使 lidar_z 填錯, 濾波依然正確 (只有輸出的座標原點會差一個高度)。
        # 'manual': 用下面的 z_min / z_max (base_link 座標) 硬濾。
        p('ground_mode', 'auto')
        p('ground_clearance', 0.15)            # auto: 地面以上多少才算有效結構
        p('ground_max_height', 1.50)           # auto: 地面以上多高以內
        p('z_min', 0.15)                       # manual 用
        p('z_max', 1.50)                       # manual 用
        p('angle_bins', 720)                   # 依角度降採樣, 每格留最近的一點
        p('min_points', 60)
        # --- ICP ---
        p('icp_max_iter', 25)
        p('icp_tol', 1.0e-4)
        p('icp_max_correspondence', 1.0)       # 對應點最大距離 (m)
        p('icp_keep_ratio', 0.75)              # trimmed ICP: 只留最好的 75% 對應
        p('icp_min_inlier_ratio', 0.35)        # 低於此值視為比對失敗, 改用 IMU 推估
        # --- 關鍵幀 (map_mode=False 時才用得到) ---
        p('keyframe_dist', 0.30)
        p('keyframe_angle', 0.25)
        # --- 地圖模式: 這是「會漂」與「不會漂」的分界 ---
        # 關鍵幀模式本質上是里程計: 每一幀都跟「不久前的某一幀」比對, 誤差一幀一幀
        # 往下傳, 沒有任何絕對參考, 所以一定會漂 —— 這是演算法的性質, 不是調參能解決的。
        #
        # 但這個場景不需要忍受漂移: 房間是 10x6 的封閉空間, 完全靜態, 而且 360 度
        # LiDAR 從房間裡任何一點都看得到全部四面牆。也就是說「整個地圖隨時都在視野裡」。
        # 這種情況下可以直接把每一幀對「累積起來的房間地圖」配準, 得到的是相對於地圖
        # 原點的絕對位姿 —— 誤差不會累積, 而且完全不需要回環偵測那一套機制。
        #
        # 地圖只在開頭 map_build_sec 秒內累積, 之後凍結, 從此只讀不寫。
        #
        # 為什麼一定要凍結 (這裡踩過一次坑):
        #   直覺會想「用格點去重就好, 一個格子被佔了就不再寫入」。但去重只擋得住
        #   「重寫同一格」, 擋不住「寫進全新的錯誤格子」—— 位姿一旦漂了 20 cm,
        #   同一面牆會被寫成一道平行的鬼牆, 那些都是全新的格子, 全部進得去。
        #   實測不凍結時地圖長到 68583 格 (這個房間只需要約 830 格), 全是鬼牆,
        #   ICP 之後就鎖到鬼牆上, 誤差反而比純里程計還大。
        #
        # 凍結在這個場景是安全的: 360 度 LiDAR 從房間裡任何一點都看得到全部四面牆,
        # 所以第一幀就幾乎是完整地圖了; 給 2 秒只是為了填滿柱子後面的遮蔽死角。
        # 如果之後換到更大、需要一邊走一邊建圖的環境, 把 map_freeze 設成 false。
        p('map_mode', True)
        p('map_grid', 0.05)                    # m, 地圖的格點解析度
        p('map_build_sec', 2.0)                # 開頭這幾秒無條件累積
        # 建圖期結束後不是完全停寫, 而是「只補洞」: 一個點只有在「離地圖上任何
        # 既有的點都超過 map_gap_dist」時才寫得進去。
        #   為什麼需要補洞: 開頭 2 秒車子還在起點, 柱子後面是看不到的。實測這樣建
        #   出來的地圖只覆蓋 86% 的真實幾何, 最大缺口 0.70 m。車子後來開進那些
        #   死角時, 局部沒有地圖可以對, 位姿就會沿著牆滑走。
        #   為什麼不能無條件續建: 把建圖期拉到 40 秒雖然覆蓋率到 98.9%, 但格數從
        #   1382 暴增到 14673 —— 位姿漂掉之後寫進去的全是「離真牆 20 cm 的鬼牆」,
        #   ICP 之後會鎖到鬼牆上。
        #   這兩者的差別剛好可以用距離分開: 鬼牆離真牆只有 0.1~0.3 m (就是位姿的
        #   誤差量), 而真正沒看過的死角離任何既有點都有 0.5 m 以上。
        p('map_gap_dist', 0.35)                # m, 離既有地圖這麼遠才算「沒看過的地方」
        p('map_gap_fill', True)                # False = 建圖期後完全停寫
        # --- 存檔 / 載入地圖 ---
        # 即時建圖的問題是「建出來的地圖沒有人看過」: 起跑位置不同、前幾秒的配準
        # 品質不同, 每次的地圖都不一樣, 而「地圖是壞的」跟「定位是壞的」在結果上
        # 長得一模一樣, 根本分不開。
        # 正確流程是把這兩件事拆開:
        #   1) 建圖: 慢慢開一圈, 存成檔案
        #        ros2 launch car_navigation build_map.launch.py map_save_path:=maps/room.npz
        #   2) 檢查: 親眼確認地圖像個房間 (牆是細線, 不是兩條平行線)
        #        python3 -m car_navigation.gridmap show maps/room.npz
        #   3) 定位: 載入這張確認過的地圖, 從此不再建圖
        #        ros2 launch car_navigation lidar_imu_localization.launch.py map_path:=maps/room.npz
        p('map_path', '')                      # 載入既有地圖; 空字串 = 即時建圖
        p('map_save_path', '')                 # 結束時把地圖存到這裡
        # 載入地圖後, 車子必須從地圖座標系裡的這個位姿開始 (x, y, yaw 度)。
        # 定位是從這裡「接續」下去的, 填錯的話第一次配準就會對到錯的地方。
        p('initial_pose', [0.0, 0.0, 0.0])
        p('map_oob_margin', 1.0)               # m, 位姿超出地圖邊界多少就報錯
        p('map_max_points', 120000)
        p('map_insert_min_inlier', 0.60)       # 配準品質低於此值的幀不准寫入地圖
        p('map_rebuild_every', 200)            # 累積這麼多新點才重建 KD-tree
        # --- 快速旋轉的處理 (實測這台車原地打轉可以到 24.6 rad/s) ---
        # RTX LiDAR 的 fullScan 是「累積一整圈才發一則」, 整則訊息只有一個時間戳。
        # 車子在累積那一圈的期間如果自己轉了 δ 度, 掃描圖形就被抹開 δ 度。
        #
        # deskew_enable: 用 gyro 把這個抹開補償掉 (見 _deskew)。這是正解, 平常一定要開。
        # max_scan_rotation: 最後一道保險, 不是調校用的旋鈕。運動補償實測非常有效
        #   (單幀轉 57 度時, 掃描與靜止基準的差距從 0.536 m 降到 0.017 m), 所以
        #   這個上限訂得比實際會遇到的轉角 (實測單幀最大 65 度 = 1.13 rad) 高很多,
        #   平常不會觸發。訂太緊反而會丟掉補償得回來的好幀 (實測設 1.0 rad 會讓
        #   誤差從 0.28 m 變成 0.45 m)。它擋的是掃描頻率突然掉下來、或車子轉得比
        #   設計預期更快的情況。上限必須小於 pi, 否則後面跟 gyro 對帳時 wrap 會失效。
        # yaw_gate_*: 單幀的粗差保護 —— ICP 算出來的轉角與 gyro 差太多就丟棄這一幀。
        #   注意這不是修好快速旋轉的關鍵 (實測 de-skew 開啟後單幀分歧最多只有
        #   0.6 度, 這道閘門根本不會觸發); 它擋的是「突然出現的大幅錯配」,
        #   例如有人走過感測器前面、或訊息中斷後拿過期的關鍵幀硬對。
        # 運動補償靠「方位角就是時間軸」這個假設 (見 _sweep_fraction)。這個假設對
        # 旋轉式 LiDAR 成立, 但實際輸出的排列方式沒辦法在寫程式的時候確定 ——
        # 假設錯了的話, 每一幀都會被補償歪掉, 而且完全不會報錯。
        # 所以不要用猜的: 開頭幾十個「有在轉」的幀, 兩種版本都算一次「掃描貼合
        # 參考的程度」, 哪個好就用哪個。這是拿真實資料直接驗證假設, 不是靠信仰。
        p('deskew_verify_frames', 30)          # 用幾個轉動中的幀來驗證 (0 = 不驗證)
        p('deskew_verify_min_rot', 0.10)       # rad, 單幀轉這麼多才拿來驗證
        p('deskew_enable', True)
        p('max_scan_rotation', 2.50)           # rad, 一次掃描期間允許的最大轉角 (< pi)
        p('yaw_gate_base', 0.15)               # rad, 固定容差
        p('yaw_gate_ratio', 0.25)              # 再加上轉角大小的這個比例
        # yaw_icp_gain: 角度用互補濾波, 而不是每幀直接採用 ICP 的值。
        #   兩個角度來源的性質完全不同:
        #     gyro  -- 直接量角速度, 短時間內非常準 (實測逐幀誤差 < 0.1 度),
        #              但會被 bias 慢慢帶偏, 沒有絕對參考。
        #     ICP   -- 有絕對參考 (房間本身), 但每幀有幾度的隨機誤差, 且車子
        #              轉快時關鍵幀每幀都要重建 (實測 42% 的幀) -> 退化成逐幀
        #              比對, 誤差變成隨機遊走。實測單幀轉 30~45 度時角度誤差
        #              2.6 度/幀, 400 幀的旋轉測試累積下來就是幾十度 —— 這才是
        #              舊版 yaw 最後錯到 ±150 度的真正原因 (是慢慢漂掉的,
        #              不是某一幀突然對錯邊)。
        #   舊版每幀直接把 ICP 的角度當答案, 等於把 gyro 的優點整個丟掉。
        #   改成「以 gyro 積分為主, ICP 只用很小的增益慢慢修正它的漂移」:
        #   快速旋轉時 gyro 主導 (ICP 的雜訊被 gain 壓掉), 靜止/慢速時 ICP
        #   仍然能把 gyro 的 bias 拉回來 —— 兩者的長處都留住。
        #   增益不是固定值, 而是隨「這一幀轉了多少」遞減 (見 _yaw_gain):
        #   車子幾乎沒在轉時掃描很乾淨、關鍵幀也穩定, ICP 的角度可信, 用大增益
        #   把 gyro 的 bias 拉回來; 轉很快時 ICP 的角度是隨機遊走, 用小增益避免
        #   它污染 gyro。這樣「快速旋轉靠 gyro、長時間靠 LiDAR」兩者兼得 ——
        #   固定小增益會讓 gyro bias 沒人管 (實測 bias 0.05 rad/s 沒校正時,
        #   固定 0.1 的增益會漂到 3.6 m, 自適應則能壓回來)。
        # 註: 這兩個參數只在關鍵幀模式 (map_mode=False) 下生效; 地圖模式一律
        #     完全採信 ICP 的角度, 原因見 _yaw_gain。
        p('yaw_icp_gain', 0.50)                # 車子不轉時的增益
        p('yaw_icp_gain_knee', 0.10)           # rad, 單幀轉角到這個值時增益減半
        p('coast_vel_decay', 0.8)              # 放棄比對時, 速度先驗每幀衰減多少
        # --- IMU ---
        p('use_imu_orientation', False)        # True = 直接吃 IMU 的絕對 yaw (模擬器很準, 真車不建議)
        p('gyro_calib_sec', 1.0)               # 開機前 N 秒靜止, 估 gyro bias
        p('gyro_calib_max_bias', 0.05)         # 估出來超過這個值就視為「校正時車子在動」而不採用
        p('zupt_enable', True)
        p('zupt_gyro_thresh', 0.02)            # rad/s
        # 用「水平加速度大小」而不是「|加速度長度 - g|」: 後者被重力蓋掉,
        # 直線加速 1 m/s^2 只會讓長度變化 0.04 m/s^2, 根本分辨不出動或不動。
        p('zupt_accel_thresh', 0.20)           # m/s^2 (水平分量 hypot(ax, ay))
        # 等速直行時 gyro 與加速度都接近 0, 光看 IMU 一定會誤判成靜止,
        # 所以還要求 ICP 自己量到的位移也接近 0。
        p('zupt_speed_thresh', 0.05)           # m/s (ICP 量到的速度低於此值才算靜止)
        p('log_stats', False)                  # 逐幀印診斷 (dt / 點數 / ICP inlier), 調參用
        p('status_period', 5.0)                # 每隔幾秒印一次健康狀態 (0 = 關閉)
        # 發布的 header.stamp 用哪個時間。
        # Isaac 的各個 publisher 可能用不同的時間源: /clock 與 IMU 跟著 playback
        # (按 Play 歸零), 但 RTX LiDAR 的 resetSimulationTimeOnStop 預設是 False
        # (跨 Stop/Play 單調累加)。兩者一旦差開, 下游的 robot_localization 會把
        # 我們的 odometry 當成「未來的資料」直接丟掉 —— EKF 就只剩 IMU, 位置卡在原點。
        # 'auto' (預設): 沿用感測器時間, 但偵測到與本節點時鐘差太多就改用節點時鐘並警告。
        # 'sensor' / 'node': 強制指定。
        p('stamp_source', 'auto')
        p('stamp_skew_thresh', 1.0)            # 秒

        def g(k):
            return self.get_parameter(k).value

        self.input_type = g('input_type')
        self.odom_frame, self.base_frame = g('odom_frame'), g('base_frame')
        self.publish_tf = g('publish_tf')
        self.range_min, self.range_max = g('range_min'), g('range_max')
        self.z_min, self.z_max = g('z_min'), g('z_max')
        self.ground_mode = g('ground_mode')
        self.ground_clearance = g('ground_clearance')
        self.ground_max_height = g('ground_max_height')
        self.ground_z = None                    # 感測器座標下的地面高度 (= -感測器離地高)
        self._ground_samples = []
        self.angle_bins, self.min_points = int(g('angle_bins')), int(g('min_points'))
        self.icp_max_iter, self.icp_tol = int(g('icp_max_iter')), g('icp_tol')
        self.icp_max_corr, self.icp_keep = g('icp_max_correspondence'), g('icp_keep_ratio')
        self.icp_min_inlier = g('icp_min_inlier_ratio')
        self.kf_dist, self.kf_angle = g('keyframe_dist'), g('keyframe_angle')
        self.deskew_enable = g('deskew_enable')
        self.deskew_verify_n = int(g('deskew_verify_frames'))
        self.deskew_verify_min_rot = g('deskew_verify_min_rot')
        self._deskew_votes = []                 # [(補償後的貼合度, 未補償的貼合度), ...]
        self._deskew_verified = False
        self.max_scan_rot = g('max_scan_rotation')
        self.yaw_gate_base, self.yaw_gate_ratio = g('yaw_gate_base'), g('yaw_gate_ratio')
        self.yaw_icp_gain = g('yaw_icp_gain')
        self.yaw_gain_knee = g('yaw_icp_gain_knee')
        self.coast_vel_decay = g('coast_vel_decay')
        self.map_mode = g('map_mode')
        self.map_grid = g('map_grid')
        self.map_build_sec = g('map_build_sec')
        self.map_gap_dist = g('map_gap_dist')
        self.map_gap_fill = g('map_gap_fill')
        self.map_max_points = int(g('map_max_points'))
        self.map_min_inlier = g('map_insert_min_inlier')
        self.map_rebuild_every = int(g('map_rebuild_every'))
        self.map_path = g('map_path')
        self.map_save_path = g('map_save_path')
        self.initial_pose = list(g('initial_pose'))
        self.map_oob_margin = g('map_oob_margin')
        self._map_bounds = None
        self._oob = 0
        self.use_imu_orient = g('use_imu_orientation')
        self.gyro_calib_sec = g('gyro_calib_sec')
        self.gyro_calib_max = g('gyro_calib_max_bias')
        self.zupt_enable = g('zupt_enable')
        self.zupt_gyro, self.zupt_accel = g('zupt_gyro_thresh'), g('zupt_accel_thresh')
        self.zupt_speed = g('zupt_speed_thresh')
        self.log_stats = g('log_stats')
        # 健康狀態統計
        self._st = dict(n=0, raw=0, kept=0, inlier=0.0, fail=0, nocloud=0, thin=0,
                        moving=0, spin=0, gated=0, max_rot=0.0)
        self._st_pose = np.zeros(3)
        self._st_cells = 0
        self._st_total = 0
        self._ground_tries = 0
        self._raw_n = 0
        self.stamp_source = g('stamp_source')
        self.stamp_skew_thresh = g('stamp_skew_thresh')
        self._use_node_clock = (self.stamp_source == 'node')
        self._skew_samples = []
        period = g('status_period')
        if period and period > 0:
            self.create_timer(period, self._status)

        # --- 狀態 ---
        self.pose = np.array([0.0, 0.0, 0.0])   # x, y, yaw (odom frame)
        self.kf_pose = np.array([0.0, 0.0, 0.0])
        self.kf_tree = None
        self.last_scan_time = None
        self.last_vel = np.array([0.0, 0.0, 0.0])  # vx, vy (base frame), wz
        # 放棄比對後關鍵幀就過期了 (位姿是 IMU 推的, 已經漂掉)。下一幀不要拿舊關鍵幀
        # 硬對 —— 先驗是錯的, ICP 很可能 snap 到錯的牆。直接用當下的掃描重建關鍵幀。
        self._need_keyframe = False
        self._sweep_checked = False
        # --- 地圖 ---
        self.map_pts = np.empty((0, 2))
        self.map_tree = None
        self._map_cells = set()
        self._map_pending = 0
        self._map_t0 = None
        self._map_gap_only = False

        # IMU
        self.gyro_bias = 0.0
        self._bias_buf, self._bias_t0 = [], None
        self.bias_ready = False
        self.imu_yaw = 0.0                      # gyro 積分出來的連續 yaw
        self.imu_yaw_at_last_scan = None
        self.imu_abs_yaw = None
        self.imu_abs_yaw_at_last_scan = None
        self.last_imu_time = None
        self.last_gyro_z = 0.0
        self.stationary = False

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None
        self._sensor_tf = None                  # (R 3x3, t 3,)
        self._sensor_tf_warned = False
        self._diag_done = False

        if self.map_path:
            self._load_map(self.map_path)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)

        self.odom_pub = self.create_publisher(Odometry, g('odom_topic'), 10)
        # 建圖跑完不必關掉節點就能存檔:
        #   ros2 service call /save_map std_srvs/srv/Trigger
        self.create_service(Trigger, 'save_map', self._save_map_srv)
        self.create_subscription(Imu, g('imu_topic'), self.imu_cb, sensor_qos)
        if self.input_type == 'scan':
            self.create_subscription(LaserScan, g('scan_topic'), self.scan_cb, sensor_qos)
            src = g('scan_topic')
        else:
            self.create_subscription(PointCloud2, g('cloud_topic'), self.cloud_cb, sensor_qos)
            src = g('cloud_topic')

        self.get_logger().info(
            f"lidar_imu_odometry: {src} + {g('imu_topic')} -> {g('odom_topic')} "
            f"(publish_tf={self.publish_tf})")

    # ------------------------------------------------------------------ IMU
    def imu_cb(self, msg: Imu):
        t = stamp_to_sec(msg.header.stamp)
        wz = msg.angular_velocity.z
        a = msg.linear_acceleration

        # gyro bias: 開機前 N 秒假設車子靜止
        if not self.bias_ready:
            if self._bias_t0 is None:
                self._bias_t0 = t
            self._bias_buf.append(wz)
            if t - self._bias_t0 >= self.gyro_calib_sec and self._bias_buf:
                # 用中位數, 對少數離群值比平均數穩
                est = float(np.median(self._bias_buf))
                if abs(est) > self.gyro_calib_max:
                    self.get_logger().warn(
                        f'gyro-z bias 估出 {est:+.4f} rad/s, 超過上限 '
                        f'{self.gyro_calib_max}; 校正期間車子應該在動 -> 不採用, bias 設為 0。'
                        f'請讓車子靜止後再啟動, 或把 gyro_calib_sec 設成 0。')
                    est = 0.0
                else:
                    self.get_logger().info(f'gyro-z bias = {est:+.5f} rad/s '
                                           f'({len(self._bias_buf)} samples)')
                self.gyro_bias = est
                self.bias_ready = True
            self.last_imu_time = t
            return

        wz -= self.gyro_bias
        if self.last_imu_time is not None:
            dt = t - self.last_imu_time
            if 0.0 < dt < 0.5:
                # 梯形積分, 比矩形積分在轉彎時準
                self.imu_yaw += 0.5 * (wz + self.last_gyro_z) * dt
        self.last_imu_time = t
        self.last_gyro_z = wz

        q = msg.orientation
        if abs(q.w) + abs(q.x) + abs(q.y) + abs(q.z) > 1e-6:
            self.imu_abs_yaw = yaw_from_quat(q)

        a_horiz = math.hypot(a.x, a.y)
        self.stationary = (abs(wz) < self.zupt_gyro) and (a_horiz < self.zupt_accel)

    # --------------------------------------------------------------- sensor
    def cloud_cb(self, msg: PointCloud2):
        self._process(pointcloud2_to_xyz(msg), msg.header)

    def scan_cb(self, msg: LaserScan):
        self._process(laserscan_to_xyz(msg), msg.header)

    def _sensor_to_base(self, frame_id):
        """查一次 base_link <- sensor 的靜態外參並快取。"""
        if self._sensor_tf is not None:
            return self._sensor_tf
        if not frame_id or frame_id == self.base_frame:
            self._sensor_tf = (np.eye(3), np.zeros(3))
            return self._sensor_tf
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, rclpy.time.Time())
        except Exception as e:                                   # noqa: BLE001
            if not self._sensor_tf_warned:
                self.get_logger().warn(
                    f"查不到 TF {self.base_frame} <- {frame_id} ({e}); "
                    f"暫時當作單位矩陣, z 濾波會不準。請確認 static_transform_publisher "
                    f"的 child frame 跟感測器 header.frame_id 一致。")
                self._sensor_tf_warned = True
            return (np.eye(3), np.zeros(3))
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ])
        tr = tf.transform.translation
        self._sensor_tf = (R, np.array([tr.x, tr.y, tr.z]))
        self.get_logger().info(f'sensor extrinsic {self.base_frame} <- {frame_id}: '
                               f't={self._sensor_tf[1].round(3).tolist()}')
        return self._sensor_tf

    def _estimate_ground(self, xyz):
        """從感測器座標下的點雲量出地面高度。

        平坦地面上的回波不管距離多遠, z 都等於 -(感測器離地高度), 所以在 z
        直方圖上是一根很尖的峰。取那根峰就得到地面, 不需要任何人工參數。
        """
        z = xyz[:, 2]
        below = z[z < 0.0]
        if below.size < 4 * self.min_points:
            return None
        hist, edges = np.histogram(below, bins=120)
        i = int(np.argmax(hist))
        # 峰要夠尖才算數, 不然可能只是斜坡或雜訊
        if hist[i] < 0.15 * below.size:
            return None
        return float(0.5 * (edges[i] + edges[i + 1]))

    def _update_ground(self, xyz):
        """開頭幾幀估地面, 之後固定不變 (感測器離地高度是常數)。"""
        if self.ground_z is not None:
            return
        est = self._estimate_ground(xyz)
        if est is not None:
            self._ground_samples.append(est)
        self._ground_tries += 1
        if len(self._ground_samples) < 5:
            if self._ground_tries == 60:
                self.get_logger().warn(
                    f'auto 模式在 {self._ground_tries} 幀內量不到地面 (可能感測器'
                    f'看不到地面, 或地面不平), 改用 z_min/z_max = '
                    f'[{self.z_min}, {self.z_max}] 手動濾波。請確認這個區間正確。')
            return
        self.ground_z = float(np.median(self._ground_samples))
        tf_h = self._sensor_tf[1][2] if self._sensor_tf is not None else 0.0
        self.get_logger().info(
            f'從點雲量到地面在感測器下方 {-self.ground_z:.3f} m; '
            f'有效區間取地面以上 [{self.ground_clearance}, {self.ground_max_height}] m')
        if abs(-self.ground_z - tf_h) > 0.05:
            self.get_logger().warn(
                f'警告: static TF 說感測器裝在 {tf_h:.3f} m, 但從點雲量到的是 '
                f'{-self.ground_z:.3f} m, 差了 {abs(-self.ground_z - tf_h):.3f} m。'
                f'濾波會自動用量到的值, 但輸出位姿的座標原點會差這個高度 —— '
                f'請把 launch 的 lidar_z 改成 {-self.ground_z:.3f}。')

    def _check_full_scan(self, az):
        """確認每則訊息真的是「完整的一圈」, 這是運動補償成立的前提。

        _sweep_fraction 靠方位角當時間軸, 前提是一則訊息剛好涵蓋一整圈。
        ROS2RtxLidarHelper 的 fullScan 若是 False, Isaac 會「每個 render frame
        發一小片」(實測每則只有 26~292 點, 而完整一圈是 675x16=10800),
        這時方位角只掃過一小段, 拿它當時間軸算出來的補償量會整個錯掉。
        """
        if self._sweep_checked:
            return
        self._sweep_checked = True
        span = float(np.ptp(np.mod(az, 2.0 * math.pi)))
        if span > 0.8 * 2.0 * math.pi:
            return
        self.deskew_enable = False
        self.get_logger().warn(
            f'一則點雲只涵蓋 {math.degrees(span):.0f} 度的方位角, 不是完整的一圈。'
            f'運動補償靠方位角當時間軸, 這種資料算不出正確的補償量, 已自動關閉 '
            f'(角度也會改回完全採信 ICP)。這通常表示 ROS2RtxLidarHelper 的 '
            f'fullScan 是 False —— 請跑 scripts/fix_car_usd_lidar.py 設成 True, '
            f'快速旋轉時的定位精度差很多。')

    def _sweep_fraction(self, xyz):
        """每個點在這一圈掃描裡的相對時間 (0 = 這圈的開頭, 1 = 結尾)。

        旋轉式 LiDAR 的方位角就是它的時間軸: 掃描頭勻速轉一圈, 所以一個點的
        方位角相對於這圈起點轉過多少, 就等於它是在這圈的第幾成量到的。
        用方位角而不是用點在 buffer 裡的索引, 是因為 multiScan136 是多層掃描,
        同一個方位角會有 16 個點, 索引和時間不成正比; 方位角則永遠成正比。
        """
        az = np.arctan2(xyz[:, 1], xyz[:, 0])
        self._check_full_scan(az)
        # 以第一個點的方位角當這圈的起點, 之後的點依旋轉方向累積到 [0, 2pi)
        return np.mod(az - az[0], 2.0 * math.pi) / (2.0 * math.pi)

    def _deskew(self, xy, frac, omega, vel_body, scan_period):
        """把「累積一圈掃描的期間, 車子自己也在動」造成的變形補回去。

        RTX LiDAR 的 fullScan 累積一整圈才發一則訊息, 但整則只有一個時間戳,
        下游預設所有點是同一瞬間量到的。車子原地打轉時這個假設會壞得很徹底:
        實測這台車角速度最高 24.6 rad/s, 在 20 Hz 的掃描週期內轉了 71 度 ——
        一圈掃描的頭尾看到的是房間完全不同的兩塊, 圖形被抹成一團, 任何 ICP
        都對不上。這是 LiDAR 定位在 20 秒後開始發散的第一個原因。

        補償方式: 把每個點從「它自己被量到的那個時刻的車體座標」搬到「這圈結束
        時的車體座標」。設 δ = (1 - frac) * scan_period 是該點距離掃描結束還有
        多久, 期間車子轉了 omega*δ、平移了 vel_body*δ, 則

            p_end = Rot(-omega*δ) @ (p_meas - vel_body*δ)

        omega 用 gyro 積分出來的平均角速度 (獨立於 LiDAR, 快速旋轉時仍然可信),
        vel_body 用上一幀 ICP 量到的速度。
        """
        if not self.deskew_enable or scan_period <= 0.0:
            return xy
        # 轉角很小的時候補償量小於點雲本身的雜訊, 省下這筆計算
        if abs(omega) * scan_period < 0.01 and float(np.hypot(*vel_body)) * scan_period < 0.005:
            return xy
        delta = (1.0 - frac) * scan_period                       # (N,)
        th = -omega * delta
        c, sn = np.cos(th), np.sin(th)
        dx = xy[:, 0] - vel_body[0] * delta
        dy = xy[:, 1] - vel_body[1] * delta
        return np.stack([c * dx - sn * dy, sn * dx + c * dy], axis=1)

    def _preprocess(self, xyz, frame_id, omega=0.0, vel_body=np.zeros(2), scan_period=0.0):
        """3D 點雲 -> base_link 下的 2D 點集 (N,2), 含運動補償。"""
        if xyz.size == 0:
            return np.empty((0, 2))
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        self._raw_n = xyz.shape[0]
        if xyz.size == 0:
            return np.empty((0, 2))
        # 方位角要在感測器自己的座標系下算 (還沒轉到 base_link), 因為它代表的是
        # 掃描頭轉到哪裡, 跟感測器怎麼掛在車上無關。
        frac = self._sweep_fraction(xyz)

        R, t = self._sensor_to_base(frame_id)

        if self.ground_mode == 'auto':
            self._update_ground(xyz)
        if self.ground_mode == 'auto' and self.ground_z is not None:
            # 在感測器座標下依「離地高度」濾, 不受 static TF 高度填錯影響
            zs = xyz[:, 2] - self.ground_z
            m = (zs > self.ground_clearance) & (zs < self.ground_max_height)
            pts = xyz[m] @ R.T + t
            frac = frac[m]
        else:
            pts = xyz @ R.T + t
            m = (pts[:, 2] > self.z_min) & (pts[:, 2] < self.z_max)
            pts, frac = pts[m], frac[m]
        if pts.shape[0] == 0:
            return np.empty((0, 2))

        # 先補償運動變形, 再做距離濾波與角度分格 —— 分格用的是補償後的角度才對
        xy = self._deskew(pts[:, :2], frac, omega, vel_body, scan_period)

        r = np.hypot(xy[:, 0], xy[:, 1])
        m = (r > self.range_min) & (r < self.range_max)
        xy, r = xy[m], r[m]
        if xy.shape[0] == 0:
            return np.empty((0, 2))

        self._diagnose(xyz, pts, xy.shape[0])

        # 依角度分格, 每格只留最近的一點 -> 等效把 3D 壓成一張乾淨的 2D scan
        ang = np.arctan2(xy[:, 1], xy[:, 0])
        b = ((ang + math.pi) / (2 * math.pi) * self.angle_bins).astype(np.int32)
        np.clip(b, 0, self.angle_bins - 1, out=b)
        order = np.lexsort((r, b))              # 先照 bin, 同 bin 內照距離
        b_sorted = b[order]
        first = np.ones(b_sorted.size, dtype=bool)
        first[1:] = b_sorted[1:] != b_sorted[:-1]
        return xy[order][first]

    def _diagnose(self, raw, kept, n_kept):
        """第一幀點雲印出 z 分布與濾波結果。

        最常見也最難察覺的設定錯誤是 static TF 的感測器高度寫錯: z 濾波會整個
        錯位, 地面點全部留下來。地面回波比牆近得多, 會贏走每一個角度格, 於是
        掃描圖形每幀都不一樣, ICP 靜默失效 —— 位姿看起來就是「不太會動」。
        """
        if self._diag_done:
            return
        self._diag_done = True
        if kept.shape[0] == 0:
            self.get_logger().warn(
                f'警告: 本幀 {raw.shape[0]} 個點全部被 z 濾波 '
                f'[{self.z_min}, {self.z_max}] 濾掉了。請確認 static TF 的感測器'
                f'高度, 並依實際點雲高度調整 z_min / z_max。')
            return
        z = kept[:, 2]
        # 仰角要用「原始、未經濾波」的點雲算, 而且在感測器自己的座標系下,
        # 這樣才反映感測器實際看得到什麼, 不受 z 濾波與 TF 設定影響。
        el = np.degrees(np.arctan2(raw[:, 2],
                                   np.maximum(np.hypot(raw[:, 0], raw[:, 1]), 1e-6)))
        self.get_logger().info(
            f'第一幀點雲診斷: 原始 {raw.shape[0]} 點, z 濾波後 {kept.shape[0]} 點; '
            f'濾波後 base_link z 範圍 '
            f'[{z.min():.3f}, {z.max():.3f}] 中位數 {np.median(z):.3f}; '
            f'原始仰角 [{el.min():+.1f}, {el.max():+.1f}] 度; '
            f'z 濾波 [{self.z_min}, {self.z_max}] 後剩 {n_kept} 點')
        # 感測器如果被自己的車身/輪子擋住, 看不到水平線以下的東西 ->
        # 少了地面與下半部結構, 掃描比對會很不穩
        if el.min() > -5.0:
            self.get_logger().warn(
                f'警告: 最低仰角只有 {el.min():+.1f} 度, 完全看不到水平線以下。'
                f'通常表示 LiDAR 被自己的車身或輪子擋住 (裝太低或裝在車殼內部), '
                f'請把感測器移到高過車身與輪子的位置。')
        if n_kept == 0:
            self.get_logger().warn(
                f'警告: z 濾波 [{self.z_min}, {self.z_max}] 把所有點都濾掉了, '
                f'本幀無法比對。請依上面的 z 範圍調整 z_min / z_max, '
                f'並確認 static TF 的感測器高度正確。')
            return
        # 地面是一個薄平面: 若留下來的點有一大半擠在最低的 5cm 內, 幾乎可以確定
        # 是地面沒被濾掉 (通常代表 static TF 的高度跟實際感測器高度不符)
        lo = z.min()
        frac = float(np.mean(z < lo + 0.05))
        # 點數太少的時候這個比例沒有意義 (1 個點必然是 100%)
        if n_kept >= self.min_points and frac > 0.6:
            self.get_logger().warn(
                f'警告: 濾波後有 {100*frac:.0f}% 的點擠在最低的 5 cm ({lo:.3f}~{lo+0.05:.3f} m) '
                f'內, 這幾乎一定是「地面沒被濾掉」。請確認 static TF 的感測器高度是否等於'
                f'實際掛載高度, 並讓 z_min 高過地面。地面點會讓 ICP 靜默失效。')

    # ------------------------------------------------------------------ core
    def _check_clock_skew(self, t_sensor):
        """感測器時間戳與本節點時鐘差太多 -> 下游 EKF 會整批丟棄我們的訊息。"""
        if self.stamp_source != 'auto' or self._use_node_clock:
            return
        if len(self._skew_samples) >= 20:
            return
        self._skew_samples.append(t_sensor - self.get_clock().now().nanoseconds * 1e-9)
        if len(self._skew_samples) < 20:
            return
        skew = float(np.median(self._skew_samples))
        if abs(skew) <= self.stamp_skew_thresh:
            return
        self._use_node_clock = True
        self.get_logger().warn(
            f'警告: 點雲的時間戳比本節點時鐘快了 {skew:+.1f} 秒。這表示 Isaac 裡的'
            f'各個 publisher 用了不同的時間源 —— /clock 與 IMU 跟著 playback (按 Play '
            f'歸零), 但 RTX LiDAR 的 resetSimulationTimeOnStop 預設是 False (跨 '
            f'Stop/Play 單調累加)。這樣 robot_localization 會把我們的 odometry 當成'
            f'未來資料丟掉, EKF 只剩 IMU, 位置永遠卡在原點。'
            f'本節點先改用自己的時鐘發布以繞過問題; 根治請跑 '
            f'scripts/fix_car_usd_lidar.py 把時間源設成一致。')

    def _process(self, xyz, header):
        t_now = stamp_to_sec(header.stamp)
        self._check_clock_skew(t_now)

        # 掃描週期 = 相鄰兩則 fullScan 的間隔。運動補償要用它把「點被量到的時刻」
        # 換算成距離掃描結束多久, 所以必須在前處理之前先算出來。
        if self.last_scan_time is None:
            dt = 0.0
        else:
            dt = t_now - self.last_scan_time
            if dt <= 0.0 or dt > 1.0:
                dt = 0.0

        # --- IMU 給的角度增量。這是本節點唯一獨立於 LiDAR 的角度來源, 既是 ICP 的
        #     初始猜測, 也是後面用來裁決 ICP 對不對的依據。---
        if self.use_imu_orient and self.imu_abs_yaw is not None \
                and self.imu_abs_yaw_at_last_scan is not None:
            d_yaw = wrap_pi(self.imu_abs_yaw - self.imu_abs_yaw_at_last_scan)
        elif self.imu_yaw_at_last_scan is not None:
            # 用未 wrap 的連續積分值相減: 一幀轉超過 180 度時 wrap 會把方向弄反
            d_yaw = self.imu_yaw - self.imu_yaw_at_last_scan
        else:
            d_yaw = 0.0
        omega = d_yaw / dt if dt > 0.0 else 0.0

        pts = self._preprocess(xyz, header.frame_id, omega, self.last_vel[:2], dt)

        self._st['n'] += 1
        self._st_total += 1
        self._st['max_rot'] = max(self._st['max_rot'], abs(d_yaw))
        self._st['raw'] += self._raw_n
        if not self.stationary:
            self._st['moving'] += 1
        self._st['kept'] += pts.shape[0]
        if pts.shape[0] < self.min_points:
            self._st['thin'] += 1
            self.get_logger().warn(
                f'有效點數不足 (原始 {self._raw_n} -> 濾波後 {pts.shape[0]} < '
                f'{self.min_points}), 跳過這一幀', throttle_duration_sec=2.0)
            return

        # 第一幀: 建立參考 (地圖模式下, 這一幀就定義了地圖原點)
        if (self.map_tree is None) if self.map_mode else (self.kf_tree is None):
            if self.map_mode:
                self._map_insert(pts, self.pose)
            else:
                self._set_keyframe(pts)
            self.last_scan_time = t_now
            self.imu_yaw_at_last_scan = self.imu_yaw
            self.imu_abs_yaw_at_last_scan = self.imu_abs_yaw
            self._publish(header.stamp, 1.0)
            return

        if dt <= 0.0:
            dt = 1e-3

        # --- 位姿先驗: 上一幀位姿 + (等速平移, IMU 轉角) ---
        d_xy_body = self.last_vel[:2] * dt
        prior = self._compose(self.pose, np.array([d_xy_body[0], d_xy_body[1], d_yaw]))

        # --- 轉太快就不要比對 ---
        # 運動補償有極限: 掃描期間轉角太大時, 這圈的頭尾看到的是房間完全不同的
        # 兩塊, 重疊區小到再怎麼補償都對不上。這種幀硬算出來的一定是錯的答案,
        # 而且錯得「很有自信」(矩形房間的 inlier 依然漂亮)。直接放棄, 用 IMU 撐過去,
        # 並且把共變異數放大讓下游 EKF 知道這段不要信。
        if abs(d_yaw) > self.max_scan_rot:
            self._st['spin'] += 1
            self.get_logger().warn(
                f'掃描期間轉了 {math.degrees(abs(d_yaw)):.0f} 度 (上限 '
                f'{math.degrees(self.max_scan_rot):.0f}), 掃描圖形無法比對, '
                f'這一幀改用 IMU 推估', throttle_duration_sec=2.0)
            self._coast(prior, t_now, header.stamp)
            return

        # 從放棄比對的狀態恢復。
        # 地圖模式不需要做任何事: 地圖是絕對參考, 一直有效, 下一幀直接對它配準就會
        # 自己把 IMU 推估期間漂掉的量修回來 —— 這正是地圖模式相對於關鍵幀模式的
        # 根本差別 (關鍵幀模式沒有絕對參考, 漂掉的量永遠回不來)。
        if self._need_keyframe and not self.map_mode:
            self.pose = prior
            self._set_keyframe(pts)
            self._need_keyframe = False
            self.last_scan_time = t_now
            self.imu_yaw_at_last_scan = self.imu_yaw
            self.imu_abs_yaw_at_last_scan = self.imu_abs_yaw
            # 這一幀只是重新錨定關鍵幀, 位姿本身還是 IMU 推出來的, 共變異數照樣放大
            self._publish(header.stamp, -1.0)
            return

        # 配準的參考。地圖模式下地圖就存在里程計座標系裡, 所以初始猜測直接就是
        # 位姿先驗 (等同於 ref_pose = 原點); 關鍵幀模式則要換算到關鍵幀座標系下。
        ref_pose = np.zeros(3) if self.map_mode else self.kf_pose
        ref_tree = self.map_tree if self.map_mode else self.kf_tree
        init = self._between(ref_pose, prior)

        self._verify_deskew(xyz, header.frame_id, omega, dt, ref_tree, init)

        R, tvec, inlier, err, yaw_free = self._icp(
            pts, ref_tree, rot2(init[2]), init[:2].copy(),
            yaw_prior=init[2], gain=self._yaw_gain(d_yaw))

        self._st['inlier'] += inlier
        if inlier < self.icp_min_inlier:
            self._st['fail'] += 1
            self.get_logger().warn(
                f'ICP 比對品質不佳 (inlier={inlier:.2f}, err={err:.3f}), 這一幀改用 IMU 推估',
                throttle_duration_sec=2.0)
            self._coast(prior, t_now, header.stamp)
            return

        rel = np.array([tvec[0], tvec[1], math.atan2(R[1, 0], R[0, 0])])
        new_pose = self._compose(ref_pose, rel)
        fitness = inlier

        # --- 單幀粗差保護: ICP 的轉角與 gyro 差太多就丟掉這一幀 ---
        # 這是保險, 不是主角。實測 de-skew 正常運作時單幀分歧最多 0.6 度, 這裡
        # 不會觸發; 它擋的是突發的大幅錯配 (感測器被擋住、訊息中斷後關鍵幀過期)。
        # 比較的必須是「未受 gyro 先驗約束」的 ICP 解 —— 約束後的值已經被拉向
        # gyro 了, 拿它來檢查等於自己驗證自己。
        # 比的是「相對於參考」的轉角: yaw_free 是 ICP 自己算的, init[2] 是 gyro 推的。
        disagree = wrap_pi(yaw_free - init[2])
        gate = self.yaw_gate_base + self.yaw_gate_ratio * abs(d_yaw)
        if abs(disagree) > gate:
            self._st['gated'] += 1
            self.get_logger().warn(
                f'ICP 相對關鍵幀轉了 {math.degrees(yaw_free):+.1f} 度, gyro 說是 '
                f'{math.degrees(init[2]):+.1f} 度, 差 {math.degrees(abs(disagree)):.1f} 度 '
                f'(容差 {math.degrees(gate):.1f}), inlier={inlier:.2f} —— 多半是對到'
                f'矩形房間的另一個方向。採信 gyro, 丟棄這次比對。',
                throttle_duration_sec=2.0)
            self._coast(prior, t_now, header.stamp)
            return

        # --- 更新速度 (給下一幀當先驗, 也放進 Odometry.twist) ---
        delta = self._between(self.pose, new_pose)

        # --- ZUPT: 只在「IMU 說靜止」而且「ICP 也量到幾乎沒動」時才凍結位姿。
        # 這裡是事後確認而不是事前把關 -> ICP 每一幀都照跑, 不可能鎖死。
        if (self.zupt_enable and self.stationary
                and math.hypot(delta[0], delta[1]) < self.zupt_speed * dt
                and abs(delta[2]) < self.zupt_gyro * dt):
            new_pose = self.pose.copy()
            delta = np.zeros(3)
        self.last_vel = np.array([delta[0] / dt, delta[1] / dt, wrap_pi(delta[2]) / dt])
        self.pose = new_pose
        self.last_scan_time = t_now
        self.imu_yaw_at_last_scan = self.imu_yaw
        self.imu_abs_yaw_at_last_scan = self.imu_abs_yaw

        # --- 更新參考 ---
        if self.map_mode:
            # 只有配準品質夠好的幀才准寫進地圖 —— 品質差的幀位姿不可信, 讓它寫入
            # 等於把誤差固化進地圖, 之後每一幀都會被它帶偏。
            self._map_maybe_end_build(t_now)
            if inlier >= self.map_min_inlier:
                self._map_insert(pts, self.pose)
        else:
            rel_kf = self._between(self.kf_pose, self.pose)
            if math.hypot(rel_kf[0], rel_kf[1]) > self.kf_dist \
                    or abs(rel_kf[2]) > self.kf_angle:
                self._set_keyframe(pts)

        if self.log_stats:
            self.get_logger().info(
                f'dt={dt:.3f} pts={pts.shape[0]:4d} inlier={inlier:.2f} err={err:.4f} '
                f'd_yaw_imu={math.degrees(d_yaw):+6.2f}deg '
                f'prior=({prior[0]:.2f},{prior[1]:.2f}) pose=({self.pose[0]:.2f},{self.pose[1]:.2f},'
                f'{math.degrees(self.pose[2]):.1f})')

        self._check_in_map()
        self._publish(header.stamp, fitness)

    def _coast(self, prior, t_now, stamp):
        """放棄這一幀的掃描比對, 純靠 IMU 推估位姿。

        角度仍然是可信的 (gyro 直接量角速度, 快速旋轉時反而是最可靠的來源),
        會漂的是位置 —— 只能用上一幀的速度外推。所以:
          - 速度先驗逐幀衰減: 抓著一個過期的速度連續外推好幾幀, 誤差累積得比
            衰減到 0 更快, 尤其原地打轉時車子其實幾乎沒有平移。
          - 標記關鍵幀過期: 恢復比對時要用當下的掃描重建, 不能拿舊的硬對。
          - fitness 傳 -1: 讓 _publish 給出很大的共變異數, 下游 EKF 會自動改用
            IMU 而不是相信這段位置。
        """
        self.pose = prior
        self.last_vel = np.array([self.last_vel[0] * self.coast_vel_decay,
                                  self.last_vel[1] * self.coast_vel_decay,
                                  0.0])
        self._need_keyframe = True
        self.last_scan_time = t_now
        self.imu_yaw_at_last_scan = self.imu_yaw
        self.imu_abs_yaw_at_last_scan = self.imu_abs_yaw
        self._publish(stamp, -1.0)

    def _status(self):
        """每隔幾秒印一行健康狀態, 遇到問題直接貼這行就能判斷卡在哪一關。"""
        st = self._st
        if st['n'] == 0:
            self.get_logger().warn(
                '狀態: 這段期間沒有收到任何點雲。請確認 cloud_topic 是否正確 '
                '(ros2 topic hz /lidar/point_cloud), 以及 Isaac 是否在 Play。')
            return
        moved = math.hypot(self.pose[0] - self._st_pose[0], self.pose[1] - self._st_pose[1])
        turned = math.degrees(abs(wrap_pi(self.pose[2] - self._st_pose[2])))
        ok = st['n'] - st['fail'] - st['thin']
        ok -= st['spin'] + st['gated']
        self.get_logger().info(
            f"狀態: {st['n']} 幀 | 點數 原始 {st['raw'] // st['n']} -> 濾波後 "
            f"{st['kept'] // st['n']} | ICP 成功 {ok}/{st['n']} "
            f"(inlier 均 {st['inlier'] / max(st['n'] - st['thin'], 1):.2f}) | "
            f"轉太快跳過 {st['spin']} | gyro 否決 {st['gated']} | "
            f"單幀最大轉角 {math.degrees(st['max_rot']):.0f} 度 | "
            f"本段移動 {moved:.3f} m / 轉 {turned:.1f} 度")
        if self.map_mode:
            phase = '只補洞' if self._map_gap_only else '建圖中'
            self.get_logger().info(
                f"地圖: {len(self._map_cells)} 格 ({phase}), 本段新增 "
                f"{len(self._map_cells) - self._st_cells} 格")
            # 建圖期結束後還在大量新增, 表示補洞門檻太鬆, 開始長鬼牆了。
            # 正常補洞只會填柱子後面的死角, 幾百格就飽和。
            if self._map_gap_only and len(self._map_cells) - self._st_cells > 500:
                self.get_logger().warn(
                    f'地圖在建圖期結束後還在快速長大 (本段 +'
                    f'{len(self._map_cells) - self._st_cells} 格)。正常只該補柱子後面的'
                    f'死角然後飽和; 持續長大表示位姿誤差已經大過 map_gap_dist '
                    f'({self.map_gap_dist} m), 寫進去的是鬼牆。請把 map_gap_dist 調大。')
            self._st_cells = len(self._map_cells)
        if st['spin'] > st['n'] * 0.3:
            self.get_logger().warn(
                f"狀態: {100 * st['spin'] // st['n']}% 的幀因為轉太快而無法比對 "
                f"(單幀最大 {math.degrees(st['max_rot']):.0f} 度)。這段期間位置是純靠 "
                f"IMU 外推的, 會漂。根治要提高 LiDAR 的掃描頻率 —— fullScan 在 "
                f"20 Hz 下, 車子轉 20 rad/s 就等於一圈掃描被抹開 57 度。")
        if st['gated'] > st['n'] * 0.2:
            self.get_logger().warn(
                f"狀態: {100 * st['gated'] // st['n']}% 的幀 ICP 轉角與 gyro 不合而被否決。"
                f"矩形房間本來就容易對錯邊; 若比例持續偏高, 檢查 gyro bias 是否估準 "
                f"(啟動前車子要靜止), 或把 yaw_gate_base 稍微放寬。")
        # 地面沒濾乾淨時 ICP 會對齊到「跟著車子一起走的地面」: inlier 很漂亮,
        # 轉角也對, 但位移恆為 0 —— 完全不會報錯, 只能靠這裡抓出來。
        if (st['n'] >= 40 and moved < 0.02 and st['moving'] > st['n'] * 0.5
                and st['fail'] + st['spin'] + st['gated'] < st['n'] * 0.5):
            self.get_logger().warn(
                'ICP 量到幾乎沒有位移, 但 IMU 顯示車子在動。這幾乎一定是地面回波'
                '沒有濾乾淨 —— 地面會跟著車子一起移動, ICP 對齊到它就得到「沒有'
                '位移」, 而且 inlier 會很漂亮, 不會報錯。請確認 ground_mode 是 auto, '
                '或把 static TF 的感測器高度改成正確值。')
        if st['thin'] > st['n'] * 0.5:
            self.get_logger().warn('狀態: 過半的幀點數不足 -> 濾波區間或感測器高度不對')
        elif st['fail'] > st['n'] * 0.5:
            self.get_logger().warn(
                '狀態: 過半的幀 ICP 比對失敗 -> 通常是地面沒濾乾淨, 或場景結構不足, '
                '或車子轉太快 (幀間轉角過大)')
        self._st = dict(n=0, raw=0, kept=0, inlier=0.0, fail=0, nocloud=0, thin=0,
                        moving=0, spin=0, gated=0, max_rot=0.0)
        self._st_pose = self.pose.copy()

    def _load_map(self, path):
        """載入事先建好、而且人眼確認過的地圖, 直接進入純定位。"""
        from car_navigation import gridmap
        points, meta = gridmap.load(path)
        st = gridmap.stats(points, self.map_grid)
        self.map_pts = points
        self._map_cells = set(
            map(tuple, np.floor(points / self.map_grid).astype(np.int64).tolist()))
        self.map_tree = cKDTree(self.map_pts)
        self._map_bounds = (self.map_pts.min(axis=0), self.map_pts.max(axis=0))
        # 已經有地圖了, 不要再無條件累積。而且既然這張地圖是事先建好、人眼確認過的,
        # 預設連補洞都不要 —— 純定位, 地圖完全不動, 就不可能長出鬼牆。
        # (真的需要邊定位邊補未知區域時, 才把 map_gap_fill 設回 true。)
        self._map_gap_only = True
        if self.map_gap_fill:
            self.get_logger().info(
                'map_gap_fill=true 且載入了既有地圖: 定位過程仍會往地圖補未知區域。'
                '若這張地圖已經完整, 建議設成 false 讓地圖完全不動。')
        self.pose = np.array([float(self.initial_pose[0]), float(self.initial_pose[1]),
                              math.radians(float(self.initial_pose[2]))])
        self.get_logger().info(
            f'載入地圖 {path}: {st["n_points"]} 點 / {st["n_cells"]} 格, '
            f'範圍 {st["extent"][0]:.2f} x {st["extent"][1]:.2f} m, '
            f'座標系 {meta["frame_id"]} (建於 {meta.get("created", "?")})')
        self.get_logger().info(
            f'起始位姿 (地圖座標系): x={self.pose[0]:.3f} y={self.pose[1]:.3f} '
            f'yaw={math.degrees(self.pose[2]):.1f} 度')
        # 牆面應該是薄的。點間距的 p95 明顯大於格點解析度, 通常代表地圖有鬼牆
        # (建圖時位姿漂了, 同一面牆被寫成兩條)。
        if st['nn_p95'] > 4.0 * self.map_grid:
            self.get_logger().warn(
                f'地圖的點間距 p95 是 {st["nn_p95"]:.3f} m, 比格點 {self.map_grid} m '
                f'大不少 —— 這張地圖可能有鬼牆。請先用 '
                f'"python3 -m car_navigation.gridmap show {path}" 看過再用。')

    def _save_map_srv(self, request, response):
        path = self.save_map()
        response.success = path is not None
        response.message = f'已存到 {path}' if path else '存檔失敗 (地圖是空的或沒設路徑)'
        return response

    def save_map(self, path=None):
        """把目前的地圖存檔。建圖跑完之後呼叫。"""
        from car_navigation import gridmap
        path = path or self.map_save_path
        if not path:
            self.get_logger().warn('沒有指定 map_save_path, 不存檔')
            return None
        if self.map_pts.shape[0] == 0:
            self.get_logger().error('地圖是空的, 不存檔')
            return None
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
        st = gridmap.stats(self.map_pts, self.map_grid)
        gridmap.save(path, self.map_pts, frame_id=self.odom_frame, grid=self.map_grid,
                     source=f'lidar_imu_odometry 建圖 ({self._st_total} 幀點雲)')
        self.get_logger().info(
            f'地圖已存到 {path}: {st["n_points"]} 點 / {st["n_cells"]} 格, '
            f'範圍 {st["extent"][0]:.2f} x {st["extent"][1]:.2f} m, '
            f'點間距 p95 {st["nn_p95"]:.3f} m')
        self.get_logger().info(
            f'請務必先看過再拿去定位: python3 -m car_navigation.gridmap show {path}')
        return path

    def _verify_deskew(self, xyz, frame_id, omega, dt, ref_tree, prior_rel):
        """拿真實資料檢查運動補償到底有沒有幫上忙。

        做法: 同一幀點雲, 補償版與未補償版都用同一個位姿先驗投影到參考
        (地圖或關鍵幀) 上, 量「每個點到最近的參考點有多遠」的中位數。
        補償如果是對的, 這個距離會明顯變小; 如果方位角當時間軸的假設不成立,
        補償只會把點推到更不對的地方, 距離反而變大。
        累積幾十個轉動中的幀再表決, 免得被單一幀的雜訊誤導。
        """
        if self._deskew_verified or self.deskew_verify_n <= 0:
            return
        if abs(omega) * dt < self.deskew_verify_min_rot:
            return                                     # 沒在轉的幀分辨不出差別

        def fit(pts):
            if pts.shape[0] < self.min_points:
                return None
            c, sn = math.cos(prior_rel[2]), math.sin(prior_rel[2])
            p = np.stack([prior_rel[0] + c * pts[:, 0] - sn * pts[:, 1],
                          prior_rel[1] + sn * pts[:, 0] + c * pts[:, 1]], axis=1)
            d = ref_tree.query(p)[0]
            d = d[np.isfinite(d)]
            return float(np.median(d)) if d.size >= self.min_points else None

        keep = self.deskew_enable
        self.deskew_enable = True
        on = fit(self._preprocess(xyz, frame_id, omega, self.last_vel[:2], dt))
        self.deskew_enable = False
        off = fit(self._preprocess(xyz, frame_id, 0.0, np.zeros(2), 0.0))
        self.deskew_enable = keep
        if on is None or off is None:
            return
        self._deskew_votes.append((on, off))

        if len(self._deskew_votes) < self.deskew_verify_n:
            return
        v = np.array(self._deskew_votes)
        m_on, m_off = float(np.median(v[:, 0])), float(np.median(v[:, 1]))
        wins = int((v[:, 0] < v[:, 1]).sum())
        self._deskew_verified = True
        if m_on < m_off * 0.9:
            self.get_logger().info(
                f'運動補償驗證通過: 掃描貼合參考的距離 {m_off:.3f} -> {m_on:.3f} m '
                f'({wins}/{len(v)} 幀變好)。維持開啟。')
        else:
            self.deskew_enable = False
            self.get_logger().warn(
                f'運動補償沒有幫助, 已自動關閉: 貼合距離 {m_off:.3f} -> {m_on:.3f} m '
                f'({wins}/{len(v)} 幀變好)。這表示「方位角 = 掃描時間軸」的假設在'
                f'這個感測器上不成立 —— 可能是點雲不是一整圈 (ROS2RtxLidarHelper 的 '
                f'fullScan 沒開)、訊息週期不等於掃描週期, 或方位角不是勻速掃描。'
                f'快速旋轉時的精度會因此受限, 但至少不會被錯誤的補償弄得更糟。')

    def _check_in_map(self):
        """位姿跑出地圖範圍就大聲報錯。

        定位失效時最糟的不是「不準」, 而是「安靜地輸出垃圾」—— 實測失敗的那次
        位置飄到 166 m, 而房間只有 10x6 m, 下游卻完全不知道該不該信。
        地圖的邊界是已知的, 車子不可能在地圖外面, 所以這是一個免費又可靠的檢查。
        """
        if self.map_tree is None or self._map_bounds is None:
            return
        lo, hi = self._map_bounds
        m = self.map_oob_margin
        if lo[0] - m <= self.pose[0] <= hi[0] + m and lo[1] - m <= self.pose[1] <= hi[1] + m:
            self._oob = 0
            return
        self._oob += 1
        if self._oob in (1, 20) or self._oob % 200 == 0:
            self.get_logger().error(
                f'位姿 ({self.pose[0]:+.2f}, {self.pose[1]:+.2f}) 已經跑到地圖外面了 '
                f'(地圖 x[{lo[0]:+.2f},{hi[0]:+.2f}] y[{lo[1]:+.2f},{hi[1]:+.2f}], '
                f'容許超出 {m} m), 連續 {self._oob} 幀。定位已經失效, '
                f'下游不應該相信 {self.odom_frame} 的輸出。'
                f'常見原因: 起始位姿 (initial_pose) 填錯、地圖跟現場對不上、'
                f'或點雲前處理把牆濾掉了。請看上面的「狀態:」那幾行。')

    def _map_maybe_end_build(self, t_now):
        """建圖期一到就切換成「只補洞」模式 (見 map_gap_dist 的說明)。"""
        if self._map_gap_only:
            return
        if self._map_t0 is None:
            self._map_t0 = t_now
            return
        if t_now - self._map_t0 < self.map_build_sec:
            return
        self._map_gap_only = True
        mode = (f'之後只補「離既有地圖 {self.map_gap_dist} m 以上」的空白處'
                if self.map_gap_fill else '之後不再寫入')
        self.get_logger().info(
            f'地圖建圖期結束: {self.map_pts.shape[0]} 點 / {len(self._map_cells)} 格 '
            f'(格點 {self.map_grid} m); {mode}。每一幀都直接對這張地圖配準, '
            f'位姿是相對於地圖原點的絕對值, 誤差不會累積。')
        # 這個房間的牆加柱子大約 800~1500 格。遠低於這個數字表示建圖期看到的
        # 結構太少 (可能被擋住, 或 z 濾波太嚴), 之後的定位會不穩。
        if len(self._map_cells) < 300:
            self.get_logger().warn(
                f'地圖只有 {len(self._map_cells)} 格, 偏少。請確認 z 濾波區間與'
                f'感測器高度正確, 或把 map_build_sec 調大讓車子先跑一段再結束建圖。')

    def _map_insert(self, pts_body, pose):
        """把這一幀的點依 pose 轉到地圖座標後寫進地圖 (格點去重)。"""
        if self.map_pts.shape[0] >= self.map_max_points:
            return
        if self._map_gap_only and not self.map_gap_fill:
            return
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        w = np.stack([pose[0] + c * pts_body[:, 0] - sn * pts_body[:, 1],
                      pose[1] + sn * pts_body[:, 0] + c * pts_body[:, 1]], axis=1)

        cells = np.floor(w / self.map_grid).astype(np.int64)
        keys = [(int(a), int(b)) for a, b in cells]
        fresh = [i for i, k in enumerate(keys) if k not in self._map_cells]
        if not fresh:
            return
        # 只補洞模式: 只收「離地圖上任何既有點都夠遠」的點, 擋掉位姿誤差造成的鬼牆
        if self._map_gap_only and self.map_tree is not None:
            far = self.map_tree.query(w[fresh])[0] > self.map_gap_dist
            fresh = [fresh[i] for i in np.nonzero(far)[0]]
            if not fresh:
                return

        self._map_cells.update(keys[i] for i in fresh)
        self.map_pts = np.vstack([self.map_pts, w[fresh]])
        self._map_pending += len(fresh)

        # 建圖期一次進來上千點, 分批重建 KD-tree 省時間。
        # 但補洞期整段加起來也才一兩百點, 用同一個門檻會跨不過去 —— 補進來的點
        # 就永遠不會進到樹裡, 等於白補。所以補洞期一有新點就立刻重建
        # (這個房間的地圖只有一千多點, 重建成本可以忽略)。
        thresh = 1 if self._map_gap_only else self.map_rebuild_every
        if self.map_tree is None or self._map_pending >= thresh:
            self.map_tree = cKDTree(self.map_pts)
            self._map_pending = 0
            self._map_bounds = (self.map_pts.min(axis=0), self.map_pts.max(axis=0))

    def _set_keyframe(self, pts):
        self.kf_tree = cKDTree(pts)
        self.kf_pose = self.pose.copy()

    @staticmethod
    def _compose(a, d):
        """a(世界位姿) ⊕ d(a 的車體座標下的增量) -> 新的世界位姿。"""
        c, s = math.cos(a[2]), math.sin(a[2])
        return np.array([a[0] + c * d[0] - s * d[1],
                         a[1] + s * d[0] + c * d[1],
                         wrap_pi(a[2] + d[2])])

    @staticmethod
    def _between(a, b):
        """求 a 到 b 的相對位姿 (表示在 a 的車體座標下)。"""
        c, s = math.cos(a[2]), math.sin(a[2])
        dx, dy = b[0] - a[0], b[1] - a[1]
        return np.array([c * dx + s * dy, -s * dx + c * dy, wrap_pi(b[2] - a[2])])

    def _yaw_gain(self, d_yaw):
        """ICP 修正角度的增益, 隨這一幀的轉角遞減 (見 yaw_icp_gain 的說明)。

        兩種情況一律回傳 1.0 (= 完全採信 ICP, 不做約束):

        1) 地圖模式。用 gyro 約束角度的理由是「關鍵幀之間的 ICP 角度會隨機遊走」
           —— 轉快時關鍵幀每幀重建, 每次比對的幾度誤差會一路累積。但對絕對地圖
           配準時每一幀都是獨立的絕對量測, 根本沒有累積, 那個理由就不成立了。
           實測差別很大, 而且是在「gyro 不好」的時候差最多:
                              模擬器   好MEMS  便宜MEMS  bias沒校正
             約束 (gain 0.5)   0.024   0.031    0.305     3.917
             不約束 (gain 1.0)  0.023   0.024    0.025     0.029
           也就是說地圖模式下的定位精度幾乎與 gyro 品質無關 —— 這對 sim2real
           很重要, 真車的 IMU 不會有模擬器這麼乾淨。
           (gyro 在地圖模式下依然不可或缺: ICP 的初始猜測、運動補償的角速度、
            放棄比對時的推估都要靠它, 只是不該再混進最終的角度。)

        2) 沒有做運動補償時。約束旋轉的前提是「掃描的幾何是對的, 只有角度需要
           交給 gyro」。掃描本身還是歪的時候把旋轉鎖死, ICP 就只能把對不起來的
           部分全部推給平移 —— 實測這種組合比完全不約束還糟 (0.28 m -> 6.2 m)。
        """
        if self.map_mode or not self.deskew_enable:
            return 1.0
        if self.yaw_gain_knee <= 0.0:
            return self.yaw_icp_gain
        return self.yaw_icp_gain / (1.0 + (abs(d_yaw) / self.yaw_gain_knee) ** 2)

    def _icp(self, src, tree, R, t, yaw_prior=None, gain=1.0):
        """Trimmed point-to-point ICP。回傳 (R, t, inlier_ratio, mean_error)。

        回傳的 yaw_free 是「完全不受 gyro 先驗約束」的 ICP 旋轉解。呼叫端要拿它
        (而不是拿約束後的結果) 去跟 gyro 對帳 —— 約束後的值本來就已經被拉向
        gyro 了, 拿它來檢查等於自己驗證自己, 永遠不會發現問題。

        yaw_prior 不是 None 時, 每次迭代都把總旋轉拉回 gyro 給的先驗
        (拉的比例由 gain 決定), 然後在「旋轉已固定」的前提下重新解平移。

        為什麼要在迭代裡面做, 而不是事後把 yaw 換掉:
            ICP 回傳的是一個剛體變換 —— 旋轉與平移是一起解出來、互相配套的。
            事後只把旋轉換成 gyro 的值、留著原本的平移, 兩者就不再是同一個解,
            位置反而會變差 (實測 yaw 從 11.2 度降到 0.3 度, 但位置從 0.76 m
            惡化到 1.82 m)。正確做法是把旋轉當成已知, 重新解對應它的最佳平移
            —— 固定旋轉下的最佳平移有閉式解, 就是兩組對應點形心的差。
        """
        n_src = src.shape[0]
        inlier, mean_err = 0.0, float('inf')
        yaw_free = math.atan2(R[1, 0], R[0, 0])
        dst = tree.data
        max_corr = self.icp_max_corr

        for _ in range(self.icp_max_iter):
            p = src @ R.T + t
            d, idx = tree.query(p, k=1, distance_upper_bound=max_corr)
            valid = np.isfinite(d)
            nv = int(valid.sum())
            if nv < self.min_points:
                return R, t, nv / max(n_src, 1), mean_err, yaw_free

            dv, pv, iv = d[valid], p[valid], idx[valid]
            keep = max(self.min_points, int(nv * self.icp_keep))
            sel = np.argsort(dv)[:keep]
            a, b = pv[sel], dst[iv[sel]]

            ca, cb = a.mean(axis=0), b.mean(axis=0)
            A, B = a - ca, b - cb
            th = math.atan2(float(np.sum(A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0])),
                            float(np.sum(A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1])))
            Rd = rot2(th)
            td = cb - Rd @ ca

            # ca 是「已經被這一輪開頭的 R, t 搬過」的形心; 還原成原始 src 座標下的
            # 形心, 等一下固定旋轉重解平移時要用 (R 是正交矩陣, 轉置即為逆)
            src_c = R.T @ (ca - t)

            R_free = Rd @ R
            t_free = Rd @ t + td
            yaw_free = math.atan2(R_free[1, 0], R_free[0, 0])

            if yaw_prior is None or gain >= 1.0:
                R, t = R_free, t_free
            else:
                # 把總旋轉往 gyro 先驗拉回去, 再在這個旋轉下重解最佳平移
                R = rot2(wrap_pi(yaw_prior + gain * wrap_pi(yaw_free - yaw_prior)))
                t = cb - R @ src_c

            inlier = nv / n_src
            mean_err = float(np.mean(dv[sel]))
            # 收斂就提早結束, 並逐步縮小對應距離讓比對更精細
            if abs(th) < self.icp_tol and float(np.hypot(*td)) < self.icp_tol:
                break
            max_corr = max(self.icp_max_corr * 0.3, max_corr * 0.9)

        return R, t, inlier, mean_err, yaw_free

    # --------------------------------------------------------------- output
    def _publish(self, stamp, fitness):
        if self._use_node_clock:
            stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.pose
        qx, qy, qz, qw = quat_from_yaw(yaw)

        # 比對品質越差, 共變異數給越大, 讓下游 EKF 自動降低對它的信任。
        # fitness < 0 表示這一幀根本沒有比對 (轉太快 / 被 gyro 否決), 位置純粹是
        # IMU 外推出來的。這種時候一定要給很大的值, 否則 EKF 會把外推當成量測,
        # 誤差就這樣被鎖進狀態裡 —— 舊版把失敗幀也發成 s=25 (yaw 標準差只有 0.22
        # rad), EKF 照單全收, 這是 yaw 一旦錯掉就回不來的第二個原因。
        if fitness < 0.0:
            pc, yc = 5.0, 4.0         # 標準差 ~2.2 m / ~2 rad: 等於告訴 EKF「別信」
        else:
            s = 1.0 if fitness > 0.6 else 3.0
            pc = 0.002 * s
            yc = 0.002 * s

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        cov[0] = cov[7] = pc          # x, y
        cov[14] = 1e6                 # z (2D, 不可信)
        cov[21] = cov[28] = 1e6       # roll, pitch
        cov[35] = yc                  # yaw
        msg.pose.covariance = cov

        msg.twist.twist.linear.x = float(self.last_vel[0])
        msg.twist.twist.linear.y = float(self.last_vel[1])
        msg.twist.twist.angular.z = float(self.last_vel[2])
        tcov = [0.0] * 36
        tv = 4.0 if fitness < 0.0 else 0.01 * s
        tcov[0] = tcov[7] = tv
        tcov[14] = tcov[21] = tcov[28] = 1e6
        tcov[35] = tv
        msg.twist.covariance = tcov
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = LidarImuOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 建圖模式下, Ctrl-C 也要把辛苦跑出來的地圖存下來
        if node.map_save_path:
            try:
                node.save_map()
            except Exception as e:                               # noqa: BLE001
                node.get_logger().error(f'存檔失敗: {e}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
