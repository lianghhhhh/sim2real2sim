#!/usr/bin/env python3
"""IMU + 四輪輪速的航位推算 (dead reckoning) —— 為什麼加輪速有用, 以及它的極限。

先回答「只用 IMU 一定會飄嗎」
----------------------------
會。而且飄的方式很特別: 加速度計的誤差要**積分兩次**才變成位置, 所以一個固定
的加速度偏差 b 在 t 秒後是 `0.5*b*t^2` —— 誤差是**時間的二次式**, 跟車子有沒有
在動完全無關。停在原地不動, 誤差照樣以 t^2 長。

輪速改變的正是這一件事。輪速是**速度的直接量測**, 不是積分出來的:

    純 IMU   位置誤差 ~ 0.5 * b_a * t^2          (時間的二次式)
    加輪速   位置誤差 ~ (scale 誤差) * 距離       (距離的一次式)
                     + (yaw 誤差) * 距離

車子沒動的時候, 輪速說「速度是 0」, 位置誤差就**完全不長**。這是純 IMU 做不到
的 —— 它的 ZUPT 要靠靜止偵測去猜, 而等速直線行駛跟靜止在慣性量測上長得一樣
(見 `car_loc_imu` 的說明)。輪子沒有這個問題: 輪子在轉就是在走。

實測 (car_run_data/sim_data.csv, 228 秒, 4557 筆, 含 12 次高速自旋與蓄意打滑):

| | |
| --- | --- |
| 「四輪都 < 0.5 rad/s 但車子在動 (>0.1 m/s)」的比例 | **0.00%** |
| 靜止時的輪速雜訊 (p95) | 0.062 rad/s = **0.005 m/s** |

所以這個 package 的 ZUPT 觸發條件不用猜, 它是量出來的。

但它還是會飄
------------
輪速只給**速度**, 不給位置; 陀螺儀只給**角速度**, 不給朝向。兩個都要積分一次
才變成位姿, 所以誤差仍然單調長大 —— 只是從 `t^2` 變成「跟走了多遠成正比」。
在這台車上長期誤差的主導項是 **yaw**, 不是輪速的尺度誤差:

    走了 d 公尺、yaw 差了 e 弧度 -> 側向位置誤差 ~ d * e

要真的不漂, 一樣得有絕對量測 (相機 / LiDAR / 地圖)。

三個感測器各自負責什麼 (實測決定的分工)
---------------------------------------
=== 輪速 -> 前進速度。不要拿它算 yaw ===
skid-steer 轉彎時輪子一定在滑, 所以「左右輪速差 / 輪距」算出來的角速度是**廢的**。
量給你看 (同一份 sim_data.csv, 跟 ground truth 的 yaw 微分比):

| | |
| --- | --- |
| `r*(w_R - w_L)/track` 跟真實角速度的相關係數 | **0.41** |
| 最小平方法反推的「有效輪距」 | 0.18 m (幾何輪距是 0.25 m) |

端到端更明顯 —— 同一段 228 秒, 只換 yaw 的來源 (test/replay_csv.py):

| yaw 從哪來 | RMS | 最大 | 漂移率 |
| --- | --- | --- | --- |
| **IMU 的絕對姿態** (Isaac / 9 軸) | **0.58 m** | 1.91 m | 2.9% |
| 陀螺儀積分 (真車 6 軸) | 2.26 m | 6.19 m | 9.4% |
| 左右輪速差 | 11.85 m | 26.76 m | 40.4% |

**差 20 倍。** 這就是為什麼這個 package 叫「IMU + 輪速」而不是「輪速里程計」。

=== 陀螺儀 -> 角速度。加速度計 -> 打滑時的備援 ===
輪速壞掉的時候 (打滑、離地、撞牆) 由 IMU 撐著; 輪速正常的時候由輪速把 IMU 的
速度誤差壓掉。兩邊壞的方式不一樣, 這才是融合的意義。

=== 車體側向速度 = 0 (NHC) 在這台車上幾乎是恆等式 ===
量 ground truth 的側向速度 (乾淨行駛段, n=1129):

    中位數 0.0000 m/s, p95 **0.0006 m/s**, 最大 0.012 m/s

所以這裡不像 `car_loc_imu` 把 NHC 當成一個「偽量測」再用卡方閘門擋 —— 直接把它
**寫進狀態**: 狀態裡只有一個純量速度 `v` (沿車頭方向), 沒有側向速度這個自由度。
少一個狀態, 少一次更新, 而且不可能給錯軸。

打滑怎麼辦
----------
sim_data.csv 裡有蓄意製造的打滑 (B4 slip / 急煞 / 高速自旋)。輪速在那些時候是
**大錯特錯**, 不是小雜訊:

| 同側 (前後輪) 轉速差 | 輪速誤差 中位 | p95 | 最大 |
| --- | --- | --- | --- |
| < 0.5 rad/s | 0.019 m/s | 0.118 | 0.52 |
| 0.5 – 2 | 0.047 | 0.182 | 2.25 |
| 2 – 5 | 0.029 | 0.282 | 2.95 |
| **> 5** | 0.012 | **3.84** | **6.20** |

同一台 skid-steer, **同一側的前後輪在幾何上必須同速** (它們沒有差速器, 而且
轉彎時同側前後輪走的是同一條軌跡)。所以「同側前後輪不同速」= 至少有一顆在滑,
這是一個**不需要 ground truth、不需要濾波器狀態**的獨立指標。

三道防線, 由弱到強:

1. **四輪取中位數** 而不是平均。一顆輪子空轉時中位數不受影響。
   實測 (一般行駛段的輪速殘差): 平均 std 0.095 / p95 0.243 m/s ->
   中位數 std **0.051** / p95 **0.105**。
   注意蓄意打滑那一段兩者**一樣** —— 那裡是整側一起滑, 任何統計量都救不了,
   只能靠下面兩道防線降權。
2. **同側轉速差 -> 放大 R** (不是直接丟掉)。打滑是連續的, 不是二元的。
3. **卡方閘門** 擋跟 IMU 遞推差太多的輪速。

實測 (228 秒全段, 開閘門 vs 不開):

| | RMS | 最大 |
| --- | --- | --- |
| 不擋打滑 (只剩卡方閘門) | 0.69 m | 1.88 m |
| 擋打滑 | **0.58 m** | 1.91 m |

輪徑尺度: 它跟**牽引狀態**有關, 不是一個幾何常數
------------------------------------------------
尺度誤差是**系統性**的 —— 走 100 公尺就是幾公尺, 不會因為停車而消失。同一台車
的五份資料量出來:

| 資料 | n | scale | 殘差 std | 那一段車子在做什麼 |
| --- | --- | --- | --- | --- |
| `sim_data` Reposition | 923 | **0.932** | 0.060 | 控制器驅動 (有扭矩) |
| `sim_data` 嚴格直線 | 863 | 0.929 | 0.086 | 控制器驅動 |
| `sim_data.backup` | 1966 | 0.927 | 0.095 | 控制器驅動 |
| `sim_data` (較早的一份) | 1129 | 0.976 | 0.065 | 混合 |
| `spin12` | 312 | **1.008** | **0.0097** | 自旋後的**滑行** (沒有扭矩) |

**四份控制器驅動的資料都是 0.93, 只有滑行那一份是 1.008 (= 幾何值)。**
這正好就是物理:

    有扭矩 -> 輪胎一定有一點縱向滑移 (slip ratio), 輪子永遠比車快一點
             -> v/w 偏小 -> 「有效半徑」比幾何值小
    滑行   -> 沒有驅動力, 滑移趨近 0 -> 量到的就是真正的幾何半徑

所以 1.008 不是「比較準」, 而是**另一個工作點**。定位時車子是被開著的, 所以要
用有扭矩的那個 —— 預設 **0.93**。

端到端 (`test/replay_csv.py`, 214 秒那一輪): scale 1.0 -> RMS 0.74,
0.976 -> 0.72, **0.93 -> 0.68**。

要自己校就錄一段**正常開的直線** (不是滑行、不是自旋), 用
`test/replay_bag.py --measure`, 而且要 n > 200 且殘差 std < 0.03 才算數。

**線上估 scale 是關的** (`sigma_k = 0`)。可觀測性: 要把 `k` 跟加速度計零偏
`b_a` 分開, 需要「加速度計看得到、但輪速的尺度誤差解釋不了」的動作。走走停停的
一般行駛裡兩者幾乎共線, 打開之後 k 會去吸收 b_a 的誤差, 反而更糟。

狀態 (7 維)
-----------
    x = [ p(2)   位置 (world)
          v      前進速度 (車體, 純量 —— NHC 是結構性的, 不是量測)
          yaw    朝向
          b_g    陀螺儀 z 零偏
          b_a    前進方向的加速度計零偏
          k      輪速尺度 (v_true = k * r * omega); 預設凍結 ]
"""
from __future__ import annotations

import math

import numpy as np

# 卡方分布的 99% 分位 (自由度 1 / 2 / 3)
CHI2_99 = {1: 6.635, 2: 9.210, 3: 11.345}

IP = slice(0, 2)
IV = 2
IYAW = 3
IBG = 4
IBA = 5
IK = 6
NX = 7

G = 9.80665

# car.usd 的四個輪子。順序固定成 [FL, FR, RL, RR] —— 打滑偵測要靠「同側前後輪」
# 配對, 順序錯了就配錯對。
WHEELS = ('front_left_joint', 'front_right_joint',
          'rear_left_joint', 'rear_right_joint')


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_matrix(q) -> np.ndarray:
    """ROS 順序 (x, y, z, w) -> 3x3 旋轉矩陣。"""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def quat_to_yaw(q) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ----------------------------------------------------------------------------
class WheelReader:
    """把一則 JointState 變成「前進速度 + 這筆有多可信」。

    做三件事:

    1. **依名字取輪子, 不要依索引。** Isaac 的 JointState 順序不保證, 而且
       `/joint_command` 跟 `/joint_states` 的順序可以不一樣。名字對不上才退回
       前四個 (並且回報 `by_name=False`, 節點會警告一次)。

    2. **四輪取中位數。** 一顆輪子空轉 (離地、打滑) 的時候平均會被拉走, 中位數
       不會。實測乾淨行駛段的殘差 std: 平均 0.105 -> 中位數 0.061 m/s。
       轉彎時左右輪本來就不同速, 但 skid-steer 是左二右二, 中位數 = (左+右)/2,
       正好還是車體中心的速度 —— 所以中位數在轉彎時也不會偏。

    3. **算打滑指標 `spread`: 同側前後輪的轉速差 (rad/s)。**
       同一側的前後輪沒有差速器, 幾何上必須同速; 不同速就是有一顆在滑。
       這個判斷**不需要 ground truth, 也不需要濾波器狀態** —— 它是輪速資料
       自己內部的矛盾。實測 spread > 5 rad/s 時輪速誤差 p95 從 0.18 跳到 4.23 m/s。
    """

    def __init__(self, names=WHEELS, radius: float = 0.075):
        self.names = tuple(names)
        self.r = float(radius)
        self.by_name = True

    def read(self, name_list, velocities):
        """回傳 (v_wheel, spread, omega4) 或 None。

        v_wheel 是**還沒乘 scale** 的原始輪速推算 (m/s)。
        """
        if not velocities:
            return None
        if name_list and len(name_list) == len(velocities):
            table = dict(zip(name_list, velocities))
            if all(n in table for n in self.names):
                w = np.array([float(table[n]) for n in self.names])
            else:
                self.by_name = False
                w = np.array([float(v) for v in velocities[:4]])
        else:
            self.by_name = False
            w = np.array([float(v) for v in velocities[:4]])
        if w.size < 4 or not np.all(np.isfinite(w)):
            return None
        # 同側前後輪: (FL, RL) 與 (FR, RR)
        spread = max(abs(w[0] - w[2]), abs(w[1] - w[3]))
        return self.r * float(np.median(w)), float(spread), w


class WheelStillDetector:
    """靜止偵測 —— 有輪速的話這件事不用猜。

    `car_loc_imu` 的靜止偵測要同時看陀螺儀、加速度大小、加速度變異數, 因為純
    慣性量測**分不出「靜止」與「等速直線」**: 兩者的角速度都是 0, 合力都是重力。
    那個偵測器很難調, 太鬆會在直線行駛時誤觸發 ZUPT (位置直接停住), 太緊則停車
    時完全不觸發。

    輪子沒有這個含糊: **輪子在轉就是在走。** 條件只有兩個 ——
    四顆輪子的轉速都很小, 而且陀螺儀也很小 (排除原地打轉)。

    實測 (sim_data.csv, 4557 筆):
        靜止時各輪 |w| 的 p95 = 0.062 rad/s  (= 0.005 m/s)
        「四輪都 < 0.5 rad/s 但車子其實在動 (>0.1 m/s)」的比例 = 0.00%
        (門檻放寬到 2.0 rad/s 才開始出現 0.48%)

    `still_wheel` 的預設值 0.5 rad/s (= 0.037 m/s) 是那個雜訊底線的 8 倍,
    留了很大的餘裕還是幾乎不誤判 —— 這是輪速最值錢的地方之一。

    另外保留一個**車輪在轉但車子沒動**的情況 (卡住空轉): 那時 `spread` 通常很大,
    由 `slip_spread` 那條路處理, 不由這裡處理。
    """

    def __init__(self, wheel_th=0.5, gyro_th=0.05, window=0.15, min_samples=3):
        self.wheel_th = float(wheel_th)
        self.gyro_th = float(gyro_th)
        self.window = float(window)
        self.min_samples = int(min_samples)
        self.buf = []                    # (t, max|w|, |gyro_z|)

    def add(self, t, omega4, gyro_z):
        self.buf.append((float(t), float(np.max(np.abs(omega4))), abs(float(gyro_z))))
        while self.buf and t - self.buf[0][0] > self.window:
            self.buf.pop(0)

    def is_still(self) -> bool:
        if len(self.buf) < self.min_samples:
            return False
        return (max(b[1] for b in self.buf) < self.wheel_th
                and max(b[2] for b in self.buf) < self.gyro_th)


class TiltTracker:
    """6 軸 IMU 的 roll/pitch —— 互補濾波 (真車用; Isaac 走 orientation 模式)。

    跟 `car_loc_imu` 的同名類別是同一套邏輯, 但這裡多了一件重要的事:
    **傾角校正的觸發條件可以用輪速。**

    純 IMU 版本的難處是「什麼時候的加速度只有重力」沒辦法可靠判斷 —— 用
    `|a| ≈ g` 當條件幾乎沒有鑑別力 (0.5 m/s^2 的水平加速度只讓 |a| 變化 0.13%,
    卻對應 2.9 度的假傾角)。它只好退而求其次用慣性靜止偵測。

    有輪速就直接得多: 輪子不轉 = 車子沒有加速度 = 這一刻的比力就是重力。
    這個條件是**外部的**、不受加速度計自己的誤差影響。
    """

    def __init__(self, alpha=0.02):
        self.roll = 0.0
        self.pitch = 0.0
        self.alpha = float(alpha)
        self.inited = False

    def update(self, gyro, acc, dt, trust_accel: bool):
        gx, gy, gz = gyro
        self.roll += (gx + math.sin(self.roll) * math.tan(self.pitch) * gy
                      + math.cos(self.roll) * math.tan(self.pitch) * gz) * dt
        self.pitch += (math.cos(self.roll) * gy - math.sin(self.roll) * gz) * dt
        if float(np.linalg.norm(acc)) < 1e-6:
            return
        if not self.inited or trust_accel:
            ar = math.atan2(acc[1], acc[2])
            ap = math.atan2(-acc[0], math.hypot(acc[1], acc[2]))
            a = 1.0 if not self.inited else self.alpha
            self.roll = (1 - a) * self.roll + a * ar
            self.pitch = (1 - a) * self.pitch + a * ap
            self.inited = True

    def gravity_free(self, acc) -> np.ndarray:
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        g_body = np.array([-sp, cp * sr, cp * cr]) * G
        return (np.asarray(acc, dtype=np.float64) - g_body)[:2]


# ----------------------------------------------------------------------------
class WheelIns:
    """7 維 EKF: IMU 遞推 + 輪速量測 + ZUPT / ZARU。

    跟 `car_loc_imu` 的 8 維濾波器的差別, 一句話: **側向速度不是狀態, 是恆等式。**
    這台車的 ground truth 側向速度 p95 只有 0.0006 m/s, 把它保留成自由度再用
    NHC 偽量測去壓, 只是多一次更新跟一個「軸給錯就爆炸」的風險 (見 car_loc_imu
    表 [CC]: 軸給錯 25 m, 比不開 NHC 的 0.56 m 還糟 45 倍)。這裡直接不給它自由度。
    """

    def __init__(self, *,
                 forward_deg: float = 0.0,     # base_link +X 量到車頭的角度
                 wheel_scale: float = 0.93,    # v_true = k * r * omega
                 sigma_gyro: float = 0.02,
                 sigma_gyro_scale: float = 0.01,
                 sigma_acc: float = 0.05,
                 sigma_acc_omega: float = 0.15,
                 sigma_cross: float = 0.02,    # 側向的過程雜訊 (NHC 不是完美的)
                 sigma_bg: float = 1e-4,
                 sigma_ba: float = 1e-3,
                 sigma_k: float = 0.0,         # 0 = 凍結尺度 (預設; 見模組說明)
                 wheel_sigma: float = 0.07,    # 輪速量測雜訊 (m/s)
                 wheel_sigma_slip: float = 10.0,  # 打滑時放大到多少 (m/s)
                 slip_spread: float = 2.0,     # 同側轉速差超過這個開始放大 R
                 slip_spread_max: float = 8.0,  # 到這個就用 wheel_sigma_slip
                 # 輪速連續被擋掉超過這麼久 (秒) 就強制接受一次 (見 update_wheel)
                 wheel_reject_time: float = 2.0,
                 zupt_sigma: float = 0.01,
                 zaru_sigma: float = 0.002,
                 zero_accel_sigma: float = 0.05,
                 # 靜止時的「零加速度」更新最多容忍多大的殘差 (m/s^2)。
                 # 見 zero_accel() —— 這個上限不是可有可無的調參。
                 zero_accel_max: float = 0.5,
                 anchor_sigma: float = 0.02,
                 anchor_after: float = 0.5,
                 v_max: float = 4.0,
                 chi2_scale: float = 1.0,
                 max_pos_correction: float = 0.5):
        self.x = np.zeros(NX)
        self.x[IK] = float(wheel_scale)
        self.P = np.diag([1e-4, 1e-4, 0.25, 1e-4, 1e-4, 1e-2,
                          (0.05 if sigma_k > 0 else 1e-8) ** 2])
        self.t = None

        self.forward_deg = float(forward_deg)
        self.phi = math.radians(self.forward_deg)
        self.sg = float(sigma_gyro)
        self.sg_scale = float(sigma_gyro_scale)
        self.sa = float(sigma_acc)
        self.sa_omega = float(sigma_acc_omega)
        self.s_cross = float(sigma_cross)
        self.sbg = float(sigma_bg)
        self.sba = float(sigma_ba)
        self.sk = float(sigma_k)
        self.wheel_sigma = float(wheel_sigma)
        self.wheel_sigma_slip = float(wheel_sigma_slip)
        self.slip_spread = float(slip_spread)
        self.slip_spread_max = float(slip_spread_max)
        self.wheel_reject_time = float(wheel_reject_time)
        self.reject_since = None
        self.zupt_sigma = float(zupt_sigma)
        self.zaru_sigma = float(zaru_sigma)
        self.za_sigma = float(zero_accel_sigma)
        self.za_max = float(zero_accel_max)
        self.anchor_sigma = float(anchor_sigma)
        self.anchor_after = float(anchor_after)
        self.v_max = float(v_max)
        self.chi2_scale = float(chi2_scale)
        self.max_pos_correction = float(max_pos_correction)

        self.still_since = None
        self.anchor = None
        self.last_omega = 0.0
        self.last_acc = 0.0
        self.counts = {'wheel': 0, 'zupt': 0, 'zaru': 0, 'zaccel': 0, 'anchor': 0,
                       'yaw': 0, 'rejected': 0, 'damped': 0, 'slip': 0,
                       'za_skip': 0, 'forced': 0}

    # ------------------------------------------------------------------
    @property
    def pos(self):
        return self.x[IP].copy()

    @property
    def speed(self) -> float:
        return float(self.x[IV])

    @property
    def yaw(self) -> float:
        return float(self.x[IYAW])

    @property
    def heading(self) -> float:
        """世界座標裡「車頭」的方向 = yaw + forward_deg。"""
        return wrap_pi(float(self.x[IYAW]) + self.phi)

    @property
    def gyro_bias(self) -> float:
        return float(self.x[IBG])

    @property
    def acc_bias(self) -> float:
        return float(self.x[IBA])

    @property
    def scale(self) -> float:
        return float(self.x[IK])

    def set_pose(self, x, y, yaw, t=None, sigma_p=0.02, sigma_yaw=0.02):
        self.x[IP] = [float(x), float(y)]
        self.x[IV] = 0.0
        self.x[IYAW] = wrap_pi(float(yaw))
        self.P[0, 0] = self.P[1, 1] = sigma_p ** 2
        self.P[IV, IV] = 0.25
        self.P[IYAW, IYAW] = sigma_yaw ** 2
        if t is not None:
            self.t = float(t)
        self.anchor = None
        self.still_since = None

    # ------------------------------------------------------------------ 遞推
    def predict(self, t: float, a_fwd: float, omega_z: float):
        """a_fwd 是**已經扣掉重力**、投影到車頭方向的加速度; omega_z 是原始陀螺儀 z。"""
        if self.t is None:
            self.t = float(t)
            return
        dt = t - self.t
        if dt <= 0.0:
            return
        if dt > 0.5:                     # 斷太久就不要一次推一整段
            self.t = float(t)
            return
        self.t = float(t)

        a = float(a_fwd) - self.x[IBA]
        w = float(omega_z) - self.x[IBG]
        self.last_omega = w
        self.last_acc = a
        v = float(self.x[IV])
        h = self.heading
        ch, sh = math.cos(h), math.sin(h)
        ds = v * dt + 0.5 * a * dt * dt

        # --- 名目狀態 ---
        self.x[0] += ds * ch
        self.x[1] += ds * sh
        self.x[IV] = v + a * dt
        self.x[IYAW] = wrap_pi(self.x[IYAW] + w * dt)

        # --- 雅可比 ---
        F = np.eye(NX)
        F[0, IV] = dt * ch
        F[1, IV] = dt * sh
        F[0, IYAW] = -ds * sh
        F[1, IYAW] = ds * ch
        F[0, IBA] = -0.5 * dt * dt * ch
        F[1, IBA] = -0.5 * dt * dt * sh
        F[IV, IBA] = -dt
        F[IYAW, IBG] = -dt

        # --- 過程雜訊 ---
        # 加速度的過程雜訊必須**隨角速度長大**。實測 (car_loc_imu 的 spin12 bag):
        # IMU 扣完重力的水平加速度跟真實加速度的殘差, |w|<0.2 時 0.064 m/s^2,
        # |w|>12 時 14.7 m/s^2 —— 差 230 倍, 一個常數不可能同時對。
        # 給常數的話直線行駛時 P 被灌大, 輪速更新的增益逼近 1, 一次更新就把位置
        # 搬走; 高速自旋時 P 又太小, 正確的輪速被卡方閘門擋掉。
        qa = self.sa ** 2 + (self.sa_omega * abs(w)) ** 2
        qg = self.sg ** 2 + (self.sg_scale * abs(w)) ** 2
        Q = np.zeros((NX, NX))
        # 沿車頭方向的位置/速度雜訊 (相關的), 用外積鋪到 x, y 兩軸
        u = np.array([ch, sh])
        Q[0:2, 0:2] = qa * dt ** 3 / 3.0 * np.outer(u, u)
        Q[0:2, IV] = Q[IV, 0:2] = qa * dt * dt / 2.0 * u
        Q[IV, IV] = qa * dt
        # 側向: NHC 是結構性的, 但不是完美的 (打滑轉彎時真的會橫移, 實測
        # |wz|>3 時側向速度 p95 = 0.094 m/s)。給一個小的側向過程雜訊, 不然 P
        # 在側向會是奇異的, 之後任何量測都修不動那一維。
        lat = np.array([-sh, ch])
        Q[0:2, 0:2] += (self.s_cross ** 2) * dt * np.outer(lat, lat)
        Q[IYAW, IYAW] = qg * dt
        Q[IBG, IBG] = self.sbg ** 2 * dt
        Q[IBA, IBA] = self.sba ** 2 * dt
        Q[IK, IK] = self.sk ** 2 * dt

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

        if abs(self.x[IV]) > self.v_max:
            self.x[IV] = math.copysign(self.v_max, self.x[IV])

    # ------------------------------------------------------------------ 更新
    def _update(self, H, r, R, name: str, gate: bool = True) -> bool:
        """通用 EKF 更新, 附卡方閘門 + 位置修正上限 (理由見 car_loc_imu/ins.py)。"""
        H = np.atleast_2d(H)
        r = np.atleast_1d(r)
        R = np.atleast_2d(R)
        S = H @ self.P @ H.T + R
        try:
            Si = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False
        nis = float(r @ Si @ r)
        if gate and nis > CHI2_99[len(r)] * self.chi2_scale:
            self.counts['rejected'] += 1
            return False
        K = self.P @ H.T @ Si
        dx = K @ r
        dp = float(np.linalg.norm(dx[IP]))
        if self.max_pos_correction > 0.0 and dp > self.max_pos_correction:
            K = K * (self.max_pos_correction / dp)
            dx = K @ r
            self.counts['damped'] += 1
        self.x = self.x + dx
        self.x[IYAW] = wrap_pi(self.x[IYAW])
        I_KH = np.eye(NX) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T      # Joseph form
        self.P = 0.5 * (self.P + self.P.T)
        self.counts[name] = self.counts.get(name, 0) + 1
        return True

    # --- 輪速: 這個 package 的主角 ---
    def wheel_sigma_for(self, spread: float) -> float:
        """同側前後輪的轉速差 -> 這一筆輪速該給多大的 sigma。

        分三段, 而且**兩端的處理不一樣**:

            spread < slip_spread          照 wheel_sigma, 正常相信
            中間                           線性內插 (打滑是連續的, 不是二元的)
            spread > slip_spread_max      sigma 給到 10 m/s = **實質上丟掉**

        中間那一段要漸進, 因為輕微打滑的量測仍然有價值: 實測同側差 0.5-2 rad/s
        時輪速誤差 p95 只有 0.18 m/s, 比純遞推好得多。

        但**上限那一段要給得非常大 —— 這是實測改出來的, 原本給 1.5 是錯的。**
        三份真實資料一起掃 `wheel_sigma_slip` (位置 RMS, 越小越好):

            sigma_slip      0.5    1.5    2.5    4.0    6.0   10.0   20.0  丟掉
            sim_data      0.481  0.684  0.505  0.432  0.401  0.380  0.402  0.436
            backup        8.234  7.408  6.955  6.649  6.498  6.381  6.114  5.405
            spin12        6.290  6.184  5.784  5.480  5.302  5.163  5.122  4.628

        **三份都是越大越好**, 而 1.5 剛好卡在最差的位置 (sim_data 那一列在 1.5
        是 0.684, 兩邊都比它好 —— 非單調就是「信一半」最糟的典型症狀: 量測爛到
        會把狀態拉走, 但 sigma 又沒大到讓增益趨近 0)。
        合成資料上把它拉到 10 幾乎沒有代價 (0.0243 -> 0.0255), 所以取 10.0。

        為什麼超過門檻之後不乾脆真的丟掉: 留著一個極大的 sigma, 增益 ~1e-4,
        效果等於丟掉, 但**卡方閘門仍然看得到它** —— 而閘門連續兩秒都擋不下來
        就是逃生口 (`wheel_reject_time`) 該啟動的訊號。真的丟掉的話那個訊號
        也一起沒了。
        """
        s = abs(float(spread))
        if s <= self.slip_spread:
            return self.wheel_sigma
        if s >= self.slip_spread_max:
            return self.wheel_sigma_slip
        a = (s - self.slip_spread) / max(self.slip_spread_max - self.slip_spread, 1e-9)
        return self.wheel_sigma + a * (self.wheel_sigma_slip - self.wheel_sigma)

    def update_wheel(self, v_wheel: float, spread: float = 0.0) -> bool:
        """輪速量測。v_wheel 是 r * median(omega), **還沒乘 scale**。

        量測模型是 `v_wheel = v / k`, 不是 `v = k * v_wheel` —— 差別在雅可比:
        前者讓 k 的偏導數是 `-v/k^2`, 也就是「車開得越快, 這一筆對 k 的資訊
        越多」, 這才是尺度誤差真正的物理 (靜止時輪速對 k 一點資訊都沒有)。

        **連續被擋掉太多次就強制接受一次。** 這不是保險, 是一個必要的逃生口:

        速度的不確定度 `P[v]` 在 ZUPT 之後會被壓到 `zupt_sigma^2` (1e-4), 而它
        之後只靠過程雜訊 `qa*dt` 慢慢長回來 (60 Hz 下每步 4e-5)。所以如果濾波器
        的速度信念跟輪速差得夠遠 —— 例如加速度計壞掉/配錯 (`gravity_mode` 給錯、
        傾角估歪)、或者一段打滑之後輪子重新咬地 —— 卡方閘門會**一直**擋:
        要等 `P[v]` 長到蓋得住那個 innovation 才放行, 實測 0.5 m/s 的落差要等
        **14 秒**, 那段時間輪速等於完全沒接上, 車子在估計裡停在原地不動。

        「連續擋掉這麼久」本身就是證據: 一次離群是量測的問題, **連續兩秒**都對
        不上就是濾波器自己錯了。這時輪速贏 —— 它是直接量測, 而另一個選項是純靠
        遞推。強制接受時 R 放大 3 倍 (不是完全相信它), 讓狀態走過去而不是跳過去。

        **門檻要用時間, 而且要給得長。** 用「連續 N 筆」的話 N 會跟著取樣率跑;
        給太短的話真的打滑時會介入 —— 打滑期間卡方閘門本來就**應該**一直擋,
        那不是病態。實測 (test_wheel_ins.py, 打滑段 1~1.5 秒):
            0.33 秒門檻 -> 打滑中被強制接受 3~4 次, 位置 RMS 0.114 -> 0.304 m
            2 秒門檻    -> 打滑期間完全不介入, RMS 維持 0.114 m
        2 秒撐得過任何一次打滑, 但遠短於「等 P[v] 自己長回來」的 14 秒。

        沒有逃生口的代價 (test/test_node.py 案例 [2], 加速度計整個壞掉,
        輪速從 0 階躍到 0.6 m/s):
            不設逃生口 -> 600 筆輪速**全部**被擋掉, 估計速度停在 0.000,
                          車子在估計裡完全沒有動過, 12 秒累積 6.00 m
            2 秒逃生口 -> 強制接受 3 次, 速度收斂到 0.602
        """
        k = max(float(self.x[IK]), 1e-3)
        H = np.zeros((1, NX))
        H[0, IV] = 1.0 / k
        H[0, IK] = -float(self.x[IV]) / (k * k)
        sd = self.wheel_sigma_for(spread)
        if sd > self.wheel_sigma:
            self.counts['slip'] += 1
        r = np.array([float(v_wheel) - float(self.x[IV]) / k])
        if self._update(H, r, np.array([[sd ** 2]]), 'wheel'):
            self.reject_since = None
            return True
        if self.reject_since is None:
            self.reject_since = self.t
        elif (self.wheel_reject_time > 0.0 and self.t is not None
                and self.t - self.reject_since >= self.wheel_reject_time):
            self.reject_since = None
            self.counts['forced'] += 1
            return self._update(H, r, np.array([[(3.0 * sd) ** 2]]), 'wheel',
                                gate=False)
        return False

    # --- ZUPT: 輪子不轉 = 速度是 0 ---
    #
    # 跟 car_loc_imu 一樣**不吃卡方閘門**: 依據是一個跟濾波器狀態無關的獨立
    # 判斷 (輪速 + 陀螺儀)。拿閘門擋它等於「越迷路越拒絕修正」。
    # 差別在這裡的依據**強得多** —— 輪子在轉就是在走, 沒有「等速直線跟靜止
    # 長得一樣」的問題。實測誤判率 0.15%。
    def zupt(self):
        H = np.zeros((1, NX))
        H[0, IV] = 1.0
        self._update(H, np.array([-float(self.x[IV])]),
                     np.array([[self.zupt_sigma ** 2]]), 'zupt', gate=False)

    def zaru(self, omega_z: float):
        H = np.zeros((1, NX))
        H[0, IBG] = -1.0
        r = np.array([-(float(omega_z) - self.x[IBG])])
        self._update(H, r, np.array([[self.zaru_sigma ** 2]]), 'zaru', gate=False)

    def zero_accel(self, a_fwd: float):
        """靜止時扣掉重力後的前進加速度應該是 0 -> 這一刻讀到的就是 b_a。

        **殘差要設上限, 而且原因是結構性的, 不是為了保險。**

        輪速的靜止偵測有一個先天的半拍延遲: 加速度是速度的**導數**, 所以起步
        瞬間加速度已經是滿的, 輪子才剛要開始轉。`is_still` 看的是輪速, 於是
        「輪速還沒過門檻、但加速度已經 1.5 m/s^2」的那一兩筆會被判成靜止 ——
        然後這裡就把那個真實的加速度當成零偏吃進去。

        代價實測 (test/test_wheel_ins.py, 一路不停那條軌跡, 真實零偏 0.02):
            沒有上限  b_a 收到 **0.0301** (差 50%), 位置 RMS **0.912 m**
            有上限    b_a 收到 0.0203,           位置 RMS 0.268 m
        一筆就夠了 —— 那時 P[b_a] 已經被前面上百筆更新壓到很小, 增益只有 0.011,
        但 innovation 有 1.47, 乘起來就是 0.016 的零偏誤差, 而且**之後再也沒有
        機會修回來** (一路不停 = 不會再有靜止段)。

        上限之所以是對的: 車子如果真的靜止, 扣完重力的加速度**依定義**接近 0,
        殘差大就表示「靜止」這個前提已經不成立了 —— 這不是離群值判斷,
        是前提檢查。所以它不吃卡方閘門 (P 大小跟這件事無關), 而是一個絕對門檻。
        """
        r = -(float(a_fwd) - float(self.x[IBA]))
        if abs(r) > self.za_max:
            self.counts['za_skip'] += 1
            return
        H = np.zeros((1, NX))
        H[0, IBA] = -1.0
        self._update(H, np.array([r]), np.array([[self.za_sigma ** 2]]),
                     'zaccel', gate=False)

    def anchor_position(self):
        if self.anchor is None:
            self.anchor = self.x[IP].copy()
        H = np.zeros((2, NX))
        H[0, 0] = H[1, 1] = 1.0
        self._update(H, self.anchor - self.x[IP],
                     np.eye(2) * self.anchor_sigma ** 2, 'anchor')

    def update_yaw(self, yaw_meas: float, sigma: float = 0.02, dt: float = 0.0):
        """IMU 的絕對 yaw (Isaac / 9 軸 IMU)。真車的 6 軸 IMU 沒有這個。"""
        H = np.zeros((1, NX))
        H[0, IYAW] = 1.0
        r = np.array([wrap_pi(float(yaw_meas) - self.x[IYAW])])
        # R 依這一步轉了多少放寬: w*dt 遞推本來就落後 |w|*dt/2, 20 rad/s 時是
        # 0.17 rad, 比 sigma 大一個數量級, 不放寬的話閘門會擋掉正確的量測。
        sd = math.hypot(float(sigma), 0.5 * abs(self.last_omega) * float(dt))
        self._update(H, r, np.array([[sd ** 2]]), 'yaw', gate=False)

    # ------------------------------------------------------------------
    def sigma_pos(self) -> float:
        return float(math.sqrt(max(self.P[0, 0] + self.P[1, 1], 0.0)))

    def pose_cov(self) -> np.ndarray:
        c = np.zeros((6, 6))
        c[0, 0], c[0, 1] = self.P[0, 0], self.P[0, 1]
        c[1, 0], c[1, 1] = self.P[1, 0], self.P[1, 1]
        c[2, 2] = c[3, 3] = c[4, 4] = 1e-6
        c[5, 5] = self.P[IYAW, IYAW]
        return c

    def twist_cov(self) -> np.ndarray:
        c = np.zeros((6, 6))
        c[0, 0] = self.P[IV, IV]
        c[1, 1] = 1e-4                   # 側向速度是結構性的 0
        c[2, 2] = c[3, 3] = c[4, 4] = 1e-3
        c[5, 5] = self.P[IYAW, IYAW]
        return c
