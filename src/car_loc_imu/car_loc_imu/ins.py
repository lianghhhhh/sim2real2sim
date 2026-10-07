#!/usr/bin/env python3
"""純慣性導航 (INS) + EKF —— 只有 IMU 的時候能做到的極限, 以及怎麼壓住漂移。

先講一件不能迴避的事
--------------------
**只用 IMU 一定會漂。** 加速度計的誤差要積分兩次才變成位置, 所以一個固定的
加速度偏差 b 在 t 秒後是 0.5*b*t^2 的位置誤差 —— 0.01 m/s^2 (很小的偏差) 在
60 秒後就是 18 公尺。這不是參數沒調好, 是數學。這個模組能做的是把漂移的**成長
速度**壓下來, 不是消除它。要真的不漂就需要一個絕對量測 (地圖、相機、GPS),
那就是方法一與方法二在做的事。

姿態誤差比零偏更致命
--------------------
IMU 量到的是**比力** (specific force), 靜止水平時 az ≈ +9.81。車身一旦傾斜,
重力就會漏進水平軸: 傾斜 1 度 = 0.17 m/s^2 的假加速度 = 10 秒後 8.5 公尺。
所以「重力扣得準不準」比「零偏估得準不準」重要一個數量級。三種扣法:

* `orientation` (預設): 直接用 IMU 訊息裡的 orientation 把加速度轉到世界座標
  再扣掉 [0,0,g]。Isaac 的 IsaacReadIMU 給的是模擬器的精確姿態, 所以在模擬裡
  這一項幾乎沒有誤差 —— 這也是為什麼模擬裡的純 IMU 表現會比真車好很多。
* `complementary`: 6 軸 IMU 沒有絕對姿態, 用陀螺儀積分 roll/pitch, 再用加速度
  的方向 (只在接近靜止時可信) 慢慢拉回來。真車走這條。
* `none`: 假設車身永遠水平, 直接用 ax, ay。平地短時間可以, 長時間不行。

四道抗漂移的防線 (都只用 IMU 自己的資料)
----------------------------------------
1. **開機靜止校正。** 車子還沒動的時候把陀螺儀零偏、加速度計零偏、重力方向
   一次量掉。這是最便宜也最有效的一步。
2. **ZUPT (零速更新)。** 偵測到靜止就宣告「速度是 0」。這會把速度誤差歸零,
   位置因此停止漂移 —— 停車越頻繁, 純 IMU 越撐得住。
   **陷阱: ZUPT 不能只看陀螺儀。** 等速直線行駛的車子角速度是 0、加速度也
   接近 0, 純慣性量測**分不出**「靜止」與「等速直線」。這裡的靜止判定同時看
   陀螺儀、加速度大小、以及一段時間內加速度的**變異數** —— 真的靜止時感測器
   雜訊還在, 但訊號的結構跟行駛完全不同。
3. **ZARU (零角速更新)。** 靜止時陀螺儀讀到的東西全部是零偏, 直接拿來修 b_g。
   這是唯一能持續修正 yaw 漂移的機制 (在 `orientation` 模式下 yaw 另有來源)。
4. **NHC (非完整約束)。** 車子不會橫著走, 所以**車體座標的側向速度應該是 0**。
   這是輪式載具特有的免費量測, 而且行駛中一直有效 (ZUPT 只在停車時有效)。
   它會持續修正「速度方向」與 yaw 的耦合誤差。
   注意 skid-steer 原地打滑轉彎時這個假設會被違反, 所以 R 要給鬆一點,
   而且角速度大的時候要關掉。

   **哪一軸是「側向」不能用猜的。** REP-103 說 base_link 的 +X 是車頭, 但這台
   車的 USD 不是 —— `car.usd` 的 base_link 是 **+X 朝左、車頭 -Y**
   (`car_teleop/cmd_vel_bridge.py` 裡就寫著這件事)。軸弄反的話 NHC 會把
   **前進速度**當成側向速度歸零, 車子在估計裡幾乎不會前進, 而真正的側向
   (這裡是 body x) 反而完全沒有約束, 誤差自由累積 —— 比不開 NHC 還糟很多。
   所以車頭方向由 `forward_deg` 給 (base_link +X 量到車頭的角度, 這台車是 -90)。

狀態 (8 維)
-----------
    x = [ p(2)    位置 (world)
          v(2)    速度 (world)
          yaw     朝向
          b_g     陀螺儀 z 零偏     (動態部分, 見下)
          b_a(2)  加速度計 xy 零偏 (動態部分, body) ]

零偏模型
--------
    b(t) = b0 + b_dyn(t)

    b0     開機常數零偏。由開機靜止校正量出來 (`set_bias0`), **放在狀態外面**,
           之後不再變。
    b_dyn  會慢慢變的那一部分, 就是狀態裡的 b_g / b_a。兩種模型 (`bias_model`):

      gm (預設)  一階 Gauss-Markov:  db_dyn/dt = -b_dyn/tau + n_b
                 離散化 (精確解):     b[k+1] = phi*b[k] + w,  phi = exp(-dt/tau)
                                      Var(w) = sigma_gm^2 * (1 - phi^2)
                 不確定度會飽和在 sigma_gm; 沒有量測的時候估計值往 0 衰減。
      rw         隨機遊走:            db_dyn/dt = n_b,  Var(w) = sigma_b^2 * dt
                 GM 在 tau -> 無限大的極限 (sigma_b^2 = 2*sigma_gm^2/tau)。

**b0 一定要拆出來。** 衰減項如果作用在整個零偏上, 會把已經量到的常數零偏也
往 0 拉 —— 60 秒沒有量測、tau = 300 s 的話就拉掉 18%。所以 GM 只描述扣掉 b0
之後的殘差; 開機校正沒成功 (b0 不知道) 的時候不該用 GM, 節點會自動退回 rw。

tau / sigma_gm 不是用猜的: 靜止錄一段資料, `fit_noise.py` 用 Allan variance
擬合出來。
"""
from __future__ import annotations

import math

import numpy as np

# 卡方分布的 99% 分位 (自由度 1 / 2 / 3)
CHI2_99 = {1: 6.635, 2: 9.210, 3: 11.345}

IP = slice(0, 2)
IV = slice(2, 4)
IYAW = 4
IBG = 5
IBA = slice(6, 8)
NX = 8

J90 = np.array([[0.0, -1.0], [1.0, 0.0]])       # d/dyaw 的旋轉導數
G = 9.80665


def rot2(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


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


class StillDetector:
    """靜止偵測 —— ZUPT / ZARU 的觸發條件, 也是整個抗漂移的核心。

    三個條件同時成立才算靜止:
        a) |gyro| 小
        b) |accel| 跟重力大小差不多 (車子在動的時候合力會偏離 g)
        c) 一段視窗內 accel 的**變異數**小

    (c) 是關鍵。等速直線行駛時 (a) 與 (b) 都會成立 —— 純慣性量測分不出靜止與
    等速直線。但真的靜止時只剩感測器雜訊, 變異數很小而且穩定; 行駛時輪子的
    震動、路面的起伏會讓變異數高一個數量級。這個差別在真車上非常明顯,
    在模擬裡則要靠 Isaac 的物理雜訊 (所以模擬裡門檻要調緊一點)。
    """

    def __init__(self, gyro_th=0.03, acc_th=0.25, var_th=0.05, window=0.30,
                 min_samples=8):
        self.gyro_th = float(gyro_th)
        self.acc_th = float(acc_th)
        self.var_th = float(var_th)
        self.window = float(window)
        self.min_samples = int(min_samples)
        self.buf = []               # (t, |gyro|, |acc|, acc_xyz)

    def add(self, t, gyro, acc):
        self.buf.append((t, float(np.linalg.norm(gyro)),
                         float(np.linalg.norm(acc)), np.asarray(acc, dtype=np.float64)))
        while self.buf and t - self.buf[0][0] > self.window:
            self.buf.pop(0)

    @property
    def ready(self) -> bool:
        return len(self.buf) >= self.min_samples

    def is_still(self) -> bool:
        if not self.ready:
            return False
        gn = np.array([b[1] for b in self.buf])
        an = np.array([b[2] for b in self.buf])
        acc = np.stack([b[3] for b in self.buf])
        if gn.max() > self.gyro_th:
            return False
        if abs(an.mean() - G) > self.acc_th:
            return False
        return float(acc.var(axis=0).sum()) < self.var_th ** 2

    def mean_gyro_acc(self):
        acc = np.stack([b[3] for b in self.buf])
        return acc.mean(axis=0)


class StillConfirm:
    """靜止的第二階段確認 —— StillDetector 說靜止之後, 再用濾波器自己知道的事
    把三種「讀數很安靜但車子其實在動」的情況擋掉。

    StillDetector 只看原始讀數的大小與變異數。路面平、感測器白雜訊又蓋過車體
    震動的時候 (Isaac 就是這樣: 等速行駛的加速度抖動 0.008 m/s^2, 注入的白雜訊
    0.031), 下面三種情況它全部會說靜止。實測 (Isaac bag, 行駛段):

    1. **平順的直線加減速** (0.2~0.6 m/s^2)。零加速度更新把真實的加速度當成
       零偏: 車頭軸的加速度零偏誤差 0.089 m/s^2, 是注入量的兩倍多。
       -> 視窗平均的水平加速度扣掉零偏要夠小 (`acc_mean`)。之後 0.007。
    2. **高速自旋後的慢轉** (約 0.01 rad/s, 低於 still_gyro)。ZARU 把真實的
       轉速當成零偏: 陀螺零偏誤差 0.57 mrad/s。
       -> 視窗平均的角速度扣掉零偏要夠小 (`gyro_mean`)。之後 0.15。
    3. **等速直線行駛。** ZUPT 把速度歸零, 估計的位置停在原地: 一次 3 秒的
       誤判就是 4 公尺。這一種的 IMU 讀數跟靜止**完全相同**, 只能靠「濾波器
       記得自己剛剛加速過」來分辨:
       -> 剛離開靜止不久 (`trust` 秒內), 而濾波器估的速度還明顯不是 0,
          就不承認靜止 (`speed_gate`)。三道都開之後, 行駛中被判成靜止的比例
          11.0% -> 0.3% (十二趟平均)。

    速度那一道的門檻是 `speed_gate + speed_frac * (離開靜止後速度變化量的總和)`。
    後面那一項是誤差預算: 加減速越劇烈, 積分出來的速度越不準 (衝刺到 2.5 m/s
    再煞停, 實測估計速度會殘留 0.4 m/s), 那時就不該拿它來否決靜止。超過
    `trust` 秒也不再否決 —— 寧可吃一次錯的 ZUPT, 也不要因為速度估錯而永遠
    不承認真的停車。

    **轉得快過之後速度那一道不算數** (`speed_max_omega`)。高速自旋時加速度的
    雜訊跟著角速度長大 (|w| > 12 rad/s 時 14.7 m/s^2), 自旋完估計速度會殘留
    0.5 m/s 以上的誤差 —— 那正是最需要 ZUPT 來救的時刻。實測一趟: 自旋後車子
    真的停了, 速度那一道卻否決了 6 秒, 估計位置滑出去 3.5 公尺。

    三道檢查都拿「目前估計的零偏 / 速度」當基準, 所以只能在開機靜止校正完成
    之後用。設成 0 就是關掉那一道。
    """

    def __init__(self, window=0.30, acc_mean=0.10, gyro_mean=0.002,
                 speed_gate=0.20, speed_frac=0.15, trust=15.0, speed_max_omega=3.0):
        self.window = float(window)
        self.acc_mean = float(acc_mean)
        self.gyro_mean = float(gyro_mean)
        self.speed_gate = float(speed_gate)
        self.speed_frac = float(speed_frac)
        self.trust = float(trust)
        self.speed_max_omega = float(speed_max_omega)
        self.buf = []               # (t, acc_xy, omega_z)
        self.reset()

    def reset(self):
        self.buf.clear()
        self.was_still = False
        self.t_leave = None         # 最後一次確認靜止的時間
        self.dv_sum = 0.0           # 離開靜止之後 |a|*dt 的總和 (m/s)
        self.w_max = 0.0            # 離開靜止之後轉得最快有多快 (rad/s)
        self.t_prev = None
        self.rejected = {'acc': 0, 'gyro': 0, 'speed': 0}

    def update(self, t, dt, raw_still: bool, acc_xy, omega_z: float, ins) -> bool:
        """每一筆 IMU 呼叫一次 (在 ins.predict 之前)。回傳確認後的靜止。"""
        acc_xy = np.asarray(acc_xy, dtype=np.float64)[:2].copy()
        # 上一筆確認了靜止, 而且 ZUPT 真的把速度壓下來了 -> 這才算「從靜止重新
        # 出發」, 速度那一道重新計數。只確認靜止但速度還沒歸零的話不能重來
        # (ZUPT 的修正量超過 max_pos_correction 時整個增益會被縮小, 速度一次
        # 修不完): 否則下一筆就會拿那個還沒歸零的速度去否決靜止 —— 實測自旋後
        # 停車, 一筆 ZUPT 之後被否決了 4 秒, 位置滑出去 1.7 公尺。
        if self.was_still and ins.speed < max(self.speed_gate, 1e-6):
            self.t_leave = self.t_prev
            self.dv_sum = 0.0
            self.w_max = 0.0
        self.t_prev = t
        self.buf.append((t, acc_xy, float(omega_z)))
        while self.buf and t - self.buf[0][0] > self.window:
            self.buf.pop(0)

        still = bool(raw_still)
        if still and self.acc_mean > 0.0:
            mean_a = np.mean([b[1] for b in self.buf], axis=0)
            if not ins.accel_is_quiet(mean_a, self.acc_mean):
                still = False
                self.rejected['acc'] += 1
        if still and self.gyro_mean > 0.0:
            mean_w = float(np.mean([b[2] for b in self.buf]))
            if not ins.gyro_is_quiet(mean_w, self.gyro_mean):
                still = False
                self.rejected['gyro'] += 1
        # 速度那一道只在「要從行駛進入靜止」的那一刻問 —— 已經在靜止裡就不問,
        # ZUPT 之後速度本來就是 0。
        if (still and self.speed_gate > 0.0 and not self.was_still
                and self.t_leave is not None and t - self.t_leave < self.trust
                and self.w_max < self.speed_max_omega
                and ins.speed > self.speed_gate + self.speed_frac * self.dv_sum):
            still = False
            self.rejected['speed'] += 1

        if not still:
            self.w_max = max(self.w_max, abs(float(omega_z) - ins.gyro_bias))
            if dt:
                self.dv_sum += float(np.linalg.norm(acc_xy - ins.acc_bias)) * dt
        self.was_still = still
        return still


class SpinDetector:
    """原地自旋偵測 —— 補 ZUPT 的一個大洞。

    ZUPT 要求「陀螺儀也要小」, 所以 skid-steer **原地打轉**的時候完全不會觸發:
    平移速度明明是 0, 但角速度有 1~20 rad/s。而那正好是最需要它的時刻 ——
    高速自旋會把速度誤差灌到好幾 m/s, 自旋一結束那個誤差就直接積分成位置。
    實測 (spin12 bag): 13 次高速自旋灌進 +30.0 m 的誤差, 停車只救回 0.6 m。

    怎麼分辨「原地打轉」跟「繞圈行駛」—— 兩者角速度都大:

        繞圈行駛: 側向比力 = v * w  (向心加速度), v=1 m/s 且 w=2 rad/s 就是
                  2 m/s^2, 很明顯。
        原地打轉: v = 0, 所以水平比力**接近 0** (前提是 IMU 就在旋轉中心上;
                  偏了 r 的話會量到 w^2*r, 用 replay_bag.py --measure 檢查,
                  car.usd 量到 0.6 mm, 可以忽略)。

    實測 (spin12 bag, |wz|>0.5 且視窗內 |水平比力|<0.2):
        涵蓋 2133 筆, 其中 85.9% 車子真的沒在平移;
        誤判的那些實際車速也只有 0.09~0.11 m/s。
        對照組「只看 |wz|>0.5 不看加速度」只有 48.7% —— 加速度那一項是關鍵。

    因為誤判時車子可能真的在爬行 (~0.1 m/s), 這個更新的 sigma 要比一般 ZUPT
    鬆得多 (`spin_zupt_sigma`), 而且**只修速度**: 不錨定位置 (車子可能真的在
    慢慢移動), 也不拿它去估加速度計零偏 (自旋時的加速度訊號不乾淨)。
    """

    def __init__(self, omega_th=0.5, omega_max=6.0, acc_th=0.2, window=0.30,
                 min_samples=8):
        self.omega_th = float(omega_th)
        # 上限: 超過這個角速度就不做。60 Hz 下 6 rad/s 已經是一步轉 5.7 度,
        # 再快的話遞推、離散化、加速度雜訊全部一起崩 (實測 |wz|>12 時加速度
        # 殘差 14.7 m/s^2), 偵測的準確率跟著掉, 硬做反而更糟。
        # 實測分段誤差成長 (spin12): |wz| 0.5-2 開了 -4.33 m / 關了 -1.27 m (有幫助);
        # |wz| 12-25 開了 +18.40 m / 關了 +16.63 m (有害)。
        self.omega_max = float(omega_max)
        self.acc_th = float(acc_th)
        self.window = float(window)
        self.min_samples = int(min_samples)
        self.buf = []               # (t, |omega|, |acc_xy|)

    def add(self, t, omega_z, acc_xy):
        self.buf.append((t, abs(float(omega_z)),
                         np.asarray(acc_xy, dtype=np.float64)[:2].copy()))
        while self.buf and t - self.buf[0][0] > self.window:
            self.buf.pop(0)

    def is_spinning_in_place(self) -> bool:
        if len(self.buf) < self.min_samples:
            return False
        wn = np.array([b[1] for b in self.buf])
        av = np.stack([b[2] for b in self.buf])
        # **要先把向量平均起來再取長度**, 不是把每一筆的長度平均。
        # 自旋時單筆加速度的雜訊非常大 (實測 |wz|>12 時 std 14.7 m/s^2), 但那是
        # 零均值的; 先平均向量的話雜訊會互相抵消, 剩下的才是真正的系統性分量
        # (繞圈行駛的向心加速度)。取長度再平均的話雜訊不會抵消, 門檻永遠過不了。
        # 實測 (spin12, |wz|>12 且車體中心沒動): mean(|a|)=3.82, |mean(a)|=0.28。
        return bool(wn.min() > self.omega_th and wn.max() < self.omega_max
                    and float(np.linalg.norm(av.mean(axis=0))) < self.acc_th)


class TiltTracker:
    """6 軸 IMU 的 roll/pitch —— 互補濾波。

    陀螺儀積分短期準但會漂; 加速度的方向長期準 (平均而言指向重力反方向) 但
    車子一加速就被污染。所以: 用陀螺儀推, 只在「這一刻的合力幾乎只有重力」的
    時候才用加速度把它慢慢拉回來。

    **什麼時候才算「只有重力」是這個類別唯一重要的決定。** 直覺的做法是看
    |a| 是不是接近 g, 但那個測試幾乎沒有鑑別力: 0.5 m/s^2 的水平加速度只會讓
    |a| 從 9.807 變成 9.820 (差 0.13%), 測試照過, 而它對應的假傾角是 **2.9 度**
    —— 換算成漏進水平軸的重力是 0.5 m/s^2, 剛好就是那個加速度本身。也就是說
    這個測試會讓傾角估計去追隨車子的加速度, 等於把誤差原封不動地繞回來。
    離線實測 (60 秒走走停停, 6 軸 IMU): 只用 |a| 測試 -> 位置誤差 RMS 1.6 m;
    改成只在**靜止偵測成立**時才修 -> 0.35 m。

    所以 `update` 的 `trust_accel` 要由外面的靜止偵測餵進來。沒餵的時候才退回
    |a| 測試 (聊勝於無)。

    `orientation` 模式用不到這個類別 —— 模擬器/9 軸 IMU 直接給姿態。
    """

    def __init__(self, alpha=0.02, acc_tol=0.5):
        self.roll = 0.0
        self.pitch = 0.0
        self.alpha = float(alpha)     # 每一步往加速度的答案拉多少 (0 = 純積分)
        self.acc_tol = float(acc_tol)
        self.inited = False

    def update(self, gyro, acc, dt, trust_accel=None):
        gx, gy, gz = gyro
        # 陀螺儀積分 (小角度近似; 60 Hz 下一步的角度很小)
        self.roll += (gx + math.sin(self.roll) * math.tan(self.pitch) * gy
                      + math.cos(self.roll) * math.tan(self.pitch) * gz) * dt
        self.pitch += (math.cos(self.roll) * gy - math.sin(self.roll) * gz) * dt

        n = float(np.linalg.norm(acc))
        if n < 1e-6:
            return
        ok = (abs(n - G) < self.acc_tol if trust_accel is None else bool(trust_accel))
        if not self.inited or ok:
            ar = math.atan2(acc[1], acc[2])
            ap = math.atan2(-acc[0], math.hypot(acc[1], acc[2]))
            a = 1.0 if not self.inited else self.alpha
            self.roll = (1 - a) * self.roll + a * ar
            self.pitch = (1 - a) * self.pitch + a * ap
            self.inited = True

    def gravity_free(self, acc) -> np.ndarray:
        """扣掉重力之後的車體水平加速度 (2,)。"""
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        # 重力在車體座標的分量
        g_body = np.array([-sp, cp * sr, cp * cr]) * G
        return (np.asarray(acc, dtype=np.float64) - g_body)[:2]


class ImuIns:
    """8 維 EKF: IMU 遞推 + ZUPT / ZARU / NHC 偽量測。"""

    def __init__(self, *,
                 sigma_gyro: float = 0.02,      # rad/s/sqrt(Hz)
                 sigma_gyro_scale: float = 0.01,  # 陀螺儀比例因子誤差 (無單位)
                 sigma_acc: float = 0.03,       # m/s^2/sqrt(Hz) (自旋以外的底線)
                 sigma_acc_omega: float = 0.15,  # 每 rad/s 要額外加多少 (見 predict)
                 bias_model: str = 'gm',        # gm (一階 Gauss-Markov) | rw (隨機遊走)
                 tau_bg: float = 300.0,         # GM: 陀螺零偏相關時間 (s)
                 sigma_gm_bg: float = 1.2e-3,   # GM: 陀螺零偏穩態標準差 (rad/s)
                 tau_ba: float = 300.0,         # GM: 加速度零偏相關時間 (s)
                 sigma_gm_ba: float = 1.2e-2,   # GM: 加速度零偏穩態標準差 (m/s^2)
                 sigma_bg: float = 1e-4,        # rw: 陀螺零偏隨機遊走
                 sigma_ba: float = 1e-3,        # rw: 加速度零偏隨機遊走
                 zupt_sigma: float = 0.02,      # ZUPT 宣告的速度雜訊 (m/s)
                 zaru_sigma: float = 0.002,     # ZARU 宣告的角速度雜訊 (rad/s)
                 nhc_sigma: float = 2.0,        # NHC 宣告的側向速度雜訊 (m/s), 見 nhc
                 spin_zupt_sigma: float = 0.15,  # 原地自旋時宣告的速度雜訊 (m/s)
                 nhc_min_speed: float = 0.20,   # 太慢就不做 NHC (方向沒意義)
                 nhc_max_omega: float = 1.5,    # 打滑轉彎時 NHC 假設會壞掉
                 forward_deg: float = 0.0,      # base_link +X 量到車頭的角度
                 anchor_sigma: float = 0.02,    # 長時間靜止時的位置錨定 (m)
                 anchor_after: float = 0.5,     # 靜止多久才開始錨定 (s)
                 v_max: float = 4.0,
                 chi2_scale: float = 1.0,
                 max_pos_correction: float = 0.5,   # 單次更新最多能搬動位置多少 (m)
                 nhc_pos_cap: float | None = None):  # NHC 單次最多動位置多少 (m); None = 不另外設限
        self.x = np.zeros(NX)
        # 初始不確定度: 位置/朝向由外部給 (set_pose), 速度給大一點
        self.P = np.diag([1e-4, 1e-4, 1.0, 1.0, 1e-4, 1e-4, 1e-2, 1e-2])
        self.t = None

        self.sg = float(sigma_gyro)
        self.sg_scale = float(sigma_gyro_scale)
        self.sa = float(sigma_acc)
        self.sa_omega = float(sigma_acc_omega)
        if bias_model not in ('gm', 'rw'):
            raise ValueError(f"bias_model 只能是 'gm' 或 'rw', 收到 {bias_model!r}")
        self.bias_model = bias_model
        # GM 的預設值是從 rw 的預設值換算來的 (sigma_gm = sigma_b*sqrt(tau/2),
        # tau = 300 s), 所以短時間內兩個模型的行為一樣。實際的值用 fit_noise.py 量。
        self.tau_bg = float(tau_bg)
        self.tau_ba = float(tau_ba)
        self.sgm_bg = float(sigma_gm_bg)
        self.sgm_ba = float(sigma_gm_ba)
        self.sbg = float(sigma_bg)
        self.sba = float(sigma_ba)
        # 開機常數零偏 b0 —— 狀態外面的常數, 狀態裡的 b_g / b_a 只是動態部分
        self.b0_g = 0.0
        self.b0_a = np.zeros(2)
        self.zupt_sigma = float(zupt_sigma)
        self.zaru_sigma = float(zaru_sigma)
        self.spin_zupt_sigma = float(spin_zupt_sigma)
        self.nhc_sigma = float(nhc_sigma)
        self.nhc_min_speed = float(nhc_min_speed)
        self.nhc_max_omega = float(nhc_max_omega)
        # 車頭在車體座標的方向。NHC 要約束的是**垂直於車頭**的那一軸,
        # 不是 base_link 的 y 軸 —— 這台車的 base_link 車頭是 -Y (forward_deg=-90),
        # 照 REP-103 假設 +X 是車頭的話會把前進速度歸零。
        self.forward_deg = float(forward_deg)
        phi = math.radians(self.forward_deg)
        self.fwd_body = np.array([math.cos(phi), math.sin(phi)])
        self.lat_body = np.array([-math.sin(phi), math.cos(phi)])
        self.anchor_sigma = float(anchor_sigma)
        self.anchor_after = float(anchor_after)
        self.v_max = float(v_max)
        self.chi2_scale = float(chi2_scale)
        self.max_pos_correction = float(max_pos_correction)
        self.nhc_pos_cap = None if nhc_pos_cap is None else float(nhc_pos_cap)

        self.still_since = None
        self.anchor = None
        self.counts = {'zupt': 0, 'zaru': 0, 'nhc': 0, 'anchor': 0, 'rejected': 0,
                       'damped': 0, 'spin': 0}
        self.last_omega = 0.0

    # ------------------------------------------------------------------
    @property
    def pos(self):
        return self.x[IP].copy()

    @property
    def vel(self):
        return self.x[IV].copy()

    @property
    def yaw(self) -> float:
        return float(self.x[IYAW])

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.x[IV]))

    @property
    def gyro_bias(self) -> float:
        """陀螺儀 z 的總零偏 (b0 + 動態部分) —— 原始讀數要扣掉的就是這個。"""
        return float(self.b0_g + self.x[IBG])

    @property
    def acc_bias(self):
        """加速度計 xy 的總零偏 (b0 + 動態部分)。"""
        return self.b0_a + self.x[IBA]

    def accel_is_quiet(self, mean_acc_xy, threshold: float) -> bool:
        """視窗平均的水平加速度扣掉零偏之後夠不夠小 (見 StillConfirm)。

        門檻是固定的, **不**隨零偏的不確定度放寬。放寬的話門檻會跟著零偏模型
        的參數變: sigma_gm_ba = 0.019 時 3 sigma 就把 0.10 放寬到 0.18, 煞車
        尾段 0.1~0.2 m/s^2 的減速度剛好全部漏進來。實測三趟 (Isaac bag):
        放寬 -> 位置 RMS 1.71 / 2.30 / 1.16 m; 不放寬 -> 1.04 / 1.32 / 0.76 m,
        車頭軸的加速度零偏誤差也從 0.011~0.017 降到 0.007~0.010 m/s^2。

        代價: 零偏估計錯超過門檻就再也進不了靜止。開機靜止校正有完成的話
        不會發生 (之後的漂移遠小於門檻); 沒完成的話節點會把這一道關掉。
        """
        m = np.asarray(mean_acc_xy, dtype=np.float64).reshape(2) - self.acc_bias
        return bool(float(np.linalg.norm(m)) < float(threshold))

    def gyro_is_quiet(self, mean_omega_z: float, threshold: float,
                      k_sigma: float = 3.0) -> bool:
        """視窗平均的角速度扣掉零偏之後夠不夠小 (見 StillConfirm)。"""
        thr = float(threshold) + k_sigma * math.sqrt(max(self.P[IBG, IBG], 0.0))
        return bool(abs(float(mean_omega_z) - self.gyro_bias) < thr)

    def set_bias0(self, gyro_z: float, acc_xy):
        """開機靜止校正量到的常數零偏 b0。動態部分從 0 開始。"""
        self.b0_g = float(gyro_z)
        self.b0_a = np.asarray(acc_xy, dtype=np.float64).reshape(2).copy()
        self.x[IBG] = 0.0
        self.x[IBA] = 0.0

    def set_pose(self, x, y, yaw, t=None, sigma_p=0.02, sigma_yaw=0.02):
        self.x[IP] = [float(x), float(y)]
        self.x[IV] = 0.0
        self.x[IYAW] = wrap_pi(float(yaw))
        self.P[0, 0] = self.P[1, 1] = sigma_p ** 2
        self.P[2, 2] = self.P[3, 3] = 0.25
        self.P[4, 4] = sigma_yaw ** 2
        if t is not None:
            self.t = float(t)
        self.anchor = None
        self.still_since = None

    # ------------------------------------------------------------------ 遞推
    def predict(self, t: float, acc_xy, omega_z: float):
        """acc_xy 是**已經扣掉重力**的車體水平加速度; omega_z 是原始陀螺儀 z。"""
        if self.t is None:
            self.t = float(t)
            return
        dt = t - self.t
        if dt <= 0.0:
            return
        if dt > 0.5:            # 斷太久就不要一次推一整段, 那只會產生天文數字
            self.t = float(t)
            return
        self.t = float(t)

        a_b = np.asarray(acc_xy, dtype=np.float64).reshape(2) - self.acc_bias
        w = float(omega_z) - self.gyro_bias
        self.last_omega = w
        yaw = self.x[IYAW]
        R = rot2(yaw)
        a_w = R @ a_b

        # --- 名目狀態 ---
        self.x[IP] = self.x[IP] + self.x[IV] * dt + 0.5 * a_w * dt * dt
        self.x[IV] = self.x[IV] + a_w * dt
        self.x[IYAW] = wrap_pi(yaw + w * dt)
        # 零偏的動態部分: GM 往 0 衰減, rw 不動
        gm = self.bias_model == 'gm'
        phi_g = math.exp(-dt / self.tau_bg) if gm else 1.0
        phi_a = math.exp(-dt / self.tau_ba) if gm else 1.0
        self.x[IBG] *= phi_g
        self.x[IBA] *= phi_a

        # --- 雅可比 ---
        F = np.eye(NX)
        F[IP, IV] = np.eye(2) * dt
        JRa = J90 @ a_w                                  # d(a_w)/d(yaw)
        F[0:2, IYAW] = 0.5 * dt * dt * JRa
        F[2:4, IYAW] = dt * JRa
        F[0:2, IBA] = -0.5 * dt * dt * R
        F[2:4, IBA] = -dt * R
        F[IYAW, IBG] = -dt
        F[IBG, IBG] = phi_g
        F[6, 6] = F[7, 7] = phi_a

        # --- 過程雜訊 ---
        # 速度的過程雜訊一定要是 sigma_a^2 * dt, 不是 sigma_a^2 * dt^2。
        # 後者在 60 Hz 下把它算小了 60 倍, P 長得比實際誤差慢 -> 濾波器過度自信
        # -> 正確的量測被卡方門檻擋掉 -> 更相信 IMU -> 更漂。
        # 加速度的過程雜訊必須**隨角速度長大**。實測 (spin12 bag, 拿 IMU 扣完
        # 重力的水平加速度跟 ground truth 微分出來的真實加速度比):
        #
        #   |wz| < 0.2 rad/s  -> 殘差 0.064 m/s^2   (連續時間 0.008)
        #   |wz| ~ 1-2        -> 殘差 0.27          (連續時間 0.035)
        #   |wz| > 12         -> 殘差 14.7          (連續時間 1.9)
        #
        # 差了 230 倍。**一個常數不可能同時對。** 給常數 0.35 的話:
        #   直線行駛時 P 被灌大 43 倍 -> 速度偽量測的增益逼近 1, 一次 NHC 更新
        #     就把位置搬走好幾公尺 (實測單步跳動最大 27 公尺), 而且 S 也跟著
        #     大, 卡方閘門變成擺設, 什麼都擋不住;
        #   高速自旋時 P 又太小 -> 正確的 ZUPT / NHC 被當成離群值擋掉。
        # 兩頭都錯, 而且錯的方向相反 —— 這就是為什麼「調 sigma_acc」怎麼調都
        # 不對。正確做法是讓它跟著 |w| 走 (跟 sigma_gyro_scale 同一個道理)。
        qa = self.sa ** 2 + (self.sa_omega * abs(w)) ** 2
        # 陀螺儀的比例因子誤差: 轉得越快這一項越大, 而白雜訊那一項跟轉速無關。
        # 少了它, 高速自旋時 P[yaw] 還停在停著時的大小, ZARU/NHC 的增益就上不去。
        qg = self.sg ** 2 + (self.sg_scale * abs(w)) ** 2
        Q = np.zeros((NX, NX))
        Q[0, 0] = Q[1, 1] = qa * dt ** 3 / 3.0
        Q[2, 2] = Q[3, 3] = qa * dt
        Q[0, 2] = Q[2, 0] = Q[1, 3] = Q[3, 1] = qa * dt * dt / 2.0
        Q[IYAW, IYAW] = qg * dt
        if gm:
            Q[IBG, IBG] = self.sgm_bg ** 2 * (1.0 - phi_g ** 2)
            Q[6, 6] = Q[7, 7] = self.sgm_ba ** 2 * (1.0 - phi_a ** 2)
        else:
            Q[IBG, IBG] = self.sbg ** 2 * dt
            Q[6, 6] = Q[7, 7] = self.sba ** 2 * dt

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)          # 保持對稱

        s = self.speed
        if s > self.v_max:                          # 發散時第一個爆掉的是速度
            self.x[IV] *= self.v_max / s

    # ------------------------------------------------------------------ 更新
    def _update(self, H, r, R, name: str, gate: bool = True,
                pos_cap: float | None = None) -> bool:
        """通用的 EKF 更新, 附卡方閘門。r 是 innovation (量測 - 預測)。

        兩個閘門, 缺一不可:

        **卡方閘門**擋的是「跟自己的不確定度比起來太離譜」的量測。它的盲點是
        P 一大 S 就跟著大, NIS 就永遠過關 —— 濾波器越迷路, 這道門越擋不住東西。

        **位置修正上限** (`max_pos_correction`) 補的就是這個盲點。ZUPT / ZARU /
        NHC 都是**速度**的偽量測, 它們之所以能改到位置, 完全來自 P 裡面
        位置-速度的相關項。那一項在純積分下會長到相關係數 sqrt(3)/2, 於是
        位置的修正量 ~ 0.87 * (sigma_p / sigma_v) * innovation。sigma_p 漲到
        8 公尺的時候, 一個 1 m/s 的側滑 innovation 就會把位置瞬間搬 2 公尺 ——
        實測資料裡就有 23 次單步跳超過 0.5 公尺, 最大 4 公尺, 而同一步車子只
        走了 4 公分。那不是在修正, 是在亂丟。

        超過上限時把**整個增益**等比例縮小 (不是只截斷位置那兩維) —— 這樣
        修正的方向還是對的, 只是走短一點。共變異數用 Joseph form 更新, 它對
        **任意**增益都成立 (不限於最佳增益), 所以縮過的增益仍然是一致的。
        """
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
        if pos_cap is not None:
            # 這個量測能動位置的量另外設上限: 只縮增益的**位置那兩列**, 速度 /
            # 朝向 / 零偏的修正照常。Joseph form 對任意增益都成立, 共變異數
            # 仍然一致。
            dp = float(np.linalg.norm(dx[IP]))
            if dp > pos_cap:
                K[IP, :] *= (pos_cap / dp) if dp > 0.0 else 0.0
                dx = K @ r
        dp = float(np.linalg.norm(dx[IP]))
        if self.max_pos_correction > 0.0 and dp > self.max_pos_correction:
            K = K * (self.max_pos_correction / dp)
            dx = K @ r
            self.counts['damped'] += 1
        self.x = self.x + dx
        self.x[IYAW] = wrap_pi(self.x[IYAW])
        I_KH = np.eye(NX) - K @ H
        # Joseph form: 數值上對稱且保持正定, 長時間跑不會慢慢變成非對稱矩陣
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self.counts[name] = self.counts.get(name, 0) + 1
        return True

    # --- ZUPT: 靜止時速度是 0 ---
    #
    # 這三個 (ZUPT / ZARU / 零加速度) 都**不吃卡方閘門**, 而 NHC 吃。差別在
    # 「憑什麼相信這個偽量測」:
    #
    #   ZUPT/ZARU 的依據是**靜止偵測** —— 一個跟濾波器狀態完全獨立的判斷,
    #   看的是感測器訊號本身的結構。它說靜止, 速度就是 0, 這件事跟「濾波器
    #   現在以為自己在跑多快」一點關係也沒有。拿卡方閘門去擋它, 意思變成
    #   「濾波器越迷路 -> innovation 越大 -> 越拒絕接受修正」, 剛好把最需要
    #   救的情況擋掉。實測 (合成 6 軸 + ZUPT/ZARU): 吃閘門 64.7 m, 不吃 3.9 m。
    #
    #   NHC 沒有這種獨立依據 —— 它是一個**假設** (車不會橫著走), 而 skid-steer
    #   打滑轉彎時這個假設真的會壞掉, 沒有任何東西會事先告訴你。所以 NHC 要
    #   留著閘門, 讓明顯違反的時候被擋下來。
    def zupt(self):
        H = np.zeros((2, NX))
        H[0, 2] = H[1, 3] = 1.0
        self._update(H, -self.x[IV], np.eye(2) * self.zupt_sigma ** 2, 'zupt',
                     gate=False)

    # --- 原地自旋時平移速度是 0 (見 SpinDetector) ---
    #
    # 跟 ZUPT 一樣不吃卡方閘門 —— 依據同樣是一個獨立的偵測, 而不是「跟遞推
    # 合不合」。只修速度: 不錨定位置、不估零偏。
    def spin_zupt(self):
        H = np.zeros((2, NX))
        H[0, 2] = H[1, 3] = 1.0
        self._update(H, -self.x[IV], np.eye(2) * self.spin_zupt_sigma ** 2,
                     'spin', gate=False)

    # --- ZARU: 靜止時陀螺儀讀到的全部是零偏 ---
    def zaru(self, omega_z: float):
        H = np.zeros((1, NX))
        H[0, IBG] = -1.0                 # 量測模型: 0 = omega_m - b_g
        r = np.array([-(float(omega_z) - self.gyro_bias)])
        self._update(H, r, np.array([[self.zaru_sigma ** 2]]), 'zaru', gate=False)

    # --- 加速度計零偏: 靜止時扣掉重力後的水平加速度應該是 0 ---
    def zero_accel(self, acc_xy):
        H = np.zeros((2, NX))
        H[0, 6] = H[1, 7] = -1.0
        r = -(np.asarray(acc_xy, dtype=np.float64).reshape(2) - self.acc_bias)
        self._update(H, r, np.eye(2) * 0.05 ** 2, 'zaccel', gate=False)

    # --- 位置錨定: 停久了不要讓位置隨機遊走 ---
    def anchor_position(self):
        if self.anchor is None:
            self.anchor = self.x[IP].copy()
        H = np.zeros((2, NX))
        H[0, 0] = H[1, 1] = 1.0
        self._update(H, self.anchor - self.x[IP],
                     np.eye(2) * self.anchor_sigma ** 2, 'anchor')

    # --- 絕對 yaw (IMU 的 orientation) ---
    def update_yaw(self, yaw_meas: float, sigma: float = 0.02, dt: float = 0.0):
        """用 IMU 訊息裡的 orientation 修 yaw。

        這仍然**只用 IMU 自己的資料** —— 9 軸 IMU (含磁力計) 或模擬器都給得出
        絕對姿態。Isaac 的 IsaacReadIMU 給的是精確值, 所以模擬裡開這個之後
        yaw 幾乎沒有誤差, 漂移只剩加速度那一路。真車的 6 軸 IMU 沒有這個東西,
        要用 yaw_source:=gyro。
        """
        H = np.zeros((1, NX))
        H[0, IYAW] = 1.0
        r = np.array([wrap_pi(float(yaw_meas) - self.x[IYAW])])
        # 這一步用 w*dt 遞推 yaw, 本來就會落後真實轉角約 |w|*dt/2 (一階保持的
        # 離散化誤差)。原地自旋 20 rad/s 時那是 0.17 rad —— 比 sigma 0.02 大
        # 一個數量級, 卡方閘門會把**完全正確的量測**當成離群值擋掉。
        # 實測 (spin12 bag): 擋掉 65%, 狀態的 yaw 中位誤差 61 度, 最大 180 度。
        # 所以: R 依這一步轉了多少放寬, 而且不吃閘門 —— 在 orientation 模式下
        # 量測本來就比遞推可信, 拿「跟遞推差太多」當理由丟掉它是本末倒置。
        sd = math.hypot(float(sigma), 0.5 * abs(self.last_omega) * float(dt))
        self._update(H, r, np.array([[sd ** 2]]), 'yaw', gate=False)

    # --- NHC: 車體側向速度是 0 ---
    #
    # **nhc_sigma 不是「側滑有多大」, 而是「每一筆 NHC 該算多少份量」。**
    # NHC 每一筆 IMU 都做一次 (60 Hz), 而 skid-steer 轉彎時的側滑是**持續好幾秒
    # 的同一個偏差**, 不是每一筆獨立的雜訊。把它當成獨立量測的話, 一秒鐘就重複
    # 算了 60 次: 實測車子以 0.6 m/s 等速行駛、剛開始緩轉 (0.1 rad/s, 真實側滑
    # 0.013 m/s) 的 2 秒內, NHC 把估計的**前進速度**拉掉 0.28 m/s —— 連續行駛
    # 時速度的不確定度主要在車頭方向, 車頭一轉, 那個不確定度就投影到側向,
    # 被重複計入的側滑一路修掉。
    #
    # 所以 sigma 要乘上 sqrt(相關時間 x 取樣率)。0.15 m/s 的側滑、相關時間約
    # 3 秒、60 Hz -> 2.0。實測 10 趟 (Isaac bag, 各約 35 m) 的位置 RMS 平均:
    #   0.15 -> 1.24 m,  0.5 -> 0.95,  1.2 -> 0.66,  2.0 -> 0.53,  不開 NHC -> 1.25
    # 代價: NHC 完全成立的情況 (合成資料、沒有側滑) 會慢一點收斂
    # (test_ins.py 的「全開」0.03 -> 0.13 m)。IMU 取樣率不是 60 Hz 的話照
    # sqrt(取樣率 / 60) 縮放。
    #
    # `nhc_pos_cap` 可以另外限制 NHC 單次動到位置的量 (只縮增益的位置那兩列)。
    # sigma 還是 0.15 的時候它有幫助, sigma 調對之後就不需要了, 預設不設限。
    def nhc(self):
        if self.speed < self.nhc_min_speed:
            return
        if abs(self.last_omega) > self.nhc_max_omega:
            # skid-steer 打滑轉彎時車子真的會橫移, 這時候 NHC 是錯的
            return
        R = rot2(self.x[IYAW])
        lat = R @ self.lat_body                     # 側向軸在世界座標
        H = np.zeros((1, NX))
        H[0, 2:4] = lat
        # d(R u)/d(yaw) = J90 @ (R u)
        H[0, IYAW] = float((J90 @ lat) @ self.x[IV])
        r = np.array([-float(lat @ self.x[IV])])
        self._update(H, r, np.array([[self.nhc_sigma ** 2]]), 'nhc',
                     pos_cap=self.nhc_pos_cap)

    # --- 車頭方向的速度 (只給 log / odom twist 看, 不參與濾波) ---
    @property
    def forward_speed(self) -> float:
        return float((rot2(self.x[IYAW]) @ self.fwd_body) @ self.x[IV])

    # ------------------------------------------------------------------
    def sigma_pos(self) -> float:
        return float(math.sqrt(max(self.P[0, 0] + self.P[1, 1], 0.0)))

    def pose_cov(self) -> np.ndarray:
        c = np.zeros((6, 6))
        c[0, 0], c[0, 1] = self.P[0, 0], self.P[0, 1]
        c[1, 0], c[1, 1] = self.P[1, 0], self.P[1, 1]
        c[2, 2] = c[3, 3] = c[4, 4] = 1e-6
        c[5, 5] = self.P[4, 4]
        return c

    def twist_cov(self) -> np.ndarray:
        c = np.zeros((6, 6))
        c[0, 0], c[1, 1] = self.P[2, 2], self.P[3, 3]
        c[2, 2] = c[3, 3] = c[4, 4] = 1e-3
        c[5, 5] = self.P[4, 4]
        return c
