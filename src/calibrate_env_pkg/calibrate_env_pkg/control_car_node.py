#!/usr/bin/env python3
"""跨環境地面摩擦力識別用的開迴路測試腳本 (輪速控制版)。

目標: 在不同 groundCollider 摩擦係數的場景裡送出**完全相同**的輪速指令,
      讓 collect_data_node 記下車子的反應 (輪子扭矩、gyro、加速度), 之後回歸出 mu。

為什麼是開迴路: 系統識別要看的是「同樣輸入 -> 不同輸出」。量測段的指令**只由
時間決定**, 不看任何回授; 只有「把車開回起點」那一段才用閉迴路 (phase 標成
reposition, 分析時整段丟掉)。

────────────────────────────────────────────────────────────────────────
輪速控制下摩擦力藏在哪裡 (跟舊的 effort 控制版完全不同)
────────────────────────────────────────────────────────────────────────
這台車 (kros_car.usd / 實體 KROS car) 的 /joint_command 收的是四輪的**輪速**
(rad/s), 馬達自己的速度環會出到最大 2.0 N·m 去追。後果:

1) 沒有「斷油滑行」。指令 0 是主動煞車, 不是放開 —— 舊版拿來當主特徵的
   coast-down (斷油後的減速度) 在這裡不存在。

2) 車體的**運動**不是好的摩擦特徵。速度環把輪速釘在指令上, 原地轉的穩態車體
   轉速在理想 Coulomb 摩擦下由幾何決定:
       Omega = r * w_wheel * b / (a^2 + b^2)     (a = 半軸距, b = 半輪距)
   跟 mu 無關。(PhysX 實測會隨 mu 稍微變, 但那是接觸模型的細節, 不是能拿來
   反推 mu 的東西。)

3) 摩擦跑到**馬達出多少扭矩**上。skid-steer 原地轉時四輪都在滑, 每一輪的
   摩擦力大小是 mu*N, 方向由滑移方向決定, 穩態時
       tau_wheel = r * mu * N * a / sqrt(a^2 + b^2)
   正比於 mu, 跟轉速無關 (Coulomb)。而且這是**穩態量**, 可以平均好幾秒 ——
   比舊版擬合 0.15 s 的減速度好量得多。
   回授來源: 模擬是 /joint_states 的 effort, 實體車是 /wheel_feedback 的 effort
   (馬達電流 x 扭矩常數)。

   **前提是馬達扭矩沒有飽和。** 馬達出得了的扭矩隨轉速下降 (DDSM115: 堵轉
   2.0 N·m, 115 rpm 剩 0.96), 需要的扭矩超過它時輪速追不上指令, 量到的就是馬達的
   上限, 跟 mu 無關 (車太重 / mu 太大 / 轉太快時會發生)。所以自旋用比較低的輪速
   (spin_wheel_speed), 分析端 (estimate_friction.py) 會檢查「扭矩貼著那個轉速下的
   上限」與「輪速追不上」, 飽和的那一檔不用。

   **回授的扭矩裡還有馬達自己的損耗** (實體車: 電流 x 扭矩常數, 含軸承 / 鐵損),
   Block 3 的等速直行就是用來量它的, 分析時從自旋扭矩裡扣掉。

Block 1  等速原地轉         輪子扭矩 ∝ mu_dynamic = 主特徵     位移 ~0
Block 2  慢速 ramp 原地轉   起轉瞬間的峰值扭矩 (靜摩擦, 副)   位移 ~0
Block 3  等速直行 + 急煞    馬達損耗 (扣掉用) + 煞停減速度     每趟 < 1.5 m
Block 4  起步 (輪速階躍)    起步加速度與 slip ratio (副)       每趟 < 1.5 m

Block 3/4 只有在抓地力先到上限 (輪子打滑 / 鎖死) 時才帶摩擦資訊; 扭矩先到
上限的話它們跟地面無關。分析端用 slip ratio 分辨這兩種情況。

────────────────────────────────────────────────────────────────────────
指令介面
────────────────────────────────────────────────────────────────────────
* /joint_command: sensor_msgs/JointState, `name` 四個輪子、`velocity` 是 rad/s。
  正值 = 往前。跟實體車的 wheel_velocity_controller 同一種訊息。
* **要持續送。** 實體車 0.3 秒沒收到指令輪子就停; 模擬則相反, 會一直維持最後
  一筆 —— 所以腳本跑完 / 被 Ctrl-C 時要補送 0 (見 stop_car)。
* THROTTLE / STEER 現在的單位是**輪速 rad/s**: 左輪 = throttle - steer,
  右輪 = throttle + steer。超過 max_wheel_speed 時等比縮放。
* /test_phase 給 collect_data_node 標 CSV:
      measure     開迴路量測段 <- **分析時只取這個**
      reposition  閉迴路開回起跑點
      brake       量測段之間的煞停 (讓下一段從靜止開始)
      aborted     撞牆保護中止的殘缺資料
      idle        腳本跑完
* geofence: 離牆太近就中止當前情境、標成 aborted、開回起點重試。

────────────────────────────────────────────────────────────────────────
狀態從哪裡來: state_source = sensor (預設) | gt
────────────────────────────────────────────────────────────────────────
真車沒有 ground truth, 所以預設就用感測器跑, 模擬器裡的 /odom 只拿來**對答案**
(gt_check_topic, 定期印誤差, 不參與控制)。

  量                  sensor 模式                      為什麼
  ─────────────────── ──────────────────────────────── ─────────────────────────
  位置 / 朝向         /fusion_loc/odom 的 pose         geofence 與歸位要絕對位置
  轉速 wz             /imu gyro z (扣開機零偏)          不受打滑影響、不會斷
  geofence 的車速     位姿差分與 IMU 積分取大的         融合的速度吃輪速, 打滑時
                                                       會錯
  煞車 / 歸位的速度   /fusion_loc/odom twist.linear.x   閉迴路段, 準不準只影響收斂
  靜止判定            gyro + 加速度抖動 + 融合速度

安全上的差別: GT 不會斷, 感測器會。所以 sensor 模式下
  * 定位過期 (pose_timeout) 或 sigma 太大 (pose_sigma_max) -> 量測段直接中止、煞停。
  * 牆邊餘裕再加 sigma_margin_k x 定位 sigma。
  * 歸位前先等定位回到可信 (repos_sigma_max)。
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

JOINT_NAMES = ['front_left_joint', 'front_right_joint',
               'rear_left_joint', 'rear_right_joint']

ODOM_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=10)

# ── 車體幾何 (kros_car.usd 量出來的; 都有對應的參數可以蓋掉) ───────────
WHEEL_RADIUS = 0.05035       # m
HALF_TRACK = 0.135           # m, 輪胎接地中心到車體中線 (輪距 0.27)
HALF_WHEELBASE = 0.144       # m (軸距 0.288)
# skid-steer 原地轉的「等效輪距」: Omega = r*(w_R - w_L) / T_eff。四輪都在橫向滑,
# 所以比幾何輪距 (0.27) 大。理想 Coulomb 摩擦是 2*(a^2 + b^2)/b = 0.577; 無頭 PhysX
# 實測 0.68~1.04 (mu 大、轉得慢時比較大)。歸位的積分項會補剩下的落差。
# estimate_friction.py 的 B1 會印實際量到的值: 2026-10-13 kros_car.usd (12.5 kg,
# 地面 0.5) 兩輪實跑是 0.945 / 1.055。
EFFECTIVE_TRACK = 0.95

# 實體車的馬達驅動 (DDSM115) 只收**整數 rpm**: rad/s 換成 rpm 之後四捨五入。模擬的
# 關節沒有這個限制, 所以這裡自己先量化 —— 兩邊實際執行的輪速才會是同一個數字
# (4.2 rad/s = 40.1 rpm, 實體車跑的是 40 rpm = 4.189 rad/s)。
RPM = 2.0 * math.pi / 60.0               # 1 rpm = 0.1047 rad/s


def snap_rpm(w, step=1.0):
    """輪速 (rad/s) -> 最接近的 step rpm 的整數倍。step <= 0 = 不量化。"""
    if step <= 0.0:
        return float(w)
    q = step * RPM
    return round(float(w) / q) * q


# ── Block 1: 等速原地轉 (主特徵: 輪子扭矩) ───────────────────────────
# 三檔輪速不是為了掃描, 是為了驗證 Coulomb 假設 (扭矩跟轉速無關): 如果扭矩隨
# 輪速明顯上升, 代表有黏滯項 (軸承 / 馬達反電動勢), 分析時要看的是截距。
# 兩個方向都跑: 左右馬達 / 載重不對稱會讓單一方向有偏差, 正反平均掉。
# 自旋的輪速另外給 (spin_wheel_speed), 不跟直線段共用 max_wheel_speed: 馬達出得了的
# 扭矩隨轉速下降 (DDSM115: 堵轉 2.0 N·m, 6 rad/s 剩 1.61, 10 rad/s 剩 1.18), 而自旋
# 要的扭矩跟轉速無關 —— 轉得愈快愈容易飽和, 飽和的那一檔量到的是馬達不是地面。
# 2026-10-13 無頭 PhysX (12.5 kg): 地面 1.0 時 10 rad/s 三檔有兩檔飽和, 6 rad/s 只有
# 最高那一檔; 地面 0.5 兩種都不飽和。所以預設 6。
SPIN_WHEEL_SPEED = 6.0                   # rad/s
SPIN_LEVELS = (0.4, 0.7, 1.0)            # x spin_wheel_speed
# 前 ~1 秒是起轉過渡 (輪子先到速、車體還在加速), 分析端會丟掉, 剩下的取平均。
SPIN_HOLD_DURATION = 4.0                 # ← 特徵取在這一段的後 3 秒
SPIN_REPEATS = 3                         # x 3 檔 x 2 方向 = 18 次

# ── Block 2: 慢速 ramp 原地轉 (副特徵: 起轉峰值扭矩 = 靜摩擦) ────────
# 輪速從 0 慢慢拉高。車體還沒轉之前輪子被靜摩擦咬住, 速度環的扭矩會一路升到
# 咬不住為止 —— 那個峰值是靜摩擦, 之後掉回動摩擦的水準。速度環很硬的話峰值
# 只有一兩個取樣, 所以這只是副特徵 (校正迴圈預設 static 跟著 dynamic 走)。
RAMP_END_FRAC = 0.5                      # x spin_wheel_speed
RAMP_DURATION = 6.0
RAMP_REPEATS = 2                         # x 2 方向 = 4 次

# ── Block 3: 等速直行 + 急煞 ─────────────────────────────────────────
# 直行段量的是**馬達自己的損耗** (等速直行時輪胎不滑, 扭矩 = 損耗 + 滾動阻力, 跟地面
# 摩擦係數無關)。實體車的扭矩回授是「電流 x 扭矩常數」, 裡面含這一塊; 分析時要從
# 自旋扭矩裡扣掉, 而且要扣**同一個輪速**下的值 —— 所以直行的兩檔輪速就取自旋的
# 最低檔與最高檔, 中間那一檔用內插 (estimate_friction.py 的 fit_loss)。
# 急煞 = 輪速指令瞬間歸零。馬達煞得住輪子 (扭矩夠) 的話輪子鎖死、車體在地上滑,
# 減速度 = mu*g; 煞不住就是扭矩受限, 減速度跟地面無關。
CRUISE_LEVELS = (SPIN_LEVELS[0], SPIN_LEVELS[-1])    # x spin_wheel_speed
CRUISE_HOLD = 1.5                        # 到速之後再維持這麼久
CRUISE_REPEATS = 2
BRAKE_DURATION = 1.0

# ── Block 4: 起步 (輪速階躍) ─────────────────────────────────────────
# 跳過斜率限制, 輪速指令直接跳到上限。抓地力不夠時輪子空轉, 車體加速度 = mu*g。
LAUNCH_DURATION = 1.0
LAUNCH_REPEATS = 3

REST_DURATION = 1.0
# 輪速指令的斜率上限 (rad/s^2), instant 型會略過。20 rad/s^2 = 車體約 1 m/s^2。
WHEEL_ACCEL_LIMIT = 20.0

# IMU 靜止判定 (sensor 模式)。門檻給的是真車 MEMS 的量級, Isaac 的 IMU 沒雜訊一定過。
IMU_STILL_WINDOW = 0.3  # s
STILL_GYRO = 0.03       # rad/s, 窗內平均 (扣零偏後)
STILL_ACC_STD = 0.08    # m/s^2, 窗內抖動 (馬達 / 路面振動)
STILL_ACC_MEAN = 0.15   # m/s^2, 窗內平均前向加速度 —— 擋掉「等加速度」被當成靜止
LOC_WAIT_MAX = 30.0     # s, 歸位前等定位恢復的上限
# 定位一致性檢查 (sensor 模式): 位姿 yaw 在 LOC_CHECK_WINDOW 秒內的變化 vs 同一段
# gyro 積分。sigma 抓不到「定位很有自信地錯了」—— 2026-09-15 那一輪融合定位整輪
# 凍結 (相機時戳跨場景沒歸零), yaw 誤差 100 度, sigma 卻只有 0.001。
LOC_CHECK_WINDOW = 1.0  # s
LOC_CHECK_MIN_TURN = 0.5  # rad, 兩邊都轉不到這麼多就判斷不了 (靜止時不下結論)
LOC_CHECK_TOL = 0.35    # rad (20 度)


def _rate_limit(target, current, max_rate, dt):
    max_delta = max_rate * dt
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


def _wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


# ═══════════════════════════════════════════════════════════════════════
#  測試腳本
# ═══════════════════════════════════════════════════════════════════════
def _const(name, block, throttle, steer, duration, min_clear,
           min_ahead=None, group=None, min_useful=0.5, from_rest=False):
    """throttle / steer 是**輪速** (rad/s): 左輪 = throttle - steer, 右輪 = throttle + steer。

    group = 這一段屬於哪個 trial。中止重試時整個 trial 一起重跑 —— 只重跑煞車段
    是沒有意義的, 因為它的初始速度是前面那個直行段給的。

    min_useful : 幾何圍籬踩線時, 已經收滿這麼多秒就當作「提早收工」而不是
                 「中止」—— 資料留著, 不重試。
    from_rest : 這一段從靜止起步 -> 進來時把 IMU 的速度積分歸零 (geofence 用)。
    """
    return {'name': name, 'block': block, 'type': 'const', 'phase': 'measure',
            'throttle': throttle, 'steer': steer, 'duration': duration,
            'min_clear': min_clear, 'min_ahead': min_ahead, 'group': group,
            'min_useful': min_useful, 'from_rest': from_rest}


def _instant(name, block, throttle, steer, duration, min_clear,
             min_ahead=None, group=None, min_useful=0.5, from_rest=False):
    """跳過斜率限制直接跳到目標輪速。急煞 (throttle=0) 與起步階躍都用它。"""
    d = _const(name, block, throttle, steer, duration, min_clear, min_ahead,
               group, min_useful, from_rest)
    d['type'] = 'instant'
    return d


def _ramp(name, block, var, start, end, duration, min_clear,
          min_ahead=None, group=None):
    return {'name': name, 'block': block, 'type': 'ramp', 'phase': 'measure',
            'ramp_var': var, 'ramp_start': start, 'ramp_end': end,
            'throttle': 0.0, 'steer': 0.0, 'duration': duration,
            'min_clear': min_clear, 'min_ahead': min_ahead, 'group': group,
            'min_useful': 0.5, 'from_rest': False}


def _rest(duration=REST_DURATION):
    """組間的靜止。標成 measure 是刻意的 —— 靜止段可以拿來估感測器零偏。"""
    return _const('Rest', '-', 0.0, 0.0, duration, min_clear=0.0)


def _brake(name, phase='brake', duration=3.0):
    """煞停並等車子真的靜止。輪速指令 0 就是煞車 (馬達的速度環會出反向扭矩)。

    每個量測段後面都接一個: 下一個量測段要從**靜止**開始, 否則初始條件跟著上
    一段的殘餘速度走, 跨環境就不可比了。
    phase 用 'brake' (中止時才用 'aborted'), 兩者都不是 measure, 分析時都丟掉。
    """
    return {'name': name, 'block': '-', 'type': 'brake', 'phase': phase,
            'duration': duration, 'min_clear': 0.0, 'min_ahead': None}


def _repos(name, x, y, heading_deg, timeout):
    return {'name': name, 'block': '-', 'type': 'reposition', 'phase': 'reposition',
            'target_x': x, 'target_y': y, 'target_heading': math.radians(heading_deg),
            'duration': timeout, 'min_clear': 0.0, 'min_ahead': None}


def _calibrate(duration, timeout):
    """開跑前靜止: 等定位與 IMU 都有資料, 再靜止 duration 秒量 gyro 零偏與加速度基準。
    phase 標 'calibrate' —— 分析腳本也拿這段估零偏 (比 Rest 可靠, 見 estimate_friction)。"""
    return {'name': 'Calibrate (stand still)', 'block': '-', 'type': 'calibrate',
            'phase': 'calibrate', 'duration': duration, 'timeout': timeout,
            'min_clear': 0.0, 'min_ahead': None}


def build_scenarios(room, repos_timeout, wall_margin, blocks=(1, 2, 3, 4),
                    max_wheel_speed=10.0, calib_duration=2.0,
                    spin_wheel_speed=SPIN_WHEEL_SPEED, rpm_step=1.0):
    """room = (x_min, x_max, y_min, y_max) 的內牆範圍。

    min_clear = 離最近的牆至少要有幾公尺 (0 = 不檢查)
    min_ahead = 沿車頭方向至少要有幾公尺 (None = 不檢查); 實際門檻還會再加上
                目前速度的煞停距離, 見 _violation。
    """
    x_min, x_max, y_min, y_max = room
    cx, cy = 0.5 * (x_min + x_max), 0.5 * (y_min + y_max)
    wmax = snap_rpm(max_wheel_speed, rpm_step)
    wspin = min(float(spin_wheel_speed), float(max_wheel_speed))
    # 沿長軸兩端的起跑點, 留 1.5 m 讓車子擺得下並且有反應空間
    launch = [(x_min + 1.5, cy, 0.0),        # 面向 +X
              (x_max - 1.5, cy, 180.0)]      # 面向 -X

    S = [_calibrate(calib_duration, timeout=60.0)]

    # ── Block 1: 等速原地轉 ─────────────────────────────────────────
    if 1 in blocks:
        S.append(_repos('Reposition -> center (B1)', cx, cy, 0.0, repos_timeout))
        for rep in range(SPIN_REPEATS):
            for frac in SPIN_LEVELS:
                for sgn, way in ((1.0, 'ccw'), (-1.0, 'cw')):
                    w = round(snap_rpm(frac * wspin, rpm_step), 3)
                    tag = f'w={w:g} {way} rep{rep + 1}'
                    S.append(_const(f'B1 spin_hold {tag}', 'B1_spin_hold',
                                    0.0, sgn * w, SPIN_HOLD_DURATION,
                                    min_clear=wall_margin,
                                    group=f'B1_{rep}_{w:g}_{way}', min_useful=2.5))
                    S.append(_brake(f'Brake after B1 {tag}'))
                    S.append(_rest(0.5))

    # ── Block 2: 慢速 ramp 原地轉 ───────────────────────────────────
    if 2 in blocks:
        S.append(_repos('Reposition -> center (B2)', cx, cy, 0.0, repos_timeout))
        for rep in range(RAMP_REPEATS):
            for sgn, way in ((1.0, 'ccw'), (-1.0, 'cw')):
                tag = f'{way} rep{rep + 1}'
                S.append(_ramp(f'B2 spin_ramp {tag}', 'B2_spin_ramp', 'steer',
                               0.0, sgn * snap_rpm(RAMP_END_FRAC * wspin, rpm_step),
                               RAMP_DURATION,
                               min_clear=wall_margin, group=f'B2_{rep}_{way}'))
                S.append(_brake(f'Brake after B2 {tag}'))
                S.append(_rest(1.0))

    # ── Block 3: 等速直行 + 急煞 ────────────────────────────────────
    if 3 in blocks:
        k = 0
        for rep in range(CRUISE_REPEATS):
            for frac in CRUISE_LEVELS:
                lx, ly, lh = launch[k % 2]      # 兩端交替, 兩個方向都取樣
                k += 1
                w = round(snap_rpm(frac * wspin, rpm_step), 3)
                tag = f'w={w:g} rep{rep + 1}'
                gid = f'B3_{rep}_{w:g}'
                S.append(_repos(f'Reposition -> launch (B3 {tag})',
                                lx, ly, lh, repos_timeout))
                S.append(_const(f'B3 cruise {tag}', 'B3_cruise', w, 0.0,
                                w / WHEEL_ACCEL_LIMIT + CRUISE_HOLD,
                                min_clear=0.4, min_ahead=1.0, group=gid,
                                min_useful=1.0, from_rest=True))
                S.append(_instant(f'B3 brake {tag}', 'B3_brake', 0.0, 0.0,
                                  BRAKE_DURATION, min_clear=0.0, group=gid))
                S.append(_brake(f'Brake after B3 {tag}'))
                S.append(_rest(0.5))

    # ── Block 4: 起步 (輪速階躍) ────────────────────────────────────
    if 4 in blocks:
        for rep in range(LAUNCH_REPEATS):
            lx, ly, lh = launch[rep % 2]
            tag = f'rep{rep + 1}'
            gid = f'B4_{rep}'
            S.append(_repos(f'Reposition -> launch (B4 {tag})',
                            lx, ly, lh, repos_timeout))
            S.append(_instant(f'B4 launch {tag}', 'B4_launch', wmax, 0.0,
                              LAUNCH_DURATION, min_clear=0.4, min_ahead=1.0,
                              group=gid, min_useful=0.3, from_rest=True))
            S.append(_instant(f'B4 brake {tag}', 'B4_brake', 0.0, 0.0,
                              BRAKE_DURATION, min_clear=0.0, group=gid))
            S.append(_brake(f'Brake after B4 {tag}'))
            S.append(_rest(0.5))

    S.append(_rest(2.0))
    return S


# ═══════════════════════════════════════════════════════════════════════
class ControlCarNode(Node):

    def __init__(self):
        super().__init__('control_car_node')

        p = self.declare_parameter
        # 房間內牆範圍 (car.usd 的 /World/Room: x in [-5,5], y in [-3,3])
        # 車子**實際到得了**的那一間的內牆範圍, 不是 car.usd 整張圖。
        # 量法: 對 car_usd.npz 的距離場, 從車子出生點 (0,0) 對「淨空 > 0.4 m」
        # 的格子做連通區域填充 -> x -4.58..4.58, y -2.58..2.58。
        # car.usd 整張圖是 x -7.1..6.55, y -4.5..4.7, 但 y=3.0 那裡有一道牆,
        # 車子出生的那一間到不了外面。
        #
        # 原本的預設 (-5, 5, -3, 3) 每邊大約多了 0.4 m, 後果是:
        #   * block 3/4 的起跑點 (x_min+1.5 = -3.5) 只剩 0.48 m 淨空
        #   * 四個角落有三個的淨空是 0.03 m —— 也就是在牆上
        #   * 幾何圍籬拿這個框算 wall_clearance, 車子以為還有空間就撞上去 -> abort
        # 實測一趟 block 3/4: 78 秒裡 GT 只走了 6.7 m, 其餘都在
        # 「reposition 逾時 -> abort -> retry」的迴圈裡。
        p('room_x_min', -4.58)
        p('room_x_max', 4.58)
        p('room_y_min', -2.58)
        p('room_y_max', 2.58)
        # 離牆的**靜態**餘裕。實際門檻是 wall_margin + 當下速度的煞停距離
        # (見 _violation) —— 舊版是固定 1.5 m, 兩頭都不對:
        #   * 高速時不夠: 自旋段實測車體會以 3.4 m/s 橫向滑出去, 煞停距離
        #     1.9 m, 等剩 1.5 m 才中止已經來不及。
        #   * 低速時太多: 房間半寬只有 2.58 m, 1.5 m 的死區讓車子幾乎不能離開
        #     中心 —— run2 的 block 1/2 有 14 個情境是在**幾乎停著**的時候被
        #     「離牆只剩 1.48 m」中止的。
        p('wall_margin', 0.6)
        # ---- 狀態來源 (見檔頭「狀態從哪裡來」) ----
        p('state_source', 'sensor')           # sensor | gt
        # 位姿 topic。空字串 = 依 state_source: sensor -> /fusion_loc/odom, gt -> /odom
        p('odom_topic', '')
        p('imu_topic', '/imu')
        # 模擬器裡拿來對答案的 GT (只印誤差, 不參與控制)。真車上不存在就自動沒事。
        p('gt_check_topic', '/odom')
        p('pose_timeout', 0.3)                # s, 位姿多久沒更新算斷線
        p('imu_timeout', 0.2)                 # s
        # 位置 1-sigma (m) 超過這個就不信 -> 量測段中止。融合在絕對量測掉線時靠
        # IMU+輪速遞推, sigma 會慢慢長大; 自旋幾秒是撐得住的, 所以門檻給寬。
        p('pose_sigma_max', 0.5)
        p('repos_sigma_max', 0.10)            # 歸位前要等 sigma 降到這裡
        p('sigma_margin_k', 3.0)              # 牆邊餘裕 += k * sigma
        # ---- 車體 / 馬達 ----
        p('joint_command_topic', '/joint_command')
        # 輪速指令上限 (rad/s)。B4 起步的輪速階躍就是跳到這個值, 歸位的車速也由它
        # 決定。預設 10 = 實體車目前的 max_rpm 100 (10.47); USD 的上限是 200 rpm。
        # **要比較的兩個環境, 這個值與 spin_wheel_speed 都必須一樣。**
        p('max_wheel_speed', 10.0)
        # B1 / B2 自旋與 B3 直行用的輪速 (rad/s), 見檔頭 SPIN_WHEEL_SPEED。分析報
        # 「馬達飽和」就把它調低。
        p('spin_wheel_speed', SPIN_WHEEL_SPEED)
        # 發出去的輪速指令量化成幾 rpm 的整數倍 (實體車的馬達只收整數 rpm)。0 = 不量化。
        p('command_rpm_step', 1.0)
        p('wheel_accel_limit', WHEEL_ACCEL_LIMIT)     # rad/s^2
        p('wheel_radius', WHEEL_RADIUS)
        # 歸位時把「要轉多快」換成左右輪速差用的等效輪距 (見檔頭 EFFECTIVE_TRACK)。
        # 只影響閉迴路歸位的收斂速度, 量測段不用它。
        p('effective_track', EFFECTIVE_TRACK)
        # IMU 在 base_link 裡的位置 [x, y] (m)。不在旋轉中心的話自旋時會量到向心
        # 加速度 w^2 * r, 這裡把它從前向加速度裡扣掉 (geofence 的速度積分用)。
        p('imu_lever_arm', [0.125, -0.0286])
        # **Isaac 的 IMU 不模擬安裝位置**: 它回報的是車體原點的加速度, 不是 imu_link
        # 那一點的。2026-10-13 用三輪資料對 ground truth 擬合「IMU 等效位置」都是
        # (+0.006~0.008, -0.001) m, 不是 USD 裡的 (0.125, -0.0286) —— 模擬裡照實際
        # 位置去扣, 等於憑空加進 w^2 * 0.125 的假加速度 (1.5 rad/s 自旋 0.28 m/s^2)。
        # 所以 use_sim_time 開著時預設不補償; 實體車 (use_sim_time:=false) 照常補。
        # Isaac 哪天改成有模擬了, 把這一項設成 true。
        p('imu_lever_arm_in_sim', False)
        # 車體轉速的安全上限 (rad/s), 超過 1.5 倍就中止。0 = 自動: 輪子完全不滑時
        # 的運動學上限 r * max_wheel_speed / 半輪距 (實際只會到它的一半左右)。
        p('spin_max_w', 0.0)
        p('calib_duration', 2.0)              # 開跑前靜止量零偏的秒數
        # 想只跑其中幾個 block:
        #   ros2 run calibrate_env_pkg calibrate_env_node --ros-args -p blocks:="[1,2]"
        p('blocks', [1, 2, 3, 4])
        p('retry_limit', 1)           # 每個情境被 abort 之後最多重試幾次
        # 橫越房間 (5.8 m) + 轉半圈實測 17~22 s, 25 s 只剩不到 4 s 的餘裕
        p('reposition_timeout', 35.0)
        # 車頭方向相對 base_link 的角度。kros_car.usd 照 REP-103 (+X 是車頭) 所以是 0;
        # 舊的 car.usd 車頭是 -Y, 要給 -90。
        p('forward_axis_deg', 0.0)
        # 中止時估得出來的減速度 (m/s^2), 用來算 geofence 的煞停距離。輪速指令歸零時
        # 是 min(扭矩上限 4*2.0/(r*m), 抓地力上限 mu*g) —— 3.0 是兩者的保守下界。
        p('brake_decel', 3.0)

        g = self.get_parameter
        self.room = (float(g('room_x_min').value), float(g('room_x_max').value),
                     float(g('room_y_min').value), float(g('room_y_max').value))
        self.wall_margin = float(g('wall_margin').value)
        self.retry_limit = int(g('retry_limit').value)
        self.fwd_off = math.radians(float(g('forward_axis_deg').value))
        self.brake_decel = max(float(g('brake_decel').value), 0.1)
        blocks = tuple(int(b) for b in g('blocks').value)

        self.source = str(g('state_source').value).strip().lower()
        if self.source not in ('sensor', 'gt'):
            raise ValueError(f'state_source 要是 sensor 或 gt, 收到 {self.source!r}')
        self.sensor = self.source == 'sensor'
        odom_topic = g('odom_topic').value or ('/fusion_loc/odom' if self.sensor else '/odom')
        self.pose_timeout = float(g('pose_timeout').value)
        self.imu_timeout = float(g('imu_timeout').value)
        self.pose_sigma_max = float(g('pose_sigma_max').value)
        self.repos_sigma_max = float(g('repos_sigma_max').value)
        self.sigma_k = float(g('sigma_margin_k').value)
        self.wmax = max(float(g('max_wheel_speed').value), 0.1)
        self.wspin = min(max(float(g('spin_wheel_speed').value), 0.1), self.wmax)
        self.rpm_step = max(float(g('command_rpm_step').value), 0.0)
        self.wheel_acc = max(float(g('wheel_accel_limit').value), 0.1)
        self.wheel_r = float(g('wheel_radius').value)
        self.eff_track = float(g('effective_track').value)
        la = [float(v) for v in g('imu_lever_arm').value] + [0.0, 0.0]
        if g('use_sim_time').value and not g('imu_lever_arm_in_sim').value:
            la = [0.0, 0.0]          # Isaac 的 IMU 不模擬安裝位置, 見上面的參數說明
        # IMU 位置在「車頭方向」上的分量: 自旋時前向加速度會多出 -w^2 * 這個值
        self.lever_fwd = la[0] * math.cos(self.fwd_off) + la[1] * math.sin(self.fwd_off)
        self.spin_max_w = float(g('spin_max_w').value) or (
            self.wheel_r * self.wmax / HALF_TRACK)

        self.cmd_pub = self.create_publisher(
            JointState, g('joint_command_topic').value, 10)
        self.scenario_pub = self.create_publisher(String, '/test_scenario', 10)
        # phase 是給 collect_data_node 標 CSV 用的: 分析時只取 'measure'。
        self.phase_pub = self.create_publisher(String, '/test_phase', 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, ODOM_QOS)
        if self.sensor:
            self.create_subscription(Imu, g('imu_topic').value, self.on_imu, ODOM_QOS)
            gt_topic = g('gt_check_topic').value
            if gt_topic and gt_topic != odom_topic:
                self.create_subscription(Odometry, gt_topic, self.on_gt, ODOM_QOS)

        # 50 Hz: 段落標籤的時間解析度 20 ms; 實體車 0.3 秒沒收到指令就停,
        # 這個頻率下掉幾包也不會觸發。
        self.hz = 50.0
        self.dt = 1.0 / self.hz

        # 位姿 (geofence 與 reposition 用)。世界速度用位置差分算, 避免去猜
        # Isaac 的 twist 是車體座標還是世界座標 —— 猜錯閉迴路會直接發散。
        # 差分用訊息**自己的時戳**, 不是收到的時間: 感測器鏈有延遲而且到達時間會
        # 抖, 用收到時間差分會出現尖峰。
        self.pos = None
        self.yaw = 0.0
        self.wz = 0.0
        self.vel_w = (0.0, 0.0)
        self.v_fused = None          # 融合的前向速度 (twist.linear.x); gt 模式不用
        self.pose_sigma = 0.0
        self._prev_pos = None
        self._prev_t = None
        self.t_odom = -1e9

        # IMU (sensor 模式)。零偏 / 加速度基準在 calibrate 段量, 之後每次靜止時更新。
        self.t_imu = -1e9
        self._imu_last_stamp = None
        self.gyro_bias = 0.0
        self.acc_ref = None          # 靜止時的 (ax, ay), 扣掉之後就是運動加速度
        self.v_imu = 0.0             # 前向加速度從最近一次靜止開始的積分
        self._imu_win = deque()      # (stamp, gz, ax, ay) 最近 IMU_WIN 秒
        self.calibrated = not self.sensor
        self._collecting = False
        self._calib_buf = []
        self._calib_ready_t = None
        self._loc_wait_since = None

        # 定位一致性檢查用的歷史: (stamp, 展開後的位姿 yaw) 與 (stamp, gyro 積分)
        self._pose_hist = deque()
        self._gyro_hist = deque()
        self._gyro_int = 0.0
        self._loc_bad_since = None   # 判定「定位跟 gyro 不一致」的時刻 (node clock)
        self._loc_bad_why = ''

        self.gt = None               # (x, y, wz) 只拿來印誤差
        self._gt_err = []

        self.current_cmd = [0.0, 0.0, 0.0, 0.0]      # 四輪輪速指令 (rad/s)
        self.finished = False
        self.aborted = []
        self.truncated = []      # 踩到圍籬但資料還是留著的段落

        self.scenarios = build_scenarios(
            self.room, float(g('reposition_timeout').value),
            self.wall_margin, blocks, max_wheel_speed=self.wmax,
            calib_duration=float(g('calib_duration').value),
            spin_wheel_speed=self.wspin, rpm_step=self.rpm_step)
        # trial 的原始段落 (重試時整組重放) 與各自的重試次數
        self.trials = {}
        for sc in self.scenarios:
            if sc.get('group') is not None:
                self.trials.setdefault(sc['group'], []).append(dict(sc))
        self._group_retries = {}
        self.idx = 0
        self.start_time = self._now()
        self.scenario_start = self.start_time
        # use_sim_time 時, /clock 的第一則還沒到之前 _now() 是 0; 到了之後直接跳到
        # 「模擬已經跑了多久」。逾時如果從 0 起算, Isaac 按 Play 超過 60 秒才啟動這個
        # 節點就會在第一個週期判定「等了 N 秒狀態還是不齊」直接放棄整份腳本
        # (2026-10-13 實測: Play 了 1154 s 之後啟動, 0.04 秒就放棄)。
        # 所以起點等時鐘真的有值之後再定, 見 control_callback。
        self._clock_anchored = False
        self._repos_state = 'turn_to_bearing'
        self._i_w = 0.0

        est = sum(s['duration'] for s in self.scenarios
                  if s['type'] != 'reposition')
        spin_ws = [round(snap_rpm(f * self.wspin, self.rpm_step), 3) for f in SPIN_LEVELS]
        self.get_logger().info(
            f'測試腳本: {len(self.scenarios)} 個情境, block {list(blocks)}, '
            f'量測時間約 {est:.0f} s (不含 reposition)\n'
            f'  狀態來源 {self.source}: 位姿 {odom_topic}'
            + (f', 轉速/加速度 {g("imu_topic").value}' if self.sensor else '')
            + f'\n  輪速指令上限 {self.wmax:g} rad/s ({self.wmax * self.wheel_r:.2f} m/s), '
            f'自旋輪速 {spin_ws} rad/s'
            + (f' (指令量化成 {self.rpm_step:g} rpm 的整數倍)' if self.rpm_step > 0 else '') + '\n'
            f'  房間 x[{self.room[0]}, {self.room[1]}] y[{self.room[2]}, {self.room[3]}], '
            f'離牆 {self.wall_margin} m (再加上當下速度的煞停距離) 內提前結束情境\n'
            f'  分析時只取 phase == "measure" 的列')

        self.timer = self.create_timer(self.dt, self.control_callback)
        if self.sensor:
            self.create_timer(10.0, self._gt_report)

    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg):
        now = self._now()
        # 差分用訊息時戳; 發布端沒填時戳 (0) 才退回收到的時間
        t = _stamp(msg.header) or now
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._prev_pos is not None and self._prev_t is not None:
            dt = t - self._prev_t
            if 1e-3 < dt < 0.5:
                vx = (x - self._prev_pos[0]) / dt
                vy = (y - self._prev_pos[1]) / dt
                # 輕度平滑: 差分在 20 Hz 的模擬真值上很乾淨, 但仍會有量化台階;
                # 感測器位姿 (10~30 Hz 絕對量測 + 遞推) 更需要
                self.vel_w = (0.6 * self.vel_w[0] + 0.4 * vx,
                              0.6 * self.vel_w[1] + 0.4 * vy)
        if self._prev_t is None or t > self._prev_t:
            self._prev_pos = (x, y)
            self._prev_t = t
        self.pos = (x, y)
        self.yaw = _yaw_of(msg.pose.pose.orientation)
        self.t_odom = now
        if self.sensor:
            yaw = self.yaw
            if self._pose_hist:
                prev = self._pose_hist[-1][1]
                yaw = prev + _wrap_pi(yaw - prev)
            if not self._pose_hist or t > self._pose_hist[-1][0]:
                self._pose_hist.append((t, yaw))
            while self._pose_hist and t - self._pose_hist[0][0] > 3.0:
                self._pose_hist.popleft()
            cov = msg.pose.covariance
            s = cov[0] + cov[7]
            self.pose_sigma = math.sqrt(s) if s > 0.0 else 0.0
            # 融合節點的 twist.linear.x 是沿車頭的速度 (它自己有 forward_deg)
            self.v_fused = msg.twist.twist.linear.x
        else:
            self.wz = msg.twist.twist.angular.z

    def on_imu(self, msg):
        now = self._now()
        st = _stamp(msg.header) or now
        gz = msg.angular_velocity.z
        ax, ay = msg.linear_acceleration.x, msg.linear_acceleration.y
        dt = 0.0
        if self._imu_last_stamp is not None:
            dt = st - self._imu_last_stamp
            if not 0.0 < dt < 0.1:           # 亂序 / 掉包太久 -> 這一步不積分
                dt = 0.0
        self._imu_last_stamp = st
        self.t_imu = now

        self._imu_win.append((st, gz, ax, ay))
        while self._imu_win and st - self._imu_win[0][0] > IMU_STILL_WINDOW:
            self._imu_win.popleft()
        if self._collecting:
            self._calib_buf.append((gz, ax, ay))

        self.wz = gz - self.gyro_bias
        self._gyro_int += self.wz * dt
        self._gyro_hist.append((st, self._gyro_int))
        while self._gyro_hist and st - self._gyro_hist[0][0] > 3.0:
            self._gyro_hist.popleft()
        if self.acc_ref is not None and dt > 0.0:
            self.v_imu += self._fwd_acc(ax, ay, self.wz) * dt

        if self.calibrated and self._imu_still():
            self.v_imu = 0.0
            # 輪速指令是 0 時才更新零偏/基準: 有指令但卡住不動 (高 mu 的 ramp 前段)
            # 也是靜止, 但那時車體可能被扭矩壓得微微傾斜
            if all(c == 0.0 for c in self.current_cmd):
                n = len(self._imu_win)
                self.gyro_bias += 0.02 * (sum(w[1] for w in self._imu_win) / n - self.gyro_bias)
                mx = sum(w[2] for w in self._imu_win) / n
                my = sum(w[3] for w in self._imu_win) / n
                self.acc_ref = (self.acc_ref[0] + 0.02 * (mx - self.acc_ref[0]),
                                self.acc_ref[1] + 0.02 * (my - self.acc_ref[1]))

    def on_gt(self, msg):
        self.gt = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                   msg.twist.twist.angular.z)
        if self.pos is not None and self.odom_ok:
            self._gt_err.append((math.hypot(self.pos[0] - self.gt[0], self.pos[1] - self.gt[1]),
                                 abs(self.wz - self.gt[2])))

    def _gyro_at(self, t):
        """gyro 積分在時刻 t 的值 (線性內插); t 落在歷史之外回 None。"""
        h = self._gyro_hist
        if not h or t < h[0][0] or t > h[-1][0]:
            return None
        prev = h[0]
        for cur in h:
            if cur[0] >= t:
                if cur[0] == prev[0]:
                    return cur[1]
                k = (t - prev[0]) / (cur[0] - prev[0])
                return prev[1] + k * (cur[1] - prev[1])
            prev = cur
        return h[-1][1]

    def _check_loc_consistency(self):
        """位姿 yaw 的變化跟 gyro 積分對不對得上 (sensor 模式每個 tick 呼叫)。

        抓的是 sigma 抓不到的失效: 定位凍結、鎖到錯的解、時鐘錯位。只在車子有在轉
        的時候下結論; 判定不一致之後, 要等到一段「有在轉而且對得上」的窗才解除。"""
        ph = self._pose_hist
        if len(ph) < 2 or len(self._gyro_hist) < 2:
            return
        t1, y1 = ph[-1]
        old = None
        for t, y in ph:
            if t <= t1 - LOC_CHECK_WINDOW:
                old = (t, y)
            else:
                break
        if old is None:
            return
        t0, y0 = old
        g0, g1 = self._gyro_at(t0), self._gyro_at(t1)
        now = self._now()
        if g0 is None or g1 is None:
            gap = t1 - self._gyro_hist[-1][0]
            if abs(gap) > 1.0:
                self._set_loc_bad(now, f'位姿時戳跟 IMU 差 {gap:+.1f} s (時鐘不同步)')
            return
        dg, dp = g1 - g0, y1 - y0
        if abs(dg) < LOC_CHECK_MIN_TURN and abs(dp) < LOC_CHECK_MIN_TURN:
            return
        if abs(dp - dg) > LOC_CHECK_TOL:
            self._set_loc_bad(now, f'{LOC_CHECK_WINDOW:g} s 內位姿轉了 {math.degrees(dp):+.0f}°, '
                                   f'gyro 轉了 {math.degrees(dg):+.0f}°')
        elif self._loc_bad_since is not None:
            self.get_logger().info(
                f'定位跟 gyro 重新對上了 (失效 {now - self._loc_bad_since:.1f} s)')
            self._loc_bad_since = None

    def _set_loc_bad(self, now, why):
        if self._loc_bad_since is None:
            self._loc_bad_since = now
            self.get_logger().error(f'定位失效: {why} —— sigma {self.pose_sigma:.3f} 看不出來')
        self._loc_bad_why = why

    def _fwd_acc(self, ax, ay, wz=0.0):
        """base_link 原點的前向加速度 (扣掉靜止基準)。IMU 跟 base_link 同向。

        IMU 不在旋轉中心時, 車子一轉它就量到向心加速度 -w^2 * r; 前向分量是
        -w^2 * lever_fwd, 這裡補回去。不補的話自旋 4 秒 (1.7 rad/s) v_imu 會
        積出 -1.5 m/s 的假速度, geofence 的煞停距離跟著膨脹。切線項 (角加速度 x r)
        沒補: 它積分起來是 dw * r, 轉速回到 0 就自己抵消了。"""
        return (math.cos(self.fwd_off) * (ax - self.acc_ref[0])
                + math.sin(self.fwd_off) * (ay - self.acc_ref[1])
                + wz * wz * self.lever_fwd)

    def _imu_still(self):
        """靜止判定: 最近 IMU_STILL_WINDOW 秒的 gyro 與加速度都沒在動。"""
        win = self._imu_win
        if len(win) < 5 or win[-1][0] - win[0][0] < 0.8 * IMU_STILL_WINDOW:
            return False
        n = len(win)
        mg = sum(w[1] for w in win) / n
        if abs(mg - self.gyro_bias) > STILL_GYRO:
            return False
        mx = sum(w[2] for w in win) / n
        my = sum(w[3] for w in win) / n
        var = sum((w[2] - mx) ** 2 + (w[3] - my) ** 2 for w in win) / n
        if var > STILL_ACC_STD ** 2:
            return False
        return self.acc_ref is None or abs(self._fwd_acc(mx, my)) < STILL_ACC_MEAN

    @property
    def imu_ok(self):
        return (self._now() - self.t_imu) < self.imu_timeout

    @property
    def pose_fresh(self):
        return self.pos is not None and (self._now() - self.t_odom) < self.pose_timeout

    @property
    def odom_ok(self):
        """控制需要的狀態都在而且可信。sensor 模式 = 位姿新鮮 + sigma 夠小 + IMU 在。"""
        if not self.pose_fresh:
            return False
        if self.sensor:
            return self.imu_ok and self.pose_sigma <= self.pose_sigma_max
        return True

    def _heading(self):
        """車頭在世界座標的角度。"""
        return _wrap_pi(self.yaw + self.fwd_off)

    def _fwd_speed(self):
        """閉迴路 (煞車 / 歸位) 用的前向速度。sensor 模式用融合的 twist。"""
        if self.sensor and self.v_fused is not None:
            return self.v_fused
        h = self._heading()
        return self.vel_w[0] * math.cos(h) + self.vel_w[1] * math.sin(h)

    def _omni_speed(self):
        """geofence 用的全向速率。

        sensor 模式取「位姿差分」與「IMU 積分」的較大值, **不含**融合的 twist:
        那個速度吃輪速, 起步打滑時輪子空轉, 它會報 4 m/s 而車子實際 2 m/s ——
        假模擬器 E2E 實測, B4 在 mu=0.5 時 6 次全部被這個假速度的煞停距離中止。
        位姿差分本身有絕對量測 (相機 / LiDAR) 按著, 不會被打滑帶跑。"""
        s = math.hypot(self.vel_w[0], self.vel_w[1])
        if self.sensor:
            s = max(s, abs(self.v_imu))
        return s

    def _ahead_speed(self):
        """geofence 前方檢查用的前向速度 (同上, sensor 模式不用融合 twist)。"""
        h = self._heading()
        v_pose = self.vel_w[0] * math.cos(h) + self.vel_w[1] * math.sin(h)
        return max(v_pose, self.v_imu) if self.sensor else self._fwd_speed()

    def _clearance_margin(self):
        return self.sigma_k * self.pose_sigma if self.sensor else 0.0

    # ---------------------------------------------------------- geofence
    def _wall_clearance(self):
        x, y = self.pos
        return min(x - self.room[0], self.room[1] - x,
                   y - self.room[2], self.room[3] - y)

    def _ahead_clearance(self):
        """沿車頭方向到牆的距離 (ray-box)。"""
        x, y = self.pos
        h = self._heading()
        dx, dy = math.cos(h), math.sin(h)
        t = float('inf')
        for lo, hi, p0, d in ((self.room[0], self.room[1], x, dx),
                              (self.room[2], self.room[3], y, dy)):
            if d > 1e-6:
                t = min(t, (hi - p0) / d)
            elif d < -1e-6:
                t = min(t, (lo - p0) / d)
        return t

    def _violation(self, sc):
        """回傳中止原因, 沒問題就回 None。

        前方門檻是**動態**的: 靜態餘裕再加上目前速度的煞停距離 v^2/(2a)。
        固定門檻不是太鬆 (高速時來不及煞) 就是太緊 (低速時白白中止掉還有效的
        資料)。"""
        # 舊版這裡是「沒有 odom 就回 None」= 不檢查、開迴路繼續衝。GT 不會斷所以
        # 沒出過事; 感測器會 (自旋時 LiDAR 追丟、相機被擋), 必須當成違規處理。
        if not self.pose_fresh:
            return '定位中斷 (位姿過期)'
        if self.sensor and not self.imu_ok:
            return 'IMU 中斷'
        if self.sensor and self.pose_sigma > self.pose_sigma_max:
            return f'定位不可信 (sigma {self.pose_sigma:.2f} m > {self.pose_sigma_max:.2f})'
        if self.sensor and self._loc_bad_since is not None:
            return f'定位跟 gyro 不一致 ({self._loc_bad_why})'

        # 轉速防呆: 輪速是指令給的, 正常不會超過運動學上限; 超過代表車子在
        # 失控飄轉 (撞到東西、輪子離地) 或 IMU 壞了。
        if abs(self.wz) > 1.5 * self.spin_max_w:
            return f'轉速過快 wz={self.wz:.1f} rad/s, 有失控飄移風險'

        mc = float(sc.get('min_clear') or 0.0)
        if mc > 0.0:
            # 這裡用的是**全向**速率, 不是前向: 自旋段車體是橫著滑出去的,
            # 只看車頭方向的分量會漏掉大部分的動能。
            speed = float(self._omni_speed())
            loc = self._clearance_margin()
            need = mc + loc + speed * speed / (2.0 * self.brake_decel)
            clear = self._wall_clearance()
            if clear < need:
                return (f'離牆只剩 {clear:.2f} m (需要 {need:.2f} m '
                        f'= 餘裕 {mc:.2f} + 定位 {loc:.2f} + {speed:.2f} m/s 的煞停距離 '
                        f'{need - mc - loc:.2f})')

        ma = sc.get('min_ahead')
        if ma is not None:
            v = max(self._ahead_speed(), 0.0)
            stop_dist = v * v / (2.0 * self.brake_decel)
            need = float(ma) + self._clearance_margin() + stop_dist
            ahead = self._ahead_clearance()
            if ahead < need:
                return (f'前方只剩 {ahead:.2f} m (需要 {need:.2f} m '
                        f'= 餘裕 {float(ma):.1f} + {v:.2f} m/s 的煞停距離 '
                        f'{stop_dist:.2f})')
        return None

    # ------------------------------------------------------- 情境流程控制
    def _enter(self, idx):
        self.idx = idx
        self.scenario_start = self._now()
        self._repos_state = 'turn_to_bearing'
        self._i_w = 0.0
        self._loc_wait_since = None
        if idx < len(self.scenarios) and self.scenarios[idx].get('from_rest'):
            # 這一段從靜止開始 (前面是歸位 settle, 已確認停住) -> IMU 速度積分歸零。
            # 不等靜止偵測自己歸零: settle 剛結束時 0.3 s 的窗還沒安靜下來。
            self.v_imu = 0.0
        name = (self.scenarios[idx]['name'] if idx < len(self.scenarios)
                else 'Finished')
        self.get_logger().info(f'[{idx + 1}/{len(self.scenarios)}] -> {name}')

    def _abort(self, reason):
        sc = self.scenarios[self.idx]
        gid = sc.get('group')
        self.aborted.append(sc['name'])
        self.get_logger().warn(f'中止「{sc["name"]}」: {reason}')

        # 同一個 trial 剩下的段落全部丟掉。留著會讓急煞段在車子已經被煞停之後
        # 才執行, 記到一整段速度為 0 的假資料 —— 比沒有資料更糟。
        j = self.idx + 1
        while j < len(self.scenarios) and self.scenarios[j].get('group') == gid \
                and gid is not None:
            del self.scenarios[j]

        home = self._home_for(self.idx)
        insert = [_brake(f'Brake (abort: {sc["name"]})', phase='aborted'),
                  _repos(f'Reposition (after abort: {sc["name"]})', *home,
                         float(self.get_parameter('reposition_timeout').value))]

        # 重試是以 **trial** 為單位, 不是以段落為單位: 急煞段的初始速度是前面
        # 那個直行段給的, 單獨重跑一個急煞只會得到一條全 0 的軌跡。
        key = gid if gid is not None else sc['name']
        tries = self._group_retries.get(key, 0)
        if tries < self.retry_limit:
            self._group_retries[key] = tries + 1
            members = self.trials.get(gid, [dict(sc)]) if gid else [dict(sc)]
            for m in members:
                r = dict(m)
                r['name'] = f'{m["name"]} retry{tries + 1}'
                insert.append(r)
        else:
            self.get_logger().warn(f'「{key}」已達重試上限, 跳過整個 trial')
        self.scenarios[self.idx + 1:self.idx + 1] = insert

        self._enter(self.idx + 1)

    def _home_for(self, idx):
        """往前找最近的一個 reposition, 當作這個情境的起跑點。"""
        for j in range(idx - 1, -1, -1):
            s = self.scenarios[j]
            if s['type'] == 'reposition':
                return (s['target_x'], s['target_y'],
                        math.degrees(s['target_heading']))
        cx = 0.5 * (self.room[0] + self.room[1])
        cy = 0.5 * (self.room[2] + self.room[3])
        return (cx, cy, 0.0)

    # ═══════════════════════════════════════════════════ MAIN CONTROL LOOP
    def control_callback(self):
        now = self._now()
        if not self._clock_anchored:
            if now <= 0.0:
                return                       # 模擬時鐘還沒來
            self._clock_anchored = True
            self.start_time = self.scenario_start = now
        elapsed = now - self.scenario_start
        if self.sensor:
            self._check_loc_consistency()

        if self.idx >= len(self.scenarios):
            if elapsed > 2.0 and not self.finished:
                n = len(self.aborted)
                self.get_logger().info(
                    f'所有測試情境已完成 ({n} 個被中止, '
                    f'{len(self.truncated)} 個提前收工但資料留著), 設定 finished 旗標。'
                    + (f'\n  被中止的: {self.aborted}' if n else '')
                    + (f'\n  提前收工的: {self.truncated}' if self.truncated else ''))
                self.finished = True
            self._publish('idle', 'Finished', [0.0] * 4)
            return

        sc = self.scenarios[self.idx]

        if sc['type'] == 'calibrate':
            self._run_calibrate(sc, now, elapsed)
            return

        if sc['type'] == 'brake':
            # 煞停: 輪速指令 0, 馬達的速度環自己出反向扭矩; 這裡只負責判斷停了沒。
            # sensor 模式: 轉速永遠用 gyro (定位斷了也有); 前向速度用融合的,
            # 融合過期就退回 IMU 積分。兩個都沒有就只能等固定時間。
            have_w = self.imu_ok if self.sensor else self.pose_fresh
            if not have_w:
                stopped = elapsed > 1.0
                dangerous = False
            else:
                if self.sensor:
                    v = self.v_fused if (self.pose_fresh and self.v_fused is not None) \
                        else self.v_imu
                else:
                    v = self._fwd_speed()
                w = self.wz
                stopped = abs(v) < 0.05 and abs(w) < 0.08
                if self.sensor:
                    # 融合的速度吃輪速: 車子在滑、輪子被煞住時它會說 0。
                    # 加上 IMU 靜止判定才算真的停了。
                    stopped = stopped and self._imu_still()
                dangerous = abs(v) > 0.3 or abs(w) > 1.0
            self._publish(sc['phase'], sc['name'], [0.0] * 4, rate_limited=False)
            # 固定 duration 是給正常情況 (通常 <1 s 就停) 的下限, 不是上限:
            # 車子還在滑就切下一段, 下一個情境的初始條件就不是靜止了。
            hard_cap = max(sc['duration'], 12.0) if dangerous else sc['duration']
            if stopped or elapsed > hard_cap:
                self._enter(self.idx + 1)
            return

        if sc['type'] == 'reposition':
            cmd, done = self._run_reposition(sc, elapsed)
            self._publish('reposition', sc['name'], cmd, rate_limited=False)
            if done or elapsed > sc['duration']:
                if not done:
                    self.get_logger().warn(f'{sc["name"]} 逾時, 直接進下一個情境')
                self._enter(self.idx + 1)
            return

        # ---- 量測段 (開迴路) ----
        bad = self._violation(sc)
        if bad is not None:
            # 已經收到夠長的一段就當「提早收工」: 資料留著, 不重試。
            # 截斷的量測段還是合法資料; 丟掉再重跑一次只會在同一個地方再撞一次。
            need = sc.get('min_useful')
            if need is not None and elapsed >= float(need):
                self.truncated.append(sc['name'])
                self.get_logger().info(
                    f'「{sc["name"]}」提前結束 ({bad}); 已收 {elapsed:.1f} s, '
                    '資料留著, 不重試。')
                self._enter(self.idx + 1)
            else:
                self._abort(bad)
            return

        if sc['type'] == 'ramp':
            frac = _clamp(elapsed / sc['duration'], 0.0, 1.0)
            val = sc['ramp_start'] + frac * (sc['ramp_end'] - sc['ramp_start'])
            throttle = val if sc['ramp_var'] == 'throttle' else sc['throttle']
            steer = val if sc['ramp_var'] == 'steer' else sc['steer']
        else:
            throttle, steer = sc['throttle'], sc['steer']

        self._publish(sc['phase'], sc['name'], self._mix(throttle, steer),
                      rate_limited=(sc['type'] != 'instant'))

        if elapsed > sc['duration']:
            self._enter(self.idx + 1)

    # -------------------------------------------------- 閉迴路歸位 (非量測)
    def _run_reposition(self, sc, elapsed):
        """把車開回起跑點並轉到指定朝向。回傳 (四輪輪速指令, done)。

        這一段是**唯一**的閉迴路, 而且標成 phase='reposition' —— 分析時要整段
        丟掉, 否則等於把「控制器補償摩擦力」的行為混進資料裡。
        """
        if self.sensor and not (self.pose_fresh and self.imu_ok
                                and self.pose_sigma <= self.repos_sigma_max):
            # 自旋完 LiDAR 可能鎖到對稱解或還在重新收斂。拿不可信的位置去開車,
            # 就是朝著牆以為是空地開過去。停著等它回來 (靜止時比較好重新鎖定)。
            now = self._now()
            if self._loc_wait_since is None:
                self._loc_wait_since = (now, elapsed)
            waited = now - self._loc_wait_since[0]
            self.scenario_start = now - self._loc_wait_since[1]    # 等待不算進歸位逾時
            self.get_logger().warn(
                f'{sc["name"]}: 等定位恢復 (sigma {self.pose_sigma:.3f} m, '
                f'位姿{"" if self.pose_fresh else "過期"}, 已等 {waited:.0f} s)',
                throttle_duration_sec=2.0)
            if waited > LOC_WAIT_MAX:
                self.get_logger().error(f'定位 {LOC_WAIT_MAX:.0f} s 沒恢復, 跳過這次歸位')
                return [0.0] * 4, True
            return [0.0] * 4, False
        self._loc_wait_since = None

        if self.sensor and self._loc_bad_since is not None:
            # 定位跟 gyro 對不上: 位置和朝向都不能信。原地轉是安全的 (位移 ~0),
            # 而且只有在轉的時候才驗證得了它有沒有恢復 -> 只准轉, 不准直線開。
            bad_for = self._now() - self._loc_bad_since
            if bad_for > LOC_WAIT_MAX:
                self.get_logger().error(
                    f'定位失效 {bad_for:.0f} s 沒恢復 ({self._loc_bad_why}), 放棄整份測試腳本 —— '
                    '在錯的位置繼續收只會得到錯的資料 (真車上就是撞牆)。'
                    '檢查定位節點的 log (時鐘偏移、追丟)。')
                del self.scenarios[self.idx + 1:]
                return [0.0] * 4, True
            self.get_logger().warn(f'{sc["name"]}: 定位跟 gyro 不一致, 只原地轉等它恢復 '
                                   f'({bad_for:.0f}/{LOC_WAIT_MAX:.0f} s)',
                                   throttle_duration_sec=2.0)
            if self._repos_state in ('drive', 'settle'):
                self._goto('turn_to_bearing')

        if not self.odom_ok:
            return [0.0] * 4, elapsed > 2.0      # 沒 odom 就只煞停一下帶過

        tx, ty = sc['target_x'], sc['target_y']
        dx, dy = tx - self.pos[0], ty - self.pos[1]
        dist = math.hypot(dx, dy)
        h = self._heading()

        if self._repos_state == 'turn_to_bearing':
            if dist < 0.20:
                self._goto('turn_to_heading')
                return [0.0] * 4, False
            err = _wrap_pi(math.atan2(dy, dx) - h)
            if abs(err) < 0.12:
                self._goto('drive')
            return self._drive(0.0, self._turn_rate(err)), False

        if self._repos_state == 'drive':
            if dist < 0.15:
                self._goto('turn_to_heading')
                return [0.0] * 4, False
            err = _wrap_pi(math.atan2(dy, dx) - h)
            if abs(err) > 0.6:                   # 偏太多就重新對準
                self._goto('turn_to_bearing')
                return [0.0] * 4, False
            # 歸位這一段沒有 geofence (它本來就是要開過房間), 所以速度上限
            # 直接綁在「剩下的距離煞不煞得住」上。
            # 留 20% 的輪速給轉向修正用。速度直接當指令送, 馬達的速度環自己會追。
            ahead = max(self._ahead_clearance() - 0.6, 0.0)
            v_cap = min(0.6, 0.8 * self.wmax * self.wheel_r,
                        math.sqrt(2.0 * self.brake_decel * ahead))
            v_des = _clamp(1.2 * dist, -v_cap, v_cap)
            return self._drive(v_des, self._turn_rate(err, kp=1.2)), False

        if self._repos_state == 'turn_to_heading':
            err = _wrap_pi(sc['target_heading'] - h)
            if abs(err) < 0.10 and abs(self.wz) < 0.30:
                self._goto('settle')
                return [0.0] * 4, False
            # 線速度指令 0 = drive 段殘餘的速度由馬達煞掉
            return self._drive(0.0, self._turn_rate(err)), False

        # settle: 維持朝向直到真的靜止。朝向跑掉太多就回去重轉 —— 直線段朝著側牆
        # 發車的話一起步就會被 geofence 中止。
        err = _wrap_pi(sc['target_heading'] - h)
        if dist > 0.5:                       # 被自己的慣性帶離目標點
            self._goto('turn_to_bearing')
            return [0.0] * 4, False
        if abs(err) > 0.20:                  # 轉過頭了
            self._goto('turn_to_heading')
            return [0.0] * 4, False
        v = self._fwd_speed()
        if abs(v) < 0.05 and abs(self.wz) < 0.08:
            return [0.0] * 4, True
        return self._drive(0.0, self._turn_rate(err)), False

    # ------------------------------------------------------ 開跑前靜止校正
    def _run_calibrate(self, sc, now, elapsed):
        """等位姿 (+ IMU) 都到齊, 再靜止 duration 秒量 gyro 零偏與加速度基準。

        融合 / IMU 定位節點自己也會做開機靜止校正, 但那是**它們的**零偏;
        這裡的 wz 與 v_imu 直接吃 /imu, 要自己量。等不到資料就整份腳本放棄 ——
        沒有定位還開迴路自旋, 在真車上就是撞牆。
        """
        self._publish('calibrate', sc['name'], [0.0] * 4, rate_limited=False)
        ready = self.pose_fresh and (not self.sensor or (
            self.imu_ok and self.pose_sigma <= self.repos_sigma_max))
        if not ready:
            self._collecting = False
            self._calib_buf = []
            self._calib_ready_t = None
            if elapsed > sc['timeout']:
                what = ('定位 ' + ('OK' if self.pose_fresh else '沒資料')
                        + (f' (sigma {self.pose_sigma:.3f})' if self.sensor else '')
                        + ('' if not self.sensor else
                           ', IMU ' + ('OK' if self.imu_ok else '沒資料')))
                self.get_logger().error(
                    f'等了 {elapsed:.0f} s 狀態還是不齊 ({what}), 放棄整份測試腳本。'
                    + ('檢查 localization 節點有沒有開、Isaac 有沒有按 Play。'))
                del self.scenarios[self.idx + 1:]
                self._enter(self.idx + 1)
            else:
                self.get_logger().info('等待定位 / IMU ...', throttle_duration_sec=5.0)
            return

        if self._calib_ready_t is None:
            self._calib_ready_t = now
            self._calib_buf = []
            self._collecting = self.sensor
            return
        if now - self._calib_ready_t < sc['duration']:
            return

        if self.sensor:
            buf = self._calib_buf
            if len(buf) < 10:
                self.get_logger().warn('靜止校正期間 IMU 樣本太少, 再等一輪')
                self._calib_ready_t = None
                return
            n = len(buf)
            mg = sum(b[0] for b in buf) / n
            gstd = math.sqrt(sum((b[0] - mg) ** 2 for b in buf) / n)
            if gstd > 3 * STILL_GYRO:
                self.get_logger().warn(f'靜止校正時 gyro 在抖 (std {gstd:.3f} rad/s), '
                                       '車子可能還在動, 重來')
                self._calib_ready_t = None
                return
            self.gyro_bias = mg
            self.acc_ref = (sum(b[1] for b in buf) / n, sum(b[2] for b in buf) / n)
            self.v_imu = 0.0
            self.calibrated = True
            self._collecting = False
            self.get_logger().info(
                f'靜止校正完成: gyro 零偏 {mg * 1e3:+.2f} mrad/s (std {gstd * 1e3:.2f}), '
                f'加速度基準 ({self.acc_ref[0]:+.3f}, {self.acc_ref[1]:+.3f}) m/s², '
                f'{n} 筆 IMU')
        self._enter(self.idx + 1)

    def _gt_report(self):
        """sensor 模式在模擬器裡跑時, 拿 GT 對答案 (只印, 不參與控制)。"""
        if not self._gt_err:
            return
        pe = sorted(e[0] for e in self._gt_err)
        we = sorted(e[1] for e in self._gt_err)
        k = int(0.95 * (len(pe) - 1))
        self.get_logger().info(
            f'[GT 對照] 位置誤差 中位 {pe[len(pe) // 2] * 100:.1f} cm / p95 {pe[k] * 100:.1f} cm, '
            f'轉速誤差 中位 {we[len(we) // 2]:.3f} / p95 {we[k]:.3f} rad/s '
            f'(sigma {self.pose_sigma:.3f} m, gyro 零偏 {self.gyro_bias * 1e3:+.2f} mrad/s)')
        if we[len(we) // 2] > 1.0:
            self.get_logger().warn('gyro 跟 GT 轉速對不上 —— IMU 軸向 / 正負號可能不對')
        self._gt_err = []

    def _goto(self, state):
        self._repos_state = state
        self._i_w = 0.0

    def _turn_rate(self, yaw_err, kp=2.0):
        """角度誤差 -> 要下的車體轉速指令 (rad/s)。

        目標轉速 = kp * 誤差; 再加一個小的積分項修正「指令轉速 vs 實際轉速」的
        落差: 輪速換成車體轉速用的是 effective_track, 它跟地面有關 (輪子滑多少),
        給得不準時純前饋會一直轉太慢或太快。積分讓收斂不靠它準不準。
        """
        w_des = _clamp(kp * yaw_err, -1.2, 1.2)
        e = w_des - self.wz
        self._i_w = _clamp(self._i_w + e * self.dt, -1.0, 1.0)
        return w_des + 0.8 * self._i_w

    def _drive(self, v, w):
        """車體速度 (m/s, rad/s) -> 四輪輪速指令 (rad/s)。閉迴路歸位用。"""
        half = 0.5 * self.eff_track * w
        return self._mix(v / self.wheel_r, half / self.wheel_r)

    # ------------------------------------------------------------------
    def _mix(self, throttle, steer):
        """[FL, FR, RL, RR] = [thr-steer, thr+steer, thr-steer, thr+steer] (rad/s)。

        steer > 0 = 右輪比較快 = 車體往左轉 (wz > 0)。超過 max_wheel_speed 時
        **等比縮放**而不是各自 clamp, 保住 throttle:steer 的比例 (各自 clamp 會讓
        直線速度吃掉轉向, 歸位時車子就轉不過來)。
        """
        left = throttle - steer
        right = throttle + steer
        peak = max(abs(left), abs(right))
        if peak > self.wmax:
            k = self.wmax / peak
            left, right = left * k, right * k
        return [left, right, left, right]

    def _publish(self, phase, name, target, rate_limited=True):
        if rate_limited:
            for i in range(4):
                self.current_cmd[i] = _rate_limit(
                    target[i], self.current_cmd[i], self.wheel_acc, self.dt)
        else:
            self.current_cmd = list(target)
        self._send_wheels(self.current_cmd)

        m = String()
        m.data = name
        self.scenario_pub.publish(m)
        m = String()
        m.data = phase
        self.phase_pub.publish(m)

    def _send_wheels(self, speeds):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        # 量化只做在送出去的值上; current_cmd (斜率限制的狀態) 保持連續, 不然斜率
        # 小於 1 rpm / 週期的 ramp 會卡在原地爬不上去。
        msg.velocity = [snap_rpm(v, self.rpm_step) for v in speeds]
        self.cmd_pub.publish(msg)

    def stop_car(self):
        """結束前補送幾次輪速 0。Isaac 的 articulation controller 會**一直維持最後
        一筆指令**, 不送的話車子會照最後的輪速繼續跑 (實體車有 0.3 s 逾時, 沒差)。"""
        self.current_cmd = [0.0] * 4
        try:
            for _ in range(5):
                self._send_wheels(self.current_cmd)
        except Exception:        # context 已經關掉了就算了
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ControlCarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
