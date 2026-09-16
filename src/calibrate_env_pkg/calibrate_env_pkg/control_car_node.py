#!/usr/bin/env python3
"""跨環境地面摩擦力識別用的開迴路測試腳本。

目標: 在不同 groundCollider 摩擦係數的場景裡送出**完全相同**的 effort 指令,
      讓 collect_data_node 記下車子的反應, 之後回歸出 mu。

為什麼是開迴路: 系統識別要看的是「同樣輸入 -> 不同輸出」。如果量測段用
cmd_vel 那種 PI 閉迴路, 控制器會主動補償掉摩擦力差異, 就什麼都量不到了。
所以**量測段一律開迴路**, 只有「把車開回起點」那一段才用閉迴路 (phase 標成
reposition, 分析時整段丟掉)。

────────────────────────────────────────────────────────────────────────
為什麼是這四個 block (而不是原本的 throttle x steer 矩陣掃描)
────────────────────────────────────────────────────────────────────────

1) 直線加速在高摩擦區間對 mu 完全不敏感。
   抓地力上限是 a_max = mu*g (mu=0.8 -> 7.85 m/s^2), 但這台車的扭矩上限只
   給得出 a = 0.34 * effort = 3.4 m/s^2 (見 car_teleop/README.md 的實測回歸)。
   只要 mu > 0.35, 車子就是**扭矩受限**而不是抓地力受限 —— 換地面, 加速度
   一模一樣。量到的是馬達, 不是地面。

2) 穩態量不出摩擦力。
   穩態時「驅動力 = 阻力」, 但驅動力你只知道 effort 指令, 中間隔著馬達模型、
   joint drive damping 與打滑, 解不出阻力。**斷油後驅動力 = 0, 減速度完全由
   摩擦決定** —— 所以每個量測段後面都接一個 instant cutoff 的 coast 段, 特徵
   取在 coast 段而不是 hold 段。

3) 原地旋轉是這台車對 mu 最敏感的操作。
   skid-steer 原地轉時四個輪子全部在橫向滑動, 阻力矩
       M_r = mu * m * g * (半軸距)
   正比於 mu, 而且跟轉速無關 (Coulomb)。斷油後
       dw/dt = -M_r / I_z = 常數   ->   w(t) 是直線, 斜率正比於 mu
   擬合那條直線的斜率就是 mu 特徵。而且原地轉位移 ~0, 在 10x6 的房間裡跑得完。

   (原本的腳本在這個房間裡第一個情境就撞牆: creep ramp 10 秒會走 57 m,
    pre-brake 的 effort 10 持續 4 秒會走 27 m, 房間只有 10 m 長。)

Block 1  旋轉 coast-down    量橫向滑動摩擦 = mu 主特徵      位移 ~0
Block 2  旋轉 creep ramp    量靜摩擦臨界                    位移 ~0
Block 3  直線短衝 + coast   量滾動阻力 (跟 mu 不同的物理量)  每趟 ~4 m
Block 4  起步打滑           量 slip ratio, 低 mu 才有訊號    每趟 ~2 m

Block 3/4 量到的滾動阻力與 slip ratio 跟 Block 1/2 的滑動摩擦是不同的物理量,
不同地面材質對它們的影響不會同比例 —— 兩組特徵都留著, 讓下游模型自己挑。

────────────────────────────────────────────────────────────────────────
跟舊版的介面差異
────────────────────────────────────────────────────────────────────────
* THROTTLE/STEER 直接寫 **effort 實值 (0~10)**, 不再有 _SCALE 這層換算。
  舊版 STEER_LEVELS 的 +-8 乘上 _SCALE=2 就是 +-16, 遠超過 MAX_EFFORT=10,
  被 _mix 各自 clamp 之後 (T=1,S=8)/(T=2,S=8)/(T=3,S=8) 塌成同一個指令
  (-10,+10) —— 25 格裡有 6 格完全重複, 另外 10 格有一側全飽和。
* _mix 超出上限時改成**等比縮放**而不是各自 clamp, 保住 throttle:steer 比例。
* 新增 /test_phase, collect_data_node 會把它記成 CSV 的 phase 欄位:
      measure     開迴路量測段 <- **分析時只取這個**
      reposition  閉迴路開回起跑點 (控制器會補償摩擦力, 不能用)
      brake       量測段之間的主動煞停 (讓下一段從靜止開始)
      aborted     撞牆保護中止的殘缺資料
      idle        腳本跑完
* 新增 geofence: 訂 /odom, 離牆太近就中止當前情境、標成 aborted、開回起點重試。

────────────────────────────────────────────────────────────────────────
狀態從哪裡來: state_source = sensor (預設) | gt
────────────────────────────────────────────────────────────────────────
真車沒有 ground truth, 所以預設就用感測器跑, 模擬器裡的 /odom 只拿來**對答案**
(gt_check_topic, 定期印誤差, 不參與控制)。每一個量用的來源刻意不一樣:

  量                  sensor 模式                      為什麼
  ─────────────────── ──────────────────────────────── ─────────────────────────
  位置 / 朝向         /fusion_loc/odom 的 pose         geofence 與歸位要絕對位置
  轉速 wz             /imu gyro z (扣開機零偏)          LiDAR 自旋 >8 rad/s 會追丟;
                                                       gyro 不受打滑、不會斷
  stop_at_speed       IMU 前向加速度從靜止積分          融合的速度吃輪速, 打滑時是
                                                       錯的, 而打滑就是摩擦造成的
                                                       -> coast 進入速度會跟著 mu
                                                       跑 (循環論證)。量測段只有
                                                       1~3 秒, 積分漂移可忽略。
  煞車 / 歸位的速度   /fusion_loc/odom twist.linear.x   閉迴路段, 準不準只影響收斂
  靜止判定            gyro + 加速度抖動 + 融合速度

安全上的差別: GT 不會斷, 感測器會。所以 sensor 模式下
  * 定位過期 (pose_timeout) 或 sigma 太大 (pose_sigma_max) -> 量測段直接中止、煞停,
    **不是**像舊版那樣「沒有 odom 就不檢查」繼續開迴路衝。
  * 牆邊餘裕再加 sigma_margin_k x 定位 sigma。
  * 歸位前先等定位回到可信 (repos_sigma_max) —— 自旋完 LiDAR 可能還沒重新鎖定。
  * 自旋目標轉速上限 spin_max_w 預設 10.0 rad/s (sensor 模式)。真車換算: LiDAR 在
    高轉速會追丟 (靠 IMU 撐過去), 而且要低於 IMU 的 gyro 量程 —— 常見的 ±250 dps
    只有 4.4 rad/s, 那種 IMU 要把 spin_max_w 調到 4 以下, 買之前/設定時就要確認。
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

MAX_EFFORT = 10.0

# ── Block 1: 旋轉 coast-down ──────────────────────────────────────────
# 安全轉速上限。實測 (car_run_data/sim_data.csv) 低 mu 時舊版 (效果直接用
# steer effort 3/5/7/10) 在 4 秒內就衝到 wz≈20~30 rad/s, 遠超過高 mu 下的
# 穩態假設 (mu=0.8, S=10 只到 16.4)。位移在自轉時 ~0, 幾何圍籬完全看不到
# 這個危險, 車子會一路轉到失控飄移、甚至撞牆。
SPIN_MAX_SAFE_W = 12.0                   # rad/s   (gt 模式; sensor 模式見 spin_max_w 參數)

# 四個目標初始角速度 (不是 effort!)。實際值由 spin_targets() 依 spin_max_w 縮放,
# spin_max_w = 12 時就是下面這組。用跟 B3/B4 的 stop_at_speed 同一招:
# 轉速觸發取代時間觸發, 用固定的 SPIN_UP_STEER 衝上去, 到了目標值就放開。
# 舊版是固定 4 個 steer effort, 在低 mu 下沒有阻力矩制衡, S=5/7/10 全部會
# 衝破安全上限被同一個 SPIN_MAX_SAFE_W 拍平成幾乎一樣的 w0 (實測只剩
# ~1.5 跟 ~13 兩檔), 驗證「衰減斜率跟初始 w 無關」的 Coulomb 假設就沒意義
# 了。改成直接指定目標轉速, 在任何 mu 下都是真正分開的初始條件；
# 全部低於 SPIN_MAX_SAFE_W, 觸發不到安全上限。
#
# **只取高轉速的三檔 (80% ~ 100%), 每檔 5 次。** 舊版 [2, 5, 8, 11] 的最低檔在
# 高 mu 地面整段 coast 只有 ~50 ms (60 Hz IMU 3 個點), 幾乎全是放開後的過渡段,
# 減速度系統性偏低 (2026-09-15 校正: mu 1.5 時 32 vs 其他檔 50), 分析端只好丟掉
# —— 12 次裡能用的剩一半。總次數不變, 全部集中在量得準的區間。
#
# 2026-09-16: coast 段的角減速度是**用 5~6 個 IMU 點擬合一條直線**量出來的
# (mu 2.0 時整段 coast 只有 0.14 s = 8 個點), 擬合本身的標準誤就有 2.4%,
# 佔了 trial 間變異的 70~170% —— 也就是「雜訊極限」的真正來源。解法只有兩個:
#   1) 讓 coast 更長 -> w0 更高 (擬合誤差 ~ 1/span^1.5, 所以最有效)
#   2) 更多 trial (誤差 ~ 1/sqrt(n))
# 所以 w0 往上推到安全上限, 每檔次數 4 -> 5 (共 15 次)。
SPIN_TARGET_W = [9.12, 10.26, 11.4]      # rad/s (spin_max_w = 12 時)
SPIN_UP_STEER = MAX_EFFORT               # 固定用最大 steer 衝, 只有目標轉速是變數
# rate limiter 是 6 effort/s, 所以 1.67 s 就踩到底; 再給餘裕讓轉速長到目標值。
# 高 mu 時可能衝不到某些高檔目標 (穩態轉速比目標低), 這裡當逾時保護, 跟
# SPRINT_ACCEL_TIMEOUT 是同一個角色。
SPIN_UP_TIMEOUT = 5.0
# coast 只需要跑到轉速掉進尾段: mu 0.5 (最慢) 從 11 rad/s 掉到 0 也只要 0.45 s。
# 舊版的 3.0 s 有 2.5 s 是空等, 15 次就浪費 40 s。
SPIN_COAST_DURATION = 1.5                # ← 特徵取在這一段
SPIN_REPEATS = 5

# ── Block 2: 旋轉 creep (靜摩擦臨界) ─────────────────────────────────
# 舊版的 creep 是直線的, 10 秒會走 57 m。改成原地轉, 一樣量靜摩擦但不會跑掉。
CREEP_STEER_START, CREEP_STEER_END = 0.0, 10.0
CREEP_DURATION = 10.0
# 靜摩擦的「每次起轉的臨界 effort」本身就有 ~8% 的物理散佈 (每次歸位後四輪的
# 預載不同, 先打滑的輪子也不同), 不是量測雜訊, 只能靠次數平均掉:
# 中位數的標準誤 ≈ 1.2533 x 8% / sqrt(n), n=3 -> 5.8% (mu ±0.14), n=6 -> 4.1%。
# 另外 B1 spin_up 的「從靜止到 w_ref 的時間」也是靜摩擦特徵 (見 estimate_friction
# 的 spinup_time), 那個每輪有 15 次、cv 只有 0.7%, 是靜摩擦的主力;
# B2 留著當**不依賴馬達模型**的交叉檢查。
CREEP_REPEATS = 6

# ── Block 3: 直線短衝 + coast (滾動阻力) ─────────────────────────────
# 加速段刻意只有 1 秒: effort 10 時 a=3.4 m/s^2, 1 秒走 1.7 m, 進入 coast
# 時 3.4 m/s。coast 距離在低 mu 時會拉很長, 所以靠 geofence 保護而不是算剛好。
#
# **加速段改成速度觸發, 不是時間觸發。** 舊版是「effort T 固定踩 2.0 秒」,
# 而實測 (run1+run2, 12 個 trial) 是:
#     T=4  -> 2.0 s 後 3.85 m/s     T=7 -> 1.6 s 後 4.8 m/s
#     T=10 -> 1.6 s 後 4.9 m/s      (T=7 與 T=10 幾乎一樣, 扭矩已經飽和)
# 房間長 9.16 m, 起跑點離對面牆 8.4 m; 4 m/s 的煞停距離就 2.7 m, 再加上
# min_ahead 2.5 m 的餘裕 = 5.2 m, 所以車子跑到 3.2 m 就一定違規。
# 結果是 **12 個 sprint trial 全部中止, 一次 coast 資料都沒收到** ——
# run1/run2 的 `B3 sprint_coast` 是 0 筆。
#
# 用「加到指定車速就放開」取代「踩固定時間」有三個好處:
#   1) coast 進入速度是**指定的**, 不是被地面摩擦係數決定的 -> 跨環境可比。
#   2) 房間放得下: 2.6 m/s 的煞停距離只有 1.1 m。
#   3) 油門值不再是自變數 (T=7 和 T=10 本來就給不出不同的答案), 真正的自變數
#      是 coast 進入速度, 而那正是擬合滾動阻力要掃的東西。
SPRINT_THROTTLE = 7.0                    # 加速用的固定油門
SPRINT_TARGET_SPEEDS = [1.0, 1.8, 2.6]   # coast 進入速度 (m/s)
SPRINT_ACCEL_TIMEOUT = 3.0               # 加不到目標速度就放棄 (低 mu 地面)
SPRINT_COAST_DURATION = 3.0
# B3 只是對照組 (應該 ≈ 1), 3 個 trial 就夠看出「不只摩擦不一樣」, 省下的
# ~60 s 拿去給 B2 的靜摩擦次數。
SPRINT_REPEATS = 1

# ── Block 4: 起步打滑 (traction limit) ───────────────────────────────
# 跟 Block 3 的 T=10 差在**跳過 rate limiter**: 瞬間給滿扭矩才會突破抓地力上限。
# slip ratio = (r*w_wheel - v_true) / (r*w_wheel), 兩邊 collect_data_node 都有記。
# mu < 0.35 時訊號很強, mu 高時會趨近 0 —— 趨近 0 本身就是有用的資訊。
SLIP_THROTTLE = 10.0
# 打滑訊號集中在放開煞車後的前 0.2~0.3 秒 (輪子轉起來、車還沒動)。1.0 秒會
# 衝到 4.5 m/s, 煞停距離 3.4 m, 房間放不下 —— 實測 6 個 trial 的 slip_coast
# 全部中止。改成速度觸發, 2.0 m/s 就放開, 打滑段一樣完整。
SLIP_LAUNCH_DURATION = 1.0               # 上限, 正常會被 SLIP_TARGET_SPEED 先結束
SLIP_TARGET_SPEED = 2.0
SLIP_COAST_DURATION = 1.5
SLIP_REPEATS = 2                         # 副特徵, 2 次即可 (省時間給 B1/B2)

REST_DURATION = 1.0
RATE_LIMIT = 6.0        # effort/s, 模擬人踩油門的急促程度 (instant 型會略過)

# 「到了目標值就放開」的觸發延遲補償 (s)。2026-09-15 的資料: 目標 5.16 / 6.02 /
# 6.88 rad/s 三檔, 放開時的實際轉速全都是 7~10 rad/s —— 因為從 wz 越過目標到
# Isaac 真的斷油中間有 ~0.12 s (控制迴圈 20 Hz + topic 傳輸 + 指令生效),
# 而自旋末段的角加速度有 25 rad/s^2, 0.12 s 就是 +3 rad/s。三檔被糊成一檔,
# w0 還在 6.5~10.2 之間亂跳。解法: 用 gyro 算出當下的角加速度, 提前 LEAD 秒放開。
# 控制迴圈拉到 50 Hz 之後實測延遲剩 ~0.05 s。
TRIGGER_LEAD = 0.05

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


def spin_targets(spin_max_w):
    """B1 的三檔目標轉速: 上限的 80% / 90% / 100%, 最高檔再留 5% 給殘餘 overshoot。

    三檔不是為了掃描, 是為了驗證 Coulomb 假設 (減速度跟 w 無關) 並讓分析端能
    把「跟轉速有關的阻力」迴歸掉。範圍從舊版的 75~100% 收到 80~100%: 低檔的
    coast 短、擬合點少, 是雜訊的主要來源。"""
    top = 0.95 * spin_max_w
    return [round(top * f, 2) for f in (0.8, 0.9, 1.0)]


# ═══════════════════════════════════════════════════════════════════════
#  測試腳本
# ═══════════════════════════════════════════════════════════════════════
def _const(name, block, throttle, steer, duration, min_clear,
           min_ahead=None, group=None, stop_at_speed=None, min_useful=0.5,
           stop_at_wz=None):
    """group = 這一段屬於哪個 trial。中止重試時整個 trial 一起重跑 —— 只重跑
    coast 段是沒有意義的, 因為它的初始速度是前面那個加速段給的。

    stop_at_speed : 前向車速到這個值就提早結束 (m/s)。用來讓 coast 的進入速度
                    由**我們**決定, 而不是由地面摩擦係數決定。
    stop_at_wz : 跟 stop_at_speed 同一招, 但用在轉速上 —— 原地旋轉時位移 ~0,
                 幾何圍籬看不到失控加速, 低 mu 時 wz 會一路長到失控飄移甚至
                 撞牆 (見 SPIN_MAX_SAFE_W 的說明)。到了這個轉速就提早放開。
    min_useful : 幾何圍籬踩線時, 已經收滿這麼多秒就當作「提早收工」而不是
                 「中止」—— 資料留著, 不重試。截斷的 coast-down 仍然是完全
                 合法的資料 (少幾個點而已); 把它丟掉再重跑一次才是浪費。
                 實測 run2: 105 個情境裡 70% 的時間花在 abort -> reposition
                 -> retry 的迴圈上, 而重試通常在同一個地方再撞一次。
    """
    return {'name': name, 'block': block, 'type': 'const', 'phase': 'measure',
            'throttle': throttle, 'steer': steer, 'duration': duration,
            'min_clear': min_clear, 'min_ahead': min_ahead, 'group': group,
            'stop_at_speed': stop_at_speed, 'min_useful': min_useful,
            'stop_at_wz': stop_at_wz}


def _instant(name, block, throttle, steer, duration, min_clear,
             min_ahead=None, group=None, stop_at_speed=None, min_useful=0.5,
             stop_at_wz=None):
    """跳過 rate limiter 直接跳到目標值。coast 段 (throttle=0) 與打滑起步都用它。"""
    d = _const(name, block, throttle, steer, duration, min_clear, min_ahead,
               group, stop_at_speed, min_useful, stop_at_wz)
    d['type'] = 'instant'
    return d


def _ramp(name, block, var, start, end, duration, min_clear,
          min_ahead=None, group=None, stop_at_wz=None):
    return {'name': name, 'block': block, 'type': 'ramp', 'phase': 'measure',
            'ramp_var': var, 'ramp_start': start, 'ramp_end': end,
            'throttle': 0.0, 'steer': 0.0, 'duration': duration,
            'min_clear': min_clear, 'min_ahead': min_ahead, 'group': group,
            'stop_at_speed': None, 'min_useful': 0.5, 'stop_at_wz': stop_at_wz}


def _rest(duration=REST_DURATION):
    """組間的煞停。標成 measure 是刻意的 —— 靜止段可以拿來估感測器零偏。"""
    return _const('Rest', '-', 0.0, 0.0, duration, min_clear=0.0)


def _brake(name, phase='brake', duration=3.0):
    """主動煞停。**零 effort 不等於煞車** —— 這台車幾乎沒有滾動阻力, 低 mu 的
    地面放開油門還會滑好幾公尺, 所以要閉迴路給反向扭矩。

    每個 coast 量測段後面都接一個, 理由有二:
      1) 安全: coast 結束時車子還在動, 開迴路的 Rest 擋不住它滑進牆裡。
      2) 一致性: 下一個量測段要從**靜止**開始, 否則初始條件跟著上一段的殘餘
         速度走, 跨環境就不可比了 (低 mu 的環境殘餘速度大得多)。
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
                    spin_max_w=SPIN_MAX_SAFE_W, calib_duration=2.0):
    """room = (x_min, x_max, y_min, y_max) 的內牆範圍。

    min_clear = 離最近的牆至少要有幾公尺 (0 = 不檢查)
    min_ahead = 沿車頭方向至少要有幾公尺 (None = 不檢查); 實際門檻還會再加上
                目前速度的煞停距離, 見 _violation。
    """
    x_min, x_max, y_min, y_max = room
    cx, cy = 0.5 * (x_min + x_max), 0.5 * (y_min + y_max)
    # 沿長軸兩端的起跑點, 留 1.5 m 讓車子擺得下並且有反應空間
    launch = [(x_min + 1.5, cy, 0.0),        # 面向 +X
              (x_max - 1.5, cy, 180.0)]      # 面向 -X

    S = [_calibrate(calib_duration, timeout=60.0)]

    # ── Block 1: 旋轉 coast-down ─────────────────────────────────────
    if 1 in blocks:
        S.append(_repos('Reposition -> center (B1)', cx, cy, 0.0, repos_timeout))
        for rep in range(SPIN_REPEATS):
            for w0 in spin_targets(spin_max_w):
                tag = f'w0={w0:g} rep{rep + 1}'
                gid = f'B1_{rep}_{w0:g}'
                S.append(_const(f'B1 spin_up {tag}', 'B1_spin_up',
                                0.0, SPIN_UP_STEER, SPIN_UP_TIMEOUT,
                                min_clear=wall_margin, group=gid,
                                stop_at_wz=w0))
                S.append(_instant(f'B1 spin_coast {tag}', 'B1_spin_coast',
                                  0.0, 0.0, SPIN_COAST_DURATION,
                                  min_clear=wall_margin, group=gid))
                S.append(_brake(f'Brake after {tag}'))
                S.append(_rest())

    # ── Block 2: 旋轉 creep (靜摩擦臨界) ────────────────────────────
    if 2 in blocks:
        S.append(_repos('Reposition -> center (B2)', cx, cy, 0.0, repos_timeout))
        for rep in range(CREEP_REPEATS):
            S.append(_ramp(f'B2 creep_spin rep{rep + 1}', 'B2_creep_spin', 'steer',
                           CREEP_STEER_START, CREEP_STEER_END,
                           CREEP_DURATION, min_clear=wall_margin,
                           group=f'B2_{rep}', stop_at_wz=spin_max_w))
            S.append(_brake(f'Brake after B2 rep{rep + 1}'))
            S.append(_rest(1.5))

    # ── Block 3: 直線短衝 + coast ───────────────────────────────────
    if 3 in blocks:
        k = 0
        for rep in range(SPRINT_REPEATS):
            for vt in SPRINT_TARGET_SPEEDS:
                lx, ly, lh = launch[k % 2]      # 兩端交替, 兩個方向都取樣
                k += 1
                tag = f'v={vt:g} rep{rep + 1}'
                S.append(_repos(f'Reposition -> launch (B3 {tag})',
                                lx, ly, lh, repos_timeout))
                gid = f'B3_{rep}_{vt:g}'
                # 加速段: 到達目標車速就結束 (min_useful 給 0.2 是因為在高 mu
                # 的地面 1.0 m/s 只要 0.35 秒, 那樣短也是完整的資料)
                S.append(_const(f'B3 sprint_accel {tag}', 'B3_sprint_accel',
                                SPRINT_THROTTLE, 0.0, SPRINT_ACCEL_TIMEOUT,
                                min_clear=0.4, min_ahead=1.5, group=gid,
                                stop_at_speed=vt, min_useful=0.2))
                S.append(_instant(f'B3 sprint_coast {tag}', 'B3_sprint_coast',
                                  0.0, 0.0, SPRINT_COAST_DURATION,
                                  min_clear=0.4, min_ahead=0.8, group=gid,
                                  min_useful=0.4))
                S.append(_brake(f'Brake after B3 {tag}'))
                S.append(_rest(0.5))

    # ── Block 4: 起步打滑 ───────────────────────────────────────────
    if 4 in blocks:
        for rep in range(SLIP_REPEATS):
            lx, ly, lh = launch[rep % 2]
            tag = f'rep{rep + 1}'
            S.append(_repos(f'Reposition -> launch (B4 {tag})',
                            lx, ly, lh, repos_timeout))
            gid = f'B4_{rep}'
            S.append(_instant(f'B4 slip_launch {tag}', 'B4_slip_launch',
                              SLIP_THROTTLE, 0.0, SLIP_LAUNCH_DURATION,
                              min_clear=0.4, min_ahead=1.5, group=gid,
                              stop_at_speed=SLIP_TARGET_SPEED, min_useful=0.2))
            S.append(_instant(f'B4 slip_coast {tag}', 'B4_slip_coast',
                              0.0, 0.0, SLIP_COAST_DURATION,
                              min_clear=0.4, min_ahead=0.8, group=gid,
                              min_useful=0.4))
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
        # 自旋目標轉速上限 (rad/s)。0 = 依 state_source: sensor 10.0, gt 12.0
        p('spin_max_w', 0.0)
        p('calib_duration', 2.0)              # 開跑前靜止量零偏的秒數
        # 想只跑其中幾個 block:
        #   ros2 run calibrate_env_pkg calibrate_env_node --ros-args -p blocks:="[1,2]"
        p('blocks', [1, 2, 3, 4])
        p('retry_limit', 1)           # 每個情境被 abort 之後最多重試幾次
        p('reposition_timeout', 25.0)
        # 車頭方向相對 base_link 的角度。car.usd 的車頭是車體 -Y (不是 ROS 慣例
        # 的 +X), 所以是 -90 度。換車或把 USD 裡的車轉正之後要改這裡。
        p('forward_axis_deg', -90.0)
        # 中止時估得出來的減速度 (m/s^2)。扭矩上限 a=0.34*10=3.4, 抓地力上限是
        # mu*g —— 取 3.0 是兩者的保守下界, 用來算 geofence 的煞停距離。
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
        # sensor 模式 7.5 -> 10.0。理由是實測而不是猜的: 舊版因為觸發延遲,
        # 實際 w0 本來就已經衝到 8~10.2 rad/s (還差點碰到 1.5 x 7.5 = 11.25 的
        # 失控中止門檻), 而那一整輪的定位一致性檢查是 0% 異常、對 GT 的位置誤差
        # 中位 1.7 cm —— 也就是這個轉速區間實測是穩的, 只是以前是**失控地**穩。
        # 現在觸發準了, 就明確把上限設在 10.0 (目標 7.6 / 8.55 / 9.5):
        # coast 越長, 角減速度的擬合誤差越小 (~1/span^1.5), 這是動摩擦解析度的
        # 主要來源。中止門檻跟著變成 15 rad/s。
        self.spin_max_w = float(g('spin_max_w').value) or (10.0 if self.sensor else SPIN_MAX_SAFE_W)

        self.effort_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.scenario_pub = self.create_publisher(String, '/test_scenario', 10)
        # phase 是給 collect_data_node 標 CSV 用的: 分析時只取 'measure'。
        self.phase_pub = self.create_publisher(String, '/test_phase', 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, ODOM_QOS)
        if self.sensor:
            self.create_subscription(Imu, g('imu_topic').value, self.on_imu, ODOM_QOS)
            gt_topic = g('gt_check_topic').value
            if gt_topic and gt_topic != odom_topic:
                self.create_subscription(Odometry, gt_topic, self.on_gt, ODOM_QOS)

        # 50 Hz (舊版 20 Hz)。三個理由:
        #   1) 觸發延遲 50 ms -> 20 ms, 配合 TRIGGER_LEAD 才能讓 w0 落在目標上
        #   2) rate limiter 的 effort 階梯 6/20 = 0.3 -> 6/50 = 0.12, B1 spin_up
        #      起轉 effort 的量化誤差跟著從 5.9% 掉到 2.4% (靜摩擦特徵)
        #   3) 段落標籤的時間解析度 50 ms -> 20 ms (spinup_time 的起點)
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
        self.acc_fwd = 0.0           # 低通後的前向加速度 (m/s^2), 觸發提前量用
        self.wz_rate = 0.0           # 低通後的角加速度 (rad/s^2), 同上
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

        self.current_efforts = [0.0, 0.0, 0.0, 0.0]
        self.finished = False
        self.aborted = []
        self.truncated = []      # 踩到圍籬但資料還是留著的段落

        self.scenarios = build_scenarios(
            self.room, float(g('reposition_timeout').value),
            self.wall_margin, blocks, spin_max_w=self.spin_max_w,
            calib_duration=float(g('calib_duration').value))
        # trial 的原始段落 (重試時整組重放) 與各自的重試次數
        self.trials = {}
        for sc in self.scenarios:
            if sc.get('group') is not None:
                self.trials.setdefault(sc['group'], []).append(dict(sc))
        self._group_retries = {}
        self.idx = 0
        self.start_time = self._now()
        self.scenario_start = self.start_time
        self._repos_state = 'turn_to_bearing'
        self._i_v = 0.0
        self._i_w = 0.0

        est = sum(s['duration'] for s in self.scenarios
                  if s['type'] != 'reposition')
        self.get_logger().info(
            f'測試腳本: {len(self.scenarios)} 個情境, block {list(blocks)}, '
            f'量測時間約 {est:.0f} s (不含 reposition)\n'
            f'  狀態來源 {self.source}: 位姿 {odom_topic}'
            + (f', 轉速/加速度 {g("imu_topic").value}' if self.sensor else '')
            + f'; 自旋目標 {spin_targets(self.spin_max_w)} rad/s\n'
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

        prev_wz = self.wz
        self.wz = gz - self.gyro_bias
        # 角加速度 (低通): 觸發提前量用。自旋末段 ~25 rad/s^2, 乘上 TRIGGER_LEAD
        # 就是「現在放開的話, 實際會停在哪個轉速」的修正量。
        if dt > 0.0:
            a = (abs(self.wz) - abs(prev_wz)) / dt
            self.wz_rate += 0.3 * (a - self.wz_rate)
        self._gyro_int += self.wz * dt
        self._gyro_hist.append((st, self._gyro_int))
        while self._gyro_hist and st - self._gyro_hist[0][0] > 3.0:
            self._gyro_hist.popleft()
        if self.acc_ref is not None and dt > 0.0:
            acc = self._fwd_acc(ax, ay)
            self.v_imu += acc * dt
            self.acc_fwd += 0.3 * (acc - self.acc_fwd)

        if self.calibrated and self._imu_still():
            self.v_imu = 0.0
            # 沒在出力時才更新零偏/基準: 出力但卡住不動 (高 mu 的 creep 前段)
            # 也是靜止, 但那時車體可能被扭矩壓得微微傾斜
            if all(e == 0.0 for e in self.current_efforts):
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

    def _fwd_acc(self, ax, ay):
        """IMU 前向加速度 (扣掉靜止基準)。IMU 裝在車體上、跟 base_link 同向。"""
        return (math.cos(self.fwd_off) * (ax - self.acc_ref[0])
                + math.sin(self.fwd_off) * (ay - self.acc_ref[1]))

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

    def _trigger_speed(self):
        """stop_at_speed 用的前向速度。sensor 模式用 IMU 積分: 不吃輪速, 不受打滑影響。"""
        return self.v_imu if self.sensor else self._fwd_speed()

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
        固定門檻在這裡沒有用 —— 低 mu 的地面 coast 段還維持著 3.4 m/s, 等到
        「剩 1 m」才踩煞車已經來不及; 高 mu 的地面則會被過度保守的門檻白白
        中止掉還有效的資料。"""
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

        # 通用轉速防呆: 不管有沒有設 stop_at_wz, 轉速真的衝到危險區就中止。
        # stop_at_wz 是主要防線 (在還沒到 spin_max_w 就先放開), 這裡抓的
        # 是 overshoot 或其他 block 意外飄轉的情況。
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
        self._i_v = self._i_w = 0.0
        self._loc_wait_since = None
        if idx < len(self.scenarios) and self.scenarios[idx].get('stop_at_speed') is not None:
            # 加速段從靜止開始 (前面是歸位 settle, 已確認停住) -> IMU 速度積分歸零。
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

        # 同一個 trial 剩下的段落全部丟掉。留著會讓 coast 段在車子已經被煞停之後
        # 才執行, 記到一整段速度為 0 的假資料 —— 比沒有資料更糟, 因為它看起來
        # 像是「這個環境摩擦力大到一秒就停」。
        j = self.idx + 1
        while j < len(self.scenarios) and self.scenarios[j].get('group') == gid \
                and gid is not None:
            del self.scenarios[j]

        home = self._home_for(self.idx)
        insert = [_brake(f'Brake (abort: {sc["name"]})', phase='aborted'),
                  _repos(f'Reposition (after abort: {sc["name"]})', *home,
                         float(self.get_parameter('reposition_timeout').value))]

        # 重試是以 **trial** 為單位, 不是以段落為單位: coast 段的初始速度是前面
        # 那個加速段給的, 單獨重跑一個 coast 只會得到一條全 0 的軌跡。
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
            # 主動煞停: 目標速度 0, 給反向扭矩。放開油門是停不下來的。
            # sensor 模式: 轉速永遠用 gyro (定位斷了也有); 前向速度用融合的,
            # 融合過期就退回 IMU 積分。兩個都沒有才只能放開等。
            have_w = self.imu_ok if self.sensor else self.pose_fresh
            if not have_w:
                stopped = elapsed > 1.0
                dangerous = False
                efforts = [0.0] * 4
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
                efforts = self._mix(_clamp(-5.0 * v, -MAX_EFFORT, MAX_EFFORT),
                                    _clamp(-3.0 * w, -MAX_EFFORT, MAX_EFFORT))
            self._publish(sc['phase'], sc['name'], efforts, rate_limited=False)
            # 固定 duration 是給正常情況 (通常 <1 s 就停) 的下限, 不是上限:
            # 低 mu 的自轉段煞車前 wz 可能還有 20~30 rad/s, 3 s 常常煞不完,
            # 若強行到點就切下一段, 車子會帶著這個轉速衝進下一個情境, 造成
            # 不規則飄移甚至撞牆。改成「沒停就至多再等到 hard_cap」, hard_cap
            # 只在真的還沒煞停時才會被拉長。
            hard_cap = max(sc['duration'], 12.0) if dangerous else sc['duration']
            if stopped or elapsed > hard_cap:
                self._enter(self.idx + 1)
            return

        if sc['type'] == 'reposition':
            efforts, done = self._run_reposition(sc, elapsed)
            self._publish('reposition', sc['name'], efforts, rate_limited=False)
            if done or elapsed > sc['duration']:
                if not done:
                    self.get_logger().warn(f'{sc["name"]} 逾時, 直接進下一個情境')
                self._enter(self.idx + 1)
            return

        # ---- 量測段 (開迴路) ----
        bad = self._violation(sc)
        if bad is not None:
            # 已經收到夠長的一段就當「提早收工」: 資料留著, 不重試。
            # 截斷的 coast-down 還是合法資料; 丟掉再重跑一次只會在同一個地方
            # 再撞一次 —— run2 有 70% 的時間耗在這個迴圈裡。
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

        efforts = self._mix(throttle, steer)
        self._publish(sc['phase'], sc['name'], efforts,
                      rate_limited=(sc['type'] != 'instant'))

        # 速度觸發的結束條件 (加速段用)。到了目標速度就放開, coast 的進入速度
        # 因此是**指定的**, 不是被地面摩擦係數決定的。
        #
        # 判斷用的是 **TRIGGER_LEAD 秒之後的預測值**, 不是當下值: 從這裡決定放開
        # 到 Isaac 真的斷油有 ~0.05 s, 加速中的車在這段時間還會再往上衝
        # (自旋末段 25 rad/s^2 -> +1.2 rad/s)。不補的話目標值形同虛設。
        # 安全: 預測只在已經到目標 70% 之後才算數, 免得起步瞬間的大加速度誤觸發。
        vt = sc.get('stop_at_speed')
        if vt is not None and self.odom_ok:
            v = self._trigger_speed()
            if v >= 0.7 * float(vt) and v + max(self.acc_fwd, 0.0) * TRIGGER_LEAD >= float(vt):
                self._enter(self.idx + 1)
                return

        wt = sc.get('stop_at_wz')
        if wt is not None and self.odom_ok:
            w = abs(self.wz)
            if w >= 0.7 * float(wt) and w + max(self.wz_rate, 0.0) * TRIGGER_LEAD >= float(wt):
                self._enter(self.idx + 1)
                return

        if elapsed > sc['duration']:
            self._enter(self.idx + 1)

    # -------------------------------------------------- 閉迴路歸位 (非量測)
    def _run_reposition(self, sc, elapsed):
        """把車開回起跑點並轉到指定朝向。回傳 (efforts, done)。

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
            return self._mix(0.0, self._steer_for(err)), False

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
            ahead = max(self._ahead_clearance() - 0.6, 0.0)
            v_cap = min(0.6, math.sqrt(2.0 * self.brake_decel * ahead))
            v_des = _clamp(1.2 * dist, -v_cap, v_cap)
            e = v_des - self._fwd_speed()
            self._i_v = _clamp(self._i_v + e * self.dt, -3.0, 3.0)
            thr = _clamp(5.0 * e + 4.0 * self._i_v, -8.0, 8.0)
            return self._mix(thr, self._steer_for(err, kp=1.2)), False

        if self._repos_state == 'turn_to_heading':
            err = _wrap_pi(sc['target_heading'] - h)
            v = self._fwd_speed()
            # 順手把 drive 段殘餘的線速度煞掉 —— 低 mu 的地面光靠滑行會滑出好幾公尺
            thr = _clamp(-5.0 * v, -6.0, 6.0)
            if abs(err) < 0.10 and abs(self.wz) < 0.30:
                self._goto('settle')
                return [0.0] * 4, False
            return self._mix(thr, self._steer_for(err)), False

        # settle: 主動**維持**位置與朝向直到真的靜止, 不是放開 effort 等它自己停。
        # 放開 effort 在低 mu 的地面會轉過頭 30~40 度 (離線模擬 mu=0.4 實測), 接著
        # 直線段就朝著側牆發車, 前方餘裕從 8.5 m 掉到 3.4 m, 一發車就被 geofence
        # 中止 —— Block 4 在 mu<=0.4 時 6 次全滅, 偏偏那正是它最有訊號的區間。
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
        return self._mix(_clamp(-5.0 * v, -6.0, 6.0), self._steer_for(err)), False

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
        self._i_v = self._i_w = 0.0

    def _steer_for(self, yaw_err, kp=2.0):
        """角度誤差 -> steer effort。串級: 角度 -> 目標角速度 -> effort。

        角速度那一層**必須有積分項**。這台車的自轉阻力是 Coulomb 的 (跟轉速無關),
        純 P 控制在誤差變小時輸出也跟著變小, 到某個點就再也推不動車子 —— 車子
        卡在死區裡不動, 一路撐到 reposition 逾時。實測 (離線模擬, mu=1.2):
        沒有積分項時 16 個 reposition 全部逾時, 整份腳本從 4 分鐘變成 10 分鐘。
        """
        w_des = _clamp(kp * yaw_err, -1.5, 1.5)
        e = w_des - self.wz
        self._i_w = _clamp(self._i_w + e * self.dt, -4.0, 4.0)
        return _clamp(3.0 * e + 3.0 * self._i_w, -8.0, 8.0)

    # ------------------------------------------------------------------
    @staticmethod
    def _mix(throttle, steer):
        """[FL, FR, RL, RR] = [thr-steer, thr+steer, thr-steer, thr+steer]。

        超過 MAX_EFFORT 時**等比縮放**而不是各自 clamp。舊版是各自 clamp,
        結果 (T=1,S=8)/(T=2,S=8)/(T=3,S=8) 全部塌成同一個指令 (-10,+10),
        矩陣掃描裡有 6/25 格是重複的。等比縮放至少保住 throttle:steer 的比例。
        """
        left = throttle - steer
        right = throttle + steer
        peak = max(abs(left), abs(right))
        if peak > MAX_EFFORT:
            k = MAX_EFFORT / peak
            left, right = left * k, right * k
        return [left, right, left, right]

    def _publish(self, phase, name, target_efforts, rate_limited=True):
        if rate_limited:
            for i in range(4):
                self.current_efforts[i] = _rate_limit(
                    target_efforts[i], self.current_efforts[i], RATE_LIMIT, self.dt)
        else:
            self.current_efforts = list(target_efforts)

        msg = JointState()
        msg.name = list(JOINT_NAMES)
        msg.effort = [float(e) for e in self.current_efforts]
        self.effort_pub.publish(msg)

        m = String()
        m.data = name
        self.scenario_pub.publish(m)
        m = String()
        m.data = phase
        self.phase_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ControlCarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
