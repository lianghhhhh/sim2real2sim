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
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
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
SPIN_MAX_SAFE_W = 12.0                   # rad/s

# 四個目標初始角速度 (不是 effort!)。用跟 B3/B4 的 stop_at_speed 同一招:
# 轉速觸發取代時間觸發, 用固定的 SPIN_UP_STEER 衝上去, 到了目標值就放開。
# 舊版是固定 4 個 steer effort, 在低 mu 下沒有阻力矩制衡, S=5/7/10 全部會
# 衝破安全上限被同一個 SPIN_MAX_SAFE_W 拍平成幾乎一樣的 w0 (實測只剩
# ~1.5 跟 ~13 兩檔), 驗證「衰減斜率跟初始 w 無關」的 Coulomb 假設就沒意義
# 了。改成直接指定目標轉速, 四檔在任何 mu 下都是四個真正分開的初始條件；
# 全部低於 SPIN_MAX_SAFE_W, 觸發不到安全上限。
SPIN_TARGET_W = [2.0, 5.0, 8.0, 11.0]    # rad/s
SPIN_UP_STEER = MAX_EFFORT               # 固定用最大 steer 衝, 只有目標轉速是變數
# rate limiter 是 6 effort/s, 所以 1.67 s 就踩到底; 再給餘裕讓轉速長到目標值。
# 高 mu 時可能衝不到某些高檔目標 (穩態轉速比目標低), 這裡當逾時保護, 跟
# SPRINT_ACCEL_TIMEOUT 是同一個角色。
SPIN_UP_TIMEOUT = 4.0
SPIN_COAST_DURATION = 3.0                # ← 特徵取在這一段
SPIN_REPEATS = 3

# ── Block 2: 旋轉 creep (靜摩擦臨界) ─────────────────────────────────
# 舊版的 creep 是直線的, 10 秒會走 57 m。改成原地轉, 一樣量靜摩擦但不會跑掉。
CREEP_STEER_START, CREEP_STEER_END = 0.0, 10.0
CREEP_DURATION = 10.0
CREEP_REPEATS = 2

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
SPRINT_REPEATS = 2

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
SLIP_REPEATS = 3

REST_DURATION = 1.0
RATE_LIMIT = 6.0        # effort/s, 模擬人踩油門的急促程度 (instant 型會略過)


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


def build_scenarios(room, repos_timeout, wall_margin, blocks=(1, 2, 3, 4)):
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

    S = []

    # ── Block 1: 旋轉 coast-down ─────────────────────────────────────
    if 1 in blocks:
        S.append(_repos('Reposition -> center (B1)', cx, cy, 0.0, repos_timeout))
        for rep in range(SPIN_REPEATS):
            for w0 in SPIN_TARGET_W:
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
                           group=f'B2_{rep}', stop_at_wz=SPIN_MAX_SAFE_W))
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
        p('odom_topic', '/odom')
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

        self.effort_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.scenario_pub = self.create_publisher(String, '/test_scenario', 10)
        # phase 是給 collect_data_node 標 CSV 用的: 分析時只取 'measure'。
        self.phase_pub = self.create_publisher(String, '/test_phase', 10)
        self.create_subscription(Odometry, g('odom_topic').value,
                                 self.on_odom, ODOM_QOS)

        self.hz = 20.0
        self.dt = 1.0 / self.hz

        # 位姿 (geofence 與 reposition 用)。世界速度用位置差分算, 避免去猜
        # Isaac 的 twist 是車體座標還是世界座標 —— 猜錯閉迴路會直接發散。
        self.pos = None
        self.yaw = 0.0
        self.wz = 0.0
        self.vel_w = (0.0, 0.0)
        self._prev_pos = None
        self._prev_t = None
        self.t_odom = -1e9

        self.current_efforts = [0.0, 0.0, 0.0, 0.0]
        self.finished = False
        self.aborted = []
        self.truncated = []      # 踩到圍籬但資料還是留著的段落

        self.scenarios = build_scenarios(
            self.room, float(g('reposition_timeout').value),
            self.wall_margin, blocks)
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
            f'  房間 x[{self.room[0]}, {self.room[1]}] y[{self.room[2]}, {self.room[3]}], '
            f'離牆 {self.wall_margin} m (再加上當下速度的煞停距離) 內提前結束情境\n'
            f'  分析時只取 phase == "measure" 的列')

        self.timer = self.create_timer(self.dt, self.control_callback)

    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg):
        t = self._now()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._prev_pos is not None and self._prev_t is not None:
            dt = t - self._prev_t
            if 1e-3 < dt < 0.5:
                vx = (x - self._prev_pos[0]) / dt
                vy = (y - self._prev_pos[1]) / dt
                # 輕度平滑: 差分在 20 Hz 的模擬真值上很乾淨, 但仍會有量化台階
                self.vel_w = (0.6 * self.vel_w[0] + 0.4 * vx,
                              0.6 * self.vel_w[1] + 0.4 * vy)
        self._prev_pos = (x, y)
        self._prev_t = t
        self.pos = (x, y)
        self.yaw = _yaw_of(msg.pose.pose.orientation)
        self.wz = msg.twist.twist.angular.z
        self.t_odom = t

    @property
    def odom_ok(self):
        return self.pos is not None and (self._now() - self.t_odom) < 1.0

    def _heading(self):
        """車頭在世界座標的角度。"""
        return _wrap_pi(self.yaw + self.fwd_off)

    def _fwd_speed(self):
        h = self._heading()
        return self.vel_w[0] * math.cos(h) + self.vel_w[1] * math.sin(h)

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
        if not self.odom_ok:
            return None                      # 沒有 odom 就沒有 geofence (會另外警告)

        # 通用轉速防呆: 不管有沒有設 stop_at_wz, 轉速真的衝到危險區就中止。
        # stop_at_wz 是主要防線 (在還沒到 SPIN_MAX_SAFE_W 就先放開), 這裡抓的
        # 是 overshoot 或其他 block 意外飄轉的情況。
        if abs(self.wz) > 1.5 * SPIN_MAX_SAFE_W:
            return f'轉速過快 wz={self.wz:.1f} rad/s, 有失控飄移風險'

        mc = float(sc.get('min_clear') or 0.0)
        if mc > 0.0:
            # 這裡用的是**全向**速率, 不是前向: 自旋段車體是橫著滑出去的,
            # 只看車頭方向的分量會漏掉大部分的動能。
            speed = float(math.hypot(self.vel_w[0], self.vel_w[1]))
            need = mc + speed * speed / (2.0 * self.brake_decel)
            clear = self._wall_clearance()
            if clear < need:
                return (f'離牆只剩 {clear:.2f} m (需要 {need:.2f} m '
                        f'= 餘裕 {mc:.2f} + {speed:.2f} m/s 的煞停距離 '
                        f'{need - mc:.2f})')

        ma = sc.get('min_ahead')
        if ma is not None:
            v = max(self._fwd_speed(), 0.0)
            stop_dist = v * v / (2.0 * self.brake_decel)
            need = float(ma) + stop_dist
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

        if sc['type'] == 'brake':
            # 主動煞停: 目標速度 0, 給反向扭矩。放開油門是停不下來的。
            if not self.odom_ok:
                stopped = elapsed > 1.0
                dangerous = False
                efforts = [0.0] * 4
            else:
                v, w = self._fwd_speed(), self.wz
                stopped = abs(v) < 0.05 and abs(w) < 0.08
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
        if not self.odom_ok:
            self.get_logger().warn(
                '收不到 odom -> geofence 與 reposition 停用, 車子可能撞牆。'
                '檢查 Isaac 有沒有按 Play。', throttle_duration_sec=10.0)

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
        vt = sc.get('stop_at_speed')
        if vt is not None and self.odom_ok and self._fwd_speed() >= float(vt):
            self._enter(self.idx + 1)
            return

        wt = sc.get('stop_at_wz')
        if wt is not None and self.odom_ok and abs(self.wz) >= float(wt):
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
