#!/usr/bin/env python3
"""比較兩個 (或多個) 環境的地面摩擦力: 變大還變小、大概變多少。不需要 ROS。

吃的是 calibrate_env_node (control_car_node + collect_data_node) 收的 CSV:

    ./scripts/estimate_friction.py car_run_data/sim_data.csv car_run_data/real_data.csv
    ./scripts/estimate_friction.py REF.csv A.csv B.csv          # 多個目標都跟 REF 比
    ./scripts/estimate_friction.py REF.csv A.csv --ref-mu 0.5   # 給 REF 的有效 mu -> 換算絕對值
    ./scripts/estimate_friction.py REF.csv A.csv --source fus   # 用感測器融合的位姿, 不用 GT
    ./scripts/estimate_friction.py REF.csv A.csv --plot out/    # 每個特徵的逐 trial 散佈圖

第一個檔案是**參考環境**, 後面的都算「相對於它」的比值 (target / ref)。
不寫死任何環境的數字 —— 摩擦係數、房間大小、trial 數、有沒有跑某個 block 都不假設,
缺的特徵就跳過, 被中止 / 截斷的 trial 照樣能用。

**這是輪速控制版** (kros_car.usd / 實體 KROS car: /joint_command 給輪速, 馬達的
速度環出扭矩)。舊的 effort 控制版 (car.usd) 收的 CSV 讀不了 —— 那一版的特徵
(斷油滑行的減速度、起轉 effort) 在輪速控制下不存在, 見 control_car_node.py 檔頭。

────────────────────────────────────────────────────────────────────────
用了哪些特徵, 為什麼
────────────────────────────────────────────────────────────────────────
輪速控制下, 車體的運動被速度環釘死, 摩擦力跑到**馬達出多少扭矩**上。
skid-steer 原地轉時四輪都在滑, 每輪摩擦力大小 mu*N, 穩態時

    tau_wheel = r * mu * N * a / sqrt(a² + b²)      (a 半軸距, b 半輪距)

正比於 mu, 跟轉速無關 (Coulomb)。取兩環境的比值就把幾何、質量全部消掉了。

  [主] B1 spin_torque   等速原地轉時四輪 |扭矩| 的平均。  ∝ mu_dynamic
       丟掉每段前 1 秒 (起轉過渡), 剩下的取中位數。穩態量, 可以平均好幾秒,
       每個 trial 的雜訊很小。
       **前提: 馬達沒飽和。** 需要的扭矩超過馬達在那個轉速下出得了的扭矩時,
       量到的就是馬達的上限、輪速也追不上指令 —— 那一趟會被標成「飽和」。
       上限**隨轉速下降** (DDSM115: 堵轉 2.0 N·m, 115 rpm 剩 0.96, 200 rpm 是 0;
       --torque-limit / --no-load-speed / --rated-speed / --rated-torque)。
       飽和的環境不能拿來校正, 比值也不能信 (不保證偏哪一邊: 2026-10-13 無頭
       PhysX 地面 0.5 -> 1.0, 最高檔飽和, 量到 1.58, 真值 1.50)。
       把 control_car_node 的 spin_wheel_speed 調低再收一次。

       **馬達自己的損耗要扣掉。** 回授的扭矩是「電流 x 扭矩常數」, 裡面除了輪胎
       的摩擦力還有馬達 / 軸承自己吃掉的那一塊 (DDSM115 規格書: 額定點 1.5 A x
       0.75 = 1.13 N·m, 軸輸出只有 0.96 -> 差 0.17 N·m)。模擬的馬達沒有這一塊,
       不扣的話實體車的 mu 會被高估三成左右。B3 等速直行量到的就是它 (直行時輪胎
       不滑, 扭矩 = 損耗 + 滾動阻力), 用兩檔車速擬合 loss(w) = c0 + c1*w, 從 B1 / B2
       的扭矩裡扣掉該輪速的值。模擬只有 0.015 N·m, 扣不扣幾乎沒差; 兩邊都扣才一致。
       (--no-loss-correction 可以關掉。)

  [副] B2 ramp_torque   慢速 ramp 滑動段的扭矩。跟 B1 量的是同一個量, 低速版;
       兩者差很多代表扭矩跟轉速有關 (有黏滯項), 不是純 Coulomb。

  [副] B2 breakaway_torque   ramp 起轉瞬間 (車體開始轉之前) 的峰值扭矩。
       ∝ mu_static。速度環很硬時峰值只有一兩個取樣, 解析度差; 靜摩擦預設跟著
       動摩擦走 (friction_calib_step.py 的 --static-mode tied)。

  [副] B3/B4 brake_decel   輪速指令瞬間歸零後的車體減速度。
       輪子鎖死 (brake_slip → 1) 時 = mu*g, 直接就是絕對值; 輪子沒鎖死
       (brake_slip → 0) 代表扭矩先到上限, 減速度跟地面無關。
       整段只有 0.1~0.2 秒, IMU 全速率檔才量得到。

  [副] B4 launch_accel / launch_slip   輪速階躍起步的加速度與 slip ratio。
       抓地力受限時加速度 = mu*g; 扭矩受限時跟地面無關 (slip → 0)。

  [副] B1 spin_yaw_gain   車體轉速 / 輪速。理想 Coulomb 摩擦下是純幾何量
       b*r/(a²+b²) = 0.174, 跟 mu 無關 —— **但 PhysX 實測不是**: 無頭 PhysX 量到
       0.10~0.15, 隨 mu 與轉速變 (mu 大 -> 轉得慢)。所以它不能當對照組, 只印出來
       參考, 順便給出等效輪距 T_eff = 2r / gain (cmd_vel_bridge 的 effective_track)。

  [副] B3 cruise_torque   等速直行的扭矩 = 滾動阻力 + 馬達損耗, 跟地面摩擦
       係數無關。模擬裡接近 0 (0.015 N·m)。上面說的損耗就是從這裡來的。

驗證 (2026-10-12, 無頭 PhysX 直接跑同一套指令, 車重 12.5 kg, 輪子材質 0.5):
    地面 0.5 -> 1.0 (有效 0.50 -> 0.75, 真值比值 1.50):
        spin_torque 0.578 -> 0.869  比值 1.50   (理論值 r*mu*N*a/sqrt(a²+b²) = 0.563 / 0.845)
        ramp_torque 1.53, breakaway_torque 1.50, launch_accel 4.83 -> 7.20 (= mu*g)
    地面 2.0: 需要 1.41 N·m, 個別輪子已經貼到 2.0 的上限 -> 輪速追不上, 判定飽和。
    USD 原本的車重 45 kg: 地面 0.5 就轉不動 (車體轉速 0.03 rad/s), 全部飽和。

最後把 [主] 特徵的 log 比值用 bootstrap 變異數做反變異數加權, 得到綜合比值與信賴區間。

────────────────────────────────────────────────────────────────────────
「有效 mu」vs「地面 mu」 (Isaac / PhysX 限定)
────────────────────────────────────────────────────────────────────────
量到的是**輪子與地面合起來**的有效摩擦。PhysX 預設 combine mode 是 average:
    mu_eff = (mu_ground + mu_wheel) / 2
kros_car*.usd 的輪子沒綁物理材質 -> 用預設材質 0.5。所以 ground 0.5 -> 2.0 (4 倍)
量到的是 mu_eff 0.5 -> 1.25 (2.5 倍), 不是 4 倍。

**要的是地面摩擦係數就給參考環境的 USD**, 最後會印一個 ★ 區塊:

    ./scripts/estimate_friction.py sim.csv real.csv --ref-usd kros_car_05_sim.usd

  physics:dynamicFriction  <- B1 等速自旋扭矩 (滑動中量的)
  physics:staticFriction   <- B2 起轉峰值扭矩 (從靜止推到開始動)
  換算: 參考地面 + 輪子 -> 參考有效值 -> x 比值 -> 目標有效值 -> 反解目標地面。
  USD 讀不了 (沒有 pxr) 就用數字: --ref-ground-mu 0.5 [--wheel-mu 0.5 --combine average]
  真實世界沒有「輪子材質」這層, 直接看比值。

────────────────────────────────────────────────────────────────────────
訊號來源 (--source)
────────────────────────────────────────────────────────────────────────
扭矩永遠來自輪子回授 (`*_effort` 欄); --source 只決定**車體運動**從哪裡來:

auto   (預設) 有 <csv>_imu.csv 就用 gyro, 沒有才退回 gt
gyro   **真車流程**: collect_data_node 全速率記的 /imu。
         轉速 = gyro z - 零偏 (零偏從 phase=='calibrate' 的靜止段估)
         前向速度 = 前向加速度從靜止積分 (B3 cruise / B4 launch 起點歸零,
         基準取起點前 0.25 s 的靜止平均)。不吃輪速、不吃定位。
         IMU 假設跟 base_link 同向; 直線段幾乎不轉, 所以沒做槓桿臂補償。
gt     Isaac ground truth (car_position_*, gt_yaw, twist)。模擬器裡對答案用。
fus / lid / cam / imu / whl   各定位線的位姿 (主 CSV 取樣, 依 _stamp 去重)
         * whl (以及吃輪速的 fus 的速度) 打滑時是錯的, 而打滑就是摩擦造成的 ——
           B3/B4 不要信。

--compare-gt       同一份資料再用 GT 算一次, 並排印出 (模擬器驗證感測器流程用)
--gyro-noise 等    對 IMU 注入雜訊 / 零偏, 模擬真車 MEMS (Isaac 的 IMU 是理想的)
"""
import argparse
import json
import math
import os
import re
import sys

import numpy as np
import pandas as pd

G = 9.81
STRAIGHT_MAX_TURN = math.radians(10.0)   # B3/B4 直行段車頭最多轉這麼多, 超過就排除
SPIN_SETTLE = 1.0                        # B1 每段開頭丟掉幾秒 (起轉過渡)
CRUISE_TAIL = 1.0                        # B3 直行段取最後幾秒當穩態
SPIN_ONSET_W = 0.05                      # B2 車體轉速超過這個值算「開始轉」 [rad/s]
SAT_TORQUE_FRAC = 0.95                   # 扭矩 >= 這個比例 x 上限 -> 飽和
SAT_TRACK_ERR = 0.10                     # 輪速比指令低這麼多 -> 飽和 (追不上)
MAX_SAT_FRAC = 0.3                       # 自旋 trial 超過這個比例飽和 -> 那個環境的主特徵不能用


class Motor:
    """馬達在某個輪速下最多出得了多少扭矩 —— 規格書上那條直線。

        limit(w) = min(堵轉扭矩, 額定扭矩 x (空載轉速 - w) / (空載轉速 - 額定轉速))

    no_load <= 0 = 不管轉速, 一律用堵轉扭矩 (沒有扭矩-轉速曲線的馬達 / 舊的 USD)。
    """

    def __init__(self, stall=2.0, no_load=20.94, rated_speed=12.04, rated_torque=0.96):
        self.stall = float(stall)
        self.no_load = float(no_load)
        self.rated_speed = float(rated_speed)
        self.rated_torque = float(rated_torque)

    def limit(self, w) -> float:
        if self.no_load <= self.rated_speed or self.no_load <= 0.0:
            return self.stall
        lim = self.rated_torque * (self.no_load - abs(float(w))) / (self.no_load - self.rated_speed)
        return max(0.0, min(self.stall, lim))


def fit_loss(rows):
    """B3 等速直行的 (輪速, 扭矩) -> loss(w) = c0 + c1*|w| 的 (c0, c1)。沒資料回 None。

    兩檔車速才擬合得出斜率; 只有一檔、或擬合出來是負斜率 (損耗不會隨轉速變小,
    那是雜訊) 就當成常數。
    """
    a = np.asarray([r for r in rows if np.all(np.isfinite(r))], float)
    if not len(a):
        return None
    w, tq = np.abs(a[:, 0]), a[:, 1]
    if len(a) >= 2 and np.ptp(w) > 0.2 * w.max():
        c1, c0 = np.polyfit(w, tq, 1)
        if c1 >= 0.0:
            return float(c0), float(c1)
    return float(np.median(tq)), 0.0


def loss_at(model, w) -> float:
    if model is None or not np.isfinite(w):
        return 0.0
    return max(0.0, model[0] + model[1] * abs(float(w)))
WHEEL_KEYS = ('front_left', 'front_right', 'rear_left', 'rear_right')
WHEELS = [f'{k}_velocity' for k in WHEEL_KEYS]
EFFORTS = [f'{k}_effort' for k in WHEEL_KEYS]

# 特徵 -> (說明, 角色)。角色: primary 參與綜合估計; secondary 只印; control 應該 ≈ 1
FEATURES = {
    'spin_torque':      ('B1 等速自旋的輪子扭矩 [N·m]', 'primary'),
    'ramp_torque':      ('B2 慢速 ramp 滑動段扭矩 [N·m]', 'secondary'),
    'breakaway_torque': ('B2 起轉峰值扭矩 [N·m] (靜摩擦)', 'secondary'),
    'brake_decel':      ('B3/B4 急煞減速度 [m/s²]', 'secondary'),
    'brake_slip':       ('B3/B4 急煞 slip (1=輪子鎖死在滑)', 'secondary'),
    'launch_accel':     ('B4 階躍起步加速度 [m/s²]', 'secondary'),
    'launch_slip':      ('B4 起步 slip ratio', 'secondary'),
    'cruise_torque':    ('B3 等速直行扭矩 [N·m] (滾動阻力)', 'secondary'),
    'spin_yaw_gain':    ('B1 車體轉速/輪速 [-]', 'secondary'),
}


# ═══════════════════════════════════════════════════════════════════════
#  讀檔: 把 CSV 變成統一格式的 track (不管位姿來自哪個來源)
# ═══════════════════════════════════════════════════════════════════════
def _check_format(path, cols):
    if 'velocity_command_front_left' in cols:
        return
    if 'effort_command_front_left' in cols:
        sys.exit(f'{path}: 這是舊的 effort 控制版收的 CSV (欄位是 effort_command_*)。'
                 '這支腳本現在只吃輪速控制版的資料 —— 兩種控制方式的特徵完全不同。')
    sys.exit(f'{path}: 缺少欄位 velocity_command_* —— 不是 calibrate_env_node 收的 CSV?')


def _wheel_cols(d):
    """輪速指令 / 輪速 / 扭矩 -> track 共用的那幾欄。缺的就是 NaN。"""
    fl, fr = d['velocity_command_front_left'], d['velocity_command_front_right']
    out = {'steer': 0.5 * (fr - fl), 'thr': 0.5 * (fr + fl)}
    if set(WHEELS) <= set(d.columns):
        out['wheel'] = d[WHEELS].abs().mean(axis=1)
        out['wheel_fwd'] = d[WHEELS].mean(axis=1)
    else:
        out['wheel'] = out['wheel_fwd'] = np.nan
    if set(EFFORTS) <= set(d.columns):
        out['tq'] = d[EFFORTS].abs().mean(axis=1)
        out['tq_max'] = d[EFFORTS].abs().max(axis=1)
        # 沿著輪子指令方向的扭矩 (正 = 在推)。直行時扭矩很小, 取絕對值會把雜訊
        # 折成正的偏差, 所以損耗用這個算。
        cmd = np.sign(d[[f'velocity_command_{k}' for k in WHEEL_KEYS]].to_numpy(float))
        out['tq_dir'] = pd.Series((d[EFFORTS].to_numpy(float) * cmd).mean(axis=1), index=d.index)
    else:
        out['tq'] = out['tq_max'] = out['tq_dir'] = np.nan
    return out


def load_track(path, source, forward_axis_deg, max_sigma, max_age):
    d = pd.read_csv(path)
    _check_format(path, d.columns)
    if not {'scenario_name', 'phase'} <= set(d.columns):
        sys.exit(f'{path}: 缺少 scenario_name / phase —— 不是 calibrate_env_node 收的 CSV?')

    if source == 'gt':
        t, x, y, yaw = (d['odom_stamp'], d['car_position_x'],
                        d['car_position_y'], d['gt_yaw'])
        valid = t.notna() & x.notna() & yaw.notna()
    else:
        cols = [f'{source}_{k}' for k in ('x', 'y', 'yaw', 'stamp')]
        if not set(cols) <= set(d.columns) or d[cols[0]].notna().sum() == 0:
            sys.exit(f'{path}: 來源 {source} 沒有資料 ({cols[0]} 全是 NaN)')
        x, y, yaw, t = (d[c] for c in cols)
        valid = t.notna() & x.notna() & yaw.notna()
        if f'{source}_age' in d:
            valid &= d[f'{source}_age'] < max_age
        if max_sigma is not None and f'{source}_sigma' in d:
            valid &= d[f'{source}_sigma'] < max_sigma

    tr = pd.DataFrame({
        'name': d['scenario_name'].astype(str), 'phase': d['phase'].astype(str),
        't': t.astype(float), 'x': x.astype(float), 'y': y.astype(float),
        'yaw': yaw.astype(float), 'valid': valid.values, **_wheel_cols(d),
    })
    # 感測器欄位是複製最新一則 -> 同一個 stamp 只算一次, 之後 forward-fill
    fresh = tr['valid'] & (tr['t'] != tr['t'].where(tr['valid']).ffill().shift())
    s = tr[fresh]
    yaw_u = np.unwrap(s['yaw'].to_numpy())
    dt = np.diff(s['t'].to_numpy(), prepend=np.nan)
    dt[~(dt > 1e-4)] = np.nan
    vx = np.diff(s['x'].to_numpy(), prepend=np.nan) / dt
    vy = np.diff(s['y'].to_numpy(), prepend=np.nan) / dt
    w = np.diff(yaw_u, prepend=np.nan) / dt
    head = yaw_u + math.radians(forward_axis_deg)
    tr.loc[fresh, 'yaw_u'] = yaw_u
    tr.loc[fresh, 'w'] = w
    tr.loc[fresh, 'vf'] = vx * np.cos(head) + vy * np.sin(head)
    if source == 'gt' and 'car_angular_velocity_z' in d:
        # GT 有瞬時角速度, 比位姿差分 (區間平均) 準; 感測器那邊沒記 twist
        tr.loc[fresh, 'w'] = d.loc[fresh, 'car_angular_velocity_z']
    for c in ('yaw_u', 'w', 'vf', 't'):
        tr[c] = tr[c].where(tr['valid']).ffill()
    tr['fresh'] = fresh
    tr['seg'] = (tr['name'] != tr['name'].shift()).cumsum()
    return tr


def imu_path_for(csv_path):
    stem, _ = os.path.splitext(csv_path)
    return stem + '_imu.csv'


def load_imu_track(path, forward_axis_deg, noise, rng):
    """collect_data_node 的 <csv>_imu.csv -> 跟 load_track 同格式的 track。

    noise = dict(gyro, gyro_bias, acc, acc_bias): 注入到原始量測上, 估計端**不知道**
    注入的零偏是多少 —— 它必須自己從 calibrate 段估出來, 跟真車一樣。"""
    d = pd.read_csv(path)
    if len(d) < 10:
        sys.exit(f'{path}: IMU 檔幾乎是空的 ({len(d)} 列) —— /imu 有沒有在發?')
    _check_format(path, d.columns)
    d = d.sort_values('stamp').drop_duplicates('stamp').reset_index(drop=True)
    n = len(d)
    gz = d['gyro_z'].to_numpy(float).copy()
    ax = d['acc_x'].to_numpy(float).copy()
    ay = d['acc_y'].to_numpy(float).copy()
    if noise:
        gz += noise['gyro_bias'] + rng.normal(0, noise['gyro'], n)
        ax += noise['acc_bias'] + rng.normal(0, noise['acc'], n)
        ay += rng.normal(0, noise['acc'], n)

    t = d['stamp'].to_numpy(float)
    phase = d['phase'].astype(str)
    name = d['scenario_name'].astype(str)

    # gyro 零偏: calibrate 段 (去頭 0.3 s) > Rest 段 > 0
    still = (phase == 'calibrate').to_numpy().copy()
    if still.sum() > 10:
        still &= t > t[still][0] + 0.3
    if still.sum() < 10:
        still = ((phase == 'measure') & name.str.startswith('Rest')).to_numpy()
    bias = float(np.median(gz[still])) if still.sum() >= 10 else 0.0
    if still.sum() < 10:
        print(f'! {path}: 找不到靜止段估 gyro 零偏, 當作 0')

    w = gz - bias
    dt = np.diff(t, prepend=t[0])
    dt[(dt <= 0) | (dt > 0.1)] = 0.0                 # 掉包 / 亂序那一步不積分
    yaw_u = np.cumsum(0.5 * (w + np.r_[w[0], w[:-1]]) * dt)

    fo = math.radians(forward_axis_deg)
    a_f = math.cos(fo) * ax + math.sin(fo) * ay

    tr = pd.DataFrame({
        'name': name, 'phase': phase, 't': t, 'x': np.nan, 'y': np.nan,
        'yaw': np.nan, 'valid': True, **_wheel_cols(d),
        'yaw_u': yaw_u, 'w': w, 'vf': np.nan, 'fresh': True,
    })
    tr['seg'] = (tr['name'] != tr['name'].shift()).cumsum()

    # 前向速度: 從靜止起步的段落 (B3 cruise / B4 launch) 起點歸零, 一路積分到它
    # 後面的急煞段結束。基準取起點前 0.25 s 的靜止平均 (扣掉零偏與車體傾斜)。
    vf = np.full(n, np.nan)
    active, v, ref = False, 0.0, 0.0
    for _, g in tr.groupby('seg', sort=True):
        i0, i1 = g.index[0], g.index[-1]
        nm, meas = g['name'].iloc[0], g['phase'].iloc[0] == 'measure'
        start = i1 + 1                                   # 不積分
        if meas and nm.startswith(('B3 cruise', 'B4 launch')):
            pre = (t >= t[i0] - 0.25) & (t < t[i0])
            ref = float(np.median(a_f[pre])) if pre.sum() >= 3 else float(a_f[i0])
            active, v = True, 0.0
            vf[i0] = 0.0
            start = i0 + 1
        elif active and meas and nm.startswith(('B3 brake', 'B4 brake')):
            start = i0
        else:
            active = False
        for i in range(start, i1 + 1):
            v += (a_f[i] - ref) * dt[i]
            vf[i] = v
    tr['vf'] = vf
    tr.attrs['gyro_bias'] = bias
    tr.attrs['vf_integrated'] = True     # 前向速度是加速度積分來的 (見 extract 的 min_rows)
    return tr


def loc_quality(csv_path, source_prefix='fus'):
    """收資料時控制用的定位有沒有壞掉。回傳 (摘要字串, 是否有問題) 或 None。

    跟 control_car_node 的一致性檢查同一套: 1 秒窗內「定位 yaw 轉了多少」vs
    「gyro 積分轉了多少」, 只看有在轉的窗。定位壞掉時控制節點會在錯的位置 / 錯的
    方向執行測試 (B3/B4 起點不對、直線段其實在轉), 而**摩擦力特徵本身看不出來** ——
    2026-09-15 那一輪 sim 的融合定位整輪凍結, B3 對照組比值跑到 1.85 才露餡。"""
    ip = imu_path_for(csv_path)
    if not os.path.exists(ip):
        return None
    d = pd.read_csv(csv_path, usecols=lambda c: c in {
        f'{source_prefix}_yaw', f'{source_prefix}_stamp', 'phase',
        'car_position_x', 'car_position_y', f'{source_prefix}_x', f'{source_prefix}_y'})
    if f'{source_prefix}_yaw' not in d or d[f'{source_prefix}_yaw'].notna().sum() < 20:
        return None
    im = pd.read_csv(ip, usecols=['stamp', 'phase', 'gyro_z']).sort_values('stamp')
    im = im.drop_duplicates('stamp')
    t_g = im['stamp'].to_numpy(float)
    calib = (im['phase'] == 'calibrate').to_numpy()
    bias = float(np.median(im['gyro_z'][calib])) if calib.sum() > 10 else 0.0
    dt = np.diff(t_g, prepend=t_g[0])
    dt[(dt <= 0) | (dt > 0.1)] = 0.0
    g_int = np.cumsum((im['gyro_z'].to_numpy(float) - bias) * dt)

    f = d[[f'{source_prefix}_stamp', f'{source_prefix}_yaw']].dropna()
    f = f.drop_duplicates(f'{source_prefix}_stamp').sort_values(f'{source_prefix}_stamp')
    t_p = f[f'{source_prefix}_stamp'].to_numpy(float)
    y_p = np.unwrap(f[f'{source_prefix}_yaw'].to_numpy(float))

    lo, hi = max(t_p[0], t_g[0]), min(t_p[-1], t_g[-1])
    msgs, bad = [], False
    if hi - lo < 5.0:
        gap = np.median(t_p) - np.median(t_g)
        return (f'{source_prefix} 位姿時戳跟 IMU 幾乎沒有重疊 (差 {gap:+.0f} s) —— 時鐘不同步', True)
    starts = np.arange(lo, hi - 1.0, 0.5)
    dp = np.interp(starts + 1.0, t_p, y_p) - np.interp(starts, t_p, y_p)
    dg = np.interp(starts + 1.0, t_g, g_int) - np.interp(starts, t_g, g_int)
    turning = np.maximum(np.abs(dp), np.abs(dg)) > 0.5
    if turning.sum() >= 5:
        frac = float(np.mean(np.abs(dp - dg)[turning] > 0.35))
        msgs.append(f'{source_prefix} yaw 跟 gyro 對不上的轉動窗 {100 * frac:.0f}%')
        bad |= frac > 0.10
    if 'car_position_x' in d and d['car_position_x'].notna().any():
        e = np.hypot(d[f'{source_prefix}_x'] - d['car_position_x'],
                     d[f'{source_prefix}_y'] - d['car_position_y']).dropna()
        if len(e):
            msgs.append(f'對 GT 位置誤差 中位 {e.median() * 100:.1f} cm / p95 {e.quantile(0.95) * 100:.0f} cm')
            bad |= e.median() > 0.15
    return ('; '.join(msgs), bad) if msgs else None


def segments(tr):
    """依 scenario_name 切成連續段落, 只留 measure。回傳 [(block, name, df)]。"""
    out = []
    for _, g in tr.groupby('seg', sort=True):
        if g['phase'].iloc[0] != 'measure':
            continue
        name = g['name'].iloc[0]
        m = re.match(r'^(B\d+ \w+)', name)
        block = m.group(1) if m else ('Rest' if name.startswith('Rest') else None)
        if block:
            out.append((block, name, g))
    return out


def _cross_time(t, v, level):
    """v 第一次掉到 level 以下的時刻 (線性內插); 一開始就在下面或從沒掉下來回 None。"""
    below = np.flatnonzero(v <= level)
    if not len(below) or below[0] == 0:
        return None
    k = below[0]
    return float(np.interp(level, [v[k], v[k - 1]], [t[k], t[k - 1]]))


def spin_onset(t, w, th=SPIN_ONSET_W, hold=3):
    """車體開始轉的時刻: |w| 連續 hold 個點超過門檻的第一個點。沒轉回 None。

    要連續幾個點: 起轉前有一段預滑移 (輪胎與底盤被扭矩扭出彈性變形), 車體會
    抖一下又停住, 單點超過門檻不算。"""
    over = np.abs(w) > th
    run = 0
    for i, o in enumerate(over):
        run = run + 1 if o else 0
        if run >= hold:
            return float(t[i - hold + 1])
    return None


# ═══════════════════════════════════════════════════════════════════════
#  逐 trial 抽特徵
# ═══════════════════════════════════════════════════════════════════════
def extract(tr, r_wheel, motor, loss_correction=True):
    segs = segments(tr)
    F = {k: [] for k in FEATURES}
    # B1 每個 trial 一列: (輪速指令, 扣掉損耗的扭矩, 車體轉速, 實際輪速, 是否飽和, 原始扭矩)
    F['_spin_rows'] = []
    # 損耗要等 B3 跑完才知道, 所以先記「原始扭矩 + 當時的輪速」, 最後一起扣
    raw = {'spin': [], 'ramp': [], 'breakaway': [], 'cruise': []}
    n_spin = n_no_effort = 0
    # 急煞 / 起步的速度變化只有 0.05~0.15 s。IMU 積分出來的速度在取樣點之間是準的
    # (加速度幾乎是常數), 兩個點就能內插; 位姿差分的速度是「上一個區間的平均」,
    # 落後半個取樣而且有雜訊, 窗內不到 4 個點時算出來的加速度可以差到 2~3 倍
    # (60 Hz 的 GT 實測: 真值 7.2 量到 25) -> 那種 trial 直接不採用。
    min_rows = 2 if tr.attrs.get('vf_integrated') else 4

    excluded = []
    bad_trial = False
    for b, name, g in segs:
        g = g[g['valid']]
        if len(g) < 3:
            continue
        t = g['t'].to_numpy() - g['t'].iloc[0]

        # 直行段車頭轉了很多 = 起點 / 朝向不對 (通常是定位壞掉, 歸位把車擺歪了)。
        # 那一趟的加速度與後面的急煞都不是直線的量, 整趟排除。
        if b in ('B3 cruise', 'B4 launch'):
            turn = abs(g['yaw_u'].iloc[-1] - g['yaw_u'].iloc[0])
            bad_trial = turn > STRAIGHT_MAX_TURN
            if bad_trial:
                excluded.append(f'{name} (轉了 {math.degrees(turn):.0f}°)')
                continue
        elif b in ('B3 brake', 'B4 brake'):
            if bad_trial:
                continue
        else:
            bad_trial = False

        if b == 'B1 spin_hold':
            k = t >= SPIN_SETTLE
            if k.sum() < 5:
                continue
            n_spin += 1
            s = g[k]
            cmd = float(s['steer'].abs().median())
            wheel = float(s['wheel'].median())
            w_body = float(s['w'].abs().median())
            tq = float(s['tq'].median())
            if not np.isfinite(tq):
                n_no_effort += 1
                continue
            # 飽和: 任何一輪的扭矩貼著上限, 或輪速明顯追不上指令。兩個都看 ——
            # 扭矩回授的刻度不準 (真車的扭矩常數) 時只有後者靠得住。
            # 上限用**實際輪速**查: 馬達轉得愈快出得了的扭矩愈少 (見 Motor)。
            sat = bool(float(s['tq_max'].median()) >= SAT_TORQUE_FRAC * motor.limit(wheel)
                       or (cmd > 0.5 and wheel < (1.0 - SAT_TRACK_ERR) * cmd))
            raw['spin'].append((cmd, tq, w_body, wheel, sat))
            if wheel > 0.5:
                F['spin_yaw_gain'].append(w_body / wheel)

        elif b == 'B2 spin_ramp':
            w = g['w'].to_numpy()
            tq = g['tq'].to_numpy()
            if not np.isfinite(tq).any():
                continue
            ton = spin_onset(t, w)
            if ton is None:
                continue                     # 整段都沒轉起來 (扭矩不夠), 沒有起轉點
            # 峰值找在起轉點前後: 前 0.5 s (扭矩爬升) 到後 0.1 s (回授的延遲)
            win = (t >= ton - 0.5) & (t <= ton + 0.1)
            wh = g['wheel'].to_numpy()
            if win.any() and np.isfinite(tq[win]).any():
                j = int(np.nanargmax(np.where(win, tq, -np.inf)))
                raw['breakaway'].append((float(tq[j]), float(wh[j])))
            slide = (t >= ton + 0.3) & (np.abs(w) > 0.2)
            if slide.sum() >= 5:
                raw['ramp'].append((float(np.nanmedian(tq[slide])), float(np.nanmedian(wh[slide]))))

        elif b == 'B3 cruise':
            k = t >= t[-1] - CRUISE_TAIL
            tq = g.loc[k, 'tq_dir']
            if k.sum() >= 5 and tq.notna().any():
                F['cruise_torque'].append(float(tq.median()))
                raw['cruise'].append((float(g.loc[k, 'wheel'].median()), float(tq.median())))

        elif b in ('B3 brake', 'B4 brake'):
            vf = g['vf'].to_numpy()
            if not np.isfinite(vf).all() or vf[0] < 0.15:
                continue
            v0 = float(vf[0])
            t90, t10 = _cross_time(t, vf, 0.9 * v0), _cross_time(t, vf, 0.1 * v0)
            # 90% 那個點落在第一個取樣之前 (指令一變就開始減速) -> 從段落起點算
            if t10 is None:
                continue
            ta, va = (t90, 0.9 * v0) if t90 is not None else (0.0, v0)
            if t10 - ta < 1e-3:
                continue
            mid = (t >= ta) & (t <= t10) & (vf > 0.05)
            if mid.sum() < min_rows:
                continue
            F['brake_decel'].append((va - 0.1 * v0) / (t10 - ta))
            if g['wheel_fwd'].notna().any():
                rw = r_wheel * g.loc[mid, 'wheel_fwd'].to_numpy()
                F['brake_slip'].append(float(np.median(np.clip(1.0 - rw / vf[mid], 0, 1))))

        elif b == 'B4 launch':
            vf = g['vf'].to_numpy()
            v_t = r_wheel * float(g['thr'].abs().max())
            if not np.isfinite(vf).all() or v_t < 0.1:
                continue
            # 10% -> 60% 目標車速花多久。上面那 40% 不取: 快到速時輪子不滑了,
            # 加速度由速度環的收斂決定, 不是摩擦。
            t10, t60 = _cross_time(t, -vf, -0.1 * v_t), _cross_time(t, -vf, -0.6 * v_t)
            if t60 is None:
                continue
            ta = t10 if t10 is not None else 0.0
            if t60 - ta < 1e-3 or ((t >= ta) & (t <= t60)).sum() < min_rows:
                continue
            F['launch_accel'].append((0.6 * v_t - (0.1 * v_t if t10 is not None else 0.0))
                                     / (t60 - ta))
            early = (t <= t60) & (g['wheel_fwd'].to_numpy() * r_wheel > 0.1)
            if early.sum() >= 2:
                rw = r_wheel * g.loc[early, 'wheel_fwd'].to_numpy()
                F['launch_slip'].append(float(np.median(np.clip((rw - vf[early]) / rw, 0, 1))))

    # ── 扣掉馬達自己的損耗 (B3 量到的), 剩下的才是輪胎的摩擦力 ──
    loss = fit_loss(raw['cruise']) if loss_correction else None
    for cmd, tq, w_body, wheel, sat in raw['spin']:
        net = tq - loss_at(loss, wheel)
        F['spin_torque'].append(net)
        F['_spin_rows'].append((cmd, net, w_body, wheel, sat, tq))
    F['ramp_torque'] = [tq - loss_at(loss, wh) for tq, wh in raw['ramp']]
    F['breakaway_torque'] = [tq - loss_at(loss, wh) for tq, wh in raw['breakaway']]

    rows = F['_spin_rows']
    spin_w = [r[3] for r in rows]
    F['_meta'] = {
        'excluded': excluded,
        'spin_trials': n_spin,
        'no_effort': n_no_effort,
        'saturated': sum(1 for r in rows if r[4]),
        'sat_frac': (sum(1 for r in rows if r[4]) / len(rows)) if rows else 0.0,
        # 連輪速最低的那一檔都飽和 = 降低轉速也救不了 (馬達的上限在低轉速就是堵轉扭矩,
        # 不會再變大) -> 這個地面超出這台車量得到的範圍。
        'out_of_range': bool(rows) and all(r[4] for r in rows
                                           if r[0] <= min(q[0] for q in rows) + 0.05),
        'stall_torque': motor.stall,
        'spin_torque_raw': float(np.median([r[5] for r in rows])) if rows else None,
        # 扭矩回授全是 0 = 發布端沒填 (不是摩擦力真的是 0)
        'effort_zero': bool(rows) and max(r[5] for r in rows) < 1e-4,
        # 損耗模型 loss(w) = c0 + c1*w [N·m]; None = 沒有 B3 的資料 (或被關掉), 沒扣
        'loss': None if loss is None else {
            'c0': loss[0], 'c1': loss[1], 'n': len(raw['cruise']),
            'cruise_speeds': sorted({round(r[0], 1) for r in raw['cruise']}),
            # 自旋輪速範圍裡的損耗, 以及它佔原始扭矩的比例 (太大 = 結果很吃這個修正)
            'at_spin': [loss_at(loss, min(spin_w)), loss_at(loss, max(spin_w))] if spin_w else None,
            'frac_of_spin': (float(np.median([loss_at(loss, r[3]) / r[5] for r in rows if r[5] > 1e-6]))
                             if rows else None),
        },
        'loss_requested': bool(loss_correction),
    }
    return F


def stat(F, key, idx=None):
    """一個環境的一個特徵的代表值 (idx = bootstrap 抽樣的 trial 索引)。"""
    v = np.asarray(F[key], float)
    if idx is not None:
        v = v[idx]
    return float(np.median(v)) if len(v) else np.nan


def n_trials(F, key):
    return len(F[key])


def _cv(values):
    v = np.asarray(values, float)
    return float(np.std(v, ddof=1) / abs(np.mean(v))) if len(v) >= 2 and np.mean(v) else 0.0


# ═══════════════════════════════════════════════════════════════════════
#  比較
# ═══════════════════════════════════════════════════════════════════════
def compare(Fr, Ft, n_boot, rng):
    res = {}
    boots = {}
    for key, (desc, role) in FEATURES.items():
        nr, nt = n_trials(Fr, key), n_trials(Ft, key)
        if nr == 0 or nt == 0:
            continue
        ref, tgt = stat(Fr, key), stat(Ft, key)
        if not (ref > 0 and tgt > 0):
            continue
        lr = np.empty(n_boot)
        for b in range(n_boot):
            sr = stat(Fr, key, rng.integers(0, nr, nr))
            st = stat(Ft, key, rng.integers(0, nt, nt))
            lr[b] = np.log(st / sr) if sr > 0 and st > 0 else np.nan
        lr = lr[np.isfinite(lr)]
        ratio = tgt / ref
        res[key] = {
            'desc': desc, 'role': role, 'ref': ref, 'target': tgt,
            'n_ref': nr, 'n_target': nt, 'ratio': ratio,
            'ci90': [float(np.exp(np.percentile(lr, 5))), float(np.exp(np.percentile(lr, 95)))]
            if len(lr) > 20 else [np.nan, np.nan],
            # 只有 1~2 個 trial 時 bootstrap 變異數是 0, 給一個保守下限避免它獨占權重
            'log_sd': max(float(np.std(lr)) if len(lr) > 20 else 0.0,
                          0.3 / math.sqrt(min(nr, nt)), 0.02),
            # log 比值的標準誤, 用**實際的 trial 分散度**算 (中位數的標準誤 ≈ 1.2533 σ/√n)。
            # log_sd 的下限是給加權用的保守值, 拿來報解析度會悲觀好幾倍; 這個才是
            # 「這份資料能量多準」。每個 trial 的相對雜訊至少算 2% (n=2 時 std 不可靠)。
            'log_se': math.sqrt(sum((1.2533 * max(_cv(F[key]), 0.02)) ** 2 / n
                                    for F, n in ((Fr, nr), (Ft, nt)))),
        }
        boots[key] = lr

    prim = [k for k, r in res.items() if r['role'] == 'primary']
    combined = None
    if prim:
        w = np.array([1 / res[k]['log_sd'] ** 2 for k in prim])
        w /= w.sum()
        mean = sum(wi * math.log(res[k]['ratio']) for wi, k in zip(w, prim))
        m = min(len(boots[k]) for k in prim)
        comb_b = sum(wi * boots[k][:m] for wi, k in zip(w, prim)) if m > 20 else None
        # 特徵之間互相不一致時, 光看 bootstrap 會太樂觀 -> 把特徵間的離散也算進去
        spread = (np.std([math.log(res[k]['ratio']) for k in prim]) if len(prim) > 1 else 0.0)
        sd = math.sqrt((np.std(comb_b) if comb_b is not None else 0.1) ** 2 + spread ** 2)
        combined = {'ratio': math.exp(mean), 'ci90': [math.exp(mean - 1.645 * sd),
                                                        math.exp(mean + 1.645 * sd)],
                    'weights': dict(zip(prim, w.round(3).tolist())), 'features': prim}
    return res, combined


def ground_mu(mu_eff, wheel_mu, mode):
    """有效 mu -> 地面 mu (PhysX combine mode 的反函數)。解不出來回 nan。"""
    if mode == 'average':
        return 2 * mu_eff - wheel_mu
    if mode == 'multiply':
        return mu_eff / wheel_mu
    if mode == 'min':        # 有效值 < 輪子才是地面決定的; 否則地面只知道 >= 輪子
        return mu_eff if mu_eff < wheel_mu else np.nan
    if mode == 'max':
        return mu_eff if mu_eff > wheel_mu else np.nan
    raise ValueError(mode)


def effective_mu(mu_ground, wheel_mu, mode):
    return {'average': (mu_ground + wheel_mu) / 2, 'multiply': mu_ground * wheel_mu,
            'min': min(mu_ground, wheel_mu), 'max': max(mu_ground, wheel_mu)}[mode]


# PhysX: 兩個材質的 combine mode 不同時, 取優先權高的 (average < min < multiply < max)
COMBINE_PRIORITY = ['average', 'min', 'multiply', 'max']
# Omni PhysX 沒有綁物理材質時的預設材質
PHYSX_DEFAULT_MATERIAL = {'static': 0.5, 'dynamic': 0.5, 'combine': 'average'}


def read_usd_friction(usd_path, ground_prim, wheel_pattern):
    """從 USD 讀地面與輪子的物理材質: {'ground': {...}, 'wheel': {...}, 'combine': str}。

    綁定解析跟 PhysX 一樣: 先找 material:binding:physics, 再找 material:binding,
    自己沒有就往上找祖先; 綁到的材質沒有 PhysicsMaterialAPI (只有外觀的材質) 就當
    沒綁 -> PhysicsScene 的預設材質 -> 0.5/0.5。

    輪子是用 prim **路徑**比對的, 而且會走進 instance proxy: URDF 匯入的車
    (kros_car.usd) 輪子的碰撞體是 /kros_car/new_wheel_v1_1/collisions/mesh_0/cylinder,
    名字本身沒有 wheel, 又在 instanceable 的子樹裡, 一般的 Traverse 看不到。

    需要 pxr (Isaac Sim 的 python 有; 一般環境 `pip install usd-core`)。"""
    try:
        from pxr import Usd
    except ImportError:
        sys.exit('--ref-usd 需要 pxr: `pip install usd-core` (或用 Isaac Sim 的 python), '
                 '不然就改用 --ref-ground-static / --ref-ground-dynamic 直接給數字')
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        sys.exit(f'開不了 USD: {usd_path}')

    def material_of(prim):
        p = prim
        while p and p.IsValid() and not p.IsPseudoRoot():
            for rel_name in ('material:binding:physics', 'material:binding'):
                rel = p.GetRelationship(rel_name)
                if rel and rel.GetTargets():
                    m = stage.GetPrimAtPath(rel.GetTargets()[0])
                    if m.IsValid() and m.GetAttribute('physics:staticFriction').IsValid():
                        return m
            p = p.GetParent()
        return None

    def props(m):
        if m is None:
            return None
        def get(name, default):
            a = m.GetAttribute(name)
            v = a.Get() if a.IsValid() else None
            return default if v is None else v
        return {'path': str(m.GetPath()),
                'static': float(get('physics:staticFriction', 0.0)),
                'dynamic': float(get('physics:dynamicFriction', 0.0)),
                'combine': str(get('physxMaterial:frictionCombineMode', 'average'))}

    default = dict(PHYSX_DEFAULT_MATERIAL, path='(PhysX 預設材質)')
    for prim in stage.Traverse():
        if prim.GetTypeName() == 'PhysicsScene':
            default = props(material_of(prim)) or default

    g = stage.GetPrimAtPath(ground_prim)
    if not g.IsValid():
        sys.exit(f'{usd_path}: 找不到地面 prim {ground_prim} (用 --ground-prim 指定)')
    ground = props(material_of(g))
    if ground is None:
        ground = dict(default)
        print(f'! {usd_path}: 地面 {ground_prim} 沒綁物理材質, 用預設 {ground["static"]}')

    pat = re.compile(wheel_pattern, re.I)
    wheels = [p for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
              if pat.search(str(p.GetPath())) and p.HasAPI(_collision_api())]
    if not wheels:
        sys.exit(f'{usd_path}: 找不到路徑符合 /{wheel_pattern}/ 的碰撞體 (用 --wheel-pattern 指定)')
    mats = {}
    for w in wheels:
        m = props(material_of(w)) or dict(default)
        mats[(m['static'], m['dynamic'], m['combine'])] = m
    if len(mats) > 1:
        print(f'! {usd_path}: 輪子材質不一致 {list(mats)}, 用第一個')
    wheel = next(iter(mats.values()))
    combine = max(ground['combine'], wheel['combine'], key=COMBINE_PRIORITY.index)
    return {'ground': ground, 'wheel': wheel, 'combine': combine,
            'n_wheels': len(wheels)}


def _collision_api():
    from pxr import UsdPhysics
    return UsdPhysics.CollisionAPI


# 地面的哪個係數由哪個特徵決定。B1 是在滑動中量的 -> 動摩擦; B2/B1 spin_up 是
# 「從靜止推到開始動」-> 靜摩擦。
# 地面的哪個係數由哪個特徵決定。B1 是在滑動中量的 -> 動摩擦; B2 的起轉峰值是
# 「從靜止推到開始動」-> 靜摩擦。
GROUND_TERMS = [
    ('dynamic', 'physics:dynamicFriction', ['spin_torque', 'ramp_torque']),
    ('static', 'physics:staticFriction', ['breakaway_torque']),
]


def ground_report(res, args, p=print):
    """把特徵比值換算成目標環境的地面摩擦係數 (靜 / 動 各一個)。"""
    ref = args.ref_material
    if ref is None:
        p('  (要換算地面摩擦係數, 給 --ref-usd <參考環境.usd> 或 --ref-ground-static/-dynamic)')
        return None
    mode = ref['combine']
    s = args.ratio_scale
    p('═' * 78)
    p(f'★ 目標環境地面摩擦係數   (PhysX combine={mode}, 輪子 static {ref["wheel"]["static"]:g} '
      f'/ dynamic {ref["wheel"]["dynamic"]:g}' + (f', 比值校正 x{s:g}' if s != 1 else '') + ')')
    out = {}
    for term, attr, keys in GROUND_TERMS:
        key = next((k for k in keys if k in res), None)
        if key is None:
            p(f'  {attr:<26} 沒有可用的特徵 ({"/".join(keys)})')
            continue
        r = res[key]
        # 信賴區間: bootstrap 與「trial 太少時的保守下限」取寬的那個
        # (B2 只有幾個 trial, bootstrap 會窄得不合理)
        lo_b, hi_b = r['ci90']
        k = math.exp(1.645 * r['log_sd'])
        lo = min(x for x in (lo_b, r['ratio'] / k) if np.isfinite(x))
        hi = max(x for x in (hi_b, r['ratio'] * k) if np.isfinite(x))
        g_ref, w = ref['ground'][term], ref['wheel'][term]
        eff_ref = effective_mu(g_ref, w, mode)
        vals = [ground_mu(eff_ref * x * s, w, mode) for x in (r['ratio'], lo, hi)]
        p(f'  {attr:<26} ≈ {vals[0]:6.2f}   90% CI [{vals[1]:.2f}, {vals[2]:.2f}]'
          f'   (參考 {g_ref:g}, {key} 比值 {r["ratio"]:.2f})')
        if not np.isfinite(vals[0]):
            p(f'    ! combine={mode} 下地面不是瓶頸 (輪子 {w:g} 那側決定), 只能知道地面 '
              f'{"≥" if mode == "min" else "≤"} {w:g}')
        elif vals[1] < 0:
            p('    ! 信賴區間下界是負的: 有效摩擦接近輪子那一側, 地面值對它不敏感, 數字不可靠')
        if key != keys[0]:
            p(f'    (主特徵沒資料, 用副特徵 {key}, 準度較差)')
        if np.isfinite(vals[0]) and lo <= 1.0 <= hi:
            # 比值跟 1 分不出來 -> 點估計只是雜訊, 不要照它去改參數
            p(f'    => 跟參考**分不出差別** (比值 CI [{lo:.2f}, {hi:.2f}] 含 1): 維持參考值 {g_ref:g}。'
              f'這份資料能分辨的地面差異約 ±{0.5 * (vals[2] - vals[1]):.2f}, '
              f'比這小的差距要更多 trial 才量得出來')
        out[term] = {'value': vals[0], 'ci90': vals[1:], 'feature': key, 'ratio': r['ratio'],
                     'distinguishable': not (lo <= 1.0 <= hi)}
    if out and not any(o['distinguishable'] for o in out.values()):
        p(f'  結論: 目標環境的地面摩擦跟參考 ({ref["ground"]["dynamic"]:g}) 量不出差別, 不需要調整。')
    elif len(out) == 2:
        a, b = out['static']['value'], out['dynamic']['value']
        if np.isfinite(a) and np.isfinite(b):
            if abs(a - b) <= 0.15 * max(a, b):
                p(f'  靜 / 動差不多 -> 只想填一個數字的話: {0.5 * (a + b):.2f}')
            elif b > a:
                p('  ! 動摩擦 > 靜摩擦: 物理上不合理, 通常是 B2 的起轉峰值沒被取樣到, 以動摩擦為準')
    if mode == 'average':
        p('  註: average 模式下 地面 = 2 x 有效值 - 輪子, 比值的誤差換到地面會放大約 2 倍。')
    return out


def verdict(ci, ratio, tol):
    lo, hi = ci
    if np.isfinite(lo) and hi < 1 - tol:
        return '變小'
    if np.isfinite(lo) and lo > 1 + tol:
        return '變大'
    if abs(math.log(ratio)) < math.log(1 + tol):
        return '差不多 (在容許誤差內)'
    return f'{"偏小" if ratio < 1 else "偏大"}, 但信賴區間跨過 1, 資料不足以確定'


def motor_saturated(*Fs):
    """任何一邊的自旋 trial 有超過 MAX_SAT_FRAC 馬達飽和 -> 主特徵不能當摩擦力用。"""
    return any(F['_meta']['sat_frac'] > MAX_SAT_FRAC for F in Fs)


def use_clean_levels(Fr, Ft):
    """B1 只留**兩邊都沒飽和**的輪速檔。回傳 (Fr, Ft); 沒有東西要拿掉就原樣回傳。

    馬達出得了的扭矩隨轉速下降, 所以飽和幾乎都從最高那一檔開始。整份資料丟掉太可惜:
    低的那幾檔還是乾淨的 (2026-10-13 無頭 PhysX, 地面 0.5 -> 1.0, 輪速 2.4 / 4.2 / 6.0:
    最高檔飽和, 三檔全用比值 1.54, 只用乾淨的兩檔 1.50, 真值 1.50)。
    兩邊要拿掉同樣的檔位 —— 扭矩隨輪速有幾個百分點的變化, 只拿掉一邊會變成拿
    不同轉速在比。剩不到 3 趟就不處理, 讓後面照「飽和」報。
    """
    def by_level(F):
        out = {}
        for r in F['_spin_rows']:
            out.setdefault(round(r[0], 2), []).append(bool(r[4]))
        return out

    bad = sorted({c for F in (Fr, Ft) for c, v in by_level(F).items()
                  if np.mean(v) > MAX_SAT_FRAC})
    if not bad:
        return Fr, Ft

    def keep(F):
        rows = [r for r in F['_spin_rows'] if round(r[0], 2) not in bad]
        G = dict(F)
        G['_spin_rows_all'] = F['_spin_rows']
        G['_spin_rows'] = rows
        G['spin_torque'] = [r[1] for r in rows]
        G['spin_yaw_gain'] = [r[2] / r[3] for r in rows if r[3] > 0.5]
        G['_meta'] = dict(F['_meta'], dropped_levels=bad,
                          saturated=sum(1 for r in rows if r[4]),
                          sat_frac=(sum(1 for r in rows if r[4]) / len(rows)) if rows else 0.0)
        return G

    Gr, Gt = keep(Fr), keep(Ft)
    if min(len(Gr['_spin_rows']), len(Gt['_spin_rows'])) < 3:
        return Fr, Ft
    return Gr, Gt


def spin_table(F, r_wheel, p=print, tag=''):
    """B1 逐檔輪速的扭矩 / 車體轉速 —— 驗證 Coulomb 假設 (扭矩跟轉速無關) 用。"""
    rows = F.get('_spin_rows_all', F['_spin_rows'])
    if not rows:
        return
    dropped = F['_meta'].get('dropped_levels', [])
    a = np.asarray([r[:4] for r in rows], float)
    sat = np.asarray([r[4] for r in rows], bool)
    lost = np.asarray([r[5] - r[1] for r in rows], float)
    p(f'  {tag}B1 逐檔: {"輪速指令":>8}{"實際輪速":>9}{"車體轉速":>9}{"扭矩":>8}{"(已扣損耗)":>9}   n  飽和')
    for c in sorted(set(np.round(a[:, 0], 2))):
        k = np.abs(a[:, 0] - c) < 0.02
        p(f'  {"":<{len(tag) + 9}}{c:>10.2f}{np.median(a[k, 3]):>11.2f}{np.median(a[k, 2]):>11.2f}'
          f'{np.median(a[k, 1]):>10.3f}{np.median(lost[k]):>12.3f}  {k.sum():>2}  {int(sat[k].sum())}'
          + ('   <- 這一檔不用' if round(float(c), 2) in dropped else ''))
    gain = np.asarray(F['spin_yaw_gain'], float)
    if len(gain):
        g = float(np.median(gain))
        p(f'  {"":<{len(tag) + 9}}車體轉速/輪速 = {g:.4f}  ->  等效輪距 T_eff = 2r/gain = '
          f'{2 * r_wheel / g:.3f} m' if g > 1e-6 else '  車體沒在轉')


def report(ref_path, tgt_path, res, combined, Fr, Ft, args):
    p = print
    p('═' * 78)
    p(f'參考: {ref_path}')
    p(f'目標: {tgt_path}')
    p(f'車體運動來源: {args.source}   (扭矩一律來自輪子回授的 *_effort)')
    problems = []
    for tag, path, F in (('參考', ref_path, Fr), ('目標', tgt_path, Ft)):
        q = args.loc_quality.get(path)
        if q is not None:
            msg, bad = q
            p(f'{"!" if bad else " "} {tag}收資料時的定位: {msg}'
              + ('  <- 定位壞了, 控制是在錯的位置 / 朝向跑的' if bad else ''))
        m = F['_meta']
        if m['excluded']:
            p(f'! {tag}排除 {len(m["excluded"])} 趟不是直線的 B3/B4: '
              + ', '.join(m['excluded'][:4]) + (' ...' if len(m['excluded']) > 4 else ''))
        if m['spin_trials'] and m['no_effort'] == m['spin_trials']:
            problems.append(f'{tag}沒有扭矩回授 (*_effort 全是 NaN)。模擬: /joint_states 有沒有 effort? '
                            '實體車: collect_data_node 要給 wheel_feedback_topic:=/wheel_feedback')
        elif m['effort_zero']:
            problems.append(f'{tag}的扭矩回授全是 0 —— 發布端沒填 effort, 不是摩擦力真的是 0')
        elif m['sat_frac'] > MAX_SAT_FRAC:
            problems.append(
                f'{tag}有 {m["saturated"]}/{len(F["_spin_rows"])} 趟自旋**馬達飽和** (扭矩貼著那個轉速下的'
                '上限, 或輪速追不上指令): 量到的是馬達上限不是摩擦力, 比值不能信。'
                + (f'**連最低那一檔都飽和, 再調低輪速也沒用** (低轉速時馬達的上限就是堵轉扭矩 '
                   f'{m["stall_torque"]:g} N·m): 四輪平均要 {m["spin_torque_raw"]:.2f} N·m, 而載重會在'
                   '對角輪之間跳, 個別輪子瞬間要的是平均的 1.4 倍以上。這個地面的摩擦力超出這台車 '
                   '(車重 / 馬達) 量得到的範圍 —— 換一個摩擦小一點的地面, 或確認車重'
                   if m['out_of_range'] else
                   '把 spin_wheel_speed 調低再收一次 (轉得慢, 馬達出得了的扭矩才多)'))
        ls = m['loss']
        if ls is not None:
            lo, hi = ls['at_spin'] or (float('nan'),) * 2
            p(f'  {tag}馬達損耗 (B3 直行 {ls["n"]} 趟, 輪速 {ls["cruise_speeds"]}): '
              f'loss(w) = {ls["c0"]:.3f} + {ls["c1"]:.4f}·w N·m -> 自旋輪速下 {lo:.3f}~{hi:.3f}, '
              f'佔原始扭矩 {100 * (ls["frac_of_spin"] or 0):.0f}%, 已扣掉')
        elif m['loss_requested'] and m['spin_trials']:
            problems.append(f'{tag}沒有 B3 等速直行的資料 -> **馬達損耗沒有扣**。實體車的扭矩回授'
                            '含馬達自己的損耗, 不扣會高估摩擦力; 兩邊要一致 (都扣或都不扣)')
    dropped = Ft['_meta'].get('dropped_levels')
    if dropped:
        problems.append(
            f'B1 輪速 {", ".join(f"{c:g}" for c in dropped)} rad/s 那幾檔馬達飽和 (扭矩貼著那個轉速下的'
            '上限), 兩邊都拿掉, 只用剩下的檔位。下次把 spin_wheel_speed 調低, 三檔才都用得上')
    for x in problems:
        p(f'! {x}')
    p('─' * 78)
    spin_table(Fr, args.wheel_radius, p, '參考 ')
    spin_table(Ft, args.wheel_radius, p, '目標 ')
    p('─' * 78)
    p(f'{"特徵":<44}{"參考":>8}{"目標":>8}{"比值":>7}   90% CI      n')
    for role, title in (('primary', '主特徵 (∝ mu)'), ('secondary', '副特徵'),
                        ('control', '對照組 (跟摩擦無關, 應≈1)')):
        rows = [(k, r) for k, r in res.items() if r['role'] == role]
        if not rows:
            continue
        p(f'  [{title}]')
        for k, r in rows:
            lo, hi = r['ci90']
            p(f'  {r["desc"]:<42}{r["ref"]:>8.3f}{r["target"]:>8.3f}{r["ratio"]:>7.2f}'
              f'   [{lo:.2f}, {hi:.2f}]  {r["n_ref"]}/{r["n_target"]}')
    missing = [FEATURES[k][0] for k in FEATURES if k not in res]
    if missing:
        p(f'  (沒資料, 跳過: {", ".join(missing)})')
    p('─' * 78)

    if combined is None:
        p('沒有主特徵 (B1 spin_hold 的輪子扭矩) 可用, 無法估計。')
        return None

    r, (lo, hi) = combined['ratio'], combined['ci90']
    p(f'綜合 (主特徵加權 {combined["weights"]}):')
    p(f'  有效摩擦係數 目標/參考 = {r:.2f}   90% CI [{lo:.2f}, {hi:.2f}]')
    if motor_saturated(Fr, Ft):
        # 不下「至少 / 至多」的結論: 部分飽和時整台車的受力分布跟著變, 沒飽和的那幾輪
        # 扭矩反而變大, 比值不保證偏向哪一邊 (實測真值 1.50 量到 1.58)。
        sr, st = Fr['_meta']['sat_frac'] > MAX_SAT_FRAC, Ft['_meta']['sat_frac'] > MAX_SAT_FRAC
        p('  => **馬達飽和, 這個比值不是摩擦力的比值**: '
          + ('兩邊都飽和, 無法判斷。' if sr and st else
             '目標飽和。' if st else '參考飽和。')
          + (' 連最低那一檔都飽和, 超出這台車量得到的範圍 (見上面的 ! 訊息);'
             if Ft['_meta']['out_of_range'] or Fr['_meta']['out_of_range'] else
             ' 把 spin_wheel_speed 調低重收;')
          + ' 這個數字不要拿來用。')
    else:
        p(f'  => 摩擦力{verdict(combined["ci90"], r, args.tol)},  約 {100 * (r - 1):+.0f}%')

    warn = []
    if 'ramp_torque' in res and 'spin_torque' in res:
        a, b = res['spin_torque']['ratio'], res['ramp_torque']['ratio']
        if max(a, b) / min(a, b) > 1.3:
            warn.append(f'B1 與 B2 的扭矩比值差很多 ({a:.2f} vs {b:.2f}): 扭矩跟轉速有關 '
                        '(黏滯項 / 飽和), 不是純 Coulomb。')
    for k, x in res.items():
        if x['role'] == 'control' and abs(math.log(x['ratio'])) > math.log(1.2):
            warn.append(f'對照組「{x["desc"]}」比值 {x["ratio"]:.2f} 偏離 1 —— '
                        '兩個環境不只摩擦不同 (幾何 / 載重 / 馬達)。')
    for key, slip in (('brake_decel', 'brake_slip'), ('launch_accel', 'launch_slip')):
        # 綜合比值本身就接近 1 的時候不判斷: 兩邊摩擦差不多, 這個特徵的比值當然也
        # 接近 1, 那不是「被壓向 1」(2026-10-13: 1.04 vs 1.00 就跳了這個警告)。
        if (key in res and abs(math.log(r)) > math.log(1.15)
                and abs(math.log(res[key]['ratio'])) < 0.5 * abs(math.log(r))):
            s = res.get(slip)
            warn.append(f'{FEATURES[key][0]} 的比值被壓向 1: 至少一邊是扭矩受限 (輪子沒在滑'
                        + (f', slip 參考 {s["ref"]:.2f} / 目標 {s["target"]:.2f}' if s else '')
                        + '), 這個特徵在這台車上不帶摩擦資訊。')
    for w in warn:
        p(f'  ! {w}')

    out = {'ratio': r, 'ci90': [lo, hi]}
    if args.ref_mu is not None:
        mu = args.ref_mu * r
        p(f'  參考有效 mu = {args.ref_mu:g}  ->  目標有效 mu ≈ {mu:.3f}  '
          f'[{args.ref_mu * lo:.3f}, {args.ref_mu * hi:.3f}]')
        out['target_mu_eff'] = mu
    return out


def plot(ref_path, tgt_path, Fr, Ft, res, outdir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(outdir, exist_ok=True)
    keys = list(res)
    fig, axes = plt.subplots(1, len(keys) + 1, figsize=(3.2 * (len(keys) + 1), 3.6))
    for ax, k in zip(axes, keys):
        for j, (F, lab) in enumerate(((Fr, 'ref'), (Ft, 'target'))):
            v = np.asarray(F[k])
            ax.scatter(np.full(len(v), j) + np.random.uniform(-.1, .1, len(v)), v, s=12)
            ax.hlines(stat(F, k), j - .25, j + .25, color='k')
        ax.set_xticks([0, 1], ['ref', 'target'])
        ax.set_title(f'{k}\nratio {res[k]["ratio"]:.2f}', fontsize=9)
    ax = axes[-1]
    for F, lab in ((Fr, 'ref'), (Ft, 'target')):
        a = np.asarray([r[:4] for r in F['_spin_rows']], float)
        if len(a):
            ax.scatter(a[:, 0], a[:, 1], s=12, label=f'{lab} tau={stat(F, "spin_torque"):.2f}')
    ax.set_xlabel('wheel speed cmd [rad/s]')
    ax.set_ylabel('wheel torque [N m]')
    ax.legend(fontsize=8)
    ax.set_title('B1: torque vs wheel speed', fontsize=9)
    fig.tight_layout()
    name = f'{os.path.splitext(os.path.basename(ref_path))[0]}_vs_' \
           f'{os.path.splitext(os.path.basename(tgt_path))[0]}.png'
    fig.savefig(os.path.join(outdir, name), dpi=120)
    print(f'  圖: {os.path.join(outdir, name)}')


def resolve_ref_material(args):
    """--ref-usd 讀到的值, 再用命令列數字覆蓋。什麼都沒給 -> None (不換算地面值)。"""
    numbers = (args.ref_ground_mu, args.ref_ground_static, args.ref_ground_dynamic)
    if args.ref_usd:
        m = read_usd_friction(args.ref_usd, args.ground_prim, args.wheel_pattern)
        print(f'參考 USD {args.ref_usd}:\n'
              f'  地面 {m["ground"]["path"]}: static {m["ground"]["static"]:g}, '
              f'dynamic {m["ground"]["dynamic"]:g}, combine {m["ground"]["combine"]}\n'
              f'  輪子 ({m["n_wheels"]} 個) {m["wheel"]["path"]}: static {m["wheel"]["static"]:g}, '
              f'dynamic {m["wheel"]["dynamic"]:g}, combine {m["wheel"]["combine"]}\n'
              f'  -> 生效的 combine mode: {m["combine"]}')
    elif any(x is not None for x in numbers):
        m = {'ground': {}, 'wheel': dict(PHYSX_DEFAULT_MATERIAL), 'combine': 'average'}
    else:
        return None
    for side, both, st, dy in (('ground', args.ref_ground_mu, args.ref_ground_static,
                                args.ref_ground_dynamic),
                               ('wheel', args.wheel_mu, args.wheel_static, args.wheel_dynamic)):
        for term, v in (('static', st), ('dynamic', dy)):
            v = v if v is not None else both
            if v is not None:
                m[side][term] = v
    if args.combine:
        m['combine'] = args.combine
    missing = [t for t in ('static', 'dynamic') if t not in m['ground']]
    if missing:
        sys.exit(f'參考地面缺 {missing}: 給 --ref-ground-mu 或 --ref-ground-static/-dynamic')
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ref', help='參考環境的 CSV')
    ap.add_argument('targets', nargs='+', help='要跟參考比較的 CSV (可多個)')
    ap.add_argument('--source', default='auto',
                    choices=['auto', 'gyro', 'gt', 'fus', 'lid', 'cam', 'imu', 'whl'],
                    help='車體運動的來源 (預設 auto: 有 _imu.csv 就用 gyro, 否則 gt)')
    ap.add_argument('--compare-gt', action='store_true',
                    help='同一份資料再用 GT 算一次並排印 (驗證感測器流程)')
    ap.add_argument('--gyro-noise', type=float, default=0.0,
                    help='注入 gyro 白雜訊 std [rad/s] (每個樣本)')
    ap.add_argument('--gyro-bias', type=float, default=0.0, help='注入 gyro 零偏 [rad/s]')
    ap.add_argument('--acc-noise', type=float, default=0.0, help='注入加速度白雜訊 std [m/s²]')
    ap.add_argument('--acc-bias', type=float, default=0.0, help='注入前向加速度零偏 [m/s²]')
    ap.add_argument('--forward-axis-deg', type=float, default=0.0,
                    help='車頭相對 base_link 的角度, 跟 control_car_node 的參數一致 '
                         '(kros_car 是 0; 舊的 car.usd 是 -90)')
    ap.add_argument('--wheel-radius', type=float, default=0.05035,
                    help='輪半徑 [m] (算 slip ratio 與等效輪距用)')
    ap.add_argument('--torque-limit', type=float, default=2.0,
                    help='馬達堵轉扭矩 [N·m]。自旋扭矩貼著上限 = 飽和, 那一趟量到的不是摩擦')
    ap.add_argument('--no-load-speed', type=float, default=20.94,
                    help='馬達空載轉速 [rad/s] (DDSM115: 200 rpm)。0 = 上限不隨轉速變')
    ap.add_argument('--rated-speed', type=float, default=12.04,
                    help='額定轉速 [rad/s] (DDSM115: 115 rpm)')
    ap.add_argument('--rated-torque', type=float, default=0.96,
                    help='額定轉速下的扭矩 [N·m] (DDSM115: 0.96)')
    ap.add_argument('--no-loss-correction', action='store_true',
                    help='不要用 B3 等速直行的扭矩去扣馬達損耗')
    ap.add_argument('--max-sigma', type=float, default=None,
                    help='感測器 _sigma 上限 (lid 預設 0.0025, 其他不濾)')
    ap.add_argument('--max-age', type=float, default=0.3, help='感測器 _age 上限 [s]')
    ap.add_argument('--tol', type=float, default=0.05,
                    help='比值在 1±tol 之內算「差不多」')
    ap.add_argument('--boot', type=int, default=1000, help='bootstrap 次數')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--ref-mu', type=float, default=None, help='參考環境的有效 mu (已知的話)')
    g = ap.add_argument_group(
        '地面摩擦係數換算 (PhysX)',
        '參考環境的材質從 --ref-usd 讀, 或用數字直接給 (數字會覆蓋 USD 讀到的值)')
    g.add_argument('--ref-usd', help='參考環境的 USD, 自動讀地面 / 輪子材質與 combine mode')
    g.add_argument('--ground-prim', default='/Environment/groundCollider',
                   help='地面碰撞體的 prim 路徑')
    g.add_argument('--wheel-pattern', default='wheel',
                   help='輪子碰撞體 prim 路徑的 regex (不分大小寫)')
    g.add_argument('--ref-ground-mu', type=float, default=None,
                   help='參考環境地面 mu (靜 / 動相同時的簡寫)')
    g.add_argument('--ref-ground-static', type=float, default=None)
    g.add_argument('--ref-ground-dynamic', type=float, default=None)
    g.add_argument('--wheel-mu', type=float, default=None,
                   help='輪子材質 mu (靜 / 動相同時的簡寫; 沒給 USD 時預設 0.5)')
    g.add_argument('--wheel-static', type=float, default=None)
    g.add_argument('--wheel-dynamic', type=float, default=None)
    g.add_argument('--combine', default=None, choices=COMBINE_PRIORITY,
                   help='friction combine mode (沒給 USD 時預設 average)')
    g.add_argument('--ratio-scale', type=float, default=1.0,
                   help='比值校正係數。用已知 mu 的環境量出系統性偏差後填 (真值比值 / 估計比值)')
    ap.add_argument('--plot', metavar='DIR', help='輸出逐 trial 散佈圖')
    ap.add_argument('--json', metavar='FILE', help='把結果寫成 JSON')
    args = ap.parse_args()
    args.ref_material = resolve_ref_material(args)
    paths = list(dict.fromkeys([args.ref] + args.targets))
    args.loc_quality = {}
    for pth in paths:
        try:
            args.loc_quality[pth] = loc_quality(pth)
        except (ValueError, KeyError) as e:          # 舊格式的 CSV 缺欄位
            print(f'  ({pth}: 定位品質檢查略過: {e})')
    if args.source == 'auto':
        have_imu = all(os.path.exists(imu_path_for(p)) for p in paths)
        args.source = 'gyro' if have_imu else 'gt'
        print(f'--source auto -> {args.source}'
              + ('' if have_imu else ' (找不到 *_imu.csv; 舊資料或 imu_csv:=false)'))
    if args.max_sigma is None and args.source == 'lid':
        args.max_sigma = 0.0025
    if args.source == 'whl':
        print('! --source whl: 輪速里程計在打滑時是錯的, 而打滑就是摩擦造成的 (循環論證)。')
    noise = None
    if any((args.gyro_noise, args.gyro_bias, args.acc_noise, args.acc_bias)):
        if args.source != 'gyro':
            sys.exit('--gyro-noise / --acc-noise 只對 --source gyro 有意義')
        noise = {'gyro': args.gyro_noise, 'gyro_bias': args.gyro_bias,
                 'acc': args.acc_noise, 'acc_bias': args.acc_bias}

    rng = np.random.default_rng(args.seed)

    def load(p, source):
        if source == 'gyro':
            ip = imu_path_for(p)
            if not os.path.exists(ip):
                sys.exit(f'找不到 {ip} (--source gyro 需要 collect_data_node 的 IMU 全速率檔)')
            tr = load_imu_track(ip, args.forward_axis_deg, noise, rng)
            print(f'  {ip}: {len(tr)} 筆 IMU, 估得 gyro 零偏 {tr.attrs["gyro_bias"] * 1e3:+.2f} mrad/s')
            return tr
        return load_track(p, source, args.forward_axis_deg, args.max_sigma, args.max_age)

    def run(source):
        motor = Motor(args.torque_limit, args.no_load_speed, args.rated_speed,
                      args.rated_torque)
        return {p: extract(load(p, source), args.wheel_radius, motor,
                           loss_correction=not args.no_loss_correction)
                for p in paths}

    feats = run(args.source)
    gt_feats = None
    if args.compare_gt and args.source != 'gt':
        try:
            gt_feats = run('gt')
        except (KeyError, SystemExit):
            print('! --compare-gt: 資料裡沒有 GT (真車?), 略過')

    results = {}
    for tp in args.targets:
        Fr, Ft = use_clean_levels(feats[args.ref], feats[tp])
        res, combined = compare(Fr, Ft, args.boot, rng)
        out = report(args.ref, tp, res, combined, Fr, Ft, args)
        if gt_feats is not None and out is not None:
            gres, gcomb = compare(*use_clean_levels(gt_feats[args.ref], gt_feats[tp]),
                                  args.boot, rng)
            if gcomb:
                err = 100 * (combined['ratio'] / gcomb['ratio'] - 1)
                p = print
                p(f'  [GT 對照] 同一份資料用 GT 算: 比值 {gcomb["ratio"]:.2f} '
                  f'[{gcomb["ci90"][0]:.2f}, {gcomb["ci90"][1]:.2f}]  -> '
                  f'{args.source} 相對 GT 偏 {err:+.1f}%')
                for k in res:
                    if k in gres:
                        p(f'      {k:<18} {args.source} {res[k]["ratio"]:.2f}   gt {gres[k]["ratio"]:.2f}')
                out['gt_ratio'] = gcomb['ratio']
        if res and motor_saturated(Fr, Ft):
            print('═' * 78)
            print('★ 不換算地面摩擦係數: 馬達飽和, 量到的扭矩不是摩擦力 (見上面的 ! 訊息)。')
            ground = None
        else:
            ground = ground_report(res, args) if res else None
        if args.plot and res:
            plot(args.ref, tp, Fr, Ft, res, args.plot)
        lq = {k: (None if args.loc_quality.get(pth) is None else
                  {'message': args.loc_quality[pth][0], 'bad': bool(args.loc_quality[pth][1])})
              for k, pth in (('ref', args.ref), ('target', tp))}
        # 馬達飽和 / 沒有扭矩回授: friction_calib_step.py 靠這個決定資料能不能用
        motor = {k: {key: F['_meta'][key] for key in
                     ('spin_trials', 'no_effort', 'saturated', 'sat_frac', 'effort_zero',
                      'loss', 'out_of_range', 'stall_torque', 'spin_torque_raw')} | {'dropped_levels': F['_meta'].get('dropped_levels', [])}
                 for k, F in (('ref', Fr), ('target', Ft))}
        results[tp] = {'summary': out, 'combined': combined, 'features': res,
                       'ground_friction': ground, 'loc_quality': lq, 'motor': motor,
                       'excluded': {'ref': Fr['_meta']['excluded'],
                                    'target': Ft['_meta']['excluded']}}
    if args.json:
        with open(args.json, 'w') as f:
            json.dump({'ref': args.ref, 'source': args.source, 'results': results}, f,
                      indent=2, ensure_ascii=False, default=float)
        print(f'JSON: {args.json}')


if __name__ == '__main__':
    main()
