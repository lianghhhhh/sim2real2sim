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

────────────────────────────────────────────────────────────────────────
用了哪些特徵, 為什麼
────────────────────────────────────────────────────────────────────────
這台 skid-steer 原地轉時四輪都在橫向滑, 阻力矩 M = mu*m*g*(半軸距), 跟轉速無關
(Coulomb)。所以下面兩個量都**正比於 mu**, 取兩環境的比值就把車子的幾何、質量、
慣量全部消掉了, 不需要知道它們:

  [主] B1 spin_coast   斷油後的角減速度 alpha。  alpha = M / I_z  ∝ mu_dynamic
       放開之後轉速不是馬上開始等減速 (Isaac 60 Hz IMU 實測):
         1) 先再往上衝 30~70 ms —— 指令延遲 + 輪子轉得比車體快, 還在推車
         2) 過渡段 —— 輪子跟地面同步中, 減速度逐漸變大
         3) 線性段 —— 純 Coulomb, 斜率就是 alpha      <- 取這段
         4) 尾段 —— 低速時減速度又變小
       做法: 從 0.2 s 內的轉速峰值開始, 取 80%~20% 峰值之間的點做直線擬合。
       高 mu 地面整段只有 0.15~0.3 s, 需要 IMU 全速率檔 (60 Hz 以上) 才湊得到點;
       主 CSV 20 Hz 的位姿資料湊不滿就所有檔案一起退回能量法
           alpha = (w_peak² - w_end²) / (2 dθ)
       它把過渡段也平均進去, 高 mu 那邊 coast 短、過渡段佔比大 -> 比值被壓向 1。
       2026-09-15 那一輪 (ground 0.5 vs 2.0): 線性段 2.27, 能量法 gyro 1.98;
       GT 的 2.73 則是主 CSV 標籤晚了 50~67 ms、錯過轉最快那段造成的高估。

  [主] B2 creep_spin   steer effort 從 0 慢慢拉高, 車子開始轉的那一刻的 effort。
       驅動力矩 ∝ effort, 臨界時 = 靜摩擦力矩 ∝ mu_static。
       「角度超過門檻」的那一刻已經比真正起轉晚了, 而且這個延遲在兩個環境裡換算
       成 effort 差不多一樣 -> 加性偏差, 會把比值往 1 壓 (實測 2.07, 真值 2.5)。
       effort 線性 ramp -> 起轉後多出來的力矩 ∝ (t - t_b) -> 角度 ∝ (t - t_b)³,
       所以拿 ∛θ 對 t 做直線, 外插回 0 的時刻 t_b 才是起轉點 (實測 2.32)。

  [副] B1 spin_up      同上, 但 ramp 快 (6 effort/s), 20 Hz 下解析度只有 0.3。

  [副] B1 spinup_time  spin_up 從靜止到轉速第一次到 w_ref 的**時間**。
       這是**靜摩擦**特徵: 把兩個環境的 w(t) 疊起來看, 起轉之後的那一段幾乎
       重合 (Δt(2->5 rad/s) 對 mu 的敏感度只有 0.04~0.12, 因為那時是馬達扭矩
       說了算), 差異全部集中在「推得動之前卡住多久」。
       跟 creep_breakaway 比:
         敏感度  d ln t / d ln mu ≈ 0.47   (creep 是 ~0.8, 時間特徵比較鈍)
         精度    每個 trial 的 cv 0.6~0.9%, 一輪 15 個 trial -> 標準誤 0.2%
                 (creep 每個 trial 的散佈是 8%, 一輪 6 個 -> 4%)
       淨效果是解析度好 5~10 倍: 2026-09-15 的資料用它反推 real 的地面 mu 得到
       2.01 (真值 2.0), 用 creep 只能給 1.7~2.6 的區間。
       **代價: 它包含馬達 ramp, 所以兩個環境的驅動系統必須一樣。**
       sim-to-sim 校正 (同一台車只改地面) 成立; 對真車要先看 B3 對照組是不是 ≈ 1。
       比值**不正比於 mu** -> 不進綜合比值, 也不進 ★ 地面係數換算, 由校正迴圈
       (friction_calib_step.py) 用割線法直接內插。
  [副] B4 slip_launch  瞬間滿油門起步的平均加速度。抓地力受限時 = mu*g, 但扭矩上限
       ~3.4~6 m/s², 高 mu 的環境會被馬達卡住 -> 只能當下界, 比值會被壓向 1。
       另外印 slip ratio 當參考。

  [對照] B3 sprint_accel / B3 sprint_coast
       直線加速是扭矩受限、直線 coast 是輪子 joint damping 主導, 兩者理論上**跟地面
       摩擦無關**。它們的比值應該 ≈ 1; 如果差很多, 代表這兩個環境不只摩擦不同
       (馬達、質量、阻尼也變了), 主特徵的比值就不能直接解讀成摩擦比。

最後把 [主] 特徵的 log 比值用 bootstrap 變異數做反變異數加權, 得到綜合比值與信賴區間。

────────────────────────────────────────────────────────────────────────
「有效 mu」vs「地面 mu」 (Isaac / PhysX 限定)
────────────────────────────────────────────────────────────────────────
量到的是**輪子與地面合起來**的有效摩擦。PhysX 預設 combine mode 是 average:
    mu_eff = (mu_ground + mu_wheel) / 2
car_*.usd 的輪子沒綁物理材質 -> 用預設材質 0.5。所以 ground 0.5 -> 2.0 (4 倍)
量到的是 mu_eff 0.5 -> 1.25 (2.5 倍), 不是 4 倍。

**要的是地面摩擦係數就給參考環境的 USD**, 最後會印一個 ★ 區塊:

    ./scripts/estimate_friction.py sim.csv real.csv --ref-usd car_sim.usd

  physics:dynamicFriction  <- B1 自旋減速 (滑動中量的)
  physics:staticFriction   <- B2 起轉 effort (從靜止推到開始動)
  換算: 參考地面 + 輪子 -> 參考有效值 -> x 比值 -> 目標有效值 -> 反解目標地面。
  USD 讀不了 (沒有 pxr) 就用數字: --ref-ground-mu 0.5 [--wheel-mu 0.5 --combine average]
  真實世界沒有「輪子材質」這層, 直接看比值。

────────────────────────────────────────────────────────────────────────
訊號來源 (--source)
────────────────────────────────────────────────────────────────────────
auto   (預設) 有 <csv>_imu.csv 就用 gyro, 沒有才退回 gt
gyro   **真車流程**: collect_data_node 全速率記的 /imu。
         轉速 = gyro z - 零偏 (零偏從 phase=='calibrate' 的靜止段估)
         角度 = 轉速積分;  前向速度 = 前向加速度從靜止積分 (每個加速段起點歸零,
         基準取起點前 0.25 s 的靜止平均)。不吃輪速、不吃定位 -> 不受打滑與
         LiDAR 追丟影響。IMU 假設跟 base_link 同向 (car.usd 是這樣裝的)。
gt     Isaac ground truth (car_position_*, gt_yaw, twist)。模擬器裡對答案用。
fus / lid / cam / imu / whl   各定位線的位姿 (主 CSV 20 Hz 取樣, 依 _stamp 去重)
         * 自旋 > ~8 rad/s 時 LiDAR 會追丟, 用 lid 算 B1 會壞掉。
         * whl (以及吃輪速的 fus 的速度) 打滑時是錯的, 而打滑就是摩擦造成的 ——
           循環論證。角度類特徵 (B1/B2) 相對安全, B4 不要信。

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
STRAIGHT_MAX_TURN = math.radians(10.0)   # B3/B4 加速段車頭最多轉這麼多, 超過就排除
SPIN_MIN_PTS = 5                         # B1 線性段至少幾個點才採用 (見 run() 的說明)
WHEELS = ['front_left_velocity', 'front_right_velocity',
          'rear_left_velocity', 'rear_right_velocity']

# 特徵 -> (說明, 角色)。角色: primary 參與綜合估計; secondary 只印; control 應該 ≈ 1
FEATURES = {
    'spin_decel':      ('B1 自旋 coast 角減速度 [rad/s²]', 'primary'),
    'spin_decel_energy': ('B1 同上, 能量法 (含過渡段, 偏低)', 'secondary'),
    'creep_breakaway': ('B2 慢速 ramp 起轉 steer effort', 'primary'),
    'spinup_time':     ('B1 spin_up 從靜止到 w_ref 的時間 [s] (靜摩擦, 需馬達一致)', 'secondary'),
    'spinup_breakaway': ('B1 spin_up 起轉 steer effort', 'secondary'),
    'launch_accel':    ('B4 滿油門起步加速度 [m/s²]', 'secondary'),
    'launch_slip':     ('B4 起步 slip ratio', 'secondary'),
    'sprint_accel':    ('B3 直線加速度 [m/s²]  (對照, 應≈1)', 'control'),
    'coast_lin_decel': ('B3 直線 coast 減速度/速度 [1/s]  (對照, 應≈1)', 'control'),
}


# ═══════════════════════════════════════════════════════════════════════
#  讀檔: 把 CSV 變成統一格式的 track (不管位姿來自哪個來源)
# ═══════════════════════════════════════════════════════════════════════
def load_track(path, source, forward_axis_deg, max_sigma, max_age):
    d = pd.read_csv(path)
    need = {'scenario_name', 'phase', 'effort_command_front_left',
            'effort_command_front_right'}
    missing = need - set(d.columns)
    if missing:
        sys.exit(f'{path}: 缺少欄位 {sorted(missing)} —— 不是 calibrate_env_node 收的 CSV?')

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
        'yaw': yaw.astype(float), 'valid': valid.values,
        'steer': 0.5 * (d['effort_command_front_right'] - d['effort_command_front_left']),
        'thr': 0.5 * (d['effort_command_front_right'] + d['effort_command_front_left']),
        'wheel': d[WHEELS].abs().mean(axis=1) if set(WHEELS) <= set(d.columns) else np.nan,
    })
    # 感測器欄位是 20 Hz 複製最新一則 -> 同一個 stamp 只算一次, 之後 forward-fill
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
    steer = 0.5 * (d['effort_command_front_right'] - d['effort_command_front_left'])
    phase = d['phase'].astype(str)
    name = d['scenario_name'].astype(str)

    # gyro 零偏: calibrate 段 (去頭 0.3 s) > B2 creep 開頭 (steer<1, 車子還沒動) > 0
    still = (phase == 'calibrate').to_numpy().copy()
    if still.sum() > 10:
        still &= t > t[still][0] + 0.3
    if still.sum() < 10:
        still = (name.str.startswith('B2 creep') & (steer.abs() < 1.0)).to_numpy()
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
        'yaw': np.nan, 'valid': True, 'steer': steer,
        'thr': 0.5 * (d['effort_command_front_right'] + d['effort_command_front_left']),
        'wheel': d[[f'{k}_velocity' for k in
                    ('front_left', 'front_right', 'rear_left', 'rear_right')]].abs().mean(axis=1),
        'yaw_u': yaw_u, 'w': w, 'vf': np.nan, 'fresh': True,
    })
    tr['seg'] = (tr['name'] != tr['name'].shift()).cumsum()
    # 段落起點: 新版 collect_data_node 會記 scenario_recv (收到段落標籤的時刻,
    # 節點時鐘)。同一列的 recv 也是節點時鐘 -> 用兩者的中位數差換算到 stamp 時鐘。
    # 舊檔沒有這兩欄就是 NaN, 分析端退回「第一列帶著新標籤的資料」(誤差 1 個 IMU 週期)。
    if 'scenario_recv' in d.columns and 'recv' in d.columns:
        off = float(np.median(d['recv'].to_numpy(float) - t))
        tr['t0'] = d['scenario_recv'].to_numpy(float) - off
    else:
        tr['t0'] = np.nan

    # 前向速度: 加速段起點歸零, 積分到它後面的 coast 段結束
    vf = np.full(n, np.nan)
    active, v, ref = False, 0.0, 0.0
    for _, g in tr.groupby('seg', sort=True):
        i0, i1 = g.index[0], g.index[-1]
        nm, meas = g['name'].iloc[0], g['phase'].iloc[0] == 'measure'
        start = i1 + 1                                   # 不積分
        if meas and nm.startswith(('B3 sprint_accel', 'B4 slip_launch')):
            pre = (t >= t[i0] - 0.25) & (t < t[i0])
            ref = float(np.median(a_f[pre])) if pre.sum() >= 3 else float(a_f[i0])
            active, v = True, 0.0
            vf[i0] = 0.0
            start = i0 + 1
        elif active and meas and nm.startswith(('B3 sprint_coast', 'B4 slip_coast')):
            start = i0
        else:
            active = False
        for i in range(start, i1 + 1):
            v += (a_f[i] - ref) * dt[i]
            vf[i] = v
    tr['vf'] = vf
    tr.attrs['gyro_bias'] = bias
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


def yaw_noise(segs):
    """位姿來源的 yaw 雜訊 (決定起轉偵測門檻)。

    取 B2 creep 開頭 steer effort < 1 的那段 —— 那時車子一定還沒動。不用 Rest 段:
    Rest 接在 brake 後面, 低 mu 的環境車子常常還在微微滑 (sim 0.5 實測 Rest 的
    yaw 抖動是 real 2.0 的一萬倍), 會把門檻墊高、兩邊不一致。"""
    vals = [g.loc[g['fresh'] & (g['steer'].abs() < 1.0), 'yaw_u'].diff().dropna()
            for b, _, g in segs if b == 'B2 creep_spin']
    vals = pd.concat(vals) if vals else pd.Series(dtype=float)
    return float(vals.std()) if len(vals) > 5 else 0.0


def breakaway_effort_w(g, w_lo=0.10, w_hi=1.0):
    """慢速 ramp 的起轉 effort, 從**角速度**外插回去 (不看累積角度)。

    起轉之後多出來的力矩 ∝ (t - tb) -> w ∝ (t - tb)², 所以 √w 對 t 是直線,
    外插回 0 的時刻就是起轉點。

    為什麼不用角度門檻 (breakaway_effort): 真正打滑之前有一段**預滑移** ——
    輪胎與底盤被扭矩扭出彈性變形, 車體會非常慢地轉個 1~4 度然後停住。
    2026-09-15 的資料裡 9 個 B2 trial 有 3 個是這樣, 角度在 effort≈2 就越過
    0.02 rad 的門檻, 量到的起轉 effort 是 2.0 而不是真正的 5.6 (差 3 倍),
    3 個 trial 的中位數因此完全不可靠 (散佈 69%)。預滑移的角速度只有
    ~0.02 rad/s, 而真正起轉後 0.1 rad/s 是瞬間的事 -> 用角速度區間就避開了。

    實測 (同一份資料, 中位數的散佈 / 真值 2.50 的比值):
        角度門檻 + ∛θ   散佈 69%   比值 2.17 (偏向 1)
        角速度 + √w     散佈 14%   比值 2.53
    """
    t = g['t'].to_numpy() - g['t'].iloc[0]
    w = np.abs(g['w'].to_numpy())
    st = g['steer'].abs().to_numpy()
    m = (w >= w_lo) & (w <= w_hi)
    if m.sum() >= 4:
        i = np.flatnonzero(m)
        p = np.polyfit(t[i], np.sqrt(w[i]), 1)
        if p[0] > 0:
            tb = -p[1] / p[0]
            if t[0] <= tb <= t[i[0]]:          # 外插點要落在區間開始之前
                return float(np.interp(tb, t, st))
    return None


def spinup_time(g, w_ref, t0=None):
    """spin_up 段從**指令發出**到轉速第一次到 w_ref 的時間 [s]。

    t0 = 段落真正的起點 (collect_data_node 記的 scenario_recv)。沒有的話只能用
    第一列帶著這個標籤的 IMU 資料, 誤差是一個 IMU 週期 (60 Hz -> 16.7 ms,
    對 1.5 s 的量測是 1.1% 的抖動, 跟特徵本身 0.7% 的散佈同一個量級)。
    """
    t = g['t'].to_numpy()
    w = np.abs(g['w'].to_numpy())
    if w.max() < w_ref:
        return None
    k = int(np.argmax(w >= w_ref))
    if k == 0:                                  # 一開始就在轉 -> 沒有從靜止起步
        return None
    start = t0 if (t0 is not None and np.isfinite(t0) and
                   t[0] - 0.1 <= t0 <= t[0]) else t[0]
    return float(np.interp(w_ref, [w[k - 1], w[k]], [t[k - 1], t[k]]) - start)


def breakaway_effort(g, th_yaw):
    """慢速 ramp 裡車子開始轉的 steer effort。∛θ 外插, 做不到就退回門檻穿越點。"""
    t = g['t'].to_numpy() - g['t'].iloc[0]
    th = np.abs(g['yaw_u'] - g['yaw_u'].iloc[0]).to_numpy()
    st = g['steer'].abs().to_numpy()
    moved = th > th_yaw
    if not moved.any() or moved.argmax() == 0:
        return None
    k0 = moved.argmax()
    # 擬合窗: 從過門檻到轉了 0.5 rad (再後面動摩擦 / 轉速效應開始進來)
    k1 = np.argmax(th > 0.5) if (th > 0.5).any() else len(th)
    k1 = max(k1, k0 + 4)
    tt, yy = t[k0:k1], np.cbrt(th[k0:k1])
    if len(tt) >= 4:
        p = np.polyfit(tt, yy, 1)
        if p[0] > 0:
            tb = -p[1] / p[0]
            if t[max(k0 - 20, 0)] <= tb <= t[k0]:          # 外插合理才用
                return float(np.interp(tb, t, st))
    return float(st[k0])


# ═══════════════════════════════════════════════════════════════════════
#  逐 trial 抽特徵
# ═══════════════════════════════════════════════════════════════════════
def extract(tr, th_yaw=None):
    segs = segments(tr)
    if th_yaw is None:
        th_yaw = max(0.02, 6.0 * yaw_noise(segs))
    F = {k: [] for k in FEATURES}
    F['_spin_rows'] = []           # 畫圖用: (w0, w_end, dθ)
    F['_spinup_segs'] = []         # spinup_time 用: (spin_up 段, 段落起點)
    F['_spin_pts'] = []            # spin_decel 每個 trial 線性段用了幾個點
    F['_spin_w0'] = []             # spin_decel 每個 trial 的起始轉速 (w0 校正用)

    # 輪半徑: 直線 coast 時輪子是自由滾動的 -> r ≈ v / w_wheel
    r_samples = []
    for b, _, g in segs:
        if b == 'B3 sprint_coast':
            ok = (g['vf'] > 0.5) & (g['wheel'] > 3.0)
            r_samples += (g.loc[ok, 'vf'] / g.loc[ok, 'wheel']).tolist()
    r_wheel = float(np.median(r_samples)) if len(r_samples) > 5 else None

    excluded = []
    bad_trial = False
    for i, (b, name, g) in enumerate(segs):
        g = g[g['valid']]
        if len(g) < 3:
            continue
        t = g['t'].to_numpy() - g['t'].iloc[0]

        # 直線段 (B3/B4) 的加速段車頭轉了很多 = 起點 / 朝向不對 (通常是定位壞掉,
        # 歸位把車擺歪了)。那一趟的加速度與後面的 coast 都不是直線的量, 整趟排除。
        if b in ('B3 sprint_accel', 'B4 slip_launch'):
            turn = abs(g['yaw_u'].iloc[-1] - g['yaw_u'].iloc[0])
            bad_trial = turn > STRAIGHT_MAX_TURN
            if bad_trial:
                excluded.append(f'{name} (轉了 {math.degrees(turn):.0f}°)')
                continue
        elif b in ('B3 sprint_coast', 'B4 slip_coast'):
            if bad_trial:
                continue
        else:
            bad_trial = False

        if b == 'B1 spin_coast':
            w = g['w'].abs().to_numpy()
            th = g['yaw_u'].to_numpy()
            # 起點取放開後 0.2 s 內的轉速峰值, 不是放開那一刻: 指令傳進模擬器 / 馬達
            # 驅動器要時間, 輪子也還轉得比車體快在推車, 轉速會再往上衝 30~70 ms
            # (Isaac 60 Hz IMU 實測)。峰值之前淨力矩還是正的, 不算 coast。
            k = int(np.argmax(w[:max(3, np.searchsorted(t, 0.2))]))
            w0, we = w[k], w[-1]
            if w0 < 1.0:
                continue
            dth = abs(th[-1] - th[k])
            if dth > 1e-3 and w0 > we:
                F['_spin_rows'].append((w0, we, dth))
                F['spin_decel_energy'].append((w0 ** 2 - we ** 2) / (2 * dth))
            # 線性段: 峰值之後落在 80%~20% 峰值的點。過渡段 (輪子跟地面還沒同步)
            # 與尾段 (低速時的靜摩擦 / 接觸抖動) 都不是純 Coulomb, 夾掉。
            after = w[k:]
            below = np.flatnonzero(after < 0.2 * w0)
            idx = k + np.arange(below[0] if len(below) else len(after))
            idx = idx[w[idx] <= 0.8 * w0]
            if len(idx) >= 3:
                F['spin_decel'].append(-np.polyfit(t[idx], w[idx], 1)[0])
                F['_spin_pts'].append(len(idx))    # run() 依點數挑 trial, 見 SPIN_MIN_PTS
                F['_spin_w0'].append(float(w0))    # 見 match_spin_w0

        elif b == 'B2 creep_spin':
            # 先用角速度外插 (避開預滑移), 做不到才退回角度門檻
            e = breakaway_effort_w(g)
            if e is None:
                e = breakaway_effort(g, th_yaw)
            if e is not None:
                F['creep_breakaway'].append(e)

        elif b == 'B1 spin_up':
            # ramp 快 (6 effort/s), 門檻穿越點會被 0.3 effort 的 tick 量化 (5.4 / 6.3 兩檔);
            # 60 Hz IMU 下 ∛θ 外插有足夠的點, 給的是連續值。20 Hz 資料外插不起來時
            # breakaway_effort 自己會退回門檻穿越點。
            e = breakaway_effort(g, th_yaw)
            if e is not None:
                F['spinup_breakaway'].append(e)
            # spinup_time 的 w_ref 要**所有檔案共用**, 所以只先把軌跡存起來,
            # 由 run() 決定共同的 w_ref 之後再算 (見 fill_spinup_time)
            t0 = g['t0'].iloc[0] if 't0' in g else np.nan
            F['_spinup_segs'].append((g, t0))

        elif b == 'B4 slip_launch':
            if t[-1] > 0.1:
                F['launch_accel'].append(np.polyfit(t, g['vf'], 1)[0])
            if r_wheel:
                ok = g['wheel'] > 5.0
                if ok.sum() >= 2:
                    rw = r_wheel * g.loc[ok, 'wheel']
                    F['launch_slip'].append(float(np.median(
                        np.clip((rw - g.loc[ok, 'vf']) / rw, 0, 1))))

        elif b == 'B3 sprint_accel':
            if t[-1] > 0.3:
                F['sprint_accel'].append(np.polyfit(t, g['vf'], 1)[0])

        elif b == 'B3 sprint_coast':
            k = t < 1.0
            v0 = g['vf'].iloc[0]
            if k.sum() >= 4 and v0 > 0.5:
                F['coast_lin_decel'].append(-np.polyfit(t[k], g['vf'][k], 1)[0] / v0)

    F['_meta'] = {'yaw_threshold': th_yaw, 'wheel_radius': r_wheel, 'excluded': excluded}
    return F


def match_spin_w0(Fs):
    """把每個 trial 的自旋減速度校正到**共同的起始轉速**, 再比較。

    純 Coulomb 的話減速度跟轉速無關, 但實測不是: 迴歸 decel = a + b*w0 得到的 b 是
    +0.25 ~ +0.81 (rad/s² per rad/s), 也就是還有一個跟轉速有關的阻力 (輪子 joint
    damping / 空氣)。它不隨地面摩擦等比例變化, 所以只要兩個檔案的實際 w0 不一樣,
    比值就有系統性偏差。

    w0 為什麼會不一樣: 「到目標轉速就放開」有殘餘的觸發延遲 (+1~9%), 而延遲期間的
    角加速度本身就跟地面摩擦有關 —— 摩擦大的環境衝得慢、overshoot 小。也就是說
    **這個偏差跟要量的東西相關**, 不會自己抵消。

    做法: 每個檔案各自迴歸 (斜率跟 mu 有關, 不能共用), 每個 trial 減掉
    b*(w0_i - w_ref), w_ref 取所有檔案所有 trial 的中位數。

    2026-09-16 的那一輪 (sim 2.0 -> real 0.5, 真值 0.5): 校正前內插出 0.4865
    (-2.7%), 校正後 0.4968 (-0.6%); 每個 trial 的 cv 也從 2.0~2.3% 掉到 1.1%。"""
    ok = [F for F in Fs if len(F['spin_decel']) == len(F['_spin_w0']) >= 5]
    if len(ok) < len(Fs) or not ok:
        return None
    allw = [w for F in ok for w in F['_spin_w0']]
    w_ref = float(np.median(allw))
    for F in ok:
        w0 = np.asarray(F['_spin_w0'], float)
        d = np.asarray(F['spin_decel'], float)
        if w0.max() - w0.min() < 0.3:        # 轉速沒拉開, 斜率不可信 -> 不校正
            F['_meta']['spin_w0_slope'] = None
            continue
        b = float(np.polyfit(w0, d, 1)[0])
        F['spin_decel'] = (d - b * (w0 - w_ref)).tolist()
        F['_meta']['spin_w0_slope'] = b
        F['_meta']['spin_w0_shift'] = float(np.median(w0) - w_ref)
    for F in ok:
        F['_meta']['spin_w0_ref'] = w_ref
    return w_ref


def fill_spinup_time(Fs):
    """所有檔案共用一個 w_ref, 算 B1 spin_up 的「從靜止到 w_ref」時間。

    w_ref 必須是**每個檔案的每個 trial 都到得了**的轉速, 否則某一邊會少 trial
    (而且少的一定是摩擦大的那邊 -> 系統性偏差)。取所有 trial 峰值最小的那個的
    60%: 低一點比較接近純起轉 (敏感度略高), 又離噪聲夠遠。"""
    peaks = [float(np.abs(g['w'].to_numpy()).max())
             for F in Fs for g, _ in F['_spinup_segs']]
    if not peaks:
        return None
    w_ref = round(0.6 * min(peaks), 2)
    if w_ref < 1.0:
        return None
    for F in Fs:
        F['spinup_time'] = [v for v in (spinup_time(g, w_ref, t0)
                                        for g, t0 in F['_spinup_segs']) if v is not None]
        F['_meta']['spinup_w_ref'] = w_ref
    return w_ref


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
    自己沒有就往上找祖先; 綁到的材質沒有 PhysicsMaterialAPI (例如 car.usd 輪子綁的
    是只有外觀的 /World/Looks/wheel) 就當沒綁 -> PhysicsScene 的預設材質 -> 0.5/0.5。

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
    wheels = [p for p in stage.Traverse()
              if pat.search(p.GetName()) and p.HasAPI(_collision_api())]
    if not wheels:
        sys.exit(f'{usd_path}: 找不到名字符合 /{wheel_pattern}/ 的碰撞體 (用 --wheel-pattern 指定)')
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
GROUND_TERMS = [
    ('dynamic', 'physics:dynamicFriction', ['spin_decel', 'spin_decel_energy']),
    ('static', 'physics:staticFriction', ['creep_breakaway', 'spinup_breakaway']),
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
        # (B2 通常只有 2 個 trial, bootstrap 會窄得不合理)
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
        if key.endswith('_energy') or key.startswith('spinup'):
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
                p('  ! 動摩擦 > 靜摩擦: 物理上不合理, 通常是 B2 trial 太少或偵測門檻問題, 以動摩擦為準')
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


def report(ref_path, tgt_path, res, combined, Fr, Ft, args):
    p = print
    p('═' * 78)
    p(f'參考: {ref_path}')
    p(f'目標: {tgt_path}')
    p(f'位姿來源: {args.source}   (起轉偵測門檻 yaw {Fr["_meta"]["yaw_threshold"]:.3f} rad)')
    for tag, path, F in (('參考', ref_path, Fr), ('目標', tgt_path, Ft)):
        q = args.loc_quality.get(path)
        if q is not None:
            msg, bad = q
            p(f'{"!" if bad else " "} {tag}收資料時的定位: {msg}'
              + ('  <- 定位壞了, 控制是在錯的位置 / 朝向跑的' if bad else ''))
        if F['_meta']['excluded']:
            p(f'! {tag}排除 {len(F["_meta"]["excluded"])} 趟不是直線的 B3/B4: '
              + ', '.join(F['_meta']['excluded'][:4])
              + (' ...' if len(F['_meta']['excluded']) > 4 else ''))
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
        p('沒有任何主特徵 (B1 spin_coast / B2 creep_spin) 可用, 無法估計。')
        return None

    r, (lo, hi) = combined['ratio'], combined['ci90']
    p(f'綜合 (主特徵加權 {combined["weights"]}):')
    p(f'  有效摩擦係數 目標/參考 = {r:.2f}   90% CI [{lo:.2f}, {hi:.2f}]')
    p(f'  => 摩擦力{verdict(combined["ci90"], r, args.tol)},  約 {100 * (r - 1):+.0f}%')

    warn = []
    prim = combined['features']
    if len(prim) > 1:
        signs = {np.sign(math.log(res[k]['ratio'])) for k in prim
                 if abs(math.log(res[k]['ratio'])) > math.log(1 + args.tol)}
        if len(signs) > 1:
            warn.append('主特徵方向不一致 (一個說變大一個說變小), 結果不可信。')
        ratios = [res[k]['ratio'] for k in prim]
        if max(ratios) / min(ratios) > 1.5:
            warn.append(f'主特徵之間差很多 ({", ".join(f"{x:.2f}" for x in ratios)}), '
                        '靜摩擦與動摩擦可能不是同比例變化。')
    for k, x in res.items():
        if x['role'] == 'control' and abs(math.log(x['ratio'])) > math.log(1.2):
            warn.append(f'對照組「{x["desc"]}」比值 {x["ratio"]:.2f} 偏離 1 —— 兩個環境'
                        '可能不只摩擦不同 (馬達 / 質量 / 阻尼 / 位姿來源品質)。')
    if 'launch_accel' in res and \
            abs(math.log(res['launch_accel']['ratio'])) < 0.5 * abs(math.log(r)):
        warn.append('B4 起步加速度的比值被壓向 1: 至少一邊是扭矩受限 (馬達卡住), 這是預期的。')
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
        a = np.asarray(F['_spin_rows'])
        if len(a):
            ax.scatter(a[:, 0], a[:, 2], s=12, label=f'{lab} α={stat(F, "spin_decel"):.1f}')
    ax.set_xlabel('w0 [rad/s]')
    ax.set_ylabel('coast angle dθ [rad]')
    ax.legend(fontsize=8)
    ax.set_title('B1 coast: dθ vs w0', fontsize=9)
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
                    help='訊號來源 (預設 auto: 有 _imu.csv 就用 gyro, 否則 gt)')
    ap.add_argument('--compare-gt', action='store_true',
                    help='同一份資料再用 GT 算一次並排印 (驗證感測器流程)')
    ap.add_argument('--gyro-noise', type=float, default=0.0,
                    help='注入 gyro 白雜訊 std [rad/s] (每個樣本)')
    ap.add_argument('--gyro-bias', type=float, default=0.0, help='注入 gyro 零偏 [rad/s]')
    ap.add_argument('--acc-noise', type=float, default=0.0, help='注入加速度白雜訊 std [m/s²]')
    ap.add_argument('--acc-bias', type=float, default=0.0, help='注入前向加速度零偏 [m/s²]')
    ap.add_argument('--forward-axis-deg', type=float, default=-90.0,
                    help='車頭相對 base_link 的角度, 跟 control_car_node 的參數一致')
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
                   help='輪子碰撞體名稱的 regex (不分大小寫)')
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
    if args.source in ('whl', 'lid'):
        print(f'! --source {args.source}: '
              + ('輪速里程計在打滑時是錯的, 而打滑就是摩擦造成的 (循環論證)。'
                 if args.source == 'whl' else 'B1 自旋 > ~8 rad/s 時雷射會追丟。'))
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
        tracks = {p: load(p, source) for p in paths}
        # 起轉偵測門檻**所有檔案共用一個**: 門檻不同 = 偵測延遲不同 = 起轉 effort 的
        # 偏差不同, 比值就不公平了。取最吵的那個來源決定。
        th = max(0.02, max(6.0 * yaw_noise(segments(t)) for t in tracks.values()))
        F = {p: extract(tr, th) for p, tr in tracks.items()}
        # B1 只用線性段點數夠的 trial。點太少的 coast (高 mu + 低轉速, 整段 < 0.1 s)
        # 幾乎全是放開後的過渡段, 減速度系統性偏低 —— 2026-09-15 校正資料:
        # sim mu 1.5 的 w0=2 那組 32 vs 其他 50, 混進去中位數的 cv 從 0.03 變 0.17。
        # 門檻**所有檔案共用**, 從 SPIN_MIN_PTS 往下找到每個檔案都還有 3 個 trial 為止。
        for m in range(SPIN_MIN_PTS, 2, -1):
            if all(sum(p >= m for p in f['_spin_pts']) >= 3 for f in F.values()):
                break
        for f in F.values():
            keep = [p >= m for p in f['_spin_pts']]
            dropped = len(keep) - sum(keep)
            f['spin_decel'] = [v for v, k in zip(f['spin_decel'], keep) if k]
            f['_spin_w0'] = [v for v, k in zip(f['_spin_w0'], keep) if k]
            f['_meta']['spin_min_pts'] = m
            f['_meta']['spin_dropped'] = dropped
        if m < SPIN_MIN_PTS:
            print(f'! [{source}] B1 線性段 >= {SPIN_MIN_PTS} 點的 trial 不夠, 門檻降到 {m} 點 (偏差較大)')
        # B1 線性擬合每個 coast 要 >= 3 個點。20 Hz 的位姿資料在高 mu 環境常常湊不滿
        # (coast 只有 0.15 s) -> **所有檔案一起**退回能量法。只有一邊退回的話兩邊
        # 偏差不同, 比值就不公平。
        wr = match_spin_w0(list(F.values()))
        if wr is None:
            print(f'! [{source}] trial 數不夠, spin_decel 沒有做 w0 校正')
        w_ref = fill_spinup_time(list(F.values()))
        if w_ref is None:
            print(f'! [{source}] B1 spin_up 沒有可用的軌跡, 沒有 spinup_time')
        if any(len(f['spin_decel']) < 3 for f in F.values()):
            print(f'! [{source}] B1 線性段點數不夠 (IMU 全速率檔才夠密), '
                  'B1 主特徵改用能量法 —— 會含放開後的過渡段, 比值偏向 1')
            for f in F.values():
                f['spin_decel'], f['spin_decel_energy'] = f['spin_decel_energy'], []
        return F

    feats = run(args.source)
    gt_feats = None
    if args.compare_gt and args.source != 'gt':
        try:
            gt_feats = run('gt')
        except (KeyError, SystemExit):
            print('! --compare-gt: 資料裡沒有 GT (真車?), 略過')

    Fr = feats[args.ref]
    results = {}
    for tp in args.targets:
        Ft = feats[tp]
        res, combined = compare(Fr, Ft, args.boot, rng)
        out = report(args.ref, tp, res, combined, Fr, Ft, args)
        if gt_feats is not None and out is not None:
            gres, gcomb = compare(gt_feats[args.ref], gt_feats[tp], args.boot, rng)
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
        ground = ground_report(res, args) if res else None
        if args.plot and res:
            plot(args.ref, tp, Fr, Ft, res, args.plot)
        lq = {k: (None if args.loc_quality.get(pth) is None else
                  {'message': args.loc_quality[pth][0], 'bad': bool(args.loc_quality[pth][1])})
              for k, pth in (('ref', args.ref), ('target', tp))}
        results[tp] = {'summary': out, 'combined': combined, 'features': res,
                       'ground_friction': ground, 'loc_quality': lq,
                       'excluded': {'ref': Fr['_meta']['excluded'],
                                    'target': Ft['_meta']['excluded']}}
    if args.json:
        with open(args.json, 'w') as f:
            json.dump({'ref': args.ref, 'source': args.source, 'results': results}, f,
                      indent=2, ensure_ascii=False, default=float)
        print(f'JSON: {args.json}')


if __name__ == '__main__':
    main()
