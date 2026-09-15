#!/usr/bin/env python3
"""融合的離線 A/B —— 不需要 ROS, 不需要 Isaac, 不需要 YOLO。

    python3 test/test_fusion.py

**README 裡的每一個數字都是這支印出來的。** 改了參數想知道值不值得就重跑它。

它做的事: 生一條 8 字形軌跡, 由同一條真值長出四種感測器的讀數 ——

    IMU     60 Hz  加速度/陀螺零偏 + 雜訊
    輪速    60 Hz  尺度誤差 + 雜訊 + 打滑段
    相機    30 Hz  sigma 2.1 cm, **延遲 79.5 ms** (car_loc_camera 量到的值)
    LiDAR   10 Hz  sigma 3.5 cm / yaw 2.9 度, 延遲 50 ms, 含「鎖到 180 度」的段落

—— 然後把它們餵進 FusionEkf, 跟只用其中一種比。

**量測是照「到達時刻」餵進去的** (t_meas + delay), 不是照時戳, 所以延遲是真的
延遲, 不是模擬出來的參數。這是這支測試唯一不能省的細節: 把延遲當參數傳進去的話,
倒帶重放那一段就等於沒有被測到。
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, '..', '..', 'car_loc_wheel')))

from car_loc_fusion.fusion_ekf import AbsMeas, FusionEkf, Step      # noqa: E402
from car_loc_fusion.sources import AbsSource                        # noqa: E402
from car_loc_wheel.wheel_ins import WheelStillDetector, wrap_pi     # noqa: E402

DT = 1.0 / 60.0
RADIUS = 0.075
# 真值的有效輪半徑比設定檔給的大 2.2% —— 這是**刻意**的, 而且是真的會發生的事:
# car_loc_wheel 量到同一台車在「控制器驅動」時 scale 是 0.93, 「滑行」時是 1.008
# (輪胎有扭矩就一定有縱向滑移)。設定檔只能填一個工作點, 所以航位推算永遠帶著
# 幾個 % 的尺度誤差 —— 那正是絕對量測要修的東西。
# 兩個都給同一個值的話, 這支 bench 的航位推算會準到 0.1 cm, 融合就顯得沒有用。
TRUE_SCALE = 0.95
CFG_SCALE = 0.93
FORWARD_DEG = -90.0

GYRO_BIAS, ACC_BIAS = 0.004, 0.02
GYRO_NOISE, ACC_NOISE, WHEEL_NOISE = 0.004, 0.02, 0.05

# 感測器規格 —— 全部是這個 repo 量出來的值, 不是猜的:
#   相機 2.09 cm / 79.5 ms  <- report_5methods.md §2.3, camera_ground.yaml
#   LiDAR 3.45 cm / 2.9 deg <- test_matcher.py [5] 5 cm 地圖那一列 + [1] 的 yaw
CAM_HZ, CAM_SIGMA, CAM_DELAY = 30.0, 0.021, 0.0795
LID_HZ, LID_SIGMA, LID_YAW_SIGMA, LID_DELAY = 10.0, 0.035, 0.051, 0.05
# 節點自己回報的 sigma (collect_data_node 實測: 正常 ~0.0015, 追丟 ~0.0145)
LID_SIGMA_OK, LID_SIGMA_LOST = 0.0015, 0.0145


# ---------------------------------------------------------------- 真值與感測器
def make_truth(seed=0, dur=90.0, v_peak=0.6, stops=True,
               cam_blackout=(), lidar_lock=(), slips=(), kidnap=None,
               cam_rho=0.7):
    rng = np.random.default_rng(seed)
    n = int(dur / DT)
    t = np.arange(n) * DT

    v = np.zeros(n)
    for i, ti in enumerate(t):
        if ti < 2.0:
            continue                       # 開機靜止段 (零偏校正)
        if stops:
            ph = ti % 20.0
            v[i] = v_peak * 0.5 * (np.tanh((ph - 2.0) / 0.4)
                                   - np.tanh((ph - 16.0) / 0.4))
        else:
            v[i] = v_peak * 0.5 * (1.0 + np.tanh((ti - 2.5) / 0.4))
    wz = 0.8 * np.sin(2 * np.pi * t / 20.0) * (v > 0.05)

    # 真值必須用**跟濾波器一模一樣的離散模型**積出來, 否則量到的「誤差」有一
    # 部分只是兩種積分法的差 (實測: 3 m/s 的 8 字形跑 60 秒差 10 cm, 而且是
    # 系統性的 —— 濾波器會把它當成偏差, 然後卡方閘門開始擋掉**正確的**量測,
    # 高速那幾列就整個變成噪音)。這不是感測器融合的性質, 是 bench 的 bug。
    a_fwd = np.gradient(v) / DT
    yaw = np.zeros(n)
    px = np.zeros(n)
    py = np.zeros(n)
    # 輪速要回報**真正的**速度, 不是速度剖面 v —— 位置是用 a_fwd 積出來的, 而
    # a_fwd 是 v 的中央差分, 積回去跟 v 差約 0.1%。看起來可以忽略, 但那是一個
    # **速度**誤差: 60 秒累積成 10 cm 的位置差, 而且濾波器會把它當成真的誤差,
    # 然後卡方閘門開始擋掉正確的相機量測 (實測 1.8 m/s 時擋掉一半)。
    # 合成資料裡「真值」有好幾個版本的時候, 每一個感測器都要對到**它量的那一個**。
    vtrue = np.zeros(n)
    vv = 0.0
    for i in range(1, n):
        h = yaw[i - 1] + math.radians(FORWARD_DEG)
        ds = vv * DT + 0.5 * a_fwd[i] * DT * DT
        px[i] = px[i - 1] + ds * math.cos(h)
        py[i] = py[i - 1] + ds * math.sin(h)
        vv += a_fwd[i] * DT
        vtrue[i] = vv
        yaw[i] = yaw[i - 1] + wz[i] * DT
    hd = yaw + math.radians(FORWARD_DEG)

    if kidnap is not None:
        # 綁架: 車子被瞬間搬走。IMU 與輪速**完全看不到**這件事 (它們量的是
        # 運動, 不是位置), 只有絕對量測知道 —— 而濾波器這時對自己很有信心,
        # 所以每一則正確的量測都會被卡方閘門擋掉。這是逃生口存在的理由。
        tk, dx, dy = kidnap
        m = t >= tk
        px[m] += dx
        py[m] += dy

    gyro = wz + GYRO_BIAS + GYRO_NOISE * rng.standard_normal(n)
    acc = a_fwd + ACC_BIAS + ACC_NOISE * rng.standard_normal(n)

    w_true = vtrue / (TRUE_SCALE * RADIUS)
    W = np.tile(w_true, (4, 1)) + WHEEL_NOISE * rng.standard_normal((4, n))
    for k, (t0, t1, mult) in enumerate(slips):
        m = (t >= t0) & (t < t1)
        W[0, m] *= mult
        if k % 2:
            W[2, m] *= mult

    # --- 絕對量測。t 是**量測發生**的時刻, arrive 是收到的時刻 ---
    meas = []
    for src, hz, sig, delay in (('camera', CAM_HZ, CAM_SIGMA, CAM_DELAY),
                                ('lidar', LID_HZ, LID_SIGMA, LID_DELAY)):
        stride = max(int(round((1.0 / hz) / DT)), 1)
        # 相機那條路發的是**卡爾曼濾波之後**的位置, 所以連續兩則的誤差是相關的
        # (rho ~ 0.7 @ 30 Hz)。EKF 的更新式假設每一筆量測的誤差互相獨立, 拿
        # 相關的量測全速餵它, P 會被壓到比真實精度樂觀 —— 見主程式 [G]。
        # 生成獨立雜訊的話這件事在 bench 上就完全看不到, 所以這裡要生相關的。
        rho = cam_rho if src == 'camera' else 0.0
        ex = ey = 0.0
        for i in range(0, n, stride):
            ti = float(t[i])
            if src == 'camera' and any(a <= ti < b for a, b in cam_blackout):
                continue          # 車子被柱子擋住 / YOLO 漏偵測 -> 整段沒有輸出
            k = math.sqrt(max(1.0 - rho * rho, 0.0))
            ex = rho * ex + k * sig * rng.standard_normal()
            ey = rho * ey + k * sig * rng.standard_normal()
            x, y = px[i] + ex, py[i] + ey
            yw = rep = None
            if src == 'lidar':
                yw = yaw[i] + LID_YAW_SIGMA * rng.standard_normal()
                rep = LID_SIGMA_OK
                if any(a <= ti < b for a, b in lidar_lock):
                    # 長方形房間對 180 度幾乎對稱, 鎖住之後位置也跟著鏡射,
                    # 而且**回不來**。節點自己的 sigma 對這件事很靈敏。
                    x, y, yw = -x, -y, wrap_pi(yw + math.pi)
                    rep = LID_SIGMA_LOST
            meas.append(dict(t=ti, arrive=ti + delay, src=src, x=x, y=y,
                             yaw=yw, reported=rep if rep is not None else CAM_SIGMA))
    meas.sort(key=lambda m: m['arrive'])
    return dict(t=t, v=vtrue, wz=wz, yaw=yaw, px=px, py=py, hd=hd,
                gyro=gyro, acc=acc, W=W, meas=meas)


# ---------------------------------------------------------------------- 跑一輪
def make_sources(*, camera=True, lidar=True, sigma_max=0.0025,
                 cam_min_dt=0.0, r_inflate=1.5, lidar_yaw=True,
                 clock_fix=True):
    # clock_fix=False 是把時鐘偵測整個關掉 (兩個判斷條件都設成天文數字),
    # 拿來做 A/B —— 那是 2026-09-10 那一輪的行為。
    cmo = 1.0 if clock_fix else 1e9
    cft = 0.1 if clock_fix else 1e9
    return {
        'camera': AbsSource('camera', enabled=camera, sigma_floor=CAM_SIGMA,
                            r_inflate=r_inflate, min_dt=cam_min_dt,
                            clock_max_offset=cmo, clock_future_tol=cft, use_yaw=False),
        'lidar': AbsSource('lidar', enabled=lidar, sigma_floor=LID_SIGMA,
                           r_inflate=r_inflate, sigma_max=sigma_max,
                           clock_max_offset=cmo, clock_future_tol=cft,
                           use_yaw=lidar_yaw, yaw_sigma=LID_YAW_SIGMA),
    }


def run(d, *, camera=True, lidar=True, rewind=True, wheel=True, calib=True,
        abs_reject_time=3.0, **src_kw):
    src = make_sources(camera=camera, lidar=lidar, **src_kw)
    f = FusionEkf(rewind=rewind, rewind_horizon=0.4,
                  abs_reject_time=abs_reject_time,
                  forward_deg=FORWARD_DEG, wheel_scale=CFG_SCALE,
                  sigma_acc=0.05, sigma_k=0.0)
    det = WheelStillDetector()
    f.set_pose(d['px'][0], d['py'][0], d['yaw'][0], d['t'][0])
    if calib:
        m = d['t'] < 1.5
        f.ins.x[4] = float(d['gyro'][m].mean())
        f.ins.x[5] = float(d['acc'][m].mean())

    meas = d['meas']
    mi = 0
    errs, yerrs, nees = [], [], []
    for i in range(1, len(d['t'])):
        ti = float(d['t'][i])
        det.add(ti, d['W'][:, i], d['gyro'][i])
        still = det.is_still()
        v_wheel = None
        if wheel:
            # **`r * median(omega)`, 不要先乘 scale。** WheelIns 的量測模型是
            # `v_wheel = v / k`, 尺度是狀態的一部分 —— 先乘進去的話濾波器會再
            # 除一次, 速度就固定偏 (1-k) 倍 (實測 1.8 m/s 時慢 126 mm/s,
            # 位置每秒偏 12 cm, 然後絕對量測全部被卡方閘門擋掉)。
            v_wheel = float(np.median(d['W'][:, i])) * RADIUS
        # 同側前後輪: (FL, RL) 與 (FR, RR) —— 跟 WheelReader.read 一樣取 max
        spread = max(abs(d['W'][0, i] - d['W'][2, i]),
                     abs(d['W'][1, i] - d['W'][3, i]))
        f.step(Step(ti, float(d['acc'][i]), float(d['gyro'][i]),
                    v_wheel=v_wheel, spread=spread, still=still,
                    yaw_meas=float(d['yaw'][i]), yaw_sigma=0.02, dt=DT))

        # 所有「已經到達」的絕對量測
        while mi < len(meas) and meas[mi]['arrive'] <= ti:
            mm = meas[mi]
            mi += 1
            s = src[mm['src']]
            if not s.enabled:
                continue
            # **跟節點走同一條路**: 先把時戳換到濾波器的時鐘上, 再判收不收。
            # bench 如果跳過這一步, 時鐘那一類的 bug 就測不到 (而那正是
            # 2026-09-10 那一輪踩到的東西)。
            mt = s.to_filter_clock(mm['t'], f.ins.t)
            ok, _ = s.accept(mt, mm['reported'], ti)
            if not ok:
                continue
            sigma = s.sigma_for(mm['reported'])
            am = AbsMeas(mt, mm['src'], mm['x'], mm['y'], sigma,
                         yaw=mm['yaw'], yaw_sigma=s.yaw_sigma)
            if f.absolute(am, use_yaw=s.use_yaw):
                s.mark_used(mt)
            else:
                s.counts['gate'] += 1

        r = np.array([f.pos[0] - d['px'][i], f.pos[1] - d['py'][i]])
        errs.append(float(np.linalg.norm(r)))
        yerrs.append(abs(wrap_pi(f.yaw - d['yaw'][i])))
        # NEES = 誤差用**濾波器自己報的共變異數**正規化。2 個自由度的理想值是
        # 2.0: 遠大於 2 = 過度自信 (P 太小), 遠小於 2 = 過度保守。
        # **這是唯一能判斷 P 誠不誠實的指標**, RMS 看不出來。
        Pp = f.ins.P[0:2, 0:2]
        try:
            nees.append(float(r @ np.linalg.inv(Pp) @ r))
        except np.linalg.LinAlgError:
            pass
    return np.array(errs), np.array(yerrs), f, src, np.array(nees)


def hold_baseline(d, which, use_delay=True):
    """「只用這一條」的體感: 60 Hz 去問它, 拿到的是最新一則 (含延遲與空窗)。"""
    meas = [m for m in d['meas'] if m['src'] == which]
    errs, gaps = [], []
    cur, last_t = None, None
    j = 0
    for i in range(1, len(d['t'])):
        ti = float(d['t'][i])
        while j < len(meas) and (meas[j]['arrive'] if use_delay else meas[j]['t']) <= ti:
            cur, last_t = meas[j], ti
            j += 1
        if cur is None:
            continue
        errs.append(math.hypot(cur['x'] - d['px'][i], cur['y'] - d['py'][i]))
        gaps.append(ti - last_t)
    return np.array(errs), np.array(gaps)


def stat(e):
    if len(e) == 0:
        return '  (沒有輸出)'
    return (f'RMS {np.sqrt((e ** 2).mean()) * 100:7.2f} cm  '
            f'p95 {np.percentile(e, 95) * 100:7.2f}  '
            f'最大 {e.max() * 100:8.2f}')


# ---------------------------------------------------------------------- 主程式
def main():
    line = '=' * 92
    print(line)
    print('  car_loc_fusion 離線 A/B —— 8 字形 90 秒, 60 Hz IMU + 輪速,')
    print('  相機 30 Hz (2.1 cm, 延遲 79.5 ms), LiDAR 10 Hz (3.5 cm/2.9°, 延遲 50 ms)')
    print(line)

    d = make_truth(seed=1, slips=((25.0, 26.0, 3.0), (55.0, 56.0, 4.0)))

    print('\n[A] 融合 vs 單獨用一種 (同一條軌跡, 同一份感測器讀數)')
    e_cam, _ = hold_baseline(d, 'camera')
    e_lid, _ = hold_baseline(d, 'lidar')
    e_dr, y_dr, f_dr, _, _ = run(d, camera=False, lidar=False)
    e_f, y_f, f_all, src, nees_f = run(d)
    print(f'  只有相機 (60 Hz 拿最新一則)  {stat(e_cam)}')
    print(f'  只有 LiDAR (60 Hz 拿最新一則){stat(e_lid)}')
    print(f'  只有航位推算 (IMU+輪速)      {stat(e_dr)}  yaw {np.degrees(y_dr).mean():.2f}°')
    print(f'  **全部融合**                 {stat(e_f)}  yaw {np.degrees(y_f).mean():.2f}°')
    print(f'    收件狀況: {f_all.report()}')
    print('  「60 Hz 拿最新一則」是下游 (TF / nav / 控制) 真正感受到的誤差 —— 它包含')
    print('  **延遲**與**空窗**, 所以比那條路自己報的精度差。融合把兩者都補掉: 高頻由')
    print('  遞推撐著, 絕對量測只負責把漂移按住。')

    print('\n[B] 延遲補償 (倒帶重放) 值多少 —— 車速越快差越多')
    print('    絕對量測的時戳是**過去**的。當成「現在」直接更新 = 灌 v x delay 的誤差,')
    print('    而且方向跟著車頭轉, 平均不掉。')
    for vp, name in ((0.6, '慢速 0.6'), (1.8, '中速 1.8'), (3.0, '高速 3.0')):
        dv = make_truth(seed=2, dur=60.0, v_peak=vp, stops=False)
        e_on, _, f_on, _, _ = run(dv, rewind=True)
        e_off, _, f_off, _, _ = run(dv, rewind=False)
        d_rms = (np.sqrt((e_off ** 2).mean()) - np.sqrt((e_on ** 2).mean())) * 100
        print(f'  {name} m/s  倒帶   {stat(e_on)}')
        print(f'  {" " * len(name)}      不倒帶 {stat(e_off)}  差 {d_rms:5.1f} cm '
              f'(理論 v x delay = {vp * CAM_DELAY * 100:4.1f} cm)')
        print(f'  {" " * len(name)}      倒帶 {f_on.counts["rewind"]} 次 / 重放 '
              f'{f_on.counts["replay"]} 步; 不倒帶時閘門擋掉 {f_off.counts["gate"]} 則')

    print('\n[C] 相機被擋住 (柱子後面 / YOLO 漏偵測) —— 30~40 秒整段沒有輸出')
    db = make_truth(seed=3, cam_blackout=((30.0, 40.0),))
    seg = (db['t'][1:] >= 30.0) & (db['t'][1:] < 40.0)
    e_cb, _ = hold_baseline(db, 'camera')
    e_fb, _, _, _, _ = run(db)
    e_fb_nolid, _, _, _, _ = run(db, lidar=False)
    print(f'  只有相機, 遮蔽期間          {stat(e_cb[seg[:len(e_cb)]])}')
    print(f'  融合 (相機+LiDAR+推算)      {stat(e_fb[seg])}')
    print(f'  融合但沒有 LiDAR            {stat(e_fb_nolid[seg])}')
    print('  相機那條路的可用性由**中斷**決定, 不是由平均誤差。少一個來源不是「變不準」')
    print('  而是「還在跑」: 遞推把空窗接起來, 誤差以航位推算的速率慢慢長。')

    print('\n[D] LiDAR 鎖到 180 度 (長方形房間的對稱解, 而且回不來) 45~60 秒')
    dl = make_truth(seed=4, lidar_lock=((45.0, 60.0),))
    for cam_on, tag in ((True, '相機也在 (兩個來源)'), (False, '只有 LiDAR')):
        e_g, _, f_g, sg, _ = run(dl, camera=cam_on, sigma_max=0.0025)
        e_n, _, f_n, sn, _ = run(dl, camera=cam_on, sigma_max=0.0)
        print(f'  {tag}')
        print(f'    有 sigma 閘門  {stat(e_g)}  sigma擋{sg["lidar"].counts["sigma"]} '
              f'閘門擋{f_g.counts["gate"]} 強制{f_g.counts["forced"]}')
        print(f'    沒有           {stat(e_n)}  sigma擋{sn["lidar"].counts["sigma"]} '
              f'閘門擋{f_n.counts["gate"]} 強制{f_n.counts["forced"]}')
    print('  兩個來源都在的時候卡方閘門一個人就夠了 —— 相機一直在提供正確的位置,')
    print('  鎖住的 LiDAR 跟它差 10 公尺, NIS 破表。**只有 LiDAR 的時候就不是了**:')
    print('  鎖住之後那些量測彼此一致, 閘門連續擋幾秒就會觸發逃生口 (強制接受),')
    print('  然後估計整個被搬到鏡射的位置。sigma 閘門不看濾波器狀態, 所以擋得住。')

    print('\n[E] 絕對量測全部掉線 40~70 秒 -> 退化成航位推算, 回來之後拉得回來嗎')
    dd = make_truth(seed=5, cam_blackout=((40.0, 70.0),), dur=100.0)
    dd['meas'] = [m for m in dd['meas']
                  if not (m['src'] == 'lidar' and 40.0 <= m['t'] < 70.0)]
    e_dd, _, f_dd, _, _ = run(dd)
    tt = dd['t'][1:]
    for lo, hi, tag in ((20.0, 40.0, '掉線前'), (40.0, 70.0, '掉線中'),
                        (70.0, 71.0, '回來後 1 秒'), (71.0, 100.0, '回來後')):
        m = (tt >= lo) & (tt < hi)
        print(f'  {tag:12s} {stat(e_dd[m[:len(e_dd)]])}')
    print(f'  逃生口強制接受 {f_dd.counts["forced"]} 次, 閘門擋掉 {f_dd.counts["gate"]} 則。')
    print('  掉線期間 P 是**跟著長大**的, 所以量測一回來 NIS 就過得了關, 不需要逃生口')
    print('  —— 這是對的。逃生口要處理的是另一種情況: P 很小但**是錯的**, 見 [F]。')

    print('\n[F] 綁架 (車子被瞬間搬走 3 m) —— P 很小但是錯的, 逃生口在這裡')
    print('    IMU 與輪速**看不到**這件事 (它們量的是運動不是位置), 所以濾波器對')
    print('    自己非常有信心, 而每一則正確的絕對量測都會被卡方閘門擋掉。')
    dk = make_truth(seed=6, kidnap=(50.0, 3.0, -1.5), dur=80.0)
    tk = dk['t'][1:]
    for art, tag in ((3.0, '有逃生口 (3 秒)'), (0.0, '沒有逃生口')):
        e_k, _, f_k, _, _ = run(dk, abs_reject_time=art)
        after = e_k[(tk >= 52.0) & (tk < 80.0)]
        # 多久之後回到 20 cm 以內
        idx = np.argmax(e_k[(tk >= 50.0)] < 0.20) if (e_k[(tk >= 50.0)] < 0.20).any() else -1
        rec = f'{idx * DT:.1f} s' if idx >= 0 else '**沒有回來**'
        print(f'  {tag:16s} 綁架後 2 秒起 {stat(after)}  收斂 {rec}  '
              f'強制 {f_k.counts["forced"]} 次')

    print('\n[G] 不要把已經濾波過的輸出當成獨立量測 (min_dt / r_inflate)')
    print('    相機發的是**卡爾曼濾波之後**的位置, 連續兩則的誤差是相關的 (bench 用')
    print('    rho=0.7 的 AR(1) 生)。EKF 的更新式假設獨立, 全速吃 = 同一份資訊算很多次。')
    print('    判斷的指標是 **NEES** (誤差用濾波器自己報的 P 正規化), 2 自由度的理想值')
    print('    是 2.0 —— 遠大於 2 就是過度自信。RMS 看不出這件事。')
    for md, ri, tag in ((0.0, 1.0, '全吃 30 Hz, R 不放大'),
                        (0.0, 1.5, '全吃 30 Hz, R x1.5'),
                        (0.1, 1.5, '限流到 10 Hz + R x1.5'),
                        (0.1, 2.5, '10 Hz + R x2.5')):
        e_x, _, f_x, sx, nees_x = run(d, cam_min_dt=md, r_inflate=ri)
        print(f'  {tag:24s} {stat(e_x)}  NEES {np.median(nees_x):6.2f}  '
              f'閘門擋 {f_x.counts["gate"]:4d}')
    print('  **量出來跟我原本想的不一樣。** 原本準備用 min_dt 限流 (「30 Hz 的相關量測')
    print('  只該算 10 Hz」), 但限流之後 NEES 反而變差 —— 因為兩次更新之間 P 長回來的')
    print('  速度追不上誤差長大的速度, 少吃的那幾則資訊補不回來。**R x1.5 一項就夠了**,')
    print('  所以 min_dt 預設是 0 (功能留著: 真車上如果來源的平滑比模擬更重, 再打開)。')
    print('  重點是判斷的方式: 看 NEES 跟擋掉的數量, 不是只看 RMS。過度自信的濾波器在')
    print('  乾淨資料上 RMS 更好看, 代價是遇到離群值 ([D]/[F]) 時完全沒有防禦。')
    print()

    print('\n[H] 來源的時鐘跟 IMU 不同基準 (2026-09-10 那一輪: /rgb 早 625.55 秒)')
    print('    這不是假設出來的情境 —— 那一輪的 fus_ 整段凍結在起點, sigma 0.42 mm。')
    dc = make_truth(seed=1, slips=())
    for off in (0.0, 0.5, 625.55):
        for fix in (True, False):
            dd = dict(dc)
            dd['meas'] = [dict(m) for m in dc['meas']]
            for m in dd['meas']:
                if m['src'] == 'camera':
                    m['t'] += off          # 只有時戳偏掉, 到達時刻不變
            e_c, _, f_c, _, _ = run(dd, clock_fix=fix)
            tag = '有時鐘偵測' if fix else '關掉偵測'
            print(f'  相機時戳偏 {off:7.2f} s  {tag}  {stat(e_c)}  '
                  f'sigma {f_c.sigma_pos() * 1000:6.2f} mm  '
                  f'abs_log {len(f_c.abs_log):4d}  未來 {f_c.counts["future"]:4d}')
    print('  沒有偵測的話: 時戳在未來 -> 緩衝區的過期判斷永遠是負的 -> 量測永遠不會')
    print('  被丟掉 -> 每次倒帶都把整段歷史重放一次 -> 同一筆被算幾百次 -> P 被壓垮')
    print('  -> 之後所有量測都被閘門擋掉 -> **估計凍結在起點**。')
    print('  0.5 秒那一組差得沒那麼離譜, 但一樣要修 —— 判斷的依據不是「差多少」而是')
    print('  **方向**: 真正的延遲只會讓時戳變舊, 持續落在未來就是時鐘不對。')
    print('  **偵測救得回「不要壞掉」, 救不回精度** —— 扣掉偏移之後每一則量測看起來')
    print('  都像剛剛才發生, 延遲補償等於沒有。要拿回 v x 80 ms 只能去源頭修時鐘。')

    print('\n' + line)
    ok = (np.sqrt((e_f ** 2).mean()) < 0.05
          and np.sqrt((e_f ** 2).mean()) < np.sqrt((e_dr ** 2).mean()))
    print(f'  {"PASS" if ok else "FAIL"}: 融合 {np.sqrt((e_f ** 2).mean()) * 100:.2f} cm '
          f'vs 只有航位推算 {np.sqrt((e_dr ** 2).mean()) * 100:.2f} cm')
    print(line)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
