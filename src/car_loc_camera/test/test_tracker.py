#!/usr/bin/env python3
"""離線驗證投影與濾波器 —— 不需要 ROS, 不需要 YOLO, 不需要 Isaac。

    cd src/car_loc_camera && python3 test/test_tracker.py

量四件事:
  1. 等速卡爾曼濾波把逐幀量測雜訊壓成多少
  2. yaw (由速度方向推) 準到什麼程度, 以及 allow_reverse 的風險
  3. 離群值閘門擋不擋得住誤判, 以及車被搬走時逃不逃得回來
  4. peek() 外推到「現在」有沒有效, 以及它不能改動濾波器狀態

拿真實資料跑的版本在 scripts/replay_camera_csv.py (吃 collect_data_node 的 CSV)。
"""
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from car_loc_camera.projection import GroundProjection   # noqa: E402
from car_loc_camera.tracker import ConstVelTracker       # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB = os.path.join(HERE, '..', 'config', 'camera_ground.yaml')
PARAMS = os.path.join(HERE, '..', 'config', 'camera_loc.yaml')
RATE = 20.0
DT = 1.0 / RATE

# 量測雜訊與預設 accel_sigma 都從 config 讀 —— 寫死的話改了校正檔就會偷偷
# 對不上, 而這支測試印出來的數字是 camera_loc.yaml 註解引用的來源。
_CAL = yaml.safe_load(open(CALIB))
_PAR = yaml.safe_load(open(PARAMS))['camera_localizer']['ros__parameters']
MEAS_SIGMA = float(_CAL['residual_rms'])
DEFAULT_SA = float(_PAR['accel_sigma'])


def test_projection():
    proj = GroundProjection(yaml.safe_load(open(CALIB)))
    print(f'校正檔: {proj.ref_w:.0f}x{proj.ref_h:.0f}, 殘差 {proj.sigma * 100:.2f} cm, '
          f'延遲 {proj.delay * 1000:.0f} ms')
    cx, cy = proj(proj.center[0], proj.center[1])
    print(f'  影像中心 -> ({cx:+.4f}, {cy:+.4f}) m')
    # 換解析度: 同一個實體位置在不同解析度下要投影到同一個地方
    a = proj(500.0, 400.0, 1920, 1536)
    b = proj(250.0, 200.0, 960, 768)
    d = math.hypot(a[0] - b[0], a[1] - b[1])
    print(f'  1920x1536 的 (500,400) 與 960x768 的 (250,200): 差 {d * 1000:.3f} mm '
          f'{"OK" if d < 1e-6 else "<-- 解析度換算壞了"}')


# 三種機動程度 (名稱, 角頻率, 振幅) —— 峰值加速度差 6 倍
REGIMES = (('慢速 (teleop 上限 0.6 m/s)', 0.25, 1.6),
           ('中速', 0.45, 1.6),
           ('高機動', 0.60, 1.6))


def run_track(sa, meas_sigma=MEAS_SIGMA, T=40.0, allow_reverse=False, seed=0,
              outliers=0.0, kidnap=False, w=0.6, amp=1.6):
    rng = np.random.default_rng(seed)
    tr = ConstVelTracker(accel_sigma=sa, meas_sigma=meas_sigma,
                         allow_reverse=allow_reverse)
    ep, ey, ev, amax, n_used = [], [], [], 0.0, 0
    for k in range(1, int(T * RATE) + 1):
        t = k * DT
        gx, gy = amp * math.sin(w * t), 0.7 * amp * math.sin(2 * w * t)
        vx = amp * w * math.cos(w * t)
        vy = 1.4 * amp * w * math.cos(2 * w * t)
        amax = max(amax, math.hypot(amp * w * w * math.sin(w * t),
                                    2.8 * amp * w * w * math.sin(2 * w * t)))
        gyaw = math.atan2(vy, vx)
        if kidnap and t > T / 2:                 # 車子被搬走 3 公尺
            gx, gy = gx + 3.0, gy - 2.0
        z = np.array([gx, gy]) + rng.normal(0, meas_sigma, 2)
        if outliers and rng.random() < outliers:  # 誤判到影子上
            z = z + rng.normal(0, 1.5, 2)
        n_used += tr.update(t, z)
        if k > 40:
            ep.append(math.hypot(tr.x[0] - gx, tr.x[1] - gy))
            ev.append(math.hypot(tr.x[2] - vx, tr.x[3] - vy))
            ey.append(abs(math.degrees(math.atan2(math.sin(tr.yaw - gyaw),
                                                  math.cos(tr.yaw - gyaw)))))
    return np.array(ep), np.array(ey), tr, n_used, np.array(ev), amax


def main():
    print('=== 投影 ===')
    test_projection()

    print('\n=== 外推到現在 (peek) ===')
    test_peek()

    print(f'\n=== 濾波: 量測雜訊 {MEAS_SIGMA * 100:.2f} cm (= 校正殘差) 被壓成多少 ===')
    print('  規則: accel_sigma 設成車子的**峰值加速度**。設太小比設太大危險得多 ——')
    print('  太小的時候不只誤差變大, 連卡方閘門都會開始擋掉正確的量測。')
    for name, w, amp in REGIMES:
        _, _, _, _, _, amax = run_track(2.0, w=w, amp=amp)
        print(f'\n  {name} (峰值加速度 {amax:.2f} m/s^2):')
        for sa in (1.0, 2.0, 3.0, 5.0, 8.0):
            ep, ey, tr, _, ev, _ = run_track(sa, w=w, amp=amp)
            tag = '  <-- 目前設定' if sa == DEFAULT_SA else ''
            print(f'    accel_sigma={sa:4.1f}: 位置 RMS '
                  f'{np.sqrt((ep ** 2).mean()) * 100:5.2f} cm, 速度 '
                  f'{np.sqrt((ev ** 2).mean()):5.3f} m/s, yaw '
                  f'{np.sqrt((ey ** 2).mean()):5.2f}°, 擋掉 '
                  f'{tr.n_rejected}/{tr.n_rejected + tr.n_accepted}{tag}')
    print('\n  yaw 的誤差大約就是 atan(速度估計誤差 / 車速) —— 這是「用移動方向')
    print('  當車頭」的物理上限, 不是參數問題。車越慢越差。')

    print('\n=== yaw: allow_reverse 的風險 (三個種子) ===')
    print('  倒車時移動方向跟車頭差 180 度, 而單一個 bbox 中心分不出來。')
    print('  allow_reverse 用連續性去猜, 但第一次定案定反了就一路錯下去:')
    for seed in (0, 1, 2):
        _, y0, _, _, _, _ = run_track(2.0, allow_reverse=False, seed=seed)
        _, y1, _, _, _, _ = run_track(2.0, allow_reverse=True, seed=seed)
        f0, f1 = np.sqrt((y0 ** 2).mean()), np.sqrt((y1 ** 2).mean())
        print(f'  seed={seed}: allow_reverse=False yaw RMS {f0:6.2f}°, '
              f'True {f1:6.2f}°{"  <-- 鎖到反方向" if f1 > 90 else ""}')

    print('\n=== 離群值閘門 (10% 的幀誤判到 1.5 m 外) ===')
    for out in (0.0, 0.10, 0.25):
        ep, _, tr, used, _, _ = run_track(2.0, outliers=out, seed=3)
        print(f'  誤判率 {out:4.0%}: 位置 RMS {np.sqrt((ep ** 2).mean()) * 100:5.2f} cm, '
              f'採信 {tr.n_accepted} / 擋掉 {tr.n_rejected}')

    print('\n=== 車子被搬走 (第 20 秒瞬間移動 3.6 m) ===')
    T = 40.0
    ep, _, tr, _, _, _ = run_track(2.0, kidnap=True, seed=4, T=T)
    # ep 從第 41 幀開始記, 搬走發生在 t > T/2
    jump = int(T / 2 * RATE) - 40
    after = ep[jump:]
    back = np.nonzero(after < 0.30)[0]
    if back.size:
        print(f'  搬走後 {back[0]} 幀 ({back[0] * DT:.2f} s) 回到 30 cm 以內, '
              f'之後 RMS {np.sqrt((after[back[0]:] ** 2).mean()) * 100:.2f} cm')
    else:
        print('  **沒回來** —— 逃生門壞了')
    print(f'  擋掉 {tr.n_rejected} 次。逃生門 (force_accept_after) 是讓它回得來的原因:')
    print('  連續被閘門擋掉這麼多次, 就代表錯的是濾波器不是量測, 整個重設到最新的量測上。')


def test_peek():
    """peek() 是給「下游想知道車現在在哪」用的外推。兩件事要成立:

    * **不能改動濾波器。** 它是旁路查詢, 不是一次 predict —— 如果它動了狀態,
      predict_rate 設多少就會影響定位結果, 那是最難查的那種 bug。
    * **等速段要真的補回來。** delay + 半個影像週期 = 100 ms 上下, 車速 2 m/s
      時那是 20 cm。
    """
    tr = ConstVelTracker(accel_sigma=5.0, meas_sigma=0.0153)
    v = 2.0
    for i in range(40):                     # 沿 x 等速跑
        t = i * DT
        tr.update(t, (v * t, 0.0))

    t_meas = 39 * DT
    x0, P0 = tr.x.copy(), tr.P.copy()
    lag = 0.10
    x, P = tr.peek(t_meas + lag)
    assert np.array_equal(tr.x, x0) and np.array_equal(tr.P, P0), \
        'peek() 改動了濾波器狀態'
    truth = v * (t_meas + lag)
    print(f'  等速 {v} m/s, 外推 {lag * 1000:.0f} ms:')
    print(f'    不外推 (直接用最新狀態) 差 {abs(x0[0] - truth) * 100:6.2f} cm')
    print(f'    peek 外推後         差 {abs(x[0] - truth) * 100:6.2f} cm')
    print(f'    共變異數 P[0,0] 長大 {P[0, 0] / P0[0, 0]:.2f} 倍 (下游看得出這是推的)')
    assert abs(x[0] - truth) < abs(x0[0] - truth), '外推沒有比不外推好'

    # 沒初始化的時候要回 (None, None), 不能爆
    assert ConstVelTracker().peek(1.0) == (None, None)
    print('  未初始化時回 (None, None) OK')


if __name__ == '__main__':
    main()
