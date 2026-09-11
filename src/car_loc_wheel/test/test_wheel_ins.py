#!/usr/bin/env python3
"""合成資料的 A/B —— 回答「只用 IMU 一定會飄嗎, 加輪速差多少」。

    python3 test/test_wheel_ins.py

`replay_csv.py` 跑的是**真的 Isaac 資料**, 但那份 CSV 裡沒有原始的加速度計讀數,
所以量不到「純 IMU vs IMU+輪速」這件最重要的事。這支用合成資料補上: 加速度計、
陀螺儀、四輪轉速全部由同一條軌跡生出來, 誤差模型 (零偏、雜訊、打滑) 是我們自己
放進去的, 所以每一項的代價都可以單獨關掉再量一次。

兩支要一起看: 合成資料是「假設都成立」的乾淨情況, 真資料才有側滑與離散化。

軌跡: 60 Hz, 60 秒, 8 字形, 每 15 秒平滑停 3 秒。
誤差: 陀螺零偏 0.004 rad/s, 加速度零偏 0.02 m/s^2, 輪徑尺度真值 0.976,
      另外在 4 個時段製造打滑 (輪子空轉, 車子沒有跟上)。
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from car_loc_wheel.wheel_ins import (WheelIns, WheelReader,      # noqa: E402
                                     WheelStillDetector, wrap_pi)

DT = 1.0 / 60.0
T = 60.0
RADIUS = 0.075
TRUE_SCALE = 0.976          # 有效輪半徑 / 幾何輪半徑 (sim_data.csv 量到 0.976)
FORWARD_DEG = -90.0         # car.usd 的車頭是 -Y

GYRO_BIAS = 0.004
ACC_BIAS = 0.02
GYRO_NOISE = 0.004
ACC_NOISE = 0.02
WHEEL_NOISE = 0.05          # rad/s
# 打滑時段 (秒): 起、迄、輪子比車子快多少倍
SLIPS = ((12.0, 13.5, 3.0), (28.0, 29.0, 5.0), (41.0, 42.5, 2.5), (52.0, 53.0, 4.0))


def make_truth(seed=0, stops=True, dur=T):
    """8 字形軌跡。回傳每一步的真值與感測器讀數。

    `stops=True`  每 15 秒平滑停 4.5 秒 (走走停停, 一般的使用情境)
    `stops=False` 起步後**一路開到底不停** —— 這才看得出純 IMU 的 t^2

    兩者都在前 2 秒讓車子完全靜止, 開機零偏校正要在那裡做。**這一段必須是
    真的靜止**: 校正窗裡只要車子有在轉, 量到的「零偏」就會把真實的角速度算進去,
    之後每一秒都用那個錯的值去扣 —— 實測那會讓 60 秒累積 150 度以上的 yaw 誤差。
    這不是合成資料的怪癖, 真車上一模一樣 (節點的靜止校正因此要等輪速說靜止)。
    """
    rng = np.random.default_rng(seed)
    n = int(dur / DT)
    t = np.arange(n) * DT

    # 速度剖面。用 tanh 過渡, 不要用階躍 —— 階躍會生出無限大的加速度。
    v = np.zeros(n)
    for i, ti in enumerate(t):
        if ti < 2.0:
            continue                       # 開機靜止段 (零偏校正用)
        if stops:
            ph = ti % 15.0
            v[i] = 0.6 * 0.5 * (np.tanh((ph - 2.0) / 0.4)
                                - np.tanh((ph - 12.5) / 0.4))
        else:
            v[i] = 0.6 * 0.5 * (1.0 + np.tanh((ti - 2.5) / 0.4))
    wz = 0.8 * np.sin(2 * np.pi * t / 20.0) * (v > 0.05)

    yaw = np.cumsum(wz) * DT
    a_fwd = np.gradient(v) / DT
    px = np.cumsum(v * np.cos(yaw + math.radians(FORWARD_DEG))) * DT
    py = np.cumsum(v * np.sin(yaw + math.radians(FORWARD_DEG))) * DT

    # --- 感測器 ---
    gyro = wz + GYRO_BIAS + GYRO_NOISE * rng.standard_normal(n)
    acc = a_fwd + ACC_BIAS + ACC_NOISE * rng.standard_normal(n)

    # 四輪轉速。真值是 v / (scale * r)。打滑時段裡:
    #   偶數段 —— **單輪**空轉, 同側前後輪的差看得到 -> slip_spread 那道防線抓
    #   奇數段 —— **整側兩顆一起**空轉, 同側差看不出來 -> 只有卡方閘門 (跟 IMU
    #             遞推比) 抓得到。故意讓兩道防線各有各的守備範圍。
    w_true = v / (TRUE_SCALE * RADIUS)
    W = np.tile(w_true, (4, 1)) + WHEEL_NOISE * rng.standard_normal((4, n))
    slip_mask = np.zeros(n, dtype=bool)
    for k, (t0, t1, mult) in enumerate(SLIPS):
        m = (t >= t0) & (t < t1)
        slip_mask |= m
        W[0, m] *= mult
        if k % 2:
            W[2, m] *= mult
    return dict(t=t, v=v, wz=wz, yaw=yaw, px=px, py=py,
                gyro=gyro, acc=acc, W=W, slip=slip_mask)


def run(d, *, wheel=True, slip_gate=True, zupt=True, zaru=True, calib=True,
        wheel_scale=TRUE_SCALE, **kw):
    ins = WheelIns(forward_deg=FORWARD_DEG, wheel_scale=wheel_scale,
                   sigma_acc=0.05, sigma_k=0.0, **kw)
    reader = WheelReader(radius=RADIUS)
    det = WheelStillDetector()
    ins.set_pose(d['px'][0], d['py'][0], d['yaw'][0], d['t'][0])
    if calib:
        # 開機靜止校正: 前 0.9 秒車子是停著的, 拿那段量零偏
        m = d['t'] < 1.5
        ins.x[4] = float(d['gyro'][m].mean())
        ins.x[5] = float(d['acc'][m].mean())

    errs, yerrs = [], []
    for i in range(1, len(d['t'])):
        ins.predict(d['t'][i], d['acc'][i], d['gyro'][i])
        det.add(d['t'][i], d['W'][:, i], d['gyro'][i])
        still = det.is_still()
        if still and zupt:
            if ins.still_since is None:
                ins.still_since = d['t'][i]
            ins.zupt()
            ins.zero_accel(d['acc'][i])
            if zaru:
                ins.zaru(d['gyro'][i])
            if d['t'][i] - ins.still_since >= ins.anchor_after:
                ins.anchor_position()
        else:
            ins.still_since = None
            ins.anchor = None
        if wheel:
            v_w, spread, _ = reader.read(None, list(d['W'][:, i]))
            ins.update_wheel(v_w, spread if slip_gate else 0.0)
        errs.append(math.hypot(ins.x[0] - d['px'][i], ins.x[1] - d['py'][i]))
        yerrs.append(abs(math.degrees(wrap_pi(ins.yaw - d['yaw'][i]))))
    e = np.array(errs)
    return {'rms': float(np.sqrt((e ** 2).mean())), 'p95': float(np.percentile(e, 95)),
            'max': float(e.max()), 'final': float(e[-1]), 'curve': e,
            'yaw': float(np.array(yerrs).max()), 'counts': dict(ins.counts)}


def row(name, r):
    print(f'  {name:34s} {r["rms"]:7.3f} {r["p95"]:7.3f} {r["max"]:7.3f} '
          f'{r["final"]:7.3f} {r["yaw"]:6.2f}°')


def head(title):
    print(f'\n{title}')
    print(f'  {"":34s} {"RMS":>7} {"p95":>7} {"最大":>7} {"終點":>7} {"yaw":>7}')


def main():
    d = make_truth(stops=True)
    dn = make_truth(stops=False, dur=180.0)
    dist = float(np.sum(d['v']) * DT)
    print('=' * 92)
    print(f'  合成軌跡 (8 字形 @ {1 / DT:.0f} Hz), 兩條:')
    print(f'    走走停停 —— {T:.0f} 秒, 每 15 秒停 4.5 秒, 走了 {dist:.1f} m '
          f'(靜止 {100 * float((d["v"] < 0.05).mean()):.0f}%)')
    print(f'    一路不停 —— 180 秒起步後開到底, 走了 '
          f'{float(np.sum(dn["v"]) * DT):.1f} m '
          f'(靜止 {100 * float((dn["v"] < 0.05).mean()):.0f}%)')
    print(f'  誤差模型: 陀螺零偏 {GYRO_BIAS} rad/s, 加速度零偏 {ACC_BIAS} m/s^2, '
          f'輪徑尺度真值 {TRUE_SCALE}, 4 段打滑')
    print('=' * 92)

    raw, pure, full = run(d, wheel=False, zupt=False), run(d, wheel=False), run(d)
    raw_n, pure_n, full_n = (run(dn, wheel=False, zupt=False),
                             run(dn, wheel=False), run(dn))

    head('[A] 加輪速值多少')
    row('走走停停  純慣性 (開迴路積分)', raw)
    row('走走停停  純 IMU + ZUPT', pure)
    row('走走停停  IMU + 輪速', full)
    row('一路不停  純慣性 (開迴路積分)', raw_n)
    row('一路不停  純 IMU + ZUPT', pure_n)
    row('一路不停  IMU + 輪速', full_n)
    print('  「純 IMU + ZUPT」那兩列**對純 IMU 是偏有利的** —— 它的靜止偵測仍然')
    print('  用輪速 (幾乎不會誤判)。真正只有 IMU 的話還要靠加速度變異數去猜靜止,')
    print('  會更差 (見 car_loc_imu 的 still_var 那一段)。')
    print(f'  走走停停 {pure["rms"] / max(full["rms"], 1e-9):.0f} 倍, '
          f'一路不停 {pure_n["rms"] / max(full_n["rms"], 1e-9):.0f} 倍 '
          '—— **差距取決於「多久停一次」**: ZUPT 只在停車時有效, 輪速一直有效。')

    head('[B] 誤差長大的**形狀**不一樣 (這比倍數重要)')
    print('  純 IMU: 加速度的誤差積分兩次 -> 位置誤差 ~ t^2, 停著不動也照樣長。')
    print('  加輪速: 輪速是速度的直接量測 -> 位置誤差 ~ 走過的距離, 停著就不長。')
    print('  一路不停那條 (180 秒) 每 45 秒區間裡的**最大**誤差 —— 單點誤差會隨')
    print('  車頭方向擺盪, 看區間最大值才看得出趨勢:')
    print(f'  {"":30s} {"0-45 s":>9} {"45-90":>9} {"90-135":>9} {"135-180":>9}   倍率')
    for name, r in (('純慣性 (開迴路積分)', raw_n), ('純 IMU + ZUPT', pure_n),
                    ('IMU + 輪速', full_n)):
        c = r['curve']
        k = len(c) // 4
        pts = [float(c[i * k:(i + 1) * k].max()) for i in range(4)]
        print(f'  {name:30s} ' + ' '.join(f'{v:8.3f}m' for v in pts)
              + '   ' + ' '.join(f'{pts[i + 1] / max(pts[i], 1e-9):.1f}x'
                                 for i in range(3)))
    # 用 log-log 的斜率把「二次 vs 一次」量出來, 不要用眼睛看倍率
    print(f'  {"":30s} 誤差 ~ t^p 的 p (log-log 最小平方):')
    for name, r in (('純慣性 (開迴路積分)', raw_n), ('純 IMU + ZUPT', pure_n),
                    ('IMU + 輪速', full_n)):
        c = r['curve']
        i0 = int(10.0 / DT)                       # 跳過起步的暫態
        tt = np.arange(i0, len(c)) * DT
        cc = np.maximum.accumulate(c[i0:])        # 取到目前為止的最大值 -> 單調
        pexp = float(np.polyfit(np.log(tt), np.log(np.maximum(cc, 1e-9)), 1)[0])
        print(f'  {name:30s} p = {pexp:.2f}')
    print('  怎麼讀這幾個指數:')
    print('   * 只有 IMU 的三列都明顯 **> 1** (1.4~1.5); 加輪速之後掉到 ~1.1。')
    print('     p ~ 1 的意思是誤差跟著**走過的距離**走, p > 1 是跟著**時間**走。')
    print('   * 為什麼不是課本說的 2.0: 課本的 0.5*b*t^2 假設零偏在**世界座標**')
    print('     固定不動。實際上加速度計的零偏固定在**車體**上, 8 字形一直在轉,')
    print('     那個假加速度的方向跟著轉, 於是有一部分自己抵消掉了。另外速度上限')
    print('     (v_max) 也會把發散截斷。所以量到的指數比 2 小 —— 但仍然遠大於 1,')
    print('     而「大於 1」才是問題所在。')
    print('   * 形狀不一樣代表能撐多久的量級不一樣: p=1.5 的話時間拉長 2 倍誤差')
    print('     變 2.8 倍, p=1 只變 2 倍; 而且純 IMU 那條**停著不動也照樣長**。')

    head('[C] 各個機制值多少 —— 左邊走走停停, 右邊一路不停')
    for name, kw in (('全開', {}),
                     ('不擋打滑 (slip_gate 關)', dict(slip_gate=False)),
                     ('沒有 ZUPT / ZARU', dict(zupt=False)),
                     ('沒有 ZARU (有 ZUPT)', dict(zaru=False)),
                     ('沒有開機靜止校正', dict(calib=False))):
        a, b = run(d, **kw), run(dn, **kw)
        print(f'  {name:34s} RMS {a["rms"]:7.3f} / {b["rms"]:7.3f}   '
              f'最大 {a["max"]:7.3f} / {b["max"]:7.3f}   '
              f'yaw {a["yaw"]:5.2f}° / {b["yaw"]:5.2f}°')
    print('  ZUPT / ZARU 在**走走停停**那條上幾乎看不出差別 —— 輪速本來就會說')
    print('  「現在沒在動」, ZUPT 的資訊是重複的。它們真正的價值在:')
    print('    (a) 一路不停那條: 沒有停車就沒有 ZARU, 陀螺零偏沒人修 -> yaw 一直漂;')
    print('    (b) 輪速掉線時的備援 (那時就退回純 IMU 了)。')
    print('  「沒有開機靜止校正」那一列看不出差別, 是因為兩條軌跡開頭都有 2 秒')
    print('  靜止, ZARU 與零加速度更新線上就把零偏修掉了。開機校正真正的價值在')
    print('  **車子一開始就在動**的時候 —— 那時 ZARU 也沒有機會, 零偏會一路帶著跑。')
    print('  反過來也要小心: 校正窗裡車子如果沒有真的停著, 量到的「零偏」會把真實')
    print('  的角速度算進去, 之後每一秒都用錯的值去扣 (實測 60 秒累積 150 度以上)。')

    head('[D] 輪徑尺度給錯的代價 (系統性誤差, 不會因為停車消失)')
    for sc in (1.0, 0.95, TRUE_SCALE):
        row(f'wheel_scale = {sc:.3f} ({100 * (sc / TRUE_SCALE - 1):+.1f}% 誤差)',
            run(d, wheel_scale=sc))
    print(f'  走了 {dist:.1f} m; 2.4% 的尺度誤差就是 {0.024 * dist:.2f} m 的里程誤差,')
    print('  而且它跟走多遠成正比 —— 這是唯一真正值得離線校正的參數。')

    print('\n[E] 打滑偵測抓到了嗎')
    n_slip = int(d['slip'].sum())
    on, off = run(d), run(d, slip_gate=False)
    print(f'  資料裡有 {n_slip} 筆打滑 (2 段單輪空轉, 2 段整側空轉)。')
    print(f'  全開: R 放大 {on["counts"]["slip"]} 筆, 卡方閘門擋掉 '
          f'{on["counts"]["rejected"]} 筆')
    print(f'  關掉 slip_gate: 卡方閘門擋掉 {off["counts"]["rejected"]} 筆 '
          f'(它接手了一部分)')
    print('  兩道防線守備範圍不同: 同側前後輪轉速差抓「單輪空轉」, 卡方閘門抓')
    print('  「整側一起空轉」—— 後者同側差看不出來, 只有跟 IMU 遞推比才知道。')
    print('  合成資料上兩者幾乎等價 (卡方閘門一個人就夠), 但**真資料上不是**:')
    print('  sim_data.csv 實測 RMS 0.69 -> 0.58 m, 見 test/replay_csv.py。')
    print('=' * 92)

    ok = (full['rms'] < pure['rms'] and full_n['rms'] < pure_n['rms']
          and full['rms'] < 0.5)
    print(('  PASS' if ok else '  FAIL')
          + f': 走走停停 {full["rms"]:.3f} vs 純 IMU {pure["rms"]:.3f} m; '
            f'一路不停 {full_n["rms"]:.3f} vs {pure_n["rms"]:.3f} m')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
