#!/usr/bin/env python3
"""拿 collect_data_node 記的 CSV 算五條定位線的誤差。不需要 ROS。

    ./scripts/eval_loc_csv.py                       # 預設 car_run_data/sim_data.csv
    ./scripts/eval_loc_csv.py path/to/run.csv
    ./scripts/eval_loc_csv.py --sigma 0.005         # 換過濾門檻
    ./scripts/eval_loc_csv.py --no-filter           # 完全不濾, 看原始分布

**每一條路線用不一樣的判讀方式** —— 這一點是 2026-09-10 踩過的坑
--------------------------------------------------------------
把 LiDAR 那一套 (sigma 門檻 + 「|yaw 誤差|>90 度 = 追丟」) 套到別條線上, 會得到
完全錯誤的結論。實測那一輪的相機: 整體「追丟 25.9%」看起來很糟, 但

    車速 > 0.5 m/s 時「追丟」只有 1.7%, 而位置中位數是 0.7 cm

—— 因為**相機的 yaw 是從速度方向推的**, 車停著的時候它本來就沒有意義 (節點是
維持上一個值), 而那一輪 64% 的幀車子是停著的。同樣地, `cam_sigma` 是等速卡爾曼
濾波的共變異數, 尺度跟 `lid_sigma` 差一個數量級 (中位 0.013 vs 0.0017), 套 0.0025
的門檻會把**整段**擋掉 —— 然後印出「0 漏網」, 因為根本沒有幀通過。

所以: `lid` 用 sigma 門檻與追丟率; `cam` 依**車速**分組看 yaw; `imu`/`whl` 看
漂移率; `fus` 看「估計走過的路徑長 vs ground truth 走過的距離」(它的失效模式是
**凍結**, 不是變不準)。細節見 SIGMA_GATE_APPLIES 那一段的註解。

為什麼要過濾, 以及為什麼是這兩個條件
------------------------------------
**`lid_age < 0.3`** —— `_age` 是「這個值放多久了」。定位節點掛掉或 topic 斷掉
時, 最後一個值會被一路複製到檔案結束, 看起來像車子停在那裡不動。不濾掉就是
拿重複的舊值在算統計。

**`lid_sigma < 門檻`** —— 節點自己回報的位置 1-sigma。純雷射在長方形房間裡會
鎖到 180 度的對稱解 (見 car_loc_lidar/README.md 第 4 節), 鎖住之後**位置看起來
還很合理但 yaw 差 180 度**, 而且回不來。實測 sigma 對這件事非常靈敏:

    正常時 sigma 中位數 ~0.0015    追丟時 ~0.0145

門檻預設 **0.0025** 是量出來的, 不是猜的。三輪資料的實測 (`--sweep` 會重算):

    0.005   保留 82%, 有一輪漏掉 14 幀追丟 -> yaw RMS 被拉到 9.15 度
    0.0025  保留 71%, 三輪都是 0 漏網      -> yaw RMS 1.69 度

**看中位數, 不要只看 RMS。** 追丟幀的 yaw 誤差接近 180 度, 漏個十幾幀就能讓
RMS 翻倍, 但中位數幾乎不動。RMS 突然變大時先看是不是漏網, 不要當成精度變差。

還有一件事: 摩擦力測試腳本 (`bringup_pkg my_launch`) 會把車子自旋到 14 rad/s,
遠超過 10 Hz 雷射的 ~8 rad/s 追蹤上限。**那一輪的追丟是物理極限, 不是 bug** ——
下面的「依角速度分組」會把這件事攤開。要乾淨的定位數字請用 teleop 跑
(`car_teleop` 轉向上限 1.2 rad/s)。
"""
import argparse
import os
import sys

import numpy as np

try:
    import pandas as pd
except ImportError:
    sys.exit('需要 pandas: pip install pandas')

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CSV = os.path.join(REPO, 'car_run_data', 'sim_data.csv')
SOURCES = (('lid', 'LiDAR'), ('cam', '相機'), ('imu', 'IMU'),
           ('whl', 'IMU+輪速'), ('fus', '融合 (方法五)'))

# 每一條路線要用**不一樣的**判讀方式。用同一套會得到完全錯誤的結論:
#
#   lid  絕對定位, 失效模式是「鎖到 180 度對稱解」-> sigma 門檻 + 追丟率
#   cam  絕對定位, 但 **yaw 是從速度方向推的** -> 車停著時 yaw 沒有意義,
#        拿「|yaw 誤差|>90 度」當追丟會把停車的幀全部算成追丟 (實測 2026-09-10:
#        整體 25.9%, 但分速度看 >0.5 m/s 時只有 1.7%)。而且它的 sigma 是等速
#        KF 的共變異數, 尺度跟 lid_sigma 差一個數量級 (中位 0.013 vs 0.0017),
#        套 LiDAR 的門檻會把**整段**擋掉 (然後印出「0 漏網」——因為根本沒有幀通過)
#   imu/whl  航位推算, 沒有絕對參考 -> 看漂移率
#   fus  絕對定位, 但它的失效模式是**凍結** (P 被壓垮之後所有量測都被擋掉) ->
#        看「估計走過的路徑長 vs ground truth 走過的距離」
SIGMA_GATE_APPLIES = ('lid',)


def wrap_deg(a):
    return np.degrees(np.arctan2(np.sin(a), np.cos(a)))


def prep(df, pre):
    """加上 <pre>_e (位置誤差, m) 與 <pre>_ey (yaw 誤差, deg)。"""
    df[pre + '_e'] = np.hypot(df[pre + '_x'] - df.car_position_x,
                              df[pre + '_y'] - df.car_position_y)
    df[pre + '_ey'] = wrap_deg(df[pre + '_yaw'] - df.gt_yaw)
    return df


def stats(s, pre):
    e, y = s[pre + '_e'], s[pre + '_ey']
    return (len(s), 100 * e.median(), 100 * np.sqrt((e ** 2).mean()),
            100 * e.quantile(.9), 100 * e.max(),
            y.median(), np.sqrt((y ** 2).mean()), y.abs().max())


def show(tag, s, pre):
    if not len(s):
        print(f'  {tag:<22} (沒有資料)')
        return
    n, med, rms, p90, mx, ym, yr, ymx = stats(s, pre)
    print(f'  {tag:<22} n={n:5d} | 位置 中位: {med:5.1f}, RMS: {rms:6.1f}, '
          f'p90: {p90:5.1f}, max: {mx:6.1f} cm | yaw 中位: {ym:+6.2f}, '
          f'RMS: {yr:6.2f}, max: {ymx:6.1f} deg')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?', default=DEFAULT_CSV)
    ap.add_argument('--sigma', type=float, default=0.0025,
                    help='lid_sigma 過濾門檻 (預設 0.0025)')
    ap.add_argument('--age', type=float, default=0.3, help='_age 上限 (秒)')
    ap.add_argument('--no-filter', action='store_true')
    ap.add_argument('--sweep', action='store_true',
                    help='掃一遍 sigma 門檻, 看漏網幾幀')
    a = ap.parse_args()

    if not os.path.isfile(a.csv):
        sys.exit(f'找不到 {a.csv}')
    df = pd.read_csv(a.csv)
    df['t'] = df.odom_stamp - df.odom_stamp.iloc[0]
    df['w'] = df.car_angular_velocity_z.abs()
    print(f'{a.csv}')
    print(f'  {len(df)} 列, {df.t.max():.1f} s, '
          f'|omega| max {df.w.max():.2f} p99 {df.w.quantile(.99):.2f} rad/s')

    # 舊的 CSV 沒有 whl_* 那幾欄 (car_loc_wheel 是後來加的), 所以要先看欄位
    # 在不在, 不能直接取 —— 不然舊資料一讀就 KeyError。
    have = [(p, n) for p, n in SOURCES if p + '_x' in df.columns]
    missing = [n for p, n in SOURCES if p + '_x' not in df.columns]
    if missing:
        print(f'  CSV 裡沒有這些欄位 (舊格式的資料): {", ".join(missing)}')
    live = [p for p, _ in have if df[p + '_x'].notna().any()]
    off = [n for p, n in have if p not in live]
    if off:
        print(f'  沒開的來源 (整欄 NaN, 略過): {", ".join(off)}')
    if not live:
        sys.exit('四條定位線都沒有資料 —— 是不是只跑了 collect_data_node?')

    for pre, name in SOURCES:
        if pre not in live:
            continue
        prep(df, pre)
        print(f'\n=== {name} ({pre}_*) ===')
        show('原始 (全部)', df, pre)
        if a.no_filter:
            continue
        f = df[df[pre + '_age'] < a.age]
        show(f'age<{a.age}', f, pre)
        if pre in SIGMA_GATE_APPLIES:
            g = f[f[pre + '_sigma'] < a.sigma]
            show(f'+sigma<{a.sigma}', g, pre)
        else:
            print(f'  (sigma 門檻只對 lid_ 有意義, 見 SIGMA_GATE_APPLIES 的說明; '
                  f'這條的 sigma 中位數是 {f[pre + "_sigma"].median():.5f})')

        # 時鐘: 這一條的時戳跟 /odom 在同一個基準上嗎
        if pre + '_stamp' in df and 'odom_stamp' in df:
            off = (f[pre + '_stamp'] - f.odom_stamp).median()
            if abs(off) > 1.0:
                print(f'  !! 時戳比 /odom **偏 {off:+.2f} 秒** —— 那不是延遲, 是'
                      f'**時鐘基準不一樣**。融合會被它整段搞死 (見 '
                      f'src/car_loc_fusion/README.md 的 [H]), 而且拿時戳做內插'
                      f'對齊的分析全部要先扣掉它。')

        if pre in ('imu', 'whl'):
            # 航位推算沒有絕對參考, 所以下面那些「追丟 / sigma 門檻」的概念
            # 對它們不成立: sigma 是單調長大的共變異數, 不是「這一幀不可信」的
            # 指標, 拿它當門檻只是在切「跑了多久」。看它們要看漂移率。
            gt_d = float(np.hypot(np.diff(df.car_position_x),
                                  np.diff(df.car_position_y)).sum())
            print(f'  ground truth 走了 {gt_d:.1f} m -> 漂移率 '
                  f'(最大誤差/距離) {100 * df[pre + "_e"].max() / max(gt_d, 1e-6):.2f}%'
                  f', 結束時 {100 * df[pre + "_e"].iloc[-1] / max(gt_d, 1e-6):.2f}%')
            print('  (航位推算的誤差跟**走過的距離**走, 不跟時間走 —— 下面的')
            print('   sigma 門檻與「追丟」是為 LiDAR 的 180 度對稱解設計的,')
            print('   對這兩條沒有意義, 只是在切「跑了多久」。)')
            continue

        if pre == 'cam':
            # 相機的 yaw 來自**速度方向**, 低於 yaw_min_speed (0.15 m/s) 時節點
            # 是「維持上一個值」—— 那時候拿 yaw 去判追丟量到的是車子停了多久,
            # 不是定位好不好。所以分速度看。
            # dt 可能有 0 (同一個 odom 時戳被記了兩列) —— 直接除會生出 inf/NaN,
            # 然後整個分組統計變成 NaN。
            dt = np.gradient(df.odom_stamp.values)
            dt = np.where(np.abs(dt) < 1e-9, np.nan, dt)
            with np.errstate(invalid='ignore', divide='ignore'):
                spd = np.hypot(np.gradient(df.car_position_x.values) / dt,
                               np.gradient(df.car_position_y.values) / dt)
            f = f.assign(_spd=spd[f.index])
            print('  相機的 yaw 是從**速度方向**推的 -> 依車速分組看 '
                  '(低速時它本來就沒有意義):')
            for lo, hi in ((0.0, 0.15), (0.15, 0.5), (0.5, 1.5), (1.5, 99.0)):
                m = f[(f._spd >= lo) & (f._spd < hi)]
                if len(m) < 20:
                    continue
                print(f'    {lo:4.2f}-{hi:4.2f} m/s  n={len(m):5d} '
                      f'({100 * len(m) / len(f):4.1f}%)  '
                      f'位置中位 {100 * m.cam_e.median():5.2f} cm  '
                      f'|yaw|中位 {m.cam_ey.abs().median():6.1f}°  '
                      f'>90度 {100 * (m.cam_ey.abs() > 90).mean():5.1f}%')
            print('  這條路的可用性由**中斷**決定 (車被柱子擋住 / YOLO 漏偵測),')
            print('  不是由 yaw。位置誤差跟車速成正比的話是 delay 沒填 '
                  '(見 car_loc_camera/config/camera_ground.yaml)。')
            continue

        if pre == 'fus':
            # 融合的失效模式是**凍結**: 某個來源的時鐘偏掉 -> 緩衝區清不掉 ->
            # 整段歷史被重複套用 -> P 被壓垮 -> 之後所有量測都被閘門擋掉。
            # 那時候位置誤差大得很難看, 但真正的證據是「它根本沒有動過」。
            # 有 NaN 的列要跳過 —— sum() 遇到一個 NaN 整個結果就是 NaN
            xy = df[['fus_x', 'fus_y']].dropna()
            path = float(np.hypot(np.diff(xy.fus_x), np.diff(xy.fus_y)).sum())
            gt_d = float(np.hypot(np.diff(df.car_position_x),
                                  np.diff(df.car_position_y)).sum())
            print(f'  估計走過的路徑長 {path:.2f} m vs ground truth {gt_d:.1f} m'
                  f'  (比值 {path / max(gt_d, 1e-6):.3f})')
            if gt_d > 5.0 and path < 0.2 * gt_d:
                print('  !! **估計幾乎沒有動 = 凍結**, 不是「不準」。最常見的原因是')
                print('     某個來源的時鐘偏掉 (看上面的時戳警告), 而且要確認')
                print('     car_loc_fusion 有 rebuild —— 舊版沒有時鐘偵測。')
                print(f'     另一個證據: 濾波器自己報的 sigma 中位數是 '
                      f'{f.fus_sigma.median() * 1000:.2f} mm (< 1 mm 就是它'
                      f'「非常確定」自己停在原地)。')
            continue

        lost = f[pre + '_ey'].abs() > 90
        print(f'  追丟 (|yaw 誤差|>90 度): {100 * lost.mean():.1f}% 的幀 '
              f'({lost.sum()} / {len(f)})')
        miss = ((f[pre + '_sigma'] < a.sigma) & lost).sum()
        if miss:
            print(f'  !! 門檻 {a.sigma} 漏掉 {miss} 幀追丟 —— RMS 會被拉高, '
                  f'把門檻調嚴一點 (--sweep 看要多嚴)')
        else:
            print(f'  門檻 {a.sigma} 把追丟幀全部擋掉了 (0 漏網)')

        print('  依角速度分組 (未過濾):')
        for lo, hi in ((0, 1), (1, 3), (3, 5), (5, 8), (8, 99)):
            s = f[(f.w >= lo) & (f.w < hi)]
            if len(s) < 5:
                continue
            print(f'    |w| {lo:2g}~{hi:<3g} n={len(s):5d}  '
                  f'位置中位: {100 * s[pre + "_e"].median():5.1f} cm  '
                  f'|yaw|中位: {s[pre + "_ey"].abs().median():5.1f} deg  '
                  f'追丟: {100 * (s[pre + "_ey"].abs() > 90).mean():4.1f}%')

        if a.sweep:
            print('  sigma 門檻掃描:')
            for th in (0.010, 0.005, 0.003, 0.0025, 0.002, 0.0015):
                p = f[f[pre + '_sigma'] < th]
                if not len(p):
                    continue
                m = (p[pre + '_ey'].abs() > 90).sum()
                print(f'    <{th:<7.4f} 保留 {100 * len(p) / len(f):3.0f}%  '
                      f'漏網 {m:3d} 幀  yaw RMS '
                      f'{np.sqrt((p[pre + "_ey"] ** 2).mean()):6.2f} deg  '
                      f'位置 RMS {100 * np.sqrt((p[pre + "_e"] ** 2).mean()):5.1f} cm')
    return 0


if __name__ == '__main__':
    sys.exit(main())
