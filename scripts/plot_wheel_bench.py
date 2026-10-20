#!/usr/bin/env python3
"""把 wheel_bench.py 錄的懸空輪子測試畫成對照圖 (實體車 vs 一到兩份模擬)。

    python3 scripts/plot_wheel_bench.py car_run_data/bench_real.csv car_run_data/bench_sim.csv
    python3 scripts/plot_wheel_bench.py car_run_data/bench_real.csv \
        car_run_data/bench_sim_ideal.csv car_run_data/bench_sim.csv \
        --labels 實體車 '模擬 (原本)' '模擬 + 致動器模型'

輸出預設是最後一個檔案旁邊的 <檔名>_compare.png (上例: car_run_data/bench_sim_compare.png)。

第一個檔案是基準 (通常是實體車), 最多再兩份。六張圖: 起步 0->10、反轉 +6->-6、
停車 10->0、每種階躍到 90% 的時間、停車降到 10% 的時間、停止發指令後的逾時。
只要 numpy / pandas / matplotlib。

**時間軸**: 模擬的資料用 Isaac 那一幀自己的時間 (stamp), 不用 wheel_bench 收到
的時間 (t_recv)。模擬時 ROS 時鐘跟著 /clock 每 1/60 s 才跳一次, 兩次之間收到的
回授會拿到同一個 t_recv, 畫出來像鋸齒 (2026-10-20 的資料約 1/4 的列重複)。stamp
跟 wheel_bench 的時鐘是同一個來源, 只差一個固定的落後, 用中位數扣掉就能跟送指令
的時間 (seg_t0) 對齊。實體車的 stamp 是另一台電腦的時鐘, 所以照舊用 t_recv
(牆上時鐘, 本來就連續)。判斷方式: t_recv - stamp 的中位數小於 1 秒 = 同一個時鐘。
"""
import argparse
import glob
import os

import numpy as np
import pandas as pd

W = ['FL', 'FR', 'RL', 'RR']
# 第一份 (基準) 藍色實線, 其他依序。三色都過 validate_palette (all-pairs)。
COLORS = ['#2a78d6', '#eb6834', '#1baf7a']
STYLES = ['-', '-', (0, (4, 2))]
CMD, INK, MUTED, GRID, BG = '#8a8984', '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
SIZES = [1.0, 2.4, 4.2, 6.0, 8.0, 10.0]


def setup_font(plt):
    """找一個有中文字的字型。host 有 Noto Sans CJK; ros2_node 容器裡只有
    Droid Sans Fallback。都沒有的話中文會變成方塊, 印警告。"""
    from matplotlib import font_manager as fm
    paths = glob.glob('/usr/share/fonts/**/NotoSansCJK*.tt[cf]', recursive=True)
    paths += glob.glob('/usr/share/fonts/**/DroidSansFallback*.ttf', recursive=True)
    for p in paths:
        try:
            fm.fontManager.addfont(p)
        except Exception:
            pass
    names = {f.name for f in fm.fontManager.ttflist}
    plt.rcParams['axes.unicode_minus'] = False
    for cand in ('Noto Sans CJK TC', 'Noto Sans CJK JP', 'Noto Sans CJK SC',
                 'WenQuanYi Zen Hei', 'Droid Sans Fallback'):
        if cand in names:
            # 字型清單: 第一個缺的字 (Droid 沒有英文數字) 由下一個補
            plt.rcParams['font.family'] = ([cand, 'DejaVu Sans'] if cand.startswith('Noto')
                                           else ['DejaVu Sans', cand])
            return cand
    print('警告: 找不到中文字型, 圖上的中文會變成方塊 (apt install fonts-noto-cjk)')
    return None


def load(path):
    """讀 CSV, 加上 t (對齊 seg_t0 的時間)、v / c (四輪平均的輪速 / 指令)。"""
    d = pd.read_csv(path)
    d = d[d.seg >= 0].copy()
    off = float((d.t_recv - d.stamp).median())
    if abs(off) < 1.0:
        d['t'] = d.stamp + off          # 模擬: 同一個時鐘, 用 stamp
        src = f'stamp (已扣掉 ROS 時鐘比 Isaac 慢的 {-off * 1000:.0f} ms; 這不是延遲)'
    else:
        d['t'] = d.t_recv               # 實體車: 不同時鐘, 用收到的時間
        src = 't_recv'
    d = d.sort_values('t').reset_index(drop=True)
    d['v'] = d[['vel_' + w for w in W]].mean(axis=1)
    d['c'] = d[['cmd_' + w for w in W]].mean(axis=1)
    print(f'{path}: {len(d)} 筆, 時間軸 = {src}')
    return d


def window(d, seg, prev, pre=0.1, post=0.7):
    """seg 段 (緊接在 prev 段之後的那一次) 開始前後的資料, 時間從送出指令算起。"""
    m = d.seg_name == seg
    if prev is not None:
        m &= d.seg_name.shift() == prev
    if not m.any():
        return None
    i = d.index[m][0]
    t0 = d.seg_t0[i]
    w = d[(d.t >= t0 - pre) & (d.t < t0 + post)]
    return w.t.values - t0, w, i


def first_time(d, seg, prev, cond, post=1.5):
    r = window(d, seg, prev, pre=0.0, post=post)
    if r is None:
        return np.nan
    t, w, i = r
    k = np.nonzero(cond(w, d, i))[0]
    return t[k[0]] if len(k) else np.nan


def t_rise90(d, s):
    vals = [first_time(d, f'step_{sg}{s}', 'stop',
                       lambda w, d, i: w.v.values * np.sign(w.c.values) >= 0.9 * np.abs(w.c.values))
            for sg in '+-']
    return np.nanmean(vals)


def t_stop10(d, s):
    vals = [first_time(d, 'stop', f'step_{sg}{s}',
                       lambda w, d, i: np.abs(w.v.values) < 0.1 * abs(d.v[i - 1]))
            for sg in '+-']
    return np.nanmean(vals)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('files', nargs='+', help='第一個是基準 (實體車), 最多三個')
    ap.add_argument('--labels', nargs='+', default=None)
    ap.add_argument('--out', default=None,
                    help='預設 = 最後一個檔案同目錄的 <檔名>_compare.png')
    ap.add_argument('--title', default=None)
    a = ap.parse_args()
    if len(a.files) > 3:
        ap.error('最多三個檔案 (三條線以上顏色分不清楚)')
    labels = a.labels or (['實體車'] + [f'模擬 {i}' for i in range(1, len(a.files))])
    if len(labels) != len(a.files):
        ap.error('--labels 的數量要跟檔案一樣')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    setup_font(plt)
    if a.out is None:
        a.out = os.path.splitext(a.files[-1])[0] + '_compare.png'

    data = [load(f) for f in a.files]
    # 顏色跟著角色走: 基準藍色, 最後一份 (通常是最新的模擬) 一律綠色虛線,
    # 三份時中間那份橘色。基準 (實體車) 最後畫, 壓在最上面
    pick = [0] + list(range(3 - (len(data) - 1), 3))
    series = [(d, COLORS[k], STYLES[k], lab) for d, k, lab in zip(data, pick, labels)]
    draw = series[1:] + series[:1]

    def style(ax, title, xl='送出新指令後的時間 [s]', yl='輪速 (四輪平均) [rad/s]'):
        ax.set_facecolor(BG)
        ax.set_title(title, loc='left', fontsize=11.5, color=INK, pad=8)
        ax.set_xlabel(xl, color=MUTED, fontsize=9)
        ax.set_ylabel(yl, color=MUTED, fontsize=9)
        ax.grid(True, color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        for s in ('left', 'bottom'):
            ax.spines[s].set_color('#c9c8c2')
        ax.tick_params(colors=MUTED, labelsize=8.5)

    def response(ax, seg, prev, title):
        r = window(data[0], seg, prev)
        if r is not None:
            # 指令畫成 t = 0 的理想階躍 (送出的那一刻), 不用資料列的時間
            t, w, i = r
            c0, c1 = data[0].c[i - 1], data[0].c[i]
            ax.plot([t[0], 0, 0, t[-1]], [c0, c0, c1, c1], color=CMD, lw=1.2, label='指令')
        for d, col, ls, lab in draw:
            r = window(d, seg, prev)
            if r is not None:
                ax.plot(r[0], r[1].v, color=col, lw=2, ls=ls, label=lab)
        style(ax, title)

    def per_size(ax, fn, title, xl):
        ends = []
        for d, col, ls, lab in draw:
            y = [fn(d, s) for s in SIZES]
            ax.plot(SIZES, y, color=col, lw=2, ls=ls, marker='o', ms=8, mec=BG, mew=2)
            ends.append((y[-1], lab))
        # 線尾直接標名字; 只有兩條線的尾巴靠太近 (< 0.03 s) 才上下錯開
        ends = sorted([e for e in ends if np.isfinite(e[0])])
        offs = [0.0] * len(ends)
        for k in range(1, len(ends)):
            if ends[k][0] - ends[k - 1][0] < 0.03:
                offs[k - 1] -= 7
                offs[k] += 7
        for (y, lab), off in zip(ends, offs):
            ax.annotate(lab, (SIZES[-1], y), xytext=(10, off), textcoords='offset points',
                        color=INK, fontsize=8.5, va='center')
        ax.set_xlim(0.5, 13.5)
        ax.set_ylim(0, 0.5)
        style(ax, title, xl=xl, yl='時間 [s]')

    fig, axs = plt.subplots(2, 3, figsize=(17, 9.4), facecolor=BG)
    response(axs[0, 0], 'step_+10.0', 'stop', '① 起步 0 → 10 rad/s')
    axs[0, 0].legend(loc='lower right', fontsize=9, frameon=False)
    response(axs[0, 1], 'reverse', 'pre_reverse', '② 反轉 +6 → −6 rad/s')
    response(axs[0, 2], 'stop', 'step_+10.0', '③ 停車 10 → 0 rad/s')
    per_size(axs[1, 0], t_rise90, '④ 起步: 到 90% 的時間 (正反平均)', '階躍大小 [rad/s]')
    per_size(axs[1, 1], t_stop10, '⑤ 停車: 降到 10% 的時間 (正反平均)', '起始輪速 [rad/s]')

    ax = axs[1, 2]
    for d, col, ls, lab in draw:
        s = d[d.seg_name == 'silent']
        if s.empty:
            continue
        t0 = d.seg_t0[s.index[0]]
        w = d[(d.t >= t0 - 0.3) & (d.t < t0 + 1.2)]
        ax.plot(w.t - t0, w.v, color=col, lw=2, ls=ls, label=lab)
    ax.axvline(0, color=CMD, lw=1.2)
    ax.text(0.02, 0.15, '最後一筆指令\n(之後不再發)', color=MUTED, fontsize=8.5)
    style(ax, '⑥ 停止發指令之後', xl='最後一筆指令後的時間 [s]')

    fig.suptitle(a.title or '懸空輪子測試: ' + ' vs '.join(labels), x=0.01, ha='left',
                 fontsize=14, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96), h_pad=2.5)
    fig.savefig(a.out, dpi=130, facecolor=BG)
    print(f'-> {a.out}')

    # 非 0 -> 非 0 (wheel_bench 的 chg_* 段): 有的話另外畫一張, 每個轉換一格
    chg = []
    for d in data:
        for nm in d.seg_name.unique():
            if str(nm).startswith('chg_') and nm not in chg:
                chg.append(nm)
    if not chg:
        return
    pos = [n for n in chg if not n.split('_')[1].startswith('-')]
    neg = [n for n in chg if n.split('_')[1].startswith('-')]
    ncol = max(len(pos), len(neg))
    fig, axs = plt.subplots(2 if neg else 1, ncol, figsize=(3.4 * ncol, 4.4 * (2 if neg else 1)),
                            facecolor=BG, squeeze=False)
    for row, names in enumerate([pos, neg][:axs.shape[0]]):
        for col in range(ncol):
            ax = axs[row, col]
            if col >= len(names):
                ax.axis('off')
                continue
            nm = names[col]
            _, w0, w1 = nm.split('_')
            ax.plot([-0.1, 0, 0, 0.7], [float(w0), float(w0), float(w1), float(w1)],
                    color=CMD, lw=1.2, label='指令')
            for d, colr, ls, lab in draw:
                r = window(d, nm, None)
                if r is not None:
                    ax.plot(r[0], r[1].v, color=colr, lw=2, ls=ls, label=lab)
            style(ax, f'{float(w0):+g} → {float(w1):+g} rad/s', yl='輪速 [rad/s]' if col == 0 else '')
    axs[0, 0].legend(loc='best', fontsize=8.5, frameon=False)
    fig.suptitle('非 0 → 非 0: 不經過 0 直接換轉速 (加速 / 減速是不是同一個斜坡)', x=0.01,
                 ha='left', fontsize=14, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out2 = os.path.splitext(a.out)[0] + '_chg.png'
    fig.savefig(out2, dpi=130, facecolor=BG)
    print(f'-> {out2}')


if __name__ == '__main__':
    main()
