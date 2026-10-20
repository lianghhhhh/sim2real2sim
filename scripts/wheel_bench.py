#!/usr/bin/env python3
"""懸空輪子測試: 同一串輪速指令, 比較模擬與實體車的輪子反應。

車子架空 (輪子離地) 時輪子只受馬達、軸承與自己的慣量影響, 跟地面無關。
所以這個測試把「馬達 / 驅動器的模型對不對」跟「地面摩擦對不對」分開:
前者在這裡比, 後者交給摩擦力測試 (friction_test.launch.py)。

    # 1) 錄 (在 ROS 容器裡; 模擬要先用 scripts/isaac_suspend_car.py 把車架空再按 Play)
    python3 scripts/wheel_bench.py record --sim  --out car_run_data/bench_sim.csv
    python3 scripts/wheel_bench.py record --fb-topic /wheel_feedback --out car_run_data/bench_real.csv

    # 2) 比 (host 上, 只要 numpy / pandas)
    python3 scripts/wheel_bench.py compare car_run_data/bench_sim.csv car_run_data/bench_real.csv
    python3 scripts/wheel_bench.py compare car_run_data/bench_real.csv     # 只看一份

指令序列 (預設約 3.5 分鐘, --quick 約 1.5 分鐘):
  rest        靜止, 量回授的零點與雜訊
  solo_*      一次只轉一個輪子 -> 確認輪子名稱與正負號的對應 (接錯的話後面全部白量)
  step_*      0 -> w -> 0 的階躍, 正反兩個方向 -> 延遲、上升時間、過衝、穩態誤差、空載扭矩
  reverse     +6 直接跳 -6 -> 反轉的反應
  ramp        0 -> +10 -> -10 -> 0 慢慢掃 -> 空載扭矩隨轉速的曲線 (= 馬達損耗)
  low_*       1 / 2 / 3 / 5 rpm -> 低速死區與整數 rpm 量化
  silent      送 2.4 rad/s 之後**停止發指令** -> 實體車 0.3 秒會自己停, 模擬會一直轉

送出去的輪速一律量化成整數 rpm, 跟 control_car_node 與實體車驅動一樣。
時間一律用**收到回授的本機時刻** (t_recv): 實體車的回授時戳可能是另一台電腦的時鐘,
而控制端看到的延遲本來就包含傳輸。
"""
import argparse
import csv
import math
import sys

JOINT_NAMES = ['front_left_joint', 'front_right_joint', 'rear_left_joint', 'rear_right_joint']
SHORT = ['FL', 'FR', 'RL', 'RR']
RPM = 2.0 * math.pi / 60.0          # 1 rpm = 0.1047 rad/s


def snap_rpm(w, step=1.0):
    if step <= 0:
        return float(w)
    q = step * RPM
    return round(float(w) / q) * q


# ───────────────────────────────────────────────────────────── 指令序列
def build_sequence(wmax=10.0, quick=False, timeout_test=True):
    """[(名稱, 秒數, 四輪指令 (rad/s) 或 callable(t)->四輪, 要不要發指令)]"""
    S = []

    def seg(name, dur, cmd, publish=True):
        S.append((name, float(dur), cmd, publish))

    def all4(w):
        return [w, w, w, w]

    seg('rest', 3.0, all4(0.0))
    # 一次一個輪子
    for i, n in enumerate(SHORT):
        c = [0.0] * 4
        c[i] = 2.4
        seg(f'solo_{n}', 1.5, c)
        seg('stop', 1.0, all4(0.0))
    # 階躍
    levels = (2.4, 6.0, wmax) if quick else (1.0, 2.4, 4.2, 6.0, 8.0, wmax)
    hold = 2.0 if quick else 3.0
    for w in levels:
        for sgn in (1.0, -1.0):
            seg(f'step_{sgn * w:+.1f}', hold, all4(sgn * w))
            seg('stop', 2.0, all4(0.0))
    # 反轉
    for rep in range(1 if quick else 2):
        seg('pre_reverse', 2.0, all4(6.0))
        seg('reverse', 2.0, all4(-6.0))
        seg('stop', 2.0, all4(0.0))
    # 慢速掃描
    T = 10.0 if quick else 15.0
    seg('ramp', 4.0 * T,
        lambda t, T=T, w=wmax: all4(w * (t / T if t < T else
                                         (1.0 - 2.0 * (t - T) / (2.0 * T)) if t < 3.0 * T else
                                         -1.0 + (t - 3.0 * T) / T)))
    seg('stop', 2.0, all4(0.0))
    # 低速
    if not quick:
        for rpm in (1, 2, 3, 5):
            seg(f'low_{rpm}rpm', 3.0, all4(rpm * RPM))
        seg('stop', 2.0, all4(0.0))
    # 停止發指令
    if timeout_test:
        seg('pre_silent', 2.0, all4(2.4))
        seg('silent', 2.0, all4(2.4), publish=False)
        seg('stop', 2.0, all4(0.0))
    return S


# ───────────────────────────────────────────────────────────── 錄資料
def record(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState

    seq = build_sequence(args.max_speed, args.quick, not args.no_timeout_test)
    total = sum(s[1] for s in seq)

    class Bench(Node):
        def __init__(self):
            super().__init__('wheel_bench',
                             parameter_overrides=[Parameter('use_sim_time', value=args.sim)])
            self.pub = self.create_publisher(JointState, args.cmd_topic, 10)
            self.create_subscription(JointState, args.fb_topic, self.on_fb, 50)
            self.f = open(args.out, 'w', newline='')
            self.w = csv.writer(self.f)
            self.w.writerow(['t_recv', 'stamp', 'seg', 'seg_name', 'seg_t0', 'published']
                            + [f'cmd_{n}' for n in SHORT] + [f'vel_{n}' for n in SHORT]
                            + [f'eff_{n}' for n in SHORT] + [f'pos_{n}' for n in SHORT])
            self.i = -1
            self.t0 = None
            self.cmd = [0.0] * 4
            self.publishing = True
            self.n_fb = 0
            self.warned = False
            self.done = False
            self.tail = None
            self.create_timer(1.0 / args.rate, self.tick)
            self.get_logger().info(
                f'懸空輪子測試: {len(seq)} 段, 約 {total:.0f} 秒。指令 {args.cmd_topic}, '
                f'回授 {args.fb_topic}, 模擬時間 {args.sim}。**輪子要離地。**')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_fb(self, msg):
            self.n_fb += 1
            idx = []
            names = list(msg.name)
            for n in JOINT_NAMES if not args.fb_names else args.fb_names:
                idx.append(names.index(n) if n in names else None)
            if None in idx:
                if not self.warned:
                    self.warned = True
                    self.get_logger().warn(f'回授裡找不到 {JOINT_NAMES}, 收到的是 {names}; '
                                           '改用前四個 (順序不對的話用 --fb-names)')
                idx = [0, 1, 2, 3]

            def pick(arr):
                return [float(arr[k]) if k < len(arr) else float('nan') for k in idx]
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            name = seq[self.i][0] if 0 <= self.i < len(seq) else ('wait' if self.i < 0 else 'end')
            self.w.writerow([f'{self.now():.6f}', f'{st:.6f}', self.i, name,
                             f'{self.t0:.6f}' if self.t0 is not None else '',
                             int(self.publishing)]
                            + [f'{c:.5f}' for c in self.cmd]
                            + [f'{v:.5f}' for v in pick(msg.velocity)]
                            + [f'{v:.5f}' for v in pick(msg.effort)]
                            + [f'{v:.5f}' for v in pick(msg.position)])

        def send(self, cmd):
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.name = list(JOINT_NAMES)
            m.velocity = [snap_rpm(c, args.rpm_step) for c in cmd]
            self.pub.publish(m)

        def tick(self):
            t = self.now()
            if t <= 0.0:
                return                      # 模擬時鐘還沒來
            if self.done:
                return
            if self.tail is not None:       # 收尾: 送 1 秒的 0
                self.send([0.0] * 4)
                if t - self.tail > 1.0:
                    self.done = True
                return
            if self.i < 0:
                if self.n_fb == 0:
                    if self.t0 is None or t - self.t0 > 2.0:
                        self.get_logger().warn(f'還沒收到 {args.fb_topic}')
                        self.t0 = t
                    return
                self.i, self.t0 = 0, t
                self.get_logger().info('開始')
            name, dur, cmd, pub = seq[self.i]
            if t - self.t0 >= dur:
                self.i += 1
                self.t0 = t
                if self.i >= len(seq):
                    self.f.flush()
                    self.tail = t
                    self.get_logger().info(f'完成, 回授 {self.n_fb} 筆 -> {args.out}')
                    return
                name, dur, cmd, pub = seq[self.i]
                self.get_logger().info(f'[{self.i + 1}/{len(seq)}] {name}')
            c = cmd(t - self.t0) if callable(cmd) else cmd
            self.cmd = [snap_rpm(v, args.rpm_step) for v in c]
            self.publishing = pub
            if pub:
                self.send(c)

    rclpy.init()
    node = Bench()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(5):
                node.send([0.0] * 4)
        except Exception:
            pass
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


# ───────────────────────────────────────────────────────────── 分析
def analyze(path):
    import numpy as np
    import pandas as pd

    d = pd.read_csv(path)
    d = d[d['seg'] >= 0].sort_values('t_recv').reset_index(drop=True)
    R = {'path': path, 'n': len(d)}
    dt = np.diff(d['t_recv'].to_numpy())
    R['rate'] = 1.0 / float(np.median(dt[dt > 0])) if (dt > 0).any() else float('nan')
    lat = (d['t_recv'] - d['stamp']).to_numpy()
    R['latency'] = float(np.median(lat))

    V = d[[f'vel_{n}' for n in SHORT]].to_numpy()
    E = d[[f'eff_{n}' for n in SHORT]].to_numpy()
    C = d[[f'cmd_{n}' for n in SHORT]].to_numpy()
    t = d['t_recv'].to_numpy()
    R['has_effort'] = bool(np.isfinite(E).any())

    def segs(prefix):
        out = []
        for k, g in d.groupby('seg'):
            nm = g['seg_name'].iloc[0]
            if nm == prefix or nm.startswith(prefix):
                out.append((nm, g.index.to_numpy(), float(g['seg_t0'].iloc[0])))
        return out

    # 靜止: 零點與雜訊
    rs = segs('rest')
    if rs:
        ix = rs[0][1]
        R['rest_vel_std'] = float(np.nanstd(V[ix]))
        R['rest_eff_mean'] = np.nanmean(E[ix], axis=0).tolist()
        R['rest_eff_std'] = float(np.nanstd(E[ix]))

    # 單輪: 誰在轉、正負號
    solo = {}
    for nm, ix, _ in segs('solo_'):
        want = nm.split('_')[1]
        half = ix[len(ix) // 2:]
        m = np.nanmean(V[half], axis=0)
        j = int(np.nanargmax(np.abs(m)))
        solo[want] = (SHORT[j], float(m[j]))
    R['solo'] = solo

    # 階躍
    steps = []
    for nm, ix, t0 in segs('step_'):
        w = float(nm.split('_')[1])
        sg = 1.0 if w > 0 else -1.0
        cmdq = float(np.nanmedian(C[ix, 0]))
        tt = t[ix] - t0
        y = V[ix] * sg                                         # 往指令方向為正
        row = {'cmd': cmdq}
        d10, d90, ov, vss, ess, err = [], [], [], [], [], []
        for k in range(4):
            yk = y[:, k]
            a10 = np.nonzero(yk >= 0.1 * abs(cmdq))[0]
            a90 = np.nonzero(yk >= 0.9 * abs(cmdq))[0]
            d10.append(tt[a10[0]] if len(a10) else np.nan)
            d90.append(tt[a90[0]] if len(a90) else np.nan)
            tail = tt >= 0.6 * tt[-1]
            vss.append(float(np.nanmean(yk[tail])))
            ess.append(float(np.nanmean(E[ix][tail, k] * sg)))
            ov.append(float(np.nanmax(yk) / abs(cmdq) - 1.0))
            err.append(vss[-1] / abs(cmdq) - 1.0)
        row.update(t10=float(np.nanmean(d10)), t90=float(np.nanmean(d90)),
                   overshoot=float(np.nanmean(ov)), err=float(np.nanmean(err)),
                   v_ss=vss, eff_ss=ess)
        steps.append(row)
    R['steps'] = steps

    # 停車 (階躍之後那一段 stop): 從指令歸零到 |v| < 10%
    stops = []
    names = d.groupby('seg')['seg_name'].first()
    for k in names.index:
        if names[k] == 'stop' and k - 1 in names.index and str(names[k - 1]).startswith('step_'):
            g = d[d['seg'] == k]
            w0 = abs(float(names[k - 1].split('_')[1]))
            tt = g['t_recv'].to_numpy() - float(g['seg_t0'].iloc[0])
            vv = np.nanmax(np.abs(g[[f'vel_{n}' for n in SHORT]].to_numpy()), axis=1)
            a = np.nonzero(vv < 0.1 * w0)[0]
            stops.append((w0, float(tt[a[0]]) if len(a) else float('nan')))
    R['stops'] = stops

    # 反轉: 從 +6 跳到 -6, 到達 -5.4 要多久
    rev = []
    for nm, ix, t0 in segs('reverse'):
        tt = t[ix] - t0
        y = -V[ix]
        a = np.nonzero(np.nanmin(y, axis=1) >= 0.9 * 6.0)[0]
        rev.append(float(tt[a[0]]) if len(a) else float('nan'))
    R['reverse_t90'] = float(np.nanmean(rev)) if rev else float('nan')

    # 慢速掃描: 空載扭矩 = c0 + c1·|w| (沿轉動方向)
    rp = segs('ramp')
    if rp and R['has_effort']:
        ix = rp[0][1]
        v, e = V[ix].ravel(), E[ix].ravel()
        ok = np.isfinite(v) & np.isfinite(e) & (np.abs(v) > 0.5)
        if ok.sum() > 20:
            A = np.c_[np.ones(ok.sum()), np.abs(v[ok])]
            c, *_ = np.linalg.lstsq(A, e[ok] * np.sign(v[ok]), rcond=None)
            R['loss'] = (float(c[0]), float(c[1]))
            vv, cc = V[ix], C[ix]
            okt = np.abs(cc) > 1.0
            R['ramp_track'] = float(np.nanmedian(np.abs(vv[okt] - cc[okt]) / np.abs(cc[okt])))
    # 低速
    low = []
    for nm, ix, _ in segs('low_'):
        cmdq = float(np.nanmedian(C[ix, 0]))
        half = ix[len(ix) // 2:]
        low.append((nm[4:], cmdq, float(np.nanmean(V[half])) / cmdq))
    R['low'] = low
    # 停止發指令之後
    sl = segs('silent')
    if sl:
        ix, t0 = sl[0][1], sl[0][2]
        tt = t[ix] - t0
        vv = np.nanmax(np.abs(V[ix]), axis=1)
        a = np.nonzero(vv < 0.24)[0]
        R['silent_stop'] = float(tt[a[0]]) if len(a) else float('inf')
    return R


def fmt(x, unit='', nd=3):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return '—' if x is None or math.isnan(x) else '不會停'
    return f'{x:.{nd}f}{unit}'


def compare(args):
    import numpy as np
    Rs = [analyze(p) for p in args.files]
    two = len(Rs) == 2
    tag = ['A', 'B'] if two else ['結果']
    print('═' * 96)
    for k, R in enumerate(Rs):
        print(f'{tag[k]}: {R["path"]}  ({R["n"]} 筆, 回授 {R["rate"]:.0f} Hz)')
    print('═' * 96)
    flags = []

    def line(label, vals, unit='', nd=3, tol=None, why=''):
        s = f'  {label:<34s}' + ''.join(f'{fmt(v, unit, nd):>14s}' for v in vals)
        if two and tol is not None and all(isinstance(v, float) and math.isfinite(v) for v in vals):
            dif = vals[1] - vals[0]
            bad = abs(dif) > tol
            s += f'   差 {dif:+.{nd}f}{unit}' + ('  <- 不一致' if bad else '')
            if bad:
                flags.append((label, why))
        print(s)

    # 1. 接線
    print('[1] 輪子名稱與正負號 (一次只轉一個輪子, 指令 +2.4 rad/s)')
    for k, R in enumerate(Rs):
        bad = [f'{w}->{m[0]}({m[1]:+.1f})' for w, m in R['solo'].items()
               if m[0] != w or m[1] < 0]
        ok = not bad and len(R['solo']) == 4
        print(f'  {tag[k]}: ' + ('OK, 四個輪子名稱與方向都對' if ok else
                                 '**不對**: ' + ', '.join(bad) + ' (指令給哪個輪 -> 實際轉的輪)'))
        if not ok:
            flags.append(('輪子對應', '名稱或正負號接錯, 摩擦力測試的自旋方向與扭矩符號都會錯'))
    # 2. 回授
    print('\n[2] 回授')
    line('回授頻率 [Hz]', [R['rate'] for R in Rs], nd=0)
    line('靜止時輪速雜訊 [rad/s]', [R.get('rest_vel_std', float('nan')) for R in Rs])
    line('靜止時扭矩零點 (四輪平均) [N·m]',
         [float(sum(R.get('rest_eff_mean', [math.nan]))) / 4 for R in Rs], tol=0.05,
         why='扭矩回授有零點偏移; 摩擦分析扣損耗時會一起扣掉, 但要確認不是單位問題')
    for k, R in enumerate(Rs):
        if not R['has_effort']:
            print(f'  {tag[k]}: **沒有扭矩回授** (effort 全是空的) —— 摩擦力測試的主特徵就是它, 一定要有')
            flags.append(('沒有扭矩', '回授沒有 effort'))
    # 3. 階躍
    print('\n[3] 階躍 (四輪平均; 時間從送出新指令算起)')
    S = [{round(s['cmd'], 2): s for s in R['steps']} for R in Rs]
    cmds = sorted(set().union(*[set(x) for x in S]), key=lambda c: (abs(c), c))
    for c in cmds:
        row = [x.get(c) for x in S]
        lab = f'{c:+.2f} rad/s'
        line(f'{lab} 到 10% [s]', [r['t10'] if r else math.nan for r in row], tol=0.05,
             why='反應延遲不同: 影響起步 (B4) 與歸位, 不影響自旋扭矩')
        line(f'{lab} 到 90% [s]', [r['t90'] if r else math.nan for r in row], tol=0.1,
             why='加速快慢不同: 驅動器有加速度限制或速度環比較軟')
        line(f'{lab} 過衝 [%]', [r['overshoot'] * 100 if r else math.nan for r in row], nd=1, tol=5.0,
             why='速度環的阻尼不同')
        line(f'{lab} 穩態誤差 [%]', [r['err'] * 100 if r else math.nan for r in row], nd=2, tol=2.0,
             why='輪速追不上指令: 自旋時的輪速就跟模擬不同')
        line(f'{lab} 空載扭矩 [N·m]', [sum(r['eff_ss']) / 4 if r else math.nan for r in row], tol=0.05,
             why='馬達空轉損耗; 摩擦分析靠 B3 扣掉, 但兩邊差太多表示模擬沒有這一項')
    print('  空載扭矩逐輪 (最高那一檔):')
    for k, R in enumerate(Rs):
        if R['steps']:
            top = max(R['steps'], key=lambda s: abs(s['cmd']))
            print(f'    {tag[k]} {top["cmd"]:+.2f}: ' + '  '.join(f'{n} {v:+.3f}' for n, v in zip(SHORT, top['eff_ss'])))
    # 4. 停車、反轉
    print('\n[4] 停車與反轉')
    for w in sorted({round(s[0], 2) for R in Rs for s in R['stops']}):
        vals = [float(np.mean([x[1] for x in R['stops'] if round(x[0], 2) == w]))
                if any(round(x[0], 2) == w for x in R['stops']) else math.nan for R in Rs]
        line(f'從 {w:.2f} rad/s 指令歸零到停 [s]', vals, tol=0.1,
             why='煞車快慢不同: 影響急煞 (B3) 與每段之間的停車')
    line('+6 -> -6 到 90% [s]', [R['reverse_t90'] for R in Rs], tol=0.1, why='反轉快慢不同')
    # 5. 慢速掃描
    print('\n[5] 慢速掃描: 空載扭矩 = c0 + c1·|w|  (這就是馬達損耗, 摩擦分析從自旋扭矩扣掉的那一項)')
    line('c0 [N·m]', [R.get('loss', (math.nan,) * 2)[0] for R in Rs], tol=0.05,
         why='馬達損耗跟模擬不同 (模擬是 0)')
    line('c1 [N·m/(rad/s)]', [R.get('loss', (math.nan,) * 2)[1] for R in Rs], nd=4, tol=0.01,
         why='損耗隨轉速增加的斜率不同')
    line('掃描時輪速誤差 (中位) [%]', [R.get('ramp_track', math.nan) * 100 for R in Rs], nd=2, tol=2.0,
         why='慢慢變化的指令也追不上')
    # 6. 低速
    print('\n[6] 低速 (實際 / 指令)')
    lows = sorted({x[0] for R in Rs for x in R['low']}, key=lambda s: int(s[:-3]))
    for nm in lows:
        line(nm, [next((x[2] for x in R['low'] if x[0] == nm), math.nan) for R in Rs], nd=2, tol=0.2,
             why='低速死區或量化不同')
    # 7. 停止發指令
    print('\n[7] 停止發指令之後多久停 (實體車預期約 0.3 s; 模擬會一直轉)')
    print('  ' + ' ' * 34 + ''.join(f'{fmt(R.get("silent_stop"), " s", 2):>14s}' for R in Rs))
    print('  (這一項本來就不同, 不算不一致: control_car_node 會持續發指令, 結束時補送 0)')

    print('\n' + '═' * 96)
    if two:
        if not flags:
            print('結論: 兩邊的輪子反應一致 (在上面各項的容許範圍內)。')
        else:
            print(f'結論: {len(flags)} 項不一致 (同一個原因只列第一項):')
            seen = set()
            for lab, why in flags:
                if why in seen:
                    continue
                seen.add(why)
                print(f'  * {lab}: {why}')
            print('\n對摩擦力測試的影響, 由大到小:')
            print('  1. 輪子對應 / 沒有扭矩回授 -> 測試不能跑')
            print('  2. 穩態誤差 -> 自旋時實際輪速不同; 分析用實際輪速查馬達上限, 但兩邊轉速不同就不是同一個實驗')
            print('  3. 空載扭矩 / c0 c1 -> 由 B3 損耗修正處理; 差很多時考慮在 USD 的輪子關節加 jointFriction')
            print('  4. 延遲 / 加速 / 停車 -> 只影響 B4 起步、急煞與歸位, 不影響主特徵')
    print('═' * 96)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record', help='送指令並錄回授 (要 ROS)')
    r.add_argument('--out', required=True)
    r.add_argument('--sim', action='store_true', help='用模擬時間 (Isaac)')
    r.add_argument('--cmd-topic', default='/joint_command')
    r.add_argument('--fb-topic', default='/joint_states', help='實體車: /wheel_feedback')
    r.add_argument('--fb-names', nargs=4, default=None,
                   help='回授裡四個輪子的名稱 (FL FR RL RR 的順序), 跟指令的名稱不同時才要給')
    r.add_argument('--rate', type=float, default=50.0, help='發指令的頻率 Hz')
    r.add_argument('--max-speed', type=float, default=10.0,
                   help='最高輪速 rad/s (實體車 max_rpm 100 = 10.47)')
    r.add_argument('--rpm-step', type=float, default=1.0, help='指令量化 (rpm), 0 = 不量化')
    r.add_argument('--quick', action='store_true', help='短版, 約 1.5 分鐘')
    r.add_argument('--no-timeout-test', action='store_true', help='不測「停止發指令」')
    c = sub.add_parser('compare', help='分析一份, 或比較兩份 (第一份當參考)')
    c.add_argument('files', nargs='+')
    args = ap.parse_args()
    if args.cmd == 'record':
        record(args)
    else:
        if len(args.files) > 2:
            sys.exit('compare 最多兩份')
        compare(args)


if __name__ == '__main__':
    main()
