#!/usr/bin/env python3
"""摩擦力自動校正的「一步」: 比對這一輪 sim 與 real, 決定下一輪 sim 的地面摩擦。

run_car.sh 每量完一輪 sim 就呼叫一次:

    ./scripts/friction_calib_step.py --real real.csv --sim sim_iter2.csv \\
        --static 1.80 --dynamic 1.80 --history history.json --iter 2

最後一行輸出給 bash `eval`:

    STATUS=continue NEXT_STATIC=1.9213 NEXT_DYNAMIC=1.9213

STATUS:
  continue   還差超過 --tol, 用 NEXT_* 再量一輪
  converged  預測的修正量 <= --tol (靜 / 動都是) -> 停, NEXT_* = 最後的估計值
  retry      這一輪資料不能用 (定位壞掉 / 主特徵沒資料) -> 同樣的值重量一次
  failed     換算不出來或重試次數用完
  max_iter   次數用完還沒收斂 -> NEXT_* = 歷史上量起來最接近 real 的那一輪

────────────────────────────────────────────────────────────────────────
更新規則
────────────────────────────────────────────────────────────────────────
每個係數各自用一個特徵 (量出來的 real 值是目標):
    dynamicFriction <- B1 自旋減速度 (spin_decel)
    staticFriction  <- B2 慢速起轉 effort (creep_breakaway) —— 預設不單獨解, 見下面

────────────────────────────────────────────────────────────────────────
為什麼預設把 staticFriction 綁在 dynamicFriction 上 (--static-mode tied)
────────────────────────────────────────────────────────────────────────
2026-09-16 的對照實驗 (probe: 地面 static 1.2 / dynamic 0.5, 對照 real 0.5 / 0.5,
**只有靜摩擦不一樣**) 量到各特徵對靜摩擦的敏感度 d ln f / d ln mu_static:

    creep_breakaway    0.315   (9.4 sigma, 真的有反應)
    spinup_breakaway   0.208   (4.6 sigma)
    spinup_time       -0.019   (0.7 sigma = 沒反應)
    spin_decel        -0.015   (0.7 sigma = 沒反應, 符合預期)

也就是說 PhysX 這組設定下**靜摩擦只有 B2 量得到, 而且敏感度只有 0.3**
(推測是起轉前的預滑移已經讓接觸點進入滑動狀態, 之後就由 dynamic 主導)。
同一個特徵對動摩擦的敏感度是 ~0.65, 所以:

  * 拿 creep 解靜摩擦, 必須先扣掉動摩擦的貢獻, 而動摩擦本身的誤差會被放大
    0.65/0.315 ≈ 2 倍灌進靜摩擦 -> 靜摩擦的解析度大約只有 ±0.07。
  * 硬要分開解, 兩邊的量測雜訊會被塞進「靜/動的差」這個自由度裡, 產生假的
    靜動差 (2026-09-16 那一輪: 真值 0.5/0.5, 解出 0.537/0.486)。

所以預設 `--static-mode tied`: 只解一個 mu (用 spin_decel), static = dynamic。
real 的 USD 兩個值一樣時這就是正解; 真的需要分開解再用 `--static-mode free`。

* 只有一輪 sim (第 0 輪):  用 estimate_friction 的物理換算 (有效摩擦取平均、特徵 ∝ 有效
  摩擦) 估 real 的地面值。
* 兩輪以上:  **割線法**, 直接在「sim 地面係數 -> 量到的特徵」上內插 / 外插 real 的特徵值,
  取最靠近 real 的兩個點 (real 被夾在中間的那一對優先)。不需要任何物理模型。

  為什麼不一直用物理換算: 特徵對 mu 不是線性的。2026-09-15 那一輪:
      地面 0.5 -> B1 25.4    1.5 -> 49.5    real (2.0) -> 58.6
  每單位有效 mu 從 51 掉到 47, 按比例換算會把 real 估成 ~1.7, 然後在 1.5 附近就
  「量不出差別」停下來。割線法用的是實際量到的曲線, 越接近 real 越準。

新值 = 目前 + gain x (估計 - 目前), 單步最多 max_step 倍, 夾在 [mu_min, mu_max]。
下一組值跟之前量過的某一組幾乎一樣 (在兩點之間來回跳) 時步長減半。

**收斂 = |估計 - 目前| <= tol**, 不是「統計上分不出差別」—— 後者在雜訊大的時候
會提早停 (上一版在 1.5 vs 2.0 就停了)。解析度 (估計值的 1-sigma) 比 tol 差時會警告:
那表示 trial 數不夠, 收斂判定會被雜訊左右。
"""
import argparse
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# 係數 -> 依序嘗試的特徵
# spinup_time 原本被當成靜摩擦特徵, 2026-09-16 的對照實驗證明它對靜摩擦沒反應
# (敏感度 -0.019), 對動摩擦才有 (~0.47) —— 它是 spin_decel 的備援, 不是靜摩擦。
TERM_FEATURES = {
    'dynamic': ['spin_decel', 'spinup_time'],
    'static': ['creep_breakaway', 'spinup_breakaway'],
}
# spinup_time 依賴「兩個環境的馬達一樣」(它含馬達 ramp)。B3 對照組 (直線加速 /
# 直線滑行) 就是在量這件事: 它們跟地面摩擦無關, 比值應該 ≈ 1。顯著不是 1 就不能用。
MOTOR_DEPENDENT = {'spinup_time'}
CONTROL_FEATURES = ('sprint_accel', 'coast_lin_decel')
# tied 模式 (static = dynamic) 下這些特徵量的是**同一個 mu**, 可以各自估一次再
# 反變異數合併。2026-09-16 (sim 0.5 -> real 2.0) 的收斂點四個特徵都說「sim 還低」,
# 但只用 spin_decel 停在 1.925; 合併後是 1.969 (真值 2.0)。
COMBINE_FEATURES = ('spin_decel', 'creep_breakaway', 'spinup_time', 'spinup_breakaway')


def run_estimate(args, json_path):
    cmd = [sys.executable, os.path.join(HERE, 'estimate_friction.py'), args.sim, args.real,
           '--ref-ground-static', str(args.static), '--ref-ground-dynamic', str(args.dynamic),
           '--wheel-mu', str(args.wheel_mu), '--combine', args.combine,
           '--boot', str(args.boot), '--json', json_path]
    if args.source:
        cmd += ['--source', args.source]
    print('$ ' + ' '.join(cmd), flush=True)
    r = subprocess.run(cmd)
    if r.returncode != 0 or not os.path.exists(json_path):
        return None
    with open(json_path) as f:
        return json.load(f)['results'][args.real]


def load_history(path):
    if path and os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return {'iterations': []}


def save_history(path, hist):
    if not path:
        return
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(hist, f, indent=2, ensure_ascii=False, default=float)
    os.replace(tmp, path)


def emit(status, static, dynamic, reason):
    print('─' * 78)
    print(f'[calib] {status}: {reason}')
    print(f'STATUS={status} NEXT_STATIC={static:.4f} NEXT_DYNAMIC={dynamic:.4f}')


def drivetrain_ok(res):
    """B3 對照組有沒有顯著偏離 1 (= 兩邊不只地面不一樣, 驅動系統也不一樣)。

    看的是 90% CI 含不含 1, 不是點估計: 對照組的 trial 少、CI 寬, 用點估計
    會常常誤判成「不一樣」而把最準的特徵丟掉。"""
    for k in CONTROL_FEATURES:
        f = (res.get('features') or {}).get(k)
        if not f:
            continue
        lo, hi = (f.get('ci90') or [float('nan')] * 2)[:2]
        if all(map(math.isfinite, (lo, hi))) and not (lo <= 1.0 <= hi):
            return False
    return True


def pick_features(res):
    """這一輪每個係數用的特徵: {term: {name, sim, real, log_sd}}。"""
    out = {}
    feats = res.get('features') or {}
    motor_ok = drivetrain_ok(res)
    for term, names in TERM_FEATURES.items():
        for n in names:
            if n in MOTOR_DEPENDENT and not motor_ok:
                continue
            f = feats.get(n)
            if f and f.get('ref') and f.get('target'):
                out[term] = {'name': n, 'sim': f['ref'], 'real': f['target'],
                             'log_se': f.get('log_se', f.get('log_sd', 0.1))}
                break
    return out


def hist_points(hist, name, term):
    """歷史上這個特徵的 (sim 地面係數, sim 量到的特徵值)。同一個係數量多次取最後一次。"""
    pts = {}
    for it in hist['iterations']:
        if it.get('status') == 'retry':
            continue
        f = (it.get('all_features') or {}).get(name)
        if f is None:                                  # 舊版 history 只存了選中的那個
            g = (it.get('features') or {}).get(term)
            f = g if (g and g.get('name') == name) else None
        if f:
            pts[round(it[term], 6)] = f['sim']
    return pts


def secant_estimate(term, feat, cur_mu, hist):
    """用歷史上同一個特徵的 (sim 係數, sim 特徵) 點內插 real 的特徵值。

    回傳 (估計值, 1-sigma, 說明) 或 None (點不夠 / 斜率不合理)。"""
    pts = hist_points(hist, feat['name'], term)
    pts[round(cur_mu, 6)] = feat['sim']
    if len(pts) < 2:
        return None
    real = feat['real']
    items = sorted(pts.items())
    # 兩點的**特徵差**至少要有這麼多個 sigma, 斜率才不是在量雜訊。收斂到後期時
    # 相鄰兩輪的 mu 只差 0.07, 特徵差 (0.01) 比雜訊 (0.03) 還小 —— 這時候用它們
    # 算斜率會炸掉 (2026-09-15 的資料第 3 輪: 估計值從 1.96 跳到 2.38 ± 0.22)。
    # 差不夠就退回更遠的那個點: 基線長, 斜率才可信。
    noise = real * feat['log_se']
    MIN_GAP = 3.0
    p1 = min(items, key=lambda p: abs(p[1] - real))       # 特徵最接近 real 的點
    rest = [p for p in items if p[0] != p1[0] and abs(p[0] - p1[0]) > 1e-6]
    if not rest:
        return None
    far = [p for p in rest if abs(p[1] - p1[1]) >= MIN_GAP * noise]
    if far:
        brack = [p for p in far if (p[1] - real) * (p1[1] - real) < 0]
        # real 被夾在中間 -> 內插 (最可信); 否則取基線最短的那個外插
        p2 = min(brack or far, key=lambda p: abs(p[0] - p1[0]))
        how = '內插' if brack else '外插'
    else:
        p2 = max(rest, key=lambda p: abs(p[1] - p1[1]))   # 全都太近 -> 用差最大的
        if abs(p2[1] - p1[1]) < noise:
            return None                                   # 整條曲線都埋在雜訊裡
        how = '外插(點距接近雜訊)'
    (m1, f1), (m2, f2) = p1, p2
    if abs(m2 - m1) < 1e-6:
        return None
    slope = (f2 - f1) / (m2 - m1)
    if slope <= 0:
        return None                          # 特徵應該隨 mu 增加; 反過來就是雜訊太大
    est = m1 + (real - f1) / slope
    # 1-sigma: real 與 sim 特徵中位數的相對標準誤 (log_se 是比值的, 已經含兩邊)
    sigma = real * feat['log_se'] / slope
    return est, sigma, f'{how} ({m1:g}->{f1:.2f}, {m2:g}->{f2:.2f}; real {real:.2f})'


def combine_estimate(res, cur_mu, hist, term='dynamic'):
    """tied 模式: 每個特徵各自用割線法估一次 mu, 再反變異數合併。

    合併後的 1-sigma 同時含兩部分:
        組內  1/sqrt(sum 1/sigma_i^2)      每個特徵自己的量測雜訊
        組間  各特徵估計值的加權離散         特徵之間的系統差 (實測比組內大)
    只在 tied 模式用: static 跟 dynamic 綁在一起時, 這些特徵量的才是同一個量。
    回傳 (估計值, 1-sigma, 說明, 每個特徵的明細) 或 None。"""
    feats = res.get('features') or {}
    motor_ok = drivetrain_ok(res)
    parts = []
    for name in COMBINE_FEATURES:
        if name in MOTOR_DEPENDENT and not motor_ok:
            continue
        f = feats.get(name)
        if not (f and f.get('ref') and f.get('target')):
            continue
        one = {'name': name, 'sim': f['ref'], 'real': f['target'],
               'log_se': f.get('log_se', f.get('log_sd', 0.1))}
        r = secant_estimate(term, one, cur_mu, hist)
        if r and math.isfinite(r[1]) and r[1] > 0:
            parts.append((name, r[0], r[1]))
    if not parts:
        return None
    if len(parts) == 1:
        name, e, sig = parts[0]
        return e, sig, f'只有 {name} 可用', parts
    w = [1.0 / sig ** 2 for _, _, sig in parts]
    sw = sum(w)
    mu = sum(wi * e for wi, (_, e, _) in zip(w, parts)) / sw
    within = 1.0 / math.sqrt(sw)
    between = math.sqrt(sum(wi * (e - mu) ** 2 for wi, (_, e, _) in zip(w, parts)) / sw)
    how = '合併 ' + ', '.join(f'{n} {e:.3f}' for n, e, _ in parts)
    return mu, math.sqrt(within ** 2 + between ** 2), how, parts


def model_estimate(term, res):
    g = (res.get('ground_friction') or {}).get(term)
    if not g or g.get('value') is None or not math.isfinite(g['value']):
        return None
    f = (res.get('features') or {}).get(g['feature'], {})
    # 地面值對比值的導數 (數值差分), 乘上比值的標準誤
    se = f.get('log_se', float('nan'))
    lo, hi = g['ci90']
    sigma = ((hi - lo) / (2 * 1.645) * se / f['log_sd']
             if f.get('log_sd') and all(map(math.isfinite, (lo, hi, se))) else float('nan'))
    return g['value'], sigma, f'物理換算 (比值 {g["ratio"]:.3f})'


def best_iteration(hist, entry):
    """量起來最接近 real 的那一輪 (動摩擦特徵的 |log(sim/real)| 最小)。"""
    def dist(it):
        f = (it.get('features') or {}).get('dynamic')
        return abs(math.log(f['sim'] / f['real'])) if f else float('inf')
    cands = [it for it in hist['iterations'] if it.get('features')] + [entry]
    return min(cands, key=dist)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--real', required=True, help='real 環境的 CSV (目標)')
    ap.add_argument('--sim', required=True, help='這一輪 sim 的 CSV (參考)')
    ap.add_argument('--static', type=float, required=True, help='這一輪 sim 地面 staticFriction')
    ap.add_argument('--dynamic', type=float, required=True, help='這一輪 sim 地面 dynamicFriction')
    ap.add_argument('--iter', type=int, required=True, help='第幾輪 (從 0 開始)')
    ap.add_argument('--history', help='歷史紀錄 JSON (跨輪累積)')
    ap.add_argument('--tol', type=float, default=0.05,
                    help='收斂容許誤差 (地面摩擦係數的絕對值)')
    ap.add_argument('--max-iter', type=int, default=6, help='最多量幾輪 sim')
    ap.add_argument('--max-retry', type=int, default=1, help='同一組值資料不能用時最多重量幾次')
    ap.add_argument('--gain', type=float, default=1.0, help='更新步長 (1 = 直接跳到估計值)')
    ap.add_argument('--max-step', type=float, default=3.0, help='單步最多變成幾倍 / 幾分之一')
    ap.add_argument('--mu-min', type=float, default=0.05)
    ap.add_argument('--mu-max', type=float, default=5.0)
    ap.add_argument('--wheel-mu', type=float, default=0.5, help='輪子材質 mu (PhysX 預設 0.5)')
    ap.add_argument('--combine', default='average', choices=['average', 'min', 'multiply', 'max'])
    ap.add_argument('--static-mode', default='tied', choices=['tied', 'free'],
                    help='tied = static 跟著 dynamic 走 (預設, 見檔頭); free = 用 B2 分開解')
    ap.add_argument('--source', default=None, help='傳給 estimate_friction 的 --source')
    ap.add_argument('--boot', type=int, default=1000)
    args = ap.parse_args()

    hist = load_history(args.history)
    json_path = os.path.splitext(args.sim)[0] + '_vs_real.json'
    entry = {'iter': args.iter, 'sim_csv': args.sim, 'static': args.static,
             'dynamic': args.dynamic}

    res = run_estimate(args, json_path)
    retries = sum(1 for it in hist['iterations']
                  if it['static'] == args.static and it['dynamic'] == args.dynamic
                  and it.get('status') == 'retry')

    def finish(status, static, dynamic, reason):
        entry.update(status=status, next_static=static, next_dynamic=dynamic, reason=reason)
        hist['iterations'].append(entry)
        save_history(args.history, hist)
        emit(status, static, dynamic, reason)

    def retry_or_fail(reason):
        if retries < args.max_retry:
            finish('retry', args.static, args.dynamic, reason + ' -> 同樣的值重量一次')
        else:
            finish('failed', args.static, args.dynamic, reason + f' (已重試 {retries} 次)')

    if res is None:
        return retry_or_fail('estimate_friction 執行失敗')

    lq = res.get('loc_quality') or {}
    bad = [k for k, v in lq.items() if v and v.get('bad')]
    if bad:
        return retry_or_fail('收資料時定位壞掉 (' + ', '.join(
            f'{"sim" if k == "ref" else "real"}: {lq[k]["message"]}' for k in bad) + ')')

    feats = pick_features(res)
    if args.static_mode == 'tied':
        feats.pop('static', None)             # 只解 dynamic, static 等比例跟著走
    entry['features'] = feats
    # 每個特徵都存起來 (不只選中的那個): 之後的輪次才能各自畫出自己的
    # 「mu -> 特徵」曲線來合併, 見 combine_estimate
    allf = res.get('features') or {}
    entry['all_features'] = {
        n: {'sim': allf[n]['ref'], 'real': allf[n]['target'],
            'log_se': allf[n].get('log_se', allf[n].get('log_sd', 0.1))}
        for n in COMBINE_FEATURES
        if allf.get(n) and allf[n].get('ref') and allf[n].get('target')}
    entry['ratio'] = (res.get('combined') or {}).get('ratio')
    if 'dynamic' not in feats:
        return retry_or_fail('B1 自旋減速度沒有資料')

    cur = {'static': args.static, 'dynamic': args.dynamic}
    est, lines = {}, []
    for term in ('dynamic', 'static'):
        if term not in feats:
            continue
        r, parts, label = None, None, feats[term]['name']
        if term == 'dynamic' and args.static_mode == 'tied':
            # static 綁著 dynamic -> 所有特徵量的是同一個 mu, 可以合併
            c = combine_estimate(res, cur[term], hist)
            if c:
                r, parts, label = c[:3], c[3], f'{len(c[3])} 個特徵'
        if r is None:
            r = secant_estimate(term, feats[term], cur[term], hist) or model_estimate(term, res)
        if r is None:
            return finish('failed', args.static, args.dynamic,
                          f'{term} 估不出來 (combine={args.combine} 下地面不是瓶頸? 特徵不隨 mu 增加?)')
        est[term] = r
        e, sig, how = r
        lines.append(f'{term}: 目前 {cur[term]:.3f}, 估計 {e:.3f} ± {sig:.3f} [{label}, {how}]')
        if parts and len(parts) > 1:
            entry['combined_from'] = [{'name': n, 'value': v, 'sigma': g} for n, v, g in parts]
            spread = max(v for _, v, _ in parts) - min(v for _, v, _ in parts)
            if spread > 3 * min(g for _, _, g in parts):
                lines.append(f'  ! 特徵之間差到 {spread:.3f} (最小的 1-sigma 只有 '
                             f'{min(g for _, _, g in parts):.3f}) —— 已經算進上面的誤差裡, '
                             '但代表特徵之間有系統差')
        if math.isfinite(sig) and sig > args.tol:
            lines.append(f'  ! {term} 的解析度 ±{sig:.3f} 比容許誤差 {args.tol:g} 差 —— '
                         '收斂判定會被雜訊左右, 要更準得加 trial')
    entry['estimate'] = {t: {'value': v[0], 'sigma': v[1], 'how': v[2]} for t, v in est.items()}
    print('[calib] ' + '\n[calib] '.join(lines))

    if all(abs(est[t][0] - cur[t]) <= args.tol for t in est):
        fin = {t: est[t][0] if t in est else cur[t] for t in cur}
        if 'static' not in est:
            # tied: 直接等於 dynamic; free 但 B2 沒資料: 維持原本的靜/動比例
            fin['static'] = (fin['dynamic'] if args.static_mode == 'tied'
                             else cur['static'] * fin['dynamic'] / cur['dynamic'])
        return finish('converged', fin['static'], fin['dynamic'],
                      '修正量都 <= %g: ' % args.tol + '; '.join(
                          f'{t} {cur[t]:.3f} -> {fin[t]:.3f}' for t in est))

    new = dict(cur)
    for t, (e, _, _) in est.items():
        target = cur[t] + args.gain * (e - cur[t])
        new[t] = min(max(target, cur[t] / args.max_step, args.mu_min),
                     cur[t] * args.max_step, args.mu_max)
    if 'static' not in est:
        # tied: static 就是 dynamic; free 但 B2 沒資料: 維持原本的靜/動比例
        tgt = (new['dynamic'] if args.static_mode == 'tied'
               else cur['static'] * new['dynamic'] / cur['dynamic'])
        new['static'] = min(max(tgt, args.mu_min), args.mu_max)
    desc = ', '.join(f'{t} {cur[t]:.3f} -> {new[t]:.3f}' for t in cur)

    if args.iter + 1 >= args.max_iter:
        best = best_iteration(hist, entry)
        return finish('max_iter', best['static'], best['dynamic'],
                      f'{args.max_iter} 輪還沒收斂 (下一步本來是 {desc}); 用量起來最接近 real 的'
                      f'第 {best["iter"]} 輪 (static {best["static"]:.3f}, dynamic {best["dynamic"]:.3f})')

    for it in hist['iterations']:
        if (abs(it['static'] - new['static']) < 0.01 * new['static']
                and abs(it['dynamic'] - new['dynamic']) < 0.01 * new['dynamic']
                and it.get('status') != 'retry'):
            for t in cur:
                new[t] = 0.5 * (new[t] + cur[t])
            desc = ', '.join(f'{t} {cur[t]:.3f} -> {new[t]:.3f}' for t in cur)
            desc += f' (跟第 {it["iter"]} 輪幾乎一樣, 步長減半)'
            break

    finish('continue', new['static'], new['dynamic'], desc)


if __name__ == '__main__':
    main()
