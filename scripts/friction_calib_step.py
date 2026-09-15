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
    staticFriction  <- B2 慢速起轉 effort (creep_breakaway), 沒有就用 B1 spin_up 起轉

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
TERM_FEATURES = {
    'dynamic': ['spin_decel'],
    'static': ['creep_breakaway', 'spinup_breakaway'],
}


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


def pick_features(res):
    """這一輪每個係數用的特徵: {term: {name, sim, real, log_sd}}。"""
    out = {}
    feats = res.get('features') or {}
    for term, names in TERM_FEATURES.items():
        for n in names:
            f = feats.get(n)
            if f and f.get('ref') and f.get('target'):
                out[term] = {'name': n, 'sim': f['ref'], 'real': f['target'],
                             'log_se': f.get('log_se', f.get('log_sd', 0.1))}
                break
    return out


def secant_estimate(term, feat, cur_mu, hist):
    """用歷史上同一個特徵的 (sim 係數, sim 特徵) 點內插 real 的特徵值。

    回傳 (估計值, 1-sigma, 說明) 或 None (點不夠 / 斜率不合理)。"""
    pts = {}
    for it in hist['iterations']:
        f = (it.get('features') or {}).get(term)
        if f and f['name'] == feat['name'] and it.get('status') != 'retry':
            pts[round(it[term], 6)] = f['sim']        # 同一個係數量多次就取最後一次
    pts[round(cur_mu, 6)] = feat['sim']
    if len(pts) < 2:
        return None
    real = feat['real']
    items = sorted(pts.items())
    below = [p for p in items if p[1] <= real]
    above = [p for p in items if p[1] > real]
    if below and above:                      # real 被夾在中間 -> 內插, 取最靠近的一對
        p1 = max(below, key=lambda p: p[1])
        p2 = min(above, key=lambda p: p[1])
        how = '內插'
    else:                                    # 全在同一側 -> 用最靠近 real 的兩點外插
        p1, p2 = sorted(items, key=lambda p: abs(p[1] - real))[:2]
        how = '外插'
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
    entry['features'] = feats
    entry['ratio'] = (res.get('combined') or {}).get('ratio')
    if 'dynamic' not in feats:
        return retry_or_fail('B1 自旋減速度沒有資料')

    cur = {'static': args.static, 'dynamic': args.dynamic}
    est, lines = {}, []
    for term in ('dynamic', 'static'):
        if term not in feats:
            continue
        r = secant_estimate(term, feats[term], cur[term], hist) or model_estimate(term, res)
        if r is None:
            return finish('failed', args.static, args.dynamic,
                          f'{term} 估不出來 (combine={args.combine} 下地面不是瓶頸? 特徵不隨 mu 增加?)')
        est[term] = r
        e, sig, how = r
        lines.append(f'{term}: 目前 {cur[term]:.3f}, 估計 {e:.3f} ± {sig:.3f} '
                     f'[{feats[term]["name"]}, {how}]')
        if math.isfinite(sig) and sig > args.tol:
            lines.append(f'  ! {term} 的解析度 ±{sig:.3f} 比容許誤差 {args.tol:g} 差 —— '
                         '收斂判定會被雜訊左右, 要更準得加 trial')
    entry['estimate'] = {t: {'value': v[0], 'sigma': v[1], 'how': v[2]} for t, v in est.items()}
    print('[calib] ' + '\n[calib] '.join(lines))

    if all(abs(est[t][0] - cur[t]) <= args.tol for t in est):
        fin = {t: est[t][0] if t in est else cur[t] for t in cur}
        if 'static' not in est:
            fin['static'] = cur['static'] * fin['dynamic'] / cur['dynamic']
        return finish('converged', fin['static'], fin['dynamic'],
                      '修正量都 <= %g: ' % args.tol + '; '.join(
                          f'{t} {cur[t]:.3f} -> {fin[t]:.3f}' for t in est))

    new = dict(cur)
    for t, (e, _, _) in est.items():
        target = cur[t] + args.gain * (e - cur[t])
        new[t] = min(max(target, cur[t] / args.max_step, args.mu_min),
                     cur[t] * args.max_step, args.mu_max)
    if 'static' not in est:                   # 靜摩擦沒資料 -> 跟著動摩擦等比例
        new['static'] = min(max(cur['static'] * new['dynamic'] / cur['dynamic'],
                                args.mu_min), args.mu_max)
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
