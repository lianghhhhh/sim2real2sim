#!/usr/bin/env python3
"""Allan variance 與「白雜訊 + 一階 Gauss-Markov」的擬合 —— 只用 numpy。

感測器模型 (單一軸, 靜止):

    y(t) = b0 + b_dyn(t) + n_w(t)
    db_dyn/dt = -b_dyn/tau + n_b

三個要擬合的量:

    N         白雜訊密度 (單位/sqrt(Hz))。陀螺儀叫 ARW, 加速度計叫 VRW。
    sigma_gm  b_dyn 的穩態標準差
    tau       b_dyn 的相關時間 (s)

b0 是整段資料的平均, 不用擬合。

為什麼用 Allan variance 而不是直接擬合時間序列: 白雜訊在原始訊號裡比零偏
的變化大好幾個數量級, 直接看看不到零偏。Allan variance 把訊號依平均時間 T
分開 —— 白雜訊隨 T 以 1/T 下降, GM 在 T ~ 1.89*tau 附近隆起, 兩者在 log-log
圖上分得很開。

模型的 Allan variance (IEEE Std 952 的指數相關雜訊):

    AVAR(T) = N^2/T + sigma_gm^2 * g(T, tau)
    g(T, tau) = (2*tau/T) * [1 - tau/(2T) * (3 - 4*exp(-T/tau) + exp(-2T/tau))]

給定 tau 之後對 N^2 與 sigma_gm^2 是**線性**的, 所以擬合只需要對 tau 做一維
搜尋, 每個 tau 解一次 2 變數的非負最小平方 —— 不需要 scipy。
"""
from __future__ import annotations

import math

import numpy as np


def allan_variance(x, fs: float, n_taus: int = 60, max_frac: float = 0.1):
    """重疊式 Allan variance。

    x   : 靜止時某一軸的讀數 (1 維)
    fs  : 取樣率 (Hz)
    回傳 (T, avar, relerr):
        T      平均時間 (s)
        avar   Allan variance
        relerr avar 的相對標準誤 (約 1/sqrt(獨立區段數)); T 越大越不準

    max_frac: 最大的 T 是資料長度的這個比例。超過 ~0.1 的話每個點只剩
    不到 10 個獨立區段, 估計值本身就在亂跳。
    """
    x = np.asarray(x, dtype=np.float64)
    n = len(x)
    if n < 100:
        raise ValueError(f'資料太短 ({n} 筆), 算不出 Allan variance')
    theta = np.concatenate([[0.0], np.cumsum(x)]) / fs      # 積分
    m_max = max(int(n * max_frac), 2)
    ms = np.unique(np.round(np.logspace(0, math.log10(m_max), n_taus)).astype(int))
    T = ms / fs
    avar = np.empty(len(ms))
    for i, m in enumerate(ms):
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        avar[i] = float(np.mean(d * d)) / (2.0 * T[i] ** 2)
    relerr = 1.0 / np.sqrt(2.0 * np.maximum(n / ms - 1.0, 1.0))
    return T, avar, relerr


def gm_shape(T, tau: float):
    """sigma_gm = 1 的 GM 過程的 Allan variance。"""
    T = np.asarray(T, dtype=np.float64)
    u = T / tau
    # u 很小的時候括號裡是 1 - (1 - u/3 + ...) , 直接算會被捨入誤差吃掉,
    # 改用級數: g -> (2/3)*u*(1 - u/4 + ...) 。
    small = u < 1e-3
    us = np.where(small, 1.0, u)
    g = (2.0 / us) * (1.0 - (3.0 - 4.0 * np.exp(-us) + np.exp(-2.0 * us)) / (2.0 * us))
    return np.where(small, (2.0 / 3.0) * u * (1.0 - u / 4.0), g)


def model_avar(T, n_white: float, sigma_gm: float, tau: float):
    T = np.asarray(T, dtype=np.float64)
    return n_white ** 2 / T + sigma_gm ** 2 * gm_shape(T, tau)


def _nnls2(A, b):
    """2 個未知數的非負最小平方。回傳 (係數, 殘差平方和)。"""
    best = (np.zeros(2), float(b @ b))
    c, *_ = np.linalg.lstsq(A, b, rcond=None)
    cands = [c] if (c >= 0).all() else []
    for j in (0, 1):                     # 邊界解: 只留一項
        a = A[:, j]
        cj = max(float(a @ b) / float(a @ a), 0.0)
        v = np.zeros(2)
        v[j] = cj
        cands.append(v)
    for v in cands:
        r = A @ v - b
        s = float(r @ r)
        if s < best[1]:
            best = (v, s)
    return best


def fit_white_gm(T, avar, relerr, duration: float):
    """把 Allan variance 擬合成白雜訊 + 一階 GM。

    回傳 dict:
        n_white   白雜訊密度
        sigma_gm  GM 穩態標準差 (0 = 沒有偵測到)
        tau       GM 相關時間 (s)
        cost      擬合殘差 (加權後的相對誤差平方和 / 點數)
        cost_white_only  只用白雜訊去擬合的殘差 —— 跟 cost 比就知道 GM 那一項
                         是不是真的存在
        has_gm    GM 項是否顯著
        tau_resolved  tau 是否落在這段資料分辨得出來的範圍內
        rw_equiv  等效的隨機遊走強度 sqrt(2*sigma_gm^2/tau)
    """
    T = np.asarray(T, dtype=np.float64)
    y = np.asarray(avar, dtype=np.float64)
    # 用相對誤差當殘差 (曲線跨好幾個數量級), 再依每個點自己的不確定度加權。
    # 不確定度設一個下限: 短 T 的點統計上極準, 但模型在那裡不一定完全成立
    # (真實感測器有量化、低通), 不設下限的話整個擬合會被那幾個點綁架。
    w = 1.0 / (y * np.maximum(relerr, 0.05))

    def solve(tau):
        A = np.stack([1.0 / T, gm_shape(T, tau)], axis=1) * w[:, None]
        return _nnls2(A, y * w)

    # 只有白雜訊
    a0 = (1.0 / T) * w
    c0 = max(float(a0 @ (y * w)) / float(a0 @ a0), 0.0)
    cost0 = float(np.sum((a0 * c0 - y * w) ** 2)) / len(T)

    lo, hi = math.log(T[0] * 2.0), math.log(duration)
    grid = np.linspace(lo, hi, 240)
    costs = np.array([solve(math.exp(g))[1] for g in grid])
    k = int(np.argmin(costs))
    # 在最佳格點附近用黃金比例搜尋細修
    a, b = grid[max(k - 1, 0)], grid[min(k + 1, len(grid) - 1)]
    gr = (math.sqrt(5.0) - 1.0) / 2.0
    for _ in range(40):
        c, d = b - gr * (b - a), a + gr * (b - a)
        if solve(math.exp(c))[1] < solve(math.exp(d))[1]:
            b = d
        else:
            a = c
    tau = math.exp(0.5 * (a + b))
    coef, sse = solve(tau)
    cost = sse / len(T)
    n_white = math.sqrt(coef[0])
    sigma_gm = math.sqrt(coef[1])
    # GM 項「存在」的條件: 係數不是 0, 而且加了它之後殘差明顯變小
    has_gm = bool(sigma_gm > 0.0 and cost < 0.5 * cost0)
    # tau 要「分辨得出來」, 資料至少要有幾十個相關時間。少於這個的話曲線上
    # 只看得到 GM 上升的那一段, 那一段跟隨機遊走長得一模一樣。
    tau_resolved = bool(has_gm and tau < duration / 20.0)
    return dict(n_white=n_white, sigma_gm=sigma_gm if has_gm else 0.0, tau=tau,
                cost=cost, cost_white_only=cost0, has_gm=has_gm,
                tau_resolved=tau_resolved,
                rw_equiv=math.sqrt(2.0 * coef[1] / tau) if has_gm else 0.0)


def fit_axis(x, fs: float, **kw):
    """一步到位: 一個軸的靜止資料 -> (b0, 擬合結果, Allan 曲線)。"""
    x = np.asarray(x, dtype=np.float64)
    T, avar, relerr = allan_variance(x, fs, **kw)
    fit = fit_white_gm(T, avar, relerr, duration=len(x) / fs)
    fit['b0'] = float(x.mean())
    return fit, (T, avar, relerr)
