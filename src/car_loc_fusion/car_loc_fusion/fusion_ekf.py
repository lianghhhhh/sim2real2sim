#!/usr/bin/env python3
"""把「航位推算」跟「絕對定位」接起來 —— 以及延遲量測要怎麼處理。

    高頻 (60~200 Hz)   /imu + /joint_states  ──> 遞推: 連續、不跳、不會斷
    低頻 (10~30 Hz)    相機 pose / LiDAR pose ──> 修正: 不漂、可以自己找到自己

這個模組跟 ROS 無關, 所以 `test/test_fusion.py` 不需要 ROS / Isaac 就能跑。

為什麼這個 package 可以 import car_loc_wheel
--------------------------------------------
另外四個 package **刻意互不相干** —— 那是為了讓它們能拿同一趟資料公平比較。
這一個是**把它們結合起來**的那個, 所以規則不適用: 它直接用
`car_loc_wheel.wheel_ins.WheelIns` 當遞推核心, 而不是把那 700 行 (含打滑三道
防線、輪速逃生口、零加速度前提檢查) 抄第二份。**同一段程式碼只能有一份**,
不然兩邊的參數與修正會慢慢分岔, 而分岔的那一天不會有人發現。

這裡新加的只有兩件事: 絕對量測怎麼進來 (`absolute`), 以及延遲怎麼處理 (下面)。

=== 這個 package 真正在解的問題: 絕對量測是**過去**的 ===
--------------------------------------------------------
相機那條路的 pose 帶著曝光 + 傳輸 + YOLO 推論的延遲, `car_loc_camera` 量出來是
**79.5 ms**, 而且它很誠實 —— 它把時戳往回填成影像時刻 (`delay` 參數), 所以收到
的那一則 pose 說的是「80 ms 前車子在哪」。LiDAR 那條也有半圈掃描 + 配準的時間。

**把它當成「現在」直接更新, 等於灌一個 `v · Δt` 的位置誤差進去**, 而且方向跟著
車頭轉, 平均不掉:

    0.8 m/s x 0.08 s = 6.4 cm      3.5 m/s x 0.08 s = 28 cm

—— 比相機自己的 2.1 cm 精度大一個數量級。**融合做得不對, 比不融合還糟。**

處理方式是**倒帶重放** (rewind / fixed-lag re-linearization):

    1. 濾波器保留最近 `horizon` 秒的 IMU 輸入與狀態快照
    2. 收到時戳 t_m 的 pose 時, 把狀態倒回 t_m **之前**最近的一個快照
    3. 用那一步的輸入 ZOH 遞推到 t_m, 在**正確的時刻**做更新
    4. 把 t_m 之後的每一步 (含期間已經套過的其他絕對量測) 照原順序重放一次

第 4 步的「含其他絕對量測」不能省: 相機 (30 Hz, 延遲 80 ms) 與 LiDAR (10 Hz,
延遲較小) 的到達順序跟時戳順序**不一樣**, 倒帶到相機的時刻會跨過已經套用的
LiDAR 更新。不重放的話那次更新就憑空消失了。

代價是每則絕對量測要重跑 `horizon x IMU 頻率` 步 (0.3 s x 60 Hz = 18 步)。
60 Hz IMU + 40 Hz 絕對量測 -> 每秒約 700 次 7x7 的遞推, 在 Python 裡也還好。

值多少: 見 README「[2] 延遲補償」那張表 (`test/test_fusion.py` 印出來的)。
"""
from __future__ import annotations

import math
import os
import sys
from collections import deque

import numpy as np

# ament 裝好之後 car_loc_wheel 就在 PYTHONPATH 裡; 離線測試 (不裝 ROS) 時
# 從 source tree 直接找過去, 這樣 test/ 不需要 colcon build 就能跑。
try:
    from car_loc_wheel.wheel_ins import IP, IYAW, NX, WheelIns, wrap_pi
except ImportError:                                          # pragma: no cover
    _here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.abspath(os.path.join(_here, '..', '..', 'car_loc_wheel')))
    from car_loc_wheel.wheel_ins import IP, IYAW, NX, WheelIns, wrap_pi

CHI2_99 = {1: 6.635, 2: 9.210, 3: 11.345}


class Step:
    """一個 IMU 取樣, 以及那一刻要跟著做的所有事。

    **所有的遞推與高頻更新都必須走這個結構**, 因為倒帶時要一模一樣地重放一次。
    節點裡不要有「只在 live 路徑上做」的修正 —— 那種東西倒帶之後就消失了,
    而且症狀是隨機的 (取決於當時有沒有絕對量測進來)。
    """

    __slots__ = ('t', 'a_fwd', 'omega_z', 'v_wheel', 'spread', 'still',
                 'yaw_meas', 'yaw_sigma', 'dt', 'snap')

    def __init__(self, t, a_fwd, omega_z, *, v_wheel=None, spread=0.0,
                 still=False, yaw_meas=None, yaw_sigma=0.02, dt=0.0):
        self.t = float(t)
        self.a_fwd = float(a_fwd)
        self.omega_z = float(omega_z)
        self.v_wheel = None if v_wheel is None else float(v_wheel)
        self.spread = float(spread)
        self.still = bool(still)
        self.yaw_meas = None if yaw_meas is None else float(yaw_meas)
        self.yaw_sigma = float(yaw_sigma)
        self.dt = float(dt)
        self.snap = None            # 套用**之前**的狀態快照 (倒帶用)


class AbsMeas:
    """一則絕對位姿量測 (map 座標)。"""

    __slots__ = ('t', 'src', 'x', 'y', 'yaw', 'sigma', 'yaw_sigma', 'r_scale')

    def __init__(self, t, src, x, y, sigma, yaw=None, yaw_sigma=0.05):
        self.t = float(t)
        self.src = str(src)
        self.x = float(x)
        self.y = float(y)
        self.yaw = None if yaw is None else float(yaw)
        self.sigma = float(sigma)
        self.yaw_sigma = float(yaw_sigma)
        # 這一則**當初被接受時**用的 R 放大倍率 (逃生口強制接受時是 9)。
        # 重放時必須用同一個值、而且不可以重新判閘門 —— 見 _abs_update。
        self.r_scale = 1.0


class FusionEkf:
    """WheelIns (遞推) + 絕對量測 (修正) + 倒帶重放 (延遲)。"""

    def __init__(self, *, rewind: bool = True, rewind_horizon: float = 0.4,
                 abs_max_correction: float = 1.0,
                 abs_reject_time: float = 3.0,
                 chi2_scale: float = 1.0,
                 **ins_kwargs):
        self.ins = WheelIns(**ins_kwargs)
        self.rewind = bool(rewind)
        self.horizon = float(rewind_horizon)
        # 絕對量測的單次修正上限。**比 WheelIns 內部那個 (0.5 m) 大**, 這是刻意的:
        # 絕對量測本來就該有「把跑掉的估計拉回來」的權力, 那正是它存在的理由。
        # 但仍然要有上限 —— 一則誤判 (相機認到影子) 不該讓車子瞬間搬家。
        self.abs_max_correction = float(abs_max_correction)
        # 時戳比濾波器新超過這麼多就當成「未來的量測」(時鐘沒對好)。
        # 一點點正值是正常的: 量測可能比最後一則 IMU 稍晚幾毫秒。
        self.future_tol = 0.05
        # 緩衝區的筆數上限 (時間判斷之外的第二道保險)。
        # 0.4 s x 200 Hz IMU = 80 步, 512 有六倍餘裕。
        self.max_buf = 512
        # 連續這麼久沒有任何一則絕對量測通過閘門 -> 強制接受一次 (R 放大 3 倍)。
        # 理由跟 car_loc_wheel 的 wheel_reject_time 完全一樣: 一次擋掉是量測的
        # 問題, 連續好幾秒都對不上就是**濾波器自己**錯了, 而絕對量測是這裡唯一
        # 能把它拉回來的東西。沒有這個逃生口的話, 遞推一旦跑掉就再也回不來。
        self.abs_reject_time = float(abs_reject_time)
        self.chi2_scale = float(chi2_scale)

        self.initialized = False
        self.buf = deque()          # Step, 時間遞增
        self.abs_log = deque()      # AbsMeas (已套用的), 時間遞增
        self.reject_since = None
        self.recovering = False
        self.counts = {'abs': 0, 'gate': 0, 'damped': 0, 'forced': 0,
                       'rewind': 0, 'too_old': 0, 'future': 0, 'yaw': 0,
                       'replay': 0}
        self.last_abs_t = None

    # ------------------------------------------------------------------ 對外
    @property
    def pos(self):
        return self.ins.pos

    @property
    def yaw(self) -> float:
        return self.ins.yaw

    @property
    def heading(self) -> float:
        return self.ins.heading

    @property
    def speed(self) -> float:
        return self.ins.speed

    @property
    def t(self):
        return self.ins.t

    def sigma_pos(self) -> float:
        return self.ins.sigma_pos()

    def pose_cov(self):
        return self.ins.pose_cov()

    def twist_cov(self):
        return self.ins.twist_cov()

    def set_pose(self, x, y, yaw, t=None, sigma_p=0.05, sigma_yaw=0.05):
        self.ins.set_pose(x, y, yaw, t=t, sigma_p=sigma_p, sigma_yaw=sigma_yaw)
        self.initialized = True
        self.recovering = False
        self.reject_since = None
        self.buf.clear()
        self.abs_log.clear()

    # ------------------------------------------------------------------ 遞推
    def step(self, st: Step):
        """一個 IMU 取樣。節點每收到一則 /imu 就呼叫一次。"""
        st.snap = self._snapshot()
        self._apply_step(st)
        self.buf.append(st)
        self._trim(st.t)

    def _apply_step(self, st: Step):
        ins = self.ins
        ins.predict(st.t, st.a_fwd, st.omega_z)
        if st.yaw_meas is not None:
            ins.update_yaw(st.yaw_meas, st.yaw_sigma, st.dt)
        if st.v_wheel is not None:
            ins.update_wheel(st.v_wheel, st.spread)
        if st.still:
            if ins.still_since is None:
                ins.still_since = st.t
            ins.zupt()
            ins.zero_accel(st.a_fwd)
            ins.zaru(st.omega_z)
            if st.t - ins.still_since >= ins.anchor_after:
                ins.anchor_position()
        else:
            ins.still_since = None
            ins.anchor = None

    # ------------------------------------------------------- 絕對量測 (主角)
    def absolute(self, m: AbsMeas, *, use_yaw: bool = False,
                 max_correction: float = None) -> bool:
        """套用一則絕對位姿。**時戳 `m.t` 是量測發生的時刻, 不是收到的時刻。**"""
        if not self.initialized:
            # 第一則絕對量測就是初始位姿 —— 這是融合相對於方法三/四的一個實質
            # 差別: 那兩條非給 initial_pose 不可 (給錯就整段差一個常數平移),
            # 這一條自己找得到自己。
            self.set_pose(m.x, m.y, m.yaw if m.yaw is not None else 0.0,
                          t=m.t, sigma_p=max(m.sigma, 0.05),
                          sigma_yaw=(m.yaw_sigma if m.yaw is not None else 1.0))
            self.last_abs_t = m.t
            self.counts['abs'] += 1
            return True

        if self.ins.t is None:
            return False

        lag = self.ins.t - m.t
        if lag < -self.future_tol:
            # **時戳在未來。** 延遲量測的 lag 依定義 >= 0, 負的就表示時鐘不對
            # (見 sources.py 的「時鐘」那一節: 實測 /rgb 比 /imu 早 625 秒)。
            # 這裡只做最後一道保險 —— 正常情況下 AbsSource 已經把偏移扣掉了。
            # 硬套在現在的狀態上比丟掉安全 (至少位置是對的), 但要記數,
            # status 的「未來」不是 0 就表示時鐘沒對好, 延遲補償等於沒有。
            self.counts['future'] += 1
            return self._abs_update(m, use_yaw=use_yaw,
                                    max_correction=max_correction)
        if not self.rewind or lag <= 1e-6:
            return self._abs_update(m, use_yaw=use_yaw,
                                    max_correction=max_correction)

        if lag > self.horizon or not self.buf:
            # 倒帶不回去。**丟掉比硬套安全** —— 硬套等於把 lag x v 的誤差當成
            # 位置修正灌進去, 而 lag > horizon 時那個量已經很大了。
            # 這個計數器不是 0 的話, 把 rewind_horizon 調大 (代價只有記憶體)。
            self.counts['too_old'] += 1
            return False

        return self._rewind_apply(m, use_yaw=use_yaw,
                                  max_correction=max_correction)

    def _rewind_apply(self, m: AbsMeas, *, use_yaw: bool,
                      max_correction) -> bool:
        """倒帶到 m.t, 在正確的時刻更新, 再把後面的每一步重放一次。"""
        steps = list(self.buf)
        # 第一個「比量測晚」的步 —— 它的 snap 就是 <= m.t 的狀態
        i = 0
        while i < len(steps) and steps[i].t <= m.t:
            i += 1
        if i == 0:
            self.counts['too_old'] += 1
            return False

        tail = steps[i:]
        if not tail:
            # 量測比最後一步還新 —— lag > 0 時不該發生, 保守起見直接套在現在
            return self._abs_update(m, use_yaw=use_yaw,
                                    max_correction=max_correction)
        # steps[i].snap 是「套用 steps[i] **之前**」的狀態, 時刻 = steps[i-1].t
        # <= m.t, 正是我們要倒回去的地方。
        self._restore(tail[0].snap)

        # 把期間已經套用過的絕對量測一起排進事件序列, 不然倒帶會把它們洗掉
        t0 = self.ins.t if self.ins.t is not None else tail[0].t
        replay_abs = [a for a in self.abs_log if a.t > t0]
        events = sorted([(s.t, 0, s) for s in tail]
                        + [(a.t, 1, a) for a in replay_abs]
                        + [(m.t, 1, m)], key=lambda e: (e[0], e[1]))

        self.buf = deque(steps[:i])
        self.abs_log = deque(a for a in self.abs_log if a.t <= t0)
        self.counts['rewind'] += 1

        ok = False
        for k, (_, kind, ev) in enumerate(events):
            if kind == 0:
                ev.snap = self._snapshot()
                self._apply_step(ev)
                self.buf.append(ev)
                self.counts['replay'] += 1
            else:
                # 絕對量測落在兩步中間: 先用**下一步**的輸入 ZOH 遞推到它的時刻。
                # 剩下的那一段由下一步自己補 (predict 是用 dt = t - self.t 算的,
                # 所以「先推一半再推另一半」跟「一次推完」是同一件事)。
                nxt = next((e[2] for e in events[k + 1:] if e[1] == 0), None)
                if nxt is not None:
                    self.ins.predict(ev.t, nxt.a_fwd, nxt.omega_z)
                res = self._abs_update(
                    ev, use_yaw=(use_yaw if ev is m else ev.yaw is not None),
                    max_correction=max_correction, replay=(ev is not m))
                if ev is m:
                    ok = res
        return ok

    def _abs_update(self, m: AbsMeas, *, use_yaw: bool,
                    max_correction=None, replay: bool = False) -> bool:
        """在**目前**的狀態時刻套用一則絕對位姿 (位置 2 維 + 可選 yaw)。"""
        ins = self.ins
        lim = self.abs_max_correction if max_correction is None else max_correction

        H = np.zeros((2, NX))
        H[0, 0] = H[1, 1] = 1.0
        r = np.array([m.x - ins.x[0], m.y - ins.x[1]])
        R = np.eye(2) * (m.sigma ** 2)

        if replay:
            # **重放時不重新判閘門。** 這一則當初已經被接受過了 (含逃生口強制
            # 接受的那些), 重放的工作是「照原樣再做一次」, 不是再決定一次。
            #
            # 這裡踩過一個很難看的坑: 原本重放時會重新算 NIS, 於是逃生口好不容易
            # 強制吃進去的那一次修正, 在下一則量測觸發倒帶時又被判成離群值丟掉 ——
            # 修正被無聲地洗掉, 症狀是「綁架之後**永遠**回不來, 但 forced 計數
            # 一直在增加」。倒帶重放的鐵律: **重放必須重現當初的決定。**
            R = R * m.r_scale
            S = H @ ins.P @ H.T + R
            try:
                Si = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                return False
            forced = False
        else:
            S = H @ ins.P @ H.T + R
            try:
                Si = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                return False
            nis = float(r @ Si @ r)
            if nis <= CHI2_99[2] * self.chi2_scale:
                self.reject_since = None
                self.recovering = False       # 正常通過 = 濾波器跟量測和好了
            else:
                # 逃生口。**一次強制接受不夠** —— 單次修正被 max_correction 綁住
                # (預設 1 m), 而它要處理的情況 (綁架、遞推整個跑掉) 動輒好幾公尺。
                # 所以觸發之後進入**恢復模式**: 一直強制接受, 直到有一則自己通過
                # 閘門為止。收斂時間因此是「幾則量測」而不是「幾次 abs_reject_time」。
                # **計時用濾波器自己的時鐘 (`ins.t`), 不要用量測時戳。**
                # 兩個來源的時戳如果在不同的時鐘上, 一相減就是幾百秒, 逃生口會
                # 在第一次被擋的時候就觸發 —— 那正好是它最不該觸發的時候。
                tnow = self.ins.t if self.ins.t is not None else m.t
                if not self.recovering:
                    if (self.abs_reject_time <= 0.0 or self.reject_since is None
                            or (tnow - self.reject_since) <= self.abs_reject_time):
                        if self.reject_since is None:
                            self.reject_since = tnow
                        self.counts['gate'] += 1
                        return False
                    self.recovering = True
                m.r_scale = 9.0          # 放大 3 倍 sigma: 走過去, 不要跳過去
                R = R * m.r_scale
                S = H @ ins.P @ H.T + R
                Si = np.linalg.inv(S)
                self.counts['forced'] += 1

        K = ins.P @ H.T @ Si
        dx = K @ r
        dp = float(np.linalg.norm(dx[IP]))
        if lim > 0.0 and dp > lim:
            K = K * (lim / dp)
            dx = K @ r
            if not replay:
                self.counts['damped'] += 1
        ins.x = ins.x + dx
        ins.x[IYAW] = wrap_pi(ins.x[IYAW])
        I_KH = np.eye(NX) - K @ H
        ins.P = I_KH @ ins.P @ I_KH.T + K @ R @ K.T      # Joseph form
        ins.P = 0.5 * (ins.P + ins.P.T)

        if use_yaw and m.yaw is not None:
            Hy = np.zeros((1, NX))
            Hy[0, IYAW] = 1.0
            ry = np.array([wrap_pi(m.yaw - ins.x[IYAW])])
            ins._update(Hy, ry, np.array([[m.yaw_sigma ** 2]]), 'yaw')
            if not replay:
                self.counts['yaw'] += 1

        if not replay:
            self.counts['abs'] += 1
        self.last_abs_t = m.t
        self.abs_log.append(m)
        return True

    # ------------------------------------------------------------------ 內部
    def _snapshot(self):
        ins = self.ins
        return (ins.x.copy(), ins.P.copy(), ins.t, ins.still_since,
                None if ins.anchor is None else ins.anchor.copy(),
                ins.reject_since, ins.last_omega, ins.last_acc,
                dict(ins.counts))

    def _restore(self, snap):
        if snap is None:
            return
        ins = self.ins
        (ins.x, ins.P, ins.t, ins.still_since, ins.anchor,
         ins.reject_since, ins.last_omega, ins.last_acc, counts) = (
            snap[0].copy(), snap[1].copy(), snap[2], snap[3],
            None if snap[4] is None else snap[4].copy(),
            snap[5], snap[6], snap[7], dict(snap[8]))
        ins.counts = counts

    def _trim(self, now: float):
        """丟掉視窗外的東西。

        **兩個條件都要, 而且時間要用絕對值。** 原本只寫 `now - t > horizon`,
        時戳一旦跑到「未來」(時鐘沒對好) 那個式子永遠是負的, 緩衝區就再也不會
        被清空 —— 而每次倒帶都會把 `abs_log` 裡的每一則重放一次。實測 (相機時鐘
        偏 625 秒): abs_log 長到 2702 筆, 同一批量測被重複套用幾百次, P 被壓垮到
        0.4 mm, 估計凍結在起點, 位置 RMS 1.35 cm -> 319 cm。

        筆數上限是第二道保險: 就算時間判斷再出什麼錯, 緩衝區也不會無限長大。
        """
        max_n = self.max_buf
        while self.buf and (abs(now - self.buf[0].t) > self.horizon
                            or len(self.buf) > max_n):
            self.buf.popleft()
        while self.abs_log and (abs(now - self.abs_log[0].t) > self.horizon
                                or len(self.abs_log) > max_n):
            self.abs_log.popleft()

    # ------------------------------------------------------------------
    def report(self) -> str:
        c = self.counts
        return (f"絕對{c['abs']} 倒帶{c['rewind']}({c['replay']}步) "
                f"閘門擋{c['gate']} 限幅{c['damped']} 強制{c['forced']} "
                f"太舊{c['too_old']}"
                + (f" **未來{c['future']}**" if c['future'] else ''))
