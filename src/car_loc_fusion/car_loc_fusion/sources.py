#!/usr/bin/env python3
"""每一個**絕對量測來源**的收件政策 —— 在進濾波器之前先擋掉哪些。

這個模組跟 ROS 無關, 也跟濾波器無關: 它只回答一個問題 ——

    「這一則 pose 該不該用, 該給多大的 R?」

為什麼要有這一層 (而不是全部丟給卡方閘門)
------------------------------------------
卡方閘門是拿量測跟**濾波器自己的信念**比, 所以它有兩個結構性的盲點, 兩個都在
這個 repo 裡量到過:

1. **P 一大, S 就跟著大, NIS 反而變小。** 「合理但巨大」的修正它完全沒意見 ——
   `car_loc_imu` 實測過單步搬走 27 公尺, 而同一步車子只走了 4 公分。
2. **濾波器自己錯掉的時候, 閘門會擋掉正確的量測。** 而且它會一直擋
   (`car_loc_wheel` 實測: 0.5 m/s 的落差要等 P 長 14 秒才放行)。

所以這一層做的三件事全都**不看濾波器狀態** —— 它們是獨立的判斷, 只看來源自己
報了什麼:

| 防線 | 擋什麼 | 依據 |
| --- | --- | --- |
| `sigma_max` | 來源**自己知道**這一幀不準 | 節點回報的 pose 共變異數 |
| `min_dt` | 同一份資訊被重複計入 | 來源的輸出本身已經濾波過 (見下) |
| `timeout` | 節點掛掉之後最後一個值被一路重放 | 訊息時戳 |

`sigma_max`: 追丟偵測用來源自己的 sigma, 不要自己猜
-----------------------------------------------------
`car_loc_lidar` 在長方形房間裡會鎖到 180 度的對稱解, 鎖住之後**位置看起來還很
合理、殘差也很漂亮, 但回不來**。`collect_data_node` 的 docstring 記了實測值:

    正常 sigma ~0.0015,  追丟 ~0.0145   (門檻 0.0025, 三輪資料 0 漏網)

這比「用角速度濾掉自旋段」準得多 —— 車子停下來之後還是鎖著的, 角速度濾不掉。

時鐘: 來源的時戳跟濾波器**不一定在同一個時鐘上**
--------------------------------------------------
2026-09-10 那一輪錄到的實際情況: `/rgb` 的 header.stamp 比 `/imu`、`/odom`、
`/scan` **早 625.55 秒** (std 只有 10 ms —— 是固定的時鐘基準差, 不是抖動)。
`car_loc_camera` 忠實地把影像時戳傳下去, 所以 `/camera_loc/odom` 也整段偏 625 秒。

這對融合是**致命**的, 而且症狀完全不像時鐘問題:

* 時戳在「未來」-> 倒帶算出來的 lag 是負的 -> 延遲補償整段被跳過
* 緩衝區的過期判斷 (`now - t > horizon`) 永遠是負的 -> **量測永遠不會被丟掉**
  -> 每次倒帶重放都把累積的整段歷史再套用一次 -> 同一筆資料被算了幾百次
  -> P 被壓垮 -> 之後所有量測都被卡方閘門擋掉 -> **估計凍結在起點**
* 逃生口的計時 (`m.t - reject_since`) 跨來源比較, 兩個時鐘一減就是幾百秒

實測 (test/test_fusion.py 注入 625.55 s 的偏移): RMS 1.35 cm -> **319 cm**,
濾波器自己報的 sigma 13.13 mm -> 4.37 mm (它「非常確定」自己凍在原地)。

所以每個來源都自己量一個**時鐘偏移**: 前 `clock_samples` 筆的
`median(量測時戳 - 濾波器時刻)`, 超過 `clock_max_offset` 就認定是時鐘基準差,
鎖定那個值並且從此扣掉, 同時大聲警告。

> **但它只救得回「不要壞掉」, 救不回精度。** 時鐘一偏, 真正的延遲 (相機那條量
> 出來是 79.5 ms) 就**沒有辦法從資料裡分離出來** —— 扣掉偏移之後每一則量測看
> 起來都像「剛剛才發生」, 延遲補償等於沒有。要拿回那 v x 80 ms, **只能去源頭
> 把時鐘修好** (讓 `/rgb` 跟 `/clock` 同一個基準), 或是用 `extra_delay` 手動把
> 量出來的那個常數延遲加回去。

`min_dt`: 相機與 LiDAR 的輸出**已經是濾波過的**, 不是獨立量測
---------------------------------------------------------------
`/camera_loc/odom` 是等速卡爾曼濾波的輸出, `/lidar_loc/odom` 也帶著自己的平滑。
連續兩則之間的誤差**是相關的**, 而 EKF 的更新式假設每一筆量測的誤差互相獨立 ——
30 Hz 全部吃進來, 等於把同一份資訊算了好幾次, P 被壓到比真實精度樂觀得多
(過度自信的濾波器接下來就會開始擋掉正確的量測, 那是同一個病的第二期)。

兩道處理, 兩道都保守:
  * `min_dt`   —— 每個來源最多用到這個頻率 (相機 10 Hz 就夠了, 它自己 30 Hz)
  * `r_inflate`—— 把來源回報的 sigma 乘上一個係數再當 R

**這兩個值不是「調到誤差最小」調出來的, 是為了讓 P 誠實。** 判斷有沒有調對要看
NEES / 卡方擋掉的比例, 不是只看 RMS —— 過度自信的濾波器在乾淨資料上 RMS 會**更
好看**, 代價是遇到離群值時完全沒有防禦。
"""
from __future__ import annotations

import math
from collections import deque


class AbsSource:
    """一個絕對位姿來源 (相機 / LiDAR / 任何會發 map 座標 pose 的東西)。"""

    def __init__(self, name: str, *,
                 enabled: bool = True,
                 sigma_floor: float = 0.02,   # R 的下限 (m), 防止來源回報 0
                 sigma_max: float = 0.0,      # 超過這個就丟掉 (0 = 不擋)
                 r_inflate: float = 1.5,      # sigma 乘這個才當 R
                 min_dt: float = 0.0,         # 兩次採用之間至少隔這麼久 (s)
                 timeout: float = 0.5,        # 這麼久沒收到就當它掛了
                 use_yaw: bool = False,       # 要不要吃它的 yaw
                 yaw_sigma: float = 0.05,     # yaw 量測雜訊 (rad)
                 yaw_max_err: float = 0.0,    # yaw 殘差超過這個就只用位置 (rad)
                 # 時鐘偏移偵測 (見模組說明)
                 clock_max_offset: float = 1.0,   # 超過這個就認定是時鐘基準差 (s)
                 clock_future_tol: float = 0.1,   # 時戳持續比濾波器新超過這個 = 時鐘不對
                 clock_samples: int = 20,         # 拿幾筆來量
                 extra_delay: float = 0.0):       # 手動補回來的常數延遲 (s)
        self.name = name
        self.enabled = bool(enabled)
        self.sigma_floor = float(sigma_floor)
        self.sigma_max = float(sigma_max)
        self.r_inflate = float(r_inflate)
        self.min_dt = float(min_dt)
        self.timeout = float(timeout)
        self.use_yaw = bool(use_yaw)
        self.yaw_sigma = float(yaw_sigma)
        self.yaw_max_err = float(yaw_max_err)
        self.clock_max_offset = float(clock_max_offset)
        self.clock_future_tol = float(clock_future_tol)
        self.clock_samples = int(clock_samples)
        self.extra_delay = float(extra_delay)

        # 時鐘偏移: 量一次就**鎖定**, 不要持續更新 —— 持續更新的話它會把真正的
        # 延遲 (量測比現在舊多少) 一起吃掉, 最後每一則量測都變成「剛剛才發生」。
        self._clock_buf = deque(maxlen=self.clock_samples)
        self.clock_offset = 0.0
        self.clock_locked = False
        self.clock_flagged = False   # 有沒有大到要警告 (給節點印訊息用)

        self.last_used = None       # 上一次**採用**的量測時戳 (已修正時鐘)
        self.last_recv = None       # 上一次**收到**的量測時戳 (已修正時鐘)
        self.counts = {'used': 0, 'rate': 0, 'sigma': 0, 'gate': 0,
                       'old': 0, 'future': 0, 'yaw_used': 0, 'yaw_skip': 0}

    # ------------------------------------------------------------------
    def to_filter_clock(self, t: float, filter_t) -> float:
        """把來源的時戳換到濾波器的時鐘上。

        前 `clock_samples` 筆先量 `median(t - filter_t)`, 兩個條件**任一個**成立
        就認定是時鐘基準差, 鎖定那個值並從此扣掉:

          * `|median| > clock_max_offset` —— 差得離譜 (實測 /rgb 差 625 秒)
          * `median > clock_future_tol`  —— **時戳持續落在未來**

        第二個條件比第一個重要, 而且跟差多少無關: **真正的延遲只會讓時戳變舊,
        不會讓它變新。** 所以「持續比濾波器新」本身就是時鐘不對的證明, 差 0.5 秒
        跟差 625 秒是同一件事。(實測: 只用第一個條件的話, 偏 0.5 秒那一組雖然不會
        凍結, 但每一則量測都走「未來」那條保險路徑, 延遲補償全程沒有作用,
        位置 RMS 1.35 -> 19.33 cm。)

        容許值不能給 0: 量測的時戳本來就可能比最後一則 IMU 稍新幾毫秒
        (IMU 60 Hz = 16 ms)。

        鎖定之後就**不再更新** —— 持續更新會把真正的延遲也吃掉。
        """
        if not self.clock_locked:
            if filter_t is None:
                return t - self.extra_delay
            self._clock_buf.append(t - float(filter_t))
            if len(self._clock_buf) >= self.clock_samples:
                v = sorted(self._clock_buf)
                med = v[len(v) // 2]
                if abs(med) > self.clock_max_offset or med > self.clock_future_tol:
                    self.clock_offset = med
                    self.clock_flagged = True
                self.clock_locked = True
        return t - self.clock_offset - self.extra_delay

    def alive(self, now: float) -> bool:
        return (self.last_recv is not None
                and (now - self.last_recv) < self.timeout)

    def sigma_for(self, reported: float) -> float:
        """來源回報的 1-sigma -> 這一筆該用的量測 sigma。

        下限是必要的: 節點回報 0 (沒填共變異數) 的話 R 會是奇異的, 一次更新就
        把狀態整個搬過去, 而且之後 P 幾乎為 0, 任何別的量測都再也修不動。
        """
        s = float(reported) if reported and math.isfinite(reported) else 0.0
        return max(s * self.r_inflate, self.sigma_floor)

    def accept(self, t: float, sigma: float, now: float):
        """要不要用這一筆。回傳 (ok, 理由)。理由只在被擋時有意義。

        **這三道都不看濾波器狀態** —— 它們是獨立於估計的判斷, 所以濾波器再怎麼
        迷路都不會影響它們 (反過來說: 濾波器錯掉的時候, 這三道仍然會放行正確的
        量測, 這正是它跟卡方閘門的分工)。
        """
        self.last_recv = t
        if self.sigma_max > 0.0 and sigma > self.sigma_max:
            self.counts['sigma'] += 1
            return False, 'sigma'
        if (self.min_dt > 0.0 and self.last_used is not None
                and (t - self.last_used) < self.min_dt):
            self.counts['rate'] += 1
            return False, 'rate'
        return True, ''

    def mark_used(self, t: float):
        self.last_used = t
        self.counts['used'] += 1

    def report(self, now: float) -> str:
        if not self.enabled:
            return f'{self.name}=off'
        if self.last_recv is None:
            return f'{self.name}=無訊息'
        if not self.alive(now):
            return f'{self.name}=斷線({now - self.last_recv:.0f}s)'
        c = self.counts
        extra = ''
        if c['sigma'] or c['gate'] or c['future']:
            extra = (f" (sigma擋{c['sigma']} 閘門擋{c['gate']}"
                     + (f" 未來{c['future']}" if c['future'] else '') + ')')
        clk = f' 時鐘{self.clock_offset:+.1f}s' if self.clock_offset else ''
        return f"{self.name}={c['used']}{extra}{clk}"
