#!/usr/bin/env python3
"""方法 C: 每台相機的外接框各自當成 EKF 的量測, 量測模型是「視線」。

這個模組跟 ROS 無關, 所以 `test/test_multicam.py` 不需要 ROS / Isaac 就能跑。

跟 car_loc_fusion 的關係
------------------------
**遞推、倒帶重放、逃生口全部直接用 car_loc_fusion.FusionEkf** (它又是用
car_loc_wheel.WheelIns)。這裡只多一種量測: `BboxMeas`。同一段程式碼只能有一份 ——
抄一份過來改的話, 兩邊的修正會慢慢分岔。

量測模型: 為什麼說是「視線」
----------------------------
car_loc_fusion 吃的是 (x, y) + 一個圓形的 sigma —— 「車在這裡, 各方向一樣準」。
斜拍的相機不是這樣:

    * 框的**左右邊** = 從相機看過去車的左右緣的方位角 -> 垂直於視線的方向非常準
      (5 m 外 1 px = 6 mm)
    * 框的**上下邊** = 俯角 -> 透過「車在地上」換成距離, 沿著視線的方向差很多
      (5 m 外、俯角 20 度時 1 px = 2~3 cm, 而且車高、車的朝向都會混進來)

所以一台相機給的資訊是一個**沿著視線拉長的橢圓**, 不是圓。把它壓成圓形 sigma
(car_loc_fusion 的 AbsMeas) 只有兩條路: 用短軸 -> 過度自信, 沿視線方向的誤差
被硬吃進去; 用長軸 -> 垂直方向的好資訊被丟掉。

這裡直接把框的四個邊 (像素) 當量測, 量測函數 h(x, y, yaw) 就是「把車的 3D 外形
投影到這台相機」(car_model.predict_bbox)。EKF 的 H = dh/dx 自動就是那個橢圓:
左右邊的列對垂直視線方向很敏感、上下邊對沿視線方向比較敏感。**8 台相機從不同
方向看, 每一台的短軸指向不同, 疊起來就是一個小的圓** —— 這就是「不同方向的偏差
互相抵銷」在數學上的樣子。被擋住的相機就是少一個橢圓, 不會讓整個量測消失。

為什麼 yaw 要在狀態裡 (而不是像方法 B 那樣當成已知)
----------------------------------------------------
外接框的寬度與中心跟 yaw 有關。方法 B 把 yaw 當常數, yaw 差 5 度位置就跟著偏。
這裡 H 有 yaw 那一欄, 所以:
  * yaw 的不確定性會正確地傳到位置上 (P 裡的相關項)
  * 多台相機的框反過來也會修 yaw (真車沒有絕對 yaw 時這一點很有用)
Isaac 裡 yaw 有 IMU 的絕對姿態 (0.07 度) 壓著, 相機那一欄幾乎動不了它, 那也沒關係。

一則 BboxMeas = 同一時刻的所有相機
----------------------------------
Isaac 的相機是同一幀 render 的, 時戳一模一樣 —— 8 台各自倒帶一次是 8 倍的浪費。
節點會把時戳相同 (`group_tol` 以內) 的框綁成一則, 倒帶一次、每台相機依序更新。
實體相機各自拍 (只同步時鐘, 不同步快門) 的話, 每一則就只有一台, 照樣能用。

閘門與逃生口: 每台相機各自判
----------------------------
一台相機認錯 (影子、別的東西) 不該拖累另外七台, 所以卡方閘門是**每台相機各自**
判的 (自由度 = 那台能用的邊數)。逃生口沿用 FusionEkf 的狀態 (`reject_since`,
`recovering`): 連續 `abs_reject_time` 秒**所有**相機都被擋, 就進入恢復模式。
重放時照當初的決定 (`BboxObs.accepted` / `r_scale` / `mask`) 重做, 不重新判 ——
理由見 FusionEkf._abs_update 裡那段「倒帶重放的鐵律」。
"""
from __future__ import annotations

import os
import sys

import numpy as np

try:
    from car_loc_fusion.fusion_ekf import FusionEkf
    from car_loc_wheel.wheel_ins import IP, IYAW, NX, wrap_pi
except ImportError:                                          # pragma: no cover
    # 離線測試 (不裝 ROS) 時從 source tree 直接找過去
    _src = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
    sys.path.insert(0, os.path.join(_src, 'car_loc_fusion'))
    sys.path.insert(0, os.path.join(_src, 'car_loc_wheel'))
    from car_loc_fusion.fusion_ekf import FusionEkf
    from car_loc_wheel.wheel_ins import IP, IYAW, NX, wrap_pi

from .car_model import chi2_99, edge_mask, edge_sigma


# 框的時戳比濾波器新超過這麼多 (s) 就不用: 那不是延遲, 是時鐘沒對好
MAX_FUTURE = 1.0


class BboxObs:
    """一台相機在某一刻看到的車框 (已經挑過、確定是車的那一個)。"""

    __slots__ = ('cam', 'box', 'conf', 'accepted', 'r_scale', 'mask', 'nis')

    def __init__(self, cam, box, conf=1.0):
        self.cam = cam
        self.box = np.asarray(box, dtype=np.float64).reshape(4)
        self.conf = float(conf)
        # 第一次處理時的決定, 重放時照做 (見模組說明)
        self.accepted = None
        self.r_scale = 1.0
        self.mask = None
        self.nis = float('nan')


class BboxMeas:
    """同一時刻 (濾波器時鐘) 的一組車框 —— 倒帶重放的一個事件。

    `yaw = None` 是給 FusionEkf._rewind_apply 看的: 它重放 abs_log 時會用
    `ev.yaw is not None` 決定要不要吃 yaw。
    """

    __slots__ = ('t', 'src', 'obs', 'yaw')

    def __init__(self, t, obs, src='multicam'):
        self.t = float(t)
        self.src = str(src)
        self.obs = list(obs)
        self.yaw = None


class MultiCamEkf(FusionEkf):
    """FusionEkf + 外接框量測 (`BboxMeas`)。其他量測 (AbsMeas) 照原樣交給父類別。"""

    def __init__(self, model, *, sigma_px: float = 3.0, size_frac: float = 0.03,
                 border_px: float = 4.0, r_inflate: float = 1.5,
                 update_yaw: bool = True, **fusion_kwargs):
        super().__init__(**fusion_kwargs)
        self._setup(model, sigma_px, size_frac, border_px, r_inflate, update_yaw)

    def _setup(self, model, sigma_px, size_frac, border_px, r_inflate, update_yaw):
        self.model = model
        self.sigma_px = float(sigma_px)
        self.size_frac = float(size_frac)
        self.border_px = float(border_px)
        # 外接框的誤差在相鄰幀之間是相關的 (同一個 YOLO、同一個視角), 而 EKF 假設
        # 每一筆互相獨立 —— 跟 car_loc_fusion 的 camera_r_inflate 同一個理由
        self.r_inflate = float(r_inflate)
        self.update_yaw = bool(update_yaw)
        self.bbox_counts = {'obs': 0, 'used': 0, 'gate': 0, 'forced': 0,
                            'no_view': 0, 'few_rows': 0}

    @classmethod
    def adopt(cls, base: FusionEkf, model, **opts) -> 'MultiCamEkf':
        """把一個已經建好的 FusionEkf 變成 MultiCamEkf (狀態、參數全部沿用)。

        融合節點的 `_make_ekf` 有 25 個參數, 這樣就不用在這裡再抄一次。
        """
        obj = cls.__new__(cls)
        obj.__dict__.update(base.__dict__)
        obj._setup(model, opts.get('sigma_px', 3.0), opts.get('size_frac', 0.03),
                   opts.get('border_px', 4.0), opts.get('r_inflate', 1.5),
                   opts.get('update_yaw', True))
        return obj

    # ------------------------------------------------------------------
    def absolute(self, m, **kw) -> bool:
        if isinstance(m, BboxMeas) and not self.initialized:
            # 外接框沒辦法單獨當起點 (要先猜 yaw), 由節點用 solve_position 初始化
            return False
        if (isinstance(m, BboxMeas) and self.ins.t is not None
                and m.t - self.ins.t > MAX_FUTURE):
            # 父類別會把「未來的量測」直接套用並記進 abs_log —— 時鐘只差一點時那是對的,
            # 但差很多時它永遠不會過期, 每次倒帶都再套用一次。節點應該已經擋掉了, 這是第二道。
            self.counts['future'] += 1
            return False
        return super().absolute(m, **kw)

    def _abs_update(self, m, *, use_yaw: bool, max_correction=None,
                    replay: bool = False) -> bool:
        if not isinstance(m, BboxMeas):
            return super()._abs_update(m, use_yaw=use_yaw,
                                       max_correction=max_correction, replay=replay)
        lim = self.abs_max_correction if max_correction is None else max_correction
        any_ok = False
        for ob in m.obs:
            if replay and not ob.accepted:
                continue
            if self._bbox_one(ob, lim, replay):
                any_ok = True
        if not any_ok:
            return False
        if not replay:
            self.counts['abs'] += 1
        self.last_abs_t = m.t
        self.abs_log.append(m)
        return True

    def _bbox_one(self, ob: BboxObs, lim: float, replay: bool) -> bool:
        """一台相機的一次更新 (序列更新: 第二台看到的是第一台修過的狀態)。"""
        ins = self.ins
        x, y, yaw = float(ins.x[0]), float(ins.x[1]), float(ins.x[IYAW])
        pred, J = self.model.predict_bbox(ob.cam, x, y, yaw)
        if pred is None:
            if not replay:
                self.bbox_counts['no_view'] += 1
                ob.accepted = False
            return False
        if replay:
            mask = ob.mask
        else:
            self.bbox_counts['obs'] += 1
            mask = edge_mask(ob.cam, ob.box, pred, self.border_px)
            ob.mask = mask
            # 只剩 1 個邊的話資訊太少, 而且最容易是「框被裁掉」的那種錯
            if int(mask.sum()) < 2:
                self.bbox_counts['few_rows'] += 1
                ob.accepted = False
                return False

        H = np.zeros((int(mask.sum()), NX))
        H[:, 0] = J[mask, 0]
        H[:, 1] = J[mask, 1]
        if self.update_yaw:
            H[:, IYAW] = J[mask, 2]
        r = (ob.box - pred)[mask]
        sig = edge_sigma(ob.box, self.sigma_px, self.size_frac)[mask] * self.r_inflate
        R = np.diag(sig ** 2)

        if replay:
            R = R * ob.r_scale
            S = H @ ins.P @ H.T + R
        else:
            S = H @ ins.P @ H.T + R
            try:
                Si = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                ob.accepted = False
                return False
            ob.nis = float(r @ Si @ r)
            if ob.nis <= chi2_99(len(r)) * self.chi2_scale:
                self._gate_pass()
                ob.r_scale = 1.0
            else:
                # 逃生口 —— 跟 FusionEkf._abs_update 同一套規則, 共用同一組狀態
                # (所以「相機全部被擋 3 秒」跟「LiDAR 被擋 3 秒」是同一個計時器)
                tnow = ins.t if ins.t is not None else 0.0
                if self._gate_reject(tnow):
                    self.counts['gate'] += 1
                    self.bbox_counts['gate'] += 1
                    ob.accepted = False
                    return False
                ob.r_scale = 9.0
                R = R * ob.r_scale
                S = H @ ins.P @ H.T + R
                self.counts['forced'] += 1
                self.bbox_counts['forced'] += 1
        try:
            Si = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False

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
        self._shift_anchor(dx)
        if not replay:
            ob.accepted = True
            self.bbox_counts['used'] += 1
        return True

    def bbox_report(self) -> str:
        c = self.bbox_counts
        return (f"框 {c['used']}/{c['obs']} (閘門擋{c['gate']} 強制{c['forced']} "
                f"看不到{c['no_view']} 邊太少{c['few_rows']})")
