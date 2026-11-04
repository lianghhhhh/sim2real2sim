#!/usr/bin/env python3
"""多相機 + LiDAR 放進同一個節點才會出現的事 —— 不需要 ROS / Isaac / YOLO。

    python3 test/test_all_sensors.py

`test_fusion.py` 測濾波器, `car_loc_multicam/test/test_multicam.py` 測框的量測模型,
這支測**兩個來源在節點裡怎麼相處**: 誰給起點、其中一個給錯了救不救得回來、
launch 疊出來的參數對不對。真資料上的 A/B 見 `replay_all_sensors.py`。

[8]~[13] 是 2026-11-03 那一輪 (相機與 LiDAR 一起斷了 51 秒) 之後補的:
時間倒退、~/reset、abs_age、停車時的錨點、逃生口的計時、重放時的 use_yaw。
"""
from __future__ import annotations

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import replay_all_sensors as R                     # noqa: E402 (順便把假的 ROS 裝好)

DT = 1.0 / 60.0
QUIET = R._ns(info=lambda m: None, warn=lambda m: None, error=lambda m: None)


class Sim:
    """車子停在 `truth` (或照它走), 餵 IMU + 輪速 + 框 + LiDAR 給真正的節點。"""

    def __init__(self, truth, **over):
        self.clock = [0.0]
        R.fusion_node.time.monotonic = lambda: self.clock[0]
        self.n = R.make_node(**over)
        self.n.get_logger = lambda: QUIET
        self.truth = truth
        self.t = 0.0
        self.i = 0

    def _imu(self, yaw):
        r = R._ns(stamp=self.t, gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
                  acc_x=0.0, acc_y=0.0, acc_z=9.81)
        return R.imu_msg(r, yaw)

    def _joint(self):
        r = R._ns(stamp=self.t, front_left_velocity=0.0, front_right_velocity=0.0,
                  rear_left_velocity=0.0, rear_right_velocity=0.0)
        return R.joint_msg(r)

    def _boxes(self):
        x, y, yaw = self.truth(self.t)
        for c in self.n.cams:
            b, _ = self.n.car.predict_bbox(c, x, y, yaw, jac=False)
            if b is None:
                continue
            m = R._gm.PolygonStamped()
            R._stamp(m.header.stamp, self.t)
            m.header.frame_id = c.name
            m.polygon.points += [R._gm.Point32(x=b[0], y=b[1], z=0.8),
                                 R._gm.Point32(x=b[2], y=b[3], z=0.0)]
            cb = self.n.subs.get(c.detection_topic)
            if cb is not None:
                cb(m)

    def _lidar(self, pose):
        x, y, yaw = pose
        m = R.lidar_msg(R._ns(lid_stamp=self.t, lid_x=x, lid_y=y, lid_yaw=yaw, lid_sigma=0.0012))
        self.n.on_abs(m, 'lidar')

    def run(self, dur, *, cams=True, lidar=None):
        """`lidar(t) -> (x, y, yaw)` 或 None。"""
        for _ in range(int(dur / DT)):
            self.clock[0] = self.t
            self.n.flush()
            self.n.on_joint(self._joint())
            self.n.on_imu(self._imu(self.truth(self.t)[2]))
            if cams and self.i % 4 == 0:
                self._boxes()
            if lidar is not None and self.i % 6 == 0:
                self._lidar(lidar(self.t))
            self.t += DT
            self.i += 1

    def err(self):
        x, y, _ = self.truth(self.t)
        return math.hypot(self.n.ekf.pos[0] - x, self.n.ekf.pos[1] - y)


STILL = lambda t: (1.0, 0.5, 0.3)                  # noqa: E731
WRONG = lambda t: (4.0, 0.5, 0.3)                  # noqa: E731   差 3 m, 位置看起來很合理


def case1():
    print('[1] 兩個來源都在 -> 起點用多相機, 不是先到的 LiDAR')
    s = Sim(STILL, lidar_enabled=True)
    s.run(3.0, lidar=WRONG)
    assert s.n.ekf.initialized, '三秒了還沒有起點'
    print(f'     LiDAR 差 3 m 而且先到, 起點誤差 {s.err() * 100:.2f} cm')
    assert s.err() < 0.1, 'LiDAR 搶到起點了'


def case2():
    print('[2] 沒有相機 (YOLO 暖機 / stage 沒有環境相機) -> 等 init_lidar_wait 之後用 LiDAR')
    s = Sim(STILL, lidar_enabled=True)
    s.run(4.0, cams=False, lidar=STILL)
    assert not s.n.ekf.initialized, '等待期內不該用 LiDAR 當起點'
    s.run(4.0, cams=False, lidar=STILL)
    assert s.n.ekf.initialized and s.err() < 0.05, 'LiDAR 沒有接手給起點'
    print(f'     t={s.t:.1f} s 由 LiDAR 給起點, 誤差 {s.err() * 100:.2f} cm')


def case3():
    print('[3] 狀態錯得離譜 (LiDAR 給錯起點後掉線) -> 相機要救得回來')
    # 這是挑框用預測的副作用: 每台都挑不到 -> 框進不了 EKF -> 閘門計時不開始 -> 逃生口不觸發
    s = Sim(STILL, lidar_enabled=True)
    s.run(8.0, cams=False, lidar=WRONG)
    e0 = s.err()
    assert e0 > 2.5, f'這個情境的前提不成立 (誤差 {e0:.2f} m)'
    s.run(10.0)
    print(f'     錯 {e0:.2f} m -> 相機回來 10 秒後 {s.err() * 100:.2f} cm '
          f"(強制 {s.n.ekf.counts['forced']}, 挑框沒中 {s.n.n_assoc_miss})")
    assert s.err() < 0.1, f'救不回來 ({s.err():.2f} m)'


def case4():
    print('[4] 鎖到對稱解的 LiDAR 拖不走多相機')
    s = Sim(STILL, lidar_enabled=True)
    s.run(3.0)
    s.run(8.0, lidar=WRONG)
    print(f"     誤差 {s.err() * 100:.2f} cm, LiDAR 閘門擋 {s.n.ekf.counts['gate']} 則")
    assert s.err() < 0.1, 'LiDAR 把估計拖走了'


def case5():
    print('[5] 開關: multicam_enabled:=false 不訂閱框, 起點由 LiDAR 馬上給')
    s = Sim(STILL, lidar_enabled=True, multicam_enabled=False)
    assert not any(t.endswith('car_bbox') for t in s.n.subs), '還在訂閱框'
    s.run(2.5, lidar=STILL)
    assert s.n.ekf.initialized and s.err() < 0.05
    s.n.status()
    print(f'     訂閱 {sorted(s.n.subs)}')


def case6():
    print('[6] 車子慢慢蠕動但輪子不動 -> 估計要跟上, 不能凍在原地')
    # Isaac 怠速蠕動 -0.75 mm/s, 這裡加快 8 倍省時間。2026-11-02 線上: 凍 46 cm, 強制 6040 次
    creep = lambda t: (1.0 - 0.006 * max(t - 5.0, 0.0), 0.5, 0.3)    # noqa: E731
    s = Sim(creep, lidar_enabled=True)
    s.run(65.0, lidar=creep)
    print(f"     蠕動 {0.006 * 60 * 100:.0f} cm 之後誤差 {s.err() * 100:.2f} cm, "
          f"擋 {s.n.ekf.counts['gate']}, 強制 {s.n.ekf.counts['forced']}")
    assert s.err() < 0.02, '估計沒跟上 (靜止錨點把絕對量測的修正拉回去了)'


def case7():
    print('[7] launch 疊出來的參數')
    p = R.yaml_params()
    assert p['odom_topic'] == '/fusion_loc/odom' and p['pose_topic'] == '/fusion_loc/pose'
    assert p['marker_topic'] == '/fusion_loc/markers'
    assert p['abs_age_topic'] == '/fusion_loc/abs_age'
    assert p['lidar_enabled'] is True and p['camera_enabled'] is False
    assert p['bbox_sigma_px'] > 0.0 and p['multicam_enabled'] is True
    print('     輸出在 /fusion_loc/*, LiDAR 開、天花板相機關, 框的參數沿用方法六')


def case8():
    print('[8] Isaac Stop/Play (時間倒退) -> 自動重設, 不能凍在原地')
    s = Sim(STILL, lidar_enabled=True)
    s.run(5.0, lidar=STILL)
    assert s.n.ekf.initialized
    s.t = 1.0                                       # /clock 從頭開始
    s.run(5.0, lidar=STILL)
    lag = s.t - s.n.ekf.ins.t
    print(f'     倒退之後 5 秒: 濾波器時間落後 {lag:.3f} s, 誤差 {s.err() * 100:.2f} cm')
    assert s.n.ekf.initialized and abs(lag) < 0.1 and s.err() < 0.05, '凍住了'


def case9():
    print('[9] 開機 12 秒後 ~/reset -> 要重新做靜止校正, 不是直接判逾時')
    s = Sim(STILL)
    s.run(12.0)
    res = R._ns(success=False, message='')
    s.n.on_reset(None, res)
    s.run(0.5)
    assert not s.n.started, '重設之後 0.5 秒就開始了 = 沒有重新校正 (t0 沒清)'
    s.run(2.0)
    assert s.n.started and s.n.ekf.initialized and s.err() < 0.05
    print(f'     重設 0.5 s 時還在校正, 2.5 s 時重新有起點 (誤差 {s.err() * 100:.2f} cm)')


def case10():
    print('[10] abs_age: 絕對量測斷了多久')
    s = Sim(STILL, lidar_enabled=True)
    ages = []
    s.n.pub_age.publish = lambda m: ages.append(m.data)
    s.run(4.0, lidar=STILL)
    a_on = max(ages[-60:])
    s.run(3.0, cams=False)
    print(f'     有量測時最大 {a_on:.3f} s, 全斷 3 秒後 {ages[-1]:.2f} s')
    assert a_on < 0.2 and 2.8 < ages[-1] < 3.1


def case11():
    print('[11] 停著而且絕對量測全斷 -> sigma 不能縮 (錨點只知道沒動, 不知道在哪)')
    s = Sim(STILL, lidar_enabled=True)
    s.run(4.0, lidar=STILL)
    sig_on = s.n.ekf.sigma_pos()
    s.run(10.0, cams=False)
    sig_off = s.n.ekf.sigma_pos()
    print(f'     有量測 {sig_on * 100:.2f} cm -> 斷 10 秒 {sig_off * 100:.2f} cm')
    assert sig_on < 0.02, '有相機的時候 P 應該被壓小'
    assert sig_off >= 0.99 * sig_on, '錨點把 P 壓得比斷線前還小'


def case12():
    print('[12] 逃生口: 中間沒有量測的時間不算「一直被擋」')
    from car_loc_fusion.fusion_ekf import AbsMeas, FusionEkf, Step
    e = FusionEkf(rewind=False)
    e.set_pose(0.0, 0.0, 0.0, t=0.0, sigma_p=0.02)
    t = 0.0

    def until(t_end, meas_every=None, x=3.0):
        nonlocal t
        k = 0
        while t < t_end:
            t += DT
            k += 1
            e.step(Step(t, 0.0, 0.0, v_wheel=0.0))
            if meas_every and k % meas_every == 0:
                e.absolute(AbsMeas(t, 'lidar', x, 0.0, 0.03))
    until(1.0)
    e.absolute(AbsMeas(t, 'lidar', 3.0, 0.0, 0.03))      # 被擋, 開始計時
    until(11.0)                                          # 10 秒沒有量測 (斷線)
    e.absolute(AbsMeas(t, 'lidar', 3.0, 0.0, 0.03))      # 斷完的第一則, 一樣是錯的
    print(f"     斷線前被擋一則, 10 秒後又一則: 強制 {e.counts['forced']}")
    assert e.counts['forced'] == 0, '斷線的時間被算成一直被擋, 錯的量測被硬吃進去'
    until(15.0, meas_every=6)                            # 10 Hz 連續 4 秒都對不上
    print(f"     接著 10 Hz 連續 4 秒對不上: 強制 {e.counts['forced']} (逃生口照常)")
    assert e.counts['forced'] > 0


def case13():
    print('[13] use_yaw=False 的量測倒帶重放時不能吃 yaw')
    from car_loc_fusion.fusion_ekf import AbsMeas, FusionEkf, Step
    e = FusionEkf()
    e.set_pose(0.0, 0.0, 0.0, t=0.0, sigma_p=0.05, sigma_yaw=0.05)
    t = 0.0
    for _ in range(30):
        t += DT
        e.step(Step(t, 0.0, 0.0, v_wheel=0.0))
    e.absolute(AbsMeas(t - 0.05, 'camera', 0.0, 0.0, 0.03, yaw=0.03), use_yaw=False)
    e.absolute(AbsMeas(t - 0.10, 'lidar', 0.0, 0.0, 0.04), use_yaw=False)   # 觸發倒帶
    print(f'     量測 yaw 1.7°, 倒帶重放之後濾波器 yaw {math.degrees(e.yaw):.4f}°')
    assert abs(e.yaw) < 1e-6


def main():
    for f in (case1, case2, case3, case4, case5, case6, case7, case8, case9, case10,
              case11, case12, case13):
        f()
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
