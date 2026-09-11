#!/usr/bin/env python3
"""鍵盤遙控 —— 建圖的時候手動把房間開一圈。

    ros2 run car_teleop teleop_key

        w / s      前進 / 後退
        a / d      左轉 / 右轉      (可以跟 w/s 一起按)
        空白鍵     立刻停
        + / -      調整速度上限
        [ / ]      調整轉向上限
        t          切換「按住才走 / 按一下持續走」
        q 或 Ctrl-C  離開 (離開前會先把車停下來)

預設是「按住才走」: 放開超過 hold_time 就自動停。靠的是終端機的按鍵重複,
所以按住不放才會持續送指令。如果你的終端機關掉了按鍵重複 (或是透過很慢的 ssh),
按 t 切成 latched 模式 —— 按一下就一直走, 要按空白鍵才停。

發的是標準 geometry_msgs/Twist, 所以 car_teleop 的 cmd_vel_bridge、Foxglove 的
Teleop 面板、搖桿、nav2 都可以互換。
"""
from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

HELP = """
  w/s 前進後退   a/d 左右轉   空白 停
  +/- 速度上限   [/] 轉向上限   t 切換按住/持續   q 離開
"""


class TeleopKey(Node):

    def __init__(self):
        super().__init__('teleop_key')
        p = self.declare_parameter
        p('cmd_vel_topic', '/cmd_vel')
        p('linear_speed', 0.35)       # m/s, 建圖用的慢速
        p('angular_speed', 0.6)       # rad/s
        p('publish_rate', 20.0)
        # 放開按鍵多久之後算「停」。要比終端機的按鍵重複間隔長一點,
        # 不然按住的時候會一頓一頓的。
        p('hold_time', 0.4)
        p('latched', False)

        g = self.get_parameter
        self.v_step = float(g('linear_speed').value)
        self.w_step = float(g('angular_speed').value)
        self.hold = float(g('hold_time').value)
        self.latched = bool(g('latched').value)
        self.pub = self.create_publisher(Twist, g('cmd_vel_topic').value, 10)

        self.v = self.w = 0.0
        self.t_key = -1e9
        self.dt = 1.0 / float(g('publish_rate').value)
        self.create_timer(self.dt, self.tick)
        self.create_timer(0.5, self.show)

        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        print(HELP)

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def _read_keys(self):
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            keys.append(sys.stdin.read(1))
        return keys

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for k in self._read_keys():
            if k in ('q', '\x03'):
                raise KeyboardInterrupt
            if k == ' ':
                self.v = self.w = 0.0
                self.t_key = -1e9
                continue
            if k == 't':
                self.latched = not self.latched
                self.v = self.w = 0.0
                print(f'\n  -> {"按一下持續走 (latched)" if self.latched else "按住才走"}')
                continue
            if k in '+=':
                self.v_step = min(self.v_step * 1.25, 3.0)
                print(f'\n  -> 速度上限 {self.v_step:.2f} m/s')
                continue
            if k in '-_':
                self.v_step = max(self.v_step / 1.25, 0.05)
                print(f'\n  -> 速度上限 {self.v_step:.2f} m/s')
                continue
            if k == ']':
                self.w_step = min(self.w_step * 1.25, 4.0)
                print(f'\n  -> 轉向上限 {self.w_step:.2f} rad/s')
                continue
            if k == '[':
                self.w_step = max(self.w_step / 1.25, 0.05)
                print(f'\n  -> 轉向上限 {self.w_step:.2f} rad/s')
                continue
            if k in 'wsad':
                if k == 'w':
                    self.v = self.v_step
                elif k == 's':
                    self.v = -self.v_step
                elif k == 'a':
                    self.w = self.w_step
                elif k == 'd':
                    self.w = -self.w_step
                self.t_key = now

        if not self.latched and now - self.t_key > self.hold:
            self.v = self.w = 0.0

        m = Twist()
        m.linear.x = float(self.v)
        m.angular.z = float(self.w)
        self.pub.publish(m)

    def show(self):
        mode = 'latched' if self.latched else 'hold'
        sys.stdout.write(f'\r  v={self.v:+.2f} m/s  w={self.w:+.2f} rad/s   '
                         f'[上限 {self.v_step:.2f} / {self.w_step:.2f}, {mode}]   ')
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopKey()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # 離開前把停車指令多發幾次, 免得 bridge 沒收到最後一筆就放著車子滑走
            for _ in range(10):
                node.pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.02)
            node.restore()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\n已停車。')


if __name__ == '__main__':
    main()
