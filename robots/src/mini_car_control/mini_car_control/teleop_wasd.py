#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WASD 键盘遥控（步进式）—— 专为舵轮底盘设计。

用法：
    ros2 run mini_car_control teleop_wasd

行为是【按一下动一下】：
    按一次 w  →  前进一小段（默认 0.25 秒），然后自动停
    按住不放  →  键盘重复会让它一段接一段地走（等于连续移动）
    松开      →  停下

这样适合精细对位（开到场地上某个点、对着建塔处），
而且松手一定会停，不会像"按一下一直跑"那样失控。
"""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = """
╔══════════════════════════════════════════════════════╗
║   WASD 遥控（舵轮底盘 / 步进式）                      ║
╠══════════════════════════════════════════════════════╣
║     w / s      前进 / 后退                            ║
║     a / d      左移 / 右移      ← 舵轮的招牌动作      ║
║     q / e      左转 / 右转                            ║
║                                                      ║
║     按一下 = 动一小段；按住不放 = 连续走              ║
║     空格       立即停车                               ║
║     + / -      调快 / 调慢                            ║
║     [ / ]      调短 / 调长每次走的时长                ║
║     h          重新显示这个帮助                       ║
║     Ctrl+C     退出（会自动停车）                     ║
╚══════════════════════════════════════════════════════╝
"""


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ⚠ 0.50 m/s 是 2026-09-29 按用户要求从 0.30 调快的。
        #   注意场地的实际限制：能安全跑的净空只有南/北两条 0.8 m 宽的条带
        #   （轮子半径只有 5 cm，围栏 8 cm、坡道侧壁 60 cm 都是过不去的墙），
        #   车半宽 0.35 m ⇒ 中心只有 ±0.4 m 余量。
        #   一次按键走 speed×burst = 0.5×0.25 = 12.5 cm，还够控制，
        #   但别按住不放（按键连发会变成持续走）✗
        self.speed = 0.50      # 线速度 m/s
        self.turn = 0.60       # 角速度 rad/s
        self.burst = 0.25      # 每按一次键走多久（秒）
        self.delta = 0.10      # 调速度的步长

        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.move_until = 0.0  # 走到这个时刻就停

        # 20 Hz 发布：走的时候发指令，不走的时候发 0
        self.create_timer(0.05, self.publish_cmd)

    def publish_cmd(self):
        m = Twist()
        if time.monotonic() < self.move_until:
            m.linear.x = self.vx
            m.linear.y = self.vy
            m.angular.z = self.w
        self.pub.publish(m)

    def show(self):
        left = max(0.0, self.move_until - time.monotonic())
        sys.stdout.write(
            '\r  前进 %+.2f  左移 %+.2f  左转 %+.2f  [速度 %.2f  步长 %.2fs  剩余 %.2fs]   '
            % (self.vx, self.vy, self.w, self.speed, self.burst, left))
        sys.stdout.flush()

    def on_key(self, k):
        now = time.monotonic()
        if k == 'w':
            self.vx, self.vy, self.w = self.speed, 0.0, 0.0
        elif k == 's':
            self.vx, self.vy, self.w = -self.speed, 0.0, 0.0
        elif k == 'a':
            self.vx, self.vy, self.w = 0.0, self.speed, 0.0
        elif k == 'd':
            self.vx, self.vy, self.w = 0.0, -self.speed, 0.0
        elif k == 'q':
            self.vx, self.vy, self.w = 0.0, 0.0, self.turn
        elif k == 'e':
            self.vx, self.vy, self.w = 0.0, 0.0, -self.turn
        elif k == ' ':
            self.vx = self.vy = self.w = 0.0
            self.move_until = 0.0            # 立即停
            self.show()
            return
        elif k in '+=':
            self.speed = min(self.speed + self.delta, 2.0)
            self.show()
            return
        elif k in '-_':
            self.speed = max(self.speed - self.delta, 0.05)
            self.show()
            return
        elif k == '[':
            self.burst = max(self.burst - 0.05, 0.05)
            self.show()
            return
        elif k == ']':
            self.burst = min(self.burst + 0.05, 2.0)
            self.show()
            return
        elif k == 'h':
            print(HELP)
            return
        else:
            return                            # 不认识的键，忽略

        # 运动键：走一小段（键盘重复会不断刷新这个时刻 → 按住就等于连续走）
        self.move_until = now + self.burst
        self.show()

    def stop(self):
        self.move_until = 0.0
        self.publish_cmd()


def main():
    rclpy.init()
    node = WasdTeleop()
    print(HELP)
    node.show()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)                     # 单键模式，不用敲回车
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0)[0]:
                node.on_key(sys.stdin.read(1))
            node.show()
    except KeyboardInterrupt:
        pass
    finally:
        # 【必须有】退出前把终端恢复原样，否则终端会一直停在单键模式 ✗
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.stop()
        print('\n已退出（车已停车）')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
