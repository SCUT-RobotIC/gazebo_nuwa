#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手动往 stm32/write 发一个动作包，测试仿真下位机 / 真机流程。

和 stm32_control 发的是同一个协议（64 字节，字段偏移见 serial_packet.hpp）。
用途：
  - 没有跑 stm32_control 时，单独测仿真下位机能不能开车
  - 调试某个动作码的行为

用法：
    python3 tools/send_action.py 0x50 --x 1.0 --y 0.5          # 去 (1.0, 0.5) m
    python3 tools/send_action.py 0x50 --x -1.5 --y 0           # 去 (-1.5, 0)
    python3 tools/send_action.py 0x00                          # 停车
    python3 tools/send_action.py 0x61 --x 1.15 --y -3.5 --heading 0
"""
import argparse
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

HEAD = (0xAA, 0xAA)
TAIL = (0xBB, 0xBB)
SEND_LEN = 64


def build(action, x=0.0, y=0.0, heading=0, stair=0, lift=0):
    """按 serial_packet.hpp 的 serialize_packet 组包（int16 小端，x/y 单位【毫米】）"""
    b = bytearray(SEND_LEN)
    b[0], b[1] = HEAD
    struct.pack_into('<h', b, 2, action)
    # 4-7 是 x_real / y_real（上位机填的"我在哪"），手动测试时留 0
    struct.pack_into('<h', b, 8, int(round(x * 1000.0)))    # ⚠ 米 → 毫米
    struct.pack_into('<h', b, 10, int(round(y * 1000.0)))
    struct.pack_into('<h', b, 12, heading)
    struct.pack_into('<h', b, 14, stair)
    struct.pack_into('<h', b, 36, lift)
    b[62], b[63] = TAIL
    return b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('action', help='动作码，十六进制，例如 0x50')
    ap.add_argument('--x', type=float, default=0.0, help='目标 x（米）')
    ap.add_argument('--y', type=float, default=0.0, help='目标 y（米）')
    ap.add_argument('--heading', type=int, default=0, help='0前/1左/2后/3右')
    ap.add_argument('--stair', type=int, default=0, help='0上台阶/1下台阶')
    ap.add_argument('--lift', type=int, default=0, help='抬升高度 mm（200/400）')
    ap.add_argument('--topic', default='stm32/write')
    ap.add_argument('--repeat', type=float, default=2.0, help='重复发多少秒（真机是周期发的）')
    ap.add_argument('--rate', type=float, default=20.0, help='发送频率 Hz')
    a = ap.parse_args()

    action = int(a.action, 0)
    pkt = build(action, a.x, a.y, a.heading, a.stair, a.lift)

    rclpy.init()
    n = rclpy.create_node('send_action')
    pub = n.create_publisher(UInt8MultiArray, a.topic, 10)
    time.sleep(0.5)      # 等订阅方连上

    m = UInt8MultiArray()
    m.data = list(pkt)

    n.get_logger().info(
        '发动作 0x%02X → 目标 (%.3f, %.3f) m  heading=%d  发 %.1f 秒 @ %.0f Hz'
        % (action, a.x, a.y, a.heading, a.repeat, a.rate))
    n.get_logger().info('  十六进制: %s' % ' '.join('%02X' % v for v in pkt))

    t0 = time.time()
    while time.time() - t0 < a.repeat:
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0)
        time.sleep(1.0 / a.rate)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
