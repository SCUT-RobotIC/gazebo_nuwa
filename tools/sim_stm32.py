#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真下位机 —— 在 Gazebo 里替代 STM32，让 stm32_control 的流程逻辑能原样跑。

## 为什么要这个东西

真机的链路是：

    publish_target ──> /target /target_heading /stair_direction
                              ↓
                       stm32_control         ← 上位机：流程逻辑（Flow）
                              ↓  stm32/write  (64 字节串口包)
                          【STM32】           ← 下位机：真正开车、动手臂
                              ↓  stm32/read   (10 字节回包)
                       stm32_control 靠回包推进流程

仿真里没有 STM32，所以流程跑到"发第一个动作"就卡住不动了 ✗
本节点补上这一环：订阅 stm32/write，模拟 STM32 的行为，回包到 stm32/read ✓

**边界画在 stm32/write，而不是 /target** —— 这样 stm32_control 的流程逻辑
（flows.cpp / action_handler.cpp）能**原样跑**，测的是真代码 ✓

## 协议（从 serial_packet.hpp 抄的，字段偏移一个都没改）

发送包 64 字节，全部 int16 小端：
    0-1   head = AA AA            2-3   action
    4-5   x_real                  6-7   y_real
    8-9   x_target               10-11  y_target
    12-13 target_heading (0前/1左/2后/3右)
    14-15 stair_direction (0上/1下)
    16-21 stag_x / stag_y / stag_detected
    22-31 机械臂 arm_send_count + 4 个关节
    32-35 x_offset / y_offset
    36-37 stair_lift_height (mm)
    38-43 kfs_relative_x / y / theta  (uint16)
    44-61 reserved[18]           62-63  tail = BB BB

接收包 10 字节：
    0-1  head = AA AA   2-3  ack_flag   8-9  tail = BB BB

⚠ 单位：x/y 是【毫米】（action_handler.cpp: `current_pose.x * 1000.0`）
   ⇒ 本节点收到后要 /1000 变成米 ✓

## ack 的语义（action_handler.cpp:266）

    ack != 2  →  "正在执行"，同时把 ack_ready_for_step_ 置位
    ack == 2  →  "完成"
    流程推进靠【非2 → 2 的上升沿】✓

⇒ 本节点的行为：接到动作先回 0，做完回一次 2，之后回 0（等下一个动作）

## 用法

    # 终端 1: 仿真
    bash tools/start_all.sh

    # 终端 2: 仿真下位机（本文件）
    python3 tools/sim_stm32.py

    # 终端 3: 真机的流程节点（原样跑，不用改）
    ros2 launch stm32_control stm32_control_GLIM.launch.py

    # 终端 4: 发目标
    python3 ~/rc2026_ros2_ws/src/utils/publish_target/publish_target.py --mode absolute --x 1.0 --y 0.5
"""

import math
import struct
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

# ---------------------------------------------------------------- 协议常量
HEAD = (0xAA, 0xAA)
TAIL = (0xBB, 0xBB)
SEND_LEN = 64
RECV_LEN = 10

# ---------------------------------------------------------------- 动作分类
# 真机的完整动作码见 stm32_control/include/stm32_control/action_codes.hpp
# 这里只分三类：
#   MOVE   —— 需要车真的走过去（闭环控制 /cmd_vel）
#   INSTANT—— 仿真里没有对应机构（机械臂/感知），直接当"完成"
#   IDLE   —— 停车待命
#
# ⚠ 这张表是【猜的】—— 我是按动作码的名字和注释分类的，
#   真机上每个码到底做什么，要问写流程的人 ✗
#   分类错了的话，在下面 ACTION_OVERRIDE 里改，不用动代码。
ACTION_MOVE = set()
ACTION_MOVE |= set(range(0x50, 0x5F))   # GOTO_BLOCK 0x50~0x5E: 前往 15 个 block/出口点
ACTION_MOVE |= {0x60, 0x61, 0x62, 0x63}  # GOTO_THIRD_LAYER / GOTO_PUT_POS / JGG_POSA / JGG_POSB
ACTION_MOVE |= set(range(0xA0, 0xA6))   # ACTION_FIRST 0xA0~0xA5: 一区六个矛头位置
ACTION_MOVE |= set(range(0x33, 0x3F))   # GET_KFS 0x33~0x3E: 12 个梅林格子的 KFS（可能也要走）
ACTION_IDLE = {0x00}

# 想强制指定某个码的行为，写在这里，例如：{0x33: "instant"}
ACTION_OVERRIDE = {}


def classify(action: int) -> str:
    if action in ACTION_OVERRIDE:
        return ACTION_OVERRIDE[action]
    if action in ACTION_IDLE:
        return "idle"
    if action in ACTION_MOVE:
        return "move"
    return "instant"


# ---------------------------------------------------------------- 工具
def i16(buf, off):
    return struct.unpack_from('<h', buf, off)[0]


def u16(buf, off):
    return struct.unpack_from('<H', buf, off)[0]


def make_ack(flag: int) -> UInt8MultiArray:
    b = bytearray(RECV_LEN)
    b[0], b[1] = HEAD
    struct.pack_into('<h', b, 2, flag)
    b[8], b[9] = TAIL
    m = UInt8MultiArray()
    m.data = list(b)
    return m


class SimStm32(Node):
    def __init__(self):
        super().__init__('sim_stm32')

        self.declare_parameter('write_topic', 'stm32/write')
        self.declare_parameter('read_topic', 'stm32/read')
        # 用哪个位姿做反馈：
        #   glim  —— 用 GLIM 的定位结果（推荐 ✓ 能测出"定位误差→走位误差"的传导）
        #   truth —— 用 Gazebo 真值（调试用，排除定位因素）
        self.declare_parameter('pose_source', 'glim')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # 到点判定与限速
        self.declare_parameter('pos_tol', 0.08)        # m
        self.declare_parameter('heading_tol_deg', 8.0)
        self.declare_parameter('v_max', 0.35)          # m/s
        self.declare_parameter('w_max', 0.8)           # rad/s
        self.declare_parameter('move_timeout', 60.0)   # s
        self.declare_parameter('stair_timeout', 30.0)  # s
        # 台阶：仿真里车不能真的爬 15 cm 台阶，用"抬升 + 前进"近似
        self.declare_parameter('simulate_stair', True)

        p = lambda n: self.get_parameter(n).value
        self.pose_source = p('pose_source')
        self.pos_tol = p('pos_tol')
        self.heading_tol = math.radians(p('heading_tol_deg'))
        self.v_max = p('v_max')
        self.w_max = p('w_max')
        self.move_timeout = p('move_timeout')
        self.stair_timeout = p('stair_timeout')

        # 状态
        self.pose = None          # (x, y) 用来做闭环反馈
        self.yaw = None
        self.truth = None
        self.action = 0
        self.x_target = 0.0       # 米（已换算）
        self.y_target = 0.0
        self.target_heading = 0
        self.stair_direction = 0
        self.stair_lift_height = 0

        self.busy = False
        self.busy_since = 0.0
        self.ack = 0
        self.ack_dirty = True
        self.ack2_until = 0.0      # ack=2 保持到什么时候（见 publish_ack）
        self.last_rx_time = 0.0
        self.last_key = None       # (action, x_target, y_target)，见 on_write

        self.cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic'), 10)
        self.ack_pub = self.create_publisher(UInt8MultiArray, p('read_topic'), 10)
        self.create_subscription(UInt8MultiArray, p('write_topic'), self.on_write, 10)

        if self.pose_source == 'glim':
            self.create_subscription(PoseStamped, '/glim_rosnode/pose', self.on_glim_pose, 10)
        self.create_subscription(Odometry, '/mini_car/odom', self.on_odom, 10)

        self.create_timer(0.05, self.loop)      # 20 Hz
        self.create_timer(0.1, self.publish_ack)

        self.get_logger().info(
            '仿真下位机已启动 | 反馈位姿=%s | 订阅 %s | 发布 %s'
            % (self.pose_source, p('write_topic'), p('read_topic')))

    # ------------------------------------------------------------ 输入
    def on_glim_pose(self, m):
        q = m.pose.orientation
        self.pose = (m.pose.position.x, m.pose.position.y)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))

    def on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.truth = (p.x, p.y, p.z)
        if self.pose_source == 'truth':
            self.pose = (p.x, p.y)
            self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z))

    def on_write(self, msg):
        b = bytes(msg.data)
        if len(b) != SEND_LEN:
            self.get_logger().warn('包长错了: %d（应该是 %d）' % (len(b), SEND_LEN))
            return
        if b[0:2] != bytes(HEAD) or b[62:64] != bytes(TAIL):
            self.get_logger().warn('包头/包尾不对: %02X%02X ... %02X%02X'
                                   % (b[0], b[1], b[62], b[63]))
            return

        self.last_rx_time = time.time()
        new_action = i16(b, 2)
        self.x_target = i16(b, 8) / 1000.0      # ⚠ 毫米 → 米
        self.y_target = i16(b, 10) / 1000.0
        self.target_heading = i16(b, 12)
        self.stair_direction = i16(b, 14)
        self.stair_lift_height = i16(b, 36)

        # ⚠ 触发条件不能只看【动作码变没变】✗
        #   真机流程里每步的动作码都会变，所以只看动作码"碰不到"问题；
        #   但手动测试时经常是【同一个动作码 + 新目标】（比如反复发 0x50 去不同点），
        #   只看动作码会把新目标整个忽略掉 ✗（踩过一次）
        #   ⇒ 动作码 或 目标 任何一个变了，都重新触发 ✓
        key = (new_action, self.x_target, self.y_target)
        if key != self.last_key:
            kind = classify(new_action)
            self.get_logger().info(
                '收到动作 0x%02X (%s) → 目标 (%.3f, %.3f) m, heading=%d, stair=%d, 抬升=%dmm'
                % (new_action, kind, self.x_target, self.y_target,
                   self.target_heading, self.stair_direction, self.stair_lift_height))
            self.action = new_action
            self.last_key = key
            self.busy = (kind == 'move')
            self.busy_since = time.time()
            self.set_ack(0)

    def set_ack(self, v):
        if v != self.ack:
            self.ack = v
            self.ack_dirty = True
            if v == 2:
                # 完成信号保持 0.3 秒再降回 0（见 publish_ack）
                self.ack2_until = time.time() + 0.3

    def publish_ack(self):
        # ack=2 要保持【一小段时间】再降回 0，不能只发一包 ✗
        #   只发一包的话，万一 stm32_control 那一拍没收到，上升沿就丢了，
        #   流程会永远卡在那一步 ✗
        #   保持 0.3 秒（约 3 包）足够稳，只要之后回到非 2 就还是一个干净的上升沿 ✓
        if self.ack == 2 and time.time() > self.ack2_until:
            self.ack = 0
        self.ack_pub.publish(make_ack(self.ack))

    # ------------------------------------------------------------ 控制
    def loop(self):
        if self.pose is None or self.yaw is None:
            return

        # 超时保护：长时间到不了就放弃并报完成，免得流程永远卡住
        if self.busy and time.time() - self.busy_since > self.move_timeout:
            self.get_logger().warn('到点超时（%.0f s），放弃并报完成' % self.move_timeout)
            self.busy = False
            self.stop()
            self.set_ack(2)
            return

        kind = classify(self.action)
        if kind == 'idle' or not self.busy:
            self.stop()
            return

        dx = self.x_target - self.pose[0]
        dy = self.y_target - self.pose[1]
        dist = math.hypot(dx, dy)

        if dist < self.pos_tol:
            self.stop()
            self.busy = False
            self.get_logger().info('已到点 (%.3f, %.3f)，误差 %.3f m'
                                   % (self.pose[0], self.pose[1], dist))
            self.set_ack(2)
            return

        # 朝目标点走的 P 控制。车是舵轮的，可以直接横移，不用先转向 ✓
        kp = 1.5
        vx = max(-self.v_max, min(self.v_max, kp * dx))
        vy = max(-self.v_max, min(self.v_max, kp * dy))
        # 离得远就全速，快到的时候限速，避免冲过头
        speed = math.hypot(vx, vy)
        if speed > self.v_max:
            vx *= self.v_max / speed
            vy *= self.v_max / speed

        t = Twist()
        t.linear.x, t.linear.y = vx, vy
        self.cmd_pub.publish(t)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    n = SimStm32()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.stop()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
