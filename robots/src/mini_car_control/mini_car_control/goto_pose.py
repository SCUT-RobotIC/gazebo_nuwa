#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""走定点：把车开到指定的 (x, y, yaw)。

为什么不用"先转向、再直行、再转向"那套
    我们的底盘是【舵轮】，天生能横移和斜着走 —— 不需要先转向 ✗
    直接算"目标相对于我偏了多少"，让四个轮子朝那个方向一起走就行 ✓
    （舵轮运动学在 swerve_controller 里，本节点只出 vx / vy / w）

数据流
    目标位姿 ──/goal_pose──▶ 本节点 ──/cmd_vel──▶ swerve_controller ──▶ 四个轮子
                              ▲
        TF: map→base_link ────┘   （我现在的位姿；仿真=里程计，真机=Mid-360 定位）

控制律（三步纯三角）
    ① 目标相对车的位移（map 系）：ex = tx − x,  ey = ty − y
    ② 转到【车体系】（因为 /cmd_vel 的 vx/vy 是车头方向和车宽方向）：
           ex_r =  cos(yaw)·ex + sin(yaw)·ey
           ey_r = −sin(yaw)·ex + cos(yaw)·ey
    ③ 输出：vx = k·ex_r,  vy = k·ey_r,  w = k_yaw·(目标朝向 − 当前朝向)

    ⚠ 朝向误差必须归一化到 [−π, π]，否则车会绕远路转一大圈 ✗

用法
    ros2 run mini_car_control goto_pose
    # 另开终端发目标（RViz 里用 "2D Goal Pose" 也行）：
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: map}, pose: {position: {x: -3.5, y: 4.75}, orientation: {z: 0.383, w: 0.924}}}"
    # （z/w 是 yaw=45° 的四元数：z=sin(22.5°), w=cos(22.5°)）
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import tf2_ros
from rclpy.time import Time


def norm_angle(a):
    """把角度绕回 [−π, π]。"""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_of(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)      # (z, w)


class GotoPose(Node):
    def __init__(self):
        super().__init__('goto_pose')

        self.declare_parameter('kp_xy', 1.2)        # 位置比例增益
        self.declare_parameter('kp_yaw', 2.0)       # 朝向比例增益
        self.declare_parameter('vmax', 0.6)         # 最大线速度 m/s
        self.declare_parameter('wmax', 1.5)         # 最大角速度 rad/s
        self.declare_parameter('tol_xy', 0.06)      # 位置容差 m
        self.declare_parameter('tol_yaw', 0.09)     # 朝向容差 rad（约 5°）
        self.declare_parameter('hold', 0.4)         # 到位后要保持几秒才算到
        self.declare_parameter('timeout', 60.0)     # 超时保护

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/goto_pose/status', 10)

        self.goal = None            # (x, y, yaw)
        self.goal_time = None
        self.hold_since = None
        self.last_state = None
        self.create_timer(0.05, self.control)       # 20 Hz

        self.get_logger().info('走定点已就绪，等 /goal_pose ...')

    def on_goal(self, msg):
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().warn('目标坐标系是 %r，本节点只认 map ✗'
                                   % msg.header.frame_id)
            return
        p = msg.pose.position
        yaw = yaw_of(msg.pose.orientation)
        self.goal = (p.x, p.y, yaw)
        self.goal_time = self.get_clock().now()
        self.hold_since = None
        self.get_logger().info('新目标 map(%.3f, %.3f) 朝向 %.1f°'
                               % (p.x, p.y, math.degrees(yaw)))

    def say(self, s):
        """状态只在变化时发布一次，别刷屏。"""
        if s != self.last_state:
            self.status.publish(String(data=s))
            self.last_state = s

    def control(self):
        if self.goal is None:
            return
        try:
            tf = self.tf_buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            self.say('等定位')
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_of(tf.transform.rotation)
        tx, ty, tyaw = self.goal

        # ① 目标相对车的位移（map 系）
        ex, ey = tx - x, ty - y
        # ② 转到车体系
        c, s = math.cos(yaw), math.sin(yaw)
        ex_r = c * ex + s * ey
        ey_r = -s * ex + c * ey
        # ③ 朝向误差（必须归一化 ✗ 否则会绕远路）
        eyaw = norm_angle(tyaw - yaw)

        dist = math.hypot(ex, ey)
        cmd = Twist()

        if dist < self.get_parameter('tol_xy').value and \
                abs(eyaw) < self.get_parameter('tol_yaw').value:
            # 到位了 —— 但要【保持一小会儿】才算数（防止路过时误判）
            if self.hold_since is None:
                self.hold_since = self.get_clock().now()
            held = (self.get_clock().now() - self.hold_since).nanoseconds / 1e9
            if held >= self.get_parameter('hold').value:
                self.cmd.publish(cmd)                # 全零 = 停
                self.say('已到达')
                self.goal = None                     # 任务结束，不再输出
                return
            self.say('到位保持中')
        else:
            self.hold_since = None
            self.say('行驶中')

        # 超时保护
        el = (self.get_clock().now() - self.goal_time).nanoseconds / 1e9
        if el > self.get_parameter('timeout').value:
            self.cmd.publish(cmd)
            self.get_logger().warn('走定点超时（%.0f 秒），放弃' % el)
            self.say('超时放弃')
            self.goal = None
            return

        vmax = self.get_parameter('vmax').value
        wmax = self.get_parameter('wmax').value
        vx = self.get_parameter('kp_xy').value * ex_r
        vy = self.get_parameter('kp_xy').value * ey_r
        w = self.get_parameter('kp_yaw').value * eyaw

        # 限幅（各自独立；线速度按 vx/vy 的合矢量限）
        n = math.hypot(vx, vy)
        if n > vmax:
            vx, vy = vx * vmax / n, vy * vmax / n
        w = max(-wmax, min(wmax, w))

        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, w
        self.cmd.publish(cmd)


def main():
    rclpy.init()
    node = GotoPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
