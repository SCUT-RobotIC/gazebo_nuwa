#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Gazebo 的里程计变成 map → base_link 的 TF。

数据流
    Gazebo ──/mini_car/odom──▶ 本节点 ──TF──▶ RViz / tower_check / 走定点
          (场地系位姿)                    (map 系位姿)

只干一件事：把 Gazebo 给的位姿减掉原点偏移，当成 TF 发出去。
    map = 场地 − 原点      原点 = 己方启动区在场地角上的那个角
    红队 (5.5, −5.5)      蓝队 (5.5, +5.5)          （见 README §8）

为什么是 map → base_link，不是 odom → base_link
    常规 ROS 架构是 map → odom → base_link 三段。
    仿真里 Gazebo 给的是【真值位姿】、没有累积误差，所以直接当 map → base_link ✓
    真机上这一步由 Mid-360 的 LIO 提供，发的也是 map → base_link ✓ 接口一致

用法
    ros2 run mini_car_control odom_to_tf --ros-args -p team:=red
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

# 两队各自的原点在【场地系】里的位置（见 README §8）
ORIGIN = {'red': (5.5, -5.5), 'blue': (5.5, 5.5)}


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        # ── ① 读队伍参数，算出偏移量 ──
        self.declare_parameter('team', 'red')
        team = self.get_parameter('team').value
        if team not in ORIGIN:                  # 拼错队伍名时别默默算错
            self.get_logger().warn('team 只认 %s，收到 %r → 按 red 处理'
                                   % (list(ORIGIN), team))
            team = 'red'
        self.ox, self.oy = ORIGIN[team]         # 存起来，回调里每次都减它
        self.get_logger().info(
            '队伍=%s  原点(场地系)=(%.1f, %.1f)    map = 场地 − 原点'
            % (team, self.ox, self.oy))

        # ── ② TF 发布器（只要一个，建一次）──
        self.br = tf2_ros.TransformBroadcaster(self)

        # ── ③ 订阅里程计 ──
        self.create_subscription(Odometry, '/mini_car/odom', self.on_odom, 10)

    def on_odom(self, msg):
        t = TransformStamped()

        # 时间戳用【数据自己的】。
        # TF 是按时间查询的，用 now() 会引入延迟 → 车一动位姿就滞后 ✗
        t.header.stamp = msg.header.stamp

        # 父子方向不能反：TF 的含义是"子在父坐标系里的位姿"。
        # 我们要问的是"车在世界哪里" → map 是父 ✓
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # 平移：减掉原点偏移（map = 场地 − 原点）
        # ⚠ z 【不】减 —— map 系和场地系共用同一个"地面 = 0"
        t.transform.translation.x = msg.pose.pose.position.x - self.ox
        t.transform.translation.y = msg.pose.pose.position.y - self.oy
        t.transform.translation.z = msg.pose.pose.position.z

        # 朝向：原样抄。
        # map 系和场地系只差一个【纯平移】、没有旋转
        # → 同一个四元数在两个系里都成立 ✓（这就是当初选纯平移的回报）
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
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
