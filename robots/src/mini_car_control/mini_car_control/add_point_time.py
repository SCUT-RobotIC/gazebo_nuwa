#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给仿真的点云补上【逐点时间戳】。

⚠ 2026-10 起，仿真已改用 robots/src/mid360_sim 的 gz-sim 系统插件，
  插件本身就会输出正确的 t 字段（见该包的 README）。
  本脚本仅作为【旧版 gpu_lidar】的临时补丁保留，**不要在现在的仿真里再启动它**；
  它会重复加 t 字段，而且旋转雷达公式对 Mid-360 的非重复扫描是错的。


为什么要这个东西
    GLIM（激光定位）需要知道"每个点是在这一帧的什么时刻测到的"，才能把
    【扫描期间车的运动】补偿掉（叫"运动畸变校正" / deskew）。

    真机 Livox：每个点自带时间 ✓（一帧 100 ms 是【扫】出来的，不是拍的）
    我们的仿真：Gazebo 的 gpu_lidar 一次性渲染完 ✗ 所有点共用一个时间 ✗

    不补的后果：车一动，点云就是"糊"的 ✗ → 配准出错 → 定位偏
    实测：车实际走 1.085 m，GLIM 只算出 0.284 m（差 4 倍）✗

GLIM 认哪个字段名（源码 ros_cloud_converter.hpp）
    "t" / "time" / "time_stamp" / "timestamp"  都可以 ✓
    单位：秒；含义：相对【帧时间戳】的偏移（配置 perpoint_relative_time: true）

时间怎么算
    仿真的点云没有天然先后顺序 ✗
    所以按【方位角】给时间，模仿机械旋转雷达转一圈：
        t = (方位角 + π) / (2π) × 扫描周期
    真机 Mid-360 是非重复扫描 ✗ 没有这种顺序 ✗
    但运动补偿只需要"每个点大致在帧内什么时刻" ✓ 这个近似够用 ✓

用法
    ros2 run mini_car_control add_point_time
    # 然后把 GLIM 的 points_topic 指到 /mid360/points_t
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class AddPointTime(Node):
    def __init__(self):
        super().__init__('add_point_time')

        self.declare_parameter('in_topic', '/mid360/points')
        self.declare_parameter('out_topic', '/mid360/points_t')
        self.declare_parameter('scan_period', 0.1)   # 10 Hz → 一帧 100 ms

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.period = float(self.get_parameter('scan_period').value)

        self.create_subscription(PointCloud2, in_topic, self.on_cloud,
                                 qos_profile_sensor_data)      # 雷达是流式数据 ✓
        self.pub = self.create_publisher(PointCloud2, out_topic, 10)

        self.n = 0
        self.tick = 0
        self.get_logger().info('%s → %s（补 t 字段，周期 %.3f s）'
                               % (in_topic, out_topic, self.period))

    def on_cloud(self, msg):
        # ── ① 先找到 x / y / z 三个字段在哪 ──
        # PointCloud2 是"二进制大块"，字段位置要靠 fields 表查 ✓
        off = {f.name: f.offset for f in msg.fields}
        if not all(k in off for k in ('x', 'y', 'z')):
            self.get_logger().error('点云里没有 x/y/z 字段 ✗')
            return
        if any(f.name == 't' for f in msg.fields):
            self.pub.publish(msg)          # 已经有 t 了，原样转发 ✓
            return

        n = msg.width * msg.height
        step = msg.point_step
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if buf.size < n * step:
            self.get_logger().warn('点云数据长度不对，跳过这一帧')
            return

        # ── ② 把 x, y 抠出来算方位角 ──
        #   reshape 成 (点数, 每点字节数)，再按字段偏移取 4 字节当 float32 ✓
        pts = buf[:n * step].reshape(n, step)
        x = pts[:, off['x']:off['x'] + 4].copy().view(np.float32).ravel()
        y = pts[:, off['y']:off['y'] + 4].copy().view(np.float32).ravel()

        az = np.arctan2(y, x)                        # 方位角 ∈ [-π, π]
        t = (az + math.pi) / (2.0 * math.pi) * self.period   # → [0, 扫描周期]

        # ── ③ 拼出新点云：原来的字节原封不动，每点后面追加 4 字节的时间 ──
        new_step = step + 4
        out = np.zeros((n, new_step), dtype=np.uint8)
        out[:, :step] = pts
        out[:, step:] = t.astype(np.float32).view(np.uint8).reshape(n, 4)

        new = PointCloud2()
        new.header = msg.header                    # 时间戳和坐标系照抄 ✓
        new.height = msg.height
        new.width = msg.width
        new.is_bigendian = msg.is_bigendian
        new.point_step = new_step
        new.row_step = new_step * msg.width
        new.is_dense = msg.is_dense
        new.fields = list(msg.fields) + [
            PointField(name='t', offset=step, datatype=PointField.FLOAT32, count=1)
        ]
        new.data = out.tobytes()
        self.pub.publish(new)

        self.tick += 1
        if self.tick % 100 == 0:
            self.get_logger().info('已处理 %d 帧，每帧 %d 点，t ∈ [%.4f, %.4f] s'
                                   % (self.tick, n, float(t.min()), float(t.max())))


def main():
    rclpy.init()
    node = AddPointTime()
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
