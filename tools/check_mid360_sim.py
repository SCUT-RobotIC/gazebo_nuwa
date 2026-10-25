#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查仿真 Mid-360 点云是否真的"像 Mid-360"。

用法（仿真跑起来之后，另开终端）：

    source /opt/ros/humble/setup.bash
    python3 tools/check_mid360_sim.py

检查项：
  1. 字段     x,y,z,intensity,tag,line,ring,t 都在
  2. 帧率     仿真时间戳间隔 ≈ 100 ms
  3. 点数     每帧几千 ~ 2 万（房间里有回波的点）
  4. 逐点时间 t ∈ [0, 0.1) 且单调不减
  5. 视场角   仰角 ≈ -7° ~ +52°，方位角覆盖满 360°
  6. 非重复   相邻两帧的 (方位,仰角) 占据栅格不能高度重合
  7. 线号     line/ring ∈ [0, 3]
"""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

# PointField 常量
FLOAT32 = 7
FLOAT64 = 8
UINT8 = 2
UINT16 = 4


def column(buf, fields, name, dtype, size):
    offset = fields[name][0]
    raw = buf[:, offset:offset + size].copy()
    return raw.view(dtype).ravel()


class Checker(Node):
    def __init__(self, topic, count):
        super().__init__('check_mid360_sim')
        self.topic = topic
        self.need = count
        self.frames = []
        self.wall_times = []
        self.create_subscription(
            PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info('等待 %d 帧 %s ...' % (count, topic))

    def on_cloud(self, msg):
        if len(self.frames) >= self.need:
            return

        fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
        required = ['x', 'y', 'z', 'intensity', 'tag', 'line', 'ring', 't']
        missing = [n for n in required if n not in fields]
        if missing:
            self.get_logger().error('缺少字段: %s；实际字段=%s'
                                    % (missing, list(fields)))
            rclpy.shutdown()
            return

        n = msg.width * msg.height
        step = msg.point_step
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if buf.size < n * step:
            self.get_logger().error('数据长度不足: %d < %d' % (buf.size, n * step))
            return
        buf = buf[:n * step].reshape(n, step)

        x = column(buf, fields, 'x', '<f4', 4)
        y = column(buf, fields, 'y', '<f4', 4)
        z = column(buf, fields, 'z', '<f4', 4)
        t = column(buf, fields, 't', '<f4', 4)
        line = column(buf, fields, 'line', np.uint8, 1)
        ring = column(buf, fields, 'ring', np.uint8, 1)
        tag = column(buf, fields, 'tag', np.uint8, 1)

        r = np.sqrt(x * x + y * y + z * z)
        valid = r > 1e-6
        elevation = np.degrees(np.arcsin(np.clip(z[valid] / r[valid], -1, 1)))
        azimuth = np.degrees(np.arctan2(y[valid], x[valid]))

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.frames.append({
            'stamp': stamp,
            'stamp_ns': msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec,
            'frame_id': msg.header.frame_id,
            'n': n,
            't_min': float(t.min()) if n else 0.0,
            't_max': float(t.max()) if n else 0.0,
            't_monotonic': float(np.mean(np.diff(t) >= 0)) if n > 1 else 1.0,
            'elev_min': float(elevation.min()) if elevation.size else 0.0,
            'elev_max': float(elevation.max()) if elevation.size else 0.0,
            'az_bins': int(np.unique(
                np.floor((azimuth + 180.0) / 10.0).astype(np.int32) % 36).size)
                        if azimuth.size else 0,
            'line_min': int(line.min()) if n else 0,
            'line_max': int(line.max()) if n else 0,
            'ring_eq_line': bool(np.all(line == ring)),
            'tag_min': int(tag.min()) if n else 0,
            'tag_max': int(tag.max()) if n else 0,
            'azimuth': azimuth,
            'elevation': elevation,
        })
        self.wall_times.append(time.monotonic())
        self.get_logger().info('收到第 %d 帧: %d 点, frame=%s'
                               % (len(self.frames), n, msg.header.frame_id))

    def report(self):
        frames = self.frames
        if len(frames) < 2:
            print('FAIL: 收到的帧数不足（%d）' % len(frames))
            return False

        ok = True
        print('\n================ Mid-360 仿真检查 ================')
        print('话题: %s   帧数: %d' % (self.topic, len(frames)))

        f0 = frames[0]
        print('frame_id: %s' % f0['frame_id'])
        print('点数: min=%d max=%d' % (min(f['n'] for f in frames),
                                      max(f['n'] for f in frames)))
        print('t: [%.6f, %.6f] s, 单调点占比 %.4f'
              % (f0['t_min'], f0['t_max'], f0['t_monotonic']))
        print('仰角: [%.2f, %.2f] deg' % (f0['elev_min'], f0['elev_max']))
        print('方位角 10° 直方图非空桶: %d / 36' % f0['az_bins'])
        print('line: [%d, %d], ring==line: %s, tag: [%d, %d]'
              % (f0['line_min'], f0['line_max'], f0['ring_eq_line'],
                 f0['tag_min'], f0['tag_max']))

        # 仿真时间戳间隔（header stamp 来自 /clock）
        stamp_deltas = np.diff([f['stamp'] for f in frames])
        median_dt = float(np.median(stamp_deltas)) if stamp_deltas.size else 0.0
        print('header.stamp 间隔: median=%.4f s (期望 0.1)' % median_dt)

        wall_deltas = np.diff(self.wall_times)
        wall_rate = 1.0 / float(np.median(wall_deltas)) if wall_deltas.size else 0.0
        sim_rate = 1.0 / median_dt if median_dt > 0 else 0.0
        rtf = (wall_rate / sim_rate) if sim_rate > 0 else 0.0
        print('仿真帧率: %.2f Hz (wall %.2f Hz → RTF≈%.2f)'
              % (sim_rate, wall_rate, rtf))

        # 非重复性：相邻帧的方向占据栅格 Jaccard 相似度
        def occupancy(f):
            az_bin = np.floor((f['azimuth'] + 180.0) / 1.0).astype(np.int32)
            el_bin = np.floor((f['elevation'] + 90.0) / 1.0).astype(np.int32)
            az_bin = np.clip(az_bin, 0, 359)
            el_bin = np.clip(el_bin, 0, 179)
            return np.unique(az_bin.astype(np.int64) * 1000 + el_bin)

        ious = []
        for a, b in zip(frames[:-1], frames[1:]):
            sa, sb = occupancy(a), occupancy(b)
            inter = np.intersect1d(sa, sb).size
            union = np.union1d(sa, sb).size
            ious.append(inter / union if union else 1.0)
        print('相邻帧方向栅格 Jaccard: median=%.3f (越低越"非重复")'
              % (float(np.median(ious)) if ious else 0.0))

        def check(name, cond):
            nonlocal ok
            print(('  PASS  ' if cond else '  FAIL  ') + name)
            ok = ok and cond

        print('\n判定:')
        check('字段齐全', True)
        check('每帧点数 > 2000', min(f['n'] for f in frames) > 2000)
        check('t 范围在 [0, 0.1001]', all(
            -1e-5 <= f['t_min'] and f['t_max'] <= 0.1001 for f in frames))
        check('t 单调不减 (>99%)', f0['t_monotonic'] > 0.99)
        check('仰角下限 <= -6.5°', f0['elev_min'] <= -6.5)
        check('仰角上限 >= 50°', f0['elev_max'] >= 50.0)
        check('方位角覆盖 36 个 10° 桶', f0['az_bins'] >= 36)
        check('line 在 [0,3] 且 ring==line',
              f0['line_min'] >= 0 and f0['line_max'] <= 3 and f0['ring_eq_line'])
        check('header.stamp 间隔 ∈ [0.09, 0.11] s',
              0.09 <= median_dt <= 0.11)
        check('相邻帧不是同一套方向 (Jaccard < 0.95)',
              (not ious) or float(np.median(ious)) < 0.95)

        print('==================================================\n')
        return ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/mid360/points')
    parser.add_argument('--frames', type=int, default=20)
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = Checker(args.topic, args.frames)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and len(node.frames) < args.frames:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() > deadline:
                print('TIMEOUT: 只收到 %d 帧' % len(node.frames))
                break
    except KeyboardInterrupt:
        pass
    ok = node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
