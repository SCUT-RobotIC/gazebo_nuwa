#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核对一段真实 Mid-360 rosbag2 的数据流，并与仿真轨迹对比。

用法：

    conda deactivate
    source /opt/ros/humble/setup.bash
    # CustomMsg 需要 livox_ros_driver2 消息包
    source ~/rc2026_ros2_ws/install/setup.bash
    python3 tools/analyze_mid360_bag.py --bag /data/bags/mid360_static_01

输出：
  · 帧率、每帧点数、逐点 offset_time、FOV、line/反射率
  · IMU 频率、加速度单位（g 还是 m/s²）、静止/运动时间段
  · 与仿真用的 patterns/mid360.csv 的逐点方向匹配率

说明：
  · CustomMsg 的 offset_time 是相对 timebase 的纳秒；
  · PointCloud2（LivoxPointXyzrtlt）的 timestamp 是绝对纳秒；
  · 驱动 xfer_format=0 时 /livox/lidar 是 PointCloud2；
    xfer_format=4 时 /livox/lidar 是 CustomMsg，
    同时 /livox/lidar/pointcloud 是 PointCloud2。
"""

import argparse
import collections
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2

try:
    from livox_ros_driver2.msg import CustomMsg
except Exception:  # pragma: no cover
    CustomMsg = None

DEFAULT_PATTERN = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'robots', 'src', 'mid360_sim', 'patterns', 'mid360.csv')

FLOAT32, FLOAT64, UINT8 = 7, 8, 2


def stamp_ns(msg):
    return msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec


def read_fields(msg, names):
    """Return {name: numpy array} from a PointCloud2."""
    out = {}
    step = msg.point_step
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width * msg.height
    if buf.size < n * step:
        return out
    buf = buf[:n * step].reshape(n, step)
    for name, dtype, size in names:
        field = next((f for f in msg.fields if f.name == name), None)
        if field is None:
            continue
        out[name] = buf[:, field.offset:field.offset + size].copy().view(dtype).ravel()
    return out


def load_pattern(path):
    if not path or not os.path.exists(path):
        return None
    d = np.loadtxt(path, delimiter=',', skiprows=1)
    az = np.deg2rad(d[:, 1])
    el = np.deg2rad(90.0 - d[:, 2])
    return np.stack([np.cos(el) * np.cos(az),
                     np.cos(el) * np.sin(az),
                     np.sin(el)], axis=1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--points-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--pattern', default=DEFAULT_PATTERN)
    parser.add_argument('--sample-every', type=int, default=40)
    parser.add_argument('--max-frames', type=int, default=40)
    args = parser.parse_args()

    try:
        from scipy.spatial import cKDTree
    except Exception:
        cKDTree = None

    tree = None
    pattern = load_pattern(args.pattern)
    if pattern is not None and cKDTree is not None:
        tree = cKDTree(pattern)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))

    imu_t, imu_acc, imu_gyr = [], [], []
    frames = []
    frame_idx = 0
    topic_types = collections.Counter()

    while reader.has_next():
        topic, data, _ = reader.read_next()
        topic_types[topic] += 1

        if topic == args.imu_topic:
            m = deserialize_message(data, Imu)
            imu_t.append(stamp_ns(m) * 1e-9)
            imu_acc.append([m.linear_acceleration.x,
                            m.linear_acceleration.y,
                            m.linear_acceleration.z])
            imu_gyr.append([m.angular_velocity.x,
                            m.angular_velocity.y,
                            m.angular_velocity.z])
            continue

        if topic != args.points_topic:
            continue

        if frame_idx % args.sample_every != 0 or len(frames) >= args.max_frames:
            frame_idx += 1
            continue

        frame = None
        if CustomMsg is not None and args.points_topic == '/livox/lidar':
            try:
                m = deserialize_message(data, CustomMsg)
                pts = m.points
                frame = dict(
                    ns=stamp_ns(m), n=len(pts), frame_id=m.header.frame_id,
                    x=np.array([p.x for p in pts]),
                    y=np.array([p.y for p in pts]),
                    z=np.array([p.z for p in pts]),
                    off=np.array([p.offset_time for p in pts], dtype=np.int64),
                    line=np.array([p.line for p in pts], dtype=np.int16),
                    refl=np.array([p.reflectivity for p in pts], dtype=np.uint8),
                    tag=np.array([p.tag for p in pts], dtype=np.uint8))
            except Exception:
                frame = None
        if frame is None:
            m = deserialize_message(data, PointCloud2)
            vals = read_fields(m, [('x', '<f4', 4), ('y', '<f4', 4),
                                   ('z', '<f4', 4), ('intensity', '<f4', 4),
                                   ('line', np.uint8, 1),
                                   ('timestamp', '<f8', 8)])
            if not {'x', 'y', 'z'}.issubset(vals):
                frame_idx += 1
                continue
            n = len(vals['x'])
            ts = vals.get('timestamp', np.zeros(n))
            frame = dict(
                ns=stamp_ns(m), n=n, frame_id=m.header.frame_id,
                x=vals['x'], y=vals['y'], z=vals['z'],
                off=((ts - ts.min()) if 'timestamp' in vals else np.arange(n) * 5000).astype(np.int64),
                line=vals.get('line', np.zeros(n, dtype=np.uint8)).astype(np.int16),
                refl=vals.get('intensity', np.zeros(n)).astype(np.uint8),
                tag=np.zeros(n, dtype=np.uint8))
        frames.append(frame)
        frame_idx += 1

    print('=== bag ===')
    print(' topics:', dict(topic_types))
    print(' points messages: %d, sampled: %d, frame_id=%s' % (
        topic_types[args.points_topic], len(frames),
        frames[0]['frame_id'] if frames else '?'))

    print('\n=== IMU ===')
    if len(imu_t) > 2:
        t = np.array(imu_t)
        dt = np.diff(t)
        acc = np.array(imu_acc)
        gyr = np.array(imu_gyr)
        an = np.linalg.norm(acc, axis=1)
        gn = np.linalg.norm(gyr, axis=1)
        print(' rate: %.1f Hz (dt %.4f..%.4f s)' % (1 / np.median(dt), dt.min(), dt.max()))
        print(' acc norm: mean %.4f std %.4f  (%s)' % (
            an.mean(), an.std(),
            'g' if 0.8 < an.mean() < 1.2 else 'm/s^2'))
        print(' gyro norm: mean %.4f std %.4f max %.4f rad/s' % (
            gn.mean(), gn.std(), gn.max()))
        print('\n time(s)  acc_std  gyro_mean  gyro_std   state')
        t0 = t[0]
        for lo in np.arange(0, t[-1] - t0, 5.0):
            sel = (t - t0 >= lo) & (t - t0 < lo + 5.0)
            if not sel.any():
                continue
            state = 'MOVE' if (gn[sel].std() > 0.03 or an[sel].std() > 0.03) else 'static'
            print(' %6.1f   %7.4f  %9.4f  %8.4f   %s' % (
                lo, an[sel].std(), gn[sel].mean(), gn[sel].std(), state))
    else:
        print(' no IMU messages')

    print('\n=== lidar frames ===')
    for k, f in enumerate(frames[:12]):
        valid = (np.abs(f['x']) + np.abs(f['y']) + np.abs(f['z'])) > 1e-9
        if not valid.any():
            continue
        rng = np.sqrt(f['x'] ** 2 + f['y'] ** 2 + f['z'] ** 2)
        el = np.degrees(np.arcsin(np.clip(f['z'][valid] / rng[valid], -1, 1)))
        az = np.degrees(np.arctan2(f['y'][valid], f['x'][valid]))
        off = f['off']
        line = f['line'][valid]
        refl = f['refl'][valid]
        msg = (' f%02d n=%d valid=%d off=[%.3f,%.3f]ms el=[%.2f,%.2f] '
               'azbins=%d line=%s refl_med=%d') % (
            k, f['n'], valid.sum(), off.min() / 1e6, off.max() / 1e6,
            el.min(), el.max(),
            len(np.unique(np.floor((az + 180) / 10).astype(int) % 36)),
            np.unique(line).tolist(), int(np.median(refl)))
        if tree is not None:
            rdir = np.stack([f['x'][valid], f['y'][valid], f['z'][valid]], 1)
            rdir /= np.maximum(np.linalg.norm(rdir, axis=1), 1e-9)[:, None]
            dist, _ = tree.query(rdir, k=1)
            ang = np.degrees(2 * np.arcsin(np.clip(dist / 2, 0, 1)))
            msg += ('  match<0.1=%.1f%% <0.5=%.1f%%'
                    % (100 * np.mean(ang < 0.1), 100 * np.mean(ang < 0.5)))
        print(msg)

    print('\n提示：')
    print('  · 真实数据会包含无回波的零值点（x=y=z=0），仿真默认不发布这些点；')
    print('  · 真实驱动的逐点时间在 offset_time/timestamp 里，仿真用 t（相对秒）；')
    print('  · 实机 GLIM 的 acc_scale 要按 IMU 单位设置（Livox 为 g，约 9.79）。')


if __name__ == '__main__':
    main()
