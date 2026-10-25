#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""建塔处占用检测 v3：两台 2D 雷达，分高度切【塔基】和【中段】。
用法
    ros2 run mini_car_control tower_check --ros-args -p team:=red

    # 只盯 6 个公共位，每秒打一次表（状态一变立刻打）
    ros2 run mini_car_control tower_check --ros-args -p team:=red \
      -p log_every:=10 \
      -p only_spots:=L1_W_M,L1_E_M,L2_SW,L2_SE,L2_NW,L2_NE
"""

import math
import os
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(_HERE))))
DEFAULT_SPOTS = os.path.join(_ROOT, 'robots/src/mini_car/config/tower_spots.yaml')

# 状态取值。前三个是【确定的】，后两个是【没看清】。
EMPTY, BASE, FULL, UNKNOWN, NA = '空', '塔基', '满', '未知', '—'


def norm_angle(a):
    """把角度绕回 [−π, π]。"""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    """从四元数取偏航角（只绕 Z 转的那部分）。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Observation:
    """一台雷达对一条高度带的观测结果。

    seen = True / False / None
        None 有两种原因，必须区分：
          · 传感器高度根本不在这一带里（height=False）→ 这台雷达【不负责】这条带
          · 高度对，但视线被更近的东西挡住（blocked=True）→ 这条带【看不清】
    """

    def __init__(self, seen, cnt=0, near=None, closer=None,
                 height=True, fov=True, face=None):
        self.seen = seen
        self.cnt = cnt
        self.near = near
        self.closer = closer
        self.height = height        # 传感器高度落在这条带里吗
        self.fov = fov              # 方位在雷达有效视场内吗
        self.face = face            # 按【方块摆正】算的期望近面距离（精度测试用）


class TowerCheck(Node):
    def __init__(self):
        super().__init__('tower_check')

        # 启动横幅：一眼看出跑的是不是新包。
        # 旧包（gazebo_nuwa install）没有这一行，而且订阅的是 /scan ✗
        self.get_logger().info('tower_check v3（双雷达）  模块=%s'
                               % os.path.abspath(__file__))

        # ── 参数 ──
        self.declare_parameter('team', 'red')
        self.declare_parameter('spots_file', DEFAULT_SPOTS)

        # 两台雷达的话题。
        self.declare_parameter('low_topic', '/scan_low')
        self.declare_parameter('high_topic', '/scan_high')

        # 车体：base_link 在【所在层台面】上方多高
        self.declare_parameter('base_link_h', 0.30)
        # L2 比 L1 高多少
        self.declare_parameter('up_level_dz', 0.30)
        # 雷达安装位姿（base_link 系）。有静态 TF 时【以 TF 为准】，
        # 这几个只是 TF 还没起来时的兜底值。
        #   ⚠ 改了 xacro 的 <pose>，launch 里的 static_transform_publisher
        #     也要改；这两处对上了，这里自动跟上 ✓
        self.declare_parameter('low_x', 0.42)
        self.declare_parameter('low_y', 0.0)
        self.declare_parameter('low_dz', -0.125)
        self.declare_parameter('high_x', 0.0)
        self.declare_parameter('high_y', 0.0)
        self.declare_parameter('high_dz', 0.26)

        # 低雷达装车头，只有前方干净；后方被自己的车壳挡 → 不判
        # 真机上前伸支架的切线约 ±101°，留点余量取 ±95°
        self.declare_parameter('low_fov_deg', 190.0)

        # ── 判据 ──
        self.declare_parameter('half_width_deg', 10.0)  # 扇区半宽
        self.declare_parameter('dist_tol', 0.20)        # 近侧面距离容差
        self.declare_parameter('min_points', 3)         # 至少几个回波才算"有"
        self.declare_parameter('band_margin', 0.03)     # 离高度带边缘至少留这么多
        self.declare_parameter('level_tol', 0.15)       # 同层判定容差
        self.declare_parameter('occlusion_check', True)  # 被挡住时报【未知】而非【没有】
        self.declare_parameter('max_scan_age', 0.6)     # 扫描比这老就不用了
        self.declare_parameter('enabled', True)         # 给"走到定点再开"用
        self.declare_parameter('log_every', 20)
        # 只判指定建塔处，逗号分隔；空 = 全部 8 个。
        # 测 6 个公共位时：
        #   -p only_spots:=L1_W_M,L1_E_M,L2_SW,L2_SE,L2_NW,L2_NE
        self.declare_parameter('only_spots', '')

        # ── 读坐标清单 ──
        self.geom, self.spots = self.load_spots(
            self.get_parameter('spots_file').value,
            self.get_parameter('team').value)
        only = [x.strip() for x in
                self.get_parameter('only_spots').value.split(',') if x.strip()]
        if only:
            known = {s['name'] for s in self.spots}
            missing = [x for x in only if x not in known]
            if missing:
                self.get_logger().warn('only_spots 里有不认识的：%s' % missing)
            self.spots = [s for s in self.spots if s['name'] in only]
            self.get_logger().info('只判 %d 个：%s'
                                   % (len(self.spots),
                                      [s['name'] for s in self.spots]))
        self.block = float(self.geom['block'])
        self.block_half = self.block / 2.0

        # ── TF（listener 在后台自动收，buffer 里查）──
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        # ── 两台雷达 ──
        # 用哪个回调就用 which 区分；两个话题分开存，别互相覆盖 ✓
        self.scans = {'low': None, 'high': None}   # (msg, 收包时刻)
        low_topic = self.get_parameter('low_topic').value
        high_topic = self.get_parameter('high_topic').value
        self.create_subscription(
            LaserScan, low_topic,
            lambda m: self.on_scan('low', m), qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, high_topic,
            lambda m: self.on_scan('high', m), qos_profile_sensor_data)
        self.get_logger().info('低雷达 ← %s   高雷达 ← %s'
                               % (low_topic, high_topic))

        self.pub = self.create_publisher(String, '/tower_check/status', 10)

        self.last_verdicts = None       # 上次的判断结果，用来发现"有变化"
        self.tick = 0
        self.sensor_poses = {}          # frame_id → (x, y, z, yaw)
        self.sensor_warned = set()

        # 两个话题的到达频率可能不一样，用一个 10 Hz 定时器统一判，
        # 不要挂在某一台的扫描回调里（另一台可能更慢/丢帧）✓
        self.create_timer(0.1, self.on_timer)

    def load_spots(self, path, team):
        with open(path, encoding='utf-8') as f:
            d = yaml.safe_load(f)
        if team not in d:
            raise ValueError('队伍 %r 不在 %s 里（只有 %s）'
                             % (team, path, [k for k in d if k in ('red', 'blue')]))
        spots = d[team]['spots']
        self.get_logger().info('队伍=%s  载入 %d 个建塔处 ← %s'
                               % (team, len(spots), path))
        return d['geometry'], spots

    # ──────────────────────── 输入 ────────────────────────
    def on_scan(self, which, msg):
        if not self.get_parameter('enabled').value:
            return
        self.scans[which] = (msg, time.monotonic())

    def fresh_scan(self, which, now):
        """返回 max_scan_age 秒内的扫描，否则 None。"""
        item = self.scans[which]
        if item is None:
            return None
        msg, t = item
        if now - t > self.get_parameter('max_scan_age').value:
            return None
        return msg

    def sensor_pose(self, which, msg):
        """传感器在 base_link 系里的 (x, y, z, yaw)。

        优先问 TF —— 这样 xacro / launch / 本节点不会有三份手抄的安装位置 ✓
        （static_transform_publisher 发的就是 base_link → 雷达 frame）
        TF 还没起来时用参数兜底，并只警告一次。
        """
        frame = msg.header.frame_id if msg is not None else ''
        if frame and frame in self.sensor_poses:
            return self.sensor_poses[frame]
        if frame:
            try:
                tf = self.buffer.lookup_transform('base_link', frame, Time())
            except Exception:
                pass
            else:
                t = tf.transform.translation
                pose = (t.x, t.y, t.z, yaw_of(tf.transform.rotation))
                self.sensor_poses[frame] = pose
                return pose
        if which not in self.sensor_warned:
            self.get_logger().warn(
                '拿不到 %s 雷达的 base_link→%s TF，先用参数兜底'
                % (which, frame or '?'))
            self.sensor_warned.add(which)
        return (self.get_parameter('%s_x' % which).value,
                self.get_parameter('%s_y' % which).value,
                self.get_parameter('%s_dz' % which).value,
                0.0)

    # ──────────────────────── 主循环 ────────────────────────
    def on_timer(self):
        if not self.get_parameter('enabled').value:
            return
        now = time.monotonic()
        low = self.fresh_scan('low', now)
        high = self.fresh_scan('high', now)
        if low is None and high is None:
            self.get_logger().warn('两台雷达都没有新数据',
                                   throttle_duration_sec=5.0)
            return

        # ── ① 车在 map 里的位姿（从 TF 拿）──
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', Time())
        except Exception as e:
            self.get_logger().warn('拿不到 TF map→base_link：%s' % e,
                                   throttle_duration_sec=5.0)
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        rz = tf.transform.translation.z
        yaw = yaw_of(tf.transform.rotation)

        base_link_h = self.get_parameter('base_link_h').value
        surface_z = rz - base_link_h          # 轮子着地面 = 车所在的台面
        level_tol = self.get_parameter('level_tol').value
        up_dz = self.get_parameter('up_level_dz').value

        # 两台雷达的位姿（base_link 系）→ 世界系
        rp = {}
        for which, msg in (('low', low), ('high', high)):
            if msg is None:
                rp[which] = None
                continue
            dx, dy, dz, syaw = self.sensor_pose(which, msg)
            sx = rx + dx * math.cos(yaw) - dy * math.sin(yaw)
            sy = ry + dx * math.sin(yaw) + dy * math.cos(yaw)
            rp[which] = (sx, sy, rz + dz, yaw + syaw)

        # ── ② 逐个建塔处判断 ──
        verdicts = []
        for s in self.spots:
            # 建塔处相对车的高差：同层 / 高一层（L2）/ 其它
            dz = s['surface_z'] - surface_z
            if abs(dz) <= level_tol:
                kind = 'same'
            elif abs(dz - up_dz) <= level_tol:
                kind = 'up'
            else:
                verdicts.append((s['name'], NA, None, None, '不同层/在下方'))
                continue

            # 两条高度带各问两台雷达一遍，再融合
            obs = {}
            for band in ('base', 'mid'):
                cand = []
                for which, msg in (('low', low), ('high', high)):
                    if msg is None or rp[which] is None:
                        continue
                    cand.append(self.observe(
                        which, msg, rp[which], s, band))
                obs[band] = self.pick(cand)

            state = self.classify(kind, obs['base'], obs['mid'])
            verdicts.append((s['name'], state, obs['base'], obs['mid'], kind))

        # ── ③ 输出 ──
        self.pub.publish(String(data=self.report(verdicts, rx, ry, rz)))

        self.tick += 1
        key = tuple(v[1] for v in verdicts)
        log_every = self.get_parameter('log_every').value
        if key != self.last_verdicts or self.tick % log_every == 0:
            tag = '★ 有变化! ' if key != self.last_verdicts else ''
            self.get_logger().info('%s%s' % (tag, self.table(
                verdicts, rx, ry, rz, surface_z, rp)))
        self.last_verdicts = key

    # ──────────────────────── 单带观测 ────────────────────────
    def observe(self, which, msg, sensor, spot, band):
        """一台雷达看一个建塔处的一条高度带。

        band = 'base' / 'mid'
        高度带以【建塔处自己的台面】为基准：
            base = [surface, surface + 0.35]
            mid  = [surface + 0.35, surface + 0.70]
        """
        sx, sy, sz, syaw = sensor
        surface = spot['surface_z']
        lo = surface + (0.0 if band == 'base' else self.block)
        hi = lo + self.block

        margin = self.get_parameter('band_margin').value
        if not (lo + margin <= sz <= hi - margin):
            return Observation(None, height=False)

        # 车在 L1、看 L2 时，low 在 L2 台面以下：高度判据已经把它挡掉了 ✓
        # 反过来车在 L2、看 L1 的"下一层"在上面就整条跳过了（kind != same/up）

        # 方位/距离必须【从传感器算】，不能从车中心算：
        # low 在车头前方 0.42 m，从车中心算方位会差 20° 以上 ✗
        dx = spot['x'] - sx
        dy = spot['y'] - sy
        dist_center = math.hypot(dx, dy)
        bearing = norm_angle(math.atan2(dy, dx) - syaw)

        # 低雷达只有前方视场；后方是车壳，不是"没东西" ✗
        fov = math.radians(self.get_parameter('low_fov_deg').value / 2.0)
        if which == 'low' and abs(bearing) > fov:
            return Observation(None, fov=False)

        if dist_center <= self.block_half:
            return Observation(None)       # 已经站在建塔处里了，没法判

        # 方块近侧面：轴对齐的方盒，沿视线方向到表面的距离
        #   正对着看 → a；45° 斜看 → a/max(|ux|,|uy|) = 1.414a
        # 用中心距离减它 = 雷达该看到的距离 ✓
        ux, uy = dx / dist_center, dy / dist_center
        face = dist_center - self.block_half / max(abs(ux), abs(uy))

        half_w = math.radians(self.get_parameter('half_width_deg').value)
        tol = self.get_parameter('dist_tol').value
        min_pts = self.get_parameter('min_points').value
        occl = self.get_parameter('occlusion_check').value

        cnt, near, closer = self.count_returns(msg, bearing, half_w, face, tol)

        if cnt >= min_pts:
            return Observation(True, cnt, near, closer, face=face)
        if occl and closer is not None:
            return Observation(None, cnt, near, closer, face=face)
        return Observation(False, cnt, near, closer, face=face)

    def count_returns(self, msg, bearing, half_w, dist, tol):
        """在 [bearing ± half_w] 的扇区里数回波。

        返回 (cnt, near, closer)：
            cnt    —— 距离落在 [dist ± tol] 里的点数 → 说明"看到这块了"
            near   —— 这些点里最近的（只用于日志）
            closer —— 角度在扇区里、但距离【比 dist − tol 更近】的最近距离。
                      返回它 = 中途有东西挡着 → 必须报【未知】，不能报没有 ✗
        """
        lo, hi = dist - tol, dist + tol
        cnt = 0
        near = None
        closer = None
        for i, r in enumerate(msg.ranges):
            # ① 滤掉无效值：.inf = 这个方向没打到东西
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            # ② 这个点在第几度
            a = msg.angle_min + i * msg.angle_increment
            # ③ 和目标的夹角（归一化后再比，否则跨 ±180° 会漏）
            if abs(norm_angle(a - bearing)) > half_w:
                continue
            # ④ 分类
            if lo <= r <= hi:                   # A：就是它
                cnt += 1
                if near is None or r < near:
                    near = r
            elif r < lo:                        # B：中途有东西挡着
                if closer is None or r < closer:
                    closer = r
        return cnt, near, closer

    # ──────────────────────── 融合与判定 ────────────────────────
    @staticmethod
    def pick(objs):
        """两条高度带各自的多次观测（low/high 各一次）融合成一条。

        规则（保守优先）：
          · 任何一台【看到】了 → 就认为有（距离窗口很窄，误报概率低）
          · 都没看到、但至少一台【确定没有】→ 没有
          · 全都【被挡/看不见】→ None（未知）
        """
        vals = [o for o in objs if o is not None]
        if not vals:
            return None
        for o in vals:
            if o.seen is True:
                return o
        for o in vals:
            if o.seen is False:
                return o
        # 全是被挡/看不见：优先返回"高度对、但被挡"的那条，
        # 日志里才能说清真正原因（而不是被另一台雷达的"高度不覆盖"盖住）
        for o in vals:
            if o.height and o.fov:
                return o
        return vals[0]

    @staticmethod
    def classify(kind, base, mid):
        """(kind, 塔基观测, 中段观测) → 状态。

        kind='same'：同层，两条带都该看到
        kind='up'  ：建塔处比车高一层（L2），只有塔基那一条带够得着
        """
        b = None if base is None else base.seen
        m = None if mid is None else mid.seen

        if kind == 'up':
            if b is True:
                return BASE
            if b is False:
                return EMPTY
            return UNKNOWN

        if m is True:
            return FULL
        if m is False:
            if b is True:
                return BASE
            if b is False:
                return EMPTY
        return UNKNOWN

    # ──────────────────────── 输出 ────────────────────────
    def report(self, verdicts, rx, ry, rz):
        """给程序读的一行。

        取值：空 / 塔基 / 满 / 未知 / —
        括号里是 (塔基回波数/中段回波数)，"-" = 看不到（不是 0！）
        """
        parts = []
        for name, state, base, mid, _kind in verdicts:
            if state == NA:
                parts.append('%s=%s' % (name, NA))
                continue
            parts.append('%s=%s(%s/%s)' % (
                name, state,
                self._cnt(base), self._cnt(mid)))
        return 'x=%.3f y=%.3f z=%.3f %s' % (rx, ry, rz, ' '.join(parts))

    @staticmethod
    def _cnt(obs):
        if obs is None or obs.seen is None:
            return '-'
        return str(obs.cnt)

    def table(self, verdicts, rx, ry, rz, surface_z, rp):
        """给人看的日志（多行）。"""
        lines = ['车 map(%.3f, %.3f, %.3f)  台面 z=%.3f（%s）'
                 % (rx, ry, rz, surface_z,
                    'L1' if abs(surface_z - 0.60) < 0.15 else
                    'L2' if abs(surface_z - 0.90) < 0.15 else '?')]
        for which in ('low', 'high'):
            if rp[which] is None:
                lines.append('  %-4s 雷达  没有数据' % which)
            else:
                x, y, z, _ = rp[which]
                lines.append('  %-4s 雷达  map(%.2f, %.2f)  世界 z=%.3f'
                             % (which, x, y, z))

        for s, (name, state, base, mid, kind) in zip(self.spots, verdicts):
            d = s['desc'][:4]
            b = self._obs_txt(base)
            m = self._obs_txt(mid)
            if state == NA:
                lines.append('  %-8s %-4s  —（不同层，跳过）' % (name, d))
                continue
            tag = {EMPTY: '空位 → 放塔基',
                   BASE: ('已有塔基 → 放中段' if kind == 'same'
                          else 'L2 已有一层'),
                   FULL: '两层都有 → 不可放',
                   UNKNOWN: '看不全 → 未知，别放'}[state]
            lines.append('  %-8s %-4s  塔基 %-12s 中段 %-12s → 【%s】%s'
                         % (name, d, b, m, state, tag))
        return '\n'.join(lines)

    @staticmethod
    def _obs_txt(obs):
        if obs is None:
            return '-'
        if not obs.height:
            return '高度不覆盖'
        if not obs.fov:
            return '视场外'
        if obs.seen is True:
            if obs.face is None:
                return '%d 点 / %.3f m' % (obs.cnt, obs.near or 0.0)
            return '%d 点 / %.3f m（期望 %.3f，差 %+.0f mm）' % (
                obs.cnt, obs.near, obs.face,
                (obs.near - obs.face) * 1000.0)
        if obs.seen is False:
            if obs.face is None:
                return '无回波'
            return '无回波（期望 %.3f m）' % obs.face
        if obs.face is None:
            return '被挡 %.3f m' % (obs.closer or 0.0)
        return '被挡 %.3f m（目标面 %.3f）' % (obs.closer, obs.face)


def main():
    rclpy.init()
    node = TowerCheck()
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
