#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""往建塔处放方块（测试用，直接调 Gazebo，不经过机械臂）。

用法
    # 位置 [dx dy yaw_deg count]
    ros2 run mini_car_control place_block L1_W_M
    ros2 run mini_car_control place_block L1_W_M 0.05 0 15 1
    ros2 run mini_car_control place_block L1_W_M 0.10 0 45 2

    # 也可以直接跑源码（没 colcon build 时用这个）
    python3 robots/src/mini_car_control/mini_car_control/place_block.py L1_W_M 0.05 0 15 1

参数
    spot    建塔处名字（tower_spots.yaml 里的 name）
    dx,dy   方块中心相对建塔处中心的偏移，米，场地坐标轴（+x 东 / +y 北）
    yaw     方块绕 Z 的旋转，度
    count   这次放几块（往上叠；每个建塔处最多 2 块；0 = 只清空）

选项
    --team red|blue   用哪一队的 map 坐标（默认 red）
    --layer auto|0|1  auto=查 Gazebo 现有几块；0=强制塔基；1=强制中段
    --clear           放之前先把该建塔处附近的测试方块删掉
    --dry-run         只打印要执行的命令，不真放
    --world           世界名（默认 nuwa_field）
    --settle          两块之间的等待秒数（默认 0.4）
    --config/--block  坐标清单 / 方块 SDF 路径

约定
    z 不用你算：塔基中心 = 台面 + 0.175，中段 = 台面 + 0.525（方块 0.35）。
    L1 台面 0.600 → 塔基 0.776 / 中段 1.126
    L2 台面 0.900 → 塔基 1.076 / 中段 1.426
    ⚠ dx,dy 是【场地坐标轴】的偏移，不是“沿视线/垂直视线”。
"""

import argparse
import math
import os
import re
import subprocess
import sys
import time

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(_HERE))))
DEFAULT_CONFIG = os.path.join(
    _ROOT, 'robots/src/mini_car/config/tower_spots.yaml')
DEFAULT_BLOCK = os.path.join(
    _ROOT, 'robots/src/mini_car/models/block.sdf')
DEFAULT_WORLD = 'nuwa_field'

BLOCK_H = 0.35
BLOCK_HALF = 0.175
# “这个方块属于这个建塔处”的半径。建塔处 0.5，方块 0.35，
# 偏 0.2 m 也算同一个；相邻建塔处最近 2.75 m，不会误伤。
NEAR_R = 0.55
# --clear 只删这些前缀的模型，别把场地/车/道具误删 ✗
BLOCK_PREFIXES = ('blk_', 'br_', 'pub_', 'test_', 'tmp_', 'tall_', 'block_')


def sh(cmd, timeout=15):
    """跑一条命令，返回 stdout+stderr。

    ⚠ 和 br_mission 里一样：不能同时用 capture_output 和 stderr=STDOUT，
      否则会抛 ValueError，被吞掉后看起来像“命令没输出”✗
    """
    try:
        r = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE,
                           text=True, stderr=subprocess.STDOUT, timeout=timeout)
        return r.stdout or ''
    except Exception as e:
        return '【命令执行失败】%s: %s' % (type(e).__name__, e)


def load_spots(path, team):
    with open(path, encoding='utf-8') as f:
        d = yaml.safe_load(f)
    if team not in ('red', 'blue') or team not in d:
        raise ValueError('team 只认 red/blue，收到 %r' % team)
    origin = tuple(d[team]['origin_field'])
    spots = {s['name']: s for s in d[team]['spots']}
    return origin, spots


def pose_entities(world):
    """读 Gazebo 的 pose/info，返回 [(name, x, y, z), …]。

    gz topic -e 的输出是 protobuf 文本，形如：
        name: "blk_l1_w_m_0_12345"
        id: 248
        position { x: -2.75 y: 0 z: 0.776 }
    这里把所有顶层模型都抓出来，后面按“离建塔处多远”筛。
    """
    text = sh('gz topic -e -t /world/%s/pose/info -n 1' % world, timeout=8)
    if not text.strip():
        return []
    pat = (r'name:\s*"([^"]+)"\s+id:\s*\d+\s+position\s*\{\s*'
           r'x:\s*([-\d.eE+]+)\s+y:\s*([-\d.eE+]+)\s+z:\s*([-\d.eE+]+)')
    return [(m.group(1), float(m.group(2)), float(m.group(3)),
             float(m.group(4))) for m in re.finditer(pat, text)]


def near(entities, cx, cy, r=NEAR_R, min_z=None, prefixes=None):
    out = []
    for name, x, y, z in entities:
        if math.hypot(x - cx, y - cy) > r:
            continue
        if min_z is not None and z < min_z:
            continue
        if prefixes and not name.startswith(prefixes):
            continue
        out.append((name, x, y, z))
    return out


def remove_model(world, name):
    req = 'name: "%s" type: 2' % name       # type 2 = MODEL
    # ⚠ --req 必须用【单引号】包住：req 里带 name: "..." 的双引号，
    #   外面再用双引号的话 shell 会把 name 的双引号吃掉 ✗
    out = sh("gz service -s /world/%s/remove --reqtype gz.msgs.Entity "
             "--reptype gz.msgs.Boolean --timeout 3000 --req '%s'"
             % (world, req), timeout=8)
    return 'data: true' in out or 'true' in out.lower()


def main():
    ap = argparse.ArgumentParser(
        description='往建塔处放方块（测试用）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='例：\n'
               '  place_block L1_W_M\n'
               '  place_block L1_W_M 0.05 0 15 1\n'
               '  place_block L1_W_M 0.10 0 45 2\n'
               '  place_block L1_E_M --clear --dry-run\n')
    ap.add_argument('spot', help='建塔处名字，如 L1_W_M / L2_SW')
    ap.add_argument('dx', nargs='?', type=float, default=0.0,
                    help='x 偏移（米，场地坐标轴）')
    ap.add_argument('dy', nargs='?', type=float, default=0.0,
                    help='y 偏移（米，场地坐标轴）')
    ap.add_argument('yaw', nargs='?', type=float, default=0.0,
                    help='方块旋转（度）')
    ap.add_argument('count', nargs='?', type=int, default=1,
                    help='这次放几块（默认 1，最多叠到 2）')
    ap.add_argument('--team', default='red', choices=('red', 'blue'))
    ap.add_argument('--world', default=DEFAULT_WORLD)
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    ap.add_argument('--block', default=DEFAULT_BLOCK)
    ap.add_argument('--layer', default='auto', choices=('auto', '0', '1'),
                    help='auto=查现有块数；0=强制塔基；1=强制中段')
    ap.add_argument('--clear', action='store_true',
                    help='放之前删掉该建塔处附近的测试方块')
    ap.add_argument('--clear-only', action='store_true',
                    help='只删该建塔处附近的测试方块，不放')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--settle', type=float, default=0.4,
                    help='两块之间的等待秒数')
    args = ap.parse_args()
    if args.clear_only:
        args.clear = True
        args.count = 0

    if args.count < 0:
        ap.error('count 不能为负；0 = 只清空/不放')
    origin, spots = load_spots(args.config, args.team)
    if args.spot not in spots:
        ap.error('没有建塔处 %r；可选：%s'
                 % (args.spot, ', '.join(sorted(spots))))
    s = spots[args.spot]
    cx = s['x'] + origin[0]
    cy = s['y'] + origin[1]
    surface = float(s['surface_z'])

    print('建塔处 %-8s map(%7.3f, %7.3f)  世界(%7.3f, %7.3f)  台面 z=%.3f'
          % (args.spot, s['x'], s['y'], cx, cy, surface))
    print('偏移 dx=%+.3f dy=%+.3f  方块 yaw=%+.1f°  要放 %d 块'
          % (args.dx, args.dy, args.yaw, args.count))

    entities = pose_entities(args.world)

    if args.clear:
        victims = near(entities, cx, cy, prefixes=BLOCK_PREFIXES)
        if not victims:
            print('--clear：附近没有可删的测试方块')
        for name, x, y, z in victims:
            if args.dry_run:
                print('  [dry] remove %s' % name)
            else:
                ok = remove_model(args.world, name)
                print('  remove %-28s %s' % (name, 'OK' if ok else '失败'))
        if victims and args.dry_run:
            # dry-run：假装已经删掉，后面的“现有几块”和放置计划才好看 ✓
            gone = {v[0] for v in victims}
            entities = [e for e in entities if e[0] not in gone]
        elif victims:
            time.sleep(0.3)
            entities = pose_entities(args.world)

    on_spot = near(entities, cx, cy, min_z=surface + 0.05)
    print('现有方块：%d 块%s'
          % (len(on_spot),
             ('（' + ', '.join('%s@z%.2f' % (n, z)
                               for n, _x, _y, z in on_spot) + '）')
             if on_spot else ''))

    if args.layer == 'auto':
        start = len(on_spot)
    else:
        start = int(args.layer)
        if on_spot and start != len(on_spot):
            print('⚠ --layer=%d 是强制的，现有 %d 块；可能重叠'
                  % (start, len(on_spot)))

    if start >= 2:
        print('这个建塔处已经满了（2 块），不再放')
        return 0
    count = min(args.count, 2 - start)
    if count < args.count:
        print('⚠ 最多再放 %d 块（建塔处总共 2 块）' % count)
    if count == 0:
        print('count=0：只做了查询/清空，不放方块')
        return 0

    yaw_rad = math.radians(args.yaw)
    for i in range(count):
        layer = start + i
        z = surface + BLOCK_HALF + layer * BLOCK_H
        name = 'blk_%s_%d_%05d' % (
            args.spot.lower(), layer, time.time_ns() // 1000000 % 100000)
        cmd = ('ros2 run ros_gz_sim create -file %s -name %s '
               '-x %.4f -y %.4f -z %.4f -Y %.6f'
               % (args.block, name, cx + args.dx, cy + args.dy, z, yaw_rad))
        what = '塔基' if layer == 0 else '中段'
        print('  ▸ %s  layer=%d  z=%.4f  name=%s'
              % (what, layer, z, name))
        if args.dry_run:
            print('    [dry] %s' % cmd)
        else:
            out = sh(cmd, timeout=30)
            if 'OK creation' not in out:
                print('    ✗ 创建失败：%s' % out.strip()[-200:])
                return 1
            print('    OK')
            if i < count - 1:
                time.sleep(args.settle)

    return 0


if __name__ == '__main__':
    sys.exit(main())
