#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BR 建塔任务流程（仿真演示）。

做什么
    走完一个完整的建塔循环，每块方块一趟：

        ① 开到【检测点】                → 扫两个朝向，拿到同层 4 个建塔处的状态
        ② 选一个能放的（空/塔基）        → 顺带知道该放塔基还是中段
        ③ 开到那个建塔处的【作业点】     → 再扫一次确认（这几秒里状态可能变！）
        ④ 状态和检测点一致 → 放
           已被占           → 回 ②
           未知             → 报告"视线被挡，需要挪位置"
        ⑤ 回 ① 继续下一块

⚠ 这个脚本里哪部分是真机可用的
    【可搬】① ② ③ 的判断与决策 —— 就是 tower_check 那套 + 这里的选址逻辑
    【替身】"开车"   → 仿真里用 teleport（直接 set_pose），不模拟真实行驶
            "放置"   → 仿真里用 spawn 一个方块代替夹爪
    真机上这两步分别由【导航】和【机械臂】负责，本脚本只保留决策骨架 ✓

✅ 放【塔基】还是【中段】不再靠问 Gazebo
    v3 的 tower_check 用两台 2D 雷达分高度切【塔基】和【中段】：
        空   → 放塔基
        塔基 → 放中段
        满   → 换一个
    本脚本直接读这三个状态决定放什么，和真机走同一条决策路径 ✓
    （以前只有一台雷达，分不出空位和只有塔基，只能问 Gazebo —— 已删掉）

⚠ 检测点看不全的建塔处
    两种「未知」：
      · 中段回波数 = '0'：确定没有中段，只是塔基被挡（典型：L2 平台）
        → 一定记为「待定」候选，去作业点再分空位/塔基
      · 两个都是 '-'：整条视线被挡（典型：被 L2 上的塔挡住）
        → 默认跳过；加 -p confirm_blocked:=true 才去确认
    ⚠ 只做公共位时，两个 L1 公共位常是第二种，建议同时开：
        -p only_spots:=L1_W_M,L1_E_M,L2_SW,L2_SE,L2_NW,L2_NE \
        -p confirm_blocked:=true

用法
    # 终端1、2
    ros2 run mini_car_control odom_to_tf --ros-args -p team:=red
    ros2 run mini_car_control tower_check --ros-args -p team:=red

    # 终端3
    ros2 run mini_car_control br_mission --ros-args -p team:=red -p level:=L1 -p tasks:=3

    # 只做公共位；被完全挡住的点也去作业点确认
    ros2 run mini_car_control br_mission --ros-args -p team:=red -p level:=L1 \
      -p tasks:=1 -p only_spots:=L1_W_M,L1_E_M -p confirm_blocked:=true
"""

import math
import os
import re
import subprocess
import time

import rclpy
import tf2_ros
import yaml
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

WORLD = 'nuwa_field'
ROBOT = 'mini_car'

# 路径从本文件位置推出来，不写死绝对路径 ✓
#   本文件在  <项目根>/robots/src/mini_car_control/mini_car_control/br_mission.py
#   往上数 4 层就是项目根
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(_HERE))))
CONFIG = os.path.join(_ROOT, 'robots/src/mini_car/config/tower_spots.yaml')
BLOCK_SDF = os.path.join(_ROOT, 'robots/src/mini_car/models/block.sdf')
BLOCK = 0.35                # 塔基/中段边长
BASE_LINK_H = 0.30          # base_link 在台面上方多高（用来算传送高度）
# tower_check 的状态（v3 双雷达）
EMPTY, BASE, FULL, UNKNOWN, NA = '空', '塔基', '满', '未知', '—'
# 要放的方块
BLK_BASE, BLK_MID = '塔基', '中段'


def sh(cmd, timeout=15):
    """跑一条 shell 命令，返回 stdout+stderr 合并。

    ⚠ 两个坑，都踩过：
      ① `ros2 run` / `gz service` 的日志走【stderr】，只抓 stdout 会拿到空串 ✗
         所以要把 stderr 合并进来
      ② ⚠ 但【不能】用 capture_output=True 再配 stderr=STDOUT ——
         这两个一起用会抛 ValueError("stdout and stderr arguments may not be
         used with capture_output") ✗ 而它会被下面的 except 吞掉，
         表现成"命令返回空串"，看起来像命令没输出，其实是【根本没执行】✗
         正确写法：只给 stdout=PIPE + stderr=STDOUT ✓
    """
    try:
        r = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE,
                           text=True, stderr=subprocess.STDOUT, timeout=timeout)
        return r.stdout or ''
    except Exception as e:
        return '【命令执行失败】%s: %s' % (type(e).__name__, e)


class BrMission(Node):
    def __init__(self):
        super().__init__('br_mission')

        self.declare_parameter('team', 'red')
        self.declare_parameter('level', 'L1')       # 先干哪一层
        self.declare_parameter('tasks', 3)          # 一共放几块
        self.declare_parameter('config', CONFIG)
        self.declare_parameter('block_sdf', BLOCK_SDF)
        self.declare_parameter('settle', 0.6)       # 传送后等雷达数据刷新
        # use_nav=False → 【仿真替身】直接传送（快，只看决策流程）
        # use_nav=True  → 真的开过去（要另外跑 goto_pose 节点）
        self.declare_parameter('use_nav', False)
        self.declare_parameter('nav_timeout', 90.0)
        # 这一趟车上带的是什么：'auto' / '塔基' / '中段'
        #   auto  → 先找空位放塔基，没有再找有塔基的放中段
        #   指定  → 只找对应状态（真机按 TR 送来的方块类型决定）
        self.declare_parameter('want', 'auto')
        # 只做指定的建塔处，逗号分隔；空 = 全部。
        # 只做 6 个公共位：
        #   -p only_spots:=L1_W_M,L1_E_M,L2_SW,L2_SE,L2_NW,L2_NE
        self.declare_parameter('only_spots', '')
        # 检测点被完全挡住（未知(-/-)）的点，要不要也去作业点确认？
        #   false（默认）→ 跳过，省一趟
        #   true          → 去，到了再用低雷达分空位/塔基
        #   ⚠ L2 平台会挡住两个 L1 公共位的低雷达，不开这个会漏掉
        self.declare_parameter('confirm_blocked', False)

        self.team = self.get_parameter('team').value
        self.level = self.get_parameter('level').value
        self.tasks = int(self.get_parameter('tasks').value)
        self.settle = float(self.get_parameter('settle').value)
        self.want = self.get_parameter('want').value
        self.only_spots = {x.strip() for x in
                           self.get_parameter('only_spots').value.split(',')
                           if x.strip()}
        self.confirm_blocked = bool(
            self.get_parameter('confirm_blocked').value)
        self.block_sdf = self.get_parameter('block_sdf').value
        self.use_nav = bool(self.get_parameter('use_nav').value)

        with open(self.get_parameter('config').value, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        self.geom = cfg['geometry']
        self.origin = tuple(cfg[self.team]['origin_field'])
        self.vantage = cfg[self.team]['vantage']
        self.spots = {s['name']: s for s in cfg[self.team]['spots']}
        self.surface_z = {s['name']: s['surface_z'] for s in self.spots.values()}

        self.status = None          # 最近一帧 /tower_check/status
        self.seq = 0                # 收到过几帧（用来等"新的"一帧）
        self.create_subscription(String, '/tower_check/status', self._on_status, 10)

        # 查"我自己在哪" —— 给绕行规划当起点（use_nav=True 时才用得到）
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.nav_state = None       # goto_pose 报的状态（"行驶中"/"已到达"/…）
        self.create_subscription(String, '/goto_pose/status', self._on_nav, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info('BR 任务开始：队伍=%s  层=%s  要放 %d 块  开车方式=%s'
                               % (self.team, self.level, self.tasks,
                                  '真开过去' if self.use_nav else '传送(仿真替身)'))
        self.get_logger().info('只做=%s  去确认完全被挡的点=%s'
                               % (sorted(self.only_spots) or '全部',
                                  self.confirm_blocked))

    def _on_nav(self, msg):
        self.nav_state = msg.data

    # ──────────────────────── 基础设施 ────────────────────────
    def _on_status(self, msg):
        self.status = msg.data
        self.seq += 1

    def wait_status(self):
        """等一帧【新的】检测结果并解析。

        返回 {名字: (状态, 塔基回波数, 中段回波数)}。
        回波数 '-' = 这个高度带没看到（不是 0），'0' = 看到了、确实没有。

        传送完车之后必须重新等一帧 —— 旧的那帧说的是旧位置 ✗
        """
        want = self.seq + 2                       # 多等一帧，确保是传送后的
        t0 = time.time()
        while self.seq < want:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > 8.0:
                self.get_logger().warn('等检测结果超时')
                return {}
        out = {}
        for tok in self.status.split():
            # 形如 L1_W_S=塔基(4/-) 或 L2_SW=—
            m = re.match(r'^([A-Za-z0-9_]+)=([^()]+)(?:\(([^)]*)\))?$', tok)
            if not (m and m.group(1) in self.spots):
                continue
            b, _, mid = (m.group(3) or '-/-').partition('/')
            # (状态, 塔基回波数, 中段回波数)；'-' = 看不到，'0' = 确定没有
            out[m.group(1)] = (m.group(2), b or '-', mid or '-')
        return out

    @staticmethod
    def merge_status(a, b):
        """两个朝向各扫一次的结果合并成一个。

        低雷达装在车头，只有前方干净 —— 在一个朝向上永远有两个建塔处
        在车身后方，tower_check 会对它们报"—"（不是"空"✗）。
        所以检测点掉头 180° 再扫一次，两个朝向互补。

        合并规则（保守优先，宁可多跑一趟也不放错）：
            满 > 塔基 > 空；确定值 > 未知 > —
        """
        rank = {FULL: 3, BASE: 2, EMPTY: 1}

        def rk(item):
            return rank.get(item[0], 0)

        def info(item):
            # 有几个高度带的回波数是确定的（不是 '-'）
            return (item[1] != '-') + (item[2] != '-')

        out = {}
        for n in set(a) | set(b):
            ia = a.get(n, (NA, '-', '-'))
            ib = b.get(n, (NA, '-', '-'))
            ra, rb = rk(ia), rk(ib)
            if ra > rb:
                out[n] = ia
            elif rb > ra:
                out[n] = ib
            elif ra:
                out[n] = ia                       # 两个朝向都确定，结果应一致
            elif UNKNOWN in (ia[0], ib[0]):
                # 都不确定时选信息多的那个：至少能知道"中段确定没有"
                out[n] = ia if info(ia) >= info(ib) else ib
            else:
                out[n] = (NA, '-', '-')
        return out

    def scan_vantage(self, v, surface_z):
        """在检测点扫两个朝向（相差 180°）并合并。

        高雷达 360°，一个朝向就够；低雷达只有前方 ~190°，掉头一次才能
        把同层 4 个建塔处的【塔基】都看全 ✓
        """
        st = self.wait_status()
        back = (float(v['yaw_deg']) + 180.0) % 360.0
        if self.goto(v['x'], v['y'], surface_z, back):
            st2 = self.wait_status()
            st = self.merge_status(st, st2)
        return st

    def teleport(self, x, y, surface_z, yaw_deg):
        """【仿真替身】把车直接搬到 (x,y) 并转到 yaw —— 不模拟行驶。

        ⚠ 传进来的是 map 坐标，而 Gazebo 里的车在【场地系】
          → 必须先 map + origin 换算回场地系，否则会传送到错的地方 ✗
          （真机上导航模块吃的是 map 坐标，不需要这步换算）

        真机对应：导航模块把车开到指定位姿。
        """
        fx = x + self.origin[0]              # map → 场地系
        fy = y + self.origin[1]
        yaw = math.radians(yaw_deg)
        # 请求体里模型名要用双引号（protobuf 文本格式的字符串）✓
        req = ('name: "%s" position: {x: %.4f, y: %.4f, z: %.4f} '
               'orientation: {z: %.6f, w: %.6f}'
               % (ROBOT, fx, fy, surface_z + BASE_LINK_H,
                  math.sin(yaw / 2), math.cos(yaw / 2)))
        # ⚠ --req 的值必须用【单引号】包起来 ✗ 不能用双引号：
        #   用双引号的话，里面那对表示模型名的双引号会被 shell 吃掉，
        #   protobuf 解析器收到 `name: mini_car`（没有引号）→ 报
        #   "Expected string, got: mini_car" ✗
        sh("gz service -s /world/%s/set_pose --reqtype gz.msgs.Pose "
           "--reptype gz.msgs.Boolean --timeout 3000 --req '%s'" % (WORLD, req))

        # ⚠ 传送后必须【边等边 spin】，不能光 sleep：
        #   光 sleep 的话，这 0.6 秒里发布的检测结果全堆在订阅队列里，
        #   等会儿 wait_status 一 spin 就吐出来 —— 但那是【传送之前】的状态 ✗
        #   （车还在 L1 时的结果，拿到 L2 来用 → 全是 "—"）
        t0 = time.time()
        while time.time() - t0 < self.settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.seq = 0                      # 计数清零：后面等的都是传送后的

    # L2 平台在场地正中（3x3），车在 L1 时开不上去。
    # 场地系里 |x| 和 |y| 都小于这个值 = L2 附近，直线要绕开。
    #   1.98 = L2 半宽 1.5 + 车的包围圆半径 0.48
    L2_LIM = 1.98

    def _in_l2(self, mx, my):
        """map 坐标 → 场地坐标，看是不是在 L2 的禁区里。"""
        fx, fy = mx + self.origin[0], my + self.origin[1]
        return abs(fx) < self.L2_LIM and abs(fy) < self.L2_LIM

    def _seg_ok(self, a, b):
        n = 24
        return not any(self._in_l2(a[0] + (b[0] - a[0]) * i / n,
                                   a[1] + (b[1] - a[1]) * i / n)
                       for i in range(n + 1))

    def _route(self, a, b):
        """给两点之间规划路线。

        为什么不能直着走：L2 是场地正中的 0.3 m 平台，车开不上去 ✗
        从检测点到作业点的直线经常正好穿过它。

        绕法：走 L 形，拐角选在两个坐标互换的位置里，挑两条腿都不穿 L2 的 ✓
            a ────┐                a
                  │        或      │
                  └── b            └──── b
        """
        if self._seg_ok(a, b):
            return [b]
        for m in ((b[0], a[1]), (a[0], b[1])):
            if self._seg_ok(a, m) and self._seg_ok(m, b):
                return [m, b]
        return [b]          # 绕不开就直着走（会卡住，但不静默失败）

    def _drive_via(self, pts, yaw_deg):
        """依次开到每个路点，最后一段用目标朝向。"""
        for i, (px, py) in enumerate(pts):
            last = (i == len(pts) - 1)
            if last:
                yaw = yaw_deg
            else:                       # 中间路点：车头朝下一段的方向
                nx, ny = pts[i + 1]
                yaw = math.degrees(math.atan2(ny - py, nx - px))
            if not self._drive_to(px, py, yaw):
                return False
        return True

    def _drive_to(self, x, y, yaw_deg):
        """真开过去：发 /goal_pose 给 goto_pose，等它报"已到达"。

        真机上这个接口【一模一样】—— 只是 goto_pose 拿的位姿来自 Mid-360 定位，
        而不是仿真的里程计 ✓
        """
        yaw = math.radians(yaw_deg)
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)

        self.nav_state = None
        self.goal_pub.publish(m)
        t0 = time.time()
        to = float(self.get_parameter('nav_timeout').value)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.nav_state == '已到达':
                # 到位后停一下，让雷达数据刷新（和传送那条路一样）
                t1 = time.time()
                while time.time() - t1 < self.settle:
                    rclpy.spin_once(self, timeout_sec=0.05)
                self.seq = 0
                return True
            if self.nav_state == '超时放弃':
                return False
            if time.time() - t0 > to:
                self.get_logger().warn('  走定点超时（%.0f 秒）' % to)
                return False
        return False

    def _current_map(self):
        """我现在在 map 里哪儿？给路线规划当起点用。"""
        try:
            tf = self.tf_buf.lookup_transform('map', 'base_link', Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    def goto(self, x, y, surface_z, yaw_deg):
        """到指定位姿：use_nav=True 真开过去（会绕开 L2），False 传送（仿真替身）。"""
        if self.use_nav:
            a = self._current_map()
            pts = self._route(a, (x, y)) if a else [(x, y)]
            if len(pts) > 1:
                self.get_logger().info('   路线绕行 L2：%s → 目标'
                                       % ' → '.join('(%.2f, %.2f)' % p for p in pts[:-1]))
            return self._drive_via(pts, yaw_deg)
        self.teleport(x, y, surface_z, yaw_deg)
        return True

    def place(self, name, what):
        """【仿真替身】放一块 —— 真机对应：机械臂动作。

        what 由 tower_check 的状态决定（空→塔基 / 塔基→中段），
        不再问 Gazebo 现在叠了几块 ✓ 真机走的就是这条路径。
        """
        s = self.spots[name]
        fx = s['x'] + self.origin[0]              # map → 场地系
        fy = s['y'] + self.origin[1]
        n = 0 if what == BLK_BASE else 1
        z_face = s['surface_z'] + n * BLOCK
        z = z_face + BLOCK / 2
        model = 'br_%s_%d' % (name.lower(), n)
        self.get_logger().info('  ▸ 放一块【%s】到 %s，落地高度 z=%.3f'
                               % (what, name, z_face))
        out = sh('ros2 run ros_gz_sim create -file %s -name %s -x %.3f -y %.3f -z %.3f'
                 % (self.block_sdf, model, fx, fy, z), timeout=25)
        if 'OK creation' not in out:
            self.get_logger().warn('  创建返回异常：%s' % out.strip()[-120:])
        time.sleep(1.2)

    def candidates(self, same):
        """按状态挑能放的建塔处，返回 [(名字, 要放的方块), …]。

        状态 → 动作：
            空   → 放塔基
            塔基 → 放中段
            满 / 未知 / — → 不能放

        want='auto' 时先放塔基再放中段 —— 这样同一处会先长出一层，
        下一轮再补第二层，演示的是完整建塔流程 ✓
        """
        out = []
        for name, (state, _b, mid_cnt) in sorted(same.items()):
            if self.only_spots and name not in self.only_spots:
                continue
            if state == EMPTY and self.want in ('auto', BLK_BASE):
                out.append((name, BLK_BASE))
            elif state == BASE and self.want in ('auto', BLK_MID):
                out.append((name, BLK_MID))
            elif (state == UNKNOWN and self.want == 'auto'
                  and (mid_cnt == '0' or self.confirm_blocked)):
                # 看不清：中段确定没有（'0'）或被完全挡住（'-'）。
                # 前者先去作业点分空位/塔基；后者只有 confirm_blocked 才去，
                # 避免为“可能已经满了”的点白跑一趟 ✓
                out.append((name, '?'))
        order = {BLK_BASE: 0, BLK_MID: 1, '?': 2}
        out.sort(key=lambda item: order[item[1]])
        return out

    # ──────────────────────── 主流程 ────────────────────────
    def run(self):
        # 该层台面的【绝对世界高度】：从同层的建塔处读（它们都带 surface_z）
        #   L1 → 0.600    L2 → 0.900
        # ⚠ 别写成"比 L1 高多少" —— 那样 L2 会算成 0.3，把车塞进平台里 ✗
        sfc = next(s['surface_z'] for s in self.spots.values()
                   if s['layer'] == self.level)
        if self.use_nav:
            self.get_logger().info('%s 台面高度 %.3f（真开过去）' % (self.level, sfc))
        else:
            self.get_logger().info('%s 台面高度 %.3f → 车传送高度 %.3f'
                                   % (self.level, sfc, sfc + BASE_LINK_H))
        placed = 0
        fail_rounds = 0

        while rclpy.ok() and placed < self.tasks:
            # ① 开到检测点
            v = self.vantage[self.level]
            self.get_logger().info('① 到 %s 检测点 map(%.3f, %.3f)' % (self.level, v['x'], v['y']))
            if not self.goto(v['x'], v['y'], sfc, v['yaw_deg']):
                self.get_logger().warn('  到检测点失败，放弃')
                return

            # ② 扫两个朝向（低雷达只看前方，掉头补看背后两个）
            st = self.scan_vantage(v, sfc)
            same = {n: st[n] for n in st if n.startswith(self.level + '_')}
            if not same:
                self.get_logger().warn('没拿到同层建塔处的状态，重试')
                continue
            line = '  '.join('%s=%s' % (n, v[0]) for n, v in sorted(same.items()))
            self.get_logger().info('   同层状态：%s' % line)

            cand = self.candidates(same)
            if not cand:
                self.get_logger().warn('   %s 层没有能放的了（want=%s）'
                                       % (self.level, self.want))
                if any(v[0] == UNKNOWN for v in same.values()):
                    self.get_logger().warn('   （有「未知」——视线被挡，得换个位置看）')
                return

            # ③ 开到作业点
            name, what = cand[0]
            w = self.spots[name]['work']
            self.get_logger().info('② 选 %s（%s）→ 开到作业点 map(%.3f, %.3f) 朝向 %.0f°'
                                   % (name, '待定' if what == '?' else what,
                                      w['x'], w['y'], w['yaw_deg']))
            if not self.goto(w['x'], w['y'], sfc, w['yaw_deg']):
                self.get_logger().warn('  到作业点失败，放弃')
                return

            # ④ 再扫一次确认
            st2 = self.wait_status()
            verdict = st2.get(name, ('?', '-', '-'))[0]
            if what == '?':
                # 检测点只看得到"中段没有"，塔基被 L2 平台挡住；
                # 到作业点后低雷达正对着建塔处，这时才分得清空位/塔基 ✓
                if verdict == EMPTY:
                    what = BLK_BASE
                elif verdict == BASE:
                    what = BLK_MID
            expect = EMPTY if what == BLK_BASE else BASE
            self.get_logger().info('③ 确认 %s → 【%s】（期望 %s，放 %s）'
                                   % (name, verdict, expect,
                                      '待定' if what == '?' else what))

            if verdict == expect:
                self.place(name, what)
                placed += 1
                self.get_logger().info('✓ 完成 %d/%d\n' % (placed, self.tasks))
            elif verdict == UNKNOWN:
                self.get_logger().warn('  视线被挡，不能放 ✗ 需要挪个位置再看（本脚本先跳过）\n')
                fail_rounds += 1
            else:
                # 可能是对手抢先放了，也可能是状态和检测点看到的不一样
                # —— 处理方式一样：换一个 ✓
                self.get_logger().warn('  状态变了（被抢先/看错），换一个 ✗\n')
                fail_rounds += 1

            if fail_rounds > 6:
                self.get_logger().error('反复失败，退出')
                return

        self.get_logger().info('★ 任务完成：%s 层放了 %d 块' % (self.level, placed))


def main():
    rclpy.init()
    node = BrMission()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
