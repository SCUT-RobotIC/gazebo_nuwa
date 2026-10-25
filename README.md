# gazebo仿真
## 说明
## 目录

## 0. 环境预检（先看这个）

机器上有两个工作空间，`.bashrc` 先 source 了旧的
`~/gazebo_nuwa/robots/install`，所以**新终端默认会解析到旧包**。
改完 `~/work/nuwa_sim/robots/src` 后，每个终端都要补一句（必须排在旧包之后）：

```bash
source /opt/ros/humble/setup.bash
source ~/work/nuwa_sim/robots/install/setup.bash
bash ~/work/nuwa_sim/tools/check_env.sh
```

`check_env.sh` 会打印：

- `mini_car_control -> ~/work/nuwa_sim/robots/install/mini_car_control`（必须是这个）
- 可执行列表里要有 `tower_check`、`place_block`
- 当前节点/话题；仿真没起时话题为空是正常的

`~/work/nuwa_sim/robots/install` 是 `--symlink-install`：

- 改 `.py` **不用重新 build**，重新 source 都不用，直接生效
- 改 `setup.py` 的 **入口点**（比如新加 `place_block`）才需要
  `colcon build --packages-select mini_car_control --symlink-install`

⚠ 启动必须用**源码里的完整路径**：
`ros2 launch ~/work/nuwa_sim/robots/src/mini_car/launch/mini_car.launch.py`。
写成 `ros2 launch mini_car ...` 会解析到旧 gazebo_nuwa 的 `mini_car` 包
（新 install 里只有 `mini_car_control`，没有 `mini_car`）。

### 0.1 放方块后检测没反应？

先按顺序查这三件事：

```bash
# ① 解析到的是不是新包
bash ~/work/nuwa_sim/tools/check_env.sh

# ② tower_check 到底在不在跑
ros2 node list | grep tower_check
ros2 topic info /tower_check/status

# ③ 它订阅的是哪路扫描
ros2 node info /tower_check | grep -A2 "Subscribers"
```

- 第一行必须是 **`tower_check v3（双雷达）`**，并打印
  `低雷达 ← /scan_low   高雷达 ← /scan_high`。
- 如果**没有这行**，或者订阅的是 `/scan`：跑的是旧 gazebo_nuwa 包。
  新 launch 只发 `/scan_low` / `/scan_high`，旧包永远收不到 → 状态不变。
  重新 `source ~/work/nuwa_sim/robots/install/setup.bash` 后再启动。
- 如果 `/tower_check` 不在节点列表里：节点没起来（或起在别的终端已退出）。
- 节点在跑但只看终端：默认 `log_every=20` 是 2 秒一表；用
  `-p log_every:=10` 改成 1 秒，状态一变仍会立刻打印。

> 现场实测：模型 yaw = 0 时，车在 `map(-5.5, 3.25)`，低雷达在
> `map(-5.08, 3.25)` —— 正好在车体 **+x 方向 0.42 m**。车头就是 +x
> （D435/低雷达那一侧）。觉得“歪 90°”通常是对朝向的预期不同，
> 用 `spawn_yaw:=1.5708` 可让车头朝北；检测用 TF，不受影响。

### 0.2 测试流程（从零开始）

```bash
# 0) 清场：停掉本项目所有 Gazebo/ROS 进程
bash ~/work/nuwa_sim/tools/stop_all.sh

# 1) 终端1：仿真（车放在 L1 检测点）
bash ~/work/nuwa_sim/tools/launch_sim.sh \
  spawn_x:=0.0 spawn_y:=-2.25 spawn_z:=0.901 spawn_yaw:=0.0

# 2) 终端2：定位
source ~/work/nuwa_sim/tools/env.sh
ros2 run mini_car_control odom_to_tf --ros-args -p team:=red

# 3) 终端3：检测（只判 6 个公共位，每秒一表）
bash ~/work/nuwa_sim/tools/run_tower_check.sh
# 第一行必须是 tower_check v3（双雷达），并打印 /scan_low、/scan_high

# 4) 终端4：放块
bash ~/work/nuwa_sim/tools/place_block.sh L1_W_M
bash ~/work/nuwa_sim/tools/place_block.sh L2_SW
```

`launch_sim.sh` 固定用源码里的 launch，`run_tower_check.sh` / `place_block.sh`
固定用新 install，不会再解析到旧 `gazebo_nuwa`。

**L1 检测点为什么看不了两个 L1 公共位的塔基**：

- 车在检测点 yaw=0 朝东：低雷达前视 ±95°，西边的公共位在车后；
  `br_mission` 会掉头 180° 再扫，低雷达能覆盖两侧 —— 但它覆盖的是
  **两侧的私有位/侧向区域**，不是这两个公共位的塔基。
- 两个 L1 公共位在 `y=0`，正好落在 L2 平台（3x3，顶面 0.9）的南侧阴影里。
  低雷达在 0.776 m，低于平台顶面；从南侧任何位置看这两个位，视线都会先撞上
  平台侧壁（实测 `L1_E_M` 被挡在 0.927 m）。所以**掉头也解决不了**。
- 因此分工是：
  - **high 360°**：看两个 L1 公共位的【中段】+ 四个 L2 位的【第一层】；
  - **low**：看车头前方/作业点的【塔基】；
  - 两个 L1 公共位的塔基，到**作业点**正对时确认（反正也要到作业点放块）。
- 想让一个 L1 位置直接看两个公共塔基：几何上做不到（平台挡在中间），
  只能加东/西两个侧检测点，或加第三台低雷达。

**朝北也一样**：车朝北（yaw 90）时两个公共位确实都进了低雷达 ±95° 视场
（约 ±56°），但车头低雷达离 L2 平台南沿只有 0.33 m，两条视线都在约
0.6 m 处撞上平台侧壁（0.6~0.9 高）。所以不是视场角的问题，是**平台本身
挡在中间**；这个检测点任何朝向都看不到两个 L1 公共塔基。

**私有位不需要检测**：只做 6 个公共位时，`only_spots` 只列
`L1_W_M,L1_E_M,L2_SW,L2_SE,L2_NW,L2_NE`。这样低雷达在 L1 检测点就没有
任务了（它原本只是补两侧私有位的），检测点只用 **high 360°** 筛上段；
low 只在作业点用。`br_mission` 目前仍会掉头 180°（为私有位写的），
公共位模式下这一步是多余的，可以后续加开关跳过。

**L2 第二层从 L1 看不到**：high 在 L1 世界 1.16，只切 L2 第一层
`[0.90,1.25]`；L2 中段 `[1.25,1.60]` 在它上面。所以：

- 车在 L1：`L2_*` 只能报「第一层有/没有」；在上面再放一块，状态仍可能是
  `塔基`（不变）。
- 要看 L2 第二层，把车开到 L2（检测点 map(-5.5, 4.6)，台面 0.9），
  high 在 1.46，能看到 L2 中段。

L2 检测点传送：

```bash
gz service -s /world/nuwa_field/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "mini_car" position: {x: 0.0, y: -0.9, z: 1.2} orientation: {z: 0.0, w: 1.0}'
```

## 1.地图
地图相关文件存在field文件夹中：  
```
field/
├── field_config.py        ← 所有尺寸 / 颜色 / 质量都在这，改这里
├── gen_world.py           ← 生成 SDF world
├── render_plan.py         ← 生成俯视平面图（开 Gazebo 之前先对图用）
├── worlds/
│   └── nuwa_field.sdf     ← 地图文件
└── out/
    └── plan.png           ← 2D地图文件
```
### 1.1 启动与关闭地图
```bash
env -u QT_XCB_GL_INTEGRATION gz sim
  /home/wyq/work/nuwa_sim/field/worlds/nuwa_field.sdf
```
```bash
pkill -f "[n]uwa_field.sdf"
```
## 2.小车

### 2.1启动小车模型
解析urdf文件：  
```bash
source /opt/ros/humble/setup.bash
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat .../mini_car.urdf)"
```
把模型塞进gazebo
```bash
source /opt/ros/humble/setup.bash
  ros2 run ros_gz_sim create -topic robot_description -name mini_car -x 0 -y -5 -z 0.3
```
### 2.2xacro
记得关掉conda，不然会和ros2冲突
```bash
conda deactivate
cd /home/wyq/work/nuwa_sim/robots/src/mini_car
```
检查xacro是否正确
xacro urdf/mini_car.xacro | head -20  
xacro urdf/mini_car.xacro > /tmp/t.urdf && check_urdf /tmp/t.urdf

## 3.启动
### 3.1启动地图与简单小车
```bash
ros2 launch ~/work/nuwa_sim/robots/src/mini_car/launch/mini_car.launch.py
```
### 3.2启动控制器
```bash
ros2 run mini_car_control teleop_wasd
```
### 3.3启动rviz2,带配置文件
```bash
rviz2 -d ~/work/nuwa_sim/robots/src/mini_car/rviz/mini_car.rviz
```

## 测试双 2D 雷达（建塔处检测）

一台雷达只能切一条高度带：装在台面上方 0.60 m 时只看得见**中段**，
分不出"空位"和"只有塔基"。现在装两台（都以**车所在层台面**为基准）：

| 雷达 | 台面上方 | base_link 系 `(x, y, z)` | 切哪条带 |
|---|---|---|---|
| `lidar_low`  | **0.175 m** | `(0.42, 0, -0.125)` 车头前伸 | 塔基 `[0, 0.35]` |
| `lidar_high` | **0.560 m** | `(0, 0, 0.26)` 车顶 | 中段 `[0.35, 0.70]`；车在 L1 时还切到 **L2 塔基** `[0.30, 0.65]` |

层高：地面 0 / **L1 = 0.600** / **L2 = 0.900**；方块 0.35 m。
车在 L1 时 `base_link` 世界 z = 0.90，high 在 **1.16**——
这个高度同时落在 L1 中段 `[0.95, 1.30]` 和 L2 塔基 `[0.90, 1.25]` 里，
所以**一条光束能同时看 L1 的两段和 L2 的第一层**（靠建塔处自己的层区分）。

### ① 启动

先按 §0 预检并 source 新 install。车头 = 模型 **+x**（D435/低雷达那一侧），
默认 `spawn_yaw=0` 朝东；觉得车头“歪了 90°”多半是朝向预期不同，
想朝北（+y）就加 `spawn_yaw:=1.5708`。检测用 TF，车身朝哪都不影响算法。

```bash
ros2 launch ~/work/nuwa_sim/robots/src/mini_car/launch/mini_car.launch.py \
    spawn_x:=0.0 spawn_y:=-2.25 spawn_z:=0.901 spawn_yaw:=0.0
```

### ② 放方块（红队 L1 / L2 各几个，故意做成不同状态）

```bash
cd ~/work/nuwa_sim/robots/src/mini_car
BLK=./models/block.sdf
# L1_W_S：塔基+中段（满）
ros2 run ros_gz_sim create -file $BLK -name blk_l1ws_0 -x -2.75 -y -2.75 -z 0.776
ros2 run ros_gz_sim create -file $BLK -name blk_l1ws_1 -x -2.75 -y -2.75 -z 1.126
# L1_W_M：只有塔基
ros2 run ros_gz_sim create -file $BLK -name blk_l1wm_0 -x -2.75 -y  0.00 -z 0.776
# L2_SW：只有第一层
ros2 run ros_gz_sim create -file $BLK -name blk_l2sw_0 -x -1.25 -y -1.25 -z 1.076
# L1_E_S / L1_E_M / L2_SE 空着
```

### ③ 看扫描 / 状态

```bash
ros2 topic echo /scan_low  --once | head -20
ros2 topic echo /scan_high --once | head -20

ros2 run mini_car_control odom_to_tf --ros-args -p team:=red
ros2 run mini_car_control tower_check --ros-args -p team:=red
ros2 topic echo /tower_check/status
```

期望（车在 L1 检测点 map(-5.5, 3.25)，单朝向；实际输出）：

```
L1_W_S=满(-/12)      # 塔基+中段（低雷达在车后视场外，高雷达看到中段）
L1_W_M=未知(-/-)     # 低雷达被 L2 平台挡住；高雷达被 L2_SW 的塔挡住
L1_E_S=空(0/0)       # 空位
L1_E_M=未知(-/0)     # 低雷达被 L2 平台挡住；高雷达没看到中段
L2_SW=塔基(21/-)     # L2 已有一层（车在 L1 也能看到）
L2_SE=空(0/-)
```

`tower_check` 输出里的括号是 `(塔基回波数/中段回波数)`：
- `-` = 这台雷达在这个高度/方位看不到（**不是 0**）
- `0` = 看到了、确实没有回波

**为什么两个 L1 中排建塔处是「未知」**：检测点在 L2 平台南侧，
低雷达（台面上方 0.175）到这两处的视线正好被 L2 平台 0.3 m 的侧壁挡住；
高雷达再看一眼：
- `L1_E_M` 没有阻挡 → 中段回波数 0 → 含义是**中段确定没有，塔基没看清**
- `L1_W_M` 的视线正好穿过 L2_SW 的塔 → 中段也看不到 → 两个 `-`

`br_mission` 把「未知但中段=0」记为「待定」候选，开到作业点后低雷达
正对着建塔处，近距离再分「空位 / 只有塔基」，然后决定放哪一块 ✓
两个 `-` 的点不去碰 —— 那说明整条视线都被挡住了，得先挪位置。

设计取舍：
- 低雷达不能装 360°：0.175 m 在车壳内部（车壳底面 0.05），只能前伸到车头，
  前方 ±95° 干净。检测点要掉头 180° 再扫一次，才能补上背后两个建塔处。
- L2 平台一定会挡住一部分低处视线 —— 这是几何，不是算法 bug。
  报「未知」+ 作业点二次确认，比猜「空」安全 ✓

### ④ 只测 6 个公共位

公共位 = **2 个 L1 中排 + 4 个 L2**（红蓝共用）：
`L1_W_M, L1_E_M, L2_SW, L2_SE, L2_NW, L2_NE`。
两个 `L1_*_S` 是红队私有位，不在这 6 个里。

```bash
# 车直接放在 L1 检测点（真机是开上去，仿真里用 spawn 代替）
ros2 launch ~/work/nuwa_sim/robots/src/mini_car/launch/mini_car.launch.py \
    spawn_x:=0.0 spawn_y:=-2.25 spawn_z:=0.901

# 另一个终端：只盯 6 个公共位，每秒打印一次表
ros2 run mini_car_control odom_to_tf --ros-args -p team:=red
ros2 run mini_car_control tower_check --ros-args -p team:=red \
    -p log_every:=10 \
    -p only_spots:=L1_W_M,L1_E_M,L2_SW,L2_SE,L2_NW,L2_NE
```

初始摆放（L1 公共左 1 块、L2 近左 1 块）：

```bash
BLK=~/work/nuwa_sim/robots/src/mini_car/models/block.sdf
# L1 公共左（世界 -2.75, 0）：1 块塔基，中心 z = 0.600 + 0.175
ros2 run ros_gz_sim create -file $BLK -name pub_l1wm_0 -x -2.75 -y 0 -z 0.776
# L2 近侧左（世界 -1.25, -1.25）：1 块第一层，中心 z = 0.900 + 0.175
ros2 run ros_gz_sim create -file $BLK -name pub_l2sw_0 -x -1.25 -y -1.25 -z 1.076
```

检测点输出（yaw 0）：

```
L1_W_M=未知(-/-)   L1_E_M=未知(-/0)
L2_SW=塔基(21/-)   L2_SE=空(0/-)   L2_NW=空(0/-)   L2_NE=空(0/-)
```

两个 L1 公共位在检测点被 L2 平台挡住低雷达（`L1_W_M` 还被 L2_SW 的塔
挡住高雷达），所以是「未知」；4 个 L2 位由高雷达直接给出第一层状态。
L1 的两个到作业点再确认（低雷达正对，28 点）：

```
作业点 map(-7.5, 4.75) yaw 135° → L1_W_M=塔基(28/0)   # 只有塔基 → 放中段
作业点 map(-3.5, 4.75) yaw  45° → L1_E_M=空(0/0)      # 空位 → 放塔基
```

⚠ 检测点对 `L1_W_M` 是 `未知(-/-)`（整条视线被挡）。`br_mission`
默认会跳过这种点；只做公共位时要一起开 `confirm_blocked`，否则会漏掉
已经放了塔基的公共位：

```bash
ros2 run mini_car_control br_mission --ros-args \
  -p team:=red -p level:=L1 -p tasks:=1 \
  -p only_spots:=L1_W_M,L1_E_M -p confirm_blocked:=true
```

它会开到 `L1_W_M` 的作业点，确认 `塔基`，放中段（z=0.950）。

### ⑤ 精度测试

`ros_gz_sim create` 的 `-Y` 是**弧度**，`-x/-y` 直接加偏移：

```bash
# 45° 旋转
ros2 run ros_gz_sim create -file $BLK -name test_rot -x 2.75 -y 0 -z 0.776 -Y 0.7854
# 偏离建塔处中心 0.10 m
ros2 run ros_gz_sim create -file $BLK -name test_off -x 2.85 -y 0 -z 0.776
# 删掉重放
gz service -s /world/nuwa_field/remove --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "test_rot" type: 2'
```

实测（日志里的“差”= 实测近面距离 − 按方块摆正算的期望）：

| 摆放 | 实测近面 | 期望 | 差 | 点数 | 判定 |
|---|---|---|---|---|---|
| 摆正 | 0.396 m | 0.393 m | **+3 mm** | 28 | 塔基 |
| 45° 旋转 | 0.465 m | 0.393 m | **+72 mm** | 28 | 塔基 |
| L2 偏移 0.10 m | 1.445 m | 1.377 m | **+69 mm** | 15 | 塔基 |

45° 那 72 mm 正好是“方块面法线转到了视线上”造成的（0.247→0.175）。
`dist_tol=0.20` 是容差主阀：**径向偏移超过约 0.20 m 才开始漏判**；
旋转只在 ±72 mm 内，单独不会造成漏判。偏移会让点数下降（21→15），
但 `min_points=3` 还有足够余量。

### ⑥ 放块脚本 `place_block`

不用手敲一长串 `ros2 run ros_gz_sim create …`，直接用建塔处名字：

```bash
# 位置 [dx dy yaw_deg count]
ros2 run mini_car_control place_block L1_W_M
ros2 run mini_car_control place_block L1_W_M 0.05 0 15 1
ros2 run mini_car_control place_block L1_E_M 0.10 0 45 2

# 没 colcon build 时直接跑源码
python3 robots/src/mini_car_control/mini_car_control/place_block.py L1_W_M 0.05 0 15 1
```

- `spot`：`tower_spots.yaml` 里的名字（`L1_W_M` / `L2_SW` …）
- `dx dy`：方块中心相对建塔处中心的偏移，米，**场地坐标轴**
- `yaw`：方块旋转，**度**（脚本内部转弧度）
- `count`：这次放几块；`0` = 只清空/不放
- `--layer auto|0|1`：自动查现有块数，或强制塔基/中段
- `--clear`：放之前删掉该建塔处附近的测试方块
- `--clear-only`：只删不放（重测前清场）
- `--dry-run`：只打印要执行的命令
- z 自动算：塔基 = 台面 + 0.175，中段 = 台面 + 0.525

脚本会先读 Gazebo 的 `pose/info` 数该建塔处已有几块，所以“放一块”
会自动变成第二层（中段）；`--clear` 后重放会自动回到塔基。
`--clear` 只删 `blk_/br_/pub_/test_/tmp_/tall_/block_` 前缀的模型，
不会碰场地和车。

## 4. Mid-360 完全仿真（GLIM 定位）

> 详细设计、参数表和排错见
> [`robots/src/mid360_sim/README.md`](robots/src/mid360_sim/README.md)

原来的 `gpu_lidar` 是 **固定 360×55 均匀栅格 + 整帧同一瞬间采样**，
没有非重复扫描，也没有逐点时间戳。现在改成仓库自带的 gz-sim 系统插件
`mid360_sim`：

- 回放 Livox 实测的 80 万条扫描轨迹（4 秒 / 40 帧），每帧 2 万点；
- 一帧拆成小批，在**每个点自己的仿真时刻**做物理射线投射；
- 发布 `/mid360/points`，字段 `x,y,z,intensity,tag,line,ring,t`，
  `t ∈ [0, 0.1)` 秒（相对帧头）；
- 量程 40 m、FOV −7.2°~+52.2°、1 cm 量程噪声、4 条 line。

### 4.1 编译插件（只需一次，改代码后重跑）

```bash
conda deactivate
cd ~/work/nuwa_sim/robots/src/mid360_sim
./build.sh
```

`mini_car.launch.py` 会自动把 `build/` 加进 `GZ_SIM_SYSTEM_PLUGIN_PATH`。

### 4.2 启动

```bash
# ① 仿真（跑 LIO 用带墙/天花板的 world）
ros2 launch ~/work/nuwa_sim/robots/src/mini_car/launch/mini_car.launch.py \
    world:=$HOME/work/nuwa_sim/field/worlds/nuwa_field_room_ceiling.sdf

# ② GLIM（配置已经改好：global_shutter_lidar=false，逐点时间去畸变）
ros2 launch ~/work/nuwa_sim/glim/glim_sim.launch.py

# ③ RViz（可选）
rviz2 -d ~/work/nuwa_sim/glim/glim.rviz
```

### 4.3 验证

```bash
conda deactivate
python3 ~/work/nuwa_sim/tools/check_mid360_sim.py --frames 20
```

预期：每帧 20000 点、`t` 0~0.099995 s、仰角 −7.2°~52.2°、360° 覆盖、
header stamp 间隔 100 ms、相邻帧方向 Jaccard≈0.52。

实测直行 0.5 m/s × 10 s，GLIM 与 Gazebo 真值位移一致：
`dx=3.440 m, dy=0.028 m`。

### 4.4 性能

| `point_stride` | 点数/帧 | RTF（轻载 / 同时开 VSCode 等时） |
|---|---|---|
| 1（默认，完整） | 20000 | ≈0.86 / ≈0.6 |
| 2（推荐跑 LIO） | 10000 | ≈1.00 / ≈0.9 |
| 4 | 5000 | ≈1.00 / ≈1.0 |

GLIM 本来就会把点云降采样到 10000 点，所以实时性优先时可在
`mini_car.xacro` 的插件块里加：

```xml
<point_stride>2</point_stride>
```

### 4.5 两个必须注意的点

1. **世界 SDF 必须用 bullet 碰撞检测器**（dartsim 的默认 `ode` 不支持
   raycast）。`field/worlds/*.sdf` 已经加好；以后重新生成世界时要把
   `<dart><collision_detector>bullet</collision_detector></dart>` 带上。
2. **不要**再启动 `add_point_time.py`。插件已经给正确的逐点时间；
   那个脚本的旋转雷达公式对非重复扫描是错的。

### 4.6 实机 bag 核对

用 `tools/analyze_mid360_bag.py` 分析 `/data/bags/mid360_static_01` 的结果：

- 实机 10 Hz、每帧 19968/20064 点、逐点时间 0~100.4 ms、IMU 207 Hz（**g** 单位）；
- 75% 的有效点与仿真用官方轨迹方向差 <0.1°，96.5% <0.5°；
- 实机视场高仰角端到 +54°，官方轨迹到 +52.2°；
- bag 实际是多段静止 + 原地转动，没有缓慢平移；
- 详细差异表见
  [`robots/src/mid360_sim/README.md` §6](robots/src/mid360_sim/README.md#6-真实-bag-核对databagsmid360_static_01)。

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source ~/rc2026_ros2_ws/install/setup.bash
python3 tools/analyze_mid360_bag.py --bag /data/bags/mid360_static_01
```
