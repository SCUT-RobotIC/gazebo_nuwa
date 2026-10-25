# mid360_sim — Gazebo Sim 里的 Livox Mid-360 完全仿真

这个包是一个 **gz-sim 8 系统插件**（`libmid360_lidar_system.so`），
用来替代原来的 `gpu_lidar`。它解决两件事：

1. **非重复扫描**：回放 Livox 官方实测的 80 万条射线轨迹（4 秒 / 40 帧），
   不再是固定的 360×55 均匀栅格；
2. **逐点时间戳**：一帧 100 ms 被拆成小批，在**每个点自己的仿真时刻**做
   物理射线投射，并给每个点写 `t`（相对帧头的秒数）。

因此机器人运动时，点云会像真机一样带有帧内运动；GLIM 可以用它做逐点
去畸变。

---

## 1. 数据流

```
gz sim
  └─ mid360_sim 插件（挂在 base_link 上）
       ├─ 读取 patterns/mid360.csv（800000 个方向）
       ├─ 每 1 ms 取当前批次的射线 → components::RaycastData
       ├─ Physics 系统（dartsim + bullet collision detector）做射线投射
       └─ 每 100 ms 发布 gz.msgs.PointCloudPacked → /mid360/points
                                                    │
ros_gz_bridge ──────────────────────────────────────┘
  └─ sensor_msgs/PointCloud2 → /mid360/points
       fields: x, y, z, intensity, t, tag, line, ring
       frame_id: mini_car/base_link/mid360
       header.stamp: 这一帧第一个点的时刻
       t: [0, 0.099995] s（相对 header.stamp）
```

GLIM 的 `config_sensors.json` 里已经对应设置：

```jsonc
"global_shutter_lidar": false,
"autoconf_perpoint_times": false,
"perpoint_relative_time": true,
"perpoint_time_scale": 1.0,
"ring_field": "ring"
```

> ⚠ 不要再启动 `add_point_time.py`。它按机械旋转雷达的方位角公式编时间，
> 对 Mid-360 的非重复扫描是错的；插件已经给出正确的逐点时间。

---

## 2. 编译

### 依赖

- Ubuntu 22.04 + ROS 2 Humble
- **Gazebo Sim ≥ 8.15**（需要 `components::RaycastData` 和
  dartsim 的 `GetRayIntersectionFromLastStep`；本机是 8.15）
- `libgz-sim8-dev`、`libgz-msgs10-dev`、`libgz-transport13-dev`、
  `libgz-plugin2-dev`、`libgz-common5-dev`、`libgz-math7-dev`
- dartsim 物理引擎（gz-sim 默认）+ `bullet` collision detector
- C++17 编译器

```bash
conda deactivate          # 避免 conda 的 python/cmake 干扰 ROS 2
cd ~/work/nuwa_sim/robots/src/mid360_sim
./build.sh
```

产物：

```
build/libmid360_lidar_system.so
```

`mini_car.launch.py` 会自动把 `build/` 加进
`GZ_SIM_SYSTEM_PLUGIN_PATH`；用 colcon 安装到
`robots/install/mid360_sim/lib/` 也能被自动找到。

### 关于本机的 protobuf

这台机器 `/usr/local` 下有一套源码安装的 protobuf 3.6，而 Gazebo/gz-msgs
是用系统的 protobuf 3.12 编译的。GCC 默认先搜 `/usr/local/include`，
会混用两套头文件（报 `GetArenaNoVirtual` 之类的错）。

`CMakeLists.txt` 里已经做了两个隔离：

- 在 `build/protobuf_shim/google/protobuf` 建一个指向
  `/usr/include/google/protobuf` 的符号链接，并放在 include 路径最前；
- 强制 `protobuf::libprotobuf` 指向
  `/usr/lib/x86_64-linux-gnu/libprotobuf.so.23`。

换机器时如果不存在 `/usr/lib/x86_64-linux-gnu/libprotobuf.so`，这两段会自动
跳过。

---

## 3. 接入方式

`robots/src/mini_car/urdf/mini_car.xacro` 中：

```xml
<gazebo>
  <plugin filename="libmid360_lidar_system.so" name="mid360_lidar">
    <link_name>base_link</link_name>
    <sensor_pose>0 0 0.40 0 0 0</sensor_pose>
    <topic>/mid360/points</topic>
    <frame_id>mini_car/base_link/mid360</frame_id>
    <pattern_file>$(arg mid360_pattern)</pattern_file>
    <points_per_scan>20000</points_per_scan>
    <point_stride>1</point_stride>
    <scan_rate_hz>10.0</scan_rate_hz>
    <range_min>0.1</range_min>
    <range_max>40.0</range_max>
    <range_noise_stddev>0.01</range_noise_stddev>
    <intensity>100</intensity>
    <dropout_probability>0.0</dropout_probability>
    <line_count>4</line_count>
    <seed>42</seed>
    <azimuth_cycle_offset_deg>137.507764</azimuth_cycle_offset_deg>
    <log_interval>100</log_interval>
  </plugin>
</gazebo>
```

`mini_car.launch.py` 会把 `mid360_pattern` 作为绝对路径传给 xacro，
整包挪目录不用改。

### 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `link_name` | `base_link` | 射线挂在哪个 link 上（fixed joint 合并后只能挂 base_link） |
| `sensor_pose` | `0 0 0.40 0 0 0` | 传感器在 link 系里的位姿，必须和静态 TF 一致 |
| `topic` | `/mid360/points` | gz 话题名 |
| `frame_id` | `mini_car/base_link/mid360` | 点云 frame_id |
| `pattern_file` | `mid360.csv` | 相对路径时按 .so 位置找；也可用 `MID360_PATTERN_FILE` 环境变量 |
| `points_per_scan` | `20000` | 每个 10 Hz 帧消耗多少行轨迹（真机 20 万点/秒 ÷ 10 Hz） |
| `point_stride` | `1` | 每 N 行发一个点。`2` = 1 万点/帧，RTF≈1.0；`1` = 完整 2 万点，RTF≈0.86 |
| `scan_rate_hz` | `10.0` | 帧率 |
| `range_min/max` | `0.1 / 40.0` | 量程（m） |
| `range_noise_stddev` | `0.01` | 每点量程高斯噪声 1σ（m） |
| `intensity` | `45` | 反射率字段（真机是 0~255；实测中位数约 44~45，这里固定值） |
| `dropout_probability` | `0.0` | 每个命中点随机丢弃的概率 |
| `line_count` | `4` | `line`/`ring` 的取值数 |
| `azimuth_cycle_offset_deg` | `137.507764` | 4 秒轨迹回绕时整帧旋转的黄金角；设 0 关闭 |
| `log_interval` | `100` | 每多少帧打印一次统计；0 关闭 |

---

## 4. 仿真世界的必要设置

插件用物理引擎的批量射线接口 `components::RaycastData`。dartsim 的默认
碰撞检测器 `ode` **不支持 raycast**，必须换成 `bullet`：

```xml
<physics name="1ms" type="ignored">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <dart>
    <collision_detector>bullet</collision_detector>
  </dart>
</physics>
```

`field/worlds/` 下的 4 个世界文件都已经加上了。
**如果以后用 `field/gen_world.py` 重新生成世界，记得把这一段也生成进去**。

症状：不加的话日志会刷屏

```
Warning [CollisionDetector.cpp:67] [CollisionDetector] Raycast is not supported by 'ode'
```

并且没有点云。

---

## 5. 验证

仿真跑起来后：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
python3 tools/check_mid360_sim.py --frames 20
```

会检查：

- 字段 `x,y,z,intensity,tag,line,ring,t` 齐全；
- 每帧 2 万点（或 `20000/point_stride`）；
- `t ∈ [0, 0.1001]` 且单调不减；
- 仰角 ≈ −7.2° ~ +52.2°，方位角覆盖 36 个 10° 桶；
- `line/ring ∈ [0,3]`；
- header stamp 间隔 ≈ 100 ms；
- 相邻帧方向栅格 Jaccard < 0.95（非重复）。

实测（Ryzen 9 8945HX，`nuwa_field_room_ceiling.sdf`，仅雷达）：

| point_stride | 点数/帧 | RTF（轻载 / 本机同时开 VSCode 等时） |
|---|---|---|
| 1 | 20000 | ≈0.86 / ≈0.6 |
| 2 | 10000 | ≈1.00 / ≈0.9 |
| 4 | 5000 | ≈1.00 / ≈1.0 |

RTF 取决于整机负载：射线投射在 gz 的主线程里串行执行，
满负载 20 万条/秒大约吃掉一个核心的 60%~100%。
完整机器人（含 2D 雷达、D435、IMU）再加 GLIM 时会再低一些。
GLIM 的预处理本来就会把点云降采样到 10000 点（`random_downsample_target`），
所以跑 LIO 时 `point_stride=2` 基本不损失有效信息，是实时性优先的推荐值。

### 运动/去畸变验证

让车直行 0.5 m/s、10 秒，对比：

```
Gazebo 真值 /mini_car/odom : dx=3.440 m dy=0.028 m
GLIM       /glim_rosnode/odom: dx=3.440 m dy=0.028 m
```

两者位移完全一致，说明逐点时间戳和 IMU 去畸变链路是通的。

---

## 6. 真实 bag 核对（`/data/bags/mid360_static_01`）

用 `tools/analyze_mid360_bag.py` 分析了这段 123 秒的实机数据
（`xfer_format=4`，CustomMsg + PointCloud2 两份同时发布）：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source ~/rc2026_ros2_ws/install/setup.bash   # 提供 livox_ros_driver2 消息
python3 tools/analyze_mid360_bag.py --bag /data/bags/mid360_static_01
```

### 6.1 数据流

| 项目 | 实测 |
|---|---|
| 帧率 | 1231 帧 / 123.09 s，帧间隔中位数 **99.987 ms**（10 Hz） |
| 每帧点数 | **19968 / 20064**（按下发时间切帧，不是固定 2 万） |
| 有效回波 | 约 **12k / 帧（60%）**；其余 `x=y=z=0` 的无回波点也照常发布 |
| 逐点时间 | `offset_time` **0 ~ 100.4 ms**，中位步长 **4.95 µs**；约 **0.7%** 点轻微乱序 |
| 视场 | 有效点仰角约 **−7.7° ~ +54.0°**，99.3% 落在官方 CSV 的 −7.2°~52.2° 内 |
| line | 0~3，四路均匀 |
| 反射率 | 0~255，有效点中位数 **44~45** |
| PC2 字段 | `x,y,z,intensity,tag,line,timestamp`，`point_step=26`，`timestamp` 是**绝对纳秒** |
| 消息/帧 | `/livox/lidar`=CustomMsg，`/livox/lidar/pointcloud`=PC2，`frame_id=livox_frame` |
| IMU | **207 Hz**，加速度单位 **g**（模长≈1.0），静态陀螺噪声约 0.0022 rad/s，零偏约 0.01 rad/s |

### 6.2 扫描轨迹匹配

把实机每帧的有效点方向与仿真用的官方 `mid360.csv` 做最近邻比较：

- **75%** 的点方向误差 **< 0.1°**
- **96.5%** 的点方向误差 **< 0.5°**
- 约 **0.7%** 的点在 +52.2° 以上（官方 CSV 没有覆盖到的少量高仰角射线）

⇒ 官方轨迹对这台 Mid-360 的核心几何是可靠的；主要差别是实机在最高仰角
多出约 1~2°，以及两段 4 秒数据的时间相位不同。

### 6.3 运动段

IMU 显示这段 bag **不是简单的"先静止后移动"**，而是多段静止/旋转：

- 0~44 s：静止
- 45~65 s：转动（净转角约 15.5°，来回转动路径约 235°，峰值 1.33 rad/s）
- 65~90 s：基本静止（90 s 附近有短暂晃动）
- 100~120 s：转动（净转角约 8.6°，来回路径约 276°，峰值 1.30 rad/s）
- 120~123 s：静止

**没有明显的缓慢平移**；加速度模长始终在 1g 附近，运动主要是原地/手持转动。
这与你的记忆一致。

### 6.4 与仿真的差异清单

| 项目 | 实机 | 仿真 |
|---|---|---|
| 点云话题/消息 | `/livox/lidar` CustomMsg + `/livox/lidar/pointcloud` PC2 | `/mid360/points` PC2 |
| frame_id | `livox_frame` | `mini_car/base_link/mid360` |
| 逐点时间字段 | `timestamp` 绝对 ns / `offset_time` 相对 ns | `t` 相对秒（float32） |
| 无回波点 | 发布 `x=y=z=0`（约 40%） | 默认丢弃 |
| 每帧点数 | 19968 / 20064 | 固定 20000 |
| 逐点时间顺序 | 约 0.7% 轻微乱序 | 严格单调 |
| 视场上界 | 有效点约 +54.0° | +52.2° |
| IMU | ~207 Hz，**g**，σ≈0.0022 rad/s / 0.073 m/s²，零偏≈0.01 rad/s | 200 Hz，**m/s²**，噪声/零偏已按实机对齐 |
| 反射率 / tag | 0~255（中位 44~45）/ 多种 tag | 固定 45 / 0 |
| GLIM `acc_scale` | 9.7949 | 1.0 |
| GLIM IMU 噪声参数 | 0.05/0.02/0.001/1e-5 | 已对齐为同一组 |
| GLIM 逐点时间 | `autoconf_perpoint_times=true` | `false` + `t` 相对秒 |

功能上，对 GLIM 来说两者等价：同样是 10 Hz、约 2 万点/帧、非重复扫描、
逐点时间戳、−7°~+52° 视场。差异主要是消息格式/坐标系/IMU 单位和少量
高仰角射线；仿真用一份单独的 `glim/config` 来适配，不能直接拿实机
`maps/config/config_ros.json` 原样套到仿真上。

### 6.5 实机配置里需要核对的两点

1. `maps/config/config_ros.json` 里 `points_topic` 是 `/livox/lidar`。
   但这段 bag 是用 `xfer_format=4` 录的，`/livox/lidar` 是 **CustomMsg**，
   而 GLIM 只订阅 **PointCloud2**。实机跑 GLIM 时应使用
   `msg_MID360_side_GLIM.launch.py`（`xfer_format=0`，`/livox/lidar` 是 PC2），
   或者把 `points_topic` 改成 `/livox/lidar/pointcloud`。
2. 实机配置里 `lidar_frame_id= "lidar"`、`imu_frame_id= "imu"`，
   而驱动发布的是 `livox_frame`。GLIM 的里程计不一定依赖这两个名字，
   但 RViz/TF 会受影响，建议按实际 TF 树核对。

## 7. 轨迹数据说明

`patterns/mid360.csv` 来自 Livox 官方仿真仓库
`Livox-SDK/livox_laser_simulation` 的 `scan_mode/mid360.csv`
（commit `1cce1073633a062b92e30243a4c2920e45551bb5`，MIT License）。

- 800,000 行 = 200,000 点/秒 × 4 秒 = 40 个 10 Hz 帧；
- 列：`Time/s, Azimuth/deg, Zenith/deg`；
- 插件按 `elevation = 90° - zenith` 转成标准球坐标，得到 −7.2° ~ +52.2°；
  Gazebo Classic 官方插件用的是 `zenith - 90°`，会把点云上下镜像，本插件
  没有沿用；
- 每帧消耗连续 20,000 行；文件回绕时整帧加黄金角旋转，避免 4 秒后精确重复。

> 想用你们那台真实 Mid-360 的实测数据校准（方位角相位、点频、强度分布）时，
> 录一段 15~30 秒的 `/livox/lidar` + `/livox/imu` rosbag2 即可，
> 轨迹文件可以按同一格式替换。

---

## 8. 排错

| 现象 | 原因 / 处理 |
|---|---|
| `Failed to load plugin ... libmid360_lidar_system.so` | 没编译，先跑 `./build.sh` |
| `Raycast is not supported by 'ode'` 刷屏、无点云 | 世界 SDF 没写 `<dart><collision_detector>bullet</collision_detector></dart>` |
| `Raycast is not supported` 但用的是别的物理引擎 | 必须是 dartsim（默认），gazebo 的 bullet 引擎不支持这个批量射线接口 |
| 点云没有 `t` 字段 | 桥接没把 `/mid360/points` 桥过来，或用了旧的 `gpu_lidar` |
| GLIM 报 `per-point timestamps are not given` | `config_sensors.json` 的 `global_shutter_lidar` 还是 `true` |
| 同一话题时间戳乱序、帧点数忽多忽少 | 有多个 `gz sim` 进程同时在跑，互相抢同一个 gz-transport 话题；全部杀掉再启动 |
