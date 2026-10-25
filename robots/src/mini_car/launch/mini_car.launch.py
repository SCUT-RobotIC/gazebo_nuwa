#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动：Gazebo 场地 + 机器人描述 + spawn 机器人。

用法：
    ros2 launch launch/mini_car.launch.py

等价于原来手敲的三条命令（三个终端），现在一个终端搞定。

⚠️ 跑之前先确保 shell 干净：
    conda deactivate
    unset QT_XCB_GL_INTEGRATION      # 不然 Gazebo GUI 会段错误
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------------
# 路径 —— 全部从【本文件的位置】推出来，不写死绝对路径 ✓
#
#   本文件在  <项目根>/robots/src/mini_car/launch/mini_car.launch.py
#       _HERE =              .../mini_car/launch
#       _PKG  =              .../mini_car
#       _ROOT =  <项目根>
#
#   这样整包拷到任何目录、任何用户名下都能直接跑 ✓
#
#   为什么不用 get_package_share_directory('mini_car')：
#   本包是【源码直跑】的 —— 没 colcon build 过时 install/ 里没有 urdf/，
#   那个函数会直接抛 PackageNotFoundError ✗
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_PKG)))

XACRO_FILE = os.path.join(_PKG, 'urdf', 'mini_car.xacro')
FIELD_WORLD = os.path.join(_ROOT, 'field', 'worlds', 'nuwa_field.sdf')
BRIDGE_YAML = os.path.join(_HERE, 'bridge.yaml')

# ---------------------------------------------------------------------------
# Mid-360 系统插件（本仓库自带的 libmid360_lidar_system.so）
#
#   build/                      ← mid360_sim/build.sh 的产物（源码直跑）
#   robots/install/mid360_sim/lib ← 用 colcon 装出来的位置（可选）
#
# gz-sim 通过 GZ_SIM_SYSTEM_PLUGIN_PATH 找系统插件。这里在启动 gz sim
# 【之前】把这个环境变量塞进 launch 的进程环境，顺序不能反 ✗
# ---------------------------------------------------------------------------
MID360_DIR = os.path.join(_ROOT, 'robots', 'src', 'mid360_sim')
MID360_PATTERN = os.path.join(MID360_DIR, 'patterns', 'mid360.csv')
_MID360_PLUGIN_CANDIDATES = [
    os.path.join(MID360_DIR, 'build'),
    os.path.join(_ROOT, 'robots', 'install', 'mid360_sim', 'lib'),
]
_MID360_PLUGIN_DIRS = [
    d for d in _MID360_PLUGIN_CANDIDATES if os.path.isdir(d)
]
if not _MID360_PLUGIN_DIRS:
    print('\n[mini_car.launch] WARNING: libmid360_lidar_system.so 还没编译！\n'
          '  先运行:  ' + os.path.join(MID360_DIR, 'build.sh') + '\n'
          '  否则 Gazebo 会报找不到 mid360_lidar 插件。\n')
    _MID360_PLUGIN_DIRS = [os.path.join(MID360_DIR, 'build')]
MID360_PLUGIN_PATH = ':'.join(_MID360_PLUGIN_DIRS)
if os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH'):
    MID360_PLUGIN_PATH += ':' + os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']


def generate_launch_description():
    # ---------- 可调参数（ros2 launch ... spawn_x:=1.0 就能改） ----------
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    args = [
        # ---------- 世界文件（可换）----------
        # 为什么要做成参数：跑建图/定位（GLIM 这类 LIO）要用 【带围墙】 的世界，
        # 因为原场地 11x11 m、围栏只有 80 mm 高，雷达 55 个垂直圈里有 41 圈
        # 完全收不到回波，x/y/偏航没有任何约束，GLIM 会一直漂。
        #
        # 可选的世界（都由 field/gen_world.py 生成，都在 field/worlds/ 下）：
        #   nuwa_field.sdf                官方场地，演示/交付用（默认）
        #   nuwa_field_props.sdf          含方块/塔顶/五色石
        #   nuwa_field_room.sdf           场地 + 四面墙（实测 RTF 1.000，跑 LIO 用这个）
        #   nuwa_field_room_ceiling.sdf   再加天花板（RTF 0.996，但天花板对定位没增益）
        #
        # 用法：ros2 launch ... world:=$HOME/gazebo_nuwa/field/worlds/nuwa_field_room.sdf
        DeclareLaunchArgument('world', default_value=FIELD_WORLD,
                              description='要加载的 SDF 世界文件（默认官方场地）'),
        # 默认放在红队启动区中心附近（启动区 x∈[4.8,5.5], y∈[-5.5,-4.0]）
        DeclareLaunchArgument('spawn_x', default_value='5.0', description='spawn 位置 X'),
        DeclareLaunchArgument('spawn_y', default_value='-4.9', description='spawn 位置 Y'),
        # z=0.3 让轮子最低点正好落在 z=0 地面（轮心在 base_link 下方 0.30）
        DeclareLaunchArgument('spawn_z', default_value='0.3', description='spawn 高度'),
        # 车头 = 模型 +x（D435/低雷达那一侧）。默认 yaw=0，即朝场地 +x（东）。
        # 觉得“车头歪了 90°”通常是对朝向的预期不同；想朝北（+y）就传
        #   spawn_yaw:=1.5708
        # ⚠ 单位是【弧度】，和 ros_gz_sim create -Y 一致。检测用的是 TF，
        #   车身摆哪个方向都不影响算法；变的只是 low 的前视场。
        DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                              description='spawn 朝向（弧度；朝北=1.5708）'),
        # ---------- 要不要装第二台深度相机 D455 ----------
        # 默认【不装】—— 目前只弄 D435，D455 留着按需打开 ✓
        #   ros2 launch ... d455:=true
        # 打开后会一并加上：xacro 里的 link/joint/sensor、桥接、静态 TF ✓
        # ⚠ 代价：两台 640x480 RGB-D 一起开，RTF 从 1.000 掉到 0.897 ✗
        DeclareLaunchArgument('d455', default_value='false',
                              description='是否加载第二台深度相机 D455（默认 false）'),
    ]

    # ---------- 把 xacro 展开成 URDF 字符串 ----------
    # Command(...) 会在启动时执行这条命令并取它的输出
    # ParameterValue(..., value_type=str) 在 Humble 里是必须的：
    #   不加的话 ROS 会把 URDF 字符串当成 YAML 去解析，报一堆语法错
    # ⚠ 把 d455 开关透传给 xacro —— xacro 里的 <xacro:arg name="with_d455"> 决定
    #   要不要生成 D455 的 link/joint/sensor ✓
    robot_description = ParameterValue(
        Command(['xacro ', XACRO_FILE,
                 ' with_d455:=', LaunchConfiguration('d455'),
                 ' mid360_pattern:=', MID360_PATTERN]),
        value_type=str)

    return LaunchDescription(args + [
        # ---------- ⓪ Mid-360 插件搜索路径 ----------
        # 必须在 ① 启动 gz sim 之前设置，所以放在列表最前面 ✓
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', MID360_PLUGIN_PATH),

        # ---------- ① Gazebo：加载场地 ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            # gz_args 直接透传给 gz sim 命令；-r = 启动即开始仿真
            # ⚠ 这里不能写成 FIELD_WORLD + ' -r' —— 那样世界就写死了，
            #   加了 --with-room 生成带墙的世界也白搭 ✗
            #   gz_args 接受【substitution 列表】，LaunchConfiguration 会在
            #   启动时展开成用户传的值 ✓
            launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r']}.items(),
        ),

        # ---------- ② 告诉 ROS 机器人长什么样 ----------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # ---------- ③ 把机器人放进 Gazebo ----------
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',   # 从 ② 发布的那个话题拿模型
                '-name', 'mini_car',             # Gazebo 里的名字
                '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
                '-Y', spawn_yaw,
            ],
            output='screen',
        ),

        # ---------- ④ ROS ↔ Gazebo 话题桥 ----------
        # 把 8 个关节控制话题在两个世界之间打通：
        #   ROS 侧 std_msgs/Float64  ←→  Gazebo 侧 gz.msgs.Double
        #
        # ⚠ 话题名由 urdf/mini_car.xacro 里插件的 <topic> 参数【显式指定】，
        #   三处必须一致：xacro 的 <topic>、这里的桥、控制节点的发布者。
        #
        #   为什么要显式指定：JointPositionController 的默认名里有个纯数字段
        #   （/model/mini_car/joint/joint_steer_XX/0/cmd_pos 里的 "0"），
        #   ROS 2 不允许话题名任何一段以数字开头 → 桥的 ROS 侧建不出来 ✗
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                arg
                for tag in ('FL', 'FR', 'RL', 'RR')
                for arg in (
                    '/mini_car/steer_%s/cmd_pos'
                    '@std_msgs/msg/Float64@gz.msgs.Double' % tag,
                    '/mini_car/wheel_%s/cmd_vel'
                    '@std_msgs/msg/Float64@gz.msgs.Double' % tag,
                )
            ] + [
                # 关节状态：4 个轮子 + 4 个转向臂都是 continuous 关节，
                # RViz 靠它才知道该把轮子摆在哪 ✓ 没这条的话轮子全堆在原点 ✗
                # （xacro 里给插件写了 <topic>/joint_states</topic>，两边同名 ✓）
                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                # 两台 2D 雷达：xacro 里用 <topic>scan_low / scan_high</topic>
                # 指定了名字，所以两边同名（和真实 RPLIDAR 驱动的命名习惯一致）
                '/scan_low@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/scan_high@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                # 里程计：xacro 的插件里写死了 <odom_topic>/mini_car/odom</odom_topic>
                # 两边同名 → 可以用 @ 简写（不像 /joint_states 那样要 YAML）
                '/mini_car/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                # Mid-360：⚠ gz 侧实际话题是 /mid360/points（不是 /mid360）——
                #   垂直方向不止一个采样时，点云发到 <topic>/points ✗
                #   （2D 雷达反过来：LaserScan 在 <topic>，点云才在 <topic>/points）
                '/mid360/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                # IMU（Mid-360 内置的那个）
                # ⚠ 类型名是 gz.msgs.IMU【全大写】✗ 写成 gz.msgs.Imu 会报
                #   "Failed to create a bridge"（映射表里只有大写那个）
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',

                # ---------- RealSense D435 / D455（RGB-D 相机）----------
                # gz 的 rgbd_camera 在 <topic>d435</topic> 下发四个子话题：
                #     /d435/image        彩色图      gz.msgs.Image
                #     /d435/depth_image  深度图      gz.msgs.Image
                #     /d435/points       点云        gz.msgs.PointCloudPacked
                #     /d435/camera_info  内参        gz.msgs.CameraInfo
                # ⚠ 和真机 realsense2_camera 的话题名【不一样】✗
                #   真机是 /camera/color/image_raw、/camera/depth/image_rect_raw …
                #   想让算法在仿真/真机之间直接换，得再加一层 remap ✓
                '/d435/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/d435/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/d435/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/d435/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                # D455 的四个桥接在下面 ④c 那个节点里（带条件，默认不桥）✗
                # ⚠ 仿真时钟【不能】写在这里 ✗ 原因见下面 ④b 那个节点：
                #   gz 侧的时钟话题叫 /world/nuwa_field/clock，不叫 /clock，
                #   而这里的命令行写法两边【必须同名】→ 桥不出来 ✗
            ],
            output='screen',
        ),

        # ---------- ④b 仿真时钟（单独一个桥，因为两侧名字不一样）----------
        # ⚠⚠ 这个节点是【必须】的，少了它整套建图/定位都发不出 TF ✗
        #
        #   gz-sim 把时钟发在 【/world/<世界名>/clock】，不是 /clock。
        #   而 ④ 那种命令行写法两边必须同名，桥不出来 ✗ → 改用 YAML 配置。
        #
        #   后果链（实测踩过）：
        #     ROS 侧 /clock 空 → use_sim_time 的节点时钟永远停在 0
        #     → GLIM 的 tf buffer 认为"现在"是 0 秒
        #     → 它拿点云的仿真时间戳（几百秒）去 lookupTransform
        #     → 被判定成"查询未来"，抛异常
        #     → GLIM 的 map→odom→base_link 这条 TF 链一条都发不出来 ✗
        #     （GLIM 内部照样算得好好的，只是发不出来，症状很误导）
        #
        #   想自查：ros2 topic hz /clock --qos-reliability best_effort
        #           （/clock 是 BEST_EFFORT 发的，不加这个参数收不到，会误判成"没数据"）
        #
        #   ⚠ 传 YAML 用的是【ROS 参数 config_file】，不是命令行选项 ✗
        #     写成 arguments=['--config-file', ...] 的话，parameter_bridge 会
        #     把 --config-file 当成话题参数、打印一大段用法说明然后【直接退出】，
        #     而 launch 只报 "process started"，不报错 —— 很难发现 ✗
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': BRIDGE_YAML}],
            output='screen',
        ),

        # ---------- ④c D455 的桥接（默认不桥）----------
        # 只在 d455:=true 时才起，四个话题和 D435 一样 ✓
        # 不起的话，D455 的传感器根本不存在（xacro 里被 xacro:if 去掉了），
        # 所以也不会有 /d455/* 话题 ✓ 不会留一堆空话题误导人 ✓
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            condition=IfCondition(LaunchConfiguration('d455')),
            arguments=[
                '/d455/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/d455/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/d455/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/d455/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            ],
            output='screen',
        ),

        # ---------- ⑤ 舵轮控制器 ----------
        # 从 mini_car_control 包启动（所以 launch 前要先 source 工作空间 ✗）
        Node(
            package='mini_car_control',
            executable='swerve_controller',
            output='screen',
        ),

        # ---------- ⑥ 两台 2D 雷达坐标系的静态变换 ----------
        # 雷达的 frame_id 是 "mini_car/base_link/lidar_low" / "…/lidar_high"，
        # 这个名字只存在于 Gazebo 的 SDF 里 ✗ URDF/TF 里没有
        #   → RViz 会报 "No transform from ..."
        #   → tower_check 也拿不到雷达在车体系里的安装位置 ✗
        #
        # （试过在 <sensor> 里写 <gz_frame_id>base_link</gz_frame_id>，
        #   这个版本 gz-sensors 8.2.2 不生效，所以用静态变换代替）
        #
        # ⚠ 这两个 xyz 必须和 xacro 里 <sensor> 的 <pose>、对应 link 的
        #   <joint origin> 完全一致 —— 三处改一处就要同步另两处 ✗
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.42', '--y', '0.0', '--z', '-0.125',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'mini_car/base_link/lidar_low'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.26',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'mini_car/base_link/lidar_high'],
            output='screen',
        ),

        # ---------- ⑦ Mid-360 的坐标系 ----------
        # 同上：Gazebo 给的 frame_id 是 "mini_car/base_link/mid360"，
        # TF 树里没有 → 补一个静态变换。z=0.40 要和 xacro 里传感器的 <pose> 一致 ✓
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.40',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'mini_car/base_link/mid360'],
            output='screen',
        ),

        # ---------- ⑧ IMU 的坐标系 ----------
        # z=0.30，同样和 xacro 对齐 ✓
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.30',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'mini_car/base_link/imu'],
            output='screen',
        ),

        # ---------- ⑨ RealSense 两台相机的坐标系 ----------
        # gz 给的 frame_id 是 'mini_car/base_link/d435' / '.../d455'
        # （和 mid360 / lidar_low / lidar_high 同一个命名规则），TF 树里没有
        # → 各补一条静态变换。
        # ⚠ xyz 必须和 xacro 里对应 <joint> 的 <origin> 一致，改一处要改两处 ✗
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.36', '--y', '0.0', '--z', '0.10',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'mini_car/base_link/d435'],
            output='screen',
        ),
        # D455 那条只在 d455:=true 时才发 ✓ 否则 TF 树里会多一个孤立的 frame ✗
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            condition=IfCondition(LaunchConfiguration('d455')),
            arguments=['--x', '0.36', '--y', '0.0', '--z', '0.22',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'mini_car/base_link/d455'],
            output='screen',
        ),
    ])
