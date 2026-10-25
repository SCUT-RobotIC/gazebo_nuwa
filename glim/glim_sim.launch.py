#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 Gazebo 仿真用的 GLIM 启动文件。

和真机那份（BHT_ROS2/r1_decision/launch/glim.launch.py）的区别：

    真机  → 先起 livox_ros_driver2 驱动，再起 GLIM，use_sim_time=false
    仿真  → 【不起驱动】（仿真里没有真雷达）
            话题由 mini_car.launch.py 里的 ros_gz_bridge 提供
            use_sim_time=true
            配置指向本文件同级的 config/（仿真专用那份）

⚠ 跑之前要先让 glim_ros 包可见（安装方式见 GLIM编译指南.md）：
      source /opt/ros/humble/setup.bash
      source <装 glim_ros 的那个工作空间>/install/setup.bash    ← 用 PPA 装的话不用这句
  少了第二个会报 "package 'glim_ros' not found" ✗
  （libgtsam_points 的路径由本文件自动处理，见下面 SetEnvironmentVariable）

用法（三个终端，<项目路径> 换成你解压出来的目录）：

    ①  ros2 launch <项目路径>/robots/src/mini_car/launch/mini_car.launch.py \\
            world:=<项目路径>/field/worlds/nuwa_field_room.sdf

    ②  ros2 launch <项目路径>/glim/glim_sim.launch.py        ← 本文件

    ③  rviz2 -d <项目路径>/glim/glim.rviz
        （想看地图的话；GLIM 会发 map -> odom -> base 的 TF）
"""

import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

# 仿真专用配置目录。GLIM 的 config_path 要的是【目录】不是文件 ——
# 它按 config.json 里的清单去这个目录下找各个子配置。
# 从本文件位置推出来（上一级就是项目根），这样整包挪到哪都能跑 ✓
_HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(_HERE, 'config')

# ⚠ glim_rosnode 依赖 libgtsam_points.so.1，而那个库来自一条【独立编译链】
#   （gtsam / gtsam_points / iridescence），装在一个普通的 CMake 前缀下。
#   那里只有 include/lib/share、没有 setup.bash，所以任何 ROS 工作空间的
#   setup 都不会把它加进 LD_LIBRARY_PATH ✗
#   偏偏 glim_rosnode 二进制里【没有 RPATH】（readelf -d 可查），
#   运行时只能靠环境变量找 → 不加就报：
#       error while loading shared libraries: libgtsam_points.so.1
#
#   路径优先级：
#     ① 环境变量 GLIM_DEPS_LIB（推荐 —— 装在哪就指哪，不用改文件）
#     ② ~/rc_deps/install/lib（本机默认，见 GLIM编译指南.md）
RC_DEPS_LIB = os.environ.get(
    'GLIM_DEPS_LIB', os.path.expanduser('~/rc_deps/install/lib'))


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            RC_DEPS_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', ''),
        ),
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_rosnode',
            output='screen',
            parameters=[
                {'config_path': CONFIG_DIR},
                # ⚠ 必须开。mini_car.launch.py 把 /clock 桥过来了，
                #   不开的话 GLIM 的 ROS 时钟走墙上时间，而点云/IMU 的时间戳
                #   是仿真时间 → TF 时间戳和数据时间戳对不上，RViz 会报
                #   "message filter dropping message" 画不出东西 ✗
                {'use_sim_time': True},
            ],
        ),
    ])
