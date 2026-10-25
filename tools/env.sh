#!/bin/bash
# 每个终端先：source ~/work/nuwa_sim/tools/env.sh
# 作用：在 .bashrc 的旧 gazebo_nuwa 之后，把环境覆盖成新工作空间。
# 直接 bash env.sh 只会影响子进程；要影响当前终端必须 source。

source /opt/ros/humble/setup.bash

NEW="$HOME/work/nuwa_sim/robots/install/setup.bash"
if [ ! -f "$NEW" ]; then
  echo "✗ 找不到 $NEW"
  echo "  先在 ~/work/nuwa_sim/robots 下 colcon build --symlink-install"
  return 1 2>/dev/null || exit 1
fi
source "$NEW"

PREFIX=$(ros2 pkg prefix mini_car_control 2>/dev/null)
echo "mini_car_control -> ${PREFIX:-（解析失败）}"
case "$PREFIX" in
  *work/nuwa_sim*) ;;
  *) echo "⚠ 仍是旧工作空间：确认 source 顺序（新 install 必须在 gazebo_nuwa 之后）" ;;
esac
