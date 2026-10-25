#!/bin/bash
# 预检：确认 ros2 run 解析到 ~/work/nuwa_sim/robots/install，
# 而不是 .bashrc 里先 source 的旧 ~/gazebo_nuwa install。
#
# 用法：
#   bash ~/work/nuwa_sim/tools/check_env.sh
#
# 只读，不会启动/停止任何东西。

set +u
source /opt/ros/humble/setup.bash

NEW="$HOME/work/nuwa_sim/robots/install/setup.bash"
if [ ! -f "$NEW" ]; then
  echo "✗ 找不到 $NEW"
  echo "  先在 ~/work/nuwa_sim/robots 下 colcon build --symlink-install"
  exit 1
fi
# 必须排在 .bashrc 的 gazebo_nuwa 之后，才能覆盖旧包
source "$NEW"

PREFIX=$(ros2 pkg prefix mini_car_control 2>/dev/null)
echo "mini_car_control -> ${PREFIX:-（解析失败）}"
case "$PREFIX" in
  *work/nuwa_sim*) echo "✓ 新工作空间" ;;
  *) echo "✗ 还是旧工作空间：请手动 source $NEW（放在 gazebo_nuwa 之后）" ;;
esac

echo "--- 可执行 ---"
ros2 pkg executables mini_car_control 2>/dev/null | sort

echo "--- 节点 ---"
ros2 node list 2>/dev/null | grep -E "tower_check|odom_to_tf|robot_state_publisher" \
  || echo "（没有 tower_check / odom_to_tf）"

echo "--- 话题 ---"
ros2 topic list 2>/dev/null \
  | grep -E "scan_low|scan_high|tower_check|mini_car/odom" \
  || echo "（没有扫描/状态话题；仿真还没起？）"

echo "--- 启动后自查 ---"
echo "tower_check 第一行应包含：tower_check v3（双雷达）"
echo "还应打印：低雷达 ← /scan_low   高雷达 ← /scan_high"
echo "如果是旧包：没有这一行，且订阅 /scan（新 launch 不发布 /scan）"
