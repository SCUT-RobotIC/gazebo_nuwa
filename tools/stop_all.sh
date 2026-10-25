#!/bin/bash
# 停掉本项目相关的 Gazebo / ROS 进程。
# 只匹配本项目的名字（nuwa_field / mini_car）；不动 ROS daemon。
# 用法：bash tools/stop_all.sh

patterns=(
  "nuwa_field.sdf"
  "mini_car.launch.py"
  "mini_car_control"
  "mini_car/base_link"
  "ros_gz_bridge"
  "robot_state_publisher"
  "swerve_controller"
  "odom_to_tf"
  "tower_check"
  "place_block"
)

for sig in TERM KILL; do
  for pat in "${patterns[@]}"; do
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
      [ "$pid" = "$$" ] && continue
      # 别杀 shell：parent shell 的命令行里可能也含这些词（比如
      # `chmod ... run_tower_check.sh`），一杀就把调用者干掉了 ✗
      comm=$(ps -o comm= -p "$pid" 2>/dev/null)
      case "$comm" in bash|sh|zsh|dash|fish) continue ;; esac
      kill -"$sig" "$pid" 2>/dev/null && echo "kill -$sig $pid  ($pat)"
    done
  done
  sleep 1
done

echo "--- 剩余 ---"
pgrep -af "nuwa_field|mini_car|ros_gz_bridge|robot_state_publisher" 2>/dev/null \
  | grep -v pgrep || echo "（无）"
