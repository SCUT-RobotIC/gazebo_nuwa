#!/bin/bash
# 启动 tower_check：只判 6 个公共位，每秒打一次表。
# 用法：
#   bash tools/run_tower_check.sh
#   TEAM=blue bash tools/run_tower_check.sh
#
# 第一行必须是：tower_check v3（双雷达）  模块=/home/wyq/work/nuwa_sim/...

set -e
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/env.sh"

TEAM=${TEAM:-red}
exec ros2 run mini_car_control tower_check --ros-args \
  -p team:="$TEAM" \
  -p log_every:=10 \
  -p only_spots:=L1_W_M,L1_E_M,L2_SW,L2_SE,L2_NW,L2_NE
