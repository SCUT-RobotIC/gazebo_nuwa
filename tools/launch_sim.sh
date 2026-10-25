#!/bin/bash
# 启动仿真：固定用源码里的 launch（不会解析到旧包的 mini_car）。
# 用法：
#   bash tools/launch_sim.sh
#   bash tools/launch_sim.sh spawn_x:=0.0 spawn_y:=-2.25 spawn_z:=0.901 spawn_yaw:=0.0

set -e
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/env.sh"

exec ros2 launch "$HOME/work/nuwa_sim/robots/src/mini_car/launch/mini_car.launch.py" "$@"
