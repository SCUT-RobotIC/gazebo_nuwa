#!/bin/bash
# 放方块：固定用新工作空间的 place_block。
# 用法：
#   bash tools/place_block.sh L1_W_M
#   bash tools/place_block.sh L1_W_M 0.05 0 15 1
#   bash tools/place_block.sh L1_E_M --clear-only
#   bash tools/place_block.sh L1_E_M --clear --dry-run

set -e
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/env.sh"

exec ros2 run mini_car_control place_block "$@"
