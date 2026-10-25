#!/usr/bin/env bash
# Build the Mid-360 gz-sim system plugin in place.
#
# Output:
#   build/libmid360_lidar_system.so
#
# mini_car.launch.py adds this build/ directory to
# GZ_SIM_SYSTEM_PLUGIN_PATH automatically.
set -euo pipefail

cd "$(dirname "$0")"

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"

echo
echo "built: $(pwd)/build/libmid360_lidar_system.so"
