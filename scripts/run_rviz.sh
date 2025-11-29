#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "[run_rviz] Sourcing ROS 2..."
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

echo "[run_rviz] Sourcing workspace install/setup.bash..."
source "${WS_DIR}/install/setup.bash"

echo "[run_rviz] Launching bench.launch.py ..."
ros2 launch rbe550_grid_bench bench.launch.py use_rviz:=true "$@"

