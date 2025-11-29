#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "[activate.sh] Sourcing ROS 2..."
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

echo "[activate.sh] Sourcing workspace..."
source "$WORKSPACE_DIR/install/setup.bash"

echo "[activate.sh] Workspace activated!"
echo "Available commands:"
echo "  ros2 run rbe550_grid_bench bench [options]"
echo "  ros2 launch rbe550_grid_bench bench.launch.py"
