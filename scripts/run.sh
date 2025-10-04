#!/usr/bin/env bash
# Final stable local run for RBE550 Workspace

# no 'set -u' here on purpose
set -e

# Make sure COLCON_TRACE exists (prevents "unbound variable")
export COLCON_TRACE="${COLCON_TRACE:-}"

# Source ROS 2
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

# Ensure workspace is built
if [[ ! -f install/setup.bash ]]; then
  echo "[run.sh]  Workspace not built. Run: ./scripts/build.sh"
  exit 1
fi

# Source the overlay (safe)
source install/setup.bash || true

echo "[run.sh]  Running rbe550_grid_bench..."
ros2 run rbe550_grid_bench bench "$@"

