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

# make sure outputs exists (keep this near the top if you added it)
mkdir -p "$(dirname "$0")/../outputs"

# timestamped log
ts="$(date +%Y%m%d_%H%M%S)"
echo "[run.sh]  Running rbe550_grid_bench... (logging to outputs/run_${ts}.log)"
ros2 run rbe550_grid_bench bench "$@" | tee "outputs/run_${ts}.log"



