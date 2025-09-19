#!/usr/bin/env bash
# Build RBE-550 workspace (merged layout + symlinks), clean env first.

set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"

# Optional: ./scripts/build.sh clean
if [[ "${1:-}" == "clean" ]]; then
  rm -rf "$WS/build" "$WS/install" "$WS/log"
fi

# Clean any stray overlays to avoid warnings
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX ROS_PACKAGE_PATH PYTHONPATH

# Source only ROS Humble, then build
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build --merge-install --symlink-install --packages-select rbe550_grid_bench
echo "[OK] Build finished."

