#!/usr/bin/env bash
# Build RBE-550 workspace (merged layout + symlinks), clean env first.
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"

# Optional clean: ./scripts/build.sh clean
if [[ "${1:-}" == "clean" ]]; then
  rm -rf "$WS/build" "$WS/install" "$WS/log"
fi

# Ensure ament resource marker exists (harmless if already there)
mkdir -p "$WS/src/rbe550_grid_bench/resource"
echo -n "rbe550_grid_bench" > "$WS/src/rbe550_grid_bench/resource/rbe550_grid_bench"

# Clean overlays to avoid weirdness
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX ROS_PACKAGE_PATH PYTHONPATH

# Source ROS only
set +u
source /opt/ros/humble/setup.bash
set -u

# Build this package (merged layout + symlinks)
colcon build --symlink-install --packages-select rbe550_grid_bench

# Quick sanity: show that the package is registered
source "$WS/install/setup.bash"
echo "[OK] Build finished."
echo "[info] ros2 sees: $(ros2 pkg list | grep -i rbe550 || echo 'NOT FOUND')"

