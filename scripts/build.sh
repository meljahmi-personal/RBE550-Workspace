#!/usr/bin/env bash
# Final stable local build for RBE550 Workspace

rm -rf build/ install/ log/

# no 'set -u' here on purpose
set -e

# Make sure COLCON_TRACE exists (prevents "unbound variable")
export COLCON_TRACE="${COLCON_TRACE:-}"

# Source ROS 2
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --merge-install

# Source the overlay (safe)
source install/setup.bash || true

echo "[build.sh]  Build complete."


