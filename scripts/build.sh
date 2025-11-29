#!/usr/bin/env bash
# Main build script - NO HARDCODED PATHS

set -e

echo "[build.sh] Cleaning workspace..."
rm -rf build/ install/ log/

echo "[build.sh] Sourcing ROS 2..."
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

echo "[build.sh] Building workspace..."
colcon build --symlink-install

echo "[build.sh] Build complete."
