#!/usr/bin/env bash
# Development workflow script

set -e
# 1. Build (when code changes)
./scripts/build.sh

# 2. setup (in each terminal)
echo "[dev.sh] Activating workspace..." 
source scripts/activate.sh


echo "[dev.sh] Testing package recognition..."
if ros2 pkg list | grep -q rbe550_grid_bench; then
    echo "✅ Package found in ROS2!"
    echo "[dev.sh] Running quick test..."
    ros2 run rbe550_grid_bench bench --algo astar --grid 16 --no-show
    # 3. Test/Run
    ros2 run rbe550_grid_bench bench --algo astar --show
    # or use your specific scripts:
    ./scripts/run_maze32.sh
    ./scripts/run_random64.sh
else
    echo "❌ Package not found in ROS2"
    echo "[dev.sh] Debug info:"
    find install/ -name "rbe550_grid_bench" -type f | head -5
fi
