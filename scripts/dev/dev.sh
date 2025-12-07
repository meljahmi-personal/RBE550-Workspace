#!/usr/bin/env bash
# Development workflow script

set -e

echo "[dev.sh] Building workspace..."
./scripts/build.sh

echo "[dev.sh] Activating workspace..." 
source scripts/activate.sh

# 3. Test/Run
ros2 run rbe550_grid_bench bench --algo astar --show


ros2 run rbe550_grid_bench bench --grid 16 --fill 0.0 --algo jps --moves 4 --no-show

ros2 run rbe550_grid_bench bench --grid 16 --fill 0.0 --algo jps --moves 4 --show


# or use specific scripts:
./scripts/run_maze32.sh
./scripts/run_random64.sh
