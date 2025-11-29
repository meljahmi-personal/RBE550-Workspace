#!/usr/bin/env bash
# Development workflow script

set -e

echo "[dev.sh] Building workspace..."
./scripts/build.sh

echo "[dev.sh] Activating workspace..." 
source scripts/activate.sh

# 3. Test/Run
ros2 run rbe550_grid_bench bench --algo astar --show
# or use your specific scripts:
./scripts/run_maze32.sh
./scripts/run_random64.sh
