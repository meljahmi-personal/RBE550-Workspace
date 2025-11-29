#!/usr/bin/env bash
# Development workflow script

set -e

# 1. Build (when code changes)
./scripts/build.sh

# 2. Activate (in each terminal)
# Correct way to use activate.sh:
source scripts/activate.sh

# 3. Test/Run
ros2 run rbe550_grid_bench bench --algo astar --show
# or use your specific scripts:
./scripts/run_maze32.sh
./scripts/run_random64.sh
