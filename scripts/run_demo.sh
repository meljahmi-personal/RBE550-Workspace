#!/usr/bin/env bash
# Run demo - NO HARDCODED PATHS

set -e

source "$(dirname "$0")/activate.sh"

echo "[run_demo] Starting benchmark demo..."
ros2 run rbe550_grid_bench bench --algo astar --show
