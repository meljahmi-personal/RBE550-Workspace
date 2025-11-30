#!/usr/bin/env bash

cd ~/ros2projrbe550_ws
./scripts/build.sh
source ./scripts/activate.sh

ros2 launch rbe550_grid_bench bench.launch.py \
  algo:=astar \
  moves:=8 \
  grid:=64 \
  fill:=0.20 \
  seed:=42 \
  use_rviz:=true

