#!/usr/bin/env bash
set -e

echo "[Random 64x64, fill=0.20, seed=42]"

COMMON_ARGS="--grid 64 --fill 0.2 --seed 42 --steps 1 --render-every 4 --enemies 10 --moves 8"

./scripts/run.sh $COMMON_ARGS --algo bfs
./scripts/run.sh $COMMON_ARGS --algo dijkstra
./scripts/run.sh $COMMON_ARGS --algo greedy
./scripts/run.sh $COMMON_ARGS --algo astar
./scripts/run.sh $COMMON_ARGS --algo weighted_astar --weight 1.5
./scripts/run.sh $COMMON_ARGS --algo theta_star
./scripts/run.sh $COMMON_ARGS --algo jps

