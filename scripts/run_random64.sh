#!/bin/bash
# Run BFS and A* on a seeded random 64x64 grid with 20% obstacle fill
# Seed fixed for reproducibility

SEED=42
SIZE=64
FILL=0.20

echo "[Random ${SIZE}x${SIZE}, fill=${FILL}, seed=${SEED}]"

# BFS (4-connected)
./scripts/run.sh --grid $SIZE --fill $FILL --seed $SEED --algo bfs   --moves 4 --no-show

# A* (4-connected, Manhattan)
./scripts/run.sh --grid $SIZE --fill $FILL --seed $SEED --algo astar --moves 4 --no-show

# A* (8-connected, Euclidean with √2 diagonals)
./scripts/run.sh --grid $SIZE --fill $FILL --seed $SEED --algo astar --moves 8 --no-show

