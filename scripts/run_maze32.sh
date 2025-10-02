#!/bin/bash
# Run BFS and A* on a fixed ASCII map with specified start/goal
# Demonstrates difference between restricted 4-connected and 8-connected search

MAP=maps/maze_32.txt
START_GOAL="1,1:7,30"

echo "[ASCII map $MAP, start-goal=$START_GOAL]"

# BFS (4-connected)
./scripts/run.sh --map $MAP --start-goal $START_GOAL --algo bfs   --moves 4 --no-show

# A* (8-connected, Euclidean with √2 diagonals)
./scripts/run.sh --map $MAP --start-goal $START_GOAL --algo astar --moves 8 --no-show

