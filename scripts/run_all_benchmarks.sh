#!/usr/bin/env bash
# Run benchmarks for ALL algorithms

echo "[run_all_benchmarks.sh] Running comprehensive benchmarks..."

./scripts/run.sh --algo bfs --grid 64
./scripts/run.sh --algo dijkstra --grid 64
./scripts/run.sh --algo greedy --grid 64  
./scripts/run.sh --algo astar --grid 64
./scripts/run.sh --algo weighted_astar --grid 64 --weight 1.5
./scripts/run.sh --algo theta_star --grid 64
./scripts/run.sh --algo jps --grid 64

echo "[run_all_benchmarks.sh] All benchmarks completed!"
