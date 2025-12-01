#!/usr/bin/env bash
set -euo pipefail

# Optional --plot flag
PLOT=0
if [[ "${1:-}" == "--plot" ]]; then
  PLOT=1
  shift
fi

CSV="results/bench_all.csv"
mkdir -p results

echo "[run_all_benchmarks] Writing to ${CSV}"
echo "algo,moves,grid,fill,obstacles,obstacle_frac,start_r,start_c,goal_r,goal_c,success,path_len,runtime_ms,nodes_expanded,cost,peak_open,peak_closed,seed" > "${CSV}"

# 7 algorithms, single CSV
./scripts/run.sh --algo bfs            --grid 64 --moves 8 --steps 1 --csv "${CSV}"
./scripts/run.sh --algo dijkstra       --grid 64 --moves 8 --steps 1 --csv "${CSV}"
./scripts/run.sh --algo greedy         --grid 64 --moves 8 --steps 1 --csv "${CSV}"
./scripts/run.sh --algo astar          --grid 64 --moves 8 --steps 1 --csv "${CSV}"
./scripts/run.sh --algo weighted_astar --grid 64 --moves 8 --steps 1 --weight 1.5 --csv "${CSV}"
./scripts/run.sh --algo theta_star     --grid 64 --moves 8 --steps 1 --csv "${CSV}"
./scripts/run.sh --algo jps            --grid 64 --moves 8 --steps 1 --csv "${CSV}"

echo "[run_all_benchmarks] Done runs."

if [[ "${PLOT}" -eq 1 ]]; then
  echo "[run_all_benchmarks] Generating plots..."
  ros2 run rbe550_grid_bench plot_bench --csv "${CSV}" --outdir results
fi

