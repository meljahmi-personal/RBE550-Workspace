#!/usr/bin/env bash
set -euo pipefail

# ===== PARSE ARGUMENTS FIRST =====
PLOT=0
SEED=42

while [[ $# -gt 0 ]]; do
  case $1 in
    --plot)
      PLOT=1
      shift
      ;;
    --seed)
      SEED="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

echo "[run_all_benchmarks] Starting with seed=${SEED}, plot=${PLOT}"

# ===== Build package and source environment =====
echo "[run_all_benchmarks] Building rbe550_grid_bench package..."
cd ~/ros2projrbe550_ws
rm -rf build/ install/ log/
colcon build --packages-select rbe550_grid_bench

# Source the environment with error handling
echo "[run_all_benchmarks] Sourcing environment..."
if [[ -f "install/setup.bash" ]]; then
  # Temporarily disable unbound variable checking to avoid COLCON_TRACE error
  set +u
  source install/setup.bash 2>/dev/null || true
  set -u
  echo "[run_all_benchmarks] Environment sourced."
else
  echo "[run_all_benchmarks] WARNING: setup.bash not found"
fi
# ===== End build section =====

CSV="results/bench_all.csv"
mkdir -p results

echo "[run_all_benchmarks] Writing to ${CSV} with seed=${SEED}"
# Remove existing CSV to ensure fresh start
rm -f "${CSV}"

# 7 algorithms, all using the SAME seed for fair comparison
./scripts/run.sh --algo bfs            --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"
./scripts/run.sh --algo dijkstra       --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"
./scripts/run.sh --algo greedy         --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"
./scripts/run.sh --algo astar          --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"
./scripts/run.sh --algo weighted_astar --grid 64 --moves 8 --steps 1 --weight 1.5 --csv "${CSV}" --seed "${SEED}"
./scripts/run.sh --algo theta_star     --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"
./scripts/run.sh --algo jps            --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"

echo "[run_all_benchmarks] Done runs."

if [[ "${PLOT}" -eq 1 ]]; then
  echo "[run_all_benchmarks] Generating plots..."
  
  # Try ros2 run first, fall back to direct Python call
  if command -v ros2 >/dev/null 2>&1; then
    echo "[run_all_benchmarks] Using ros2 run..."
    ros2 run rbe550_grid_bench plot_bench --csv "${CSV}" --outdir results 2>&1 || {
      echo "[run_all_benchmarks] ros2 run failed, trying direct Python..."
      python3 src/rbe550_grid_bench/rbe550_grid_bench/plot_bench_all.py --csv "${CSV}" --outdir results
    }
  else
    echo "[run_all_benchmarks] Using direct Python call..."
    python3 src/rbe550_grid_bench/rbe550_grid_bench/plot_bench_all.py --csv "${CSV}" --outdir results
  fi
  
  echo "[run_all_benchmarks] Checking generated plots..."
  ls -la results/*.png 2>/dev/null || echo "[run_all_benchmarks] No plots generated"
fi
