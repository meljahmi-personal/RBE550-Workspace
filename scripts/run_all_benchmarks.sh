#!/usr/bin/env bash
set -euo pipefail

# ----- Locate repo root (parent of scripts/) -----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${SCRIPT_DIR}/.."

# ----- Parse arguments -----
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

# ----- Build package in this clone -----
echo "[run_all_benchmarks] Building rbe550_grid_bench package..."
cd "${ROOT}"
rm -rf build/ install/ log/
colcon build --packages-select rbe550_grid_bench

# ----- Source the workspace we just built -----
echo "[run_all_benchmarks] Sourcing environment..."
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  # Temporarily disable unbound variable checking to avoid COLCON_TRACE issues
  set +u
  source "${ROOT}/install/setup.bash" 2>/dev/null || true
  set -u
  echo "[run_all_benchmarks] Environment sourced."
else
  echo "[run_all_benchmarks] WARNING: ${ROOT}/install/setup.bash not found"
fi

# ----- Ensure results directory under this repo -----
RESULTS_DIR="${ROOT}/results"
CSV="${RESULTS_DIR}/bench_all.csv"

mkdir -p "${RESULTS_DIR}"

echo "[run_all_benchmarks] Writing to ${CSV} with seed=${SEED}"
# Remove existing CSV to ensure fresh start
rm -f "${CSV}"

# ----- Run the 7 algorithms (all same seed) -----
# Use SCRIPT_DIR so this works no matter current working dir
RUN_SH="${SCRIPT_DIR}/run.sh"

"${RUN_SH}" --algo bfs            --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"
"${RUN_SH}" --algo dijkstra       --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"
"${RUN_SH}" --algo greedy         --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"
"${RUN_SH}" --algo astar          --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"
"${RUN_SH}" --algo weighted_astar --grid 64 --moves 8 --steps 1 --weight 1.5 --csv "${CSV}" --seed "${SEED}"
"${RUN_SH}" --algo theta_star     --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"
"${RUN_SH}" --algo jps            --grid 64 --moves 8 --steps 1 --csv "${CSV}" --seed "${SEED}"

echo "[run_all_benchmarks] Done runs."

# ----- Optional plotting -----
if [[ "${PLOT}" -eq 1 ]]; then
  echo "[run_all_benchmarks] Generating plots..."

  # Try ros2 run first, fall back to direct Python call
  if command -v ros2 >/dev/null 2>&1; then
    echo "[run_all_benchmarks] Using ros2 run..."
    ros2 run rbe550_grid_bench plot_bench \
      --csv "${CSV}" \
      --outdir "${RESULTS_DIR}" 2>&1 || {
        echo "[run_all_benchmarks] ros2 run failed, trying direct Python..."
        python3 "${ROOT}/src/rbe550_grid_bench/rbe550_grid_bench/plot_bench_all.py" \
          --csv "${CSV}" \
          --outdir "${RESULTS_DIR}"
      }
  else
    echo "[run_all_benchmarks] Using direct Python call..."
    python3 "${ROOT}/src/rbe550_grid_bench/rbe550_grid_bench/plot_bench_all.py" \
      --csv "${CSV}" \
      --outdir "${RESULTS_DIR}"
  fi

  echo "[run_all_benchmarks] Checking generated plots..."
  ls -la "${RESULTS_DIR}"/*.png 2>/dev/null || echo "[run_all_benchmarks] No plots generated"
fi

