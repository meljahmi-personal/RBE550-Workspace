#!/usr/bin/env bash
set -e

GRID=32
FILL=0.20
SEED=42

echo "[Random ${GRID}x${GRID}, fill=${FILL}, seed=${SEED}]"

ALGS=(bfs dijkstra greedy astar weighted_astar theta_star jps)

for algo in "${ALGS[@]}"; do
  moves=8
  extra_args=()

  # Example: keep BFS 4-connected if you want
  if [[ "$algo" == "bfs" ]]; then
    moves=4
  fi

  if [[ "$algo" == "weighted_astar" ]]; then
    extra_args+=(--weight 1.5)
  fi

  ./scripts/run.sh \
    --grid "$GRID" \
    --fill "$FILL" \
    --seed "$SEED" \
    --algo "$algo" \
    --moves "$moves" \
    --steps 1 \
    --no-show \
    "${extra_args[@]}"
done

