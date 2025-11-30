#!/usr/bin/env bash
#
# scripts/feed_rviz_demo.sh
# Drive /planner_node with random algo/moves over time while RViz is running.

set -e

# Go to workspace root (this script lives in scripts/)
cd "$(dirname "$0")/.."

echo "[feed] Sourcing environment..."
source ./scripts/activate.sh

NODE="/planner_node"
SRV="/randomize_grid"

# All supported planners
ALGOS=(bfs dijkstra greedy astar weighted_astar theta_star jps)

# How many seconds to wait between requests
SLEEP_SEC=4

echo "[feed] Waiting for ${NODE} and RViz..."
ros2 node list
echo

# Quick sanity check that planner node exists
if ! ros2 node list | grep -q "${NODE}"; then
  echo "[feed] ERROR: ${NODE} not found. Make sure you ran:"
  echo "         ros2 launch rbe550_grid_bench bench.launch.py use_rviz:=true"
  exit 1
fi

echo "[feed] Driving ${NODE} via parameters + ${SRV}"
echo "       Press Ctrl-C to stop."
echo

# Choose moves based on algo
choose_moves() {
  local algo="$1"
  case "$algo" in
    theta_star)
      echo 8        # any-angle, 8-connected
      ;;
    jps)
      echo 4        # you implemented 4-connected JPS
      ;;
    *)
      # random choice: 4 or 8
      if (( RANDOM % 2 )); then
        echo 4
      else
        echo 8
      fi
      ;;
  esac
}

# For weighted A*, pick a random-ish weight in [1.0, 3.0]
random_weight() {
  # 10, 15, 20, 25, 30 -> 1.0 .. 3.0
  local raw=$(( (RANDOM % 5) * 5 + 10 ))
  printf "%.1f" "$(bc <<< "$raw / 10.0")"
}

# One cycle through all algorithms in random order
run_one_round() {
  # Make a shuffled copy of ALGOS
  local shuffled=("${ALGOS[@]}")
  # Fisher–Yates shuffle
  for (( i=${#shuffled[@]}-1; i>0; i-- )); do
    j=$(( RANDOM % (i+1) ))
    tmp=${shuffled[i]}
    shuffled[i]=${shuffled[j]}
    shuffled[j]=$tmp
  done

  for algo in "${shuffled[@]}"; do
    local moves
    moves=$(choose_moves "$algo")

    echo
    echo "[feed] ------------------------------------------"
    echo "[feed] algo=${algo}, moves=${moves}"

    # Set algo + moves
    ros2 param set "${NODE}" algo "${algo}" > /dev/null
    ros2 param set "${NODE}" moves "${moves}" > /dev/null

    # If weighted_astar, also set weight
    if [[ "$algo" == "weighted_astar" ]]; then
      local w
      w=$(random_weight)
      echo "[feed]   weight=${w}"
      ros2 param set "${NODE}" weight "${w}" > /dev/null
    fi

    # Call randomize_grid so the planner replans and RViz updates
    echo "[feed]   calling ${SRV}..."
    ros2 service call "${SRV}" std_srvs/srv/Trigger "{}"

    echo "[feed]   sleeping ${SLEEP_SEC}s so you can watch RViz..."
    sleep "${SLEEP_SEC}"
  done
}

# Main loop: keep running rounds until Ctrl-C
while true; do
  run_one_round
done

