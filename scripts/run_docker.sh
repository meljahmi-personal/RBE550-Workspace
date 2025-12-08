#!/usr/bin/env bash
set -euo pipefail

IMAGE="rbe550-bench"

usage() {
  cat <<EOF
Usage:
  $0 build
      Build the Docker image.

  $0 bench [bench-args...]
      Run the benchmark CLI inside Docker. Any extra args are passed to the
      'bench' node (e.g. --algo astar --grid 64).

  $0 rviz [planner-args...]
      Run the RViz visualization inside Docker. Extra args are passed to the
      planner_viz launch (e.g. algo:=astar grid:=64).
EOF
}

cmd="${1:-bench}"

case "$cmd" in
  build)
    # Just build the image and exit; do NOT start a container.
    docker build -t "${IMAGE}" .
    echo "Docker image '${IMAGE}' built successfully."
    ;;

  bench)
    shift || true
    mkdir -p outputs
    docker run --rm -it \
      -v "$(pwd)/outputs:/ws/outputs" \
      "${IMAGE}" "$@"
    ;;

  rviz)
    shift || true
    docker run --rm -it \
      -e DISPLAY="${DISPLAY:-}" \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      "${IMAGE}" \
      ros2 launch rbe550_grid_bench planner_viz.launch.py "$@"
    ;;

  *)
    usage
    exit 1
    ;;
esac

