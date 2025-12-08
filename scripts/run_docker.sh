#!/usr/bin/env bash
set -euo pipefail

IMAGE="rbe550-bench"
WS_IN_CONTAINER="/ws"

usage() {
  cat <<EOF
Usage:
  $0 build
      Build the Docker image (ROS 2 Humble + rbe550_grid_bench).

  $0 bench [bench-args...]
      Run the 'bench' CLI inside Docker.
      Example:
        $0 bench --algo astar --grid 64 --moves 8 --steps 5 --no-show

  $0 bench_all
      Inside Docker, run the full benchmark sweep:
        ./scripts/run_all_benchmarks.sh --seed 42 --plot
      Results and plots are written under ./results on the host.

  $0 rviz [planner-args...]
      (Optional) Try to run RViz inside Docker using bench.launch.py.
      Example:
        $0 rviz algo:=astar grid:=64 moves:=8
EOF
  exit 1
}

cmd="${1:-}"
if [[ -z "${cmd}" ]]; then
  usage
fi
shift || true

case "${cmd}" in
  build)
    docker build -t "${IMAGE}" .
    echo "Docker image '${IMAGE}' built successfully."
    ;;

  bench)
    mkdir -p results
    docker run --rm -it \
      -v "$(pwd)/results:${WS_IN_CONTAINER}/results" \
      "${IMAGE}" \
      ros2 run rbe550_grid_bench bench "$@"
    ;;

  bench_all)
    mkdir -p results
    docker run --rm -it \
      -v "$(pwd)/results:${WS_IN_CONTAINER}/results" \
      "${IMAGE}" \
      bash -lc "cd ${WS_IN_CONTAINER} && ./scripts/run_all_benchmarks.sh --seed 42 --plot"
    ;;

  rviz)
    # NOTE: this requires X11 / Wayland forwarding on the host.
    # This is optional ; host RViz is the primary workflow.
    xhost +local:root >/dev/null 2>&1 || true
    docker run --rm -it \
      -e DISPLAY \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      "${IMAGE}" \
      ros2 launch rbe550_grid_bench bench.launch.py "$@"
    ;;

  *)
    usage
    ;;
esac

