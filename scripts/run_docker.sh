#!/usr/bin/env bash
# One-command runner: (re)build image, then run your bench with any args you pass.
# Examples:
#   ./scripts/run_docker.sh --steps 10 --render-every 2 --no-show
#   ./scripts/run_docker.sh --grid 64 --fill 0.20 --seed 42 --no-show
set -euo pipefail

IMAGE="rbe550-bench"

# Always build to guarantee it's in sync with your code (simple & reliable)
docker build -t "${IMAGE}" .

# Persist outputs to host
mkdir -p outputs

# If no args given, provide a safe default
if [[ $# -eq 0 ]]; then
  set -- --steps 10 --render-every 2 --no-show
fi

# Run. Thanks to the ENTRYPOINT, we can call ros2 directly.
docker run --rm -it \
  -v "$(pwd)/outputs:/ws/outputs" \
  "${IMAGE}" \
  ros2 run rbe550_grid_bench bench "$@"

