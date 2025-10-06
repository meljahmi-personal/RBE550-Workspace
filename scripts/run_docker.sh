#!/usr/bin/env bash
# One-command runner: (re)build image, then run your bench with any args you pass.
# Examples:
#   ./scripts/run_docker.sh --steps 10 --render-every 2 --no-show
#   ./scripts/run_docker.sh --grid 64 --fill 0.20 --seed 42 --no-show
set -euo pipefail

IMAGE="rbe550-bench"

# Always rebuild to ensure image matches latest code
docker build -t "${IMAGE}" .

# Ensure host outputs/ exists (needed for bind mount and host-side logging)
mkdir -p outputs

# Default arguments if none provided
if [[ $# -eq 0 ]]; then
  set -- --steps 10 --render-every 2 --no-show
fi

# Run container. ENTRYPOINT handles ROS setup and execution.
docker run --rm -it \
  -v "$(pwd)/outputs:/ws/outputs" \
  "${IMAGE}" \
  "$@"


