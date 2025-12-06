#!/usr/bin/env bash
# One-command runner: (re)build image, then run your bench with any args you pass.
# Examples:
#   ./scripts/run_docker.sh --steps 10 --render-every 2 --no-show
#   ./scripts/run_docker.sh ./scripts/run_rviz.sh
set -euo pipefail
IMAGE="rbe550-bench"

docker build -t "${IMAGE}" .

mkdir -p outputs

if [[ $# -eq 0 ]]; then
  set -- --steps 10 --render-every 2 --no-show
fi

logfile="outputs/run_$(date +%Y%m%d_%H%M%S).log"

# Allow Docker to use X (run once per session on host if needed: xhost +local:root)
docker run --rm -it \
  --env "DISPLAY=${DISPLAY:-}" \
  --env "QT_X11_NO_MITSHM=1" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v "$(pwd)/outputs:/ws/outputs" \
  "${IMAGE}" "$@" | tee "${logfile}"

