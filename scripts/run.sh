#!/usr/bin/env bash
# Source env and run the bench. Pass any args through.
# Examples:
#   ./scripts/run.sh --steps 200 --render-every 4 --no-show
#   ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --no-show

set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"

# Source ROS + this workspace
set +u
source /opt/ros/humble/setup.bash
# Prefer merged layout setup if present; fallback to isolated
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
else
  source "$WS/install/local_setup.bash"
fi
set -u

# Ensure this workspace is first on the overlay (helps some shells)
export AMENT_PREFIX_PATH="$WS/install:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$WS/install:${CMAKE_PREFIX_PATH:-}"

# Run the package executable, forwarding all args
exec ros2 run rbe550_grid_bench bench "$@"

