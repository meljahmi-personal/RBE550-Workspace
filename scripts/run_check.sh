#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
env -i bash --noprofile --norc -c "
  set -e
  export HOME=$HOME
  set +u; source /opt/ros/humble/setup.bash; set -u
  set +u; source \"$WS/install/setup.bash\"; set -u
  export AMENT_PREFIX_PATH=\"$WS/install:\${AMENT_PREFIX_PATH:-}\"
  export CMAKE_PREFIX_PATH=\"$WS/install:\${CMAKE_PREFIX_PATH:-}\"
  ros2 run rbe550_grid_bench bench --steps 10 --render-every 2 --no-show
  echo \"[OK] rbe550_grid_bench ran successfully.\"
"

