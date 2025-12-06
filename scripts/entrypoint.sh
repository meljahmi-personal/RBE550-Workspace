#!/usr/bin/env bash
# Do NOT use -u here; ROS setup scripts access unset vars legitimately.
set -e  # keep -e for fail-fast

# Source ROS 2 (prefer env ROS_DISTRO if present)
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  source /opt/ros/humble/setup.bash
fi

# Source the workspace overlay if present
if [[ -f /ws/install/setup.bash ]]; then
  set +e
  source /ws/install/setup.bash
  set -e
fi

# NEW: force software OpenGL (makes RViz robust on any GPU/driver)
export LIBGL_ALWAYS_SOFTWARE=1

# Ensure container-side outputs dir exists (for the bind mount and any artifacts)
mkdir -p /ws/outputs

# If no command is given OR first arg is a flag, default to running the bench
if [[ $# -eq 0 || "${1:0:1}" == "-" ]]; then
  exec ros2 run rbe550_grid_bench bench "$@"
else
  exec "$@"
fi

