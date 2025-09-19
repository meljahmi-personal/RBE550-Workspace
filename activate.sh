# Usage: source ~/ros2projrbe550_ws/activate.sh
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX ROS_PACKAGE_PATH PYTHONPATH

set +u
source /opt/ros/humble/setup.bash
source "$HOME/ros2projrbe550_ws/install/setup.bash"
set -u

# Ensure your workspace is first on the overlay
export AMENT_PREFIX_PATH="$HOME/ros2projrbe550_ws/install:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$HOME/ros2projrbe550_ws/install:${CMAKE_PREFIX_PATH:-}"

echo "[RBE-550 active]"
tr ':' '\n' <<< "$AMENT_PREFIX_PATH" | nl | sed -n '1,4p'
