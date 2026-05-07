#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi

ros2 launch unitree_g1_sim_bringup g1_sim.launch.py "$@"
