#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -f "/workspace/ros2_ws/install/setup.bash" ]; then
  source "/workspace/ros2_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///workspace/ros2_ws/config/cyclonedds.xml}"

exec "$@"
