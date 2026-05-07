#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

cd "${WORKSPACE}"

colcon build \
  --merge-install \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "ROS 2 workspace built at ${WORKSPACE}/install"
