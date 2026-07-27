#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

cd "${WORKSPACE}"

colcon build \
  --merge-install \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "ROS 2 workspace built at ${WORKSPACE}/install"
