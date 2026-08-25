#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"
BUILD_BASE="${COLCON_BUILD_BASE:-${WORKSPACE}/build}"
INSTALL_BASE="${COLCON_INSTALL_BASE:-${WORKSPACE}/install}"
LOG_BASE="${COLCON_LOG_BASE:-${WORKSPACE}/log}"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

cd "${WORKSPACE}"

colcon --log-base "${LOG_BASE}" build \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --merge-install \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "ROS 2 workspace built at ${INSTALL_BASE}"
