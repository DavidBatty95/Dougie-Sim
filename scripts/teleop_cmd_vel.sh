#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

cd "${REPO_ROOT}"
docker compose run --rm unitree-g1-sim bash -lc \
  'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && source /workspace/ros2_ws/install/setup.bash && exec ros2 run unitree_g1_mujoco_bridge g1_cmd_vel_teleop "$@"' \
  bash "$@"
