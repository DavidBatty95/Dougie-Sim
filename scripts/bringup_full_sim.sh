#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export UNITREE_MUJOCO_DOMAIN_ID="${UNITREE_MUJOCO_DOMAIN_ID:-${ROS_DOMAIN_ID}}"

printf 'Starting Docker G1 simulation on ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
printf 'Starting Unitree Mujoco DDS on domain_id=%s\n' "${UNITREE_MUJOCO_DOMAIN_ID}"

ARGS=()
for arg in "$@"; do
  case "${arg}" in
    policy_path:="${REPO_ROOT}"/*)
      ARGS+=("policy_path:=/workspace/${arg#policy_path:=${REPO_ROOT}/}")
      ;;
    *)
      ARGS+=("${arg}")
      ;;
  esac
done

docker compose run --rm unitree-g1-sim bash -lc 'exec /workspace/scripts/bringup_g1_ros2.sh "$@"' bash "${ARGS[@]}"
