#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
HOST_RUN_LOG="${REPO_ROOT}/ros2_ws/src/dynamic_obstacle_benchmarks/results/demos/collision_avoidance_${STAMP}.jsonl"
CONTAINER_RUN_LOG="/workspace/${HOST_RUN_LOG#${REPO_ROOT}/}"

ARGS=("$@")
mkdir -p "$(dirname "${HOST_RUN_LOG}")"

append_default_arg() {
  local name="$1"
  local value="$2"
  local arg
  for arg in "${ARGS[@]}"; do
    if [[ "${arg}" == "${name}:="* ]]; then
      return
    fi
  done
  ARGS+=("${name}:=${value}")
}

append_default_arg "mujoco_scene" "scene_29dof_extreme_lab.xml"
append_default_arg "mujoco_pedestrians_enable" "true"
append_default_arg "mujoco_pedestrians_count" "6"
append_default_arg "run_rviz" "true"
append_default_arg "run_slam" "true"
append_default_arg "run_nav2" "true"
append_default_arg "run_waypoint_explorer" "true"
append_default_arg "run_frontier_explorer" "false"
append_default_arg "run_random_explorer" "false"
append_default_arg "run_lidar_person_detector" "false"
append_default_arg "run_dynamic_obstacle_tracker" "true"
append_default_arg "run_human_aware_motion_planner" "true"
append_default_arg "run_human_collision_avoidance" "true"
append_default_arg "run_human_aware_run_logger" "true"
append_default_arg "human_aware_run_log_path" "${CONTAINER_RUN_LOG}"
append_default_arg "max_lin_vel_x" "0.05"
append_default_arg "max_lin_vel_y" "0.0"
append_default_arg "max_ang_vel_z" "0.10"
append_default_arg "social_navigation_params_file" \
  "/workspace/ros2_ws/src/unitree_g1_sim_bringup/config/social_navigation_conservative_v1.yaml"

printf 'Starting conservative G1 collision-avoidance demo.\n'
printf 'Safety output: /cmd_vel_safe | Diagnostics: /human_collision_avoidance_debug\n'
printf 'Persistent run log: %s\n' "${HOST_RUN_LOG}"

mapfile -t stale_containers < <(
  docker ps --quiet --filter 'name=unitree_g1_ros2_mujoco-unitree-g1-sim-run-'
)
if [ "${#stale_containers[@]}" -gt 0 ]; then
  printf 'Stopping %d existing G1 simulation container(s) to avoid DDS conflicts.\n' \
    "${#stale_containers[@]}"
  docker stop "${stale_containers[@]}" >/dev/null
fi

exec "${SCRIPT_DIR}/bringup_full_sim.sh" "${ARGS[@]}"
