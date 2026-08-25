#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
POLICY_PATH="${REPO_ROOT}/vendor/unitree_rl_lab/policies/g1_29dof_velocity_v0/policy.onnx"
ARGS=("$@")

"${SCRIPT_DIR}/download_unitree_rl_lab_policy.sh"

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
append_default_arg "policy_path" "${POLICY_PATH}"
append_default_arg "policy_profile" "unitree_rl_lab_g1_29dof"
append_default_arg "policy_obs_dim" "480"
append_default_arg "policy_action_dim" "29"
append_default_arg "mujoco_pedestrians_enable" "false"
append_default_arg "mujoco_pedestrians_count" "0"
append_default_arg "run_rviz" "true"
append_default_arg "run_slam" "true"
append_default_arg "run_nav2" "true"
append_default_arg "run_waypoint_explorer" "true"
append_default_arg "run_waypoint_patrol" "false"
append_default_arg "run_frontier_explorer" "false"
append_default_arg "run_random_explorer" "false"
append_default_arg "run_human_aware_motion_planner" "false"
append_default_arg "run_human_collision_avoidance" "false"
append_default_arg "run_human_aware_run_logger" "false"
append_default_arg "run_lidar_person_detector" "false"
append_default_arg "run_dynamic_obstacle_tracker" "true"
append_default_arg "tracker_candidate_person_score_threshold" "999.0"
append_default_arg "tracker_confirmed_person_score_threshold" "999.0"
append_default_arg "tracker_confirm_confidence_threshold" "1.1"
append_default_arg "cmd_vel_topic" "/cmd_vel"
append_default_arg "cmd_vel_deadband" "0.03"
append_default_arg "allow_reverse_cmd_vel" "false"
append_default_arg "axis_exclusive_cmd_vel" "false"
append_default_arg "axis_exclusive_yaw_threshold" "0.03"
append_default_arg "turning_arc_min_forward_speed" "0.20"
append_default_arg "turning_arc_min_yaw_speed" "0.12"
append_default_arg "max_lin_vel_x" "0.35"
append_default_arg "max_lin_vel_y" "0.0"
append_default_arg "max_ang_vel_z" "0.20"
append_default_arg "cmd_accel_x" "0.22"
append_default_arg "cmd_accel_yaw" "0.35"
append_default_arg "livox_scan_points" "1200"
append_default_arg "mujoco_initial_keyframe" "unitree_rl_lab_stand"
append_default_arg "mujoco_reset_on_controller_start" "false"
append_default_arg "startup_current_pose_hold_time" "0.0"
append_default_arg "startup_blend_time" "0.0"
append_default_arg "startup_hold_time" "0.0"
append_default_arg "startup_require_upright" "true"
append_default_arg "startup_max_recovery_time" "0.0"
append_default_arg "policy_action_ramp_time" "0.0"
append_default_arg "policy_action_rate_limit" "0.0"
append_default_arg "publish_camera" "false"

printf 'Starting people-free Extreme Lab waypoint navigation demo.\n'
printf 'Policy: official Unitree RL Lab G1-29DoF velocity v0 (480 observations, 29 actions).\n'
printf 'Motion envelope: forward=0.35 m/s, turning floor=0.20 m/s, yaw=0.20 rad/s.\n'
printf 'Turn shaping: stable walking arcs; pure-yaw response is intentionally limited.\n'
printf 'RViz goals are generated continuously by g1_waypoint_explorer.\n'

mapfile -t stale_containers < <(
  docker ps --quiet --filter 'name=unitree_g1_ros2_mujoco-unitree-g1-sim-run-'
)
if [ "${#stale_containers[@]}" -gt 0 ]; then
  printf 'Stopping %d existing G1 simulation container(s) to avoid DDS conflicts.\n' \
    "${#stale_containers[@]}"
  docker stop "${stale_containers[@]}" >/dev/null
fi

exec "${SCRIPT_DIR}/bringup_full_sim.sh" "${ARGS[@]}"
