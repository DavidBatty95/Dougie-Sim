#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
POLICY_PATH="${REPO_ROOT}/vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop/2026-08-05_10-32-24_g1_navigation_finetune_from_model_5796/2026-08-05_10-32-24_g1_navigation_finetune_from_model_5796.onnx"
POLICY_PROFILE="mjlab"
INITIAL_KEYFRAME="policy_stand"
SCENE="scene_29dof_extreme_lab.xml"
LINEAR_X="0.10"
ANGULAR_Z="0.0"
DURATION="20"
STARTUP_SECONDS="12"
MAX_LINEAR_X="0.22"
MAX_ANGULAR_Z="0.30"
RESULT_DIR="${REPO_ROOT}/ros2_ws/src/dynamic_obstacle_benchmarks/results/fixed_command"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --policy) POLICY_PATH="$2"; shift 2 ;;
    --policy-profile) POLICY_PROFILE="$2"; shift 2 ;;
    --initial-keyframe) INITIAL_KEYFRAME="$2"; shift 2 ;;
    --scene) SCENE="$2"; shift 2 ;;
    --linear-x) LINEAR_X="$2"; shift 2 ;;
    --angular-z) ANGULAR_Z="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --startup-seconds) STARTUP_SECONDS="$2"; shift 2 ;;
    --max-linear-x) MAX_LINEAR_X="$2"; shift 2 ;;
    --max-angular-z) MAX_ANGULAR_Z="$2"; shift 2 ;;
    --result-dir) RESULT_DIR="$2"; shift 2 ;;
    -h|--help)
      printf 'Usage: %s [--scene FILE] [--policy FILE] [--policy-profile NAME] [--initial-keyframe NAME] [--linear-x MPS] [--angular-z RADPS] [--duration SEC] [--max-linear-x MPS] [--max-angular-z RADPS]\n' "$0"
      exit 0
      ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
done

mkdir -p "${RESULT_DIR}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BASENAME="fixed_command_${SCENE%.xml}_${STAMP}"
SIM_LOG="${RESULT_DIR}/${BASENAME}_sim.log"
HOST_JSON="${RESULT_DIR}/${BASENAME}.json"
CONTAINER_JSON="/workspace/${HOST_JSON#${REPO_ROOT}/}"
SIM_PID=""

cleanup_sim_containers() {
  docker ps -aq --filter "name=unitree_g1_ros2_mujoco-unitree-g1-sim-run-" | xargs -r docker rm -f >/dev/null 2>&1 || true
}
cleanup() {
  if [ -n "${SIM_PID}" ]; then
    kill -- -"${SIM_PID}" >/dev/null 2>&1 || kill "${SIM_PID}" >/dev/null 2>&1 || true
    wait "${SIM_PID}" >/dev/null 2>&1 || true
  fi
  cleanup_sim_containers
}
trap cleanup EXIT

cleanup_sim_containers
setsid "${SCRIPT_DIR}/bringup_full_sim.sh" \
  mujoco_scene:="${SCENE}" \
  policy_path:="${POLICY_PATH}" \
  policy_profile:="${POLICY_PROFILE}" \
  mujoco_pedestrians_enable:=false \
  mujoco_pedestrians_count:=0 \
  mujoco_initial_keyframe:="${INITIAL_KEYFRAME}" \
  mujoco_reset_on_controller_start:=false \
  startup_current_pose_hold_time:=0.0 \
  startup_blend_time:=0.0 \
  startup_hold_time:=0.0 \
  startup_max_recovery_time:=0.0 \
  policy_action_ramp_time:=0.0 \
  policy_action_rate_limit:=0.0 \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_deadband:=0.03 \
  allow_reverse_cmd_vel:=false \
  max_lin_vel_x:="${MAX_LINEAR_X}" \
  max_lin_vel_y:=0.0 \
  max_ang_vel_z:="${MAX_ANGULAR_Z}" \
  run_slam:=false \
  run_nav2:=false \
  run_slam_reset_monitor:=false \
  run_dynamic_obstacle_tracker:=false \
  run_lidar_person_detector:=false \
  run_human_aware_motion_planner:=false \
  run_human_collision_avoidance:=false \
  run_rviz:=false \
  publish_camera:=false \
  >"${SIM_LOG}" 2>&1 &
SIM_PID="$!"

sleep "${STARTUP_SECONDS}"
docker compose run --rm unitree-g1-sim bash -lc \
  'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; source /workspace/ros2_ws/install/setup.bash; exec python3 /workspace/scripts/measure_g1_fixed_command.py "$@"' \
  bash \
  --linear-x "${LINEAR_X}" \
  --angular-z "${ANGULAR_Z}" \
  --duration "${DURATION}" \
  --output "${CONTAINER_JSON}"

printf 'Result: %s\nSimulation log: %s\n' "${HOST_JSON}" "${SIM_LOG}"
