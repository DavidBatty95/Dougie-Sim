#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONTAINER_BUILD_BASE="/workspace/ros2_ws/.verification/build"
CONTAINER_INSTALL_BASE="/workspace/ros2_ws/.verification/install"
CONTAINER_LOG_BASE="/workspace/ros2_ws/.verification/log"
POLICY_PATH="${REPO_ROOT}/vendor/unitree_rl_lab/policies/g1_29dof_velocity_v0/policy.onnx"
POLICY_PROFILE="unitree_rl_lab_g1_29dof"
INITIAL_KEYFRAME="unitree_rl_lab_stand"
SCENE="scene_29dof_empty.xml"
LINEAR_X="0.0"
COMMAND_SET="0"
LINEAR_Y="0.0"
ANGULAR_Z="0.0"
DURATION="8"
STOP_DURATION="8"
STARTUP_SECONDS="12"
SKIP_BUILD="0"
MAX_LINEAR_X="0.35"
MAX_LINEAR_Y="0.12"
MAX_ANGULAR_Z="0.30"
CMD_ACCEL_X="0.22"
CMD_ACCEL_Y="0.16"
CMD_ACCEL_YAW="0.35"
RUN_RVIZ="1"
VALIDATE_RESULT="0"
RESULT_DIR="${REPO_ROOT}/ros2_ws/src/dynamic_obstacle_benchmarks/results/fixed_command"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --policy) POLICY_PATH="$2"; shift 2 ;;
    --policy-profile) POLICY_PROFILE="$2"; shift 2 ;;
    --initial-keyframe) INITIAL_KEYFRAME="$2"; shift 2 ;;
    --scene) SCENE="$2"; shift 2 ;;
    --linear-x) LINEAR_X="$2"; COMMAND_SET="1"; shift 2 ;;
    --linear-y) LINEAR_Y="$2"; COMMAND_SET="1"; shift 2 ;;
    --angular-z) ANGULAR_Z="$2"; COMMAND_SET="1"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --stop-duration) STOP_DURATION="$2"; shift 2 ;;
    --startup-seconds) STARTUP_SECONDS="$2"; shift 2 ;;
    --skip-build) SKIP_BUILD="1"; shift ;;
    --max-linear-x) MAX_LINEAR_X="$2"; shift 2 ;;
    --max-linear-y) MAX_LINEAR_Y="$2"; shift 2 ;;
    --max-angular-z) MAX_ANGULAR_Z="$2"; shift 2 ;;
    --cmd-accel-x) CMD_ACCEL_X="$2"; shift 2 ;;
    --cmd-accel-y) CMD_ACCEL_Y="$2"; shift 2 ;;
    --cmd-accel-yaw) CMD_ACCEL_YAW="$2"; shift 2 ;;
    --rviz) RUN_RVIZ="1"; shift ;;
    --no-rviz) RUN_RVIZ="0"; shift ;;
    --validate) VALIDATE_RESULT="1"; shift ;;
    --result-dir) RESULT_DIR="$2"; shift 2 ;;
    -h|--help)
      printf 'Usage: %s [--scene FILE] [--policy FILE] [--policy-profile NAME] [--initial-keyframe NAME] [--linear-x MPS] [--linear-y MPS] [--angular-z RADPS] [--duration SEC] [--stop-duration SEC] [--startup-seconds SEC] [--skip-build] [--max-linear-x MPS] [--max-linear-y MPS] [--max-angular-z RADPS] [--cmd-accel-x MPS2] [--cmd-accel-y MPS2] [--cmd-accel-yaw RADPS2] [--no-rviz] [--validate]\n' "$0"
      exit 0
      ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
done

if [ "${COMMAND_SET}" = "0" ]; then LINEAR_X="0.35"; fi
if [[ "${POLICY_PATH}" != /* ]]; then POLICY_PATH="$(realpath -e "${POLICY_PATH}")"; fi
RESULT_DIR="$(realpath -m "${RESULT_DIR}")"

mkdir -p "${RESULT_DIR}"
if [ "${SKIP_BUILD}" = "1" ]; then
  if [ ! -f "${REPO_ROOT}/ros2_ws/.verification/install/setup.bash" ]; then
    printf 'Fast start requires an existing verification build; run once without --skip-build.\n' >&2
    exit 2
  fi
  printf 'Reusing existing ROS 2 workspace build...\n'
else
  docker compose run --rm \
    -e COLCON_BUILD_BASE="${CONTAINER_BUILD_BASE}" \
    -e COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" \
    -e COLCON_LOG_BASE="${CONTAINER_LOG_BASE}" \
    unitree-g1-sim /workspace/scripts/build_ws.sh --packages-select \
      g1_msgs \
      g1_locomotion \
      unitree_g1_description \
      unitree_social_navigation_msgs \
      unitree_social_costmap \
      unitree_g1_mujoco_bridge \
      unitree_g1_sim_bringup
fi
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BASENAME="fixed_command_${SCENE%.xml}_${STAMP}"
SIM_LOG="${RESULT_DIR}/${BASENAME}_sim.log"
HOST_JSON="${RESULT_DIR}/${BASENAME}.json"
HOST_SAMPLES="${RESULT_DIR}/${BASENAME}_samples.csv"
CONTAINER_JSON="/workspace/${HOST_JSON#${REPO_ROOT}/}"
CONTAINER_SAMPLES="/workspace/${HOST_SAMPLES#${REPO_ROOT}/}"
if [[ "${POLICY_PATH}" == "${REPO_ROOT}/"* ]]; then
  CONTAINER_POLICY_PATH="/workspace/${POLICY_PATH#${REPO_ROOT}/}"
else
  CONTAINER_POLICY_PATH="${POLICY_PATH}"
fi
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
COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" setsid "${SCRIPT_DIR}/bringup_full_sim.sh" \
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
  max_lin_vel_y:="${MAX_LINEAR_Y}" \
  max_ang_vel_z:="${MAX_ANGULAR_Z}" \
  cmd_accel_x:="${CMD_ACCEL_X}" \
  cmd_accel_y:="${CMD_ACCEL_Y}" \
  cmd_accel_yaw:="${CMD_ACCEL_YAW}" \
  run_slam:=false \
  run_nav2:=false \
  run_slam_reset_monitor:=false \
  run_dynamic_obstacle_tracker:=false \
  run_lidar_person_detector:=false \
  run_human_aware_motion_planner:=false \
  run_human_collision_avoidance:=false \
  run_rviz:="${RUN_RVIZ}" \
  publish_camera:=false \
  >"${SIM_LOG}" 2>&1 &
SIM_PID="$!"

printf 'Waiting up to %s seconds for robot state and odometry...\n' "${STARTUP_SECONDS}"
if ! docker compose run --rm \
  -e COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" \
  -e READY_TIMEOUT="${STARTUP_SECONDS}" \
  unitree-g1-sim bash -lc '
    source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
    source "${COLCON_INSTALL_BASE}/setup.bash"
    timeout "${READY_TIMEOUT}" bash -c '\''
      wait_topic() {
        topic="$1"
        until ros2 topic type "${topic}" >/dev/null 2>&1; do sleep 0.2; done
        ros2 topic echo --once "${topic}" >/dev/null
      }
      wait_topic /lowstate & lowstate_pid=$!
      wait_topic /odom & odom_pid=$!
      wait "${lowstate_pid}" "${odom_pid}"
    '\''
  '; then
  printf 'Simulation did not become ready within %s seconds. See %s.\n' "${STARTUP_SECONDS}" "${SIM_LOG}" >&2
  exit 2
fi
printf 'Simulation ready; starting command measurement.\n'
docker compose run --rm \
  -e COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" \
  unitree-g1-sim bash -lc \
  'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; source "${COLCON_INSTALL_BASE}/setup.bash"; exec python3 /workspace/scripts/measure_g1_fixed_command.py "$@"' \
  bash \
  --linear-x "${LINEAR_X}" \
  --linear-y "${LINEAR_Y}" \
  --angular-z "${ANGULAR_Z}" \
  --duration "${DURATION}" \
  --stop-duration "${STOP_DURATION}" \
  --settle 3.0 \
  --policy-path "${CONTAINER_POLICY_PATH}" \
  --policy-profile "${POLICY_PROFILE}" \
  --scene "${SCENE}" \
  --max-linear-x "${MAX_LINEAR_X}" \
  --max-linear-y "${MAX_LINEAR_Y}" \
  --max-angular-z "${MAX_ANGULAR_Z}" \
  --cmd-accel-x "${CMD_ACCEL_X}" \
  --cmd-accel-y "${CMD_ACCEL_Y}" \
  --cmd-accel-yaw "${CMD_ACCEL_YAW}" \
  --trial-id "${BASENAME}" \
  --samples-output "${CONTAINER_SAMPLES}" \
  --output "${CONTAINER_JSON}"

if [ "${VALIDATE_RESULT}" = "1" ]; then
  python3 "${SCRIPT_DIR}/validate_fixed_command_result.py" "${HOST_JSON}"
fi

printf 'Result: %s\nRaw samples: %s\nSimulation log: %s\n' \
  "${HOST_JSON}" "${HOST_SAMPLES}" "${SIM_LOG}"
