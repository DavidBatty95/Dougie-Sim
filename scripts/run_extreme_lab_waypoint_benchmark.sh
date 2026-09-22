#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONTAINER_BUILD_BASE="/workspace/ros2_ws/.verification/build"
CONTAINER_INSTALL_BASE="/workspace/ros2_ws/.verification/install"
CONTAINER_LOG_BASE="/workspace/ros2_ws/.verification/log"

SEED="42"
GOALS="5"
START_SIM="1"
SKIP_BUILD="0"
SIM_STARTUP_SECONDS="28"
RUN_RVIZ="1"
MINIMUM_SCORE="80"
RESULT_DIR="${REPO_ROOT}/ros2_ws/src/dynamic_obstacle_benchmarks/results/extreme_lab_locomotion"
RESULT_DIR_SET="0"
OUTPUT=""
CONTAINER_CONFIG="/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/extreme_lab_locomotion_benchmark.yaml"
SIM_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_extreme_lab_waypoint_benchmark.sh [options]

Runs a people-free Extreme Lab locomotion baseline. After each reached goal, a
new seeded random goal is sampled from the robot's current pose and accepted
only when Nav2 computes a path to it.

Options:
  --seed N                 Random seed (default: 42)
  --goals N                Number of sequential goals (default: 5)
  --no-sim                 Use an already-running simulation
  --skip-build             Reuse the existing verification workspace build
  --no-rviz                Do not launch RViz
  --startup-seconds N      Maximum simulation readiness wait (default: 28)
  --minimum-score N        Validation threshold (default: 80)
  --config PATH            Benchmark YAML path visible inside the container
  --sim-arg ARG            Additional g1_sim.launch.py argument
  --result-dir PATH        Host artifact directory
  --output PATH            Exact JSON path under ros2_ws; exclusive with --result-dir
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --goals) GOALS="$2"; shift 2 ;;
    --no-sim) START_SIM="0"; shift ;;
    --skip-build) SKIP_BUILD="1"; shift ;;
    --no-rviz) RUN_RVIZ="0"; shift ;;
    --startup-seconds) SIM_STARTUP_SECONDS="$2"; shift 2 ;;
    --minimum-score) MINIMUM_SCORE="$2"; shift 2 ;;
    --config) CONTAINER_CONFIG="$2"; shift 2 ;;
    --sim-arg) SIM_ARGS+=("$2"); shift 2 ;;
    --result-dir)
      [ "$#" -ge 2 ] || { printf '%s\n' '--result-dir requires PATH' >&2; exit 2; }
      RESULT_DIR="$2"; RESULT_DIR_SET="1"; shift 2
      ;;
    --output)
      [ "$#" -ge 2 ] || { printf '%s\n' '--output requires PATH' >&2; exit 2; }
      OUTPUT="$2"; shift 2
      ;;
    --help|-h) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
DEFAULT_BASENAME="extreme_lab_locomotion_seed_${SEED}_${STAMP}"
source "${SCRIPT_DIR}/benchmark_output_paths.sh"
resolve_benchmark_output_paths \
  "${REPO_ROOT}" "${OUTPUT}" "${RESULT_DIR}" "${RESULT_DIR_SET}" "${DEFAULT_BASENAME}"
mkdir -p "${RESULT_DIR}"
HOST_TXT="${HOST_STEM}.txt"
HOST_SIM_LOG="${HOST_STEM}_sim.log"

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

if [ "${START_SIM}" = "1" ]; then
  cleanup_sim_containers
  if [ "${SKIP_BUILD}" = "1" ]; then
    if [ ! -f "${REPO_ROOT}/ros2_ws/.verification/install/setup.bash" ]; then
      printf 'Fast start requires an existing verification build; run once without --skip-build.\n' >&2
      exit 2
    fi
    printf 'Reusing existing ROS 2 workspace build...\n'
  else
    printf 'Building benchmark and bringup packages...\n'
    docker compose run --rm \
      -e COLCON_BUILD_BASE="${CONTAINER_BUILD_BASE}" \
      -e COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" \
      -e COLCON_LOG_BASE="${CONTAINER_LOG_BASE}" \
      unitree-g1-sim /workspace/scripts/build_ws.sh --packages-select \
        unitree_social_navigation_msgs \
        unitree_social_costmap \
        dynamic_obstacle_benchmarks \
        unitree_g1_mujoco_bridge \
        unitree_g1_sim_bringup
  fi
  printf 'Starting people-free Extreme Lab simulation...\n'
  COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" setsid "${SCRIPT_DIR}/bringup_full_sim.sh" \
    mujoco_scene:=scene_29dof_extreme_lab.xml \
    mujoco_pedestrians_enable:=false \
    mujoco_pedestrians_count:=0 \
    mujoco_initial_keyframe:=policy_stand \
    startup_current_pose_hold_time:=0.0 \
    startup_blend_time:=0.0 \
    startup_hold_time:=0.0 \
    startup_require_upright:=true \
    startup_max_recovery_time:=0.0 \
    policy_action_ramp_time:=0.0 \
    policy_action_rate_limit:=0.0 \
    mujoco_reset_on_controller_start:=false \
    cmd_vel_topic:=/cmd_vel \
    cmd_vel_deadband:=0.03 \
    axis_exclusive_cmd_vel:=false \
    axis_exclusive_yaw_threshold:=0.03 \
    turning_arc_min_forward_speed:=0.06 \
    turning_arc_min_yaw_speed:=0.12 \
    allow_reverse_cmd_vel:=false \
    max_lin_vel_x:=0.15 \
    max_lin_vel_y:=0.0 \
    max_ang_vel_z:=0.20 \
    livox_scan_points:=1200 \
    run_nav2:=true \
    run_waypoint_explorer:=false \
    run_waypoint_patrol:=false \
    run_frontier_explorer:=false \
    run_random_explorer:=false \
    run_human_aware_motion_planner:=false \
    run_human_collision_avoidance:=false \
    run_dynamic_obstacle_tracker:=true \
    run_lidar_person_detector:=false \
    run_human_aware_run_logger:=false \
    run_rviz:="${RUN_RVIZ}" \
    publish_camera:=false \
    "${SIM_ARGS[@]}" \
    >"${HOST_SIM_LOG}" 2>&1 &
  SIM_PID="$!"
  printf 'Waiting up to %s seconds for robot state, scan, map, and active Nav2...\n' "${SIM_STARTUP_SECONDS}"
  if ! docker compose run --rm \
    -e COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" \
    -e READY_TIMEOUT="${SIM_STARTUP_SECONDS}" \
    unitree-g1-sim bash -lc '
      source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
      source "${COLCON_INSTALL_BASE}/setup.bash"
      timeout "${READY_TIMEOUT}" bash -c '\''
        wait_topic() {
          topic="$1"
          until ros2 topic type "${topic}" >/dev/null 2>&1; do sleep 0.2; done
          ros2 topic echo --once "${topic}" >/dev/null
        }
        wait_nav2() {
          until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active; do sleep 0.2; done
        }
        wait_topic /lowstate & lowstate_pid=$!
        wait_topic /scan & scan_pid=$!
        wait_topic /map & map_pid=$!
        wait_nav2 & nav2_pid=$!
        wait "${lowstate_pid}" "${scan_pid}" "${map_pid}" "${nav2_pid}"
      '\''
    '; then
    printf 'Simulation did not become ready within %s seconds. See %s.\n' \
      "${SIM_STARTUP_SECONDS}" "${HOST_SIM_LOG}" >&2
    exit 2
  fi
  printf 'Simulation ready; starting waypoint benchmark.\n'
fi

{
  printf 'Extreme Lab Random Waypoint Locomotion Benchmark\n'
  printf '================================================\n'
  printf 'Seed: %s\n' "${SEED}"
  printf 'Goals: %s\n' "${GOALS}"
  printf 'People: 0\n'
  printf 'Velocity limits: x=0.15 m/s, y=0.00 m/s, yaw=0.20 rad/s\n\n'
} >"${HOST_TXT}"

docker compose run --rm \
  -e COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" \
  unitree-g1-sim bash -lc \
  'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; source "${COLCON_INSTALL_BASE}/setup.bash"; exec ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark "$@"' \
  bash \
  --config "${CONTAINER_CONFIG}" \
  --backend mujoco \
  --seed "${SEED}" \
  --people 0 \
  --goals "${GOALS}" \
  --headless \
  --output "${CONTAINER_JSON}" | tee -a "${HOST_TXT}"

set +e
python3 "${SCRIPT_DIR}/validate_benchmark_result.py" \
  "${HOST_JSON}" \
  --backend mujoco \
  --people-free \
  --minimum-score "${MINIMUM_SCORE}" | tee -a "${HOST_TXT}"
VALIDATION_STATUS="${PIPESTATUS[0]}"
set -e

printf '\nArtifacts:\n' | tee -a "${HOST_TXT}"
printf '  Report: %s\n' "${HOST_TXT}" | tee -a "${HOST_TXT}"
printf '  Result: %s\n' "${HOST_JSON}" | tee -a "${HOST_TXT}"
printf '  Goals: %s\n' "${HOST_JSON%.json}_goals.csv" | tee -a "${HOST_TXT}"
printf '  Scenario: %s\n' "${HOST_JSON%.json}_scenario.yaml" | tee -a "${HOST_TXT}"
printf '  Simulation: %s\n' "${HOST_SIM_LOG}" | tee -a "${HOST_TXT}"
exit "${VALIDATION_STATUS}"
