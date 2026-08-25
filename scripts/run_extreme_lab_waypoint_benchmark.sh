#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SEED="42"
GOALS="5"
START_SIM="1"
SIM_STARTUP_SECONDS="28"
RUN_RVIZ="1"
MINIMUM_SCORE="80"
RESULT_DIR="${REPO_ROOT}/ros2_ws/src/dynamic_obstacle_benchmarks/results/extreme_lab_locomotion"
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
  --no-rviz                Do not launch RViz
  --startup-seconds N      Simulation startup wait (default: 28)
  --minimum-score N        Validation threshold (default: 80)
  --sim-arg ARG            Additional g1_sim.launch.py argument
  --result-dir PATH        Host artifact directory
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --goals) GOALS="$2"; shift 2 ;;
    --no-sim) START_SIM="0"; shift ;;
    --no-rviz) RUN_RVIZ="0"; shift ;;
    --startup-seconds) SIM_STARTUP_SECONDS="$2"; shift 2 ;;
    --minimum-score) MINIMUM_SCORE="$2"; shift 2 ;;
    --sim-arg) SIM_ARGS+=("$2"); shift 2 ;;
    --result-dir) RESULT_DIR="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

mkdir -p "${RESULT_DIR}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BASENAME="extreme_lab_locomotion_seed_${SEED}_${STAMP}"
HOST_JSON="${RESULT_DIR}/${BASENAME}.json"
HOST_TXT="${RESULT_DIR}/${BASENAME}.txt"
if [[ "${RESULT_DIR}" == "${REPO_ROOT}"/* ]]; then
  CONTAINER_RESULT_DIR="/workspace/${RESULT_DIR#${REPO_ROOT}/}"
else
  CONTAINER_RESULT_DIR="/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/extreme_lab_locomotion"
fi
CONTAINER_JSON="${CONTAINER_RESULT_DIR}/${BASENAME}.json"
CONTAINER_CONFIG="/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/extreme_lab_locomotion_benchmark.yaml"

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
  printf 'Building benchmark and bringup packages...\n'
  docker compose run --rm unitree-g1-sim bash -lc \
    'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; cd /workspace/ros2_ws && colcon build --merge-install --symlink-install --packages-select dynamic_obstacle_benchmarks unitree_g1_mujoco_bridge unitree_g1_sim_bringup'
  printf 'Starting people-free Extreme Lab simulation...\n'
  setsid "${SCRIPT_DIR}/bringup_full_sim.sh" \
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
    >"${RESULT_DIR}/${BASENAME}_sim.log" 2>&1 &
  SIM_PID="$!"
  printf 'Waiting %s seconds for Nav2 and SLAM startup...\n' "${SIM_STARTUP_SECONDS}"
  sleep "${SIM_STARTUP_SECONDS}"
fi

{
  printf 'Extreme Lab Random Waypoint Locomotion Benchmark\n'
  printf '================================================\n'
  printf 'Seed: %s\n' "${SEED}"
  printf 'Goals: %s\n' "${GOALS}"
  printf 'People: 0\n'
  printf 'Velocity limits: x=0.15 m/s, y=0.00 m/s, yaw=0.20 rad/s\n\n'
} >"${HOST_TXT}"

docker compose run --rm unitree-g1-sim bash -lc \
  'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; source /workspace/ros2_ws/install/setup.bash; exec ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark "$@"' \
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
printf '  Simulation: %s\n' "${RESULT_DIR}/${BASENAME}_sim.log" | tee -a "${HOST_TXT}"
exit "${VALIDATION_STATUS}"
