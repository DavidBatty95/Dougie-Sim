#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONTAINER_BUILD_BASE="/workspace/ros2_ws/.verification/build"
CONTAINER_INSTALL_BASE="/workspace/ros2_ws/.verification/install"
CONTAINER_LOG_BASE="/workspace/ros2_ws/.verification/log"

SEED=""
PEOPLE="8"
GOALS="10"
MODE="standard"
BACKEND="mujoco"
START_SIM="1"
SKIP_BUILD="0"
SIM_STARTUP_SECONDS="28"
RUN_RVIZ="1"
RECORD_ROSBAG="0"
REQUIRE_AVOIDANCE="0"
VISUAL_SESSION="0"
MINIMUM_SCORE="70"
SCENE="scene_29dof_extreme_lab.xml"
ENVIRONMENT_NAME="extreme_lab"
RESULT_DIR="${REPO_ROOT}/ros2_ws/src/dynamic_obstacle_benchmarks/results/extreme_lab"
RESULT_DIR_SET="0"
OUTPUT=""
CONFIG="/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/person_avoidance_benchmark.yaml"
EXTRA_ARGS=()
SIM_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_extreme_lab_person_avoidance_benchmark.sh [options]

Options:
  --seed N                 Replay a fixed seed (default: random per run)
  --people N               Number of people (default: 8)
  --goals N                Number of goals (default: 10)
  --mode MODE              smoke|standard|stress|crossing|bottleneck|occlusion (default: standard)
  --backend BACKEND        mock|mujoco (default: mujoco)
  --scene FILE             MuJoCo scene (default: scene_29dof_extreme_lab.xml)
  --environment-name NAME  Report/artifact label (default: extreme_lab)
  --no-sim                 Do not launch the simulation first
  --skip-build             Reuse the existing verification workspace build
  --no-rviz                Do not launch RViz with the simulation
  --record-rosbag          Record diagnostic ROS topics during the benchmark
  --startup-seconds N      Maximum simulation readiness wait (default: 28)
  --minimum-score N        Validation threshold (default: 70)
  --require-avoidance      Require prolonged moving-person yield and resume evidence
  --visual-session         Long unscored session; keep running after logged contacts
  --config PATH            Benchmark YAML path inside the container
  --sim-arg ARG            Extra ROS launch argument passed to g1_sim.launch.py, e.g. avoidance_danger_radius:=2.4
  --result-dir PATH        Host result directory (default: package results/extreme_lab)
  --output PATH            Exact JSON path under ros2_ws; exclusive with --result-dir

The script writes a text score report, JSON result, scenario YAML, and CSV details.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --people) PEOPLE="$2"; shift 2 ;;
    --goals) GOALS="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;
    --scene) SCENE="$2"; shift 2 ;;
    --environment-name) ENVIRONMENT_NAME="$2"; shift 2 ;;
    --no-sim) START_SIM="0"; shift ;;
    --skip-build) SKIP_BUILD="1"; shift ;;
    --no-rviz) RUN_RVIZ="0"; shift ;;
    --record-rosbag) RECORD_ROSBAG="1"; EXTRA_ARGS+=("--record-rosbag"); shift ;;
    --startup-seconds) SIM_STARTUP_SECONDS="$2"; shift 2 ;;
    --minimum-score) MINIMUM_SCORE="$2"; shift 2 ;;
    --require-avoidance) REQUIRE_AVOIDANCE="1"; shift ;;
    --visual-session) VISUAL_SESSION="1"; EXTRA_ARGS+=("--visual-session"); shift ;;
    --config) CONFIG="$2"; shift 2 ;;
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
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

if [ -z "${SEED}" ]; then
  SEED="$(( $(od -An -N4 -tu4 /dev/urandom) & 2147483647 ))"
fi

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SAFE_ENVIRONMENT_NAME="${ENVIRONMENT_NAME//[^a-zA-Z0-9_-]/_}"
DEFAULT_BASENAME="person_avoidance_${SAFE_ENVIRONMENT_NAME}_${MODE}_seed_${SEED}_${STAMP}"
source "${SCRIPT_DIR}/benchmark_output_paths.sh"
resolve_benchmark_output_paths \
  "${REPO_ROOT}" "${OUTPUT}" "${RESULT_DIR}" "${RESULT_DIR_SET}" "${DEFAULT_BASENAME}"
mkdir -p "${RESULT_DIR}"
HOST_TXT="${HOST_STEM}.txt"
HOST_RUN_LOG="${HOST_STEM}_human_aware.jsonl"
HOST_ROSBAG="${HOST_STEM}_rosbag"
HOST_ROSBAG_LOG="${HOST_STEM}_rosbag.log"
HOST_SIM_LOG="${HOST_STEM}_sim.log"
CONTAINER_RUN_LOG="${CONTAINER_STEM}_human_aware.jsonl"
CONTAINER_ROSBAG="${CONTAINER_STEM}_rosbag"

SIM_PID=""
BAG_PID=""
cleanup_sim_containers() {
  docker ps -aq --filter "name=unitree_g1_ros2_mujoco-unitree-g1-sim-run-" | xargs -r docker rm -f >/dev/null 2>&1 || true
}

cleanup() {
  if [ -n "${BAG_PID}" ]; then
    kill -- -"${BAG_PID}" >/dev/null 2>&1 || kill "${BAG_PID}" >/dev/null 2>&1 || true
    wait "${BAG_PID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${SIM_PID}" ]; then
    kill -- -"${SIM_PID}" >/dev/null 2>&1 || kill "${SIM_PID}" >/dev/null 2>&1 || true
    wait "${SIM_PID}" >/dev/null 2>&1 || true
  fi
  cleanup_sim_containers
}
trap cleanup EXIT

if [ "${START_SIM}" = "1" ]; then
  printf 'Stopping stale Unitree G1 simulation containers before benchmark...\n'
  cleanup_sim_containers
  if [ "${SKIP_BUILD}" = "1" ]; then
    if [ ! -f "${REPO_ROOT}/ros2_ws/.verification/install/setup.bash" ]; then
      printf 'Fast start requires an existing verification build; run once without --skip-build.\n' >&2
      exit 2
    fi
    printf 'Reusing existing ROS 2 and MuJoCo builds...\n'
  else
    printf 'Refreshing person-avoidance packages...\n'
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
    printf 'Preparing Unitree MuJoCo simulator...\n'
    docker compose run --rm unitree-g1-sim /workspace/scripts/build_unitree_mujoco.sh
  fi
  printf 'Starting %s simulation for benchmark...\n' "${ENVIRONMENT_NAME}"
  COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" setsid "${SCRIPT_DIR}/bringup_full_sim.sh" \
    mujoco_scene:="${SCENE}" \
    mujoco_pedestrians_enable:=true \
    mujoco_pedestrians_count:="${PEOPLE}" \
    mujoco_pedestrian_seed:="${SEED}" \
    livox_scan_points:=600 \
    mujoco_initial_keyframe:=policy_stand \
    startup_current_pose_hold_time:=0.0 \
    startup_blend_time:=0.0 \
    startup_hold_time:=0.0 \
    startup_require_upright:=true \
    startup_max_recovery_time:=0.0 \
    policy_action_ramp_time:=0.0 \
    policy_action_rate_limit:=0.0 \
    mujoco_reset_on_controller_start:=false \
    max_lin_vel_x:=0.35 \
    max_lin_vel_y:=0.12 \
    max_ang_vel_z:=0.35 \
    lock_waist:=false \
    balance_waist:=false \
    run_nav2:=true \
    run_waypoint_explorer:=false \
    run_waypoint_patrol:=false \
    run_human_aware_motion_planner:=true \
    run_human_collision_avoidance:=true \
    run_human_aware_run_logger:=true \
    human_aware_run_log_path:="${CONTAINER_RUN_LOG}" \
    run_dynamic_obstacle_tracker:=true \
    run_lidar_person_detector:=false \
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
        set -e
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
  printf 'Simulation ready: robot state, LiDAR, SLAM map, and Nav2 are available.\n'
  if [ "${RECORD_ROSBAG}" = "1" ]; then
    printf 'Starting diagnostic rosbag recording...\n'
    setsid docker compose run --rm -e COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" unitree-g1-sim bash -lc \
      'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; source "${COLCON_INSTALL_BASE}/setup.bash"; mkdir -p "$(dirname "$1")"; exec ros2 bag record -o "$1" /cmd_vel /cmd_vel_human_aware /cmd_vel_safe /human_avoidance_state /human_avoidance_debug /human_collision_avoidance_debug /dynamic_obstacle_tracker/debug_tracks /tracked_dynamic_obstacles /predicted_dynamic_obstacles /tentative_dynamic_obstacles /scan /odom /mujoco/pedestrians /benchmark/metrics /tf /tf_static' \
      bash "${CONTAINER_ROSBAG}" \
      >"${HOST_ROSBAG_LOG}" 2>&1 &
    BAG_PID="$!"
  fi
fi

{
  if [ "${VISUAL_SESSION}" = "1" ]; then
    printf '%s Visual Navigation Session (unscored)\n' "${ENVIRONMENT_NAME}"
  else
    printf '%s Person Avoidance Benchmark\n' "${ENVIRONMENT_NAME}"
  fi
  printf '========================================\n'
  printf 'Timestamp UTC: %s\n' "${STAMP}"
  printf 'Seed: %s\n' "${SEED}"
  printf 'People: %s\n' "${PEOPLE}"
  printf 'Goals: %s\n' "${GOALS}"
  printf 'Mode: %s\n' "${MODE}"
  printf 'Backend: %s\n' "${BACKEND}"
  printf 'Environment: %s\n' "${ENVIRONMENT_NAME}"
  printf 'MuJoCo scene: %s\n' "${SCENE}"
  if [ "${VISUAL_SESSION}" = "1" ]; then
    printf 'Note: onboard sensor-driven visual session; simulator truth is used only for diagnostics.\n'
  elif [ "${BACKEND}" = "mock" ]; then
    printf 'Note: benchmark scoring uses the deterministic mock backend, not live MuJoCo state.\n'
  else
    printf 'Note: benchmark scoring uses live /odom and /mujoco/pedestrians state and sends Nav2 NavigateToPose goals.\n'
  fi
  printf '\n'
} >"${HOST_TXT}"

BENCHMARK_ARGS=(
  --config "${CONFIG}"
  --mode "${MODE}"
  --backend "${BACKEND}"
  --seed "${SEED}"
  --people "${PEOPLE}"
  --goals "${GOALS}"
  --headless
  --output "${CONTAINER_JSON}"
  "${EXTRA_ARGS[@]}"
)
docker compose run --rm -e COLCON_INSTALL_BASE="${CONTAINER_INSTALL_BASE}" unitree-g1-sim bash -lc \
  'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; source "${COLCON_INSTALL_BASE}/setup.bash"; exec ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark "$@"' \
  bash "${BENCHMARK_ARGS[@]}" | tee -a "${HOST_TXT}"

set +e
VALIDATION_ARGS=(
  "${HOST_JSON}"
  --backend "${BACKEND}"
  --minimum-score "${MINIMUM_SCORE}"
)
if [ "${REQUIRE_AVOIDANCE}" = "1" ]; then
  VALIDATION_ARGS+=(--require-avoidance)
fi
if [ "${VISUAL_SESSION}" = "1" ]; then
  printf 'Visual session completed; contact events are recorded. This is not a scored acceptance run.\n' | tee -a "${HOST_TXT}"
  VALIDATION_STATUS=0
else
  python3 "${SCRIPT_DIR}/validate_benchmark_result.py" "${VALIDATION_ARGS[@]}" | tee -a "${HOST_TXT}"
  VALIDATION_STATUS="${PIPESTATUS[0]}"
fi
set -e

{
  printf '\nArtifacts\n'
  printf '%s\n' '---------'
  if [ "${VISUAL_SESSION}" = "1" ]; then
    printf 'Session report: %s\n' "${HOST_TXT}"
  else
    printf 'Score report: %s\n' "${HOST_TXT}"
  fi
  printf 'JSON result: %s\n' "${HOST_JSON}"
  printf 'Scenario replay: %s\n' "${HOST_JSON%.json}_scenario.yaml"
  printf 'Per-goal CSV: %s\n' "${HOST_JSON%.json}_goals.csv"
  printf 'Safety events CSV: %s\n' "${HOST_JSON%.json}_events.csv"
  printf 'Human-aware JSONL: %s\n' "${HOST_RUN_LOG}"
  if [ "${RECORD_ROSBAG}" = "1" ]; then
    printf 'Rosbag: %s\n' "${HOST_ROSBAG}"
    printf 'Rosbag log: %s\n' "${HOST_ROSBAG_LOG}"
  fi
  if [ "${START_SIM}" = "1" ]; then
    printf 'Simulation log: %s\n' "${HOST_SIM_LOG}"
  fi
} | tee -a "${HOST_TXT}"

if [ "${VISUAL_SESSION}" = "1" ]; then
  printf '\nVisual session report written to %s\n' "${HOST_TXT}"
else
  printf '\nBenchmark score report written to %s\n' "${HOST_TXT}"
fi
exit "${VALIDATION_STATUS}"
