#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SEED="42"
PEOPLE="8"
GOALS="10"
MODE="standard"
BACKEND="mujoco"
START_SIM="1"
SIM_STARTUP_SECONDS="28"
RUN_RVIZ="1"
RECORD_ROSBAG="0"
RESULT_DIR="${REPO_ROOT}/ros2_ws/src/dynamic_obstacle_benchmarks/results/extreme_lab"
EXTRA_ARGS=()
SIM_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_extreme_lab_person_avoidance_benchmark.sh [options]

Options:
  --seed N                 Benchmark seed (default: 42)
  --people N               Number of people (default: 8)
  --goals N                Number of goals (default: 10)
  --mode MODE              smoke|standard|stress|crossing|bottleneck|occlusion (default: standard)
  --backend BACKEND        mock|mujoco (default: mujoco)
  --no-sim                 Do not launch Extreme Lab simulation first
  --no-rviz                Do not launch RViz with the simulation
  --record-rosbag          Record diagnostic ROS topics during the benchmark
  --startup-seconds N      Seconds to wait after launching simulation (default: 28)
  --sim-arg ARG            Extra ROS launch argument passed to g1_sim.launch.py, e.g. avoidance_danger_radius:=2.4
  --result-dir PATH        Host result directory (default: package results/extreme_lab)

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
    --no-sim) START_SIM="0"; shift ;;
    --no-rviz) RUN_RVIZ="0"; shift ;;
    --record-rosbag) RECORD_ROSBAG="1"; EXTRA_ARGS+=("--record-rosbag"); shift ;;
    --startup-seconds) SIM_STARTUP_SECONDS="$2"; shift 2 ;;
    --sim-arg) SIM_ARGS+=("$2"); shift 2 ;;
    --result-dir) RESULT_DIR="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

mkdir -p "${RESULT_DIR}"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BASENAME="person_avoidance_extreme_lab_${MODE}_seed_${SEED}_${STAMP}"
HOST_JSON="${RESULT_DIR}/${BASENAME}.json"
HOST_TXT="${RESULT_DIR}/${BASENAME}.txt"
HOST_RUN_LOG="${RESULT_DIR}/${BASENAME}_human_aware.jsonl"
HOST_ROSBAG="${RESULT_DIR}/${BASENAME}_rosbag"
if [[ "${RESULT_DIR}" == "${REPO_ROOT}"/* ]]; then
  CONTAINER_RESULT_DIR="/workspace/${RESULT_DIR#${REPO_ROOT}/}"
else
  CONTAINER_RESULT_DIR="/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/extreme_lab"
fi
CONTAINER_JSON="${CONTAINER_RESULT_DIR}/${BASENAME}.json"
CONTAINER_RUN_LOG="${CONTAINER_RESULT_DIR}/${BASENAME}_human_aware.jsonl"
CONTAINER_ROSBAG="${CONTAINER_RESULT_DIR}/${BASENAME}_rosbag"
CONTAINER_CONFIG="/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/person_avoidance_benchmark.yaml"

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
  printf 'Checking human-aware logger executable...\n'
  if ! docker compose run --rm unitree-g1-sim bash -lc 'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi; ros2 pkg executables unitree_g1_mujoco_bridge | grep -q "^unitree_g1_mujoco_bridge human_aware_run_logger$"'; then
    printf 'Building unitree_g1_mujoco_bridge so human_aware_run_logger is installed...\n'
    docker compose run --rm unitree-g1-sim bash -lc 'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; cd /workspace/ros2_ws && colcon build --merge-install --symlink-install --packages-select unitree_g1_mujoco_bridge && source /workspace/ros2_ws/install/setup.bash'
  fi
  printf 'Refreshing bridge package...\n'
  docker compose run --rm unitree-g1-sim bash -lc 'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; cd /workspace/ros2_ws && colcon build --merge-install --symlink-install --packages-select unitree_g1_mujoco_bridge >/dev/null && source /workspace/ros2_ws/install/setup.bash'
  printf 'Refreshing bringup launch files...\n'
  docker compose run --rm unitree-g1-sim bash -lc 'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; cd /workspace/ros2_ws && colcon build --merge-install --symlink-install --packages-select unitree_g1_sim_bringup >/dev/null && source /workspace/ros2_ws/install/setup.bash'
  printf 'Refreshing benchmark package...\n'
  docker compose run --rm unitree-g1-sim bash -lc 'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; cd /workspace/ros2_ws && colcon build --merge-install --symlink-install --packages-select dynamic_obstacle_benchmarks >/dev/null && source /workspace/ros2_ws/install/setup.bash'
  printf 'Starting Extreme Lab simulation for benchmark...\n'
  setsid "${SCRIPT_DIR}/bringup_full_sim.sh" \
    mujoco_scene:=scene_29dof_extreme_lab.xml \
    mujoco_pedestrians_enable:=true \
    mujoco_pedestrians_count:="${PEOPLE}" \
    livox_scan_points:=1200 \
    mujoco_initial_keyframe:=policy_stand \
    startup_current_pose_hold_time:=0.0 \
    startup_blend_time:=0.0 \
    startup_hold_time:=0.0 \
    startup_require_upright:=true \
    startup_max_recovery_time:=0.0 \
    policy_action_ramp_time:=0.0 \
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
    run_lidar_person_detector:=true \
    run_rviz:="${RUN_RVIZ}" \
    publish_camera:=false \
    "${SIM_ARGS[@]}" \
    >"${RESULT_DIR}/${BASENAME}_sim.log" 2>&1 &
  SIM_PID="$!"
  printf 'Waiting %s seconds for simulation startup...\n' "${SIM_STARTUP_SECONDS}"
  sleep "${SIM_STARTUP_SECONDS}"
  if [ "${RECORD_ROSBAG}" = "1" ]; then
    printf 'Starting diagnostic rosbag recording...\n'
    setsid docker compose run --rm unitree-g1-sim bash -lc \
      'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi; mkdir -p "$(dirname "$1")"; exec ros2 bag record -o "$1" /cmd_vel /cmd_vel_human_aware /cmd_vel_safe /human_avoidance_state /human_avoidance_debug /human_collision_avoidance_debug /dynamic_obstacle_tracker/debug_tracks /tracked_dynamic_obstacles /predicted_dynamic_obstacles /tentative_dynamic_obstacles /scan /odom /mujoco/pedestrians /benchmark/metrics /tf /tf_static' \
      bash "${CONTAINER_ROSBAG}" \
      >"${RESULT_DIR}/${BASENAME}_rosbag.log" 2>&1 &
    BAG_PID="$!"
  fi
fi

{
  printf 'Extreme Lab Person Avoidance Benchmark\n'
  printf '======================================\n'
  printf 'Timestamp UTC: %s\n' "${STAMP}"
  printf 'Seed: %s\n' "${SEED}"
  printf 'People: %s\n' "${PEOPLE}"
  printf 'Goals: %s\n' "${GOALS}"
  printf 'Mode: %s\n' "${MODE}"
  printf 'Backend: %s\n' "${BACKEND}"
  printf 'MuJoCo scene: scene_29dof_extreme_lab.xml\n'
  if [ "${BACKEND}" = "mock" ]; then
    printf 'Note: benchmark scoring uses the deterministic mock backend, not live MuJoCo state.\n'
  else
    printf 'Note: benchmark scoring uses live /odom and /mujoco/pedestrians state and sends Nav2 NavigateToPose goals.\n'
  fi
  printf '\n'
} >"${HOST_TXT}"

docker compose run --rm unitree-g1-sim bash -lc \
  'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi; if ! ros2 pkg prefix dynamic_obstacle_benchmarks >/dev/null 2>&1; then cd /workspace/ros2_ws && colcon build --merge-install --symlink-install --packages-select dynamic_obstacle_benchmarks && source /workspace/ros2_ws/install/setup.bash; fi; exec ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark "$@"' \
  bash \
  --config "${CONTAINER_CONFIG}" \
  --mode "${MODE}" \
  --backend "${BACKEND}" \
  --seed "${SEED}" \
  --people "${PEOPLE}" \
  --goals "${GOALS}" \
  --headless \
  --output "${CONTAINER_JSON}" \
  "${EXTRA_ARGS[@]}" | tee -a "${HOST_TXT}"

{
  printf '\nArtifacts\n'
  printf '%s\n' '---------'
  printf 'Score report: %s\n' "${HOST_TXT}"
  printf 'JSON result: %s\n' "${HOST_JSON}"
  printf 'Scenario replay: %s\n' "${HOST_JSON%.json}_scenario.yaml"
  printf 'Per-goal CSV: %s\n' "${HOST_JSON%.json}_goals.csv"
  printf 'Safety events CSV: %s\n' "${HOST_JSON%.json}_events.csv"
  printf 'Human-aware JSONL: %s\n' "${HOST_RUN_LOG}"
  if [ "${RECORD_ROSBAG}" = "1" ]; then
    printf 'Rosbag: %s\n' "${HOST_ROSBAG}"
    printf 'Rosbag log: %s\n' "${RESULT_DIR}/${BASENAME}_rosbag.log"
  fi
  if [ "${START_SIM}" = "1" ]; then
    printf 'Simulation log: %s\n' "${RESULT_DIR}/${BASENAME}_sim.log"
  fi
} | tee -a "${HOST_TXT}"

printf '\nBenchmark score report written to %s\n' "${HOST_TXT}"
