#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SEED="42"
PEOPLE="8"
GOALS="10"
MODE="standard"
BACKEND="mock"
OUTPUT=""
EXTRA_ARGS=()

while [ "$#" -gt 0 ]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --people) PEOPLE="$2"; shift 2 ;;
    --goals) GOALS="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;
    --output) OUTPUT="$2"; shift 2 ;;
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

if [ -z "${OUTPUT}" ]; then
  OUTPUT="/workspace/results/person_avoidance_seed_${SEED}.json"
fi

docker compose run --rm unitree-g1-sim bash -lc \
  'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi; exec ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark "$@"' \
  bash \
  --config /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/person_avoidance_benchmark.yaml \
  --mode "${MODE}" \
  --backend "${BACKEND}" \
  --seed "${SEED}" \
  --people "${PEOPLE}" \
  --goals "${GOALS}" \
  --headless \
  --output "${OUTPUT}" \
  "${EXTRA_ARGS[@]}"
