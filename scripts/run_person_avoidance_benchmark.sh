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
MINIMUM_SCORE="95"
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_person_avoidance_benchmark.sh [options]

Runs the deterministic mock benchmark and validates its artifacts and score.

Options:
  --seed N             Benchmark seed (default: 42)
  --people N           Number of simulated people (default: 8)
  --goals N            Number of goals (default: 10)
  --mode MODE          Benchmark mode (default: standard)
  --backend BACKEND    mock|mujoco (default: mock)
  --minimum-score N    Required score (default: 95)
  --output PATH        Host path or /workspace path for the JSON result
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --people) PEOPLE="$2"; shift 2 ;;
    --goals) GOALS="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;
    --minimum-score) MINIMUM_SCORE="$2"; shift 2 ;;
    --output) OUTPUT="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

if [ -z "${OUTPUT}" ]; then
  OUTPUT="${REPO_ROOT}/ros2_ws/src/dynamic_obstacle_benchmarks/results/mock/person_avoidance_seed_${SEED}.json"
fi

if [[ "${OUTPUT}" == /workspace/* ]]; then
  HOST_OUTPUT="${REPO_ROOT}/${OUTPUT#/workspace/}"
  CONTAINER_OUTPUT="${OUTPUT}"
else
  if [[ "${OUTPUT}" != /* ]]; then
    OUTPUT="${REPO_ROOT}/${OUTPUT}"
  fi
  HOST_OUTPUT="$(realpath -m "${OUTPUT}")"
  if [[ "${HOST_OUTPUT}" != "${REPO_ROOT}/"* ]]; then
    printf 'Output must be inside %s so Docker can write it: %s\n' \
      "${REPO_ROOT}" "${HOST_OUTPUT}" >&2
    exit 2
  fi
  CONTAINER_OUTPUT="/workspace/${HOST_OUTPUT#${REPO_ROOT}/}"
fi
mkdir -p "$(dirname "${HOST_OUTPUT}")"

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
  --output "${CONTAINER_OUTPUT}" \
  "${EXTRA_ARGS[@]}"

python3 "${SCRIPT_DIR}/validate_benchmark_result.py" \
  "${HOST_OUTPUT}" \
  --backend "${BACKEND}" \
  --minimum-score "${MINIMUM_SCORE}"

printf 'Benchmark artifacts: %s\n' "$(dirname "${HOST_OUTPUT}")"
