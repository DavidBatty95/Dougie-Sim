#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_IMAGE=1
SYNC_DEPS=1
AUTO_RESUME="${MJLAB_AUTO_RESUME:-0}"
TRAINING_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./scripts/launch_g1_policy_training.sh [options] [-- training-options]

Build the training image, synchronize MJLab dependencies, and train the G1
stand/creep/stop policy on the GPU. ROS 2 and the standalone simulator are not
required because MJLab creates the training simulation in-process.

Options:
  --fresh          Start a new run (default).
  --resume         Resume the latest g1_stand_creep_stop checkpoint.
  --skip-build     Do not build/update the Docker image.
  --skip-sync      Do not synchronize the MJLab Python environment.
  --show-gui       Show the MJLab viewer while training.
  --num-envs N     Train with N parallel environments (default: 8192).
  -h, --help       Show this help.

Unrecognized arguments are forwarded to MJLab. For example:
  ./scripts/launch_g1_policy_training.sh --fresh --num-envs 4096 \
    --agent.max-iterations 30000 --agent.seed 43
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --fresh)
      AUTO_RESUME=0
      shift
      ;;
    --resume)
      AUTO_RESUME=1
      shift
      ;;
    --skip-build)
      BUILD_IMAGE=0
      shift
      ;;
    --skip-sync)
      SYNC_DEPS=0
      shift
      ;;
    --show-gui)
      TRAINING_ARGS+=(--show-gui)
      shift
      ;;
    --num-envs)
      if [ "$#" -lt 2 ] || ! [[ "$2" =~ ^[1-9][0-9]*$ ]]; then
        echo "--num-envs requires a positive integer." >&2
        exit 2
      fi
      export MJLAB_NUM_ENVS="$2"
      export MJLAB_GUI_NUM_ENVS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      TRAINING_ARGS+=("$@")
      break
      ;;
    *)
      TRAINING_ARGS+=("$1")
      shift
      ;;
  esac
done

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is required but was not found on PATH." >&2
  exit 1
fi
if ! docker compose version >/dev/null 2>&1; then
  echo "Docker Compose v2 is required (the 'docker compose' command)." >&2
  exit 1
fi
if ! command -v nvidia-smi >/dev/null 2>&1; then
  echo "NVIDIA drivers and nvidia-smi are required for MJLab training." >&2
  exit 1
fi
if ! NVIDIA_STATUS="$(nvidia-smi 2>&1)"; then
  echo "NVIDIA GPU preflight failed:" >&2
  echo "${NVIDIA_STATUS}" >&2
  exit 1
fi

cd "${REPO_ROOT}"

if [ "${BUILD_IMAGE}" = "1" ]; then
  echo "Building the training container (cached layers will be reused)..."
  docker compose build unitree-g1-sim
fi

if [ "${SYNC_DEPS}" = "1" ]; then
  echo "Synchronizing the MJLab CUDA environment..."
  docker compose run --rm unitree-g1-sim /workspace/scripts/setup_mjlab.sh
fi

export MJLAB_WITH_GPUS=1
export MJLAB_AUTO_RESUME="${AUTO_RESUME}"

if [ "${AUTO_RESUME}" = "1" ]; then
  echo "Starting G1 stand/creep/stop training (resume latest checkpoint if present)..."
else
  echo "Starting a fresh G1 stand/creep/stop training run..."
fi
echo "Outputs: ${REPO_ROOT}/vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop/"

exec "${SCRIPT_DIR}/mjlab_train_g1_stand_creep_stop.sh" "${TRAINING_ARGS[@]}"
