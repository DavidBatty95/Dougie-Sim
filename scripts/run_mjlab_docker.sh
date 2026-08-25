#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

COMPOSE_FILES=()
if [ "${MJLAB_WITH_GPUS:-0}" = "1" ]; then
  if ! command -v nvidia-smi >/dev/null 2>&1; then
    echo "MJLab GPU execution requires the NVIDIA driver and nvidia-smi on the host." >&2
    exit 1
  fi
  if ! NVIDIA_STATUS="$(nvidia-smi 2>&1)"; then
    echo "MJLab GPU preflight failed; repair/reload the host NVIDIA driver before training:" >&2
    echo "${NVIDIA_STATUS}" >&2
    exit 1
  fi
  export NVIDIA_VISIBLE_DEVICES="${MJLAB_GPUS:-${NVIDIA_VISIBLE_DEVICES:-all}}"
  COMPOSE_FILES=(
    -f "${REPO_ROOT}/docker-compose.yml"
    -f "${REPO_ROOT}/docker-compose.gpu.yml"
  )
fi

cd "${REPO_ROOT}"
exec docker compose "${COMPOSE_FILES[@]}" run --rm unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh "$@"
