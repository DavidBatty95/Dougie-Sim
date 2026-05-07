#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

GPU_FLAGS=()
if [ "${MJLAB_WITH_GPUS:-0}" = "1" ]; then
  GPU_FLAGS=(--gpus "${MJLAB_GPUS:-all}")
fi

cd "${REPO_ROOT}"
exec docker compose run --rm "${GPU_FLAGS[@]}" unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh "$@"
