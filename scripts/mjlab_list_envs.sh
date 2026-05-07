#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MJLAB_WITH_GPUS="${MJLAB_WITH_GPUS:-0}" exec "${SCRIPT_DIR}/run_mjlab_docker.sh" list-envs "$@"
