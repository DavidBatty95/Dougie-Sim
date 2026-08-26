#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export MJLAB_TASK_ID="Mjlab-Velocity-Flat-NavigationFineTune-Unitree-G1"
export MJLAB_RUN_NAME_PREFIX="g1_navigation_yaw_finetune"
export MJLAB_SOURCE_RUN_GLOB="*_g1_navigation_yaw_finetune_*"
export MJLAB_FALLBACK_RUN="2026-08-05_10-32-24_g1_navigation_finetune_from_model_5796"
export MJLAB_FALLBACK_CHECKPOINT="model_6595.pt"
export MJLAB_SOURCE_RUN="${MJLAB_YAW_SOURCE_RUN:-${MJLAB_FALLBACK_RUN}}"
export MJLAB_CHECKPOINT="${MJLAB_YAW_CHECKPOINT:-${MJLAB_FALLBACK_CHECKPOINT}}"
export MJLAB_NAVIGATION_ITERATIONS="${MJLAB_YAW_ITERATIONS:-50}"

exec "${SCRIPT_DIR}/mjlab_train_g1_navigation_finetune.sh" "$@"
