#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export MJLAB_TASK_ID="Mjlab-Velocity-Flat-StepFoundation-Unitree-G1"
export MJLAB_RUN_NAME_PREFIX="g1_step_foundation"
export MJLAB_SOURCE_RUN_GLOB="*_g1_step_foundation_*"
export MJLAB_FALLBACK_RUN="2026-08-05_10-32-24_g1_navigation_finetune_from_model_5796"
export MJLAB_FALLBACK_CHECKPOINT="model_6595.pt"
export MJLAB_NAVIGATION_ITERATIONS="${MJLAB_STEP_FOUNDATION_ITERATIONS:-3000}"

exec "${SCRIPT_DIR}/mjlab_train_g1_navigation_finetune.sh" "$@"
