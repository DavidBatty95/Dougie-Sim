#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export MJLAB_TASK_ID="Mjlab-Velocity-Flat-PhaseTurnFineTune-Unitree-G1"
export MJLAB_RUN_NAME_PREFIX="g1_phase_turn_finetune"
export MJLAB_SOURCE_RUN_GLOB="*_g1_phase_turn_finetune_*"
export MJLAB_FALLBACK_RUN="2026-08-05_10-32-24_g1_navigation_finetune_from_model_5796"
export MJLAB_FALLBACK_CHECKPOINT="model_6595_phase.pt"
export MJLAB_SOURCE_RUN="${MJLAB_PHASE_TURN_SOURCE_RUN:-${MJLAB_FALLBACK_RUN}}"
export MJLAB_CHECKPOINT="${MJLAB_PHASE_TURN_CHECKPOINT:-${MJLAB_FALLBACK_CHECKPOINT}}"
export MJLAB_NAVIGATION_ITERATIONS="${MJLAB_PHASE_TURN_ITERATIONS:-800}"

exec "${SCRIPT_DIR}/mjlab_train_g1_navigation_finetune.sh" "$@"
