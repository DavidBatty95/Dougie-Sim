#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export MJLAB_TASK_ID="Mjlab-Velocity-Flat-PhaseGaitBootstrap-Unitree-G1"
export MJLAB_RUN_NAME_PREFIX="g1_phase_gait_bootstrap"
export MJLAB_SOURCE_RUN_GLOB="*_g1_phase_gait_bootstrap_*"
export MJLAB_FALLBACK_RUN="g1_flat_locomotion_teacher"
export MJLAB_FALLBACK_CHECKPOINT="model_2400_phase.pt"
export MJLAB_SOURCE_RUN="${MJLAB_PHASE_GAIT_TEACHER_RUN:-${MJLAB_FALLBACK_RUN}}"
export MJLAB_CHECKPOINT="${MJLAB_PHASE_GAIT_TEACHER_CHECKPOINT:-${MJLAB_FALLBACK_CHECKPOINT}}"
export MJLAB_NAVIGATION_ITERATIONS="${MJLAB_PHASE_GAIT_BOOTSTRAP_ITERATIONS:-600}"

exec "${SCRIPT_DIR}/mjlab_train_g1_navigation_finetune.sh" "$@"
