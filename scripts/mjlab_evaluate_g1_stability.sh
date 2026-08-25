#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_ROOT_HOST="${MJLAB_LOG_ROOT_HOST:-${REPO_ROOT}/vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop}"
LOG_ROOT_CONTAINER="${MJLAB_LOG_ROOT_CONTAINER:-/workspace/vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop}"

usage() {
  cat <<'EOF'
Usage: ./scripts/mjlab_evaluate_g1_stability.sh [latest|CHECKPOINT] [options]

Run the automated nominal stability acceptance suite. The command exits 0 on
PASS, 1 when a gate fails, and 2 if evaluation cannot run.

Examples:
  ./scripts/mjlab_evaluate_g1_stability.sh latest
  ./scripts/mjlab_evaluate_g1_stability.sh latest --mode randomized --num-envs 64
  ./scripts/mjlab_evaluate_g1_stability.sh vendor/mjlab/logs/.../model_5000.pt

Additional options are forwarded to evaluate_g1_stability.py. Use --help after
a checkpoint argument to see all thresholds.
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

REQUESTED_CHECKPOINT="${1:-latest}"
if [ "$#" -gt 0 ]; then
  shift
fi

if [ "${REQUESTED_CHECKPOINT}" = "latest" ]; then
  if [ ! -d "${LOG_ROOT_HOST}" ]; then
    echo "Log root does not exist: ${LOG_ROOT_HOST}" >&2
    exit 2
  fi
  shopt -s nullglob
  RUN_DIRS=(
    "${LOG_ROOT_HOST}"/*_g1_stand_creep_stop
    "${LOG_ROOT_HOST}"/*_g1_stand_creep_stop_finetune_from_3000
    "${LOG_ROOT_HOST}"/*_g1_stand_creep_stop_recovery_from_3400
    "${LOG_ROOT_HOST}"/*_g1_stand_creep_stop_stop_tune_from_4300
    "${LOG_ROOT_HOST}"/*_g1_stand_creep_stop_final_tune_from_4999
    "${LOG_ROOT_HOST}"/*_g1_stand_creep_stop_reverse_yaw_from_5298
    "${LOG_ROOT_HOST}"/*_g1_stand_creep_stop_yaw_polish_from_5597
  )
  shopt -u nullglob
  LATEST_RUN=""
  CHECKPOINT_HOST=""
  for RUN_DIR in "${RUN_DIRS[@]}"; do
    shopt -s nullglob
    CHECKPOINTS=("${RUN_DIR}"/model_*.pt)
    shopt -u nullglob
    BEST_STEP=-1
    BEST_CHECKPOINT=""
    for CANDIDATE in "${CHECKPOINTS[@]}"; do
      STEP="$(basename "${CANDIDATE}" .pt)"
      STEP="${STEP#model_}"
      if [[ "${STEP}" =~ ^[0-9]+$ ]] && [ "${STEP}" -gt "${BEST_STEP}" ]; then
        BEST_STEP="${STEP}"
        BEST_CHECKPOINT="${CANDIDATE}"
      fi
    done
    if [ -n "${BEST_CHECKPOINT}" ] && [[ "$(basename "${RUN_DIR}")" > "${LATEST_RUN}" ]]; then
      LATEST_RUN="$(basename "${RUN_DIR}")"
      CHECKPOINT_HOST="${BEST_CHECKPOINT}"
    fi
  done
  if [ -z "${CHECKPOINT_HOST}" ]; then
    echo "No checkpoint found under ${LOG_ROOT_HOST}" >&2
    exit 2
  fi
  CHECKPOINT_CONTAINER="${LOG_ROOT_CONTAINER}/${LATEST_RUN}/$(basename "${CHECKPOINT_HOST}")"
else
  if [[ "${REQUESTED_CHECKPOINT}" = /workspace/* ]]; then
    CHECKPOINT_CONTAINER="${REQUESTED_CHECKPOINT}"
    CHECKPOINT_HOST="${REPO_ROOT}/${REQUESTED_CHECKPOINT#/workspace/}"
  else
    if [[ "${REQUESTED_CHECKPOINT}" = /* ]]; then
      CHECKPOINT_HOST="${REQUESTED_CHECKPOINT}"
    else
      CHECKPOINT_HOST="${REPO_ROOT}/${REQUESTED_CHECKPOINT}"
    fi
    CHECKPOINT_HOST="$(realpath "${CHECKPOINT_HOST}")"
    if [[ "${CHECKPOINT_HOST}" != "${REPO_ROOT}/"* ]]; then
      echo "Checkpoint must be inside ${REPO_ROOT} so Docker can access it." >&2
      exit 2
    fi
    CHECKPOINT_CONTAINER="/workspace/${CHECKPOINT_HOST#${REPO_ROOT}/}"
  fi
  if [ ! -f "${CHECKPOINT_HOST}" ]; then
    echo "Checkpoint does not exist: ${CHECKPOINT_HOST}" >&2
    exit 2
  fi
fi

OUTPUT_HOST="${CHECKPOINT_HOST%.pt}_stability_evaluation.json"
OUTPUT_CONTAINER="${CHECKPOINT_CONTAINER%.pt}_stability_evaluation.json"
echo "Evaluating: ${CHECKPOINT_HOST}"
echo "Report:     ${OUTPUT_HOST}"

export MJLAB_WITH_GPUS=1
exec "${SCRIPT_DIR}/run_mjlab_docker.sh" python \
  /workspace/scripts/evaluate_g1_stability.py \
  --checkpoint "${CHECKPOINT_CONTAINER}" \
  --output "${OUTPUT_CONTAINER}" \
  "$@"
