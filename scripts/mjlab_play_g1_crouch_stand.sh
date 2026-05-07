#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_NAME="${MJLAB_RUN_NAME:-g1_crouch_stand}"
LOG_ROOT_HOST="${MJLAB_LOG_ROOT_HOST:-${SCRIPT_DIR}/../vendor/mjlab/logs/rsl_rl/g1_velocity}"
LOG_ROOT_CONTAINER="${MJLAB_LOG_ROOT_CONTAINER:-/workspace/vendor/mjlab/logs/rsl_rl/g1_velocity}"
PLAY_NUM_ENVS="${MJLAB_PLAY_NUM_ENVS:-1}"
PLAY_VIEWER="${MJLAB_PLAY_VIEWER:-viser}"

if [ "$#" -gt 0 ] && [ "$1" != "latest" ]; then
  CHECKPOINT_FILE="$1"
  shift
else
  if [ "$#" -gt 0 ]; then
    shift
  fi

  if [ ! -d "${LOG_ROOT_HOST}" ]; then
    echo "Log root does not exist: ${LOG_ROOT_HOST}"
    exit 2
  fi

  shopt -s nullglob
  RUN_DIRS=("${LOG_ROOT_HOST}"/*"${RUN_NAME}")
  shopt -u nullglob

  LATEST_RUN=""
  LATEST_CHECKPOINT=""
  for RUN_DIR in "${RUN_DIRS[@]}"; do
    shopt -s nullglob
    CHECKPOINTS=("${RUN_DIR}"/model_*.pt)
    shopt -u nullglob
    if [ "${#CHECKPOINTS[@]}" -eq 0 ]; then
      continue
    fi

    BEST_STEP=-1
    BEST_CHECKPOINT=""
    for CHECKPOINT in "${CHECKPOINTS[@]}"; do
      STEP="$(basename "${CHECKPOINT}" .pt)"
      STEP="${STEP#model_}"
      if [[ "${STEP}" =~ ^[0-9]+$ ]] && [ "${STEP}" -gt "${BEST_STEP}" ]; then
        BEST_STEP="${STEP}"
        BEST_CHECKPOINT="${CHECKPOINT}"
      fi
    done

    if [ -n "${BEST_CHECKPOINT}" ] && [[ "$(basename "${RUN_DIR}")" > "${LATEST_RUN}" ]]; then
      LATEST_RUN="$(basename "${RUN_DIR}")"
      LATEST_CHECKPOINT="${BEST_CHECKPOINT}"
    fi
  done

  if [ -z "${LATEST_CHECKPOINT}" ]; then
    echo "No checkpoint found in ${LOG_ROOT_HOST} matching ${RUN_NAME}"
    exit 2
  fi

  CHECKPOINT_FILE="${LOG_ROOT_CONTAINER}/${LATEST_RUN}/$(basename "${LATEST_CHECKPOINT}")"
fi

exec "${SCRIPT_DIR}/run_mjlab_docker.sh" play Mjlab-Flat-CrouchStand-Unitree-G1 \
  --checkpoint-file "${CHECKPOINT_FILE}" \
  --num-envs "${PLAY_NUM_ENVS}" \
  --viewer "${PLAY_VIEWER}" \
  "$@"
