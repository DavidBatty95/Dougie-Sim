#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_NAME="${MJLAB_RUN_NAME:-g1_flat_locomotion}"
SHOW_GUI="${MJLAB_SHOW_GUI:-0}"
PASSTHROUGH_ARGS=()

while [ "$#" -gt 0 ]; do
  case "$1" in
    --show-gui)
      SHOW_GUI=1
      shift
      ;;
    *)
      PASSTHROUGH_ARGS+=("$1")
      shift
      ;;
  esac
done

if [ "${SHOW_GUI}" = "1" ]; then
  NUM_ENVS="${MJLAB_GUI_NUM_ENVS:-1024}"
else
  NUM_ENVS="${MJLAB_NUM_ENVS:-8192}"
fi

ARGS=(
  --agent.logger tensorboard
  --agent.run-name "${RUN_NAME}"
  --env.scene.num-envs "${NUM_ENVS}"
)

if [ "${SHOW_GUI}" = "1" ]; then
  ARGS+=(
    --show-gui True
    --env.viewer.max-extra-envs 0
  )
fi

if [ "${MJLAB_AUTO_RESUME:-1}" = "1" ]; then
  LOG_ROOT_HOST="${MJLAB_LOG_ROOT_HOST:-${SCRIPT_DIR}/../vendor/mjlab/logs/rsl_rl/g1_velocity}"
  if [ -d "${LOG_ROOT_HOST}" ]; then
    shopt -s nullglob
    RUN_DIRS=("${LOG_ROOT_HOST}"/*"${RUN_NAME}")
    shopt -u nullglob
    LATEST_RUN=""
    for RUN_DIR in "${RUN_DIRS[@]}"; do
      shopt -s nullglob
      CHECKPOINTS=("${RUN_DIR}"/model_*.pt)
      shopt -u nullglob
      if [ "${#CHECKPOINTS[@]}" -gt 0 ]; then
        LATEST_RUN="$(basename "${RUN_DIR}")"
      fi
    done
    if [ -n "${LATEST_RUN}" ]; then
      ARGS+=(
        --agent.resume True
        --agent.load-run "^${LATEST_RUN}$"
        --agent.load-checkpoint "model_.*.pt"
      )
    fi
  fi
fi

exec "${SCRIPT_DIR}/run_mjlab_docker.sh" train Mjlab-Velocity-Flat-Unitree-G1 \
  "${ARGS[@]}" \
  "${PASSTHROUGH_ARGS[@]}"
