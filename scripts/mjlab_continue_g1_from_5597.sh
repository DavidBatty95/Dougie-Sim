#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_ROOT_HOST="${MJLAB_LOG_ROOT_HOST:-${REPO_ROOT}/vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop}"
SOURCE_RUN="${MJLAB_SOURCE_RUN:-}"
DRY_RUN=0
PASSTHROUGH_ARGS=()

while [ "$#" -gt 0 ]; do
  case "$1" in
    --source-run)
      if [ "$#" -lt 2 ]; then
        echo "--source-run requires a run directory name." >&2
        exit 2
      fi
      SOURCE_RUN="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      cat <<'EOF'
Usage: ./scripts/mjlab_continue_g1_from_5597.sh [options] [training-options]

Run the final yaw/action polish from exactly model_5597.pt. By default this runs
200 additional iterations, increasing the effective limit to about 5,797.

Options:
  --source-run NAME  Select the source run when more than one has model_3000.pt.
  --dry-run          Print the resolved training command without starting it.
  -h, --help         Show this help.

Environment overrides:
  MJLAB_NUM_ENVS=8192
  MJLAB_FINETUNE_ITERATIONS=200
  MJLAB_SOURCE_RUN=<run-directory-name>
EOF
      exit 0
      ;;
    *)
      PASSTHROUGH_ARGS+=("$1")
      shift
      ;;
  esac
done

if [ -n "${SOURCE_RUN}" ]; then
  CHECKPOINT="${LOG_ROOT_HOST}/${SOURCE_RUN}/model_5597.pt"
  if [ ! -f "${CHECKPOINT}" ]; then
    echo "Checkpoint does not exist: ${CHECKPOINT}" >&2
    exit 2
  fi
else
  shopt -s nullglob
  CHECKPOINTS=("${LOG_ROOT_HOST}"/*/model_5597.pt)
  shopt -u nullglob
  if [ "${#CHECKPOINTS[@]}" -eq 0 ]; then
    echo "No model_5597.pt found under ${LOG_ROOT_HOST}" >&2
    exit 2
  fi
  if [ "${#CHECKPOINTS[@]}" -gt 1 ]; then
    echo "Multiple model_5597.pt files found; select one with --source-run:" >&2
    for CHECKPOINT in "${CHECKPOINTS[@]}"; do
      echo "  $(basename "$(dirname "${CHECKPOINT}")")" >&2
    done
    exit 2
  fi
  CHECKPOINT="${CHECKPOINTS[0]}"
  SOURCE_RUN="$(basename "$(dirname "${CHECKPOINT}")")"
fi

NUM_ENVS="${MJLAB_NUM_ENVS:-8192}"
ITERATIONS="${MJLAB_FINETUNE_ITERATIONS:-200}"
if ! [[ "${NUM_ENVS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "MJLAB_NUM_ENVS must be a positive integer." >&2
  exit 2
fi
if ! [[ "${ITERATIONS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "MJLAB_FINETUNE_ITERATIONS must be a positive integer." >&2
  exit 2
fi

echo "Fine-tuning from: ${CHECKPOINT}"
echo "Additional iterations: ${ITERATIONS} (target iteration: $((5597 + ITERATIONS)))"
echo "The old optimizer will not be restored because the objective has changed."

export MJLAB_WITH_GPUS=1
export MJLAB_AUTO_RESUME=0
COMMAND=("${SCRIPT_DIR}/run_mjlab_docker.sh" train \
  Mjlab-Velocity-Flat-StandCreepStopFineTune-Unitree-G1 \
  --agent.logger tensorboard \
  --agent.run-name g1_stand_creep_stop_yaw_polish_from_5597 \
  --agent.resume True \
  --agent.load-run "^${SOURCE_RUN}$" \
  --agent.load-checkpoint '^model_5597[.]pt$' \
  --agent.max-iterations "${ITERATIONS}" \
  --agent.algorithm.learning-rate 0.0000025 \
  --agent.algorithm.entropy-coef 0.003 \
  --env.commands.twist.initial-standing-probability 0.30 \
  --env.commands.twist.moving-stop-probability 0.45 \
  --env.commands.twist.standing-hold-probability 0.25 \
  --env.commands.twist.low-speed-probability 0.40 \
  --env.commands.twist.low-speed-range '(0.05,0.10)' \
  --env.commands.twist.moderate-speed-probability 0.10 \
  --env.commands.twist.lateral-probability 0.05 \
  --env.commands.twist.reverse-probability 0.70 \
  --env.commands.twist.yaw-speed-min 0.12 \
  --env.commands.twist.yaw-speed-max 0.25 \
  --env.rewards.track-linear-velocity.weight 6.0 \
  --env.rewards.track-linear-velocity.params.std 0.10 \
  --env.rewards.track-angular-velocity.weight 8.0 \
  --env.rewards.track-angular-velocity.params.std 0.15 \
  --env.rewards.upright.weight 1.75 \
  --env.rewards.pose.weight 1.5 \
  --env.rewards.air-time.weight 0.05 \
  --env.rewards.action-rate-l2.weight -0.15 \
  --env.rewards.action-excess-l2.weight -5.0 \
  --env.rewards.action-magnitude-l2.weight -0.02 \
  --env.rewards.standing-base-velocity.weight -4.0 \
  --env.scene.num-envs "${NUM_ENVS}" \
  --resume-optimizer False \
  "${PASSTHROUGH_ARGS[@]}")

if [ "${DRY_RUN}" = "1" ]; then
  printf 'Command:'
  printf ' %q' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

exec "${COMMAND[@]}"
