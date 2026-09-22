#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_ROOT="${REPO_ROOT}/vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop"
BASE_RUN="2026-08-03_13-14-21_g1_stand_creep_stop_reverse_yaw_from_5298"
FALLBACK_RUN="${MJLAB_FALLBACK_RUN:-${BASE_RUN}}"
FALLBACK_CHECKPOINT="${MJLAB_FALLBACK_CHECKPOINT:-model_5597.pt}"
SOURCE_RUN_GLOB="${MJLAB_SOURCE_RUN_GLOB:-*_g1_navigation_finetune_*}"
TASK_ID="${MJLAB_TASK_ID:-Mjlab-Velocity-Flat-NavigationFineTune-Unitree-G1}"
RUN_NAME_PREFIX="${MJLAB_RUN_NAME_PREFIX:-g1_navigation_finetune}"
SOURCE_RUN="${MJLAB_SOURCE_RUN:-}"
CHECKPOINT="${MJLAB_CHECKPOINT:-}"
NUM_ENVS="${MJLAB_NUM_ENVS:-4096}"
ITERATIONS="${MJLAB_NAVIGATION_ITERATIONS:-1000}"
SEED="${MJLAB_SEED:-42}"
DRY_RUN=0
PASSTHROUGH=()

usage() {
  cat <<'EOF'
Usage: ./scripts/mjlab_train_g1_navigation_finetune.sh [options] [-- training-options]

Fine-tune the G1 policy for forward motion, bilateral yaw, and Nav2 arc commands.
By default the newest navigation run and its highest numbered checkpoint are
resumed. If none exists, training starts from model_5597.pt.

Options:
  --iterations N      Additional iterations (default: 1000)
  --num-envs N        Parallel environments (default: 4096)
  --seed N            Explicit training seed (default: 42)
  --source-run NAME   Explicit source run directory
  --checkpoint FILE   Explicit checkpoint filename
  --from-5597         Restart from the original model_5597.pt
  --dry-run           Print the resolved training command
  -h, --help          Show this help
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --iterations) ITERATIONS="$2"; shift 2 ;;
    --num-envs) NUM_ENVS="$2"; shift 2 ;;
    --seed) SEED="$2"; shift 2 ;;
    --source-run) SOURCE_RUN="$2"; shift 2 ;;
    --checkpoint) CHECKPOINT="$2"; shift 2 ;;
    --from-5597) SOURCE_RUN="${BASE_RUN}"; CHECKPOINT="model_5597.pt"; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    --) shift; PASSTHROUGH+=("$@"); break ;;
    *) PASSTHROUGH+=("$1"); shift ;;
  esac
done

if ! [[ "${NUM_ENVS}" =~ ^[1-9][0-9]*$ && "${ITERATIONS}" =~ ^[1-9][0-9]*$ && "${SEED}" =~ ^[0-9]+$ ]]; then
  printf 'Seed must be nonnegative; iterations and environment count must be positive integers.\n' >&2
  exit 2
fi

if [ -z "${SOURCE_RUN}" ]; then
  shopt -s nullglob
  navigation_runs=("${LOG_ROOT}"/${SOURCE_RUN_GLOB})
  shopt -u nullglob
  if [ "${#navigation_runs[@]}" -gt 0 ]; then
    latest_run="${navigation_runs[0]}"
    for run in "${navigation_runs[@]}"; do
      if [[ "${run}" > "${latest_run}" ]]; then latest_run="${run}"; fi
    done
    SOURCE_RUN="$(basename "${latest_run}")"
  else
    SOURCE_RUN="${FALLBACK_RUN}"
  fi
fi

if [ -z "${CHECKPOINT}" ]; then
  shopt -s nullglob
  checkpoints=("${LOG_ROOT}/${SOURCE_RUN}"/model_*.pt)
  shopt -u nullglob
  highest_iteration=-1
  for path in "${checkpoints[@]}"; do
    filename="$(basename "${path}")"
    iteration="${filename#model_}"
    iteration="${iteration%.pt}"
    if [[ "${iteration}" =~ ^[0-9]+$ ]] && [ "${iteration}" -gt "${highest_iteration}" ]; then
      highest_iteration="${iteration}"
      CHECKPOINT="${filename}"
    fi
  done
fi

if [ -z "${CHECKPOINT}" ] && [ "${SOURCE_RUN}" = "${FALLBACK_RUN}" ]; then
  CHECKPOINT="${FALLBACK_CHECKPOINT}"
fi

if [ -z "${CHECKPOINT}" ] || [ ! -f "${LOG_ROOT}/${SOURCE_RUN}/${CHECKPOINT}" ]; then
  printf 'Checkpoint does not exist: %s/%s\n' "${LOG_ROOT}/${SOURCE_RUN}" "${CHECKPOINT}" >&2
  exit 2
fi

RUN_NAME="${RUN_NAME_PREFIX}_from_${CHECKPOINT%.pt}_seed_${SEED}"
COMMAND=("${SCRIPT_DIR}/run_mjlab_docker.sh" train
  "${TASK_ID}"
  --agent.logger tensorboard
  --agent.run-name "${RUN_NAME}"
  --agent.resume True
  --agent.load-run "^${SOURCE_RUN}$"
  --agent.load-checkpoint "^${CHECKPOINT//./[.]}$"
  --agent.max-iterations "${ITERATIONS}"
  --agent.seed "${SEED}"
  --env.scene.num-envs "${NUM_ENVS}"
  --resume-optimizer False
  "${PASSTHROUGH[@]}")

printf 'Source: %s/%s\n' "${SOURCE_RUN}" "${CHECKPOINT}"
printf 'Training: %s environments, %s additional iterations\n' "${NUM_ENVS}" "${ITERATIONS}"
printf 'Seed: %s\n' "${SEED}"
if [ "${DRY_RUN}" = "1" ]; then
  printf 'Command:'
  printf ' %q' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

MANIFEST_DIR="${LOG_ROOT}/training_manifests"
MANIFEST_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
MANIFEST_PATH="${MANIFEST_DIR}/${MANIFEST_STAMP}_${RUN_NAME}.json"
python3 "${SCRIPT_DIR}/write_mjlab_training_manifest.py" \
  --output "${MANIFEST_PATH}" \
  --repository-root "${REPO_ROOT}" \
  --stage "${RUN_NAME_PREFIX}" \
  --task "${TASK_ID}" \
  --run-name "${RUN_NAME}" \
  --seed "${SEED}" \
  --num-envs "${NUM_ENVS}" \
  --iterations "${ITERATIONS}" \
  --source-checkpoint "${LOG_ROOT}/${SOURCE_RUN}/${CHECKPOINT}" \
  -- "${COMMAND[@]}"

export MJLAB_WITH_GPUS=1
export MJLAB_AUTO_RESUME=0
exec "${COMMAND[@]}"
