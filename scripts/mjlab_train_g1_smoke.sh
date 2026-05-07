#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/run_mjlab_docker.sh" train Mjlab-Velocity-Flat-Unitree-G1 \
  --agent.logger tensorboard \
  --agent.run-name "${MJLAB_RUN_NAME:-smoke_test}" \
  --agent.max-iterations "${MJLAB_MAX_ITERATIONS:-2}" \
  --env.scene.num-envs "${MJLAB_NUM_ENVS:-1}" \
  "$@"
