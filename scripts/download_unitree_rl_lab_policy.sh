#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
POLICY_DIR="${REPO_ROOT}/vendor/unitree_rl_lab/policies/g1_29dof_velocity_v0"
POLICY_PATH="${POLICY_DIR}/policy.onnx"
POLICY_URL="https://raw.githubusercontent.com/unitreerobotics/unitree_rl_lab/main/deploy/robots/g1_29dof/config/policy/velocity/v0/exported/policy.onnx"
POLICY_SHA256="610c27e463a8f666aa50a06346678c00b4df3859f10b54bcc1f817c28251406f"

mkdir -p "${POLICY_DIR}"
if [ ! -f "${POLICY_PATH}" ]; then
  curl --fail --location --retry 3 --output "${POLICY_PATH}.tmp" "${POLICY_URL}"
  mv "${POLICY_PATH}.tmp" "${POLICY_PATH}"
fi

printf '%s  %s\n' "${POLICY_SHA256}" "${POLICY_PATH}" | sha256sum --check --status
printf 'Verified Unitree RL Lab G1 policy: %s\n' "${POLICY_PATH}"
