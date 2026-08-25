#!/usr/bin/env bash
set -euo pipefail

readonly GROVE_G1_COMMIT="97bfb9ba13569a593b5e3ecfc5ed1d4208b2b681"
readonly GROVE_G1_URL="https://github.com/Adyansh04/grove-g1.git"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
MANIFEST="${REPO_ROOT}/integrations/grove_g1/grove_g1.repos"
VENDOR_ROOT="${REPO_ROOT}/vendor"
CHECKOUT="${VENDOR_ROOT}/grove-g1"

fail() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

for command in git vcs; do
  command -v "${command}" >/dev/null 2>&1 || fail "required command not found: ${command}"
done

printf '%s\n' \
  "Grove-G1 is an optional ROS 2 Jazzy standalone stack." \
  "It is not imported into or built by this repository's ROS 2 Humble colcon workspace."

[[ -f "${MANIFEST}" ]] || fail "manifest not found: ${MANIFEST}"

[[ ! -L "${CHECKOUT}" ]] || fail "${CHECKOUT} is a symlink; refusing to operate outside vendor"

if [[ ! -e "${CHECKOUT}" ]]; then
  mkdir -p "${VENDOR_ROOT}"
  vcs import --recursive --input "${MANIFEST}" "${VENDOR_ROOT}"
elif [[ ! -d "${CHECKOUT}/.git" ]]; then
  fail "${CHECKOUT} exists but is not a Git checkout; refusing to replace it"
else
  origin_url="$(git -C "${CHECKOUT}" remote get-url origin 2>/dev/null)" || \
    fail "${CHECKOUT} has no origin remote"
  [[ "${origin_url}" == "${GROVE_G1_URL}" ]] || \
    fail "unexpected origin ${origin_url}; expected ${GROVE_G1_URL}"

  [[ -z "$(git -C "${CHECKOUT}" status --porcelain --untracked-files=all)" ]] || \
    fail "${CHECKOUT} is dirty; commit, stash, or remove local changes before updating"

  current_commit="$(git -C "${CHECKOUT}" rev-parse --verify HEAD)"
  if [[ "${current_commit}" != "${GROVE_G1_COMMIT}" ]]; then
    git -C "${CHECKOUT}" fetch --no-tags origin "${GROVE_G1_COMMIT}"
    git -C "${CHECKOUT}" merge-base --is-ancestor "${current_commit}" "${GROVE_G1_COMMIT}" || \
      fail "checkout is ahead of or diverged from the pinned commit; refusing to overwrite it"
    git -C "${CHECKOUT}" checkout --detach "${GROVE_G1_COMMIT}"
  fi
fi

[[ -d "${CHECKOUT}/.git" ]] || fail "vcstool did not create ${CHECKOUT}"
actual_url="$(git -C "${CHECKOUT}" remote get-url origin 2>/dev/null)" || \
  fail "imported checkout has no origin remote"
[[ "${actual_url}" == "${GROVE_G1_URL}" ]] || \
  fail "imported checkout has unexpected origin ${actual_url}"
actual_commit="$(git -C "${CHECKOUT}" rev-parse --verify HEAD)"
[[ "${actual_commit}" == "${GROVE_G1_COMMIT}" ]] || \
  fail "expected ${GROVE_G1_COMMIT}, found ${actual_commit}"
[[ -z "$(git -C "${CHECKOUT}" status --porcelain --untracked-files=all)" ]] || \
  fail "checkout became dirty during setup"

printf 'Grove-G1 is ready at %s (commit %s).\n' "${CHECKOUT}" "${actual_commit}"
printf '%s\n' "Use its own Jazzy container workflow; do not add it to ros2_ws/src."
