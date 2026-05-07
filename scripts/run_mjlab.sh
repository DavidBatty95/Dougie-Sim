#!/usr/bin/env bash
set -euo pipefail

MJLAB_DIR="${MJLAB_DIR:-/workspace/vendor/mjlab}"
MJLAB_PYTHON="${MJLAB_PYTHON:-python3}"

if [ ! -d "${MJLAB_DIR}" ]; then
  echo "MJ Lab is not available at ${MJLAB_DIR}. Run /workspace/scripts/setup_mjlab.sh first."
  exit 1
fi

cd "${MJLAB_DIR}"
exec uv run --python "${MJLAB_PYTHON}" "$@"
