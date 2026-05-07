#!/usr/bin/env bash
set -euo pipefail

MJLAB_DIR="${MJLAB_DIR:-/workspace/vendor/mjlab}"
MJLAB_REPO="${MJLAB_REPO:-https://github.com/mujocolab/mjlab.git}"
MJLAB_EXTRA="${MJLAB_EXTRA:-cu128}"
MJLAB_PYTHON="${MJLAB_PYTHON:-python3}"

mkdir -p "$(dirname "${MJLAB_DIR}")"

if [ ! -d "${MJLAB_DIR}/.git" ]; then
  git clone "${MJLAB_REPO}" "${MJLAB_DIR}"
fi

if ! command -v uv >/dev/null 2>&1; then
  python3 -m pip install --user --no-cache-dir uv
  export PATH="${HOME}/.local/bin:${PATH}"
fi

cd "${MJLAB_DIR}"

if [ "${MJLAB_SYNC:-1}" = "1" ]; then
  uv sync --python "${MJLAB_PYTHON}" --extra "${MJLAB_EXTRA}"

  uv run --python "${MJLAB_PYTHON}" python - <<'PY'
import importlib.metadata

print("mjlab", importlib.metadata.version("mjlab"))
PY
else
  echo "Skipping MJ Lab dependency sync because MJLAB_SYNC=0"
fi

echo "MJ Lab is available in ${MJLAB_DIR}"
