#!/usr/bin/env bash
# Historical Geoff installer. Dougie uses install.sh / ./dougie setup.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKIP_DOCKER_INSTALL=0
SKIP_BUILD=0

usage() {
  cat <<'EOF'
Usage: ./install.sh [options]

Set up the Unitree G1 ROS 2 MuJoCo development environment.

Options:
  --skip-docker-install  Require an existing Docker Compose v2 installation
  --skip-build           Prepare the host and repository without building
  -h, --help             Show this help
EOF
}

log() {
  printf '\n\033[1;34m==> %s\033[0m\n' "$*"
}

die() {
  printf '\nError: %s\n' "$*" >&2
  exit 1
}

for arg in "$@"; do
  case "${arg}" in
    --skip-docker-install) SKIP_DOCKER_INSTALL=1 ;;
    --skip-build) SKIP_BUILD=1 ;;
    -h|--help) usage; exit 0 ;;
    *) die "Unknown option: ${arg}. Run ./install.sh --help for usage." ;;
  esac
done

[ "$(uname -s)" = "Linux" ] || die "This project requires a Linux host."
[ "$(uname -m)" = "x86_64" ] || die "The bundled MuJoCo build currently supports x86_64 only."

if [ "${EUID}" -eq 0 ]; then
  SUDO=()
elif command -v sudo >/dev/null 2>&1; then
  SUDO=(sudo)
else
  die "sudo is required to install host packages."
fi

HOST_USER="${SUDO_USER:-${USER:-root}}"
HOST_UID="$(id -u "${HOST_USER}")"
HOST_GID="$(id -g "${HOST_USER}")"

install_host_packages() {
  [ -r /etc/os-release ] || die "Cannot identify this Linux distribution."
  # shellcheck disable=SC1091
  source /etc/os-release
  case "${ID:-}" in
    ubuntu|debian) ;;
    *) die "Automatic installation supports Ubuntu and Debian. Install Docker Compose v2, Git, curl, and xauth manually." ;;
  esac

  log "Installing host prerequisites"
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" apt-get install -y ca-certificates curl git gnupg xauth

  if docker compose version >/dev/null 2>&1; then
    return
  fi
  if [ "${SKIP_DOCKER_INSTALL}" -eq 1 ]; then
    die "Docker Compose v2 is not available."
  fi

  log "Installing Docker Engine and Compose v2"
  local keyring="/etc/apt/keyrings/docker.asc"
  local codename="${VERSION_CODENAME:-}"
  [ -n "${codename}" ] || die "VERSION_CODENAME is missing from /etc/os-release."
  "${SUDO[@]}" install -m 0755 -d /etc/apt/keyrings
  curl -fsSL "https://download.docker.com/linux/${ID}/gpg" | "${SUDO[@]}" tee "${keyring}" >/dev/null
  "${SUDO[@]}" chmod a+r "${keyring}"
  printf 'deb [arch=%s signed-by=%s] https://download.docker.com/linux/%s %s stable\n' \
    "$(dpkg --print-architecture)" "${keyring}" "${ID}" "${codename}" | \
    "${SUDO[@]}" tee /etc/apt/sources.list.d/docker.list >/dev/null
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" apt-get install -y \
    containerd.io docker-buildx-plugin docker-ce docker-ce-cli docker-compose-plugin
  "${SUDO[@]}" systemctl enable --now docker

  if [ "${HOST_USER}" != "root" ]; then
    "${SUDO[@]}" usermod -aG docker "${HOST_USER}"
  fi
}

install_host_packages

DOCKER=(docker)
if ! docker info >/dev/null 2>&1; then
  if "${SUDO[@]}" docker info >/dev/null 2>&1; then
    DOCKER=("${SUDO[@]}" docker)
  else
    die "Docker daemon is unavailable. Start Docker and rerun this installer."
  fi
fi
"${DOCKER[@]}" compose version >/dev/null 2>&1 || die "Docker Compose v2 is required."

cd "${SCRIPT_DIR}"
log "Initializing pinned Git submodules"
git submodule sync --recursive
git submodule update --init --recursive

if [ ! -f .env ]; then
  log "Creating local environment configuration"
  cat >.env <<EOF
UID=${HOST_UID}
GID=${HOST_GID}
ROS_DOMAIN_ID=42
DISPLAY=${DISPLAY:-:0}
XAUTHORITY=/tmp/.docker.xauth
DOCKER_RUNTIME=runc
EOF
else
  log "Keeping existing .env configuration"
fi

touch /tmp/.docker.xauth
chmod a+r /tmp/.docker.xauth
if [ -n "${DISPLAY:-}" ] && command -v xauth >/dev/null 2>&1; then
  xauth nlist "${DISPLAY}" 2>/dev/null | sed 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge - 2>/dev/null || true
fi

if [ "${SKIP_BUILD}" -eq 1 ]; then
  log "Repository setup complete (build skipped)"
  exit 0
fi

if [ ! -e /dev/dri ]; then
  die "/dev/dri is unavailable. This GUI simulation requires a Linux graphics device."
fi

log "Building the ROS 2 Humble container"
"${DOCKER[@]}" compose build unitree-g1-sim

log "Resolving ROS dependencies and building the workspace"
"${DOCKER[@]}" compose run --rm unitree-g1-sim bash -lc \
  '/workspace/scripts/setup_sources.sh && /workspace/scripts/build_ws.sh && /workspace/scripts/build_unitree_mujoco.sh'

log "Downloading and verifying the official Unitree G1 locomotion policy"
./scripts/download_unitree_rl_lab_policy.sh

printf '\nInstallation complete.\n'
printf 'Run: ./scripts/run_waypoint_navigation_demo.sh mujoco_scene:=scene_29dof_empty.xml\n'
if ! docker info >/dev/null 2>&1; then
  printf 'Docker currently requires sudo. Log out and back in before using the project scripts.\n'
fi
