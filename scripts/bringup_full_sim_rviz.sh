#!/usr/bin/env bash
set -euo pipefail

docker compose run --rm unitree-g1-sim bash -lc 'exec /workspace/scripts/bringup_g1_ros2.sh run_rviz:=true "$@"' bash "$@"
