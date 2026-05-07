#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODE="${1:-fast}"

case "${MODE}" in
  fast|precommit|smoke)
    ;;
  *)
    echo "Usage: $0 [fast|precommit|smoke]"
    exit 2
    ;;
esac

cd "${REPO_ROOT}"

echo "[test] docker compose config"
docker compose config >/tmp/unitree_g1_compose_config.yaml

echo "[test] shell syntax"
for script in scripts/*.sh; do
  bash -n "${script}"
done

if [ "${MODE}" = "precommit" ] || [ "${MODE}" = "smoke" ]; then
  echo "[test] ROS workspace build"
  docker compose run --rm unitree-g1-sim /workspace/scripts/build_ws.sh
fi

echo "[test] container workspace checks"
docker compose run --rm unitree-g1-sim \
  bash -lc "/workspace/scripts/test_workspace_inside.sh ${MODE}"

if [ "${MODE}" = "smoke" ]; then
  echo "[test] bounded full simulation topic smoke test"
  ./scripts/bringup_full_sim.sh run_rviz:=false run_clamp_button:=false mujoco_scene:=scene_29dof_room.xml >/tmp/g1_workspace_smoke_launch.log 2>&1 &
  launch_pid=$!
  cleanup() {
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
  }
  trap cleanup EXIT

  sleep "${SIM_SMOKE_STARTUP_SECONDS:-12}"
  docker compose run --rm unitree-g1-sim bash -lc '
    source /opt/ros/humble/setup.bash
    source /workspace/ros2_ws/install/setup.bash
    timeout 8s ros2 topic echo /joint_states --once >/tmp/joint_states_once.txt
    timeout 8s ros2 topic echo /livox/lidar --once >/tmp/livox_once.txt
    timeout 8s ros2 topic echo /camera/camera/color/image_raw --once >/tmp/color_once.txt
    test -s /tmp/joint_states_once.txt
    test -s /tmp/livox_once.txt
    test -s /tmp/color_once.txt
  '
  cleanup
  trap - EXIT
fi

echo "[test] All workspace tests passed (${MODE})"
