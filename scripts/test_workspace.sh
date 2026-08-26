#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODE="${1:-fast}"
if [ -n "${TEST_RUN_RVIZ:-}" ]; then
  RUN_RVIZ="${TEST_RUN_RVIZ}"
elif [ -n "${DISPLAY:-}" ] && [ -z "${CI:-}" ]; then
  RUN_RVIZ="true"
else
  RUN_RVIZ="false"
fi

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
for script in scripts/*.sh docker/scripts/*.sh; do
  bash -n "${script}"
done

if [ "${MODE}" = "smoke" ]; then
  echo "[test] ROS workspace build"
  docker compose run --rm \
    -e COLCON_BUILD_BASE=/workspace/ros2_ws/.verification/build \
    -e COLCON_INSTALL_BASE=/workspace/ros2_ws/.verification/install \
    -e COLCON_LOG_BASE=/workspace/ros2_ws/.verification/log \
    unitree-g1-sim /workspace/scripts/build_ws.sh
fi

echo "[test] container workspace checks"
if [ "${MODE}" = "precommit" ]; then
  docker compose run --rm \
    -e BENCHMARK_MINIMUM_SCORE="${BENCHMARK_MINIMUM_SCORE:-95.0}" \
    unitree-g1-sim \
    bash -lc "/workspace/scripts/test_workspace_inside.sh ci"
else
  if [ "${MODE}" = "smoke" ]; then
    docker compose run --rm \
      -e BENCHMARK_MINIMUM_SCORE="${BENCHMARK_MINIMUM_SCORE:-95.0}" \
      -e COLCON_BUILD_BASE=/workspace/ros2_ws/.verification/build \
      -e COLCON_INSTALL_BASE=/workspace/ros2_ws/.verification/install \
      -e COLCON_LOG_BASE=/workspace/ros2_ws/.verification/log \
      unitree-g1-sim \
      bash -lc "/workspace/scripts/test_workspace_inside.sh ${MODE}"
  else
    docker compose run --rm \
      -e BENCHMARK_MINIMUM_SCORE="${BENCHMARK_MINIMUM_SCORE:-95.0}" \
      unitree-g1-sim \
      bash -lc "/workspace/scripts/test_workspace_inside.sh ${MODE}"
  fi
fi

if [ "${MODE}" = "smoke" ]; then
  echo "[test] bounded full simulation topic smoke test"
  smoke_container="unitree-g1-workspace-smoke-$$"
  COLCON_INSTALL_BASE=/workspace/ros2_ws/.verification/install \
    UNITREE_G1_CONTAINER_NAME="${smoke_container}" \
    ./scripts/bringup_full_sim.sh run_rviz:="${RUN_RVIZ}" run_clamp_button:=false publish_camera:=true mujoco_scene:=scene_29dof_room.xml >/tmp/g1_workspace_smoke_launch.log 2>&1 &
  launch_pid=$!
  cleanup() {
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
    docker stop "${smoke_container}" >/dev/null 2>&1 || true
  }
  trap cleanup EXIT

  sleep "${SIM_SMOKE_STARTUP_SECONDS:-12}"
  docker compose run --rm unitree-g1-sim bash -lc '
    source /opt/ros/humble/setup.bash
    source /workspace/ros2_ws/.verification/install/setup.bash
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
