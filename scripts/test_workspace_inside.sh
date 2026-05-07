#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"
REPO_ROOT="${REPO_ROOT:-/workspace}"
MODE="${1:-fast}"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi
set -u

cd "${REPO_ROOT}"

echo "[test] Python syntax for local ROS packages"
python3 -m compileall -q \
  "${WORKSPACE}/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge" \
  "${WORKSPACE}/src/unitree_g1_sim_bringup/launch"

echo "[test] XML/package/scene files parse"
python3 - <<'PY'
import pathlib
import xml.etree.ElementTree as ET

root = pathlib.Path('/workspace/ros2_ws')
paths = [
    root / 'src/unitree_g1_description/package.xml',
    root / 'src/unitree_g1_mujoco_bridge/package.xml',
    root / 'src/unitree_g1_sim_bringup/package.xml',
    root / 'src/unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf',
]
paths.extend(
    sorted((root / 'src/unitree_mujoco/unitree_robots/g1').glob('scene_29dof*.xml'))
)

missing = [str(path) for path in paths if not path.exists()]
if missing:
    raise SystemExit('Missing required files:\n' + '\n'.join(missing))

for path in paths:
    ET.parse(path)
    print(f'  parsed {path.relative_to(root)}')
PY

echo "[test] Robot description expands with xacro"
xacro "${WORKSPACE}/src/unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf" >/tmp/g1_robot_description.urdf

echo "[test] Required sim assets exist"
test -f "${WORKSPACE}/src/unitree_g1_description/meshes/pelvis.STL"
test -f "${WORKSPACE}/src/unitree_g1_sim_bringup/rviz/g1_sim.rviz"
test -f "${WORKSPACE}/config/cyclonedds.xml"

echo "[test] ROS package launch arguments are loadable"
if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix unitree_g1_sim_bringup >/dev/null 2>&1; then
  ros2 launch unitree_g1_sim_bringup g1_sim.launch.py --show-args >/tmp/g1_launch_args.txt
  grep -q "mujoco_scene" /tmp/g1_launch_args.txt
else
  echo "[test] Skipping launch argument check because workspace is not built"
fi

echo "[test] MJ Lab helper is usable when dependencies are installed"
if [ -d "/workspace/vendor/mjlab/.venv" ]; then
  /workspace/scripts/run_mjlab.sh list-envs >/tmp/mjlab_envs.txt || true
  grep -q "Mjlab-Velocity-Flat-Unitree-G1" /tmp/mjlab_envs.txt
else
  echo "[test] Skipping MJ Lab check because /workspace/vendor/mjlab/.venv is missing"
fi

if [ "${MODE}" = "smoke" ]; then
  echo "[test] GPU visibility, if NVIDIA runtime is available"
  if [ -d "/workspace/vendor/mjlab/.venv" ]; then
    /workspace/scripts/run_mjlab.sh python - <<'PY'
import torch

print(f'torch_cuda_available={torch.cuda.is_available()}')
print(f'torch_cuda_device_count={torch.cuda.device_count()}')
if torch.cuda.is_available():
    print(f'torch_cuda_device_name={torch.cuda.get_device_name(0)}')
PY
  else
    echo "[test] Skipping Torch CUDA check because /workspace/vendor/mjlab/.venv is missing"
  fi
fi

echo "[test] Workspace checks passed (${MODE})"
