#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"

mkdir -p "${WORKSPACE}/src" "/workspace/vendor"

if [ ! -f "${WORKSPACE}/unitree.repos" ]; then
  echo "Missing ${WORKSPACE}/unitree.repos"
  exit 1
fi

vcs import "${WORKSPACE}/src" < "${WORKSPACE}/unitree.repos"

if [ ! -d "/workspace/vendor/mjlab/.git" ]; then
  git clone https://github.com/mujocolab/mjlab.git "/workspace/vendor/mjlab"
fi

for ignore_dir in \
  "${WORKSPACE}/src/unitree_mujoco/example" \
  "${WORKSPACE}/src/unitree_mujoco/simulate" \
  "${WORKSPACE}/src/unitree_mujoco/simulate_python" \
  "${WORKSPACE}/src/unitree_sdk2"; do
  if [ -d "${ignore_dir}" ]; then
    touch "${ignore_dir}/COLCON_IGNORE"
  fi
done

rosdep update || true
rosdep install \
  --from-paths "${WORKSPACE}/src" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO:-humble}" \
  -y \
  --skip-keys "unitree_sdk2 ament_python"

echo "Sources are available in ${WORKSPACE}/src"
