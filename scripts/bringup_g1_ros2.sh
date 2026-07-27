#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi

ARGS=("$@")
detector_arg_set=0
diagnostics_arg_set=0
point_goal_arg_set=0
run_logger_arg_set=0
for arg in "${ARGS[@]}"; do
  case "${arg}" in
    run_lidar_person_detector:=*)
      detector_arg_set=1
      ;;
    run_locomotion_diagnostics:=*)
      diagnostics_arg_set=1
      ;;
    run_point_goal:=*)
      point_goal_arg_set=1
      ;;
    run_human_aware_run_logger:=*)
      run_logger_arg_set=1
      ;;
  esac
done

if [ "${detector_arg_set}" = "0" ] && ! ros2 pkg executables unitree_g1_mujoco_bridge | grep -q '^unitree_g1_mujoco_bridge lidar_person_detector$'; then
  echo "Optional lidar_person_detector executable is not installed; disabling it for this launch. Run /workspace/scripts/build_ws.sh to enable it."
  ARGS+=("run_lidar_person_detector:=false")
fi

if [ "${diagnostics_arg_set}" = "0" ] && ! ros2 pkg executables unitree_g1_mujoco_bridge | grep -q '^unitree_g1_mujoco_bridge g1_locomotion_diagnostics$'; then
  echo "Optional g1_locomotion_diagnostics executable is not installed; disabling it for this launch. Run /workspace/scripts/build_ws.sh to enable it."
  ARGS+=("run_locomotion_diagnostics:=false")
fi

if [ "${point_goal_arg_set}" = "0" ] && ! ros2 pkg executables unitree_g1_mujoco_bridge | grep -q '^unitree_g1_mujoco_bridge g1_point_goal$'; then
  echo "Optional g1_point_goal executable is not installed; disabling it for this launch. Run /workspace/scripts/build_ws.sh to enable it."
  ARGS+=("run_point_goal:=false")
fi

if [ "${run_logger_arg_set}" = "0" ] && ! ros2 pkg executables unitree_g1_mujoco_bridge | grep -q '^unitree_g1_mujoco_bridge human_aware_run_logger$'; then
  echo "Optional human_aware_run_logger executable is not installed; disabling it for this launch. Run /workspace/scripts/build_ws.sh to enable it."
  ARGS+=("run_human_aware_run_logger:=false")
fi

ros2 launch unitree_g1_sim_bringup g1_sim.launch.py "${ARGS[@]}"
