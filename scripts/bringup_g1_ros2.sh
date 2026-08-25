#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-/workspace/ros2_ws}"
INSTALL_BASE="${COLCON_INSTALL_BASE:-${WORKSPACE}/install}"

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f "${INSTALL_BASE}/setup.bash" ]; then
  source "${INSTALL_BASE}/setup.bash"
fi

ARGS=("$@")

append_default_arg() {
  local name="$1"
  local value="$2"
  local arg
  for arg in "${ARGS[@]}"; do
    if [[ "${arg}" == "${name}:="* ]]; then
      return
    fi
  done
  ARGS+=("${name}:=${value}")
}

# The policy reset pose is only stable under immediate policy control. Keep
# these defaults here as well as in the launch file so all script entry points
# use the validated startup sequence unless the caller explicitly overrides it.
append_default_arg "mujoco_initial_keyframe" "policy_stand"
append_default_arg "startup_current_pose_hold_time" "0.0"
append_default_arg "startup_blend_time" "0.0"
append_default_arg "startup_hold_time" "0.0"
append_default_arg "startup_require_upright" "true"
append_default_arg "policy_action_ramp_time" "0.0"

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
