"""Mobile simulation gate for sequential navigation, pick, carry, and place."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    simulation = _include(
        "unitree_g1_sim_bringup",
        "g1_sim.launch.py",
        {
            "mujoco_scene": "scene_29dof_transport.xml",
            "mujoco_initial_keyframe": "transport_stand",
            "mujoco_pedestrians_enable": "false",
            "run_arm_compositor": "true",
            "run_sim_manipulation_io": "true",
            "sim_manipulation_frame_id": "odom",
            "run_policy_controller": "true",
            "run_hold_controller": "false",
            "arm_compositor_goal_tolerance": "0.15",
            "run_sensors": "true",
            "run_slam": "true",
            "run_nav2": "true",
            "run_base_approach": "true",
            "run_dynamic_obstacle_tracker": "true",
            "run_human_aware_motion_planner": "true",
            "run_human_collision_avoidance": "true",
            "run_human_aware_run_logger": "false",
            "run_locomotion_diagnostics": "false",
            "run_slam_reset_monitor": "false",
            "run_waypoint_patrol": "false",
            "run_waypoint_explorer": "false",
            "run_point_goal": "false",
            "run_frontier_explorer": "false",
            "run_rviz": "false",
        },
    )
    move_group = _include("g1_moveit_config", "move_group.launch.py")
    manipulation = _include("g1_manipulation", "manipulation.launch.py")
    transport = _include("g1_task_orchestration", "transport_server.launch.py")

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting simulation-only mobile transport gate. The transport server "
                    "owns mission sequencing; do not run another Nav2 goal source."
                )
            ),
            simulation,
            TimerAction(period=6.0, actions=[move_group]),
            TimerAction(period=10.0, actions=[manipulation]),
            TimerAction(period=14.0, actions=[transport]),
        ]
    )
