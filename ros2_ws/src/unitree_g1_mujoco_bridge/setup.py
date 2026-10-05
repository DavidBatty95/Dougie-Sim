from setuptools import setup

package_name = "unitree_g1_mujoco_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Workspace User",
    maintainer_email="user@example.com",
    description="Bridge Unitree G1 Mujoco state and commands into ROS 2 control topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pedestrian_start_bridge = unitree_g1_mujoco_bridge.pedestrian_start_bridge:main",
            "mujoco_goal_visualizer = unitree_g1_mujoco_bridge.mujoco_goal_visualizer:main",
            "dynamic_route_monitor = unitree_g1_mujoco_bridge.dynamic_route_monitor:main",
            "onboard_lidar_odometry = unitree_g1_mujoco_bridge.onboard_lidar_odometry:main",
            "g1_mujoco_bridge = unitree_g1_mujoco_bridge.g1_mujoco_bridge:main",
            "g1_cmd_vel_teleop = unitree_g1_mujoco_bridge.g1_cmd_vel_teleop:main",
            "g1_hold_pose_controller = unitree_g1_mujoco_bridge.g1_hold_pose_controller:main",
            "g1_policy_cmd_vel_controller = unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller:main",
            "g1_random_explorer = unitree_g1_mujoco_bridge.g1_random_explorer:main",
            "g1_waypoint_patrol = unitree_g1_mujoco_bridge.g1_waypoint_patrol:main",
            "g1_waypoint_explorer = unitree_g1_mujoco_bridge.g1_waypoint_explorer:main",
            "g1_point_goal = unitree_g1_mujoco_bridge.g1_point_goal:main",
            "g1_frontier_explorer = unitree_g1_mujoco_bridge.g1_frontier_explorer:main",
            "g1_slam_reset_monitor = unitree_g1_mujoco_bridge.g1_slam_reset_monitor:main",
            "g1_locomotion_diagnostics = unitree_g1_mujoco_bridge.g1_locomotion_diagnostics:main",
            "g1_clamp_button = unitree_g1_mujoco_bridge.g1_clamp_button:main",
            "g1_sim_sensors = unitree_g1_mujoco_bridge.g1_sim_sensors:main",
            "livox_to_scan = unitree_g1_mujoco_bridge.livox_to_scan:main",
            "lidar_person_detector = unitree_g1_mujoco_bridge.lidar_person_detector:main",
            "scan_dynamic_filter = unitree_g1_mujoco_bridge.scan_dynamic_filter:main",
            "dynamic_obstacle_tracker = unitree_g1_mujoco_bridge.dynamic_obstacle_tracker:main",
            "human_aware_motion_planner = unitree_g1_mujoco_bridge.human_aware_motion_planner:main",
            "human_collision_avoidance = unitree_g1_mujoco_bridge.human_collision_avoidance:main",
            "human_aware_run_logger = unitree_g1_mujoco_bridge.human_aware_run_logger:main",
            "g1_arm_command_compositor = unitree_g1_mujoco_bridge.arm_command_compositor:main",
            "g1_sim_manipulation_io = unitree_g1_mujoco_bridge.sim_manipulation_io:main",
        ],
    },
)
