from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    run_sim = LaunchConfiguration("run_sim")
    publish_description = LaunchConfiguration("publish_description")
    run_bridge = LaunchConfiguration("run_bridge")
    run_hold_controller = LaunchConfiguration("run_hold_controller")
    run_policy_controller = LaunchConfiguration("run_policy_controller")
    run_clamp_button = LaunchConfiguration("run_clamp_button")
    run_sensors = LaunchConfiguration("run_sensors")
    run_slam = LaunchConfiguration("run_slam")
    run_nav2 = LaunchConfiguration("run_nav2")
    run_lidar_person_detector = LaunchConfiguration("run_lidar_person_detector")
    run_dynamic_obstacle_tracker = LaunchConfiguration("run_dynamic_obstacle_tracker")
    run_human_aware_motion_planner = LaunchConfiguration("run_human_aware_motion_planner")
    run_human_collision_avoidance = LaunchConfiguration("run_human_collision_avoidance")
    run_human_aware_run_logger = LaunchConfiguration("run_human_aware_run_logger")
    human_aware_run_log_path = LaunchConfiguration("human_aware_run_log_path")
    run_random_explorer = LaunchConfiguration("run_random_explorer")
    run_waypoint_patrol = LaunchConfiguration("run_waypoint_patrol")
    run_waypoint_explorer = LaunchConfiguration("run_waypoint_explorer")
    run_point_goal = LaunchConfiguration("run_point_goal")
    run_frontier_explorer = LaunchConfiguration("run_frontier_explorer")
    run_locomotion_diagnostics = LaunchConfiguration("run_locomotion_diagnostics")
    run_slam_reset_monitor = LaunchConfiguration("run_slam_reset_monitor")
    run_rviz = LaunchConfiguration("run_rviz")
    lock_waist = LaunchConfiguration("lock_waist")
    waist_lock_kp = LaunchConfiguration("waist_lock_kp")
    waist_lock_kd = LaunchConfiguration("waist_lock_kd")
    balance_waist = LaunchConfiguration("balance_waist")
    waist_balance_p = LaunchConfiguration("waist_balance_p")
    waist_balance_i = LaunchConfiguration("waist_balance_i")
    waist_balance_d = LaunchConfiguration("waist_balance_d")
    waist_balance_max = LaunchConfiguration("waist_balance_max")
    fall_damping_enable = LaunchConfiguration("fall_damping_enable")
    fall_damping_enter_angle = LaunchConfiguration("fall_damping_enter_angle")
    fall_damping_exit_angle = LaunchConfiguration("fall_damping_exit_angle")
    fall_damping_kd = LaunchConfiguration("fall_damping_kd")
    fall_damping_waist_kd = LaunchConfiguration("fall_damping_waist_kd")
    fall_damping_reset_grace_time = LaunchConfiguration("fall_damping_reset_grace_time")
    startup_fall_damping_grace_time = LaunchConfiguration("startup_fall_damping_grace_time")
    base_height_offset = LaunchConfiguration("base_height_offset")
    policy_path = LaunchConfiguration("policy_path")
    policy_obs_dim = LaunchConfiguration("policy_obs_dim")
    policy_action_dim = LaunchConfiguration("policy_action_dim")
    policy_command_height = LaunchConfiguration("policy_command_height")
    policy_rate_hz = LaunchConfiguration("policy_rate_hz")
    raw_cmd_vel_topic = LaunchConfiguration("raw_cmd_vel_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    cmd_vel_timeout = LaunchConfiguration("cmd_vel_timeout")
    max_lin_vel_x = LaunchConfiguration("max_lin_vel_x")
    max_lin_vel_y = LaunchConfiguration("max_lin_vel_y")
    max_ang_vel_z = LaunchConfiguration("max_ang_vel_z")
    startup_current_pose_hold_time = LaunchConfiguration("startup_current_pose_hold_time")
    startup_blend_time = LaunchConfiguration("startup_blend_time")
    startup_hold_time = LaunchConfiguration("startup_hold_time")
    startup_require_upright = LaunchConfiguration("startup_require_upright")
    startup_max_recovery_time = LaunchConfiguration("startup_max_recovery_time")
    policy_action_ramp_time = LaunchConfiguration("policy_action_ramp_time")
    mujoco_startup_hold_enable = LaunchConfiguration("mujoco_startup_hold_enable")
    mujoco_startup_hold_timeout = LaunchConfiguration("mujoco_startup_hold_timeout")
    mujoco_reset_on_controller_start = LaunchConfiguration("mujoco_reset_on_controller_start")
    robot_description_file = LaunchConfiguration("robot_description_file")
    rviz_config = LaunchConfiguration("rviz_config")
    mujoco_scene = LaunchConfiguration("mujoco_scene")
    mujoco_pedestrians_enable = LaunchConfiguration("mujoco_pedestrians_enable")
    mujoco_pedestrians_count = LaunchConfiguration("mujoco_pedestrians_count")
    lidar_rate_hz = LaunchConfiguration("lidar_rate_hz")
    livox_scan_points = LaunchConfiguration("livox_scan_points")
    livox_horizontal_min_deg = LaunchConfiguration("livox_horizontal_min_deg")
    livox_horizontal_max_deg = LaunchConfiguration("livox_horizontal_max_deg")
    livox_vertical_min_deg = LaunchConfiguration("livox_vertical_min_deg")
    livox_vertical_max_deg = LaunchConfiguration("livox_vertical_max_deg")
    use_mujoco_raycast_lidar = LaunchConfiguration("use_mujoco_raycast_lidar")
    mujoco_lidar_dead_zone_radius = LaunchConfiguration("mujoco_lidar_dead_zone_radius")
    mujoco_lidar_dead_zone_height = LaunchConfiguration("mujoco_lidar_dead_zone_height")
    mujoco_lidar_socket = LaunchConfiguration("mujoco_lidar_socket")
    publish_lidar_tf = LaunchConfiguration("publish_lidar_tf")
    publish_camera = LaunchConfiguration("publish_camera")
    camera_frame = LaunchConfiguration("camera_frame")
    depth_frame = LaunchConfiguration("depth_frame")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_rate_hz = LaunchConfiguration("camera_rate_hz")
    use_mujoco_raycast_camera = LaunchConfiguration("use_mujoco_raycast_camera")
    mujoco_camera_socket = LaunchConfiguration("mujoco_camera_socket")
    publish_depth_pointcloud = LaunchConfiguration("publish_depth_pointcloud")
    use_mujoco_lidar_pose = LaunchConfiguration("use_mujoco_lidar_pose")
    use_cmd_vel_lidar_fallback = LaunchConfiguration("use_cmd_vel_lidar_fallback")
    require_exact_lidar_pose = LaunchConfiguration("require_exact_lidar_pose")
    lidar_frame = LaunchConfiguration("lidar_frame")
    lidar_output_frame = LaunchConfiguration("lidar_output_frame")
    livox_base_offset_x = LaunchConfiguration("livox_base_offset_x")
    livox_base_offset_y = LaunchConfiguration("livox_base_offset_y")
    livox_base_offset_z = LaunchConfiguration("livox_base_offset_z")
    pelvis_to_livox_z = LaunchConfiguration("pelvis_to_livox_z")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    lidar_person_model_path = LaunchConfiguration("lidar_person_model_path")
    tracker_use_object_motion_detection = LaunchConfiguration("tracker_use_object_motion_detection")
    tracker_immediate_person_risk = LaunchConfiguration("tracker_immediate_person_risk")
    tracker_publish_candidate_predictions = LaunchConfiguration("tracker_publish_candidate_predictions")
    tracker_candidate_person_score_threshold = LaunchConfiguration("tracker_candidate_person_score_threshold")
    tracker_confirmed_person_score_threshold = LaunchConfiguration("tracker_confirmed_person_score_threshold")
    tracker_confirm_confidence_threshold = LaunchConfiguration("tracker_confirm_confidence_threshold")
    tracker_min_track_age = LaunchConfiguration("tracker_min_track_age")
    tracker_min_dynamic_observations = LaunchConfiguration("tracker_min_dynamic_observations")
    tracker_min_person_observations = LaunchConfiguration("tracker_min_person_observations")
    avoidance_danger_radius = LaunchConfiguration("avoidance_danger_radius")
    avoidance_critical_radius = LaunchConfiguration("avoidance_critical_radius")
    avoidance_corridor_width = LaunchConfiguration("avoidance_corridor_width")
    avoidance_corridor_critical_width = LaunchConfiguration("avoidance_corridor_critical_width")
    avoidance_max_reverse_linear = LaunchConfiguration("avoidance_max_reverse_linear")
    avoidance_max_lateral_linear = LaunchConfiguration("avoidance_max_lateral_linear")
    avoidance_max_avoid_angular = LaunchConfiguration("avoidance_max_avoid_angular")
    avoidance_people_repulsion_gain = LaunchConfiguration("avoidance_people_repulsion_gain")
    human_aware_max_forward_near_human = LaunchConfiguration("human_aware_max_forward_near_human")
    human_aware_max_lateral_near_human = LaunchConfiguration("human_aware_max_lateral_near_human")
    human_aware_max_reverse_velocity = LaunchConfiguration("human_aware_max_reverse_velocity")
    human_aware_max_angular_near_human = LaunchConfiguration("human_aware_max_angular_near_human")
    human_aware_comfort_radius = LaunchConfiguration("human_aware_comfort_radius")
    human_aware_front_arc_angle_deg = LaunchConfiguration("human_aware_front_arc_angle_deg")
    human_aware_front_arc_distance = LaunchConfiguration("human_aware_front_arc_distance")
    human_aware_front_arc_weight = LaunchConfiguration("human_aware_front_arc_weight")
    human_aware_w_personal_space = LaunchConfiguration("human_aware_w_personal_space")
    human_aware_w_ttc = LaunchConfiguration("human_aware_w_ttc")
    human_aware_w_pass_front = LaunchConfiguration("human_aware_w_pass_front")
    human_aware_w_smooth = LaunchConfiguration("human_aware_w_smooth")
    point_goal_x = LaunchConfiguration("point_goal_x")
    point_goal_y = LaunchConfiguration("point_goal_y")
    point_goal_yaw = LaunchConfiguration("point_goal_yaw")

    bridge_node = Node(
        condition=IfCondition(run_bridge),
        package="unitree_g1_mujoco_bridge",
        executable="g1_mujoco_bridge",
        name="g1_mujoco_bridge",
        output="screen",
        parameters=[
            {
                "use_mjlab_body_gains": True,
                "lock_waist": lock_waist,
                "waist_lock_kp": waist_lock_kp,
                "waist_lock_kd": waist_lock_kd,
                "balance_waist": balance_waist,
                "waist_balance_p": waist_balance_p,
                "waist_balance_i": waist_balance_i,
                "waist_balance_d": waist_balance_d,
                "waist_balance_max": waist_balance_max,
                "fall_damping_enable": fall_damping_enable,
                "fall_damping_enter_angle": fall_damping_enter_angle,
                "fall_damping_exit_angle": fall_damping_exit_angle,
                "fall_damping_kd": fall_damping_kd,
                "fall_damping_waist_kd": fall_damping_waist_kd,
                "fall_damping_reset_grace_time": fall_damping_reset_grace_time,
                "startup_fall_damping_grace_time": startup_fall_damping_grace_time,
                "base_height_offset": base_height_offset,
            }
        ],
    )
    hold_node = Node(
        condition=IfCondition(run_hold_controller),
        package="unitree_g1_mujoco_bridge",
        executable="g1_hold_pose_controller",
        name="g1_hold_pose_controller",
        output="screen",
    )
    policy_node = Node(
        condition=IfCondition(run_policy_controller),
        package="unitree_g1_mujoco_bridge",
        executable="g1_policy_cmd_vel_controller",
        name="g1_policy_cmd_vel_controller",
        output="screen",
        parameters=[
            {
                "policy_path": policy_path,
                "policy_obs_dim": policy_obs_dim,
                "policy_action_dim": policy_action_dim,
                "policy_command_height": policy_command_height,
                "rate_hz": policy_rate_hz,
                "cmd_vel_topic": cmd_vel_topic,
                "odom_topic": "/odom",
                "cmd_vel_timeout": cmd_vel_timeout,
                "max_lin_vel_x": max_lin_vel_x,
                "max_lin_vel_y": max_lin_vel_y,
                "max_ang_vel_z": max_ang_vel_z,
                "cmd_accel_x": 0.22,
                "cmd_accel_y": 0.16,
                "cmd_accel_yaw": 0.35,
                "action_clip": 4.0,
                "action_rate_limit": 8.0,
                "startup_current_pose_hold_time": startup_current_pose_hold_time,
                "startup_blend_time": startup_blend_time,
                "startup_hold_time": startup_hold_time,
                "startup_require_upright": startup_require_upright,
                "startup_upright_tilt_threshold": 0.80,
                "startup_max_recovery_time": startup_max_recovery_time,
                "policy_action_ramp_time": policy_action_ramp_time,
                "tilt_guard_enable": True,
                "tilt_guard_enter": 0.90,
                "tilt_guard_exit": 0.55,
                "tilt_guard_hold_time": 1.0,
                "pre_lowstate_stand_enable": False,
            }
        ],
    )
    clamp_button_node = Node(
        condition=IfCondition(run_clamp_button),
        package="unitree_g1_mujoco_bridge",
        executable="g1_clamp_button",
        name="g1_clamp_button",
        output="screen",
    )
    sensors_node = Node(
        condition=IfCondition(run_sensors),
        package="unitree_g1_mujoco_bridge",
        executable="g1_sim_sensors",
        name="g1_sim_sensors",
        output="screen",
        parameters=[
            {
                "mujoco_scene": mujoco_scene,
                "lidar_rate_hz": lidar_rate_hz,
                "livox_scan_points": livox_scan_points,
                "livox_horizontal_min_deg": livox_horizontal_min_deg,
                "livox_horizontal_max_deg": livox_horizontal_max_deg,
                "livox_vertical_min_deg": livox_vertical_min_deg,
                "livox_vertical_max_deg": livox_vertical_max_deg,
                "use_mujoco_raycast_lidar": use_mujoco_raycast_lidar,
                "mujoco_lidar_socket": mujoco_lidar_socket,
                "mujoco_pedestrian_socket": "/tmp/unitree_g1_mujoco_pedestrians.sock",
                "publish_lidar_tf": publish_lidar_tf,
                "publish_camera": publish_camera,
                "camera_frame": camera_frame,
                "depth_frame": depth_frame,
                "camera_width": camera_width,
                "camera_height": camera_height,
                "camera_rate_hz": camera_rate_hz,
                "use_mujoco_raycast_camera": use_mujoco_raycast_camera,
                "mujoco_camera_socket": mujoco_camera_socket,
                "publish_depth_pointcloud": publish_depth_pointcloud,
                "use_mujoco_lidar_pose": use_mujoco_lidar_pose,
                "use_cmd_vel_lidar_fallback": use_cmd_vel_lidar_fallback,
                "require_exact_lidar_pose": require_exact_lidar_pose,
                "suppress_lidar_when_fallen": True,
                "fallen_tilt_threshold": fall_damping_enter_angle,
                "upright_tilt_threshold": fall_damping_exit_angle,
                "lidar_resume_delay": 2.0,
                "lidar_frame": lidar_frame,
                "lidar_output_frame": lidar_output_frame,
                "livox_base_offset_x": livox_base_offset_x,
                "livox_base_offset_y": livox_base_offset_y,
                "livox_base_offset_z": livox_base_offset_z,
                "pelvis_to_livox_z": pelvis_to_livox_z,
            }
        ],
    )
    livox_to_scan_node = Node(
        condition=IfCondition(run_slam),
        package="unitree_g1_mujoco_bridge",
        executable="livox_to_scan",
        name="livox_to_scan",
        output="screen",
        parameters=[
            {
                "cloud_topic": "/livox/lidar",
                "scan_topic": "/scan_raw",
                "range_min": 0.15,
                "range_max": 25.0,
                "angle_min": -3.141592653589793,
                "angle_max": 3.141592653589793,
                "angle_increment": 0.008726646259971648,
                "min_height": -1.10,
                "max_height": 0.65,
                "min_floor_range": 1.0,
                "fill_empty_bins": True,
                "max_fill_gap_deg": 6.0,
            }
        ],
    )
    slam_node = Node(
        condition=IfCondition(run_slam),
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file],
    )
    scan_filter_node = Node(
        condition=IfCondition(run_slam),
        package="unitree_g1_mujoco_bridge",
        executable="scan_dynamic_filter",
        name="scan_dynamic_filter",
        output="screen",
        parameters=[
            {
                "input_scan_topic": "/scan_raw",
                "output_scan_topic": "/scan",
                "tracked_cloud_topic": "/tracked_dynamic_obstacles",
                "filter_radius": 0.75,
                "range_padding": 0.45,
                "cloud_timeout": 0.8,
            }
        ],
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
        ),
        condition=IfCondition(run_nav2),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": nav2_params_file,
        }.items(),
    )
    dynamic_obstacle_node = Node(
        condition=IfCondition(run_dynamic_obstacle_tracker),
        package="unitree_g1_mujoco_bridge",
        executable="dynamic_obstacle_tracker",
        name="dynamic_obstacle_tracker",
        output="screen",
        parameters=[
            {
                "cloud_topic": "/livox/lidar_world",
                "scan_topic": "/scan",
                "use_scan": False,
                "target_frame": "world",
                "robot_frame": "base_footprint",
                "marker_topic": "/dynamic_obstacles",
                "prediction_cloud_topic": "/predicted_dynamic_obstacles",
                "tracked_cloud_topic": "/tracked_dynamic_obstacles",
                "tentative_prediction_cloud_topic": "/tentative_dynamic_obstacles",
                "debug_tracks_topic": "/dynamic_obstacle_tracker/debug_tracks",
                "safety_grid_topic": "/human_safety_grid",
                "use_ml_person_detections": True,
                "person_detection_topic": "/lidar_person_detections",
                "use_mujoco_people_detections": True,
                "mujoco_people_authoritative": True,
                "mujoco_people_topic": "/mujoco/pedestrians",
                "mujoco_people_timeout": 0.6,
                "mujoco_person_height": 1.55,
                "person_detection_min_confidence": 0.50,
                "person_detection_confirm_confidence": 0.55,
                "person_detection_score_boost": 0.55,
                "use_object_motion_detection": tracker_use_object_motion_detection,
                "object_score_threshold": 0.66,
                "object_min_points": 8,
                "object_max_points": 220,
                "object_min_radius": 0.10,
                "object_max_radius": 0.70,
                "object_max_extent": 1.15,
                "object_max_height": 2.15,
                "object_min_height": 0.35,
                "object_min_cells": 3,
                "object_near_range": 4.0,
                "object_near_min_points": 10,
                "require_person_shape_for_object_detection": True,
                "local_tracking_radius": 6.0,
                "publish_safety_grid": False,
                "min_height": 0.05,
                "min_range": 0.25,
                "max_range": 12.0,
                "ground_rejection_height": -0.03,
                "min_dynamic_speed": 0.08,
                "min_confirm_displacement": 0.18,
                "require_motion_to_confirm": True,
                "min_track_age": tracker_min_track_age,
                "min_dynamic_observations": tracker_min_dynamic_observations,
                "min_person_observations": tracker_min_person_observations,
                "min_static_person_observations": 4,
                "cluster_distance": 0.50,
                "cluster_z_distance": 0.90,
                "person_cluster_distance": 0.28,
                "min_cluster_points": 6,
                "max_cluster_points": 220,
                "enable_point_noise_filter": True,
                "noise_filter_radius": 0.38,
                "min_point_neighbors": 2,
                "min_person_cluster_points": 8,
                "require_person_vertical_structure": True,
                "min_person_vertical_layers": 2,
                "min_vertical_layers": 3,
                "person_vertical_layer_height": 0.30,
                "reject_livox_pattern_artifacts": True,
                "livox_artifact_max_height": 0.28,
                "livox_artifact_max_layers": 1,
                "livox_artifact_min_extent": 0.55,
                "livox_artifact_max_points": 18,
                "max_person_aspect_ratio": 3.2,
                "candidate_person_score_threshold": tracker_candidate_person_score_threshold,
                "confirmed_person_score_threshold": tracker_confirmed_person_score_threshold,
                "max_person_radius": 0.58,
                "max_person_extent": 1.05,
                "min_person_height": 0.45,
                "max_person_height": 2.15,
                "min_person_centroid_height": 0.30,
                "max_person_centroid_height": 1.65,
                "person_fragment_merge_distance": 0.75,
                "max_person_acceleration": 2.8,
                "max_human_speed": 2.5,
                "max_human_accel": 3.5,
                "max_confirmed_measurement_jump": 0.85,
                "static_cell_observations": 4,
                "coast_confirmed_tracks": True,
                "max_coast_time": 2.0,
                "max_lost_persistence": 4.0,
                "max_visible_confirmed_tracks": 8,
                "confirm_confidence_threshold": tracker_confirm_confidence_threshold,
                "drop_confidence_threshold": 0.08,
                "confidence_decay_per_second": 0.08,
                "immediate_person_risk": tracker_immediate_person_risk,
                "candidate_person_min_age": 2,
                "candidate_person_min_confidence": 0.45,
                "candidate_person_robot_radius": 2.0,
                "candidate_person_risk_radius": 0.80,
                "robot_self_filter_radius": 0.60,
                "personal_space_radius": 1.25,
                "robot_avoidance_radius": 2.00,
                "robot_risk_inflation_radius": 1.90,
                "prediction_horizon": 3.0,
                "prediction_dt": 0.35,
                "publish_candidate_predictions": tracker_publish_candidate_predictions,
                "publish_tentative_predictions": False,
                "tentative_prediction_radius_scale": 0.45,
                "prediction_min_confidence": 0.50,
                "prediction_min_person_score": 0.52,
                "prediction_min_dynamic_observations": 1,
                "max_prediction_coast_time": 2.0,
                "min_prediction_speed": 0.10,
                # Livox-style scans are sparse; tune these with bags before real hardware use.
                "point_accumulation_time": 0.20,
                "accumulation_motion_compensation": True,
                "max_accumulation_robot_yaw_rate": 0.8,
                "max_accumulated_points": 5000,
                "voxel_leaf_size": 0.05,
                "use_livox_tag_filter": True,
                "max_noise_tag_fraction": 0.35,
                "range_adaptive_neighbor_filter": True,
                "near_noise_radius": 0.15,
                "far_noise_radius": 0.35,
                "min_neighbors_near": 3,
                "min_neighbors_far": 4,
                "range_adaptive_cluster_distance": True,
                "cluster_epsilon_base": 0.16,
                "cluster_epsilon_per_meter": 0.025,
                "cluster_epsilon_min": 0.14,
                "cluster_epsilon_max": 0.38,
                "min_cluster_points_by_range": True,
                "max_static_cell_fraction_new_track": 0.35,
                "tentative_confirm_min_hits": 2,
                "tentative_confirm_window": 5,
                "min_confirm_age": 0.20,
                "confirmed_keep_missed_time": 2.00,
                "tentative_delete_missed_time": 0.25,
                "stale_track_timeout": 1.0,
                "mahalanobis_gate": 7.8,
                "debug_markers_enabled": True,
                "marker_lifetime": 60.0,
                "candidate_marker_lifetime": 15.0,
                "marker_display_hold_time": 3.0,
                "safety_grid_size": 10.0,
                "safety_grid_resolution": 0.25,
                "safety_grid_danger_radius": 0.90,
                "safety_grid_caution_radius": 1.80,
            }
        ],
    )
    lidar_person_detector_node = Node(
        condition=IfCondition(run_lidar_person_detector),
        package="unitree_g1_mujoco_bridge",
        executable="lidar_person_detector",
        name="lidar_person_detector",
        output="screen",
        parameters=[
            {
                "cloud_topic": "/livox/lidar_world",
                "detection_topic": "/lidar_person_detections",
                "target_frame": "world",
                "robot_frame": "base_footprint",
                "model_path": lidar_person_model_path,
                "grid_size": 10.0,
                "grid_resolution": 0.10,
                "min_height": 0.05,
                "max_height": 2.20,
                "min_confidence": 0.45,
                "default_radius": 0.45,
                "max_detections": 6,
                "fallback_detection": True,
                "fallback_cluster_distance": 0.32,
                "fallback_min_points": 4,
                "fallback_max_points": 260,
                "fallback_min_height": 0.25,
                "fallback_max_height": 2.20,
                "fallback_max_extent": 1.20,
                "fallback_min_confidence": 0.50,
                "fallback_max_detections": 8,
                "fallback_nms_radius": 0.75,
                "robot_self_filter_radius": 0.60,
                "point_accumulation_time": 0.50,
                "max_accumulated_points": 9000,
                "processing_rate_hz": 5.0,
                "downsample_resolution": 0.14,
                "max_processed_points": 2500,
            }
        ],
    )
    human_aware_motion_planner_node = Node(
        condition=IfCondition(run_human_aware_motion_planner),
        package="unitree_g1_mujoco_bridge",
        executable="human_aware_motion_planner",
        name="human_aware_motion_planner",
        output="screen",
        parameters=[
            {
                "input_cmd_vel_topic": raw_cmd_vel_topic,
                "output_cmd_vel_topic": "/cmd_vel_human_aware",
                "tracked_cloud_topic": "/tracked_dynamic_obstacles",
                "predicted_cloud_topic": "/predicted_dynamic_obstacles",
                "tentative_cloud_topic": "/tentative_dynamic_obstacles",
                "debug_tracks_topic": "/dynamic_obstacle_tracker/debug_tracks",
                "scan_topic": "/scan",
                "target_frame": "world",
                "robot_frame": "base_footprint",
                "rate_hz": 12.0,
                "local_safe_space_enabled": False,
                "prediction_horizon": 2.5,
                "simulation_dt": 0.2,
                "max_person_cloud_age": 1.4,
                "robot_radius": 0.38,
                "human_body_radius": 0.35,
                "hard_safety_margin": 0.25,
                "comfort_radius": human_aware_comfort_radius,
                "front_comfort_scale": 1.7,
                "front_arc_risk_enabled": True,
                "front_arc_angle_deg": human_aware_front_arc_angle_deg,
                "front_arc_distance": human_aware_front_arc_distance,
                "front_arc_radius_scale": 1.15,
                "front_arc_weight": human_aware_front_arc_weight,
                "uncertainty_growth_rate": 0.18,
                "low_conf_uncertainty_gain": 0.35,
                "occluded_risk_scale": 0.5,
                "tentative_caution_radius": 1.0,
                "ttc_warning_threshold": 3.0,
                "ttc_hard_threshold": 1.0,
                "dcpa_comfort_threshold": 1.4,
                "crossing_angle_min_deg": 35.0,
                "side_commitment_timeout": 2.0,
                "stale_track_decay_time": 0.8,
                "yield_timeout": 4.0,
                "deadlock_timeout": 6.0,
                "max_forward_near_human": human_aware_max_forward_near_human,
                "max_lateral_near_human": human_aware_max_lateral_near_human,
                "max_reverse_velocity": human_aware_max_reverse_velocity,
                "max_angular_near_human": human_aware_max_angular_near_human,
                "static_clearance": 0.48,
                "static_influence_range": 1.25,
                "w_goal": 1.0,
                "w_path": 0.8,
                "w_static": 8.0,
                "w_personal_space": human_aware_w_personal_space,
                "w_ttc": human_aware_w_ttc,
                "w_pass_front": human_aware_w_pass_front,
                "w_yield": 8.0,
                "w_smooth": human_aware_w_smooth,
                "w_jerk": 3.0,
                "w_legibility": 6.0,
                "w_deadlock": 4.0,
                "publish_risk_grid": True,
                "risk_grid_size": 8.0,
                "risk_grid_resolution": 0.20,
            }
        ],
    )
    human_collision_avoidance_node = Node(
        condition=IfCondition(run_human_collision_avoidance),
        package="unitree_g1_mujoco_bridge",
        executable="human_collision_avoidance",
        name="human_collision_avoidance",
        output="screen",
        parameters=[
            {
                "input_cmd_vel_topic": "/cmd_vel_human_aware",
                "output_cmd_vel_topic": cmd_vel_topic,
                "debug_topic": "/human_collision_avoidance_debug",
                "predicted_cloud_topic": "/predicted_dynamic_obstacles",
                "tracked_cloud_topic": "/tracked_dynamic_obstacles",
                "scan_topic": "/scan",
                "target_frame": "world",
                "robot_frame": "base_footprint",
                "danger_radius": avoidance_danger_radius,
                "critical_radius": avoidance_critical_radius,
                "trajectory_horizon": 4.0,
                "trajectory_dt": 0.2,
                "corridor_width": avoidance_corridor_width,
                "corridor_critical_width": avoidance_corridor_critical_width,
                "corridor_min_length": 1.6,
                "max_avoid_linear": 0.30,
                "max_reverse_linear": avoidance_max_reverse_linear,
                "max_lateral_linear": avoidance_max_lateral_linear,
                "max_avoid_angular": avoidance_max_avoid_angular,
                "hold_time": 0.8,
                "require_recent_input_for_avoidance": False,
                "min_input_speed_for_avoidance": 0.03,
                "stop_on_human_risk": True,
                "emergency_stop_radius": 1.90,
                "robot_self_filter_radius": 0.55,
                "hold_start_location": False,
                "return_position_tolerance": 0.12,
                "return_yaw_tolerance": 0.18,
                "return_linear_speed": 0.22,
                "return_angular_speed": 0.40,
                "return_linear_heading_tolerance": 0.50,
                "return_max_tilt": 0.45,
                "enable_open_space_bias": True,
                "wall_repulsion_clearance": 1.15,
                "wall_repulsion_range": 2.4,
                "wall_repulsion_gain": 0.11,
                "wall_repulsion_max_linear": 0.12,
                "wall_repulsion_max_angular": 0.35,
                "people_repulsion_range": 3.0,
                "predicted_repulsion_range": 2.4,
                "predicted_repulsion_corridor_width": 1.35,
                "predicted_repulsion_forward_scale": 0.25,
                "people_repulsion_gain": avoidance_people_repulsion_gain,
                "people_repulsion_max_linear": 0.08,
                "people_repulsion_max_angular": 0.25,
                "local_risk_radius": 3.2,
                "max_repulsion_points": 24,
                "escape_search_horizon": 2.0,
                "escape_search_dt": 0.2,
                "escape_progress_weight": 0.02,
                "escape_reverse_penalty": 0.12,
                "repulsion_timeout": 1.2,
                "startup_grace_time": 2.0,
                "person_cloud_timeout": 0.8,
            }
        ],
    )
    human_aware_run_logger_node = Node(
        condition=IfCondition(run_human_aware_run_logger),
        package="unitree_g1_mujoco_bridge",
        executable="human_aware_run_logger",
        name="human_aware_run_logger",
        output="screen",
        parameters=[
            {
                "output_path": human_aware_run_log_path,
                "raw_cmd_topic": raw_cmd_vel_topic,
                "planner_cmd_topic": "/cmd_vel_human_aware",
                "safe_cmd_topic": cmd_vel_topic,
                "flush_every": 1,
            }
        ],
    )
    random_explorer_node = Node(
        condition=IfCondition(run_random_explorer),
        package="unitree_g1_mujoco_bridge",
        executable="g1_random_explorer",
        name="g1_random_explorer",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": raw_cmd_vel_topic,
                "scan_topic": "/scan",
                "startup_delay": 4.0,
                "max_linear": 0.32,
                "max_angular": 0.65,
                "front_clearance": 1.25,
                "side_clearance": 0.8,
            }
        ],
    )
    waypoint_patrol_node = Node(
        condition=IfCondition(run_waypoint_patrol),
        package="unitree_g1_mujoco_bridge",
        executable="g1_waypoint_patrol",
        name="g1_waypoint_patrol",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": raw_cmd_vel_topic,
                "scan_topic": "/scan",
                "target_frame": "world",
                "robot_frame": "base_footprint",
                "waypoints": "-2.2,-1.2;1.8,-1.2;2.0,1.2;-1.8,1.4;0.0,0.0",
                "startup_delay": 5.0,
                "goal_tolerance": 0.45,
                "max_linear": 0.18,
                "max_angular": 0.45,
                "heading_tolerance": 0.45,
                "front_clearance": 0.85,
            }
        ],
    )
    waypoint_explorer_node = Node(
        condition=IfCondition(run_waypoint_explorer),
        package="unitree_g1_mujoco_bridge",
        executable="g1_waypoint_explorer",
        name="g1_waypoint_explorer",
        output="screen",
        parameters=[
            {
                "frame_id": "map",
                "startup_delay": 3.0,
                "goal_timeout": 45.0,
                "require_known_free_goal": False,
            }
        ],
    )
    point_goal_node = Node(
        condition=IfCondition(run_point_goal),
        package="unitree_g1_mujoco_bridge",
        executable="g1_point_goal",
        name="g1_point_goal",
        output="screen",
        parameters=[
            {
                "frame_id": "map",
                "robot_frame": "base_footprint",
                "goal_topic": "/g1/point_goal",
                "goal_marker_topic": "/exploration_goal",
                "costmap_topic": "/global_costmap/costmap",
                "startup_delay": 10.0,
                "goal_x": point_goal_x,
                "goal_y": point_goal_y,
                "goal_yaw": point_goal_yaw,
                "validate_costmap": True,
                "goal_clearance": 0.45,
                "goal_timeout": 90.0,
            }
        ],
    )
    frontier_explorer_node = Node(
        condition=IfCondition(run_frontier_explorer),
        package="unitree_g1_mujoco_bridge",
        executable="g1_frontier_explorer",
        name="g1_frontier_explorer",
        output="screen",
        parameters=[
            {
                "map_topic": "/map",
                "goal_topic": "/exploration_goal",
                "robot_frame": "base_footprint",
                "map_frame": "map",
                "startup_delay": 12.0,
            }
        ],
    )
    slam_reset_monitor_node = Node(
        condition=IfCondition(run_slam_reset_monitor),
        package="unitree_g1_mujoco_bridge",
        executable="g1_slam_reset_monitor",
        name="g1_slam_reset_monitor",
        output="screen",
        parameters=[
            {
                "fallen_tilt_threshold": fall_damping_enter_angle,
                "upright_tilt_threshold": fall_damping_exit_angle,
                "reset_mujoco_on_fall": False,
                "mapping_reset_delay": 1.5,
            }
        ],
    )
    robot_state_publisher_node = Node(
        condition=IfCondition(publish_description),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command([FindExecutable(name="xacro"), " ", robot_description_file]),
                    value_type=str,
                )
            }
        ],
    )
    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )
    locomotion_diagnostics_node = Node(
        condition=IfCondition(run_locomotion_diagnostics),
        package="unitree_g1_mujoco_bridge",
        executable="g1_locomotion_diagnostics",
        name="g1_locomotion_diagnostics",
        output="screen",
        parameters=[
            {
                "raw_cmd_vel_topic": raw_cmd_vel_topic,
                "human_aware_cmd_vel_topic": "/cmd_vel_human_aware",
                "safe_cmd_vel_topic": cmd_vel_topic,
                "body_command_topic": "/g1/body_position_commands",
                "odom_topic": "/odom",
                "lowstate_topic": "/lowstate",
                "diagnostics_topic": "/g1/locomotion_diagnostics",
                "rate_hz": 2.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_sim",
                default_value="true",
                description="Start the Unitree Mujoco simulator process.",
            ),
            DeclareLaunchArgument(
                "publish_description",
                default_value="true",
                description="Publish robot_description from robot_description_file.",
            ),
            DeclareLaunchArgument(
                "run_bridge",
                default_value="true",
                description="Start the ROS 2 bridge for Unitree low-level body and hand topics.",
            ),
            DeclareLaunchArgument(
                "run_hold_controller",
                default_value="false",
                description="Start the neutral hold-pose publisher. Leave false when sending your own commands.",
            ),
            DeclareLaunchArgument(
                "run_policy_controller",
                default_value="true",
                description="Start the trained policy controller that consumes /cmd_vel.",
            ),
            DeclareLaunchArgument(
                "policy_path",
                default_value="latest",
                description="TorchScript/ONNX policy file for G1 locomotion, or 'latest' to use the newest exported lower-body walking ONNX.",
            ),
            DeclareLaunchArgument(
                "policy_obs_dim",
                default_value="0",
                description="Expected policy observation dimension. 0 infers from ONNX metadata/shape.",
            ),
            DeclareLaunchArgument(
                "policy_action_dim",
                default_value="0",
                description="Expected policy action dimension. 0 infers from ONNX metadata/shape; 29 uses whole-body control.",
            ),
            DeclareLaunchArgument(
                "policy_command_height",
                default_value="0.84",
                description="Target torso height for height-command stand/getup policies.",
            ),
            DeclareLaunchArgument(
                "policy_rate_hz",
                default_value="50.0",
                description="Policy controller update rate in Hz.",
            ),
            DeclareLaunchArgument(
                "raw_cmd_vel_topic",
                default_value="/cmd_vel",
                description="Raw velocity command topic from Nav2/explorers before human collision avoidance filtering.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel_safe",
                description="Safety-filtered Twist topic consumed by the locomotion policy controller.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_timeout",
                default_value="0.5",
                description="Seconds before stale /cmd_vel commands are replaced with zero velocity.",
            ),
            DeclareLaunchArgument(
                "max_lin_vel_x",
                default_value="0.35",
                description="Absolute x velocity limit passed to the policy.",
            ),
            DeclareLaunchArgument(
                "max_lin_vel_y",
                default_value="0.18",
                description="Absolute y velocity limit passed to the policy.",
            ),
            DeclareLaunchArgument(
                "max_ang_vel_z",
                default_value="0.45",
                description="Absolute yaw velocity limit passed to the policy.",
            ),
            DeclareLaunchArgument(
                "startup_current_pose_hold_time",
                default_value="0.0",
                description="Seconds to hold the initial lowstate joint pose before blending toward the policy stand pose.",
            ),
            DeclareLaunchArgument(
                "startup_blend_time",
                default_value="0.2",
                description="Seconds to move from current pose to the trained standing pose before policy inference.",
            ),
            DeclareLaunchArgument(
                "startup_hold_time",
                default_value="0.0",
                description="Seconds to hold the trained standing pose before policy inference.",
            ),
            DeclareLaunchArgument(
                "startup_require_upright",
                default_value="true",
                description="Require an upright posture before policy inference for stable walking startup.",
            ),
            DeclareLaunchArgument(
                "startup_max_recovery_time",
                default_value="0.5",
                description="Maximum seconds to hold the trained standing pose while tilted before policy inference. 0 waits until upright.",
            ),
            DeclareLaunchArgument(
                "policy_action_ramp_time",
                default_value="0.5",
                description="Seconds to ramp policy actions in from the trained standing pose after startup.",
            ),
            DeclareLaunchArgument(
                "mujoco_startup_hold_enable",
                default_value="true",
                description="Freeze MuJoCo physics until first low-level motor command arrives.",
            ),
            DeclareLaunchArgument(
                "mujoco_startup_hold_timeout",
                default_value="0.0",
                description="Wall seconds to keep startup physics frozen if no motor command arrives. 0 means wait indefinitely.",
            ),
            DeclareLaunchArgument(
                "mujoco_reset_on_controller_start",
                default_value="false",
                description="Optionally reset MuJoCo when the first low-level controller command arrives. Disabled by default because reset can drop G1 before policy stabilizes.",
            ),
            DeclareLaunchArgument(
                "run_clamp_button",
                default_value="false",
                description="Start a GUI button and /g1/clamp_pose service for holding the robot pose during sensor testing.",
            ),
            DeclareLaunchArgument(
                "run_sensors",
                default_value="true",
                description="Publish simulated RealSense D435i and Livox ROS 2 sensor topics.",
            ),
            DeclareLaunchArgument(
                "run_slam",
                default_value="true",
                description="Project Livox point cloud to /scan and run slam_toolbox to publish /map.",
            ),
            DeclareLaunchArgument(
                "run_nav2",
                default_value="true",
                description="Start Nav2 planner/controller/BT navigator using /scan and /map.",
            ),
            DeclareLaunchArgument(
                "run_dynamic_obstacle_tracker",
                default_value="true",
                description="Cluster Livox world cloud points and publish /dynamic_obstacles markers in RViz.",
            ),
            DeclareLaunchArgument(
                "run_lidar_person_detector",
                default_value="true",
                description="Run LiDAR BEV/fallback person detector so sparse clusters can reinforce tracker confirmation.",
            ),
            DeclareLaunchArgument(
                "lidar_person_model_path",
                default_value="",
                description="Path to ONNX LiDAR BEV person detector. Output must be [x, y, confidence, radius] in robot-local metres.",
            ),
            DeclareLaunchArgument(
                "run_human_aware_motion_planner",
                default_value="true",
                description="Sample and score smooth human-aware velocity commands before the hard collision safety filter.",
            ),
            DeclareLaunchArgument(
                "run_human_collision_avoidance",
                default_value="true",
                description="Hard safety filter from /cmd_vel_human_aware to /cmd_vel_safe when tracked humans create near collision risk.",
            ),
            DeclareLaunchArgument(
                "run_human_aware_run_logger",
                default_value="false",
                description="Write timestamped human-aware planner/filter/debug snapshots to JSONL.",
            ),
            DeclareLaunchArgument(
                "human_aware_run_log_path",
                default_value="/tmp/human_aware_run.jsonl",
                description="Output JSONL path for the human-aware run logger.",
            ),
            DeclareLaunchArgument("tracker_use_object_motion_detection", default_value="false"),
            DeclareLaunchArgument("tracker_immediate_person_risk", default_value="true"),
            DeclareLaunchArgument("tracker_publish_candidate_predictions", default_value="true"),
            DeclareLaunchArgument("tracker_candidate_person_score_threshold", default_value="0.58"),
            DeclareLaunchArgument("tracker_confirmed_person_score_threshold", default_value="0.52"),
            DeclareLaunchArgument("tracker_confirm_confidence_threshold", default_value="0.50"),
            DeclareLaunchArgument("tracker_min_track_age", default_value="2"),
            DeclareLaunchArgument("tracker_min_dynamic_observations", default_value="1"),
            DeclareLaunchArgument("tracker_min_person_observations", default_value="2"),
            DeclareLaunchArgument("avoidance_danger_radius", default_value="1.60"),
            DeclareLaunchArgument("avoidance_critical_radius", default_value="0.95"),
            DeclareLaunchArgument("avoidance_corridor_width", default_value="1.45"),
            DeclareLaunchArgument("avoidance_corridor_critical_width", default_value="0.85"),
            DeclareLaunchArgument("avoidance_max_reverse_linear", default_value="0.25"),
            DeclareLaunchArgument("avoidance_max_lateral_linear", default_value="0.20"),
            DeclareLaunchArgument("avoidance_max_avoid_angular", default_value="0.55"),
            DeclareLaunchArgument("avoidance_people_repulsion_gain", default_value="0.10"),
            DeclareLaunchArgument("human_aware_max_forward_near_human", default_value="0.35"),
            DeclareLaunchArgument("human_aware_max_lateral_near_human", default_value="0.16"),
            DeclareLaunchArgument("human_aware_max_reverse_velocity", default_value="0.10"),
            DeclareLaunchArgument("human_aware_max_angular_near_human", default_value="0.55"),
            DeclareLaunchArgument("human_aware_comfort_radius", default_value="1.40"),
            DeclareLaunchArgument("human_aware_front_arc_angle_deg", default_value="30.0"),
            DeclareLaunchArgument("human_aware_front_arc_distance", default_value="1.5"),
            DeclareLaunchArgument("human_aware_front_arc_weight", default_value="0.55"),
            DeclareLaunchArgument("human_aware_w_personal_space", default_value="12.0"),
            DeclareLaunchArgument("human_aware_w_ttc", default_value="10.0"),
            DeclareLaunchArgument("human_aware_w_pass_front", default_value="18.0"),
            DeclareLaunchArgument("human_aware_w_smooth", default_value="4.0"),
            DeclareLaunchArgument(
                "run_random_explorer",
                default_value="false",
                description="Publish random obstacle-aware /cmd_vel commands so the robot explores automatically.",
            ),
            DeclareLaunchArgument(
                "run_waypoint_patrol",
                default_value="false",
                description="Publish simple sequential room/lab waypoint /cmd_vel commands when Nav2 goals are not being used.",
            ),
            DeclareLaunchArgument(
                "run_waypoint_explorer",
                default_value="false",
                description="Send random NavigateToPose goals to Nav2 for autonomous exploration.",
            ),
            DeclareLaunchArgument(
                "run_point_goal",
                default_value="false",
                description="Send one explicit point-to-point Nav2 goal from point_goal_x/point_goal_y or /g1/point_goal.",
            ),
            DeclareLaunchArgument("point_goal_x", default_value="nan"),
            DeclareLaunchArgument("point_goal_y", default_value="nan"),
            DeclareLaunchArgument("point_goal_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "run_frontier_explorer",
                default_value="false",
                description="Use frontier exploration to navigate toward unknown map boundaries until the world is mapped.",
            ),
            DeclareLaunchArgument(
                "run_locomotion_diagnostics",
                default_value="true",
                description="Publish command-path and odometry diagnostics for movement troubleshooting.",
            ),
            DeclareLaunchArgument(
                "run_slam_reset_monitor",
                default_value="true",
                description="Reset SLAM/Nav2 mapping after robot fall transitions.",
            ),
            DeclareLaunchArgument(
                "run_rviz",
                default_value="true",
                description="Start RViz with G1 robot, TF, camera, and Livox displays.",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("unitree_g1_sim_bringup"), "config", "slam_toolbox_livox.yaml"]
                ),
                description="slam_toolbox parameter file for Livox 2D mapping.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("unitree_g1_sim_bringup"), "config", "nav2_livox.yaml"]
                ),
                description="Nav2 parameter file for Livox scan navigation.",
            ),
            DeclareLaunchArgument(
                "lock_waist",
                default_value="false",
                description="Hold waist yaw/roll/pitch at zero for lower-body locomotion stability. Leave false for whole-body policies.",
            ),
            DeclareLaunchArgument(
                "waist_lock_kp",
                default_value="120.0",
                description="Position gain used for locked waist joints.",
            ),
            DeclareLaunchArgument(
                "waist_lock_kd",
                default_value="8.0",
                description="Damping gain used for locked waist joints.",
            ),
            DeclareLaunchArgument(
                "balance_waist",
                default_value="false",
                description="Dynamically move waist roll/pitch to keep torso upright. Disable for whole-body policies.",
            ),
            DeclareLaunchArgument(
                "waist_balance_p",
                default_value="0.35",
                description="Waist upright balance proportional gain on torso roll/pitch.",
            ),
            DeclareLaunchArgument(
                "waist_balance_i",
                default_value="0.0",
                description="Waist upright balance integral gain on torso roll/pitch.",
            ),
            DeclareLaunchArgument(
                "waist_balance_d",
                default_value="0.06",
                description="Waist upright balance damping gain on torso angular velocity.",
            ),
            DeclareLaunchArgument(
                "waist_balance_max",
                default_value="0.25",
                description="Maximum absolute dynamic waist roll/pitch target in radians.",
            ),
            DeclareLaunchArgument(
                "fall_damping_enable",
                default_value="true",
                description="Switch body motors to damping-only control when the robot has fallen.",
            ),
            DeclareLaunchArgument(
                "fall_damping_enter_angle",
                default_value="0.95",
                description="Roll/pitch magnitude in radians that enters fallen damping mode.",
            ),
            DeclareLaunchArgument(
                "fall_damping_exit_angle",
                default_value="0.45",
                description="Roll/pitch magnitude in radians below which normal position control resumes.",
            ),
            DeclareLaunchArgument(
                "fall_damping_kd",
                default_value="8.0",
                description="Joint damping gain used while fallen.",
            ),
            DeclareLaunchArgument(
                "fall_damping_waist_kd",
                default_value="10.0",
                description="Waist joint damping gain used while fallen.",
            ),
            DeclareLaunchArgument(
                "fall_damping_reset_grace_time",
                default_value="3.0",
                description="Seconds to suppress fall damping after a reset/upright transition so the stand controller can recover.",
            ),
            DeclareLaunchArgument(
                "startup_fall_damping_grace_time",
                default_value="0.0",
                description="Startup seconds before fall damping can engage. 0 keeps startup falls from being hidden.",
            ),
            DeclareLaunchArgument(
                "base_height_offset",
                default_value="0.182",
                description="Calibration offset from estimated leg extension to pelvis height above base_footprint.",
            ),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("unitree_g1_description"),
                        "urdf",
                        "g1_29dof_mode_15_brainco_hand.urdf",
                    ]
                ),
                description="URDF or xacro file for robot_state_publisher when available.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("unitree_g1_sim_bringup"), "rviz", "g1_sim.rviz"]
                ),
                description="RViz config file.",
            ),
            DeclareLaunchArgument(
                "mujoco_scene",
                default_value="scene_29dof_extreme_lab.xml",
                description="G1 Mujoco scene XML under unitree_robots/g1. Default is the Extreme Lab scene.",
            ),
            DeclareLaunchArgument(
                "mujoco_pedestrians_enable",
                default_value="true",
                description="Enable animated MuJoCo pedestrian mocap bodies in scenes that define ped*_torso bodies.",
            ),
            DeclareLaunchArgument(
                "mujoco_pedestrians_count",
                default_value="8",
                description="Number of MuJoCo pedestrian mocap actors to animate and publish.",
            ),
            DeclareLaunchArgument(
                "lidar_rate_hz",
                default_value="10.0",
                description="Simulated Livox publish rate. Lower if packet generation cannot keep up.",
            ),
            DeclareLaunchArgument(
                "livox_scan_points",
                default_value="2400",
                description="Raycast points per Livox packet. Higher values improve 360 person tracking; lower values reduce latency.",
            ),
            DeclareLaunchArgument(
                "livox_horizontal_min_deg",
                default_value="0.0",
                description="Minimum horizontal LiDAR azimuth in degrees.",
            ),
            DeclareLaunchArgument(
                "livox_horizontal_max_deg",
                default_value="360.0",
                description="Maximum horizontal LiDAR azimuth in degrees; default gives full 360 degree coverage.",
            ),
            DeclareLaunchArgument(
                "livox_vertical_min_deg",
                default_value="-16.0",
                description="Minimum vertical LiDAR elevation in degrees; lower rays catch legs/kerbs.",
            ),
            DeclareLaunchArgument(
                "livox_vertical_max_deg",
                default_value="22.0",
                description="Maximum vertical LiDAR elevation in degrees; narrowed for denser human-height returns.",
            ),
            DeclareLaunchArgument(
                "use_mujoco_raycast_lidar",
                default_value="true",
                description="Use live MuJoCo mj_ray returns streamed from the simulator instead of the Python scene approximation.",
            ),
            DeclareLaunchArgument(
                "mujoco_lidar_dead_zone_radius",
                default_value="0.35",
                description="Radius around the robot base where self-filtered MuJoCo LiDAR hits are dropped.",
            ),
            DeclareLaunchArgument(
                "mujoco_lidar_dead_zone_height",
                default_value="1.45",
                description="Height of the robot self-filter dead-zone cylinder for MuJoCo LiDAR hits.",
            ),
            DeclareLaunchArgument(
                "mujoco_lidar_socket",
                default_value="/tmp/unitree_g1_mujoco_lidar.sock",
                description="Unix datagram socket used for MuJoCo LiDAR raycast packets.",
            ),
            DeclareLaunchArgument(
                "publish_lidar_tf",
                default_value="true",
                description="Publish world->mujoco_livox_frame from the exact MuJoCo LiDAR packet pose.",
            ),
            DeclareLaunchArgument(
                "publish_camera",
                default_value="false",
                description="Publish simulated D435i image topics for RViz. Disabled by default because raycast camera is expensive.",
            ),
            DeclareLaunchArgument(
                "camera_frame",
                default_value="mujoco_d435i_color_optical_frame",
                description="Exact MuJoCo camera optical frame used by D435i color images.",
            ),
            DeclareLaunchArgument(
                "depth_frame",
                default_value="mujoco_d435i_color_optical_frame",
                description="Exact MuJoCo camera optical frame used by D435i depth images.",
            ),
            DeclareLaunchArgument(
                "camera_width",
                default_value="96",
                description="Simulated D435i image width. Keep modest for MuJoCo raycast camera packet size.",
            ),
            DeclareLaunchArgument(
                "camera_height",
                default_value="72",
                description="Simulated D435i image height. Keep modest for MuJoCo raycast camera packet size.",
            ),
            DeclareLaunchArgument(
                "camera_rate_hz",
                default_value="3.0",
                description="Simulated D435i image publish rate.",
            ),
            DeclareLaunchArgument(
                "use_mujoco_raycast_camera",
                default_value="true",
                description="Use live MuJoCo multi-ray depth camera frames instead of the Python scene approximation.",
            ),
            DeclareLaunchArgument(
                "mujoco_camera_socket",
                default_value="/tmp/unitree_g1_mujoco_camera.sock",
                description="Unix datagram socket used for MuJoCo camera packets.",
            ),
            DeclareLaunchArgument(
                "publish_depth_pointcloud",
                default_value="false",
                description="Publish D435i RGB-D point cloud. Disabled by default to avoid LiDAR lag.",
            ),
            DeclareLaunchArgument(
                "use_mujoco_lidar_pose",
                default_value="true",
                description="Drive simulated Livox ray origin from live Mujoco base state instead of TF.",
            ),
            DeclareLaunchArgument(
                "use_cmd_vel_lidar_fallback",
                default_value="false",
                description="Dead-reckon Livox pose from /cmd_vel if exact Mujoco pose is unavailable.",
            ),
            DeclareLaunchArgument(
                "require_exact_lidar_pose",
                default_value="false",
                description="Skip LiDAR packets until exact Mujoco Livox pose is available.",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="mujoco_livox_frame",
                description="Exact MuJoCo LiDAR frame used by /livox/lidar. This avoids URDF approximation drift.",
            ),
            DeclareLaunchArgument(
                "lidar_output_frame",
                default_value="mujoco_livox_frame",
                description="Frame used for /livox/lidar points. Use mujoco_livox_frame for exact MuJoCo sensor pose.",
            ),
            DeclareLaunchArgument(
                "livox_base_offset_x",
                default_value="0.10",
                description="Livox x offset from Mujoco base IMU frame in meters.",
            ),
            DeclareLaunchArgument(
                "livox_base_offset_y",
                default_value="0.0",
                description="Livox y offset from Mujoco base IMU frame in meters.",
            ),
            DeclareLaunchArgument(
                "livox_base_offset_z",
                default_value="0.46",
                description="Livox z offset from Mujoco base IMU frame in meters.",
            ),
            DeclareLaunchArgument(
                "pelvis_to_livox_z",
                default_value="0.34",
                description="Livox z offset above pelvis/base joint for fallback raycast height.",
            ),
            LogInfo(
                msg="Starting Unitree G1 simulation bridge. The simulator exposes the same Unitree DDS interfaces used by the real robot."
            ),
            ExecuteProcess(
                condition=IfCondition(run_sim),
                cmd=[PathJoinSubstitution(["/workspace", "scripts", "run_mujoco_g1.sh"])],
                additional_env={
                    "UNITREE_MUJOCO_SCENE": mujoco_scene,
                    "UNITREE_MUJOCO_PEDESTRIANS_ENABLE": mujoco_pedestrians_enable,
                    "UNITREE_MUJOCO_PEDESTRIANS_COUNT": mujoco_pedestrians_count,
                    "UNITREE_MUJOCO_PEDESTRIAN_SOCKET": "/tmp/unitree_g1_mujoco_pedestrians.sock",
                    "UNITREE_MUJOCO_LIDAR_ENABLE": use_mujoco_raycast_lidar,
                    "UNITREE_MUJOCO_LIDAR_RATE_HZ": lidar_rate_hz,
                    "UNITREE_MUJOCO_LIDAR_POINTS": livox_scan_points,
                    "UNITREE_MUJOCO_LIDAR_HORIZONTAL_MIN_DEG": livox_horizontal_min_deg,
                    "UNITREE_MUJOCO_LIDAR_HORIZONTAL_MAX_DEG": livox_horizontal_max_deg,
                    "UNITREE_MUJOCO_LIDAR_VERTICAL_MIN_DEG": livox_vertical_min_deg,
                    "UNITREE_MUJOCO_LIDAR_VERTICAL_MAX_DEG": livox_vertical_max_deg,
                    "UNITREE_MUJOCO_LIDAR_DEAD_ZONE_RADIUS": mujoco_lidar_dead_zone_radius,
                    "UNITREE_MUJOCO_LIDAR_DEAD_ZONE_HEIGHT": mujoco_lidar_dead_zone_height,
                    "UNITREE_MUJOCO_LIDAR_SOCKET": mujoco_lidar_socket,
                    "UNITREE_MUJOCO_CAMERA_ENABLE": publish_camera,
                    "UNITREE_MUJOCO_CAMERA_WIDTH": camera_width,
                    "UNITREE_MUJOCO_CAMERA_HEIGHT": camera_height,
                    "UNITREE_MUJOCO_CAMERA_RATE_HZ": camera_rate_hz,
                    "UNITREE_MUJOCO_CAMERA_SOCKET": mujoco_camera_socket,
                    "UNITREE_MUJOCO_STARTUP_HOLD_ENABLE": mujoco_startup_hold_enable,
                    "UNITREE_MUJOCO_STARTUP_HOLD_TIMEOUT": mujoco_startup_hold_timeout,
                    "UNITREE_MUJOCO_RESET_ON_CONTROLLER_START": mujoco_reset_on_controller_start,
                },
                output="screen",
            ),
            TimerAction(
                period=3.0,
                actions=[
                    bridge_node,
                    hold_node,
                    policy_node,
                    clamp_button_node,
                    sensors_node,
                    livox_to_scan_node,
                    scan_filter_node,
                    slam_node,
                    lidar_person_detector_node,
                    dynamic_obstacle_node,
                    human_aware_motion_planner_node,
                    human_collision_avoidance_node,
                    human_aware_run_logger_node,
                    locomotion_diagnostics_node,
                    random_explorer_node,
                    waypoint_patrol_node,
                    frontier_explorer_node,
                    slam_reset_monitor_node,
                    robot_state_publisher_node,
                    rviz_node,
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[nav2_launch],
            ),
            TimerAction(
                period=11.0,
                actions=[waypoint_explorer_node, point_goal_node],
            ),
        ]
    )
