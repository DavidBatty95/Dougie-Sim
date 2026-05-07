from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
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
    base_height_offset = LaunchConfiguration("base_height_offset")
    policy_path = LaunchConfiguration("policy_path")
    policy_obs_dim = LaunchConfiguration("policy_obs_dim")
    policy_action_dim = LaunchConfiguration("policy_action_dim")
    policy_rate_hz = LaunchConfiguration("policy_rate_hz")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    cmd_vel_timeout = LaunchConfiguration("cmd_vel_timeout")
    max_lin_vel_x = LaunchConfiguration("max_lin_vel_x")
    max_lin_vel_y = LaunchConfiguration("max_lin_vel_y")
    max_ang_vel_z = LaunchConfiguration("max_ang_vel_z")
    startup_blend_time = LaunchConfiguration("startup_blend_time")
    startup_hold_time = LaunchConfiguration("startup_hold_time")
    mujoco_startup_hold_enable = LaunchConfiguration("mujoco_startup_hold_enable")
    mujoco_startup_hold_timeout = LaunchConfiguration("mujoco_startup_hold_timeout")
    mujoco_reset_on_controller_start = LaunchConfiguration("mujoco_reset_on_controller_start")
    robot_description_file = LaunchConfiguration("robot_description_file")
    rviz_config = LaunchConfiguration("rviz_config")
    mujoco_scene = LaunchConfiguration("mujoco_scene")
    lidar_rate_hz = LaunchConfiguration("lidar_rate_hz")
    livox_scan_points = LaunchConfiguration("livox_scan_points")
    livox_horizontal_min_deg = LaunchConfiguration("livox_horizontal_min_deg")
    livox_horizontal_max_deg = LaunchConfiguration("livox_horizontal_max_deg")
    use_mujoco_raycast_lidar = LaunchConfiguration("use_mujoco_raycast_lidar")
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
                "rate_hz": policy_rate_hz,
                "cmd_vel_topic": cmd_vel_topic,
                "cmd_vel_timeout": cmd_vel_timeout,
                "max_lin_vel_x": max_lin_vel_x,
                "max_lin_vel_y": max_lin_vel_y,
                "max_ang_vel_z": max_ang_vel_z,
                "startup_blend_time": startup_blend_time,
                "startup_hold_time": startup_hold_time,
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
                "use_mujoco_raycast_lidar": use_mujoco_raycast_lidar,
                "mujoco_lidar_socket": mujoco_lidar_socket,
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
                "lidar_frame": lidar_frame,
                "lidar_output_frame": lidar_output_frame,
                "livox_base_offset_x": livox_base_offset_x,
                "livox_base_offset_y": livox_base_offset_y,
                "livox_base_offset_z": livox_base_offset_z,
                "pelvis_to_livox_z": pelvis_to_livox_z,
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
                default_value=(
                    "/workspace/vendor/mjlab/logs/rsl_rl/g1_velocity/"
                    "2026-05-05_19-54-01_g1_flat_lower_body_locomotion/"
                    "2026-05-05_19-54-01_g1_flat_lower_body_locomotion.onnx"
                ),
                description="TorchScript or callable PyTorch policy file for G1 locomotion.",
            ),
            DeclareLaunchArgument(
                "policy_obs_dim",
                default_value="82",
                description="Expected policy observation dimension. 82 matches the stable lower-body standing policy.",
            ),
            DeclareLaunchArgument(
                "policy_action_dim",
                default_value="12",
                description="Expected policy action dimension. 12 matches the stable lower-body standing policy.",
            ),
            DeclareLaunchArgument(
                "policy_rate_hz",
                default_value="50.0",
                description="Policy controller update rate in Hz.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Twist topic used as the locomotion velocity command.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_timeout",
                default_value="0.5",
                description="Seconds before stale /cmd_vel commands are replaced with zero velocity.",
            ),
            DeclareLaunchArgument(
                "max_lin_vel_x",
                default_value="0.5",
                description="Absolute x velocity limit passed to the policy.",
            ),
            DeclareLaunchArgument(
                "max_lin_vel_y",
                default_value="0.25",
                description="Absolute y velocity limit passed to the policy.",
            ),
            DeclareLaunchArgument(
                "max_ang_vel_z",
                default_value="0.5",
                description="Absolute yaw velocity limit passed to the policy.",
            ),
            DeclareLaunchArgument(
                "startup_blend_time",
                default_value="1.5",
                description="Seconds to blend from current pose to the trained standing pose.",
            ),
            DeclareLaunchArgument(
                "startup_hold_time",
                default_value="1.0",
                description="Seconds to hold the trained standing pose before policy control.",
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
                description="Optionally reset MuJoCo when the first low-level controller command arrives. Disabled by default to preserve policy startup state.",
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
                "run_rviz",
                default_value="true",
                description="Start RViz with G1 robot, TF, camera, and Livox displays.",
            ),
            DeclareLaunchArgument(
                "lock_waist",
                default_value="true",
                description="Hold waist yaw/roll/pitch at zero for locomotion stability.",
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
                default_value="true",
                description="Dynamically move waist roll/pitch to keep torso upright.",
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
                default_value="12.0",
                description="Waist joint damping gain used while fallen.",
            ),
            DeclareLaunchArgument(
                "fall_damping_reset_grace_time",
                default_value="3.0",
                description="Seconds to suppress fall damping after a reset/upright transition so the stand controller can recover.",
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
                default_value="scene_29dof_room.xml",
                description="G1 Mujoco scene XML under unitree_robots/g1.",
            ),
            DeclareLaunchArgument(
                "lidar_rate_hz",
                default_value="10.0",
                description="Simulated Livox publish rate. Lower if packet generation cannot keep up.",
            ),
            DeclareLaunchArgument(
                "livox_scan_points",
                default_value="2500",
                description="Raycast points per Livox packet. Lower values reduce latency.",
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
                "use_mujoco_raycast_lidar",
                default_value="true",
                description="Use live MuJoCo mj_ray returns streamed from the simulator instead of the Python scene approximation.",
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
                    "UNITREE_MUJOCO_LIDAR_ENABLE": use_mujoco_raycast_lidar,
                    "UNITREE_MUJOCO_LIDAR_RATE_HZ": lidar_rate_hz,
                    "UNITREE_MUJOCO_LIDAR_POINTS": livox_scan_points,
                    "UNITREE_MUJOCO_LIDAR_HORIZONTAL_MIN_DEG": livox_horizontal_min_deg,
                    "UNITREE_MUJOCO_LIDAR_HORIZONTAL_MAX_DEG": livox_horizontal_max_deg,
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
                    robot_state_publisher_node,
                    rviz_node,
                ],
            ),
        ]
    )
