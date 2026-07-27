# Unitree G1 ROS 2 MuJoCo

Dockerized ROS 2 Humble workspace for Unitree G1 MuJoCo simulation, ROS 2/DDS bridge nodes, simulated Livox/RealSense sensors, SLAM/Nav2 bringup, dynamic obstacle tracking/avoidance, and MJLab policy training/playback helpers.

This repository is simulation-first. Do not move controllers, gains, or policies to hardware without Unitree safety procedures, current/torque limits, an emergency stop, and controlled test-stand validation.

## Repository Layout

- `docker/`, `docker-compose.yml`: ROS 2 Humble desktop container with CycloneDDS, MuJoCo build dependencies, Nav2, SLAM Toolbox, `uv`, and ONNX Runtime.
- `scripts/`: host/container helpers for setup, builds, tests, simulation bringup, teleop, and MJLab workflows.
- `ros2_ws/config/`: workspace-level ROS/DDS configuration.
- `ros2_ws/src/unitree_g1_description`: G1 URDF, meshes, and description metadata.
- `ros2_ws/src/unitree_g1_mujoco_bridge`: Python ROS 2 bridge, controllers, simulated sensors, LiDAR conversion, dynamic obstacle tracking, exploration, and velocity safety filtering.
- `ros2_ws/src/unitree_g1_sim_bringup`: launch files, Nav2/SLAM configs, and RViz config for integrated simulation.
- `ros2_ws/src/unitree_mujoco`, `ros2_ws/src/unitree_ros2`, `ros2_ws/src/unitree_sdk2`: upstream Unitree sources.
- `vendor/`: MJLab checkout and generated external build/install outputs. Heavy/generated contents are ignored.

## Current ROS 2 Packages

- `unitree_g1_description`: installs robot description assets and validates description/config files in tests.
- `unitree_g1_mujoco_bridge`: publishes `/joint_states`, bridges Unitree low-level commands/state, runs policy/hold controllers, simulates sensors, converts Livox point clouds to scans, tracks dynamic obstacles, predicts obstacle risk, plans human-aware velocity commands, and filters them into `/cmd_vel_safe`.
- `unitree_g1_sim_bringup`: launches MuJoCo, bridge nodes, policy control, SLAM, Nav2, dynamic obstacle tracking, safety filtering, robot state publisher, and RViz.

## Prerequisites

- Linux host with Docker Compose v2.
- X11 access for MuJoCo/RViz GUI.
- Optional NVIDIA container runtime for GPU MJLab training/playback.

Create local environment settings:

```bash
cp .env.example .env
```

If using GUI tools from Docker, allow local X11 access as needed:

```bash
xhost +local:docker
```

## Quick Start

Build the container:

```bash
docker compose build
```

Open a shell:

```bash
./scripts/run_shell.sh
```

Inside the container, populate and build sources:

```bash
/workspace/scripts/setup_sources.sh
/workspace/scripts/build_ws.sh
/workspace/scripts/build_unitree_mujoco.sh
```

Default container environment:

- `ROS_DOMAIN_ID=42`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI=file:///workspace/ros2_ws/config/cyclonedds.xml`

## Run Simulation

Full default simulation, without automatic goals:

```bash
./scripts/bringup_full_sim.sh
```

Run with RViz explicitly:

```bash
./scripts/bringup_full_sim_rviz.sh
```

Pedestrian scene with Nav2 waypoint exploration:

```bash
./scripts/bringup_full_sim.sh \
  mujoco_scene:=scene_29dof_pedestrians.xml \
  run_nav2:=true \
  run_waypoint_explorer:=true
```

Frontier exploration:

```bash
./scripts/bringup_full_sim.sh \
  mujoco_scene:=scene_29dof_pedestrians.xml \
  run_nav2:=true \
  run_frontier_explorer:=true
```

Do not enable `run_random_explorer`, `run_waypoint_explorer`, and `run_frontier_explorer` together. They are separate command/goal generation modes.

## Control And Safety

Teleop:

```bash
./scripts/teleop_cmd_vel.sh
```

Direct command in a sourced ROS shell:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

Velocity command path:

- Raw commands: `/cmd_vel`
- Default command source: none; enable direct teleop, `run_point_goal`, `run_waypoint_explorer`, `run_frontier_explorer`, or `run_waypoint_patrol` explicitly.
- Optional simple patrol source: `g1_waypoint_patrol`, enabled with `run_waypoint_patrol:=true` when Nav2 goals are not being used.
- Human-aware sampled planner output: `/cmd_vel_human_aware`
- Safety-filtered commands: `/cmd_vel_safe`
- Policy controller input: `/cmd_vel_safe`

The default launch starts Nav2 and diagnostics but does not send random goals. This keeps movement troubleshooting deterministic. Enable exactly one command/goal source at a time: direct teleop, point goal, waypoint exploration, frontier exploration, or simple patrol.

Point-to-point goal from launch:

```bash
./scripts/bringup_full_sim.sh \
  run_point_goal:=true \
  point_goal_x:=1.5 \
  point_goal_y:=0.0 \
  point_goal_yaw:=0.0
```

Point-to-point goal while the stack is running:

```bash
ros2 topic pub --once /g1/point_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Locomotion diagnostics are published on `/g1/locomotion_diagnostics` and logged by `g1_locomotion_diagnostics`. Important statuses:

- `WAITING_FOR_RAW_CMD`: Nav2/teleop is not sending commands.
- `NO_SAFE_CMD`: the safety pipeline is not producing the topic consumed by the policy.
- `NO_POLICY_BODY_COMMAND`: the policy is not publishing joint targets.
- `SAFETY_FILTER_STOPPING`: raw commands exist but the safety layer is zeroing them.
- `COMMAND_SENT_BUT_NO_ODOM_MOTION`: commands and joint targets exist, but MuJoCo odometry is not moving.
- `MOVING_OR_READY`: command path is healthy or idle.

Recommended staged movement troubleshooting:

1. Direct policy test without SLAM/Nav2/people avoidance:

```bash
./scripts/bringup_full_sim.sh \
  run_slam:=false \
  run_nav2:=false \
  run_dynamic_obstacle_tracker:=false \
  run_human_aware_motion_planner:=false \
  run_human_collision_avoidance:=false \
  cmd_vel_topic:=/cmd_vel
```

Then publish a small velocity:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

2. Add sensors and SLAM only after direct policy motion works:

```bash
./scripts/bringup_full_sim.sh run_nav2:=false run_dynamic_obstacle_tracker:=false run_human_collision_avoidance:=false cmd_vel_topic:=/cmd_vel
```

3. Add Nav2 point-to-point without moving people:

```bash
./scripts/bringup_full_sim.sh \
  mujoco_pedestrians_enable:=false \
  run_point_goal:=true \
  point_goal_x:=1.5 \
  point_goal_y:=0.0
```

4. Add person tracking after point-to-point navigation works:

```bash
./scripts/bringup_full_sim.sh \
  run_point_goal:=true \
  point_goal_x:=1.5 \
  point_goal_y:=0.0
```

5. Enable random waypoint exploration only after explicit point goals work:

```bash
./scripts/bringup_full_sim.sh run_waypoint_explorer:=true
```

If you explicitly enable simple patrol, keep waypoint exploration off to avoid two raw command sources:

```bash
./scripts/bringup_full_sim.sh run_waypoint_patrol:=true run_waypoint_explorer:=false
```

Dynamic obstacle safety can be disabled for debugging only:

```bash
./scripts/bringup_full_sim.sh run_human_collision_avoidance:=false
```

## Sensors, SLAM, And Dynamic Obstacles

Livox-style simulation topics:

- `/livox/lidar`: raw simulated point cloud.
- `/livox/lidar_world`: world-frame point cloud.
- `/scan_raw`: Livox-projected scan before dynamic filtering.
- `/scan`: scan after dynamic obstacle filtering for SLAM/Nav2.

Camera simulation is off by default for performance:

```bash
./scripts/bringup_full_sim.sh publish_camera:=true
```

Dynamic obstacle topics:

- `/dynamic_obstacles`: RViz markers.
- `/tracked_dynamic_obstacles`: confirmed tracked obstacle points.
- `/predicted_dynamic_obstacles`: short-horizon predicted obstacle risk points.
- `/human_safety_grid`: debug occupancy grid for person-risk areas.
- `/lidar_person_detections`: optional ONNX/fallback LiDAR BEV detector output.
- `/tentative_dynamic_obstacles`: debug/caution-only tentative risk points; not used by Nav2 by default.
- `/dynamic_obstacle_tracker/debug_tracks`: JSON diagnostic strings with cluster rejection reasons, track state, confidence, score, missed count, speed, and association distance.
- `/human_avoidance_state`: current human-aware behaviour state.
- `/human_avoidance_debug`: JSON diagnostics for selected candidate, score, rejected candidates, minimum distance/TCPA, and side commitment.
- `/human_collision_avoidance_debug`: JSON diagnostics for hard safety-filter mode, command override reason, risk source/point, input age, cloud ages, and repulsion forces.
- `human_aware_run_logger`: optional JSONL logger for timestamped planner, safety-filter, tracker, command, odometry, benchmark, and MuJoCo pedestrian snapshots.
- `/human_social_risk_grid`: debug grid for social risk around tracked people.
- `/human_avoidance_markers`: RViz candidate and selected trajectory markers.

The default avoidance path is LiDAR-only moving-object tracking from Livox clusters. Optional ONNX LiDAR person detection can be enabled with:

```bash
./scripts/bringup_full_sim.sh \
  run_lidar_person_detector:=true \
  lidar_person_model_path:=/workspace/path/to/livox_person_bev.onnx
```

Livox-style person tracking defaults prioritize consistency over early detection. Tune these with bags before real hardware use: `point_accumulation_time`, `voxel_leaf_size`, `near_noise_radius`, `far_noise_radius`, `cluster_epsilon_base`, `cluster_epsilon_per_meter`, `min_person_height`, `min_vertical_layers`, `tentative_confirm_min_hits`, `min_confirm_age`, `max_human_speed`, `max_human_accel`, and `publish_tentative_predictions`.

Offline tracker evaluation from a recorded ROS 2 bag:

```bash
python3 scripts/evaluate_person_tracking_offline.py /path/to/rosbag \
  --output results/person_tracking_eval.json
```

Optional simulation-only ground truth may be supplied as a CSV with columns `stamp,person_id,x,y`:

```bash
python3 scripts/evaluate_person_tracking_offline.py /path/to/rosbag \
  --ground-truth-csv results/pedestrian_ground_truth.csv
```

Ground truth must not be used by runtime perception; it is only for offline metrics such as false positives, missed frames, confirmation latency, fragmentation, track jumps, stale predictions, and false `/cmd_vel_safe` overrides.

Automated human-aware navigation tuning in Extreme Lab:

```bash
python3 scripts/auto_tune_human_aware_navigation.py --people 6 --goals 6 --max-trials 4
```

For a single diagnosable MuJoCo benchmark run, enable the benchmark wrapper's rosbag recorder. The wrapper also writes a `*_human_aware.jsonl` run log when it starts the simulation:

```bash
./scripts/run_extreme_lab_person_avoidance_benchmark.sh --seed 42 --people 6 --goals 6 --record-rosbag --no-rviz
```

The tuner launches repeated benchmark trials with different human-aware planner and safety-filter parameters, ranks runs by goal completion, safety events, motion, falls, and efficiency, then writes:

Any collision ends the active benchmark trial immediately and marks that trial as a failure for tuning.

- `ros2_ws/src/dynamic_obstacle_benchmarks/results/human_aware_auto_tuning/human_aware_tuning_results.csv`
- `ros2_ws/src/dynamic_obstacle_benchmarks/results/human_aware_auto_tuning/human_aware_tuning_results.json`
- `ros2_ws/src/dynamic_obstacle_benchmarks/results/human_aware_auto_tuning/best_human_aware_launch_args.txt`

## Scenes

Select scenes with `mujoco_scene:=...`:

- `scene_29dof_pedestrians.xml`: pedestrian plaza scene.
- `scene_29dof_extreme_lab.xml`: GLB-derived lab scene with animated pedestrians.
- `scene_29dof_room.xml`: enclosed room.
- `scene_29dof_empty.xml`: flat empty floor.
- `scene_29dof_obstacles.xml`: ramps, bars, pillars, platform, and box.
- `scene_29dof_steps.xml`: step sequence.
- `scene_29dof.xml`: upstream flat default.

Disable animated pedestrians:

```bash
./scripts/bringup_full_sim.sh mujoco_pedestrians_enable:=false
```

## Policy And MJLab

Default policy shape is inferred from the ONNX metadata/shape. The bundled full-body flat locomotion policies use:

```bash
policy_obs_dim:=99 policy_action_dim:=29
```

Override policy:

```bash
./scripts/bringup_full_sim.sh \
  policy_path:=/workspace/path/to/policy.onnx \
  policy_obs_dim:=0 \
  policy_action_dim:=0
```

MJLab setup inside the container:

```bash
/workspace/scripts/setup_mjlab.sh
```

Common helpers:

```bash
./scripts/mjlab_list_envs.sh
./scripts/mjlab_train_g1_smoke.sh
./scripts/mjlab_train_g1_flat.sh
./scripts/mjlab_train_g1_rough.sh
./scripts/mjlab_train_g1_getup.sh
./scripts/mjlab_play_g1_flat.sh /workspace/vendor/mjlab/logs/rsl_rl/g1_velocity/<RUN_DIR>/model_<N>.pt
./scripts/mjlab_play_g1_getup.sh latest
```

## Build And Test

Inside the container:

```bash
/workspace/scripts/build_ws.sh
colcon test --merge-install
colcon test-result --verbose
```

Host-side fast checks:

```bash
./scripts/test_workspace.sh fast
```

Pre-commit style checks, including Docker workspace build:

```bash
./scripts/test_workspace.sh precommit
```

Simulation smoke test:

```bash
./scripts/test_workspace.sh smoke
```

If the workspace was previously built inside Docker, host-side `colcon build` may fail because existing CMake caches contain `/workspace/ros2_ws`. Use fresh temporary bases instead of deleting user files:

```bash
colcon --log-base /tmp/unitree_g1_log build \
  --merge-install --symlink-install \
  --build-base /tmp/unitree_g1_build \
  --install-base /tmp/unitree_g1_install
```

## Person Avoidance Benchmark

The `person_avoidance_benchmark` evaluates random goal-to-goal navigation around moving people. It prioritizes safety over speed and outputs a `0-100` score plus JSON/CSV result files.

Smoke benchmark, CI-safe mock backend:

```bash
ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark \
  --config ros2_ws/src/dynamic_obstacle_benchmarks/config/person_avoidance_benchmark.yaml \
  --mode smoke \
  --backend mock \
  --seed 1 \
  --output results/person_avoidance_smoke.json
```

Standard benchmark:

```bash
ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark \
  --config ros2_ws/src/dynamic_obstacle_benchmarks/config/person_avoidance_benchmark.yaml \
  --mode standard \
  --backend mock \
  --seed 42 \
  --people 8 \
  --goals 10 \
  --headless \
  --output results/person_avoidance_seed_42.json
```

Docker convenience wrapper:

```bash
./scripts/run_person_avoidance_benchmark.sh --seed 42 --people 8 --goals 10
```

Extreme Lab wrapper, which launches `scene_29dof_extreme_lab.xml`, runs the benchmark, and writes a text score report under `ros2_ws/src/dynamic_obstacle_benchmarks/results/extreme_lab/`:

```bash
./scripts/run_extreme_lab_person_avoidance_benchmark.sh --seed 42 --people 8 --goals 10
```

Use `--no-sim` if Extreme Lab is already running. The current benchmark score still uses the deterministic `mock` backend unless `--backend mujoco` is implemented with repository-specific MuJoCo reset/spawn hooks.

Stress and scenario modes:

```bash
ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark --mode stress --backend mock --seed 100 --people 25 --goals 20
ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark --mode crossing --backend mock --seed 42
ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark --mode bottleneck --backend mock --seed 42
ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark --mode occlusion --backend mock --seed 42
```

Scoring formula:

```text
final_score = 0.65 * safety + 0.15 * task + 0.10 * speed + 0.10 * efficiency - hard_penalties
```

Safety dominates. One collision caps the score at `40`, multiple collisions cap it at `25`, and no movement caps it at `30`. Near misses, critical near misses, low time-to-collision, and time spent inside warning distances all reduce the safety score.

Result files:

- Main JSON: requested `--output` path.
- Scenario YAML: same basename with `_scenario.yaml` for repeatability diagnostics.
- Per-goal CSV: same basename with `_goals.csv`.
- Safety event CSV: same basename with `_events.csv`.

Example summary:

```text
Person Avoidance Benchmark
--------------------------
Seed: 42
Backend: mock
People: 8
Goals requested: 10
Goals reached: 10
Collisions: 0
Near misses: 2
Final score: 76.3 / 100
Result file: results/person_avoidance_seed_42.json
```

Current backend status:

- `mock`: implemented, deterministic, headless, CI-safe, publishes benchmark people/metrics/result topics.
- `mujoco`: CLI/backend placeholder fails clearly until repository-specific person spawning/reset services are added.

Use the same seed or a saved scenario file when comparing planners:

```bash
ros2 run dynamic_obstacle_benchmarks person_avoidance_benchmark \
  --scenario-file results/person_avoidance_seed_42_scenario.yaml \
  --backend mock \
  --output results/person_avoidance_seed_42_replay.json
```

Scenario replay currently applies to the implemented `mock` backend. MuJoCo actor replay needs repository-specific spawn/reset services.

## Known Limitations

- The dynamic obstacle stack is a practical LiDAR-only tracker/predictor/safety filter, not a full research-grade FAST-LIO2 plus risk-aware MPC/CBF architecture.
- MuJoCo LiDAR simulation approximates Livox behavior; real Mid-360 deployment still needs driver-level timestamp validation, packet-loss monitoring, calibration, and hardware safety validation.
- The person avoidance benchmark currently has a complete mock backend and scoring pipeline. MuJoCo/Nav2 reset, spawn, and goal-action hooks still need repository-specific services/actions before `--backend mujoco` can run end-to-end.
- Full `colcon test` may report lint/style failures in vendored Unitree packages; local package tests are intended to be deterministic and CI-friendly.

## Troubleshooting

- Missing Nav2/SLAM packages: rebuild the Docker image with `docker compose build`.
- No RViz/MuJoCo GUI: check `DISPLAY`, `XAUTHORITY`, `/tmp/.X11-unix`, and `xhost` permissions.
- ROS discovery issues: confirm matching `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and `CYCLONEDDS_URI` in all shells.
- LiDAR lag: reduce `livox_scan_points` or `lidar_rate_hz`.
- Missing MuJoCo binary: run `/workspace/scripts/build_unitree_mujoco.sh` or set `UNITREE_MUJOCO_BIN`.

## Hygiene

Commit source/configuration files, not generated artifacts. Keep `ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`, `vendor/*_build/`, `vendor/*_install/`, MJLab logs, model checkpoints, downloaded archives, bags, and runtime logs out of normal commits unless intentionally publishing a reproducible artifact bundle.
