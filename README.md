# Unitree G1 ROS 2 MuJoCo Docker Workspace

Dockerized ROS 2 Humble workspace for Unitree G1 MuJoCo simulation, ROS 2/DDS control, simulated Livox/RealSense sensors, and MJ Lab RL training/playback.

## Contents

- `docker/`, `docker-compose.yml`: ROS 2 Humble desktop container with CycloneDDS, RViz, MuJoCo build deps, `uv`, and `onnxruntime`.
- `ros2_ws/`: ROS 2 workspace, local G1 bridge/bringup/description packages, configs, scenes, and source checkouts.
- `vendor/`: MJ Lab, MuJoCo, Unitree SDK2 build/install outputs, trained policy logs.
- `scripts/`: setup, build, launch, teleop, MJ Lab, and test helpers.

Compose service: `unitree-g1-sim`
Container name: `unitree-g1-ros2-mujoco`
Default ROS domain: `ROS_DOMAIN_ID=42`

## Setup

From the host:

```bash
cp .env.example .env
docker compose build
docker compose run --rm unitree-g1-sim bash
```

Inside the container:

```bash
/workspace/scripts/setup_sources.sh
/workspace/scripts/build_ws.sh
/workspace/scripts/build_unitree_mujoco.sh
```

Later shell access:

```bash
./scripts/run_shell.sh
```

## Run Simulation

From the host:

```bash
./scripts/bringup_full_sim.sh
```

This starts:

- Unitree MuJoCo G1 in `scene_29dof_room.xml`.
- ROS 2 bridge for `/lowstate`, `/lowcmd`, `/joint_states`, hands, and body position commands.
- Lower-body ONNX standing/locomotion policy on `/cmd_vel`.
- Startup physics hold until controller commands arrive.
- Fall damping to stop floor flailing after a fall.
- 360 degree MuJoCo-raycast Livox LiDAR.
- RViz with robot TF/model and LiDAR view.

Camera raycasting is off by default for framerate. Enable it only when needed:

```bash
./scripts/bringup_full_sim.sh publish_camera:=true
```

## Control

Teleop from the host:

```bash
./scripts/teleop_cmd_vel.sh
```

Or publish directly:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

Default policy:

```text
/workspace/vendor/mjlab/logs/rsl_rl/g1_velocity/2026-05-05_19-54-01_g1_flat_lower_body_locomotion/2026-05-05_19-54-01_g1_flat_lower_body_locomotion.onnx
```

Default policy shape:

```bash
policy_obs_dim:=82 policy_action_dim:=12
```

Override policy:

```bash
./scripts/bringup_full_sim.sh \
  policy_path:=/workspace/path/to/policy.onnx \
  policy_obs_dim:=82 \
  policy_action_dim:=12
```

Full-body policies typically use:

```bash
policy_obs_dim:=99 policy_action_dim:=29
```

## Safety And Startup

Important launch defaults:

```bash
mujoco_startup_hold_enable:=true
mujoco_startup_hold_timeout:=0.0
mujoco_reset_on_controller_start:=false
fall_damping_enable:=true
fall_damping_enter_angle:=0.95
fall_damping_exit_angle:=0.45
fall_damping_reset_grace_time:=3.0
```

Notes:

- `mujoco_startup_hold_timeout:=0.0` waits indefinitely for controller commands.
- Fall damping switches body joints to damping-only if the robot falls.
- After reset/upright transition, damping is suppressed briefly so the stand policy can recover.

## Sensors

Livox:

- Topic: `/livox/lidar` in `mujoco_livox_frame`.
- RViz/world topic: `/livox/lidar_world`.
- Full horizontal FOV by default: `0 -> 360 deg`.
- Robot/head geometry is ignored by LiDAR ray filtering.

RealSense-style D435i:

- Enable with `publish_camera:=true`.
- Color: `/camera/camera/color/image_raw`.
- Depth: `/camera/camera/depth/image_rect_raw`.
- Exact frame: `mujoco_d435i_color_optical_frame`.
- RGB is MuJoCo geometry-color raycast output, not photorealistic rendering.

Performance-safe defaults:

```bash
lidar_rate_hz:=10.0
livox_scan_points:=2500
publish_camera:=false
publish_depth_pointcloud:=false
```

Denser LiDAR or camera:

```bash
./scripts/bringup_full_sim.sh livox_scan_points:=4800 lidar_rate_hz:=15
./scripts/bringup_full_sim.sh publish_camera:=true camera_width:=96 camera_height:=72 camera_rate_hz:=3
```

## Scenes

Select scene:

```bash
./scripts/bringup_full_sim.sh mujoco_scene:=scene_29dof_obstacles.xml
```

Available G1 scenes:

- `scene_29dof_room.xml`: default enclosed room with walls, doorway, furniture, boxes, shelf, column, bin, and pedestal.
- `scene_29dof_empty.xml`: flat empty floor.
- `scene_29dof_obstacles.xml`: bars, platform, ramp, pillars, box.
- `scene_29dof_steps.xml`: step sequence.
- `scene_29dof.xml`: upstream flat default.

## MJ Lab RL

Install MJ Lab deps inside the container:

```bash
/workspace/scripts/setup_mjlab.sh
```

CPU-only deps:

```bash
MJLAB_EXTRA=cpu /workspace/scripts/setup_mjlab.sh
```

List envs:

```bash
./scripts/mjlab_list_envs.sh
```

Train examples:

```bash
./scripts/mjlab_train_g1_smoke.sh
./scripts/mjlab_train_g1_flat.sh
./scripts/mjlab_train_g1_flat_lower_body.sh
./scripts/mjlab_train_g1_rough.sh
./scripts/mjlab_train_g1_rough_lower_body.sh
./scripts/mjlab_train_g1_crouch_stand.sh
```

If your Docker Compose supports GPU flags:

```bash
MJLAB_WITH_GPUS=1 ./scripts/mjlab_train_g1_smoke.sh
```

Pass MJ Lab overrides after the launcher:

```bash
./scripts/mjlab_train_g1_flat.sh --agent.max-iterations 1000 --env.scene.num-envs 256
MJLAB_RUN_NAME=my_run ./scripts/mjlab_train_g1_rough.sh
```

Playback examples:

```bash
./scripts/mjlab_play_g1_flat.sh /workspace/vendor/mjlab/logs/rsl_rl/g1_velocity/<RUN_DIR>/model_<N>.pt
./scripts/mjlab_play_g1_flat_lower_body.sh /workspace/vendor/mjlab/logs/rsl_rl/g1_velocity/<RUN_DIR>/model_<N>.pt
```

MJ Lab training generally requires NVIDIA GPU runtime. Import checks and some playback paths can run without training hardware.

## Tests

Fast checks:

```bash
./scripts/test_workspace.sh fast
```

Pre-upload/pre-commit checks:

```bash
./scripts/test_workspace.sh precommit
```

Smoke launch check:

```bash
./scripts/test_workspace.sh smoke
```

## Upload Notes

Include source/config files:

- `docker/`, `docker-compose.yml`, `.env.example`, `README.md`
- `scripts/`
- `ros2_ws/src/`, `ros2_ws/unitree.repos`, `ros2_ws/config/`

Exclude generated artifacts unless intentionally sharing them:

- `ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`
- `vendor/unitree_sdk2_build/`, `vendor/unitree_sdk2_install/`
- `vendor/unitree_mujoco_install/`
- large MuJoCo archives/extracts if recipients can regenerate them

Policy logs under `vendor/mjlab/logs/` can be large. Keep only the policy runs needed for upload, or document a required `policy_path` override.

## GUI

For X11 access, the host may need:

```bash
xhost +local:docker
```

NVIDIA variables are set by compose. Mesa/Intel/AMD uses mounted `/dev/dri`.

## Hardware Warning

This workspace is simulation-first. Do not move gains or policies to hardware without Unitree safety procedures, torque/current limits, emergency stop handling, and a controlled test stand.
