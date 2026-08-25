<div align="center">
  <img src="docs/media/banner.svg" width="100%" alt="Animated Geoff-Sim humanoid autonomy banner">

  <p><strong>A reproducible research platform for intelligent humanoid simulation.</strong></p>
  <p>MuJoCo locomotion · ROS 2/DDS · SLAM and Nav2 · Human-aware navigation · MJLab training</p>

  <p>
    <a href="https://github.com/DavidBatty95/Geoff-Sim/actions/workflows/verification.yml"><img src="https://github.com/DavidBatty95/Geoff-Sim/actions/workflows/verification.yml/badge.svg?branch=main" alt="Verification status"></a>
    <img src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&amp;logoColor=white" alt="ROS 2 Humble">
    <img src="https://img.shields.io/badge/MuJoCo-Simulation-00A6D6" alt="MuJoCo simulation">
    <img src="https://img.shields.io/badge/Docker-Compose%20v2-2496ED?logo=docker&amp;logoColor=white" alt="Docker Compose v2">
    <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&amp;logoColor=white" alt="Ubuntu 22.04">
  </p>

  <p>
    <a href="#quick-start">Quick Start</a> ·
    <a href="#architecture">Architecture</a> ·
    <a href="#running">Run</a> ·
    <a href="#development">Develop</a> ·
    <a href="#documentation">Documentation</a>
  </p>
</div>

**Geoff-Sim** brings the Unitree G1, MuJoCo, and ROS 2 Humble into one integrated environment for locomotion, autonomous navigation, simulated perception, dynamic-obstacle avoidance, benchmarking, and policy development.

> [!WARNING]
> **Simulation-first safety notice:** Do not deploy these controllers, gains, or policies to hardware without Unitree's safety procedures, current and torque limits, an emergency stop, and controlled test-stand validation.

## ✨ Features

- Unitree G1 29-DoF MuJoCo simulation and low-level DDS bridge
- ROS 2 joint states, odometry, TF, Livox-style LiDAR, and optional camera simulation
- SLAM Toolbox and Nav2 integration
- Dynamic-obstacle tracking, trajectory prediction, and velocity safety filtering
- Human-aware navigation demos and repeatable person-avoidance benchmarks
- ONNX locomotion policy playback and MJLab training helpers
- Docker-based reproducible ROS 2 Humble development environment
- Fast, pre-commit, and live simulation verification tiers

## 📋 Requirements

| Requirement | Supported configuration |
| --- | --- |
| Host OS | Ubuntu or Debian Linux with X11/XWayland |
| Architecture | x86_64 |
| Container | Docker Engine with Compose v2 |
| ROS | ROS 2 Humble, provided by the container |
| Graphics | `/dev/dri` and an active X display |
| GPU training | Optional NVIDIA driver and NVIDIA Container Toolkit |

Windows, macOS, ARM systems, and native non-container builds are not currently supported. The installer needs `sudo` and internet access.

<a id="quick-start"></a>
## 🚀 Quick Start

Clone the repository and run the top-level installer:

```bash
git clone --recurse-submodules https://github.com/DavidBatty95/Geoff-Sim.git
cd Geoff-Sim
./install.sh
```

The installer:

- Installs Git, curl, X11 authentication tools, Docker Engine, and Compose v2 when needed
- Initializes all pinned Git submodules
- Creates the local `.env` with the current user and display settings
- Builds the ROS 2 Humble image
- Installs ROS package dependencies with `rosdep`
- Builds the ROS workspace, Unitree SDK2, and standalone MuJoCo simulator
- Downloads and checksum-verifies the official Unitree G1 locomotion policy

Installation can take several minutes. Docker image and source downloads require a stable internet connection.

After installation, start a people-free simulation in the included empty scene:

```bash
./scripts/run_waypoint_navigation_demo.sh mujoco_scene:=scene_29dof_empty.xml
```

If Docker was installed for the first time, log out and back in before running project scripts so the new `docker` group membership takes effect.

### 🛠️ Installer Options

```text
./install.sh --skip-docker-install  # Require Docker to be installed already
./install.sh --skip-build           # Prepare the host and repository only
./install.sh --help
```

The installer is idempotent: it preserves an existing `.env` and reuses completed downloads and builds where supported.

## 🔧 Manual Setup

Use this path when Docker is managed externally or you want to inspect each setup step:

```bash
git submodule update --init --recursive
cp .env.example .env
# Update UID, GID, and DISPLAY in .env for the current host.
docker compose build
docker compose run --rm unitree-g1-sim bash -lc \
  '/workspace/scripts/setup_sources.sh && \
   /workspace/scripts/build_ws.sh && \
   /workspace/scripts/build_unitree_mujoco.sh'
./scripts/download_unitree_rl_lab_policy.sh
```

For X11 authorization errors, allow the local Docker client for the current session:

```bash
xhost +local:docker
```

Revoke that access when finished with `xhost -local:docker`.

<a id="running"></a>
## 🎮 Running The Project

Run all commands below from the repository root after completing `./install.sh`. GUI demos require a working X11/XWayland display. Only run one simulation or command-generating demo at a time.

### 🎬 Key Demo Scripts

| Script | Demonstrates | Fresh clone |
| --- | --- | --- |
| `run_waypoint_navigation_demo.sh` | G1 locomotion, SLAM, Nav2, continuous waypoint goals, and RViz | Yes, with a bundled scene |
| `run_collision_avoidance_demo.sh` | Pedestrian tracking, social planning, collision supervision, Nav2, and diagnostics | Requires Extreme Lab assets and a compatible policy |
| `bringup_full_sim.sh` | Configurable integrated simulation without forcing a specific demo mode | Requires an explicit available scene and policy |
| `bringup_full_sim_rviz.sh` | Integrated simulation with RViz enabled | Requires an explicit available scene and policy |
| `run_person_avoidance_benchmark.sh` | Deterministic, headless person-avoidance regression using the mock backend | Yes |
| `run_extreme_lab_person_avoidance_benchmark.sh` | Scored live MuJoCo/Nav2 pedestrian-avoidance run | Requires Extreme Lab assets and a compatible policy |
| `run_extreme_lab_waypoint_benchmark.sh` | Scored people-free locomotion and waypoint baseline | Requires Extreme Lab assets and a compatible policy |

#### Waypoint Navigation

This is the recommended first visual demo. It downloads and verifies the public Unitree policy, builds the current workspace, starts RViz, and continuously generates reachable Nav2 goals:

```bash
./scripts/run_waypoint_navigation_demo.sh mujoco_scene:=scene_29dof_empty.xml
```

Try another bundled environment by overriding the scene:

```bash
./scripts/run_waypoint_navigation_demo.sh mujoco_scene:=scene_29dof_obstacles.xml
```

The demo uses a measured reactive envelope of `0.45 m/s` forward velocity and `0.35 m/s²` acceleration. The normal integrated and social-navigation launch defaults retain the `0.35 m/s` policy clamp. A live empty-scene check delivered `0.354 m/s` average motion, remained upright, and settled to `0.0032 m` net drift over the five seconds after a stop command.

#### Collision Avoidance

When the external Extreme Lab assets and the configured local policy are available, run the complete social-navigation demonstration:

```bash
./scripts/run_collision_avoidance_demo.sh
```

The script starts six pedestrians, RViz, SLAM, Nav2 waypoint exploration, dynamic tracking, the conservative social-navigation profile, the final collision supervisor, and a JSONL run logger. Safety commands are published on `/cmd_vel_safe`; diagnostics are available on `/human_collision_avoidance_debug`.

#### Configurable Bringup

Use the general launcher to select individual subsystems or pass launch arguments directly:

```bash
./scripts/bringup_full_sim.sh \
  mujoco_scene:=scene_29dof_room.xml \
  policy_path:=/workspace/path/to/policy.onnx \
  run_rviz:=true
```

Open a sourced ROS shell for topic inspection or manual commands:

```bash
./scripts/run_shell.sh
```

#### Headless Regression Benchmark

Run a deterministic benchmark without MuJoCo, RViz, or external assets:

```bash
./scripts/run_person_avoidance_benchmark.sh \
  --mode smoke \
  --seed 1 \
  --people 8 \
  --goals 10 \
  --output /workspace/ros2_ws/src/dynamic_obstacle_benchmarks/results/demo.json
```

#### Live Extreme Lab Benchmarks

Run a scored pedestrian-avoidance scenario:

```bash
./scripts/run_extreme_lab_person_avoidance_benchmark.sh \
  --seed 42 --people 8 --goals 10
```

Run the people-free locomotion baseline:

```bash
./scripts/run_extreme_lab_waypoint_benchmark.sh --seed 42 --goals 5
```

Add `--no-rviz` for headless execution, or `--no-sim` when a compatible simulation is already running. Both wrappers save JSON, CSV, scenario, and text report artifacts under the benchmark package's ignored `results/` directory.

Available upstream scenes include:

| Scene | Description |
| --- | --- |
| `scene_29dof_empty.xml` | Flat empty floor; recommended first run |
| `scene_29dof_room.xml` | Enclosed room |
| `scene_29dof_obstacles.xml` | Ramps, bars, pillars, platform, and box |
| `scene_29dof_steps.xml` | Step sequence |
| `scene_29dof_pedestrians.xml` | Animated pedestrian environment |
| `scene_29dof.xml` | Upstream flat scene |

The custom `scene_29dof_extreme_lab.xml` requires external textured assets that are not committed to this repository.

### ⌨️ Teleoperation

Start keyboard teleoperation in another terminal:

```bash
./scripts/teleop_cmd_vel.sh
```

Publish commands to `/cmd_vel`; the integrated stack routes them through the human-aware planner and final safety filter before the locomotion policy consumes `/cmd_vel_safe`.

Do not enable multiple command generators at the same time. Direct teleoperation, point goals, waypoint exploration, frontier exploration, random exploration, and waypoint patrol are separate control modes.

<a id="architecture"></a>
## 🏗️ Architecture

<p align="center">
  <a href="docs/ARCHITECTURE.md">
    <img src="docs/media/architecture.svg" alt="Animated architecture diagram of the Geoff-Sim system" width="100%">
  </a>
</p>

The stack closes the loop from simulated sensors through SLAM, Nav2, dynamic-person tracking, social planning, and collision supervision into an ONNX or Torch locomotion policy. The policy produces joint targets that `g1_mujoco_bridge` converts to Unitree `LowCmd` DDS messages; MuJoCo returns robot state over DDS and high-bandwidth simulated sensor packets over Unix datagrams.

See the [system architecture evaluation](docs/ARCHITECTURE.md) for runtime boundaries, command authority, design strengths, and deployment risks.

Primary packages:

| Package | Purpose |
| --- | --- |
| `unitree_g1_description` | G1 URDF, meshes, and joint metadata |
| `unitree_g1_mujoco_bridge` | DDS bridge, controllers, sensors, tracking, and safety nodes |
| `unitree_g1_sim_bringup` | Integrated launch, Nav2/SLAM configuration, and RViz |
| `dynamic_obstacle_benchmarks` | Deterministic mock and live person-avoidance benchmarks |
| `g1_msgs` | Approach, retreat, and manipulation action interfaces |
| `g1_locomotion` | Closed-loop object approach and retreat actions through the `/cmd_vel` safety path |
| `unitree_social_navigation_msgs` | Social-navigation interfaces |

The default container uses `ROS_DOMAIN_ID=42`, CycloneDDS, and loopback-only DDS configuration. Change `ROS_DOMAIN_ID` in `.env` if it conflicts with another ROS environment.

<a id="development"></a>
## 🧪 Development

Rebuild the ROS workspace after source changes:

```bash
docker compose run --rm unitree-g1-sim /workspace/scripts/build_ws.sh
```

Run verification from the repository root:

```bash
./scripts/test_workspace.sh fast
./scripts/test_workspace.sh precommit
./scripts/test_workspace.sh smoke
```

| Tier | Coverage |
| --- | --- |
| `fast` | Shell syntax, Python compilation, XML/Xacro, assets, and launch loading |
| `precommit` | Fast checks, clean ROS build, local package tests, and deterministic benchmark |
| `smoke` | Pre-commit checks plus bounded live MuJoCo topic checks |

GitHub Actions runs the container build, local ROS package tests, and deterministic benchmark on pushes and pull requests. GUI and GPU simulation tests remain local or require a self-hosted runner.

### 🤖 MJLab

Set up MJLab inside the container:

```bash
./scripts/run_shell.sh
/workspace/scripts/setup_mjlab.sh
```

Common host-side helpers:

```bash
./scripts/mjlab_list_envs.sh
./scripts/mjlab_train_g1_smoke.sh
./scripts/mjlab_train_g1_flat.sh
./scripts/mjlab_train_g1_rough.sh
```

GPU workflows require the NVIDIA Container Toolkit and the GPU Compose override:

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml run --rm unitree-g1-sim bash
```

## 🩺 Troubleshooting

| Problem | Resolution |
| --- | --- |
| Docker permission denied | Log out and back in after installation, then verify `docker info` |
| MuJoCo or RViz does not open | Check `DISPLAY`, `/dev/dri`, `/tmp/.X11-unix`, and X11 permissions |
| X11 authorization failed | Run `xhost +local:docker` for the session |
| ROS nodes cannot discover each other | Confirm matching `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and `CYCLONEDDS_URI` |
| MuJoCo executable is missing | Run `docker compose run --rm unitree-g1-sim /workspace/scripts/build_unitree_mujoco.sh` |
| ROS package is missing after an update | Rerun `./install.sh` or `/workspace/scripts/build_ws.sh` in the container |
| Default scene cannot load | Select `mujoco_scene:=scene_29dof_empty.xml`; Extreme Lab assets are external |
| Default policy cannot load | Use `run_waypoint_navigation_demo.sh`, which downloads and configures the verified public policy |

<a id="documentation"></a>
## 📚 Documentation

- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md): animated system diagram, runtime boundaries, data flow, and architecture evaluation
- [`LOCOMOTION_STABILITY.md`](LOCOMOTION_STABILITY.md): locomotion training, controller safeguards, acceptance gates, and policy deployment
- [`SOCIAL_NAVIGATION_ARCHITECTURE.md`](SOCIAL_NAVIGATION_ARCHITECTURE.md): social-navigation architecture and safety design

## ⚠️ Known Limitations

- The default trained navigation policy and custom Extreme Lab texture assets are intentionally not stored in Git.
- MuJoCo LiDAR is an approximation; real Livox deployment requires timestamp, packet-loss, calibration, and hardware safety validation.
- Live MuJoCo actor reset and deterministic scene replay are not yet implemented.
- Vendored Unitree packages may report upstream lint failures during an unrestricted `colcon test`.

## 🗂️ Repository Layout

```text
.
|-- docker/                 ROS 2 Humble image and entrypoint
|-- ros2_ws/                ROS 2 workspace, packages, and DDS configuration
|-- scripts/                Setup, build, launch, test, benchmark, and training tools
|-- vendor/                 Submodules and generated external build outputs
|-- docker-compose.yml      Standard CPU/graphics container configuration
|-- docker-compose.gpu.yml  Optional NVIDIA runtime override
`-- install.sh              End-to-end host and workspace installer
```

Generated builds, downloaded binaries, policies, training runs, bags, and runtime logs are excluded from version control. Commit source and configuration files only.
