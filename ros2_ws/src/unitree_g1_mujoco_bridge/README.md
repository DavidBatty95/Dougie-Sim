# G1 MuJoCo bridge and autonomy

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../../README.md"><img src="../../../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../../README.md#quick-start"><img src="../../../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../../../docs/DEMO_GUIDE.md"><img src="../../../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../../../docs/INDEX.md"><img src="../../../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../../../docs/REPOSITORY_GUIDE.md"><img src="../../../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../../../docs/PROJECT_STATUS.md"><img src="../../../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../../../docs/PROJECT_PLAN.md"><img src="../../../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../../../docs/RESULTS_REVIEW.md"><img src="../../../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../../../docs/literature_review/README.md"><img src="../../../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../../../docs/DEVELOPMENT.md"><img src="../../../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[Workspace map](../README.md) · [Architecture](../../../docs/ARCHITECTURE.md) · [Source boundaries](../../../docs/ONBOARD_SOCIAL_NAVIGATION.md)

This Python package connects simulated G1 state and actuation to the onboard
navigation pipeline. It also contains arm-command composition and the newer
experimental sensor-driven transport modules.

## Find the implementation

All module names below are inside `unitree_g1_mujoco_bridge/`.

| Area | Main modules | Responsibility |
| --- | --- | --- |
| Robot boundary | `g1_mujoco_bridge.py`, `g1_constants.py` | Joint state, validated low-level body commands and hand interfaces |
| Sensors | `g1_sim_sensors.py`, `sensor_packets.py` | DDS/socket inputs, ROS sensor messages and scoring-only truth output |
| Localization | `onboard_lidar_odometry.py`, `lidar_odometry.py` | Scan-derived navigation odometry |
| Person perception | `dynamic_obstacle_tracker.py`, `scan_dynamic_filter.py` | Observed people and dynamic scan handling |
| Planning | `human_aware_motion_planner.py`, `observed_route.py`, `avoidance_candidates.py` | Route following, candidate motion and recovery decisions |
| Supervision | `human_collision_avoidance.py`, `motion_envelope.py`, `policy_safety.py` | Final command checks and executable-motion constraints |
| Locomotion | `g1_policy_cmd_vel_controller.py`, `unitree_rl_lab_policy.py` | ONNX policy input/output and joint targets |
| Manipulation | `arm_command_compositor.py`, `sim_manipulation_io.py`, `arm_kinematics.py` | Arm authority, simulated grasp I/O and measured kinematics |
| Transport | `transport_demo.py`, `transport_perception.py`, `transport_actions.py`, `transport_health.py` | RGB-D mission, action lifecycle and evaluation coverage |
| Diagnostics | `human_aware_run_logger.py`, `evaluation_alignment.py` | Decision traces and scoring alignment |

## Runtime contract

The onboard path uses measured LiDAR, IMU and encoders for navigation. Simulator
truth is reserved for evaluation and explicitly selected oracle cases. The
command path runs from navigation intent through social planning and collision
supervision, then policy inference and the DDS bridge. See
[architecture](../../../docs/ARCHITECTURE.md) for topics and authority boundaries.

Bringup composes these nodes; launching an individual module is not equivalent
to launching a complete demo. Installed node entry points are listed in
[`setup.py`](setup.py). The experimental transport module is launched by its
dedicated script rather than a `transport_demo` console entry point.

## Use and verification

From the repository root:

```bash
./geoff demo plaza --duration 120
./scripts/run_demo_suite.sh --case onboard-waypoints --goals 3 --people 4 --seed 42 --headless
```

Use the [payload guide](../../../docs/PAYLOAD_POLICY_TRAINING.md) for transport
with an explicit trained policy. Tests are in [`test/`](test/); the
[development guide](../../../docs/DEVELOPMENT.md) records the source-test and
build environment. Changes to planners should be checked with regression tests
and scored trials; visual success alone does not establish acceptance.

## Current direction

The published baseline includes navigation reliability corrections, payload
transport, action termination and evaluation-health helpers. Full navigation and
social-transport acceptance remain open. Follow
[project status](../../../docs/PROJECT_STATUS.md) and the
[S1/S2/S4 work packages](../../../docs/PROJECT_PLAN.md).
