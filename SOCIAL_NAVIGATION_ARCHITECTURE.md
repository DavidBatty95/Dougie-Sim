# Social Navigation Architecture

## Runtime Contract

The primary runtime mode is perception-only. MuJoCo pedestrian truth is available
to benchmark and logging code, but must not influence tracking or commands.

The default profile is selected automatically:

```bash
ros2 launch unitree_g1_sim_bringup g1_sim.launch.py
```

Oracle mode is an upper-bound controller ablation, not a perception result:

```bash
ros2 launch unitree_g1_sim_bringup g1_sim.launch.py \
  social_navigation_params_file:=$(ros2 pkg prefix unitree_g1_sim_bringup)/share/unitree_g1_sim_bringup/config/social_navigation_oracle_v1.yaml
```

## Data Flow

```text
LiDAR -> person detector -> dynamic tracker
      -> /tracked_persons
      -> /predicted_person_trajectories
      -> human-aware motion planner
      -> hard-stop safety supervisor
      -> locomotion policy
```

`TrackedPersons` carries identity, pose, velocity, covariance, confidence, radius,
and lifecycle state. `PredictedTrajectories` retains prediction time offsets. The
legacy JSON and point-cloud outputs remain available for diagnostics and temporary
compatibility, but are disabled as planner inputs by default.

Nav2 costmaps mark current confirmed tracks. They intentionally do not consume a
flattened future prediction cloud because that representation loses temporal
correspondence. Future occupancy is evaluated against candidate robot states inside
the social motion planner.

## Safety Responsibilities

- The social planner handles personal space, predicted interaction, yielding,
  passing behavior, static clearance, smoothness, and deadlock recovery.
- The final supervisor handles imminent collision prevention and stale-input
  stopping. `stop_on_human_risk` always means a zero command.
- Active escape requires a fresh scan and a statically collision-free trajectory.
- Missing robot transforms and stale safety-critical inputs fail closed.

## Profiles

- `social_navigation_perception_only_v1.yaml`: default evaluation mode.
- `social_navigation_oracle_v1.yaml`: simulator-truth upper bound only.
- `social_navigation_balanced_v1.yaml`: perception-only nominal tuning.
- `social_navigation_conservative_v1.yaml`: perception-only larger margins.
- `social_navigation_hardware_safe_v1.yaml`: unvalidated starting values, not a
  hardware safety certification.

## Evaluation

Live results distinguish invalid infrastructure trials from valid scored trials.
Metrics report center distance and body clearance separately, apply duration
thresholds to near misses, and debounce TTC events into episodes. Live goals can be
validated through Nav2 `ComputePathToPose` before scoring.

The mock backend validates benchmark mechanics only. It does not exercise Nav2,
tracking, the social planner, or the locomotion policy and must not be presented as
an algorithm benchmark.

## Remaining Work

These items require larger simulator, Nav2, or hardware projects:

- Deterministic MuJoCo reset/spawn/trajectory services and synchronized `/clock`.
- A Nav2 controller or MPPI critic that owns social trajectory optimization.
- Camera/LiDAR fusion, group-space models, and learned intent prediction.
- Hardware validation, latency budgets, and independent safety certification.
