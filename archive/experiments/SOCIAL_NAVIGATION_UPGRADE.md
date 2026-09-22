# Social waypoint navigation upgrade

> Archived implementation/experiment record for the oracle baseline.
> Maintained results and their scope are in `docs/DEMO_RESULTS.md`.

This document records the **oracle diagnostic baseline**. The onboard-only
implementation and its still-incomplete performance validation are documented in
[Onboard social navigation](ONBOARD_SOCIAL_NAVIGATION.md). The oracle results
below do not count as sensor-only acceptance.

## Run the mission

```bash
./scripts/run_demo_suite.sh --case social-waypoints --goals 8 --seed 44
```

The first run builds the ROS packages, including the new Nav2 people layer and
updated social messages, and rebuilds MuJoCo. Subsequent runs can add
`--fast-start`. Add `--headless` for scored runs without RViz.

This mission uses the bundled pedestrian plaza and the checksum-pinned official
Unitree G1 policy. Goals are sampled in the clear central walking area, normally
1.5–2.5 m from the robot and clear of currently observed people. Nav2 must return
a successful path ending within 0.25 m of each proposed goal. Goal sampling bounds
are distinct from the larger arena bounds available for avoidance.

RViz uses `g1_social_waypoints.rviz`: a stable world frame, the known scene map,
mission waypoint, planned route, robot, tracked people, predicted trajectories,
and executable avoidance candidates.

## Pedestrians are independent of the robot

`social_wandering_plaza` gives each pedestrian seeded destinations, an A* route
around static obstacles, bounded velocity changes, pauses, and person-person
spacing. Initial positions and start offsets are seeded rather than synchronizing
every actor on the same crossing. People do **not** read the robot's pose,
velocity, goal, or command, and do not yield to it. Their positions are continuous;
the wandering controller does not teleport or respawn them to resolve a blockage.

The robot is responsible for avoiding them. The separate
`social_waypoints_plaza` scenario retains repeatable central crossing patrols for
stress testing. The older staged endurance scenario remains a distinct test.

## Validated mode and provenance

The waypoint showcase is explicitly an **oracle-tracking, known-map simulation**:

- Robot localization and mission goals use the simulator's stable `world` frame.
- Person positions and velocities come from simulator truth. Future destinations
  and pedestrian A* routes are not supplied to the robot's predictor.
- The static navigation map is generated from scene geometry and uses conservative
  box footprints, including low furniture. It is published on
  `/simulation/static_map`; it is not represented as a SLAM-derived map.
- Current people have their own self-clearing Nav2 layer. They are not left as
  marking-only point-cloud obstacles in a static costmap.
- Raw raycast scans still participate in the benchmark's data-health contract.

Tracking changes are covered by perception-side synthetic regressions, but these
live mission results do not establish LiDAR-only navigation performance. The
response model's gain, delay, and time constants are explicit engineering
parameters, not a fitted response certificate.

## Main changes

### Tracking and prediction

- Bootstrap velocity from a timestamped measurement window, then update the
  Kalman filter using innovation/displacement gates. Ordinary walking no longer
  fails a spurious acceleration check before velocity has been initialized.
- Publish qualified tentative people on the typed interface so caution can begin
  before identity confirmation.
- Preserve observation age separately from a propagated lost-state timestamp.
- Propagate process uncertainty during coasting, and remove the old prediction
  fallback that could continue walking in a lifetime displacement direction after
  someone stopped.
- Predict a bounded short acceleration/braking transition followed by constant
  velocity. The wandering profile uses its declared walking-speed envelope.
- Use sparse covariance algebra, latest-only sensor input, efficient byte arrays,
  and subscriber-aware visualization to reduce tracking overhead.

### Executable motion and early avoidance

- `motion_envelope.py` shares command bounds, deadband behavior, reverse handling,
  turning-arc floors, and axis constraints between planning and policy execution.
- Rollouts start from measured motion and the actual filtered policy command.
  They include software command slew, physical response lag, and stopping motion.
- Full prediction-horizon conflicts activate candidate search during normal
  navigation. Swept relative segments detect crossings between samples.
- The agile profile searches translations, intermediate velocities, and short
  step-and-hold maneuvers. Side commitment also applies to turning candidates.
- The final supervisor rechecks the executable command every tick using fresh
  predictions. Limited short-horizon recovery is explicitly labelled in debug
  output when holding position predicts contact.

### Routes, goals, and liveness

- `unitree_social_costmap/PeopleLayer` paints current person footprints and marks
  the previous footprint bounds dirty. Movement, empty observations, expiry, and
  reset clear old costs without erasing underlying static obstacles.
- Both Nav2 costmaps use the known static map and current-person layer in this
  showcase. This avoids persistent human returns in a static obstacle layer.
- The 2D global planner, holonomic local planner, and velocity smoother use
  consistent lateral limits. A hidden zero-lateral smoother clamp no longer
  suppresses lateral navigation commands.
- The social planner follows the validated route associated with the active
  waypoint. Unrelated `ComputePathToPose` samples cannot overwrite that route.
- A valid route through an immutable known map survives temporary replanning
  failure, while moving-person safety is reevaluated continuously.
- `/benchmark/goal_active` is a mission heartbeat. Cancelling a goal revokes route
  following; a missing heartbeat stops it within one second. This separates
  mission ownership from transient Nav2 controller/recovery output.

### Sensor and runtime correctness

- Project scans in ground-referenced `base_footprint`, retaining low obstacles
  while rejecting the floor. Do not interpolate fake surfaces across large
  range discontinuities.
- Mask typed person body discs rather than applying a second large inflation
  around already inflated footprint points.
- MJL3 sensor packets carry the exact robot-root pose and velocity alongside the
  separate LiDAR pose, from the same MuJoCo snapshot. This avoids inferring base
  motion from a moving torso-mounted sensor when SportModeState is unavailable.
  Legacy packets remain readable, and `/g1/sim_sensor_status` reports the effective
  odometry source. Both ROS packages and MuJoCo must be rebuilt for MJL3.
- Static clearance has bounded distance-field queries and a single failed TF
  lookup per planning cycle. Timestamped odometry supports scan alignment.
- ONNX inference uses one non-spinning worker; feedback and command consumers use
  latest-only queues. Planning and tracker timing are recorded in debug output.

## Diagnostics

```bash
python3 scripts/summarize_social_run.py PATH_TO_human_aware.jsonl
python3 scripts/summarize_social_run.py PATH_TO_human_aware.jsonl --last 5
```

The summary includes control modes, rejection reasons, mission-reference status,
source ages, and planning wall/thread times. `scripts/replay_social_snapshot.py`
supports bounded translation-search diagnosis of blocked snapshots. Run it inside
a sourced ROS environment; it is a diagnostic replay, not a live benchmark score.

`scripts/profile_policy_runtime.py POLICY.onnx` measures inference latency and
process CPU duty at a specified control period. On the development host, a 50 Hz
two-second measurement used about 9.8 CPU-seconds with ORT's default pool versus
0.018 CPU-seconds with the bounded worker. Isolated inference itself remained
well below 0.1 ms in both configurations; the benefit is freeing CPU between ticks.

## Verification record

Final MJL3/known-map live runs, four independent pedestrians and eight requested
waypoints each:

| Seed | Goals reached | Time | Minimum body clearance | Score |
| --- | --- | --- | --- | --- |
| 42 | 8/8 | 91.6 s | 0.407 m | 89.156 |
| 43 | 8/8 | 92.4 s | 0.567 m | 88.823 |
| 44 | 8/8 | 85.3 s | 0.532 m | 89.628 |

All three trials were valid and passed the result validator with **zero collisions,
zero near misses, and zero falls**. This is evidence for the stated simulation
configuration, not a guarantee over arbitrary crowds or a perception-only result.
Live timing can alter goal acceptance and execution even with seeded actors.

The local artifact bundles are under
`ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite/`:

- Seed 42: `suite_20260921T153133Z/social-waypoints/`
- Seed 43: `suite_20260921T152713Z/social-waypoints/`
- Seed 44: `suite_20260921T153314Z/social-waypoints/`

Each contains a manifest, result, scenario, per-goal/event CSVs, command-chain
JSONL, and simulation log. Intermediate failed development trials remain in that
ignored artifact tree and are not promoted as successful results. The benchmark
uses its existing 0.55 m position tolerance and cancels Nav2 after arrival, so an
action cancellation in the log can accompany a correctly reached waypoint.

An additional GUI-enabled seed-44 trial reached 4/4 goals in 41.3 s with zero
collisions or near misses, 0.521 m minimum body clearance, and score 91.515
(`suite_20260921T155606Z/social-waypoints/`). RViz started and initialized OpenGL,
but logged an indexed-map GLSL shader error: "active samplers with a different
type refer to the same texture image unit". This occurred with both default
rendering (`suite_20260921T155222Z/social-waypoints/`) and an experimental
RViz-only software-rendering override in the later trial. The ineffective
override was removed. Mission execution passed, but correct visual map rendering
has not been established.

Checks completed:

- ROS build of the updated interface, bridge, benchmark, bringup, and costmap packages.
- MuJoCo simulator rebuild including MJL3 and independent pedestrian planning.
- 276 targeted Python tests: 164 bridge, 16 bringup, 66 benchmark, and 30 selected
  orchestration/validator tests.
- Nav2 people-layer CTest target (two tests) and all five MuJoCo CTest targets.
- `./scripts/test_workspace.sh fast`, including shell syntax, Python compilation,
  scene/package XML, Xacro, assets, and launch loading.
- Root and simulator-submodule whitespace checks.

One separate check remains failing: `--suite quick --headless --seed 1` produced a
collision in its mock goal-follower fixture (2/3 goals). That mock backend reports
`mock_goal_follower_no_people_input` and does not execute this navigation stack.
The live mission results above and the passing unit/build checks are reported
separately from that mock failure.

The regression coverage includes walking velocity acquisition at 5–30 Hz,
noisy centroids, turns/stops, tentative delivery, observation age, swept crossings,
body-frame/ground-frame geometry, physical stopping and reversal, stale inputs,
route ownership, waypoint reachability, and costmap footprint clearing.
