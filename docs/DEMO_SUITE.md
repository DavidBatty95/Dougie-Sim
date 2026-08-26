# G1 Demo Suite

The demo suite provides one entry point for reproducible locomotion, navigation,
and social-navigation demonstrations. Scored runs save their commands, logs,
policy identity, Git revision, durations, and exit status under a timestamped
artifact directory.

## Quick Start

List every preset and case:

```bash
./scripts/run_demo_suite.sh --list
```

Run the deterministic, headless regression that does not require MuJoCo or
external assets:

```bash
./scripts/run_demo_suite.sh --suite quick --headless
```

Run the official Unitree policy through the mock regression and a live forward
response/stop measurement in the bundled empty scene:

```bash
./scripts/run_demo_suite.sh --suite locomotion --headless
```

Run the prolonged deterministic two-pedestrian avoidance demonstration:

```bash
./scripts/run_demo_suite.sh --case social-endurance --headless
```

Preview all commands and preflight requirements without launching a demo:

```bash
./scripts/run_demo_suite.sh --suite full --headless --dry-run
```

## Presets

| Preset | Cases | Requirements |
| --- | --- | --- |
| `quick` | `mock-avoidance` | Docker image and built benchmark package |
| `locomotion` | quick plus `fixed-forward` | MuJoCo; downloaded checksum-pinned official policy |
| `navigation` | locomotion plus `waypoint-live` | Extreme Lab assets, SLAM, and Nav2 |
| `social` | quick plus `social-live` | Extreme Lab assets and social-navigation stack |
| `endurance` | quick, `fixed-forward`, and `social-endurance` | Extreme Lab assets, social-navigation stack, and official policy |
| `full` | all scored cases | All dependencies above |

The live presets are intentionally sequential. Only one simulation and one
command source should run at a time.

## Visual Cases

Visual cases are interactive and must be selected alone:

```bash
./scripts/run_demo_suite.sh --case waypoint-visual
./scripts/run_demo_suite.sh --case social-visual
```

`waypoint-visual` uses the bundled obstacle scene, the checksum-pinned official
Unitree RL Lab policy, SLAM, Nav2, generated reachable goals, and RViz.
`social-visual` requires the external Extreme Lab assets and displays the same
deterministic two-crossing oracle scenario and yield/resume gates as
`social-endurance`, with MuJoCo and RViz visible.

## Safety And Policy Selection

The suite never selects `policy_path:=latest`. Recency does not imply policy
quality and the current reactive fine-tuning checkpoints fail bilateral lateral,
yaw, stop-settling, and action-clip acceptance gates. They remain research
artifacts and are not demo defaults.

Clean-clone suite cases use the official Unitree RL Lab G1-29DoF velocity policy.
The downloader verifies SHA-256 before every required suite. Social cases keep
`max_lin_vel_y:=0.0`; avoidance therefore relies on early stopping and stable
forward arcs rather than unsupported last-moment sidesteps.

Waypoint and scored social cases use the measured official-policy walking
envelope (`0.45 m/s` clamp, `0.35 m/s²` acceleration, and a `0.20 m/s`
turning-arc floor). Social planner and safety limits remain more conservative.

The scored `waypoint-live` case uses two fixed forward goals in the Extreme Lab
central corridor. This isolates repeatable SLAM/Nav2/locomotion integration from
the policy's known weak pure-yaw response. Use the standalone random waypoint
benchmark for cross-map turning stress; it is intentionally not presented as a
passing showcase.

The fixed-forward case uses the measured official-policy showcase envelope of
`0.35 m/s` with `0.35 m/s²` acceleration and a five-second stop phase. It
validates all of the following:

- no fall and maximum tilt at most `0.35 rad`;
- a recorded policy SHA-256 and scene;
- motion onset within `1.5 s`;
- steady response at least 25% of the command;
- speed below `0.03 m/s` or `rad/s` for a sustained `0.5 s` window, or a
  final one-second p95 speed no greater than `0.08` when odometry is noisy;
- post-stop drift no greater than `0.20 m`.

Live waypoint and social cases also run the benchmark result validator. A zero
collision count, no fall, valid trial, movement, completed goals, finite metrics,
speed compliance, and the configured score threshold are required for success.
`social-live` is an inactive-person integration smoke test. It holds pedestrian
animation for 45 seconds, longer than its short fixed goal normally takes. This
validates startup, tracking, Nav2, locomotion, and command-chain compatibility,
but is not evidence of active avoidance.

`social-endurance` is the active oracle-driven command-chain demonstration; it
does not claim perception-only capability. Two analytic,
one-shot pedestrians cross the forward route at separate locations without
waypoint wrapping or teleportation. The robot must yield and resume across two
goals while remaining collision-free for at least 15 seconds. Validation also
requires at least 10 seconds of pedestrian motion, positive body clearance, an
avoidance intervention, at least two seconds where avoidance overlaps moving
pedestrians, at least `0.75 m` of progress after intervention, and no lateral
safety command. The simulation-time actor profile is selected with
`mujoco_pedestrian_scenario:=demo_endurance`.

## Artifacts

The default root is:

```text
ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite/
```

Each run creates:

```text
suite_<UTC timestamp>/
  manifest.json
  status.json
  <case>/
    suite.log
    suite_status.json
    ...case-specific JSON, CSV, YAML, and simulation logs
```

Use `--result-root PATH` to choose another location. The location must remain
inside the repository for cases whose containers write artifacts through the
workspace bind mount.

`manifest.json` captures the exact commands, case descriptions, timeouts, Git
revision and dirty state, seed, preflight results, and official policy hash.
`status.json` is written even after a case failure or interruption.

## Individual Cases

Cases can be combined with repeated `--case` arguments. Interactive cases cannot
be combined with scored cases.

```bash
./scripts/run_demo_suite.sh \
  --case mock-avoidance \
  --case fixed-forward \
  --headless
```

By default, the suite stops at the first failed gate. Use
`--continue-on-failure` when collecting diagnostics from every requested case.
This changes orchestration only; failed cases still remain failed in
`status.json` and the suite exits nonzero.

## Interpretation

The compact mock case uses four people and three goals to remain feasible across
the supported seeded smoke scenarios. It validates deterministic benchmark
mechanics and scoring. It does
not demonstrate MuJoCo locomotion, perception, Nav2, or physical person
avoidance. The live cases use MuJoCo odometry and pedestrian truth for scoring,
but actor replay is not yet deterministic across runs. Keep both distinctions in
reports and demonstrations.
