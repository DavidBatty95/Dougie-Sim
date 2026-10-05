# Socially aware G1 indoor delivery: staged roadmap

Updated 2026-10-05. This is the active implementation roadmap; the
[transport plan](TRANSPORT_DEMO_PLAN.md) retains the historical experiment ledger.

## Target and acceptance scope

Target hardware: **Unitree G1 with BrainCo hands**. First accepted mission:
repeated indoor table-to-table delivery around people, including yielding,
blocked routes, workspace intrusion and autonomous recovery. Specify the exact
hand model/API, sensors, compute, object sizes/masses, floor, lighting and aisle
limits before physical acceptance. General unknown-environment autonomy and
human handover are later extensions.

The robot must discover the requested object and destination from measurements,
select feasible approaches, physically grasp, carry, release and independently
verify delivery. Normal disturbances must not require a mission restart.
Unrecoverable conditions must produce a truthful, bounded safe-hold/assistance
state. No base pinning, object teleportation or mission use of evaluation truth.

## Assessed baseline (retained evidence, not a new validation run)

- Sensor-driven, unpinned transport has documented zero-person native simulation
  passes using a payload-trained policy and capture-radius weld attachment.
  The payload report documents two; two additional local successful artifacts
  exist. These do not form a frozen four-trial reliability cohort.
- Latest full navigation matrix v84: **10/18 strict passes, 46/54 goals, six
  reported contacts**. v85 confirmed geometric overlaps in audited failures.
- A later four-person transport run stopped during reaching without resuming.
- BrainCo articulation/contact-bench work exists; prescribed wrist bench motion
  does not establish mobile physical grasping. The bench and transport fixture
  use different object sizes (5 cm versus 11 cm).
- New sensor-driven transport bypasses the older MoveIt/action orchestration
  stack. Neither stack yet supplies the complete target mission.
- Physical G1 operation is unvalidated. The dirty working tree, partial manifests
  and historical test reports are not a clean-checkout acceptance release.

Evidence: [payload training](PAYLOAD_POLICY_TRAINING.md),
[v84](results/avoidance_v84_matrix.json),
[v85](results/avoidance_v85_alignment_audit.json),
[architecture](ARCHITECTURE.md).

## Architecture to converge on

Task request → persistent mission executive → uncertainty-aware semantic world
model → navigation/docking/manipulation actions → coordinated base/arm execution
→ final collision and command-authority supervision → locomotion policy, arm
controller and BrainCo controller → simulation or hardware adapters.

- One mission/action lifecycle, with explicit preconditions, postconditions,
  cancellation, bounded retries and recoverable checkpoints.
- Enforced command ownership, readiness-driven startup and measured health.
- Shared timestamped geometry, uncertainty and executable-motion contracts
  between planner and supervisor; independent final command validation.
- Actual articulated arm/payload swept geometry, not just a carrying-radius flag.
- Hand interfaces express supported actuator commands and measured status;
  object-ID weld services remain explicitly labelled development fixtures.
- Acquisition stamps, frame, confidence and validity accompany estimates.
- Simulator truth stays in training/evaluation, outside deployed decisions.

## Stages and promotion gates

| Stage | Work | Required exit evidence | Status |
| --- | --- | --- | --- |
| S0 Reproducible baseline | Full source/submodule/config/model/policy/build identity; effective parameters; reconcile documentation; native vs mock verification tiers | Clean rebuild reproduces baseline; all attempts retained; source and installed-code identities agree | Open |
| S1 Measurement and lifecycle correctness | Uncertain attachment; post-release evidence; cancel-and-await on all exits; timestamp continuity; evaluator health/contact coverage; command ownership; response/latency calibration | Fault injection rejects false success; bounded termination; measured stopping/prediction envelopes and replayable command decisions | In progress: first bounded fixes only |
| S2 Reliable navigation | Reproduce v85; isolate tracking/model/tactical/veto failures; shared motion contract; earlier avoidance; bounded side commitment; occlusion speed control; runtime profiling | Existing six-world/three-seed matrix passes every strict check, then held-out seeds and encounters; thresholds unchanged | Open; gates crowd-transport promotion |
| S3 BrainCo physical grasp | Confirm hardware/API; calibrate coupling/limits/friction; choose feasible objects; contact-only grasp/lift/hold/release; collision-aware arm approach; measured grasp evidence | Repeated contact-only grasps with no unintended collisions or weld; perturbation/offset/material tests; real-hand comparison when available | Open; isolated work may proceed alongside S2 |
| S4 Recoverable mission | Consolidate orchestration; wait/reobserve/redock/regrasp/replan/safe-hold states; reproject after base motion; destination occupancy; translational search | Recoverable fault scenarios resume; unrecoverable scenarios preserve known state; deadlines remain bounded | Open |
| S5 Smooth loaded transport | Loaded swept geometry; arm/load transitions; noise/delay/dynamics variation; reverse/stop training regressions; precision docking; target-compute validation | Repeated empty-room deliveries across objects/layouts without falls, drops, unintended contacts or resets; declared motion/placement tolerances met | Open |
| S6 Social transport | Crossings, head-on/overtaking, doorways, occluded emergence, workspace intrusion, occupied destinations, groups and repeated missions | Held-out complete deliveries; valid sensing; zero prohibited contacts/falls/drops; measured yielding/resumption, deadlock and human comfort | Open; requires S2–S5 |
| S7 Physical deployment | Same interfaces through calibrated hardware adapters; timing/actuation limits; watchdogs; controlled progression from supported motion to human encounters | Relevant acceptance suite passes on physical G1 within a declared operating envelope | Open |

Stage order is a promotion order, not permission to claim later-stage success
from isolated component work. Freeze each acceptance candidate before measuring
reliability. Retain invalid runs and failures; distinguish development attempts
from comparable cohorts. Commits/pushes remain explicit owner actions.

## First implementation slice: S1 attachment and placement correctness

Scope:

1. Represent attachment as `empty`, `uncertain` or `held`.
2. Mark a dispatched grasp/release uncertain until acknowledged. A lost or
   rejected acknowledgement must not clear possible possession. Do not retry an
   ambiguous grasp or release automatically.
3. Initialize payload geometry before dispatching grasp and retain conservative
   carrying geometry while attachment is uncertain.
4. Report both confirmed possession and possible possession in telemetry/results.
5. Require a fresh box observation acquired strictly after release acknowledgement
   before accepting visual placement. This is a necessary check, not independent
   proof of release, support, or grasp stability.
6. Add regression tests for lost/rejected acknowledgements, failure before
   dispatch, carrying-state publication and stale/pre-release visual evidence.

Next S1 slices: terminal action cancellation independent of expired mission
deadlines, continuous evaluator health, actual contact accounting, and complete
runtime provenance. Then measure and unify planning/supervision/execution.
Resumable recovery and physical finger grasping remain later gates.

## Verification and metrics

Track task completion, interventions/mission-hour, recovery success, wrong
destinations; contacts by body part and impulse where available; clearance,
drops/falls; acceleration/jerk, reversals, stop/start count and docking settling;
yield timing, front cuts, passing clearance, group intrusion, human delay and
resumption; perception coverage/identity continuity/acquisition age; latency
tails/deadline misses; grasp retention and placement/release error.

Use targeted lifecycle regressions for S1, native deterministic scenarios for
integration, full matrices for promotion, and held-out conditions for acceptance.
Mock tests do not establish physical behavior. Smoothing must stay within measured
stopping/avoidance constraints. A few successful examples are not reliability
estimates: approximately 299 independent representative zero-failure trials are
needed for a one-sided 95% upper failure-probability bound below 1%.

## Implementation ledger

- 2026-10-05: roadmap adopted for BrainCo-equipped indoor delivery. First S1
  attachment/placement slice implemented in `transport_demo.py`: explicit
  `empty`/`uncertain`/`held` states, conservative carrying/payload publication on
  ambiguous service outcomes, attachment state in result/heartbeat telemetry,
  and fresh post-release-acknowledgement visual placement evidence.
- Verification: **55 focused tests passed**, comprising 19 new lifecycle
  regressions plus transport completion, geometry and manipulation IO tests.
  Tests ran against workspace source in the ROS Docker image with the existing
  `.verification/install` overlay for generated messages. Host collection failed
  because those container-built message packages were unavailable at host paths;
  the container run resolved that environment issue. Documentation link checks
  and `git diff --check` passed. No new native simulation or hardware run was
  performed; S1 as a whole remains open.

Reproduce the focused tests from the repository root:

```bash
docker compose run --rm --no-deps unitree-g1-sim bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/.verification/install/setup.bash &&
  export PYTHONDONTWRITEBYTECODE=1 &&
  export PYTHONPATH="/workspace/ros2_ws/src/unitree_g1_mujoco_bridge:/workspace/ros2_ws/src/dynamic_obstacle_benchmarks:$PYTHONPATH" &&
  python3 -m pytest -p no:cacheprovider \
    src/unitree_g1_mujoco_bridge/test/test_transport_lifecycle.py \
    src/unitree_g1_mujoco_bridge/test/test_transport_completion.py \
    src/unitree_g1_mujoco_bridge/test/test_transport_geometry.py \
    src/unitree_g1_mujoco_bridge/test/test_sim_manipulation_io.py'
```

Prerequisite: the ROS verification overlay has been built, including the
`GraspObject` service. This command verifies source-level behavior; it does not
rebuild or qualify an installed native transport executable.
