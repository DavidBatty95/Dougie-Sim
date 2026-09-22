# Human-Aware Navigation Research Plan

> Archived research draft. This is background material, not the current runbook.
> Start with `docs/INDEX.md` and `docs/HANDOFF.md` for maintained instructions.

## Purpose

This document guides continued development of the human-aware navigation work
toward a focused research paper. It is written for both human contributors and
AI coding agents working in this repository.

The target is a simulation-led algorithm paper, supported by limited controlled
Unitree G1 validation if hardware is available. The intended development window
is less than three months. Scope must therefore remain focused on one testable
algorithmic contribution rather than a broad rewrite of navigation, perception,
and locomotion.

This plan does not establish that the proposed method is novel. Before fixing
the paper's claims, compare it against recent work in chance-constrained motion
planning, control barrier functions, velocity obstacles, uncertainty-aware
social navigation, and dynamics-aware safety filters.

## Research Objective

Develop and evaluate an uncertainty-aware, response-constrained command safety
filter for humanoid navigation around moving people.

The proposed filter should combine:

1. Time-indexed pedestrian predictions.
2. Calibrated pedestrian prediction uncertainty.
3. Measured G1 command latency, acceleration, stopping distance, and drift.
4. Explicit robot and pedestrian body geometry.
5. Selection of the least-disruptive command that satisfies a defined risk
   bound, with a stop fallback when no command is admissible.

The central research question is:

> Does accounting for both pedestrian prediction uncertainty and the measured
> response of a humanoid reduce collisions and critical encounters relative to
> geometric and deterministic predictive safety filters without an unacceptable
> loss of navigation success or efficiency?

The primary hypothesis is that the proposed filter improves the safety-efficiency
tradeoff over fixed-radius stopping and deterministic prediction. Secondary
hypotheses are that calibrated uncertainty is particularly useful during
occlusion and motion changes, and that a measured response envelope prevents
unsafe late stops caused by idealized instantaneous-velocity rollouts.

## Intended Contribution

The paper should claim one focused contribution:

> A time-aligned risk-bounded command filter that incorporates calibrated human
> trajectory uncertainty and a measured humanoid command-response envelope.

Supporting contributions may include:

- A reproducible ROS 2 and MuJoCo evaluation protocol for humanoid encounters.
- An empirical comparison between oracle and perception-only operation.
- Simulation-to-hardware validation of the G1 response envelope.

Do not make unsupported claims about formal safety, optimality, human comfort,
real-world readiness, learned intent prediction, or state-of-the-art performance.
A probabilistic guarantee may only be claimed if its assumptions are stated and
its empirical calibration is demonstrated.

## Existing System

Preserve the current command architecture unless a change is required for the
research question:

```text
Nav2 or teleoperation
  -> /cmd_vel
  -> HumanAwareMotionPlanner
  -> /cmd_vel_human_aware
  -> HumanCollisionAvoidance
  -> /cmd_vel_safe
  -> locomotion policy
  -> G1 bridge
  -> MuJoCo or physical G1
```

The principal implementation files are:

- `ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/human_collision_avoidance.py`
- `ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/human_aware_motion_planner.py`
- `ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/dynamic_obstacle_tracker.py`
- `ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/bridge_safety.py`
- `ros2_ws/src/dynamic_obstacle_benchmarks/dynamic_obstacle_benchmarks/benchmark_runner.py`
- `ros2_ws/src/dynamic_obstacle_benchmarks/dynamic_obstacle_benchmarks/metrics.py`
- `ros2_ws/src/dynamic_obstacle_benchmarks/dynamic_obstacle_benchmarks/result_writer.py`
- `scripts/evaluate_person_tracking_offline.py`

Useful current capabilities include typed tracked-person messages, time-indexed
predicted trajectories, separate oracle and perception profiles, seeded actor
courses, benchmark manifests, JSON/CSV/JSONL output, and rosbag capture.

Known limitations that affect the research include:

- Active social demonstrations currently rely heavily on simulator truth.
- Current future covariance propagation omits process-noise growth.
- The collision supervisor primarily uses fixed distances and immediate stops.
- Candidate rollouts assume ideal body velocity rather than measured response.
- Existing evidence is mostly single-seed and contains inconsistent outcomes.
- The benchmark has no multi-seed statistical aggregation.
- Existing TTC is effectively time to closest approach in some code paths.
- Circle-based overlap is not a substitute for MuJoCo contact evidence.
- The physical G1 path is not an independently certified safety system.

## Operating Rules For AI Agents

An AI agent continuing this work must follow these rules:

1. Read this document, `docs/ARCHITECTURE.md`, `docs/DEMO_SUITE.md`, and the
   affected implementation and tests before editing.
2. Inspect the current Git status and preserve unrelated user changes. Never
   revert or overwrite work solely because the worktree is dirty.
3. Work in the priority order defined below. Measurement correctness and
   reproducibility come before planner tuning or visual presentation.
4. Make the smallest change that advances a stated objective. Avoid replacing
   Nav2, the tracker, and the planner simultaneously.
5. Keep oracle data out of runtime perception-only control. Simulator truth may
   be used for labels and metrics, but the result must identify its use.
6. Keep the proposed safety filter independently switchable so all baselines can
   run through the same launch and benchmark infrastructure.
7. Add tests for every new metric, risk primitive, schema field, and fallback
   behavior. Test nominal, boundary, stale-input, and invalid-input cases.
8. Give every metric a definition, unit, aggregation rule, and missing-data
   policy. Do not silently change an existing metric's meaning.
9. Never tune on final evaluation seeds. Maintain separate development,
   calibration, and frozen evaluation seed lists.
10. Do not report a run as evidence merely because it looks successful in RViz.
    Evidence must come from versioned artifacts and a frozen configuration.
11. Do not hide falls, deadlocks, timeouts, perception failures, or collisions.
    Algorithmic failures remain failures; only predefined infrastructure errors
    may invalidate a trial.
12. Do not claim hardware safety. Physical tests require an independent emergency
    stop, a safety observer, a controlled area, and conservative speed limits.
13. Update this document when a major design decision changes. Record the reason,
    expected effect, and implications for previously collected data.
14. End each implementation task by running the narrow relevant tests, then the
    broader workspace checks appropriate to the change. Record any tests that
    could not be run.

## Priority 0: Freeze The Experimental Contract

Complete this before implementing the proposed algorithm.

### Objectives

- Define the methods, scenarios, metrics, seed partitions, and failure rules.
- Make benchmark artifacts machine-validatable and traceable to source/config.
- Correct metric semantics that would undermine the paper.

### Required Work

1. Add a versioned result schema or typed model for aggregate result JSON.
2. Add schema versions to scenario, JSONL, manifest, and status artifacts where
   practical.
3. Record Git revision and dirty state, policy hash, configuration hashes,
   simulator revision, container identity, host timing information, and exact
   command line in or alongside every result.
4. Store the actual live robot start pose rather than a hard-coded origin.
5. Implement deterministic actor reset and trajectory replay, or explicitly
   record why a run cannot be replayed exactly.
6. Correct the current TTC calculation to report first-contact TTC. Preserve
   time to closest approach as a separately named `tcpa` metric.
7. Report DCPA separately from TTC/TCPA.
8. Define proximity exposure as both wall time and person-time. Prevent accidental
   double counting when several people are nearby.
9. Make sampled averages time weighted for live variable-rate data.
10. Add MuJoCo person-contact and static-contact logging.
11. Add map-aware path efficiency or SPL. Keep straight-line efficiency only if
    it is clearly named and still useful.
12. Implement a batch runner accepting a method matrix, scenario matrix, frozen
    seed list, repetition count, timeout, and output root.
13. Implement aggregation with confidence intervals and per-run traceability.
14. Add publication-oriented CSV/LaTeX tables and plotting from result artifacts.

### Definition Of Done

- A tiny two-method, two-seed dry run produces independently valid artifacts.
- Re-running the aggregator produces identical tables from the same artifacts.
- Unit tests establish all revised metric semantics.
- Failed and invalid trials remain visible in aggregate output.

## Priority 1: Characterize G1 Command Response

The safety filter must model the robot that executes commands, not an ideal
holonomic or unicycle body.

### Measurements

For forward and yaw commands, measure:

- command-to-motion latency;
- rise time;
- acceleration and deceleration bounds;
- stopping time and stopping distance;
- post-stop drift;
- steady-state velocity error;
- heading and lateral drift;
- maximum tilt and falls;
- variation across initial speed and repeated trials.

Do not rely on lateral motion unless a locomotion policy passes explicit lateral
tracking and settling gates. The current validated policy should be treated as
forward-and-yaw dominant.

### Model

Implement the simplest conservative model supported by measurements. Preferred
options are:

- command delay plus bounded acceleration/deceleration; or
- first-order velocity response plus a bounded residual position error.

The model must predict a swept robot trajectory and an uncertainty or error
bound. Its parameters must come from a calibration dataset rather than manual
tuning against final navigation trials.

### Definition Of Done

- A versioned calibration dataset exists.
- Fitted parameters and confidence bounds are saved in configuration.
- Held-out simulation trials show conservative stopping-envelope coverage.
- If hardware is available, controlled hardware measurements are compared with
  the simulation envelope before navigation tests begin.

## Priority 2: Calibrate Pedestrian Prediction Uncertainty

Retain constant-velocity prediction initially. A learned trajectory model is out
of scope unless the simpler model cannot support the hypothesis.

### Required Algorithm Work

Propagate position covariance using:

```text
Sigma(t) = F(t) Sigma(0) F(t)^T + Q(t)
```

`Q(t)` should account for prediction horizon and may increase with:

- observed acceleration or Kalman innovation;
- missed detections;
- occlusion duration;
- low track confidence;
- detector or association instability.

Use simulator truth only to estimate and evaluate errors. It must not enter the
perception-only runtime state estimate.

### Calibration Metrics

- Position and velocity RMSE.
- Average and final displacement error by horizon.
- Confidence-ellipse coverage at selected probability levels.
- Negative log likelihood or another proper calibration score if assumptions
  support it.
- Track recall, precision, fragmentation, and actual ID switches.
- Performance stratified by visible, maneuvering, and occluded states.

Fix `scripts/evaluate_person_tracking_offline.py` rather than relying on its
current frame-presence approximations or hard-coded ID-switch result.

### Definition Of Done

- Prediction errors are reported at 1, 2, 3, and 5 seconds where data permits.
- Empirical uncertainty coverage is measured on held-out calibration scenarios.
- Oracle and perception-only results are reported separately.
- No runtime-control topic receives ground truth in perception-only mode.

## Priority 3: Implement The Risk-Bounded Filter

Implement the main contribution in or directly behind
`HumanCollisionAvoidance`. Keep the current planner optional so the filter can
be evaluated independently.

### Risk Calculation

For robot position `p_r(t)`, predicted human position `p_h(t)`, and unit relative
direction `n(t)`, compute directional human uncertainty:

```text
sigma_d(t) = sqrt(n(t)^T Sigma_h(t) n(t))
```

Define conservative clearance:

```text
c(t) = ||p_r(t) - p_h(t)||
       - robot_radius
       - human_radius
       - uncertainty_margin(t)
       - robot_response_bound(t)
```

The uncertainty margin may be `k * sigma_d(t)` or a better justified chance
constraint. If checking many time samples and people, document how total risk is
allocated; do not equate independent pointwise thresholds with a trajectory-wide
guarantee without justification.

### Command Selection

1. Evaluate the requested command using the measured G1 response model.
2. If unsafe, evaluate progressively slower feasible commands.
3. Evaluate a small set of forward-and-yaw alternatives supported by the
   locomotion envelope.
4. Select the admissible command closest to the requested command.
5. Select zero command if no moving candidate is admissible.
6. Remain stopped when required safety inputs are missing or stale.
7. Apply hysteresis so small risk changes do not cause command chatter.

Do not introduce unrequested translation when the upstream command is zero.
Do not allow a safe-patch exception unless the entire time-aligned trajectory is
validated against dynamic and static hazards by both planner and supervisor.

### Diagnostics

Log at least:

- selected candidate and requested command;
- minimum nominal and risk-adjusted clearance;
- pedestrian and time sample producing maximum risk;
- uncertainty and response-bound contributions;
- rejection reason for each candidate class;
- input ages and stale-input state;
- computation time and deadline misses.

### Definition Of Done

- Unit tests cover geometry, directional covariance, body radii, message age,
  response delay, multiple people, no feasible candidate, and stale inputs.
- Tests demonstrate that increased uncertainty cannot make a candidate safer.
- Tests demonstrate that increased response delay or stopping distance cannot
  make a candidate safer.
- The filter can be selected by configuration without changing benchmark code.
- A deterministic synthetic encounter confirms expected intervention timing.

## Priority 4: Baselines And Ablations

Every method must use the same goals, people, initial conditions, locomotion
policy, speed limits, and benchmark definitions.

### Required Baselines

1. `nav2_only`: no custom social planner or final human safety filter.
2. `reactive_geometric`: fixed-distance reactive stopping without prediction.
3. `deterministic_temporal`: current time-aligned prediction with fixed margins.
4. `proposed`: calibrated uncertainty plus measured G1 response.
5. `proposed_with_social_planner`: complete stack compatibility demonstration.

Where disabling a safety layer would create unacceptable hardware risk, run that
baseline only in simulation. Hardware is not required for every baseline.

### Required Ablations

- Proposed method without pedestrian uncertainty.
- Proposed method without the humanoid response model.
- Proposed method without occlusion-dependent uncertainty growth.
- Fixed covariance versus calibrated covariance.
- Oracle tracking versus perception-only tracking.
- Least-disruptive command selection versus immediate stop.

If schedule pressure requires cuts, preserve the first two ablations and the
oracle/perception comparison.

## Priority 5: Scenario Suite

Implement parameterized, replayable encounters rather than one hand-tuned plaza.

### Core Scenarios

1. Perpendicular single-person crossing.
2. Oblique crossing.
3. Head-on encounter.
4. Overtaking a slower pedestrian.
5. Pedestrian emerging from occlusion.
6. Two pedestrians crossing from opposing directions.
7. Dense plaza with four to eight pedestrians.
8. Moving pedestrians mixed with static clutter.
9. Pedestrian stopping or changing direction unexpectedly.
10. Temporary sensor dropout or track occlusion.

Vary pedestrian speed, crossing time, initial separation, noise, latency,
occlusion duration, count, and robot approach direction. Core paper comparisons
should use a manageable subset of approximately five scenario families. Use the
dense plaza as a stress test rather than the only evaluation.

### Seed Partitions

Create immutable lists such as:

- development seeds for implementation debugging;
- calibration seeds for response and uncertainty fitting;
- evaluation seeds opened only after configurations are frozen.

Record seed-list files in version control. Never move a failed final seed into
the development set or replace it without reporting the decision.

## Priority 6: Final Experiment Protocol

The target simulation matrix is:

- 5 principal scenario families;
- 4 primary methods;
- 15 paired seeds;
- 2 repetitions per method/scenario/seed.

This produces 600 principal trials. Reduce repetitions or scenario count only
after a runtime estimate, not by selectively removing difficult cases.

Run a smaller perception-only matrix if necessary, for example:

- 5 scenarios;
- 3 methods;
- 10 paired seeds;
- 2 repetitions.

### Primary Safety Outcomes

- MuJoCo person-contact collision rate.
- Critical encounter rate.
- Minimum body clearance.
- Time below predefined clearance thresholds.
- First-contact TTC, DCPA, and TCPA.
- Empirical risk-bound violation rate.
- Fall rate.

### Task Outcomes

- Goal success and timeout rates.
- Navigation time and path length.
- SPL or map-aware efficiency.
- Delay relative to the corresponding no-person route.
- Deadlock count and duration.

### Motion And Intervention Outcomes

- Filter intervention count and duration.
- Stop count and stopped duration.
- Linear and angular acceleration and jerk.
- Command tracking error and post-stop drift.
- Maximum tilt.

### System Outcomes

- Filter execution latency and update rate.
- Deadline misses.
- Input age and stale-message frequency.
- CPU utilization if measurement is reliable.

### Statistical Reporting

- Preserve per-run values; never report only aggregate scores.
- Report means and medians with dispersion.
- Use 95% confidence intervals.
- Use Wilson or bootstrap intervals for collision and success rates.
- Use paired analysis because methods share scenario seeds.
- Report effect sizes as well as statistical significance.
- Predefine treatment of infrastructure-invalid trials.
- Count collisions, falls, deadlocks, and algorithmic timeouts as failures.

The repository-specific weighted benchmark score may remain a regression aid,
but it must not be the sole or primary paper outcome.

## Priority 7: Controlled Hardware Validation

Hardware validation is secondary to the complete simulation study. Begin with
response characterization, not autonomous human encounters.

Use a mannequin, padded remotely moved obstacle, tracked cart, or another
repeatable non-participant target. Human-participant comfort claims are out of
scope without ethics approval and an appropriate study protocol.

### Minimum Hardware Evidence

- Command latency and stopping distance at several low forward speeds.
- Comparison of hardware response against the fitted conservative envelope.
- Low-speed perpendicular and head-on encounters.
- Sensor-dropout fail-safe behavior.
- Approximately 20 to 30 repeatable controlled encounters if safely feasible.

### Mandatory Safety Conditions

- Independent emergency stop.
- Dedicated safety operator.
- Restricted test area and conservative speed limit.
- Verified command-source exclusivity.
- Pre-test stand or tether validation where appropriate.
- No claims of certified safety or unrestricted human operation.

## Data And Artifact Policy

Final paper evidence must be generated from a frozen revision and configuration.
Each run must retain or reference:

- aggregate result JSON;
- per-goal and per-event CSV;
- time-series JSONL;
- scenario and seed definition;
- experiment manifest;
- method and parameter configuration;
- source, simulator, policy, and container identities;
- stdout/stderr logs;
- rosbag for all failures and a representative successful subset.

Store final aggregate tables, figure source data, and analysis scripts in version
control. Large rosbags may be stored externally, but include checksums and a
machine-readable index. Never overwrite raw final runs. Derived data should be
rebuildable from immutable raw artifacts.

## Paper Figures And Tables

The completed evaluation should be able to generate:

1. System and command-flow diagram.
2. Risk-filter diagram showing human covariance and robot response envelope.
3. Representative encounter trajectories with uncertainty ellipses.
4. Clearance, command, and filter-state timeline.
5. Collision and success rates with confidence intervals.
6. Safety-versus-efficiency comparison.
7. Prediction error and uncertainty coverage by horizon.
8. Main ablation table.
9. Oracle-versus-perception table.
10. Simulation-versus-hardware response-envelope plot.

Figures and tables must be generated from artifact files, not manually copied
from console output.

## Ten-Week Execution Schedule

### Weeks 1-2: Experimental Contract

- Complete the focused literature review.
- Freeze hypotheses, methods, scenarios, metrics, and seed partitions.
- Correct metric semantics and add versioned result validation.
- Implement the batch runner and statistical aggregation skeleton.

### Weeks 3-4: Models

- Collect simulated G1 command-response data.
- Fit and validate the conservative response envelope.
- Implement pedestrian process-noise growth.
- Upgrade offline tracking and calibration evaluation.

### Weeks 5-6: Main Algorithm

- Implement risk-bounded command checking and selection.
- Add stale-input and fallback behavior.
- Add diagnostics and unit/integration tests.
- Verify behavior in deterministic synthetic encounters.

### Week 7: Baselines And Pilot Study

- Freeze independently selectable baseline configurations.
- Run development and calibration seeds.
- Correct defects and tune only on non-evaluation seeds.
- Freeze all parameters before final evaluation.

### Weeks 8-9: Final Evidence

- Run the paired simulation matrix.
- Run the perception-only matrix.
- Conduct controlled hardware characterization and encounters if safe.
- Generate statistics, tables, figures, and a failure audit.

### Week 10: Paper And Artifact Review

- Write the method and experimental protocol.
- Document limitations and negative results.
- Verify every paper number against generated artifacts.
- Prepare a synchronized MuJoCo/RViz video and reproduction instructions.

## Decision Gates

### Gate A: Measurement Ready

Proceed to algorithm evaluation only when metrics are semantically correct,
schemas are versioned, and batch aggregation preserves failures.

### Gate B: Models Validated

Proceed to final tuning only when the response envelope is conservative on held-
out trials and prediction uncertainty has measured coverage.

### Gate C: Configuration Frozen

Proceed to final seeds only when all methods, parameters, scenario definitions,
and exclusion rules are frozen.

### Gate D: Algorithm-Paper Viability

Keep the focused algorithm framing only if the proposed method shows:

- fewer collisions or critical encounters than deterministic filtering;
- an effect that is statistically and practically meaningful;
- acceptable goal success and navigation delay;
- calibrated uncertainty behavior;
- benefit under perception-only input, not only oracle truth;
- hardware response contained by the model when hardware tests are available.

If the method improves safety only by remaining stopped, or improvement exists
only with oracle tracking, do not force the algorithm claim. Pivot to a systems
or benchmark paper centered on reproducible humanoid social-navigation evaluation
and the measured oracle-to-perception gap.

## Implementation Progress

### 2026-08-27: Priority 0 Metric Semantics, First Slice

Completed:

- Corrected `time_to_collision` to report constant-velocity first-contact time
  for circular bodies rather than time to closest approach.
- Added horizon-limited DCPA and its associated TCPA as separate metrics.
- Changed TTC calculations to use each observed pedestrian's body radius.
- Separated proximity wall time from person-time exposure while retaining the
  existing `time_below_*` fields as explicit person-time compatibility aliases.
- Changed proximity-duration scoring to use wall time, preventing simultaneous
  nearby people from multiplying a trial-duration penalty.
- Changed mean center distance and body clearance to use time weighting.
- Added validation for distance-margin and TTC-threshold ordering.
- Added benchmark and scenario `schema_version: 1`; the result validator rejects
  missing, incorrectly typed, or unsupported schema versions.
- Added event-level TTC, TCPA, and DCPA output and expanded regression tests.
- Superseded the schema-v1 artifact contract with schema v2, separating requested
  source labels from runtime-observed provenance and binding companion artifacts
  to a shared run identifier and SHA-256 checksums.

Verification:

- 46 non-launch `dynamic_obstacle_benchmarks` tests pass.
- 14 demo-suite and benchmark-validator tests pass.
- A two-person, two-goal mock smoke run generated schema-v1 artifacts and passed
  the benchmark validator with a score of 98.434.
- Python compilation and `git diff --check` pass.
- The ROS launch test could not be collected in the host shell because the ROS 2
  `launch.actions` module is unavailable there; run it in the project container
  during broader workspace verification.

Still required for Priority 0:

- Formal machine-readable schema validation beyond the version field.
- Actual live robot start-pose capture.
- Deterministic actor reset and replay.
- MuJoCo person and static-contact logging.
- Map-aware efficiency or SPL.
- Complete source, configuration, simulator, container, and host provenance.
- Publication-oriented plotting and table generation.

### 2026-08-27: Priority 0 Batch Execution And Aggregation

Completed:

- Added a versioned YAML experiment specification for methods, scenarios, seeds,
  repetitions, timeouts, invalid-trial policy, and aggregate metrics.
- Added `scripts/benchmark_batch.py` and `scripts/run_benchmark_batch.sh` with
  deterministic matrix expansion, argv-only process execution, process-group
  timeout handling, immutable trial directories, and explicit outcome classes.
- Added `scripts/aggregate_benchmark_batch.py`, which enumerates only manifest
  trials and retains unstarted, malformed, unsupported, invalid, and failed runs.
- Added Wilson intervals for binary rates, deterministic percentile-bootstrap
  intervals for continuous means, and seed-paired method-minus-reference
  comparisons after averaging repetitions within each seed cell.
- Added `runs.csv`, `groups.csv`, `paired_differences.csv`, and `aggregate.json`.
- Added usage and statistical definitions in `docs/BENCHMARK_EXPERIMENTS.md`.
- Added host-side orchestration tests to non-fast workspace verification.
- Added explicit SHA-256 capture for configured files, commit and dirty-state
  capture for configured repositories, and Docker image identity for configured
  container services. Required missing provenance files fail before execution.

Verification:

- 9 batch and aggregation tests pass, including byte-identical re-aggregation.
- 23 host-side batch, aggregation, demo-suite, and validator tests pass together.
- 46 non-launch benchmark-package tests continue to pass.
- A Docker-backed matrix of two identical mock methods and two paired seeds ran
  to completion. Both methods reproduced the same two crossing collisions; the
  batch retained all four algorithmic failures, reported collision rate `1.0`
  with Wilson intervals, and reported zero paired method differences.
- New Python files pass Flake8 with the repository's practical 120-character
  limit, Python compilation, and `git diff --check`.

Remaining batch-infrastructure work includes resumable immutable attempts,
review of final provenance declarations, and publication plots. These are not
prerequisites for beginning the G1 response-calibration protocol, but provenance
review must be complete before final paper experiments.

### 2026-08-27: Live Output, Baseline Routing, And Response Protocol

Completed:

- Added exact `--output` support to both live benchmark wrappers while preserving
  timestamped `--result-dir` behavior.
- Added strict host/container path mapping under the writable `ros2_ws` mount;
  unsupported external paths and conflicting output modes now fail early.
- Derived reports, simulation logs, JSONL, bags, and benchmark companion paths
  from the exact JSON stem.
- Exposed the collision supervisor input command topic in `g1_sim.launch.py` so
  supervisor-only baselines can consume `/cmd_vel` without an external process.
- Added method-specific parameter profiles after the independent sensing profile
  for reactive-geometric and deterministic-temporal filters.
- Added the development-only `oracle_method_development_v1.yaml` matrix covering
  Nav2 only, reactive geometric, deterministic temporal, and the existing full
  planner/supervisor on two development seeds.
- Added `docs/G1_RESPONSE_CALIBRATION_PROTOCOL.md` with the command matrix,
  metric definitions, split policy, hardware safeguards, fitting rules, and
  held-out acceptance gates.
- Added a machine-readable response dataset contract at
  `docs/schemas/g1_response_calibration.schema.json`.

The oracle method matrix has only been dry-run. It must not be described as a
quantitative comparison until live development runs verify routing and method
identity. It is explicitly not perception-only evidence.

### 2026-08-27: Response Calibration Acquisition

Completed:

- Hardened `measure_g1_fixed_command.py` as the single-owner calibration command
  publisher instead of introducing a second competing command node.
- Added settle, command, stop, and cleanup phase labels plus a repeated exact-zero
  cleanup burst.
- Changed primary response measurement to authoritative `/odom` twist while
  retaining pose-derived velocity as an independent diagnostic.
- Added one-row-per-odometry-message CSV acquisition with source timestamp,
  monotonic arrival time, commanded velocity, pose, reported and derived twist,
  tilt, and fall state.
- Added odometry and diagnostic freshness checks, finite argument validation,
  sustained onset detection, start-heading displacement, strict JSON output, and
  SHA-256 identity for raw samples.
- Updated `run_g1_fixed_command_test.sh` to create a uniquely named raw sample
  artifact beside every response summary.

No new response data has been collected yet. Existing fixed-command results were
generated under older sampling semantics and must not be mixed with the new
calibration dataset without an explicit migration and compatibility analysis.

### 2026-08-27: Offline Response Extraction And Frozen Splits

Completed:

- Added `extract_g1_response_metrics.py` for pure offline extraction from the
  timestamped acquisition CSV. It reports signed-direction onset and rise,
  steady response and absolute/relative error, stop time and displacement,
  post-stop planar/yaw drift, base height, tilt, falls, and data health.
- Froze a 0.10-second trailing-mean velocity filter and included its identity in
  every trial result.
- Added stamped `/g1/policy_cmd_vel` publication after policy-input deadband,
  saturation, axis handling, and rate limiting. Acquisition records this
  effective command, base height, and exact source-time transitions. The sample
  file is now closed before its SHA-256 is computed.
- Extended the response schema with unit-explicit linear/yaw metrics and explicit
  post-stop horizon availability, using Draft 7 for the repository's installed
  validator.
- Added `assemble_g1_response_dataset.py`, which validates trial inputs, rejects
  mixed identity or provenance, rejects duplicate trial IDs, and emits a
  deterministic combined dataset.
- Added a frozen phase-1 command file with disjoint calibration and held-out
  seeds, explicit per-seed randomized execution order, timing, and filter
  settings.
- Added focused extraction tests and an end-to-end synthetic three-command pilot
  covering `0.10 m/s`, `0.20 m/s`, and `0.10 rad/s` responses.
- A live `0.10 m/s` gate trial showed that simulation `/odom.twist` remains zero
  while exact pose moves. The calibration source is therefore frozen as the
  independently pose-derived body twist; reported twist remains diagnostic.

The pilot is synthetic validation of extraction semantics, not G1 response
evidence. No new MuJoCo or hardware response data has been collected. The live
pilot must also verify command-source exclusivity and the stamped effective
policy-command stream.

### 2026-08-27: Live Response Instrumentation Gate

- Ran an instrumentation-only three-command development sequence at `0.10 m/s`,
  `0.20 m/s`, and `0.10 rad/s`. This first sequence used the extreme-lab scene
  and is explicitly excluded from calibration evidence.
- The sequence exposed zero-valued simulation `/odom.twist`; extraction now
  differentiates exact pose over a frozen source-time window and retains the
  reported twist only as a diagnostic.
- Fixed sustained-window detection for nonuniform sample rates, exact policy
  command transition timing, callback ordering at phase boundaries, and base
  height handling when only a zero-height `base_footprint` pose is available.
- Added an effective policy-command stream after controller hold checks, a
  continuous stationary/tilt settle gate, and the required empty flat scene as
  the acquisition default.
- Completed the homogeneous empty-scene pilot at `0.10 m/s`, `0.20 m/s`, and
  `0.10 rad/s`; all three trials passed acquisition, retained all drift horizons,
  and had no fall or dropped samples.
- Added exact `/g1/base_height` publication and repeated all pilot commands after
  instrumentation. Minimum observed heights were `0.755`, `0.752`, and
  `0.759 m` respectively.
- Replaced gait-noise-dominated peak derivatives with effective acceleration
  from the 10%-to-90% rise interval and effective deceleration from steady
  response and stop time.
- Forward `0.20 m/s` tracked at `0.185 m/s` with `0.501 s` latency and `0.713 s`
  stop time. Forward `0.10 m/s` tracked at only `0.069 m/s`. The `0.10 rad/s`
  yaw command tracked at only `0.031 rad/s` with `0.990 s` stop time. These are
  development observations from one repetition, not fitted bounds.
- Both forward trials failed the predeclared steady-window stability flag; the
  yaw trial passed it. This behavior must be checked across repetitions rather
  than suppressing or retuning the flag from one pilot.

### 2026-08-27: Response Pilot Repeatability And Gait-Aware Stability

- Collected a second independent simulation repetition for each homogeneous
  empty-scene pilot command and processed all six trials with raw-data hash
  verification and separate robot-model, scene, configuration, policy, repository,
  and simulator provenance.
- Added `process_g1_response_trial.py` for reproducible trial processing and
  `summarize_g1_response_dataset.py` for deterministic command-level repeatability
  summaries.
- Confirmed persistent low-command under-tracking. Across the two repetitions,
  median steady response was `0.072 m/s` for `0.10 m/s`, `0.190 m/s` for
  `0.20 m/s`, and `0.031 rad/s` for `0.10 rad/s`. The corresponding between-run
  ranges were `0.006 m/s`, `0.008 m/s`, and `0.001 rad/s`.
- Confirmed that the prior forward stability failures measured within-gait body
  oscillation rather than between-cycle instability. Froze a separate `0.80 s`
  gait-cycle trailing mean for the stability gate while retaining the `0.10 s`
  filter for onset, rise, steady response, and stopping. The gait-aware p90-p10
  spreads are emitted explicitly; all six pilot trials pass.
- All six trials are evaluable, fall-free, and free of no-motion outcomes. The
  largest observed repeatability concern is `0.20 m/s` stopping time, which spans
  `0.419` to `0.713 s`; the full calibration must retain this variation rather
  than relying on a nominal stop response.
- Added focused tests for gait-periodic response, processor timestamp/path
  handling, stricter scene provenance, and repeatability grouping.

These revisions were made on development pilot data before full calibration or
held-out collection. The pilot is sufficient to proceed with specification work,
not to fit an envelope or make a coverage claim.

### 2026-08-27: Response-Envelope Fitting And Acceptance Freeze

- Added the normative fitting specification in
  `docs/G1_RESPONSE_ENVELOPE_FITTING_SPEC.md` and its machine-readable authority,
  `g1_response_envelope_fit_v1.yaml`.
- Froze a calibration-only binwise extreme estimator with leave-one-seed-out
  extreme-spacing slack. The reserve is explicitly an engineering estimator, not
  a claimed probabilistic bound.
- Froze cumulative adverse shape constraints, conservative adjacent-knot step
  interpolation, separate positive/negative yaw branches, zero mapping below the
  minimum nonzero command, and complete prohibition of extrapolation.
- Froze joint complete-trial coverage as the acceptance unit. Each command family
  must achieve a one-sided 95% Clopper-Pearson lower confidence bound of at least
  0.90 with at least 29 evaluable held-out trials.
- Expanded the unopened held-out split from three to five seeds. This provides 30
  planned trials for each of the forward and yaw families; the original 18 per
  family could not meet the declared confidence criterion even with no failures.
- Added `validate_g1_response_fitting_spec.py`, which rejects split overlap,
  incomplete command orders, unknown fitting metrics, inconsistent declarations,
  and statistically insufficient held-out plans.
- Froze failure handling: unsupported calibration knots reduce the command domain;
  held-out failure rejects the envelope; a revised estimator requires a new
  protocol version and new unopened held-out seeds.

The held-out seeds remain unopened. Calibration collection may begin after final
artifact-path and execution checks.

## Immediate Next Tasks

The next agent should begin with these tasks in order:

1. Collect the seven-seed calibration response matrix, fit the frozen envelope,
   and record its immutable hash before opening any held-out seed.
2. Run one development seed per oracle baseline to verify topic routing before
   any larger method comparison.
3. Add resumable batch attempts without overwriting prior attempt artifacts.
4. Review and freeze the required provenance declarations for each final method.
5. Conduct the literature review and maintain the closest-baseline matrix in
   parallel with implementation.
6. Only after response and uncertainty models pass review, implement the proposed
   risk filter.

At the end of each task, update the relevant definitions and completion state in
this document so another contributor can continue without reconstructing design
decisions from chat history.
