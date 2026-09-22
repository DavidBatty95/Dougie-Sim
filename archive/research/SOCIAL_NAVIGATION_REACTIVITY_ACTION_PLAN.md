# Social-navigation reactivity review and action plan

**Baseline review:** this document describes the source before the upgrade. See
[Social navigation upgrade](SOCIAL_NAVIGATION_UPGRADE.md) for the implementation,
new waypoint demo, source provenance, and validation record.

## Summary

The code has a predictive navigation pipeline, but several implementation and
configuration mismatches prevent it from behaving proactively. Fix tracking
initialization, early-risk delivery, and executable robot rollouts before tuning
prediction horizons or adding a learned predictor.

This review covers the current working-tree source. It includes direct execution
of the command-deadband and turning-arc helpers, but no live simulation or measured
end-to-end latency. Runtime frequency and compute bottlenecks remain hypotheses
until instrumented. Source line references below describe the reviewed revision.

## Establish which demo is being evaluated

- `scripts/run_collision_avoidance_demo.sh` uses the conservative perception
  profile, disables the separate LiDAR person detector, and caps policy commands
  at 0.05 m/s forward, zero lateral, and 0.10 rad/s yaw. Person detection therefore
  depends on the tracker's geometric point-cloud processing.
- `scripts/demo_suite.py` selects simulator-truth tracking for `social-visual`
  and `social-endurance`, disables tracker sensor input, and uses the official
  policy. Tracking receives true pedestrian velocity in this mode. This is a
  planning/command-chain baseline, not evidence that LiDAR tracking works.

Record the exact launcher, overrides, effective ROS parameters, policy hash, seed,
and source provenance with every run. Compare oracle and perception operation on
matched actor courses and robot command envelopes.

## Findings, in priority order

### 1. The perception demo's straight command is swallowed by its deadband

**Evidence:** `scripts/run_collision_avoidance_demo.sh:50-52`,
`g1_sim.launch.py:1077-1079`, `policy_safety.py:49-58`.

The forward clamp is 0.05 m/s. The deadband enters stop at magnitude <= 0.05 and
releases only above 0.06. Direct execution confirms that `[0.05, 0, 0]` becomes
zero both from rest and while moving. A sufficiently large yaw command can escape
this vector-wide deadband, producing inconsistent straight/turn behavior.

**Action:** Select one measured policy envelope and share its bounds across Nav2,
the planner, the supervisor, and the policy. Ensure executable creep commands are
above the measured motion-onset threshold and the software release threshold.
Do not increase locomotion limits beyond measured capability just to mask this.

### 2. The tracker can prevent itself from learning walking velocity

**Evidence:** `dynamic_obstacle_tracker.py:1452-1469,1494-1548`.

Tracks start with zero velocity. `_match_track()` derives velocity from a new
cluster minus the filtered track position, then gates the Kalman update using
the implied acceleration. If rejected, it still advances the timestamp without
updating position for ordinary LiDAR clusters.

For illustration, a person moving 0.10 m between samples 0.10 s apart implies
1 m/s, but the initialization gate calculates 10 m/s² against a 2.8 m/s² limit.
The update is rejected. The next sample can be compared against an even older
position over another short timestamp interval. Confirmed-track association uses
the same noise-sensitive acceleration calculation and can fragment tracks.
Oracle measured-velocity injection bypasses the update problem, so an oracle
showcase can hide it.

**Action:** Bootstrap velocity from timestamped measurements over a short window;
use innovation/covariance gating rather than differentiating filtered position
error twice. Keep measurement time, state time, and last-observed time distinct.
Reject a measurement without pretending the old state was updated. Then assess
whether greedy assignment should become global minimum-cost assignment for
crossing pedestrians.

**Regression cases:** Constant-speed walkers at multiple sensor rates; noisy
centroids; motion onset; abrupt turn/stop; two crossing walkers; missing samples.
Check velocity convergence, position error, ID continuity, and reacquisition time.

### 3. Tentative people do not reach the default planning path

**Evidence:** `dynamic_obstacle_tracker.py:870-896`,
`human_aware_motion_planner.py:433-447,582-588`,
`g1_sim.launch.py:531,558-562`.

The tracker skips tentative tracks before populating typed tracked-person output.
The planner independently rejects tentative typed states. Legacy tentative-cloud
consumption is disabled, and tentative-cloud publication is also disabled in the
launch configuration. Consequently, `immediate_person_risk=true` does not provide
an early-risk input to this planner path. Confirmation requires at least 0.20 s
plus hit, confidence, shape, and motion/persistence gates.

**Action:** Publish tentative typed tracks with honest confidence and covariance.
Use plausible near-path tentative detections for bounded early deceleration and
short-horizon occupancy checks. Retain stronger confirmation for persistent
identity and assertive passing decisions. Test false positives as well as recall.

### 4. The planner scores trajectories the robot will not execute

**Evidence:** `human_aware_motion_planner.py:650-697,1026-1036`,
`g1_policy_cmd_vel_controller.py:885-903,925-955`,
`scripts/demo_suite.py:97-115`, `policy_safety.py:90-102`.

Rollouts apply each candidate velocity instantly and use no measured robot
velocity, command-response delay, acceleration, stopping drift, or downstream
command transformation. The conservative planner permits lateral motion while
the perception demo's policy clamp removes it. Its forward/yaw limits also differ.

The oracle profile disables lateral candidates, but its policy still applies a
0.20 m/s turning-arc floor. Direct helper execution converts `[0, 0, 0.11]` into
`[0.20, 0, 0.12]` under the showcase settings. A scored turn-only trajectory is
therefore not necessarily turn-only when executed.

**Action:** Share a command-admissibility/normalization model between planning and
execution. Normalize candidates before scoring, reject unsupported motions, and
roll out from measured robot velocity using calibrated command delay,
acceleration, braking, and drift. Reuse the existing G1 response-calibration work.
Zero commands bypass software slew limiting today; that is not instantaneous
physical stopping.

### 5. Normal navigation does not search early avoidance alternatives

**Evidence:** `human_aware_motion_planner.py:606-697,946-966`.

In `NORMAL_NAVIGATION`, only `raw_limited` is evaluated. The trajectory scorer can
detect a conflict over its five-second horizon, but if the state logic has not
entered caution, there are no alternatives: rejection becomes an emergency stop.
The state logic uses a three-second closest-approach warning and other geometric
conditions, rather than the same full-horizon feasibility check.

**Action:** Evaluate a small executable candidate set whenever predicted occupancy
intersects the robot's reachable corridor. Include gradual slowing, yielding,
forward arcs, and passing behind where feasible. Use consistent time-aligned risk
for both mode selection and trajectory scoring; add entry/exit hysteresis and
side commitment. Distinguish time to closest approach from time to first body
clearance violation.

### 6. The perception profile can bypass temporal planning with spatial avoidance

**Evidence:** `human_aware_motion_planner.py:544-552,730-815`,
`g1_sim.launch.py:634-639`, `social_navigation_oracle_v1.yaml:12`.

The enabled local-safe-space branch collapses all future person samples into a
spatial danger set and can return a nearest-clear-patch command before the normal
time-aligned scorer runs. This loses the distinction between a person occupying
a location now and several seconds later. The oracle profile already disables
this branch.

**Action:** Run a controlled perception ablation with this branch disabled. Make
space-time scoring the normal authority and reserve a reachable, time-checked
escape routine for actual emergencies. Do not flatten an entire predicted path
into a simultaneously occupied region.

### 7. Accumulation and lost-track handling can create lag or ghosts

**Evidence:** `dynamic_obstacle_tracker.py:735-753,902-938,2085-2095`,
`g1_sim.launch.py:513-518,536-561`.

The launch uses a 0.20 s rolling accumulation window. Points from different times
are combined and the resulting cluster uses the latest timestamp. This can smear
moving people and bias their centroid behind their current position; it is not a
fixed 0.20 s blocking delay. The declared accumulation-motion-compensation and
yaw-rate parameters are stored but not used by the accumulation implementation.
Existing transforms into world coordinates do not remove moving-person smear.

Lost tracks can persist six seconds, while position coasting is capped at two
seconds and output is stamped with the current sample time. Consumers cannot
recover the original observation age from that stamp. The low-speed prediction
fallback can also use lifetime displacement direction after a person stops.

**Action:** Evaluate shorter/adaptive accumulation against range-dependent recall;
preserve effective measurement times. Carry last-observed age explicitly, grow
process uncertainty during occlusion, and avoid frozen-but-fresh lost states.
Start with a well-calibrated constant-velocity predictor; consider a small
stopped/walking/turning model only after tracking is reliable.

### 8. The supervisor and current evaluation can obscure root causes

**Evidence:** `human_collision_avoidance.py:498-579,952-989`,
`scripts/evaluate_person_tracking_offline.py:90-150`.

The supervisor includes geometric stopping, temporal prediction checks, a hold
timer, and post-planner open-space bias. A hold or changed output can make planner
behavior look hesitant. Temporal checking is callback-driven, so verify the age
of the risk evaluation when commands change. Expired person data also needs an
explicit degraded mode: the planner currently treats expired tracks as an empty
set, and supervisor force expiry alone does not establish perception health.

The offline evaluator currently counts nonzero safe commands as overrides rather
than comparing input and output, and initializes an ID-switch estimate without
updating it. These metrics cannot establish reactivity or identity continuity.

**Action:** Revalidate the final executable command against the latest aligned
state each control tick. Use one explicit reason for each intervention and
measured hysteresis for release. Log source loss separately from an observed
empty scene. Correct the evaluator before using it for tuning decisions.

## Implementation sequence and acceptance gates

### P0 — Reproduce and measure

1. Capture matched oracle/perception baseline runs with one crossing pedestrian.
2. Log source capture time, callback receipt, tracking completion, prediction
   epoch, planner start/end, supervisor output, policy input, and measured motion.
3. Record all command stages: `/cmd_vel`, `/cmd_vel_human_aware`, `/cmd_vel_safe`,
   `/g1/policy_cmd_vel`, plus odometry, TF, tracks, predictions, and debug reasons.
4. Fix offline matching and metrics: first-detection/confirmation latency, velocity
   error, ID switches, prediction error at 0.5/1/2 s, and intervention lead time.

**Gate:** A missed avoidance can be attributed to sensing, tracking, planning,
command alteration, or physical response rather than only a visual impression.

### P1 — Repair correctness blockers

1. Fix the deadband/envelope mismatch and normalize planner candidates.
2. Repair tracker velocity initialization and acceleration gating.
3. Deliver tentative risk through typed messages.
4. Make full-horizon conflicts trigger candidate search before emergency fallback.

**Gate:** Constant-speed tracks converge without repeated identity resets;
tentative risk reaches the planner before confirmation; scored commands match
commands accepted by the policy; early conflicts have feasible alternatives.

### P2 — Make prediction and action physically consistent

1. Integrate measured response envelopes into planner and supervisor rollouts.
2. Correct lost-track time semantics and uncertainty propagation.
3. Compare temporal planning against the local-safe-space branch.
4. Remove redundant command alterations or include them in final risk evaluation.
5. Profile before changing rates. The launch already requests 20 Hz planning and
   supervision, and the tracker already uses depth-one sensor QoS. Measure actual
   callback time and queue age; move visualization off the critical path if needed.

**Gate:** Predicted robot motion and measured motion agree within a documented
envelope, including stop and turn transitions; delayed/occluded observations
produce explicit uncertainty and degraded behavior rather than fresh certainty.

### P3 — Demonstrate proactive behavior across encounters

Use fixed development seeds and separate held-out seeds. Progress through:

- No-person navigation and false-positive controls.
- Left/right crossings at several speeds and initial distances.
- Head-on and overtaking encounters.
- Sudden start, stop, turn, and emergence from occlusion.
- Two crossing pedestrians, then four to eight pedestrians.
- Temporary sensor dropout and recovery.

Suggested initial engineering targets, to be validated against measured sensor
coverage and G1 response rather than reported as existing results:

- Sensor capture to planner-consumed track: p95 <= 150 ms.
- First usable tentative risk after visible evidence: <= 200 ms.
- Planner and supervisor p95 execution each below their 50 ms period, with
  overrun counts reported; measure the whole chain separately.
- Ordinary visible crossing response begins 2–3 s before projected conflict and
  always exceeds measured response/stop time plus data age and a margin.
- Zero collisions/falls in the declared suite, at least 95% goal completion,
  and no unsupported lateral commands.
- Comfortable yielding/passing normally resolves predictable crossings without
  hard supervisor overrides. Report hard-override rate separately from yields.
- Report body clearance, false-stop time, unnecessary stops, time-to-resume,
  progress after yielding, and command reversals alongside success rate.

## Recommended first implementation batch

Start with P0 instrumentation plus the three narrow P1 repairs: deadband/shared
command limits, tracker velocity initialization, and typed tentative risk. Follow
with early candidate search and measured robot rollouts. These address identifiable
causes of late behavior without waiting for a new learned prediction model.
# Archived development plan

> Historical planning notes. Implementation status and measured results are maintained in `docs/DEMO_RESULTS.md`.
