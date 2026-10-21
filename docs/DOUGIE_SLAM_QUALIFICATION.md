# Pose graph, uncertainty and sensor-only vertical qualification

## Pose-graph implementation

Robust fused missions now select `--slam-backend pose-graph` by default.
`--slam-backend distributed` preserves the previous time-distributed endpoint
correction comparator. The graph contains six-DoF poses, sequential observed
scan/leg-odometry factors and confirmed loop factors. The first pose fixes gauge.
Sparse robust least squares jointly optimizes the retained poses; residual,
innovation and displacement gates reject inconsistent results transactionally.

The graph is bounded at 512 poses. Cloud keyframes remain bounded separately;
evicting a cloud does not discard its graph constraints. Graph-capacity exhaustion
requests a return instead of silently dropping history. `pose_graph.json` records
poses, relative factors, assumed factor scales and solver diagnostics. Factor
scales are modelling assumptions, not calibrated marginal covariance.

Accepted graph updates emit **optimized correction knots** at every retained pose.
The existing observation journal applies the same interpolated SE(3) correction
to terrain captures, count-window bins, home/tasks, breadcrumbs and display
history. Original counts and durations are preserved. Old-revision routes remain
withheld during replay. This replaces the prior single-endpoint linear warp,
although dense trajectory poses between graph knots are still interpolated.

Tests cover 3-D loop closure, root gauge, changes to intermediate poses, outlier
rollback, capacity, correction serialization, count conservation and integration
with confirmed onboard registration. The full regression suite passed 248 tests
before the additional graph/keyframe integration check.

## Baseline vertical evidence

The retained pre-graph sensor-only mezzanine baseline (`slam_mezzanine_baseline`)
returned on the lower floor at 133.230 s / .040503 uSv and failed the strict
vertical round-trip gate. This is not a successful multi-floor mission. The
following stages investigate calibration and observed ramp admission while
retaining the original physical contact, budget and witness gates.

## Episode-level uncertainty calibration

The new calibration protocol uses one **maximum normalized pose error per
independent episode**, evaluated at every active control tick. Thousands of
correlated telemetry frames are not treated as thousands of calibration examples.
Only initial translation gauge is removed; no trajectory is fitted to truth.

The finite-sample rank is `ceil((n + 1) * (1 - alpha))`. A finite 90% marginal
episode envelope needs at least nine calibration episodes; 95% needs at least 19.
The multiplier is never below one. A preregistered harness assigns disjoint sensor
seeds to calibration and validation before any trial runs and retains failures.
Invalid/incomplete calibration evidence prevents artifact generation.

`--uncertainty-profile` accepts only aggregate scales, episode scores/identities,
and pinned source/domain metadata. Runtime cannot read evaluator poses from the
artifact. Source, software, policy, scenario, time/budget, sensing and execution
settings must match. Loaded scales only inflate the existing pose allowance.
Conditional graph marginals are also recorded, but are not themselves calibrated
probabilistic covariance.

The coverage statement is **conditional on exchangeable reference-policy
episodes** and applies to the sampled control ticks. It is not an unconditional
guarantee for unseen layouts, new sensor models, continuous time or real hardware.
If an inflation factor changes closed-loop decisions, the deployment distribution
can change; fresh runtime validation is required. Pose coverage and successful
physical mission completion are reported separately.

## Sensor-only vertical development

Two mapping/planning defects were corrected without changing the physical gates:

- Footprint support now checks piecewise-continuous neighbouring planes instead
  of extrapolating one centre plane across an entire floor/ramp junction. Abrupt
  ledges still fail local continuity and edge-fit checks. Body clearance is checked
  once at the centre; footprint neighbours need support, not clearance for another
  complete robot. Unknown footprint cells remain inadmissible.
- Reprojection restores visited body neighbours, height bands and region history.
  A bounded shortlist of observed novel connector goals—and the current committed
  goal—can receive checked smooth-route forecasts before a grid-turn deadline
  rejection. Rate, dose, support and return checks still apply.

The original **10°** fixture remains the default and is still unqualified for
sensor-only vertical completion: the retained 300 s and 600 s probes returned
without an upstairs witness. An explicit `--mezzanine-angle 8` variant provides
measurement headroom below the unchanged 10° terrain-admission limit. It keeps
the **same .705 m gallery height**, lengthens the ramps/room, and preserves the
upper source's position relative to the gallery. Its results do not qualify the
original 10° fixture or the 2.4 m facility.

The first 8° trial reached upstairs but stopped during descent when large global
graph uncertainty was treated as failed local tracking. Global uncertainty now
forces **return-only** behavior; execution of that return still requires fresh,
full-rank local registration, low local residual/conditional uncertainty, observed
support and a validated route. Bad/stale local sensing still withholds motion.
Spatial GP updates remain withheld when global pose uncertainty exceeds their
limit. This does not assert that the global home pose is accurate: physical home
distance is independently checked.

The revised seed-3/sensor-0 8° probe passed upstairs/downstairs/home at **237.490 s**,
using **.164563 uSv** with no non-foot contacts. Upper witness: **85.170 s**;
lower witness: **192.825 s**; position-error p95: **.0131 m**. This is a development
pass pending the predeclared calibration/validation cohort, not a reliability claim.

The reactor graph check also returned cleanly at **173.330 s / .660732 uSv**,
with three accepted loop optimizations. Its changed trajectory is not a matched-work
dose comparison. All unsuccessful vertical probes are retained.

Verification at this stage: **259 tests passed**, including piecewise support,
single application of body clearance, steep-step rejection, committed-route
refinement, replayed height/visit history, fixture geometry/source consistency,
and high-global-uncertainty return restricted to healthy local observations.
