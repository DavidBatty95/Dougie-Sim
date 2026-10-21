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
