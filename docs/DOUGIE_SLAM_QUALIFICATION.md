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
pass; the subsequent predeclared calibration/validation cohort is reported below.

The reactor graph check also returned cleanly at **173.330 s / .660732 uSv**,
with three accepted loop optimizations. Its changed trajectory is not a matched-work
dose comparison. All unsuccessful vertical probes are retained.

Verification at this stage: **259 tests passed**, including piecewise support,
single application of body clearance, steep-step rejection, committed-route
refinement, replayed height/visit history, fixture geometry/source consistency,
and high-global-uncertainty return restricted to healthy local observations.

The [portable development report](results/dougie_pose_graph_development_20261021.json)
retains eight runs, including failed vertical attempts, summary/config hashes,
source identities and localization diagnostics. The successful 8° artifact also
passes `scripts/check_dougie_radiation.py --require-round-trip`.

## Completed frozen-source cohort

Started on 2026-10-21 against runtime commit `7d035f8`:

- **8° mezzanine:** 19 calibration episodes, five held-out validation episodes,
  and one independent application episode, with `alpha=.05`, a 600 s limit,
  10 uSv budget, robust pose-graph navigation, legacy utility and a fixed .5 s
  planning delay. The protocol seed is `20261021`; sensor and detector seeds and
  split membership were declared before execution. All episodes require the
  strict vertical round trip. Three episodes run concurrently.
- **Full-height facility screening:** detector seed 4, sensor seed 1, 2400 s
  limit, 10 uSv budget, pose-graph backend and .5 s fixed planning delay,
  also requiring the strict vertical round trip.

The cohort protocol and per-run logs are under
`results/dougie/pose_graph_calibration_mezz8/`; the completed
[portable report](results/dougie_pose_uncertainty_mezz8.json) retains all trials.
The fitted profile is [config/dougie_pose_uncertainty_mezz8.json](../config/dougie_pose_uncertainty_mezz8.json).
Facility artifacts are under `results/dougie/pose_graph_facility_screen/`.
Containers are named `dougie-uncertainty-cohort` and `dougie-pose-graph-facility`.

The harness exited successfully. **All 25 episodes passed** the strict vertical
round-trip artifact check: 19 calibration, five held-out, and one independent
application. No failed episodes were removed. The fitted rank is 19 of 19 and
the inflation multiplier is **1.0** (largest calibration score: .224577 relative
to the existing three-sigma allowance). The method refuses to shrink that allowance.

Held-out episode coverage was **5/5**; the independent application was also
covered (maximum normalized error .216498). The application returned at
**269.570 s / .226732 uSv**, with position-error p95 **.021307 m**, upper witness
at **80.285 s** and lower-return witness at **222.485 s**. Runtime recorded the
loaded profile hash `fb0f9ac3d7255bafe063a3b06b1ba9a0f8521d21eec2d0ae53cab386c426c048`.

This is a successful scenario-specific cohort, not a measured 95% mission
reliability guarantee. The 95% conformal statement is marginal over exchangeable
reference-policy episodes; five held-out examples do not establish the true
coverage rate. The multiplier of one leaves control thresholds unchanged. The
artifact requires the exact pinned software, runtime sources and operating domain;
future estimator/planner changes require a new cohort. The original 10° fixture
and full-height facility remain unqualified.

**Version note:** the subsequent capture-time diagnostic additions change the
runtime source fingerprint. This profile is retained for runtime `7d035f8` and
is deliberately rejected by the newer runtime. It is not a current-runtime
calibration profile; do not rewrite its hashes to make it load.

### Full-height screening failure

The facility screen finished with exit 1. Recovery began at **106.660 s** with
`return_route_disconnected`; the first trace snapshot showing it is at 107.165 s.
Supported graph nodes dropped from 1836 to 1714 between the preceding snapshot
and that snapshot, and the recorded return route went from 60 points to zero.
Map revision was still zero, so this onset was **not triggered by loop replay**.
Global position standard deviation was about .136 m, below the .18 m forced-return
threshold; the scan residual was about .0061 m. The local clearance margin later
fell from .04 m to .02 m without restoring a route.

Two checked breadcrumb-retreat attempts produced no admitted motion. The robot
entered terminal stop at **119.195 s**, then continued to move downhill under
zero velocity commands. At **449.335 s**, independent contact qualification aborted
the run after contact with `service_ramp_lower_post_1_0` (two contacts, .1721 Ns
total non-foot impulse, 19.62 N peak). True dose was **.093286 uSv**, including
**.067475 uSv after the terminal stop**. Neither upper nor lower-return witness
was obtained. This is a failed mission, including a failed physical stop outcome.

The portable report now preserves navigation/recovery transitions, map revisions,
route sizes, estimated positions and contact magnitudes. These distinguish the
initial graph failure from subsequent sliding and later graph corrections. The
exact support/edge rejection causing disconnection is not established by the
archived planning snapshots; it needs capture-time graph diagnostics before a
terrain-admission change can be justified. Stationary-command stability on slopes
also remains an open controller qualification issue.

An [offline support reconstruction](results/dougie_facility_support_diagnosis_20261021.json)
reproduces all **1876 archived nodes**, split into **1724 floor nodes** and **152
ramp nodes**, with no connecting edge. The ramp component starts at about .243 m
height; the floor component remains below .018 m. Probed entrance columns at
`y=6.3`, `x=.7, 1.1, 1.5, 1.9, 2.3, 2.7` each contain observed support but no
accepted local-plane candidate. Thus, at those columns the failure precedes body
clearance and footprint admission. This narrows the investigation to local support
fitting/continuity; it does not justify relaxing slope or step limits.

Reproduce with `python3 -m scripts.diagnose_dougie_support RUN --output REPORT`
inside the simulator environment; repeat `--column X Y` for specific observed-map
columns. The report pins its input files, reconstruction source and script hashes.
It uses archived sensor geometry only and explicitly labels the final-map scope.

The reporting/diagnostic additions passed the full **262-test** suite. Runtime
sources were unchanged throughout cohort collection and application.

## Capture-time diagnostics and stop-controller screening

The next runtime revision records bounded nearby-column diagnostics when support
or a return route becomes unavailable. It distinguishes missing local returns,
discontinuities, plane rank/slope, residual rejection, body clearance and footprint
rejection. Discontinuity diagnostics include the observed point pair and excess
height difference. Diagnostics are retained across cached graph builds and do not
alter admission thresholds.

The [instrumented facility replay](results/dougie_facility_diagnostics_20261021.json)
reproduced the previous failure, including stop time and contact outcome. At onset,
the worker explicitly reported `return_graph_disconnected`; nearby ramp-centre
candidates existed but failed footprint admission, while adjacent candidates were
still admitted. The [final-map rejection probes](results/dougie_facility_support_rejections_20261021.json)
identify local discontinuity rejection near the entrance: observed-pair excesses
of approximately **9.4–10.0 mm** exceeded the **8 mm** continuity allowance.
That identifies the rejecting mechanism, but does not by itself justify treating
those discrepancies as harmless noise or weakening step rejection.

### Controlled stationary dwell experiments

`python3 -m scripts.qualify_dougie_slope_stop --output NEW_REPORT` runs a
predeclared flat/8.53°/10° screen with sensor seeds 0 and 1, five seconds of
settling and **120 seconds of zero-command dwell**, at fixture friction .4.
Initialization and scoring use fixture truth; the policy uses noisy encoder/IMU
observations. This isolates static dwell and does not test a moving-to-stopped
transition or the full mission recovery sequence.

| Controller during dwell | Flat maximum XY drift | 8.53° maximum XY drift | 10° maximum XY drift | Contact outcome |
|---|---|---|---|---|
| Baseline policy, zero command | .067–.108 m | 1.467–1.495 m | 1.647–1.710 m | No non-foot contacts in these open fixtures |
| Targets reset to measured encoders | .155–.157 m | .242–.243 m | .251–.256 m | Sustained non-foot contact; rejected |
| Retain last commanded targets | .067–.081 m | .017–.026 m | .034–.054 m | No non-foot contacts in six screening cases |

[Baseline and encoder-hold evidence](results/dougie_slope_stop_screen_20261021.json)
and [commanded-target-hold evidence](results/dougie_target_hold_screen_20261021.json)
retain all trials and evaluator traces. Both hold comparators switch only after
all four measured foot forces exceed 5 N. Holding commanded targets preserves
the existing PD load offset; resetting them to measured angles removes it.
The encoder-hold comparator accumulated thousands of Ns of non-foot impulse and
must not be judged successful merely because its XY displacement was smaller.

Commanded-target hold is a promising **experimental comparator**, not an enabled
mission-controller feature. Next acceptance gates are moving-to-stop transitions,
longer dwell, contact/torque limits, support disturbances and end-to-end terminal
recovery. Original terrain/contact gates remain unchanged, and the full-height
mission is still a failure.

## Refreshed recording and verification

The diagnostic runtime's fresh `pose_graph_mezz8_recorded` run passed the strict
8° upstairs/downstairs/home gate at **237.490 s / .164563 uSv**, with zero non-foot
contacts and .0958 m true home distance. Its measured outcome reproduced the
earlier development probe. It did not load the historical calibration profile.
[Portable recording results](results/dougie_pose_graph_recorded_20261021.json) and
[GIF provenance](DOUGIE_MEDIA.md#pose-graph-mission-refresh--21-october-2026) retain
the source, input and capture identities.

The native GIF has 73 decoded frames; the telemetry GIF has 100. Both checksums,
every frame, detector-window consistency and ordered vertical witnesses were
verified. The full current suite passed **263 tests**. The tests include cached
support-rejection diagnostics; the instrumented full-height failure remains in
the evidence rather than being presented as a successful mission.
