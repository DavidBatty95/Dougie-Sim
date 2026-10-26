# Onboard mission plan — 26 October 2026

[Project home](../README.md) · [Implemented methods](DOUGIE_ONBOARD_MISSION.md) ·
[Pose-graph evidence](DOUGIE_SLAM_QUALIFICATION.md) · [Current GIF provenance](DOUGIE_MEDIA.md#navigation-refresh--26-october-2026)

Latest [navigation recovery development](DOUGIE_NAVIGATION_RECOVERY.md): signed
surface matching, bounded home checkpoints, coverage-based cloud retention and
opt-in normal-balanced sampling are implemented; **338 tests pass**. The 8°
regression passes, while original 10° and full-height facility qualification
remain open. The normal-balanced closed-loop facility and original-mezzanine
trials both returned cleanly, but neither reached the upper landing. The subsequent
vertical-goal commitment fix passes its admission and anti-chatter tests, but
closed-loop vertical completion remains unqualified. Next isolate ramp-junction
support/footprint rejection and continuous approach connectivity. Nearest-node
changes alone must not count as repaired connectors. Retain strict vertical
witnesses as the acceptance gate.

Footprint diagnostics now isolate slope-limit rejection at the 10° approach and
missing compatible footprint support at the facility frontier. The
[24-case production stance screen](DOUGIE_TRACTION_SCREEN.md) establishes a
separate low-traction downhill stopping failure: stance entry is refused and
drift continues. Prioritize bounded braking/settling and supported sensing
approaches, then repeat end-to-end qualification and frozen-runtime calibration.

The [planned-stop taper experiments](DOUGIE_PLANNED_STOP.md) now retain 96 new
trials. The two-second taper improved development passes to 22/24, but fresh
seeds reached only 19/24 versus 17/24 for step stopping, including a regression.
It remains an anticipatory harness experiment; reactive low-traction stabilization
is still an open controller task.

Measured-yaw feedback with an estimated-downhill posture gate has now been
screened on development, retained-failure and fresh seeds. Fresh low-friction
worst drift dropped to .069365 m, but push-gate failures and development
regressions prevent mission integration. See the
[full feedback comparison](DOUGIE_PLANNED_STOP.md#measured-yaw-feedback-follow-up).

The existing third home-checkpoint allowance now has a fresh facility result:
sampled intermediate-landing witness at 583.700 s, clean return at 794.880 s,
.248470 µSv and no non-foot contacts. Its first 8,398 physical samples exactly
match the shorter two-checkpoint run. Full-height completion still fails.
Next investigate localization/observability reserve on the upper ramp using the
retained [checkpoint evidence](DOUGIE_NAVIGATION_RECOVERY.md#third-home-checkpoint--26-october),
while retaining the unchanged uncertainty and strict-witness gates.

The current 3 m anchor-search comparator exhausted 16 correction slots and became
stranded. The opt-in `--loop-correction-capacity 32` trial used 21 corrections and
returned cleanly at 887.110 s, but explored less and missed the intermediate
landing. The [matched capacity audit](DOUGIE_NAVIGATION_RECOVERY.md#upper-ramp-reserve-and-bounded-correction-history)
preserves the identical physical prefix and this trade-off. Next prioritize
root-connected observability and calibrated factor uncertainty, rather than
assuming more loop events or a wider search guarantees upper-floor progress.

## Current implementation

| Workstream | Implemented | Evidence / boundary |
|---|---|---|
| Onboard observation boundary | Body-frame noisy LiDAR; gyro/acceleration, joint encoders and foot-touch channels | No simulator base pose, frame velocity, object IDs or hidden field metadata passed to autonomy |
| Sensor fusion | Quality-gated keyframes, stance-slip checks, confirmed loops and root-fixed six-DoF pose graph | Bounded 512-node optimization with coherent correction knots; hardware calibration remains open |
| Map consistency | Versioned scan/count journal and coherent support/GP replay | Old-frame routes withheld; counts and durations conserved; archive corrections retained |
| Walking inputs | Measured proprioception and estimated gravity | Invariance test against changed hidden base pose/velocity |
| Geometry-aware radiation | PSD Matérn × observed-plane-feature kernel; at most 16 patches | PSD, wall-separation and replay/archive tests; no inferred material attenuation |
| Moving-window counts | One integrated Poisson likelihood, up to eight duration-weighted positions | Scalar-limit, midpoint-bias and correlated-expert tests; approximate joint Laplace inference |
| Mission planning | Full-cost information scorer and bounded observed-task sequencing, alongside prior utility | Mixed measured dose results; legacy utility remains default, with robust sensing/recovery |
| Recovery | Bounded resensing, checked retreats and terminal-stop accounting | Missing sensors do not authorize blind motion; stranded exposure is still scored |
| Clearance | Localization residual and follower tracking-error reserves | Wall-contact regression retained; thresholds not weakened |
| Reproducibility | Source hashes, retained failures, matched physical goals and independent dose/pose scoring | 279 tests; [pose-graph cohort](DOUGIE_SLAM_QUALIFICATION.md) and [stance recovery](DOUGIE_STANCE_RECOVERY.md); historical ideal-pose full-height runs remain separate |

Defaults: `--localization fused --gp-mode geometry --detector-likelihood window`.
Navigation uses `--navigation-profile robust --utility-mode legacy`. Selecting
legacy **utility** does not select simulator pose or disable recovery.
The synthetic sensor front-end emulates onboard channels; it does not establish
real-robot readiness. Evaluator/renderer truth must stay outside mission decisions.

## Next stages and acceptance gates

### Longer-mission evidence update

The earlier [paired 2/10 µSv experiment](DOUGIE_LONG_MISSIONS.md) covers four onboard
reactor missions with a 600 s cap. Both 2 µSv runs returned; one of two 10 µSv runs
returned. The successful high-budget pair improved physical visitation per dose
by 24.6%, but the other stalled in a support/sensing wait state and did not return.
Position p95 errors reached .644 m. This advances long-duration screening, not
the matched-work dose-reduction or localization-robustness gates below. Prioritize
support recovery, covariance consistency and exposure while stalled. Those
implementation stages are now present; their subsequent screening, retained
regressions and clearance correction are in [the robust mission report](DOUGIE_ROBUST_MISSIONS.md).

### 1. Broaden matched-work dose comparisons

The observed-circuit harness now freezes goals from onboard support and requires
independent physical visits plus return. Three seed pairs completed under both
utilities, but information mode's dose changes were mixed (+3.3%, +5.2%, −6.1%).
It remains experimental rather than replacing the default utility.

- Compare geometry/window against global/point using identical initial observed
  goals, detector/sensor seeds, sensing cadence, compute schedule and dose budget.
- Freeze task completion criteria before running: required observed targets or
  reachable-region coverage plus physical home return. A stop or premature return
  does not count as dose reduction.
- Run a development matrix across at least three layouts and three seed pairs;
  retain every failure, dose, route length, time, contacts and localization error.
- Report paired dose differences only for comparable completed work, and report
  incomplete trials separately. Follow with an independently selected cohort.

**Gate:** repeatable lower actual dose at matched completed work without worsening
return or contact outcomes. One successful noise seed is insufficient; the gate
has not been met across held-out layouts and fields.

### 2. Localization robustness and calibrated uncertainty

- Exercise longer trajectories, feature-poor corridors, slopes, slip, IMU bias,
  dropouts, timestamp delay and rejected registration updates.
- Qualify the implemented loop/anchor checks and coherent map replay against false
  matches, long map histories and different layouts. The pose-graph optimizer is
  implemented; the time-distributed backend remains a comparator.
- Measure absolute/relative pose error, yaw error, covariance consistency and
  actual home error. Distinguish arbitrary initial-frame offset from drift.
- Keep capture-time scan transforms immutable; late packets require pose-history
  interpolation rather than using the current attitude.

**Gate:** predeclared error/freshness limits, reproducible stopping on invalid
sensing, and physical return success across the matrix. Current covariance and
drift allowance do not provide unconditional confidence guarantees. The pinned
`7d035f8` 8° cohort completed 19 calibration, five validation and one application
episode, with 5/5 held-out envelope coverage and multiplier 1.0. New diagnostic
runtime hashes deliberately invalidate that historical profile; recalibration and
broader domains remain open.

### 3. Radiation accuracy and bounded compute

- Validate sharp hotspots, cross-wall/floor predictions and upper-rate shortfall
  on held-out source/shield layouts, with truth confined to offline scoring.
- Compare kernel strength/patch budgets and one/two/eight-bin quadrature against
  finer integration. Assess the non-concave window approximation explicitly.
- Profile geometry-triggered history replay and map growth. Geometry capacity
  remains bounded; count-history exhaustion must retain its return trigger.
- Propagate pose uncertainty into inference and route clearance; current drift
  allowance alone does not account for correlated map errors.

**Gate:** calibrated risk estimates and bounded p95/p99 latency under declared
load, without assimilating held-out counts or later geometry into earlier tests.

### 4. Repeat sensor-only vertical qualification

The 8° variant passed all 25 cohort episodes and a fresh recorded round trip with
the diagnostic runtime. The original 10° variant and full-height facility remain
unqualified. Capture-time diagnostics now identify support-fit/footprint rejection
at facility disconnection. Static commanded-target hold reduced ramp drift in six
screening cases, but the subsequent 20-trial walking-to-stop screen failed all
four downhill hold contact gates despite quiet four-foot entry. It is not enabled
in missions. Active stance/load stabilization, disturbances and end-to-end recovery
remain open. Encoder-angle hold also caused sustained body contact and was rejected.
See the retained evidence in the qualification report.

The subsequent 20-case active-position-hold screen kept the policy running and
avoided non-foot contact in all ten feedback trials. However, proprioceptive-only
pose feedback underestimated movement: physical ramp drift remained .37–.60 m.
That motivated the range-observable feedback, correction-aware anchor and
degraded-sensing screen below. Both versions remain harness comparators.

Range observations and revision-aware anchors have now been screened: the new
20-case healthy-range cohort reduced ramp drift to .088–.164 m, with zero non-foot
contacts. All eight separate dropout cases correctly withheld corrections but
drifted 1.315–1.522 m. Physical stabilization under loss of observations, local
clearance/support checks and end-to-end recovery remain open; command refusal
alone is not a stationary stop. No mission-controller default has changed.

The gravity-support model now mitigates that physical drift in tested fixtures:
all ten stance cases stayed below 2 cm, and all eight range-loss handovers retained
support. Opt-in `--terminal-hold gravity` was tested in the facility: 4.1 cm sampled
post-stop drift and no non-foot contact through 600 s, compared with 3.93 m and a
contact abort for the matched zero-policy baseline. Both vertical missions failed
because the return graph remained disconnected. Priority now shifts to robust
support fusion/return connectivity and broader stance disturbance/traction tests.
See [versioned evidence and acceptance boundaries](DOUGIE_STANCE_RECOVERY.md).

- Start with the mezzanine, then full-height ramps, using fused pose throughout.
- Require ordered upper/lower/home witnesses, clean contacts, budget compliance,
  actual home distance and measured per-floor completion.
- Repeat paced asynchronous trials as well as fixed-delay comparisons.

**Gate:** fresh sensor-only evidence; earlier ideal-pose full-height passes do not
transfer. Ordinary industrial stairs still require separate gait qualification.

### 5. Perception and hardware fidelity

- Replace ideal semantic IDs/positions with onboard image/depth detections before
  enabling landmark inspection in fused missions.
- Calibrate sensor extrinsics, IMU/encoder noise, touch-force response, range
  dropout and detector response against actual hardware or independent datasets.
- Add dynamic-obstacle handling and map clearing before occupied changing scenes.

## Reproduce the current checkpoint

```bash
./dougie radiation --headless --fast --duration 90 --dose-budget 2 \
  --seed 3 --sensor-seed 0 --fixed-plan-delay .5 --require-clean
./dougie radiation --headless --fast --duration 90 --dose-budget 2 \
  --seed 4 --sensor-seed 1 --fixed-plan-delay .5 --require-clean
./dougie test
python3 scripts/check_dougie_docs.py
```

The original [multi-floor specification](DOUGIE_MULTIFLOOR_PLAN.txt) and
[stair-training roadmap](DOUGIE_MULTILEVEL_ROADMAP.md) remain dated design records.
This page is the active onboard implementation/qualification plan.
