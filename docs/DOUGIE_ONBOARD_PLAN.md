# Onboard mission plan — 19 October 2026

[Project home](../README.md) · [Implemented methods](DOUGIE_ONBOARD_MISSION.md) ·
[Portable evidence](results/dougie_onboard_20261019.json) · [Current GIF provenance](DOUGIE_MEDIA.md#onboard-mission-refresh--19-october-2026)

## Current implementation

| Workstream | Implemented | Evidence / boundary |
|---|---|---|
| Onboard observation boundary | Body-frame noisy LiDAR; gyro/acceleration, joint encoders and foot-touch channels | No simulator base pose, frame velocity, object IDs or hidden field metadata passed to autonomy |
| Sensor fusion | IMU prediction, stance kinematics/anchors, bounded point-to-plane local registration | Physical walking and two short clean reactor missions; no loop closure or hardware calibration |
| Walking inputs | Measured proprioception and estimated gravity | Invariance test against changed hidden base pose/velocity |
| Geometry-aware radiation | PSD Matérn × observed-plane-feature kernel; at most 16 patches | PSD, wall-separation and replay/archive tests; no inferred material attenuation |
| Moving-window counts | One integrated Poisson likelihood, up to eight duration-weighted positions | Scalar-limit, midpoint-bias and correlated-expert tests; approximate joint Laplace inference |
| Mission planning | Observed support, upper-rate routing, return/deadline reserves, persistent regions | Existing heuristic admission remains; no proof of dose safety or optimal coverage |
| Reproducibility | Explicit ideal-pose/global/point comparators, source hashes, retained failures and portable GIF inputs | 212 tests at this checkpoint; historical full-height results remain a separate cohort |

Defaults: `--localization fused --gp-mode geometry --detector-likelihood window`.
The synthetic sensor front-end emulates onboard channels; it does not establish
real-robot readiness. Evaluator/renderer truth must stay outside mission decisions.

## Next stages and acceptance gates

### 1. Matched-work dose comparison — highest priority

- Compare geometry/window against global/point using identical initial observed
  goals, detector/sensor seeds, sensing cadence, compute schedule and dose budget.
- Freeze task completion criteria before running: required observed targets or
  reachable-region coverage plus physical home return. A stop or premature return
  does not count as dose reduction.
- Run a development matrix across at least three layouts and three seed pairs;
  retain every failure, dose, route length, time, contacts and localization error.
- Report paired dose differences only for comparable completed work, and report
  incomplete trials separately. Follow with an independently selected cohort.

**Gate:** lower actual dose at matched completed work without worsening return or
contact outcomes. This gate has not been met by the short survey GIF runs.

### 2. Localization robustness and calibrated uncertainty

- Exercise longer trajectories, feature-poor corridors, slopes, slip, IMU bias,
  dropouts, timestamp delay and rejected registration updates.
- Add loop closure/relocalization with consistent correction of support maps,
  detector trajectories and GP observations, rather than shifting only the robot.
- Measure absolute/relative pose error, yaw error, covariance consistency and
  actual home error. Distinguish arbitrary initial-frame offset from drift.
- Keep capture-time scan transforms immutable; late packets require pose-history
  interpolation rather than using the current attitude.

**Gate:** predeclared error/freshness limits, reproducible stopping on invalid
sensing, and physical return success across the matrix. Current covariance and
drift allowance are heuristics, not calibrated confidence guarantees.

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
