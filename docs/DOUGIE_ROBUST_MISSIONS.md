# Robust onboard mission stages

## Stage 1 — registration quality and consistent maps

`--navigation-profile robust` is the new development profile; `legacy` retains
the previous fusion/mission comparator. Both consume onboard observations only.

- Atomic point-to-plane registration rejects insufficient correspondences,
  inconsistent normals, excessive residuals and large pose innovations. A failed
  fit cannot partially move the estimate or contaminate the next registration map.
- Up to 64 fixed observed keyframes replace continuously overwritten registration
  voxels. Initial anchors are retained. Weak geometry is reported by observable
  rank rather than supplying fictitious motion constraints.
- Stance-velocity disagreement excludes individually slipping feet. Pose allowance
  now includes travelled distance and registration innovations, and local residuals
  add clearance margin. This is a more conservative heuristic, **not calibrated
  probabilistic uncertainty**.
- Revisits require two consistent, well-conditioned matches against an older
  observed keyframe. At most 16 loop corrections are accepted. Corrections are
  distributed in time from an unchanged anchor; this is approximate bounded
  loop correction, not a full pose-graph optimizer.
- A versioned observation journal reprojects terrain scans and detector motion
  windows and rebuilds the GP before executing routes in the new frame. Partial
  detector windows, display evidence and trails use the same correction. Counts,
  exposure durations and measured cumulative dose are never changed. Pending
  old-revision plans are withheld.
- History is bounded at 1,024 planning scans / 8,192 count windows. Capacity
  exhaustion triggers return; corrections cannot silently drop old observations.
- Severe pose-quality failures withhold spatial GP updates but retain measured
  dose accounting. Home approach uses a tighter .2 m estimated tolerance with a
  separately support-checked final segment inside the last graph cell.

Tests cover known-transform recovery, unobservable motion on a plane, rejected
scan isolation, confirmed loop correction, shared scan/count revisions, partial
window conservation, single-assimilation worker replay and the final home segment.
The full suite passed 222 checks before adding the home-segment regression; all
18 affected mapping/odometry/archive checks then passed.

Physical screening (90 s cap, 2 uSv, detector/sensor 3/0, fixed .5 s plan delay):
`robust_fusion_stage1_seed3_homefix` returned cleanly at **43.050 s**, **4.976 m**,
**.109965 uSv**, no non-foot contacts, **.1452 m** physical home error. Position
RMSE/p95: **.0510/.0924 m**. No real loop closure occurred in this short run;
loop/reprojection evidence at this stage is synthetic. Concurrent test workloads
make the recorded wall timings unsuitable as an isolated performance comparison.

The preceding `robust_fusion_stage1_seed3` is retained as a failed return: the
tighter home tolerance exposed a coarse-cell stopping deadlock, fixed by the
checked final approach. Longer missions and matched completed work remain the
acceptance gates for the following stages.

## Stage 2 — bounded, exposure-aware recovery

Recovery distinguishes missing local support, stale ranges/counts, unreliable
localization, disconnected return routes, motion stalls and route-tracking loss.
Each episode has a 3 s resensing phase, at most two checked short breadcrumb
retreat attempts, a 12 s no-route wait limit and a 30 s episode limit. A change in
failure reason does not restart the timer. Waiting exposure is reserved against
the remaining budget and known return reserve; insufficient margin requests an
immediate known-route return or an explicit unrecoverable stop. Recovery motion
requires fresh sensing, current support/clearance/connectivity checks and dose/rate
admission. No blind backtracking or turning on unknown support is authorized.

The main-loop watchdog also bounds stale plans/pose holds. Terminal stops disable
new mission planning, but **physics and dose integration continue to the original
deadline**. This avoids claiming a dose reduction by simply ending a stranded
mission early. Final reports separate post-stop exposure and remain failed returns.

A controlled fault option, `--lidar-dropout-after`, emits missing measurements
(zero/invalid ranges), not max-range clear-space observations. Ordinary random
dropout now uses the same distinction. A 60 s, 10 uSv reactor fault trial with
dropout after 8 s reported unrecoverable localization at **21 s**. It remained
upright, recorded **.150400 uSv** including **.098043 uSv after the stop**, and
correctly failed the clean-return gate. Artifact: `robust_recovery_lidar_fault`.

The normal seed-4/sensor-1, 90 s / 2 uSv screen returned cleanly at **45.050 s**,
**.127701 uSv**, **4.124 m**, no non-foot contacts, and **.1447 m** true home error.
Three route-tracking recovery episodes resolved; measured recovery-associated
exposure was .03664 uSv. This is successful recovery evidence, not matched-work
dose superiority. Artifact: `robust_recovery_stage2_seed4`.

Verification: 228 tests passed before adding the missing-range fault regression;
all 24 affected recovery/odometry/multi-floor tests subsequently passed. The
fault trial is a deliberately retained failure, not a mission-completion pass.

## Stage 3 — useful information per exposure and matched work

`--utility-mode information` implements the new experimental scorer. Candidate utility
combines fresh geometric views, a Poisson expected-count/Fisher information proxy,
known inspection work and connector novelty. Repeatedly measured cells receive a
residence-time discount. The cost includes outward **and return** exposure/time,
turning through the shared forecast, inspection dwell, planning waits and recovery
reserve. Normalization uses remaining time and dose resources. This is a heuristic
proxy objective, not exact mutual-information optimization or dose optimality.
`--utility-mode legacy` retains the earlier outbound-oriented score.

Pre-assimilation count diagnostics compare each localized window with the model's
upper latent-rate envelope plus a Poisson 99% count quantile. Counts withheld for
bad pose are excluded; map-history replay does not rescore training windows.
These diagnostics never reduce beta or the return reserve and are not a calibrated
joint Poisson-lognormal predictive distribution. Held-out field/layout calibration
is still required before relaxing conservative bounds.

`--mission-objective circuit --require-circuit` freezes four supported goals from
the initial observed graph, without a source/landmark catalogue. Both directions
of geometric reachability are required. Planner goal visits and independent
physical witnesses are separate: physical support position must be within .35 m
for .2 s at every goal, followed by a clean home return. The evaluator transforms
the initially observed goals into physics coordinates solely for scoring; that
transform never enters autonomy. Comparisons check that both initial and physical
goal sets match before reporting a paired dose difference.

Initial matched circuit matrix (240 s cap, 2 uSv, 1.5 m goal radius, robust fusion
and recovery for both scores):

| Detector/sensor | Legacy utility | Information utility | Matched task outcome |
|---|---|---|---|
| 3/0 | 105.155 s / .308865 uSv | 105.155 s / .308865 uSv | Both completed all four physical goals and home |
| 4/1 | 103.150 s / .294386 uSv | 103.150 s / .294386 uSv | Both completed all four physical goals and home |

The trajectories were identical within each pair: **no dose improvement is claimed
for this fixture**. All pre-assimilation count windows were below their upper
count bounds; this small, correlated cohort is not calibration qualification.
[Portable circuit evidence](results/dougie_robust_circuit_v1.json).

Verification: **235 tests passed**, including information cost/revisit behavior,
pre-assimilation diagnostics, immutable observed goals, disconnected/unknown goal
rejection, independent physical witnesses and non-duplicating map replay.

```bash
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_circuit.py \
  --duration 240 --dose-budget 2 --radius 1.5 --seed-pairs 3:0 4:1
./dougie radiation --headless --fast --duration 240 --dose-budget 2 \
  --mission-objective circuit --require-circuit --utility-mode information \
  --seed 3 --sensor-seed 0 --fixed-plan-delay .5
```

## Validation-driven corrections

The first 600 s robust matrix is retained in
[the v1 report](results/dougie_robust_budget_600_v1.json). Both 2 uSv runs returned,
but both 10 uSv runs reached an explicit localization-related terminal stop. No
loop correction was accepted. The expanded 2 m / 10 uSv observed-circuit matrix
also completed all four tasks under both utility modes with identical paired
trajectories ([expanded v1 evidence](results/dougie_robust_circuit_expanded_v1.json)).
These findings prompted corrections rather than a claim of general improvement:

1. **Observation availability:** detector integration completes at `t + dt`.
   Robust planning now waits until that timestamp before submitting a window.
   Previously a window could be labelled stale because it was 5 ms in the future,
   unnecessarily triggering recovery. Deferred counts remain queued and are
   assimilated once; the legacy profile preserves its original comparator timing.
2. **Distant-wall observability:** normal extraction now accounts for actual beam
   angular spacing and measured range, with one-sided-plane agreement rejecting
   discontinuities. A fixed .6 m neighbour cutoff had discarded distant wall
   structure. Registration columns are normalized before testing rank so a large
   floor's rotational leverage does not suppress smaller wall constraints.
3. **Localization return reserve:** the worker requests return at a .18 m
   heuristic position-standard-deviation allowance, before the .3 m motion-stop
   threshold. Full six-direction observability is required before accepting a
   loop correction that resets the scalar drift allowance.

Verification after these changes: **239 tests passed**, including future-window
deferral without loss, sparse-elevation wall normals, small wall constraints beside
a large floor, and proactive return before the localization stop threshold.
These are heuristic safeguards; they do not establish calibrated covariance or
independently qualified loop-closing SLAM. Fresh mission results follow separately.

### Conservative v2 screening and final refinements

The corrected v2 budget matrix returned cleanly in **all four trials**, including
the previously failed 10 uSv seed pair. The position p95 for seed 3 / 2 uSv fell
to .0189 m. However, the heuristic uncertainty reserve ended these missions early
(181–248 s). A well-conditioned revisit that already agreed with an old anchor
was being ignored because its pose correction was small. Confirmed near-zero
matches now revalidate the drift allowance without changing pose or map revision;
they still require two consistent full-rank registrations.

The expanded v2 circuit matrix completed every task, but the one-endpoint
information score used 5.9% more dose for seed 4. Both paired result sets are
retained ([budget v2](results/dougie_robust_budget_600_v2.json),
[circuit v2](results/dougie_robust_circuit_expanded_v2.json)). For mandatory circuit
work, information mode now evaluates at most **24 orders of the four remaining
observed goals**, including the home leg. Leg shortening is support/risk checked,
so grid turns do not hide the benefit of continuous diagonal routes. Only the
first leg is executed; all subsequent work is revalidated after new observations.
This bounded task-sequencing refinement is specific to the explicit circuit
objective; survey mode retains the information-per-resource candidate objective.

### Final utility selection and tracking clearance

The completed three-pair circuit comparison is retained in
[the final circuit report](results/dougie_robust_circuit_final.json). All six runs
completed identical paired physical goals and home return. Information/tour mode
used **3.3% more dose on seed 3**, **5.2% more on seed 4**, and **6.1% less on the
additional seed 5**. The evidence is mixed, so **legacy utility remains the default**;
information mode stays available explicitly. This choice affects candidate scoring,
not the new fused localization, map consistency or recovery defaults.

The information-mode long matrix is also retained
([report](results/dougie_robust_budget_600_final.json)). Three runs returned; the
seed-4 / 2 uSv trial hit `shield_west` with the base and correctly failed contact
qualification. The planned radius had not explicitly reserved the follower's
allowed .12 m execution error. Robust smooth navigation now adds that same
tracking-error limit to its base clearance, in addition to the existing local
localization margin. The contact thresholds were not relaxed. Revised default
and affected-case checks are reported separately below.

## Final clearance/default validation

Final source uses robust sensing, consistent maps, bounded recovery and the
explicit tracking-clearance reserve. Legacy utility is the default; the
information strategy remains an opt-in experiment.

| Trial | Utility / budget | Return (s) | Actual dose (uSv) | Travel (m) | Physical home error (m) | Pose p95 (m) |
|---|---|---:|---:|---:|---:|---:|
| Default, seed 4 / sensor 1 | legacy / 2 | 389.870 | 1.267477 | 40.568 | .1483 | .0171 |
| Default, seed 4 / sensor 1 | legacy / 10 | 427.965 | .775721 | 51.244 | .1358 | .1236 |
| Previously contacting case, rechecked | information / 2 | 371.825 | 1.097797 | 39.602 | .2634 | .0308 |

All three passed clean physical return with **zero non-foot contacts**. The two
default trials each visited 67 physical .5 m XY cells; the larger allowance used
38.8% less dose, but visited different locations, so this is not a matched-coverage
claim. Their raw position RMSEs were .0140/.0437 m; translation-aligned RMSEs were
.0076/.0421 m. All sampled component errors fell inside the reported three-sigma
heuristic envelopes in these two trials, which does not establish probabilistic
calibration on other layouts or failure modes.

The 10 uSv default run accepted one loop correction and consistently rebuilt its
map/GP history. The affected-case information recheck accepted two corrections;
its successful result does not erase the retained earlier wall-contact failure.
Confirmed near-zero anchor checks were also exercised (11/7 in the default pair).

The post-clearance seed-5/sensor-2 circuit recheck again completed all four physical
goals under both utilities: **.361857 uSv / 115.185 s** for legacy and
**.339942 uSv / 109.170 s** for information. That 6.1% saving on one pair does not
override the two adverse comparisons in the broader development matrix.

Evidence:
- [Final default 2/10 uSv checks](results/dougie_robust_default_checks.json)
- [Post-clearance matched circuit](results/dougie_robust_clearance_circuit.json)
- [Stage checks, including deliberate faults and retained failures](results/dougie_robust_stage_checks.json)

**243 regression tests passed.** These are development seeds on one reactor
layout, with fixed simulated planning delay. Wall timings include concurrent host
workloads and are not hard-real-time evidence. Full pose-graph optimization,
statistically calibrated uncertainty, unseen-layout dose superiority and renewed
sensor-only multi-floor/terrain qualification remain open.

### Run the revised defaults

```bash
./dougie radiation --headless --fast --duration 600 --dose-budget 10 \
  --seed 4 --sensor-seed 1 --fixed-plan-delay .5 --require-clean

# Compare utilities on the same initially observed task, with physical gates:
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_circuit.py \
  --duration 300 --dose-budget 10 --radius 2 --seed-pairs 5:2

# Controlled loss-of-LiDAR experiment; failed return is the expected outcome:
./dougie radiation --headless --fast --duration 60 --dose-budget 10 \
  --seed 3 --sensor-seed 0 --fixed-plan-delay .5 --lidar-dropout-after 8 --require-clean
```

`--navigation-profile legacy` selects the earlier navigation comparator.
`--utility-mode information` changes scoring while retaining robust localization
and recovery. Neither option selects simulator pose; that remains the separate,
explicit `--localization ground-truth` diagnostic mode.

## Archive frame contract

`measurements.jsonl` retains each sample's capture-time `map_revision` and
`position_valid` flag. `map_corrections.jsonl` contains the onboard-derived
corrections; consumers reconstruct the final frame by applying only revisions
newer than an observation's revision. Window positions use their duration-weighted
bin times. Counts and dwell durations are unchanged. The final support/GP archives
use the revision in `radiation_summary.json` → `map_consistency.map_revision`.
Do not combine uncorrected historical sample coordinates with a corrected final
map. Localization-error logs intentionally preserve the online estimate at each
time, rather than retrospectively improving the reported error.
