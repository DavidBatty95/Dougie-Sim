# Navigation recovery development — 24–25 October 2026

[Project home](../README.md) · [Active plan](DOUGIE_ONBOARD_PLAN.md) · [Media](DOUGIE_MEDIA.md)

The explicit 8° mezzanine retains a strict upstairs/downstairs/home pass after
correcting LiDAR surface-side matching. The original 10° mezzanine and full-height
2.4 m facility remain unqualified. A clean downstairs return or traversal to the
1.2 m intermediate landing does not satisfy the ordered upper/lower/home gate.

## Implementation and retained evidence

The [portable development report](results/dougie_support_alignment_20261024.json)
retains configurations, source hashes, physical witnesses, failures, recovery
transitions, exposure and post-stop movement across successive runtime versions.
These are development comparisons, not a matched reliability cohort.

| Change | Evidence and limits |
|---|---|
| Sensor-facing normals and signed normal correspondence | Prevents opposite sides from contributing false matches or an artificial sixth observable direction. The 8° regression completed at 275.585 s, .274697 µSv, p95 position error .014100 m. |
| `--support-fusion bounded-mean` | Averages only narrow retained height bands; wider bands permanently retain their observed maximum. Identical-observation replay leaves both modes disconnected. No demonstrated connectivity repair. |
| `--loop-search-radius 3` | Wider candidate search retained all registration gates but the trial failed localization. Default remains 1.25 m. |
| `--localization-checkpoints 2` | Bounded home confirmation and resumption within the original time/dose budget; visits and observations persist. Early trials still lost full-rank return registration. |
| `--keyframe-retention coverage` | Spatially diverse retention within the same 64-cloud capacity. With checkpoints, clean home return at 861.045 s and .228957 µSv, after visiting the intermediate landing; no upper witness. |
| `--mission-objective vertical` | Prioritizes newly observed height bands among already admitted goals. The spatial-sampling trial still failed return localization. |
| `--registration-sampling normal-balanced` | Retains measured points with rare surface-normal directions within the existing 1500-point cap. Opt-in; historical spatial sampling remains the default. |

The gravity-supported terminal stance remains opt-in. Failed navigation retains
its failed scoring and all subsequent dose. Slope, step, contact, dose, full-rank
return registration and strict round-trip gates are unchanged.

## Registration sampling diagnosis

The evaluator-only static probe uses an archived scene and physical pose with
ideal initialization. At 670 s in `facility_checkpoints`, historical spatial
sampling gives rank five across four noise seeds; normal-balanced sampling
restores rank six across those four probes. This isolates lost measurement
geometry, not closed-loop localization accuracy or mission qualification.

- [Earlier pose probe](results/dougie_registration_probe_642_20261025.json)
- [Failing-pose spatial probe](results/dougie_registration_probe_670_20261025.json)
- [Normal-balanced probe](results/dougie_registration_balanced_probe_670_20261025.json)

The sampling implementation is frozen in `e588712`; its full suite passed
**307 tests**. Historical uncertainty calibration is source-pinned and does not
apply to these new runtime versions.

## Closed-loop sampling results

The [completed trials and spatial-sampling comparator](results/dougie_normal_balanced_20261025.json)
retain source identities and all failed mission gates. Both new runs use detector
seed 4 / sensor seed 1, coverage retention, two bounded checkpoints, the vertical
objective, maximum support fusion and .5 s fixed planning delay.

| Normal-balanced run | Clean return | Actual dose | Position p95 | Strict vertical mission |
|---|---:|---:|---:|---|
| Facility, 2400 s cap | 419.945 s | .139867 µSv | .018638 m | Failed: no ordered vertical witnesses |
| Original 10° mezzanine, 600 s cap | 355.785 s | .247587 µSv | .014299 m | Failed: no ordered vertical witnesses |

Both had zero non-foot contacts and no terminal stop. The facility used both
checkpoint resumptions. Its true home error was .095473 m; the mezzanine's was
.092285 m. Neither reached the upper landing. Better local geometry retention and
clean returns do not demonstrate a repaired upstairs route: trajectories differ,
and these two development runs are insufficient for reliability qualification.

Reproduce the facility experiment in a fresh output directory:

```bash
docker compose run --rm -T -e MUJOCO_GL=egl dougie-sim \
  python3 -m dougie_sim.radiation_run \
  --world facility --duration 2400 --dose-budget 10 --seed 4 --sensor-seed 1 \
  --localization fused --navigation-profile robust --slam-backend pose-graph \
  --terrain-profile layered --mission-objective vertical \
  --localization-checkpoints 2 --keyframe-retention coverage \
  --registration-sampling normal-balanced --terminal-hold gravity \
  --record-support --fixed-plan-delay 0.5 --require-clean --require-round-trip \
  --record-demo --record-view routes --headless --fast \
  --output results/dougie/facility_normal_balanced_repeat
```

For the original mezzanine, change world to `mezzanine`, duration to `600`, and
use a separate output directory. No angle override is used. A clean return
without vertical witnesses intentionally exits nonzero under `--require-round-trip`.

## Vertical goal commitment follow-up — 25 October

`8b7401c` fixes a reproduced priority inversion in the opt-in vertical objective:
a committed flat-floor goal previously delayed consideration of newly reachable
height bands for up to 45 s. Admitted new-height candidates now compete immediately.
If all fail the existing return/rate/dose/deadline checks, the old admitted goal
and its original commitment timer remain. An unreached new-height goal retains
commitment to avoid repeated switching as more heights become visible.

The full suite passes **313 tests**, including rejected-preemption cases and
retention of an unreached height goal. Per-plan `vertical_frontier` telemetry
records supported graph heights, reachable new-height nodes, admitted candidates
and commitment preemption. These heights describe fitted, footprint-admitted
support nodes, not all raw range returns or hidden floor geometry.

The [matched development outcomes](results/dougie_vertical_preemption_20261025.json)
and [decision diagnostics](results/dougie_vertical_preemption_decisions_20261025.json)
retain both improvements and failures:

| Run | Return time | Actual dose | Result |
|---|---:|---:|---|
| 8° survey regression, seed 3 / sensor 0 | 275.585 s | .274697 µSv | Strict upper/lower/home pass; reproduces earlier metrics and native GIF bytes exactly |
| Facility vertical, seed 4 / sensor 1 | 419.945 s | .139867 µSv | Clean return, no upper witness; no improvement in completion |
| Original 10° vertical, seed 4 / sensor 1 | 87.115 s | .041415 µSv | Earlier ramp selection, support disconnection, clean return; no upper witness |

All three had zero non-foot contacts. The original 10° result is an earlier
aborted exploration, **not a mission-efficiency gain**. The facility's maximum
reachable supported height remained .298322 m; the original mezzanine's reached
.488142 m in its decision history, but its return graph subsequently disconnected.

The [21-scan failure replay](results/dougie_preemption_failure_replay_20261025.json)
exposes an important distinction: bounded-mean fusion changes the nearest node
from the disconnected ramp (.485 m away) to the connected ground floor (.543 m
away). Its boolean home-connectivity result improves, but its home-connected
component only reaches .018 m. It has **not repaired ramp connectivity**. Replay
reports now include the estimated current position, selected node, distance and
maximum home-connected height so this cannot be mistaken for an upstairs route.

Remaining priorities, in order:

1. Isolate ramp-junction fit and footprint rejection on frozen observations;
   compare the same physical approach and component, not only nearest-node flags.
2. Validate any repaired connector in closed-loop 10° and full-height trials,
   including paced asynchronous execution and held-out noise/layout seeds.
3. Recalibrate uncertainty on the frozen successful runtime; historical envelopes
   remain inapplicable. Broader stance disturbance/traction screens and radiation
   calibration/latency qualification remain separate open workstreams.

## Third home checkpoint — 26 October

The existing `--localization-checkpoints 3` option allows one additional bounded
home revalidation within the original 2400 s / 10 µSv caps. With detector seed 4,
sensor seed 1, normal-balanced sampling, coverage retention and the vertical
objective, `facility_third_checkpoint` reached the intermediate landing and
returned cleanly at **794.880 s**, **.248470 µSv**, with **.020579 m** position p95,
**.132937 m** true home error and no non-foot contacts. There was no terminal stop.

The [independent sampled witness](results/dougie_third_checkpoint_witness_20261026.json)
replays recorded physical joint/base poses in the archived scene. All four feet
satisfy the intermediate slab's positional bounds for at least .2 s at
**583.700 s**; the longest sampled dwell is **34.5 s**. This is 20 Hz foot-pose
evidence, not a contact-force or continuous-time guarantee. The earlier two-
checkpoint run has no intermediate witness. All **8,398** physical samples of
that shorter run match the new run's prefix exactly, including commands.

The [navigation report](results/dougie_third_checkpoint_20261026.json) retains
both trials, and [decision diagnostics](results/dougie_third_checkpoint_decisions_20261026.json)
show supported reachable heights extending to 1.436 m. The strict **2.4 m
upper/lower/home mission still fails**; intermediate traversal is not promoted
to full-height completion. The third checkpoint is an existing opt-in allowance,
not a changed default or a weakened uncertainty/return gate.

Reproduce using the facility command above with `--localization-checkpoints 3`
and a new output directory. [Native footage and provenance](DOUGIE_MEDIA.md#facility-intermediate-return--26-october-2026)
show this longer exploration and clean return.

## Upper-ramp reserve and bounded correction history

The [reserve/entry diagnosis](results/dougie_reserve_entry_20261026.json) separates
two limits. In the 1.25 m-radius third-checkpoint run, return began at 612.425 s:
position uncertainty reached .180168 m and there was no eligible older anchor
inside the radius. Only ten map corrections had been used at that point, so the
16-event history limit was not the cause of that return.

The current-runtime 3 m-radius comparator retained full-rank registration and
confirmation gates, but exhausted all 16 corrections at 691.800 s. It later
stopped at 1059.445 s with `return_route_disconnected`. Physics and exposure
continued through 2400 s: .485695 µSv total, including .190428 µSv after the stop,
with zero non-foot contacts. It did not return cleanly or complete the vertical
mission. [Retained baseline report](results/dougie_loop_budget_baseline_20261026.json)
and [trigger diagnostics](results/dougie_wide_reserve_entry_20261026.json) preserve
the failed outcome. The historical earlier wide-radius failure remains separate.

`129b739` adds an opt-in **`--loop-correction-capacity 32`** alongside the historical
default of 16. This changes the bounded correction-history allowance, not the
registration/confirmation gates, 512-pose graph, 64-cloud retention or 1500-point
sampling cap. Loop diagnostics now explicitly report correction-capacity,
cooldown, cadence or disabled inhibition instead of leaving a stale last match
as the only explanation. The option is source/domain-pinned in uncertainty
profiles and requires robust fused localization. Tests exercise both history
limits, confirmation beyond event 16, rejection of rank-deficient matches and
the unchanged graph capacity. **338 tests pass.**

The fresh default-capacity 8° regression again passes at 275.585 s and
.274697 µSv, reproducing both GIF checksums exactly. The 32-event facility
experiment used the same 3 m search, three checkpoints and original mission
budget/deadline. It returned cleanly at **887.110 s**, **.226458 µSv**, with
**.036828 m** position p95 and zero non-foot contacts. It used **21 corrections**
and 284 graph poses, within the unchanged 512-pose cap.

The [capacity-pair audit](results/dougie_loop_capacity_pair_20261026.json) confirms
matching configuration except the declared history allowance and source identity,
and **13,836 identical pre-capacity physical samples**. The first physical
difference occurs at **724.250 s**, after the 16-event comparator exhausted its
history at 691.800 s. Five additional corrections were accepted with the same
registration and confirmation gates.

The [physical witness comparison](results/dougie_loop_capacity_witness_20261026.json)
retains the trade-off: the stranded 16-event run reached the intermediate landing
at 843.300 s, whereas the clean-return 32-event run did not. Both failed full-height
completion. The [complete report](results/dougie_loop_capacity_20261026.json) and
[decision history](results/dougie_loop_capacity_decisions_20261026.json) preserve
that distinction; fewer stranded seconds or lower dose do not establish greater
completed exploration. Search radius 1.25 m and history capacity 16 remain defaults.

To reproduce this bounded-resource experiment, use the facility command above
with `--localization-checkpoints 3 --loop-search-radius 3 --loop-correction-capacity 32`
and a fresh output directory. Historical uncertainty calibration does not apply
to the new runtime/domain.

## Footprint rejection details

New capture-time diagnostics retain the first missing compatible footprint cell
or incompatible adjacent support pair, including the responsible local fit
rejection. Cached rebuilds preserve this explanation; a missing-support regression
checks that diagnostics do not fill holes or change admitted nodes.

The [10° frozen replay](results/dougie_junction_footprint_20261025.json) still
reconstructs exactly 432 maximum-fusion nodes. At the failed approach, neighboring
fits have slope .176768/.176619, above the existing `tan(10°)+.0001` threshold
(approximately .176427). Their rejection propagates through the full footprint.
This is a boundary/noise limitation, not evidence permitting a higher slope limit.

The [facility frontier replay](results/dougie_facility_frontier_diagnosis_20261025.json)
locates its highest connected support at approximately (2.9, 6.9, .2983) m.
The next candidate at (3.1, 6.9, .3283) lacks compatible footprint support near
(3.5, 6.5). Other adjacent candidates are body-blocked. More supported observations
are needed; selecting a high goal alone cannot certify the missing footprint.

The [fresh 8° regression](results/dougie_footprint_regression_20261025.json) retains
strict completion at 275.585 s and .274697 µSv. **320 tests pass.** The separate
[production stance traction screen](DOUGIE_TRACTION_SCREEN.md) advances the
disturbance workstream while retaining its low-friction failures.

## Portable identical-observation replay

The committed input archive contains the first support failure: 54 scans, map
revision zero, and the correction journal. The replay reconstructs 1714 nodes
and compares both fusion modes from exactly the same observations.

Use the inputs in `docs/results/support_replay_20261024/` with
`scripts/replay_dougie_support_fusion.py`; their exact checksums and replay
outcomes are in the [portable report](results/dougie_support_fusion_portable_20261024.json).
The [final coverage replay](results/dougie_coverage_final_replay_20261024.json)
retains a separate 430-scan, revision-16 reconstruction.

```bash
docker compose run --rm -T dougie-sim python3 scripts/replay_dougie_support_fusion.py \
  --run docs/results/support_replay_20261024 \
  --output /tmp/support-fusion-replay.json
```
