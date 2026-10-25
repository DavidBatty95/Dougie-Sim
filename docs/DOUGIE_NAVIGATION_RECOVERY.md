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
