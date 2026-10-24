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
