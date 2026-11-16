# Paired multi-seed experiments

Run a cohort of **30 paired seeds = 60 physical missions** and aggregate the
retained outcomes. Each seed gets one radiation-blind and one radiation-aware
mission with matching world, initial state, detector/sensor/layout settings,
policy, pose source and fixed 0.5-s simulated planning latency.

## One-command Apple Silicon cohort

```bash
./dougie-mac batch
./dougie-mac batch-report
./dougie-mac batch-logs -f
./dougie-mac batch-status
```

This runs in a background container, so closing the terminal does not stop it.
Defaults are **facility**, detector seeds **1–30**, **one concurrent pair**, ideal
pose, dose-weighted routing, **no cumulative dose cap**, and **no simulated-time
cap**. Batch mode skips video/rendering and display-only GP maps, but retains
physical simulation, detector observations, 3D distance, reference-floor scoring,
per-floor masks and frontier-based outcomes.

The default inference profile is the [online windowed GP](DOUGIE_WINDOWED_GP.md),
with local posterior updates and disk-paged inactive regions. Its bounded working
set does not impose a global GP sample cutoff. Use a new output directory after
updating the runtime rather than resuming a cohort with different source hashes.

The report is `results/dougie/facility_30_seeds/report.html`, refreshed while work
continues. It opens as a local file, with no dashboard-port conflict. JSON, CSV,
Markdown and SVG plots are written alongside it. An uncapped facility pair can
take significantly longer than a reactor pair, or retain unresolved frontiers;
the batch does not force a completion claim.

## Resource controls and resumption

```bash
# Optional external resource watchdog: censor a pair after 30 wall minutes.
DOUGIE_BATCH_WALL_TIMEOUT=1800 ./dougie-mac batch

# Stop gracefully, retaining active work and completed seeds:
./dougie-mac batch-stop
# Resume with the SAME protocol/environment settings:
DOUGIE_BATCH_WALL_TIMEOUT=1800 ./dougie-mac batch
```

One worker means one concurrent **pair**, not one mission: it already runs two
simulators and their planners. Start with one pair on an 8-GB Docker allocation.
`DOUGIE_BATCH_WORKERS=2` permits two pairs if CPU and memory allow.

Already terminal seeds, including failed/timed-out ones, are retained and skipped
on resume. Interrupted/crashed attempts are archived under
`interrupted_attempts/` before retry; their raw files are not erased. The analysis
unit is the latest uninterrupted or terminal attempt. The protocol and runtime
source hashes must match on resume, preventing accidental mixing of code or
settings. Choose a new output directory after changing those settings. Worker
parallelism can change without changing the scientific protocol.

The optional wall timeout is **not a simulated mission deadline or dose budget**.
Its result is explicitly censored, with stop reason and process codes retained.
It is never counted as confirmed frontier completion. Without a timeout, an
uncapped mission can keep working on persistent frontiers indefinitely.

## Configuration

| Environment variable | Default |
|---|---|
| `DOUGIE_BATCH_WORLD` | `facility` (`mezzanine` / `reactor` also supported) |
| `DOUGIE_BATCH_SEEDS` | `30` paired seeds |
| `DOUGIE_BATCH_SEED_START` | `1` |
| `DOUGIE_BATCH_SEED_AXIS` | `detector` (`sensor` / `both` also supported) |
| `DOUGIE_BATCH_LOCALIZATION` | `ground-truth` (`fused` also supported) |
| `DOUGIE_BATCH_DURATION` | `0` (no simulated-time cap) |
| `DOUGIE_BATCH_WORKERS` | `1` concurrent pair |
| `DOUGIE_BATCH_WALL_TIMEOUT` | `0` (no wall watchdog) |
| `DOUGIE_BATCH_OUTPUT` | `results/dougie/facility_30_seeds` |
| `DOUGIE_BATCH_EXPOSURE_POLICY` | `responsive`; experimental `pathwise` is also available |

For a custom output, export the setting so report/status commands use the same
directory:

```bash
export DOUGIE_BATCH_OUTPUT=results/dougie/facility_sensor_30
export DOUGIE_BATCH_SEED_AXIS=both
export DOUGIE_BATCH_LOCALIZATION=fused
./dougie-mac batch
./dougie-mac batch-report
```

The default varies **detector counting noise on a fixed layout**, matching the
single-pair planning comparison. It is not 30 independent environments. In ideal
pose mode, the sensor seed does not introduce fused-sensor noise. For layout
variation and explicit seed lists, use the module directly:

```bash
docker compose -f docker-compose.batch.yml run --rm batch \
  python3 -m dougie_sim.batch_comparison --world facility \
  --seeds 1 2 3 4 5 --seed-axis both --vary-layout --localization fused \
  --workers 1 --wall-timeout 1800 --output results/dougie/facility_layout_study --resume
```

`--vary-layout` advances the facility layout seed once per requested pair. Both
robots within each pair still receive the same layout. For Linux, the same
module can run in the existing runtime with
`docker compose run --rm -T dougie-sim python3 -m dougie_sim.batch_comparison ...`.

## How averaging works

- **Every requested seed is accounted for.** Reports show pending, running,
  terminal, failed, missing-summary and censored counts. Incomplete physical
  missions are not silently discarded.
- **Endpoint averages are descriptive.** Mean, sample SD, median and fixed-seed
  bootstrap 95% intervals are provided for dose, actual 3D distance, reference
  observation, simulated duration and remaining frontiers. Different terminal
  work/duration makes endpoint dose alone an unsuitable effectiveness headline.
- **Matched milestones are paired.** Dose comparisons at 10/25/50/75/90%
  reference-floor thresholds include a seed only when *both* trajectories reach
  that threshold. Eligible `n / requested` is shown, with paired aware-minus-blind
  differences and relative reductions. Negative dose differences favour awareness;
  worse-aware results remain negative reductions. Actual threshold overshoots and
  crossing times are retained. The percentages never terminate a mission.
- **Time curves are not padded.** Each time point uses only pairs with recorded
  observations for both modes at that time. Stopped trajectories are not carried
  forward or extrapolated. The changing `paired_n` is explicit; later means are
  conditional on that surviving observed subset. Shading is a pointwise bootstrap
  interval for the mean, not a simultaneous band or a safety guarantee.
  JSON records the RNG seed, resample counts, SD convention and analysis-code hash.
- **Multilevel performance is separate.** Reports retain per-elevation observation
  means and upper-landing physical witnesses. Large downstairs coverage does not
  imply an upper floor was explored.
- **Completion stays robot-defined.** Confirmed completion requires `finished`,
  zero frontier nodes and three fresh zero-frontier scans, with no process error
  or external censoring. A return or a high reference score is not completion.

With a single eligible seed, mean is reported but SD and confidence intervals are
unavailable. With no eligible seeds, results are null—not a fabricated 0 µSv or
0% reduction. Thirty noise repeats still do not establish generality across
worlds, source/shield layouts or physical hardware.

## Artifacts

| File | Contents |
|---|---|
| `batch_config.json`, `source_snapshot.tar.gz` | Immutable protocol, source identities and source snapshot |
| `seed_00001/`, … | Paired commands, process outcomes and complete mission artifacts |
| `pair_status.json` | Lifecycle, errors, censoring and actual worker exit codes |
| `batch_summary.json` | Means/SD/median/intervals, paired milestones and unpadded curves |
| `runs.csv` | One row per requested seed and mode, including missing outcomes |
| `REPORT.md` | Human-readable outcomes, statistics and per-floor observations |
| `report.html`, `batch_mean_curves.svg` | Auto-refreshing local results page and uncertainty plots |

Recompute reports without rerunning physics:

```bash
docker compose -f docker-compose.batch.yml run --rm batch \
  python3 -m dougie_sim.batch_comparison --report-only \
  --output results/dougie/facility_30_seeds
```

Use report-only after the running batch has stopped, so there is a single report
writer. Raw worker files and failures remain unchanged.
