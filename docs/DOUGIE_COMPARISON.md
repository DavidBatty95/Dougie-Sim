# Side-by-side radiation comparison

## Local Mac comparison profile

From your Mac, `~/Dougie-Sim/dougie-mac compare` opens
<http://localhost:8770>. The local comparison now defaults to **no cumulative
dose cap**, **no fixed time cap**, and matched **simulator ground-truth pose**
for both robots. This isolates radiation-aware routing from the short fused
localization return-reserve horizon. Select `--localization fused` when testing
onboard localization instead. The dashboard explicitly labels the pose source.

The aware robot keeps learning radiation and penalizing predicted route dose,
while pursuing observed-map frontier viewpoints. **Finished** means the planner
finds zero frontier nodes on three fresh LiDAR scans and declares completion.
A frontier is an observed support node with an occlusion-aware view of unknown
support. A visited node stays a frontier while unknown space remains visible.
Blocked or unreachable frontiers mean an incomplete stop, not finished.
The reference floor percentage is evaluation-only and never ends a mission.
The default
`--radiation-policy minimize-dose` uses predicted radiation as a route cost,
without a peak-rate-triggered return. Optional `--radiation-policy rate-limited`
restores hard rate exclusions. Known-route/physical constraints and bounded
navigation recovery remain active. This is a dose-weighted exploration heuristic,
not a claim to find the globally minimum-dose full-coverage tour. Below-100%
terminal outcomes remain visible.

An optional numeric `--dose-budget` re-enables the cumulative cap. At the raw
mission level, `--no-dose-budget` disables cumulative admission, reserve and
watchdog cutoffs while preserving sensing, dose accounting and the selected
radiation policy.

### Truth versus learned radiation

The dashboard includes two fixed-coordinate maps: **ground-truth radiation** and
the **aware robot's GP mean**, sharing a log colour scale and floor selector.
The scoring floor samples are evaluated 44 cm above their support surface.
The truth map is evaluator-only; it is never fed to navigation or GP training.
The GP panel uses the nearest exported prediction within 35 cm in XYZ and
18 cm vertically. Unmatched samples are grey/null, never zero or filled from
truth. Hovering gives truth, mean, upper bound and log-rate standard deviation.
The displayed log1p RMSE is over matched samples only, using that nearest-query
alignment. Outlined cells show the independent observed-floor mask.

The existing dose and exploration graphs are joined by actual 3D distance.
Each aware run saves `radiation_map.json`, including exact truth query positions,
nearest-query predictions, uncertainty, shared colour scale and alignment scope.

### Polished local demonstrator

The page separates matched-progress evidence, per-robot mission status, complete
trajectories, radiation-map comparison and methods. Its headline uses the first
recorded crossing of a shared reference-floor threshold (select 50/75/85/90%),
not unequal final doses. A separate card interpolates both trajectories at their
latest common simulated time. Every threshold's actual observed percentages and
crossing times remain visible, and reductions can be negative. Both incomplete
outcomes remain labelled as such; a clean return is not called finished.

`--replay <completed-run-directory>` serves retained endpoints without launching
new workers or modifying the experiment. On Mac:
`./dougie-mac replay results/dougie/comparison_<timestamp>`.

Launch two independent Go2 physics simulations and a live browser dashboard:

```bash
./dougie compare
```

Open **http://localhost:8765**. The dashboard shows radiation-blind Dougie on the
left and radiation-aware Dougie on the right, with shared graphs below:

- Cumulative simulated true radiation dose against simulation time (µSv).
- Percentage of the world's floor observed against simulation time.
- Each robot's simulation clock, dose rate, exposure, coverage and mission state.
- A top-right minimap in **each feed** showing its actual route, explored floor,
  current pose/heading, start position and estimated surveyability.

By default, both missions **target confirmed frontier exhaustion without a fixed
time cap**. Completion is the robot's own map decision, not a reference-map
percentage. A completed return or empty admissible shortlist does not mean
finished if frontier nodes remain. Optional hard dose/rate limits can stop a
mission before frontier completion.

The views are actual MuJoCo renders streamed continuously over MJPEG, targeting
**15 frames per wall-clock second** by default. Graphs and readouts refresh once
per second, independently of the video. No ROS topic isolation or desktop
window arrangement is needed. The dashboard binds to localhost.
Tall walls and overhead floors are translucent in the rendered views so the
robot stays visible; this cutaway does not change physics or LiDAR observations.

## Options

```bash
./dougie compare --world facility --seed 1
./dougie compare --world mezzanine --port 8766
./dougie compare --graphics software
./dougie compare --feed-fps 30
# Optional time-limited experiment:
./dougie compare --duration 600
```

`--graphics` also supports the existing `auto`, `nvidia` and `dri` modes.
`--duration 0` is the default (no demo time cap). A positive duration sets an
explicit simulated-time limit and retains time-for-return admission.
`--feed-fps` accepts 1–60 (default 15). Actual FPS depends on physics, planner and
rendering speed; GPU graphics can help. Frames use wall-clock scheduling and only
the latest frame is sent, avoiding a delayed video backlog. Shadows/reflections
are disabled in the feeds to reduce rendering cost. Each feed renders in a
separate process from physics, using the latest actual robot pose. Graph samples retain their
0.5 simulated-second cadence regardless of video FPS.
An optional `--dose-budget`, `--rate-limit`, `--layout-seed` and `--sensor-seed` configure both
missions identically. `--exit-on-complete` closes the dashboard after both workers
exit (useful for automated smoke runs). Otherwise final graphs and views remain
available until **Ctrl+C**, which also stops any running simulations.

## Sliced minimaps and survey outcomes

The reactor has a single floor view. Multilevel scenes detect dominant floor and
landing elevations from the scoring reference. The **robot's current elevation
slice expands automatically**, with compact previews of the other levels. The
facility includes the intermediate landing; ramps connect adjacent slices.
Vertical elevation is **Z in MuJoCo** (the requested vertical/Y-sliced view).
Each slice has its own coverage percentage and route, so overlapping floors do
not colour each other as explored. Routes are accumulated on physics ticks,
including motion between video frames, and use the panel's orange/teal colour.

| Colour / symbol | Meaning |
|---|---|
| Green | Observed floor |
| Blue | Unobserved reference samples near known, currently admissible support |
| Red | Predicted rate or radiation-blocked outbound/return route |
| Amber | Predicted trip exceeds remaining dose allowance |
| Grey | Unknown or otherwise unclassified/unavailable support |
| White arrow / square | Current robot pose and heading / starting position |

Risk/budget colours take priority over green. Admission uses the robot's observed
support graph, GP upper predictions and return/dose constraints. It is an estimate
at the latest planning update, not proof that an unseen room is safe or a true
hotspot is present. Unseen areas are not extrapolated into "can explore" areas.
The true pose is used only to align those display annotations to the reference.

The dashboard and `survey_report.json` distinguish:

- `frontiers_exhausted`: zero observed-map frontiers on three fresh scans; the
  planner explicitly declared `finished`.
- `radiation_limited`: a radiation/risk/dose condition caused the return/stop.
- `frontiers_blocked`: frontier nodes remain but none can currently be resolved;
  this is not mission completion.
- `navigation_stopped` / `incomplete_return`: localization, route recovery,
  map capacity or other navigation limits prevented further survey.
- `time_limit`, `interrupted`, or `failed`.

Both robots retain their physical/navigation constraints. The goal is exhaustion
of observed-map frontiers; it does not guarantee true coverage of inaccessible surfaces or
force continued motion after an unrecoverable navigation failure. No coverage
percentage is renormalized to make a partial survey look complete.

## Comparison contract

Both workers use the same world, starting state, policy, exploration planner,
mission deadline and seed settings. They run independently, as fast as the host
allows, with the same fixed 0.5 simulated-second planner latency. They can reach
different simulation times at the same wall-clock instant; graphs use each
robot's own **simulation time**, not dashboard arrival time.

The blind ablation receives zero exposure/risk/information-gain inputs and ignores
radiation-triggered motion inhibition and dose-triggered watchdog stops. Geometry,
localization quality, time limits and physical recovery behavior remain active.
The real field and detector still operate and are recorded for evaluation. The
aware worker retains radiation-aware route and dose admission. Different paths
produce different observations even with matching random seeds.

### Dose

Both curves integrate the true simulated field along the actual robot trajectory,
independently of the GP estimate. Units are the simulator's **dose-equivalent
proxy in µSv**, not absorbed energy in gray. The survey comparison ends each
mission at planner-declared frontier exhaustion, a completed return, an unrecoverable stop, or
the optional time cap. Dose is integrated through that endpoint. Its curve is
not padded with hypothetical data; a lower dose after an early failure is not
evidence of equivalent survey work.

### Exploration percentage

The evaluator samples exposed ground, floors, decks, ramps and steps at 25 cm
spacing, excluding exterior ground and floor samples covered by nearby obstacles.
Overlapping storeys count separately. Exploration is **3D and physical-arrival
gated**, using the robot's capture-time foot-support position rather than its
body or sensor height:

- Flat floor elevations must first be physically reached: the support position
  must be within 35 cm in 3D and 8 cm vertically of a reference surface there.
- A fresh LiDAR return must lie within 19 cm of the reference sample, and that
  sample must be within 15 cm vertically of the current support height.
- Ramp coverage grows progressively with the climb. Entering an upper minimap
  display slice midway up a ramp does not unlock the upper floor itself.
- Glimpses of upstairs from downstairs are not counted or credited retroactively
  on arrival. Returning downstairs also cannot add new upstairs coverage.

True capture pose is used solely for scoring. Coverage is the cumulative fraction
of these fixed samples. Graphs, minimap shading and slice percentages all use
the same mask; it never determines mission completion. Saved artifacts identify the metric
as `physically_reached_3d_floor_v2`; earlier run percentages are not recalculated.
The reference and score are never sent to navigation.

This is a sampled **observed-floor-area** metric, not a reachable-space guarantee,
an occupancy-map completion test, or the percentage of already-discovered map.
Equipment tops do not increase it. Review coverage and mission outcome together
with dose: lower exposure alone does not establish equivalent work completed.

## Saved results

Each invocation creates `results/dougie/comparison_<timestamp>/` containing:

- `comparison_config.json`: matched commands, settings and metric definitions.
- `blind.log` and `aware.log`: independent worker logs.
- `comparison_status.json`: worker exit codes once both finish.
- `blind/` and `aware/`: ordinary mission artifacts, plus `comparison.jsonl`
  (time-series samples), `coverage_reference.npz` (fixed scoring points), and
  `frame.jpg` (latest view). `stream_performance.json` records target FPS, rendered
  frame count, average rendered FPS and rendering time when the mission ends.
- `blind/survey_report.json` and `aware/survey_report.json`: survey outcome,
  coverage counts and current admission categories.
- Each worker's `minimap_final.npz`: floor reference, observed mask, admission
  classes, per-slice route rasters and slice elevations for offline inspection.

If a worker fails, its panel reports the nonzero exit status; inspect its log and
`radiation_summary.json`. Aware missions also return nonzero when their true dose
exceeds the configured budget. Exceeding that budget is an evaluation outcome,
not an automatic failure for the deliberately radiation-blind ablation.
