# Side-by-side radiation comparison

## Local Mac comparison profile

`./demo` now defaults to the [finite-lifetime reactor campaign](DOUGIE_CAMPAIGNS.md):
an indicative 2 µSv-equivalent physical lifetime, conservative return reserves,
replacement at the origin, retained knowledge and collision-visible wrecks.
The independent comparison described below remains available with
`DOUGIE_COMPARE_CAMPAIGN=0 ./demo` and the native comparison runner.

From your Mac, `~/Dougie-Sim/dougie-mac compare` opens
<http://localhost:8770>. The independent comparison defaults to **no cumulative
dose cap** and **no fixed time cap**; `./demo` instead enables the lifetime
campaign above. Both default to matched **simulator ground-truth pose**
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

### Confidence-qualified hotspot exclusions

New survey runs exclude an aware frontier or a connecting corridor when its
**lower-confidence predicted rate exceeds 100 µSv/h**, with **95% pointwise model
confidence**, and at least 0.25 weighted seconds of nearby detector observations.
The threshold is independent of a cumulative dose budget; the default campaign
also enforces its indicative physical lifetime, while the independent comparison
is uncapped unless configured otherwise. A high mean or uncertain
upper prediction alone is insufficient to exclude a frontier.

The lower bound is computed as `exp(mu_log - sqrt(p/(1-p)) * sigma_log)` for the
Poisson GP. Cantelli's one-sided moment bound accommodates local-expert mixture
variance and disagreement without assuming the mixture is a single Gaussian.
This confidence is conditional on the GP model, not measured real-world accuracy
or a calibrated whole-mission guarantee. Unknown, unmeasured regions are not
globally marked dangerous. Exclusions are recomputed from current beliefs at
every replan and can reopen after new observations or map reconstruction.

Both graph search and sampled execution smoothing respect exclusions. If the
robot is already in an excluded region or its ordinary return becomes excluded,
an explicit return/escape override remains available. If no admissible surveyed
work remains, it returns **radiation-limited and incomplete**. Geometric and
radiation-blocked frontiers remain distinct and stay in the unresolved count;
they are not deleted to declare frontier exhaustion. The blind ablation has no
radiation exclusions or hidden radiation information.

The dashboard shows the threshold, confidence and separate blocked-frontier and
execution-rejection counts. It also preserves the original survey stop reason
when a return subsequently experiences tracking recovery. Dose totals still
include all travel, turning, sensing and returning; a longer aware run can accrue
more total dose than an earlier-stopped blind run.

Use `--frontier-rate-limit 50 --frontier-confidence .95` with the comparison, raw
survey or batch runner to select another threshold. `--frontier-rate-limit 0`
disables these exclusions. For the Mac dashboard:

```bash
DOUGIE_COMPARE_FRONTIER_RATE_LIMIT=50 DOUGIE_COMPARE_FRONTIER_CONFIDENCE=0.95 ./demo
```

Start a fresh session after changing these settings. Retained earlier results
describe their recorded configuration and are not retroactively reclassified.

### Truth versus learned radiation

The dashboard includes two fixed-coordinate maps: **ground-truth radiation** and
the **aware robot's GP mean**, sharing a log colour scale and floor selector.
The scoring floor samples are evaluated 44 cm above their support surface.
The truth map is evaluator-only; it is never fed to navigation or GP training.
New runs export a dense GP field over observed support and use continuous local
interpolation within 35 cm horizontally and 18 cm vertically. Unmatched samples
are grey/null, never zero or filled from truth. Hovering gives truth, interpolated
mean, upper estimate, log-rate standard deviation and contributing neighbours.
The displayed log1p RMSE covers available interpolated cells only. Outlined cells
show the independent observed-floor mask. Retained older runs keep their original
nearest-query display and are labelled accordingly. See
[continuous field details and reconstruction checks](DOUGIE_WINDOWED_GP.md#continuous-field-interpolation).

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

`--replay <completed-run-directory>` initially serves retained endpoints without
launching new workers. Rerun controls can explicitly start fresh workers using
the retained mission settings. On Mac:
`./dougie-mac replay results/dougie/comparison_<timestamp>`.

### Reruns and session averages

**Learn planning between runs** adds bounded, matched-seed tuning of the aware
objective, while keeping the blind baseline fixed. The learning section shows
actual total dose/distance, tested weights, exclusions and provisional tuning
decisions. See [learning rules, persistence and controls](DOUGIE_PLANNING_LEARNING.md).

- **Rerun now** stops both current workers gracefully, retains their partial
  telemetry and outcomes, and launches a fresh pair. Repeated clicks while a
  restart is pending do not launch duplicate workers.
- **Continuously rerun** waits for both workers to end, then starts another pair
  after a short pause. Turning it off leaves the current pair running. It does
  not add a time or dose limit: an uncapped mission can remain active for a long
  time. Incomplete and failed attempts remain recorded.
- In fixed mode each new pair increments the **detector seed**, shared by blind and aware.
  World, sensor/layout seeds, pose source and mission settings remain fixed.
  These are counting-noise repeats, not independent layouts.
  Learning mode repeats each seed for retained and challenger settings before
  advancing the seed panel; its per-setting trials are labelled explicitly.
- **Average dose & distance** adds two time-aligned mean graphs alongside the
  current-run graphs. Only ended pairs contribute; interrupted/failed attempts
  with usable paired observations are included. Both robots must have recorded
  data at a plotted time for that pair to contribute. Trajectories are never
  extended after they end, and missing data is never treated as zero. Hover or
  tap for the contributing pair count and sample standard deviation. Shared
  simulation time is not necessarily matched exploration work.
- The history table separates process termination from frontier completion.
  The original pair stays in the session output directory; later pairs are
  retained under `reruns/run_00002/`, etc. `comparison_session.json` retains
  seeds, output locations, interruptions and worker exit codes. Replaying the
  session directory restores the history; continuous mode starts off unless
  explicitly enabled again. Start a new session after changing runtime code.

For command-line continuous mode, use `python3 -m dougie_sim.comparison
--continuous` inside the runtime environment. It cannot be combined with
`--exit-on-complete`. The normalized dose-versus-coverage graph has been removed;
all trajectory and average graphs use actual simulation time.

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
