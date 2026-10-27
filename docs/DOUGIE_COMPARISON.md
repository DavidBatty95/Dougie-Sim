# Side-by-side radiation comparison

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

By default, both missions now **target 100% observed-floor coverage without a
fixed time cap**. They systematically sweep remaining admitted support viewpoints
instead of returning as soon as the ordinary cluster shortlist is empty.
Radiation-aware Dougie can return earlier when predicted radiation or its dose
budget excludes further survey. A completed return alone is not reported as
100% exploration.

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
`--dose-budget`, `--rate-limit`, `--layout-seed` and `--sensor-seed` configure both
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

- `survey_complete`: all fixed reference samples have actually been observed.
- `radiation_limited`: a radiation/risk/dose condition caused the return/stop.
- `observed_frontiers_exhausted`: no remaining admitted observed viewpoints;
  coverage is still reported honestly, even when below 100%.
- `navigation_stopped` / `incomplete_return`: localization, route recovery,
  map capacity or other navigation limits prevented further survey.
- `time_limit`, `interrupted`, or `failed`.

Both robots retain their physical/navigation constraints. The feature sets 100%
as the goal; it does not guarantee complete coverage of inaccessible surfaces or
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
mission at full observed coverage, a completed return, an unrecoverable stop, or
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
of these fixed samples, and the graphs, minimap shading, slice percentages and
100% completion check all use the same mask. Saved artifacts identify the metric
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
