# Planning, inspection and scalability priorities — 15 October 2026

[Project home](../README.md) · [Full-height history](DOUGIE_FULLHEIGHT_RESULTS.md) · [Portable evidence](results/dougie_priorities_20261015.json)

This implements the first planning/evaluation tranche of the improvement assessment.
For the subsequent sensor-fusion, geometry-kernel and moving-window implementation,
see the [onboard methods](DOUGIE_ONBOARD_MISSION.md) and [active plan](DOUGIE_ONBOARD_PLAN.md).
The results below retain their original ideal-pose configuration and identities.
The baseline gait and ideal pose remain in use. Full-height results from the
previous planner are historical evidence, not qualification of this new revision.

## Implemented

### Persistent observed-frontier agenda

The clustered planner retains up to **2,048 candidate tasks**, recording visited,
pending, active, disconnected/rate-blocked, temporarily blocked, dose-limited,
time-limited and lost-support states. Every selected task is revalidated against
current observations and admission constraints. Previously remembered candidates
remain in the representative pool instead of disappearing solely because sampling
of a region changed.

The former six-metre hard candidate limit is now a local preference. When nearby
admissible work is exhausted, the planner can reposition along observed support to
a distant candidate. No unknown support, supplied building map or hidden target
position is used. This is a persistent candidate agenda, not yet a room/floor
topological mission planner or a proof that all frontiers have been exhausted.

Agenda statuses include the last assessment timestamp. Pending entries are not a
claim of present admissibility; admission is rerun before selection. Storage and
the trace preview are bounded, with overflow attempts reported. Only the current
selected exploration task is marked active.

### Explicit outbound and return-route risk

Graph search now supports a hard predicted-rate ceiling. Both outward search and
reverse-directed home search apply it, so a longer compliant route can be found
instead of rejecting a goal merely because a shorter route crosses a hot node.
Candidate admission requires a compliant return and checks the peak rates of both
selected graph routes.

If new predictions leave no rate-compliant route home, new exploration is stopped.
The executive uses the available observed dose-weighted retreat route and explicitly
reports `emergency_rate_override`; this is an emergency exception, not compliance
with the rate limit. If no observed return route exists, motion is withheld.
These remain discretized estimated-field checks, not continuous dose guarantees.

### Execution-time-aware forecasts

Route-tree time/exposure forecasts include conservative 0.06 m/s travel and
turning at 0.35 rad/s. Turn exposure is charged at the turn location. Inspection
dwell and separate planning waits are reserved. The wait allowance is at least
3 s and increases to the 95th percentile of the last 128 observed simulated plan
delays. This does not convert fixed-schedule wall waits into real-world latency.

Raw grid turns initially made forecasts too conservative: a 180 s inspection
trial returned at 63.05 s after inspecting only one target. Smooth execution now
forecasts its geometry-checked smooth home route. Up to four otherwise-rejected
known-target candidates per replan receive checked smooth outbound/return
forecasts before final rejection. Limits remain active after that refinement.
Both intermediate unsuccessful inspection results are retained.

### Inspection objective

`--mission-objective survey` remains the default. `inspect` gives priority to
admissible approaches to **already-observed, uninspected landmarks**, then resumes
ordinary frontier search. It never receives unseen catalogue positions.

```bash
./dougie radiation --world facility --headless --fast --duration 180 \
  --dose-budget 5 --gp-mode spatial --seed 3 --fixed-plan-delay .5 \
  --landmark-count 8 --landmark-seed 7 --mission-objective inspect --require-clean
```

### Bounded occupancy compaction

When occupancy storage reaches 75% of capacity, fully observed **2×2×2** blocks
whose eight cells all have confirmed free evidence can be compacted. Their least
confident child evidence is retained. Unknown, occupied and uncertain cells are
never merged into free blocks. Later hits expand a block; if expansion cannot fit,
the coarse free claim is discarded before the hit is stored. Remaining capacity
exhaustion still triggers return.

Replaying the previously saturated facility archive reduced storage records from
**200,000 to 110,393 (44.8%)**, preserving all **28,393 occupied/uncertain cells**
and the represented fine-cell extent. This is a record-count reduction, not an RSS
measurement or a new long-duration mission result. `layered_geometry.npz` schema 2
adds coarse-free keys/evidence and their voxel size; consumers must include these
when reconstructing the complete occupancy evidence. Support/obstacle geometry
used for walking is not evicted by this compression.

### Diagnostics and prediction scoring

- Worker timings separate observations, mapping updates, planning/graph work,
  execution smoothing, display prediction and return-overlay/metrics work.
  Per-stage p95 metrics use the latest 256 plans.
- Traces include plan age, route forecasts, candidate rejection reasons and an
  agenda preview. RViz/MuJoCo dashboards show pending known targets and return time.
- Observed-support visit fraction is a **proxy**, not complete area coverage.
- GP replay now scores near/far queries, high-rate queries, shielding-boundary
  proximity and sensor-height bands. It reports RMSE, upper-rate coverage,
  shortfall and Poisson deviance evaluated at the predicted mean. That deviance is
  not a full Poisson-lognormal predictive likelihood or a calibration guarantee.

On the retained 3,233-training / 809-query facility replay, global-512 log1p-rate
RMSE was **0.057 near training measurements versus 0.651 farther than 0.5 m**.
Spatial GP was **0.066 / 0.761**. These diagnostics expose where future model work
is needed; the GP model itself was not changed in this tranche. Truth geometry and
source information are used only by the offline scorer.

## Measured development outcomes

| Trial | Inspected targets | Return time | Actual dose | Result |
|---|---:|---:|---:|---|
| Historical facility smoke, seed 3/layout 0 | 2/8 | 145.255 s | 0.04654 µSv | Clean |
| Initial grid-turn forecast | 1/8 | 63.050 s | 0.01776 µSv | Clean return, inspection regression |
| Smooth home forecast only | 1/8 | 129.215 s | 0.03239 µSv | Clean return, inspection incomplete |
| Final inspection mode, seed 3/layout 0 | **2/8** | **131.220 s** | **0.02225 µSv** | Clean |
| Final inspection mode, seed 4/layout 0 | **2/8** | **119.190 s** | **0.02018 µSv** | Clean |
| Final inspection mode, seed 4/layout 1 | **2/8** | **125.205 s** | **0.02120 µSv** | Clean |

All inspected targets were downstairs. The final seed-3 run inspected the same
IDs as the historical smoke, with about 52% lower exposure, but mapped less area.
This supports inspection efficiency in a small development cohort, not general
coverage or avoidance superiority. The eight-target mission remains incomplete.

The final mezzanine run passed upstairs/downstairs/home at **259.540 s**, travelled
**26.73 m**, and recorded zero non-foot impacts. Its **0.40788 µSv** exposure is
**higher** than the historical smooth/spatial run's 0.15818 µSv. This regression
is retained; changed exploration goals do not universally reduce dose.

A paced, asynchronous, ROS-enabled reactor run returned at **73.665 simulated
seconds / 74.253 wall seconds**, using **0.19887 µSv**, with no non-foot impacts.
Its active-loop p95 was **5.21 ms**, so this is not hard-real-time qualification.
No new full-height, paced or complete-landmark qualification is claimed.

## Repeatable evaluation commands

```bash
# Explicit detector/layout matrix; retains every trial and failure log.
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_missions.py \
  --seeds 4 --layout-seeds 0 1 --duration 180

# Controlled routing microbenchmark and retained-map compaction replay.
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_priorities.py \
  --occupancy-archive results/dougie/radiation_20261014T152104_956260Z/layered_geometry.npz

# Stratified prediction scoring, without assimilating held-out counts.
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_gp_replay.py \
  results/dougie/radiation_20261012T175641_497545Z
```

The controlled routing benchmark uses prescribed rates on the same observed graph.
The constrained route reaches the goal around a hot shortcut; reactive stopping
does not complete it. Its reported exposure is graph-edge integration, not a
physical learned-GP experiment. Full closed-loop matched-goal avoidance comparisons
remain necessary. Matrix `passed_required_gate` means clean return (or explicit
round trip), not all targets inspected; the separate completion flags are retained.

Verification: **190 tests passed**. The portable report preserves successes,
regressions, source identities, routing/compaction checks and stratified GP scores.

## Remaining assessment priorities

1. Full-length inspection and round-trip matrices on unseen layouts; quantify
   dose versus matched coverage, including the mezzanine exposure regression.
2. Room/connector topology, better global task scheduling and incremental graph
   work. Map compaction alone does not solve all planning latency.
3. Calibrated, geometry-aware radiation prediction and moving-window likelihoods.
4. Noisy sensing, estimated localization and camera-based target recognition.
5. Independently qualified locomotion extensions; baseline remains the default.
