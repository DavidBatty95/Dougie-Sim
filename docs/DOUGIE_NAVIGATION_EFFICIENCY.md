# Navigation efficiency development

## Stage 1: repeated work

Branch: `improve-navigation-efficiency`. The existing planning update was first
verified with 190 passing tests and committed as `e60d168`.

Changes reuse smoothing geometry and exact checked routes within one planning
cycle, skip live display predictions in headless non-ROS runs (the final map is
still archived), and invalidate layered support-footprint checks only around
changed candidate surfaces. Global spatial preprocessing and adjacency generation
remain; this is not a fully incremental mapper.

Verification: 188 tests passed initially; four archive checks detected a missing
headless final map. After restoring final-map generation, all 36 affected
mapping/motion/archive checks passed. The other 156 checks had already passed.

Matched reactor development runs: seed 3, 180 s limit, 2 uSv, fixed .5 s plan delay,
smooth/global defaults, clean-return gate. Run folders under `results/dougie/`:
`navigation_baseline_reactor` and `navigation_cached_reactor`.

Both returned at 131.220 s, travelled 15.6059 m, observed 123.875 m² surface proxy,
used 0.346027 uSv and had no non-foot contacts. Execution-smoothing p95 fell from
15.11 to 5.57 ms; total worker p95 increased from 424.79 to 463.84 ms. Wall times
were 44.74 and 43.76 s. One matched run establishes unchanged trajectory, not a
general latency improvement. Fixed scheduling is not paced real-time evidence.

Reproduce either revision with:

```bash
docker compose run --rm -T dougie-sim python3 -m dougie_sim.radiation_run \
  --world reactor --headless --fast --duration 180 --dose-budget 2 --seed 3 \
  --fixed-plan-delay .5 --require-clean
docker compose run --rm -T dougie-sim python3 -m pytest tests/dougie -q
```

## Stage 2: consistent motion costs and tracking

Graph search, checked-polyline forecasts and smoothing exposure now share the
conservative .06 m/s effective-speed model. Smoothing includes turning exposure;
smooth return forecasts include the robot's measured starting heading. Command
limits remain distinct from conservative forecast speeds. Lookahead adapts from
.2 to .4 m with commanded speed, within certified convex spans. Polyline turn
integration is vectorized to avoid Python work per densely sampled waypoint.

The matched `navigation_motion_reactor` development run returned cleanly at
131.220 s, travelled 15.5725 m, observed 127.75 m² surface proxy, and used
0.358784 uSv. Turning-in-place fell from 31.205 to 28.315 s (9.3%), but exposure
increased 3.7% with increased observed area. This does not establish matched-area
dose improvement. Its timings predate vectorized turn integration.

192 tests passed; the remaining frozen-reference cost comparison was updated to
explicitly request the historical .1 m/s speed in both implementations (the
production default intentionally changed). All 12 clustering/reference checks
then passed. Dedicated heading/search tests check the shared cost assumptions.

## Stage 3: observed-region scheduling and planar exploration

Layered missions now group fine support nodes into bounded connected navigation
regions, retain observed directed connectors, and count repeated crossings.
Region selection balances admitted candidate utility, local completion, travel
time, aging and repeated transitions. A productive distant region can beat local
work. Explicit known-target inspection retains priority. Regions are spatial
connected partitions, not semantic room recognition; changing component anchors
can reset their history. Fine-graph dose/return admission remains authoritative.

Planar frontier scoring caches clearance and visibility by map contents/geometry,
invalidating on changes. Mission timeout/exclusion timers now use ROS simulation
time; map-receipt freshness remains a wall-clock watchdog. Actions that make no
15 cm progress for 15 simulated seconds are cancelled before the 75 s deadline.

Verification: **196 tests passed**. Matrix command:

```bash
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_missions.py \
  --seeds 4 --layout-seeds 0 1 --duration 180
```

`mission_matrix_20261019T145858_672341Z`: both runs returned cleanly, each inspecting
2/8 downstairs targets. Layout 0: 115.180 s, 13.5815 m, .019279 uSv; layout 1:
119.190 s, 14.2056 m, .020480 uSv. The earlier same-seed/layout short inspection
results were 119.190 / 125.205 s and .02018 / .02120 uSv. These are development
comparisons, not full-height completion or independent qualification.

## Stage 4: vertical regression checks and prediction sensitivity

The first hierarchical mezzanine trial returned at 193.375 s but **failed** the
upper-landing witness. A local-scheduling ablation also failed (235.480 s). These
are retained as `navigation_final_mezzanine` and `navigation_local_mezzanine`.
The trace showed repeated ramp viewpoints winning over an observed fresh landing.
Region utility now decreases with the fraction of support centres already visited.

The corrected `navigation_regional_mezzanine` run passed upstairs/downstairs/home
at **237.485 s**, **25.4749 m**, **.345615 uSv**, no non-foot contacts and zero route
join rejections. The prior planning cohort's corresponding development run was
259.540 s, 26.73 m and .40788 uSv. This is an 8.5% time / 15.3% dose reduction for
the vertical round-trip objective, not proof of equal map coverage. Hierarchical
scheduling is selectable with `--region-scheduling hierarchical`; `local` is the
default comparator until broader qualification.

```bash
./dougie radiation --world mezzanine --headless --fast --duration 300 \
  --dose-budget 2 --seed 3 --gp-mode spatial --region-scheduling hierarchical \
  --fixed-plan-delay .5 --require-round-trip
```

The paced asynchronous ROS-enabled reactor trial (`navigation_paced_reactor`)
returned cleanly at 139.415 simulated / 140.248 wall seconds, .327253 uSv and no
non-foot contacts. Worker p95 was 472.37 ms; active-loop p95 was 5.175 ms, above
the 5 ms physics budget. This establishes a successful paced development run,
not hard-real-time qualification.

GP replay now sweeps upper-bound multipliers, reporting both shortfall and excess,
and optionally injects seeded position jitter plus chronological random-walk drift
into training/query coordinates. Evaluator truth remains at the actual positions.
This is offline localization-error sensitivity, **not** onboard localization.

```bash
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_gp_replay.py \
  results/dougie/navigation_baseline_reactor
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_gp_replay.py \
  results/dougie/navigation_baseline_reactor --position-sigma .05 --position-drift .005
```

On this small reactor replay (209 training / 53 held-out samples), position RMSE
was .10274 m in the perturbed case. Global log1p-rate RMSE changed .14983 → .15573;
spatial .19591 → .16127. Noise improving one model on one trace is not evidence
that localization error helps. Beta-2 bounds covered all 53 queries in both
replays, with considerable excess; this does not justify relaxing runtime bounds.

**201 tests passed**, including simulated-time progress cancellation, map-cache
invalidation, visited-connector scheduling, seeded drift and bound scoring.
Full-height qualification, geometry-aware GP kernels, integrated moving-window
likelihoods and LiDAR/inertial/leg state estimation remain separate research work.
