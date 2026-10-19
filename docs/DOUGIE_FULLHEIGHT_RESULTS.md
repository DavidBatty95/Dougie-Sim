# Full-height exploration and landmark discovery — 14 October 2026

[Project home](../README.md) · [Facility geometry](DOUGIE_FACILITY.md) · [Motion and GP methods](DOUGIE_SMOOTH_MAPPING.md)

The subsequent [planning-priority update](DOUGIE_PLANNING_PRIORITIES.md) changes
goal selection and return forecasts. The full-height results on this page retain
their original source identities; that newer planner needs a fresh full-height cohort.

## MuJoCo executes; RViz displays the same state

MuJoCo integrates the free-floating Go2, joint torques, ramp motion and contacts.
RViz subscribes to ROS robot transforms, observed maps, routes and detected targets.
It is not a separate simulation. `--headless --fast` runs MuJoCo without either
viewer or ROS. Interactive radiation mode uses RViz by default;
`--visualizer mujoco` selects the native MuJoCo viewer.

![Actual MuJoCo facility model, diagnostic transparent cutaway](media/dougie/facility-cutaway.png)

This still is an actual model render with diagnostic transparency and all eight
seeded cyan targets visible. It is **not** the robot's observation or a mission
recording. The ordinary RViz target layer publishes discovered targets only.

## Failure-driven improvement

The initial combined smooth/spatial run returned cleanly but did **not** reach the
upper-floor witness. At 925.20 s it selected a previously visited ramp representative
(score 4.940) ahead of a supported upper-landing representative (4.785), despite no
dose, rate or time rejection. The persistent slope-acquisition reward was drawing
the robot back down the ramp.

Acquisition bonuses now decrease with the existing graph-distance visit novelty.
Observed geometry, exposure limits and return admission still gate every candidate.
Headless runs also skip smoothing a display-only return-route overlay; execution
routes still receive the same geometry/radiation checks. Route-join acceptance and
cumulative rejection counts are now archived for diagnosing motion stops.

## Full-height results

Baseline gait; layout 0; 2,400 s simulated limit; 5 µSv budget; smooth tracking;
spatial GP with capacity 32 per expert, maximum 128 experts; fixed 0.5 s simulated
planner delay. The two corrected runs used detector seeds 3 and 4. Landmarks were
disabled in this cohort.

| Run | Upper witness | Lower witness | Home return | Actual dose | Distance | Non-foot impacts |
|---|---:|---:|---:|---:|---:|---:|
| Before correction, seed 3 | **Absent** | Absent | 2,345.44 s | 0.86722 µSv | 258.33 m | 0 |
| Corrected, seed 3 | **920.14 s** | 1,644.85 s | **2,365.44 s** | 1.01076 µSv | 273.54 m | **0** |
| Corrected, seed 4 | **1,231.78 s** | 2,122.09 s | **2,207.44 s** | 0.90453 µSv | 257.82 m | **0** |

Both corrected trials passed the strict upstairs → downstairs → home gate and had
zero route-join rejections. Seed 4 returned when its bounded occupancy map reached
capacity. Its observed surface proxy was 494.08 m² versus 452.64 m² for seed 3;
these are observation proxies, not certified area coverage.

![Recorded detector heights and XY trajectories, including the failed vertical mission](media/dougie/facility-results.svg)

The figure uses downsampled actual detector positions. Height is the detector
height, not a support-height estimate. The XY projection overlaps floors. The
gray trace is the initial unsuccessful vertical mission; teal and amber are the
corrected seeds. [Portable telemetry](media/dougie/facility-telemetry.json).

**Scope:** two development seeds on one layout, not a statistical repeatability
qualification or complete-building exploration. Seed 3 informed the correction;
seed 4 is an additional detector-noise check, not a new layout. Concurrent host
workloads produced approximately 87–92 wall minutes per corrected run and
10–11 s worker p95 latency. Fixed simulated scheduling masks these wall delays;
real-time behavior under load remains an open gate. No trained gait replaced the
baseline, and industrial stairs remain unqualified.

```bash
./dougie radiation --world facility --headless --fast --duration 2400 \
  --dose-budget 5 --gp-mode spatial --gp-expert-capacity 32 --gp-max-experts 128 \
  --seed 3 --fixed-plan-delay .5 --path-mode smooth --require-round-trip
```

`--require-round-trip` implies the existing clean-contact gate and additionally
requires finite, ordered upstairs/downstairs witnesses, an upright clean home
return and exposure within budget. Evaluator truth is used for scoring, never
for choosing goals. A ground-floor-only clean return now fails this explicit gate.
The checker can also rescore older archives:

```bash
python3 scripts/check_dougie_radiation.py results/dougie/RUN --require-round-trip
python3 scripts/inspect_dougie_mission.py results/dougie/RUN \
  --start 900 --end 940 --candidates
```

## Optional seeded discovery targets

```bash
# Interactive MuJoCo physics with RViz showing discovered targets:
./dougie radiation --world facility --duration 2400 --dose-budget 5 \
  --gp-mode spatial --landmark-count 8 --landmark-seed 7

# The same simulation with the native MuJoCo viewer:
./dougie radiation --world facility --duration 2400 --dose-budget 5 \
  --gp-mode spatial --landmark-count 8 --landmark-seed 7 --visualizer mujoco
```

- Count is optional, **0 by default**, or 2–64 in the facility with a clustered
  planner. Placement alternates between the two floors, checks clear supports,
  and uses a seed separate from detector noise and building layout.
- Targets are non-colliding cyan MuJoCo sites. They do not modify radiation or
  create additional obstacles. The MuJoCo viewer renders the physical scene;
  viewing from outside the robot remains a diagnostic perspective.
- An **ideal semantic sensor** samples with planning scans, approximately every
  2 simulated seconds: **4 m range**, **120° horizontal / 80° vertical FOV**, and
  scalar MuJoCo ray occlusion against walls, equipment and floor slabs. IDs and
  world positions are ideal measurements, not a trained camera detector.
- Only visible observations cross the planning-worker boundary. Unseen IDs,
  catalogue positions and floor labels are not supplied to autonomy.
- A visible target is **discovered**; a visible target within **1.5 m** is
  **inspected**. Counts are latched and repeated sightings are not double-counted.
  Already-observed safe supports near uninspected targets receive inspection
  utility; dose, rate, deadline, connectivity and return checks still apply.
- `/radiation/landmarks` is a retained ROS MarkerArray: cyan discovered cubes,
  green inspected cubes and ID/status labels. Undetected sites are not published
  in this layer. Tests exercise real late-subscriber DDS delivery.
- `landmark_observations.json` contains discovered packets;
  `landmark_evaluation.json` contains the evaluator-only catalogue and per-floor
  counts. `landmark_mission_completed` requires all targets inspected and a clean
  return. This is distinct from `round_trip_completed` and `exploration_complete`.

The initial 180 s landmark smoke (8 targets, target seed 7, detector seed 3)
returned cleanly at **145.255 s**, travelled **17.13 m**, and inspected **2/8**
targets with **0.04654 µSv** exposure and zero non-foot impacts. Both discoveries
were downstairs; **0/4 upstairs targets** were discovered. It demonstrates target
sensing/integration, not full landmark coverage or improved exploration efficiency.
The full-height cohort above predates this optional feature. A matched full-length
landmark/no-landmark experiment and layout variation are next validation steps.

## Evidence and verification

[Portable report](results/dougie_fullheight_20261014.json) includes the initial
failure, both corrected full-height runs, and the separate landmark smoke, with
configuration/source/summary identities. Latest full regression: **175 tests passed**,
including wall/slab occlusion, FOV/range limits, no unseen target leakage,
target inspection admission, scripted policy exports and DDS publications.

The latest results build on the earlier [GP/motion/residual-policy report](results/dougie_refinements_20261014.json).
[Media provenance and regeneration](DOUGIE_MEDIA.md) explains both new graphics.
