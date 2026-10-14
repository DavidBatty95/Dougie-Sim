# Multi-floor implementation progress

The complete agreed plan is saved in
[`DOUGIE_MULTIFLOOR_PLAN.txt`](DOUGIE_MULTIFLOOR_PLAN.txt).
This page distinguishes implemented foundations and measured development runs
from the remaining qualification gates. Date: 12 October 2026.

**Later implementation:** [full-height facility, corrected Go2 RViz model,
perceptive-policy training and retained outcomes](DOUGIE_FACILITY.md). The
measurements below preserve the earlier low-gallery milestone.

## Run the new vertical exploration fixture

```bash
# Interactive RViz: two 10-degree ramps and a 0.705 m inspection gallery.
./dougie radiation --world mezzanine --duration 300 --dose-budget 2 --seed 3

# Reproducible simulated planner schedule; no display or ROS required.
./dougie radiation --world mezzanine --headless --fast --duration 300 \
  --dose-budget 2 --seed 3 --fixed-plan-delay .5
```

The robot gets observed range returns, ideal pose and noisy detector counts, not
the ramp coordinates, floor map or source locations. `mezzanine` automatically
selects its two-source scenario and experimental `layered` terrain profile.
The existing reactor command retains its conservative terrain profile and policy.
The new fixture is a low gallery, **not a qualified full-height storey**.

## Implemented

### Versioned policy contract (M1 foundation)

`config/dougie_policy.json` and `dougie_sim/policy.py` define model/weight hashes,
joint order, observation schema, timing, scales, gains, effort limit and provenance.
Invalid hashes/order/schema fail before execution. The default adapter produces
identical actions to the original pinned TorchScript artifact. Alternative
policies can be selected with `--policy-config` in radiation and qualification
commands without replacing downloaded baseline assets.

This milestone used stateless **proprio-45**. The later facility implementation
adds an explicitly versioned sparse-LiDAR schema; unversioned or recurrent
observations are still rejected rather than interpreted as proprio-45.

### Range packets and multi-layer geometry (M2 foundation)

- Detached, read-only organized range packets retain origins, directions,
  distances/no-return information, timestamps and sequence numbers.
- Normal orientation rejects ceiling undersides as support.
- Experimental layered mapping fits local planes to sensor returns, checks local
  continuity separately, and differentiates ramps from abrupt ledges. It retains
  vertically separated supports and refuses industrial stair admission.
- Bounded occupied/free voxel evidence stops at returns; no clearing behind walls.
  Saturation is reported and triggers return reasoning in layered missions.
- Return searches now use the reversed directed graph; slope-edge time estimates
  can differ for ascent/descent. They remain heuristics needing calibration.
- Layered missions run the single-owner mapper/planner in a spawned process,
  transferring detached observations and final archive state. This isolates
  Python-heavy mapping from the control-loop interpreter lock. Baseline profiles
  retain the lighter thread worker; native thread limits are set before control.
- The mezzanine has two physical ramp connections and explicitly attenuating
  slab/shield geometry. Rotated attenuation boxes are rejected because the field
  currently implements axis-aligned shielding.

The voxel evidence is a mapping foundation, not a completed persistent SLAM map or
dynamic clearing solution. It does not yet supply a full frontier lifecycle.

### Sensor correctness discovery

MuJoCo 3.3.6's `mj_multiRay` uses corner-only angular bounding boxes. On a broad
tilted ramp it returned the floor behind part of the ramp where `mj_ray` correctly
hit the ramp. Infinite planes can also be incorrectly excluded by distance to
their origin. The simulated 3-D scanner now applies exact vectorized box/plane
intersections before constructing observations. No geometry identity leaves the
sensor. Regression tests compare ramp/slab returns with scalar MuJoCo raycasts.

This correction changes some observed geometry relative to frozen older trials.
Older result cohorts retain their original source identity; do not reinterpret
them as tests of the new sensor implementation.

### Independent physical witnesses and archives

An evaluator-only witness records 0.2 s upper-landing foot-pose dwell followed by
lower-level return. It never supplies goals or geometry to the planner.
New missions archive:

- `scene.xml` and `policy_contract.json`.
- `motion_trace.npz`: 20 Hz time, base/joint pose and commanded velocities.
- `layered_geometry.npz` for layered runs: support returns and signed voxel evidence.
- Existing detector, GP, configuration, source identity and outcome artifacts.

`mission_completed` retains its historical meaning of returning home.
`exploration_complete` is explicitly false: frontier exhaustion has not yet been
validated as full unknown-space exploration. Layered area metrics count observed
surface layers per XY cell, avoiding multiple height-bin counts along a ramp.

## Measured physical missions

Same new mezzanine, baseline walking policy, 2 micro-sievert budget, 300 s limit,
fixed 0.5 s simulated planner delay. These are development repeats, not holdouts.

| Seed | Upper landing witnessed | Lower return witnessed | Home return | Actual dose | Travel | Non-foot impulse |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 69.32 s | 206.71 s | 215.43 s | 0.32923 µSv | 22.14 m | 0 N·s |
| 3 | 90.08 s | 163.15 s | 263.55 s | 0.18853 µSv | 29.75 m | 0 N·s |

Both returned upright; detector height reached approximately 1.15 m versus
approximately 0.4 m downstairs. The runs were concurrent and reached about 2×
real-time throughput; this is not an isolated performance benchmark. Planner p95
was approximately 1.5 s, and active-loop p99 approximately 18 ms in fast mode,
including waits for the scheduled worker result. The new mapping/sensor path
requires further latency work before claiming the plan's real-time gates.

Retained evidence:

- `radiation_20261012T124526_560494Z`: early trial stayed downstairs; retained.
- `radiation_20261012T125129_272095Z`: first successful elevated trajectory before
  adding the explicit landing witness; seed 3 reproduced the same terminal pose,
  path length and exposure in the later witnessed run.
- `radiation_20261012T132750_531887Z`: witnessed seed 1.
- `radiation_20261012T132750_942684Z`: witnessed seed 3.

All are under `results/dougie/`. The portable development report is
[`results/dougie_multifloor_development_20261012.json`](results/dougie_multifloor_development_20261012.json).

### Paced ROS trial and process isolation

A subsequent 300-second **paced ROS** trial, before process isolation,
`radiation_20261012T133736_125580Z`, reached the gallery but did not return before
the deadline. Planner p95 reached 5.63 s and freshness gates repeatedly stopped
motion. This failure is retained; fast-mode success did not imply paced success.

After moving layered planning into a spawned process, paced ROS run
`radiation_20261012T135002_825818Z` returned cleanly at 293.52 s with 0.18868 µSv
actual exposure. It climbed part of the ramp but **did not witness the upper
landing**, so it is not a successful vertical survey. Other heavy simulations
were running on the host; these are operational trials, not isolated benchmarks.

Support grouping and neighbourhood queries were then batched. Final fixed-delay
process run `radiation_20261012T135736_077136Z` reproduced seed 3's upper/lower
witnesses, terminal pose, 29.75 m path and 0.18853 µSv exposure. It ran at **2.50×
real time**, with main-loop p95 **0.75 ms**, p99 **1.13 ms** and planner p95
**663 ms**. The 263 over-budget steps still include scheduled scan/worker waits;
this is not hard-real-time qualification. A full paced vertical round trip remains
a gate, especially under external CPU load.

## GPU training is runnable

```bash
./dougie training-setup
./dougie train --smoke --num-envs 16 --iterations 2

# Short development experiment, initializing only the actor from the baseline.
./dougie train --terrain small-stairs --num-envs 64 --iterations 50 \
  --initialize-baseline --quiet

# Continue a specific checkpoint; this retains its actor/critic/optimizer state.
./dougie train --terrain small-stairs --num-envs 64 --iterations 50 \
  --resume results/dougie/TRAINING_RUN/checkpoint.pt --quiet
```

Setup installs pinned core packages into ignored `.dougie/training`, leaving the
ROS/CPU container separate. The tested stack is mjlab 1.6.0, MuJoCo/Warp 3.11.0,
Warp 1.18.0, PyTorch 2.14.1 CUDA 13 and RSL-RL 5.4.2. Full resolved dependencies,
task YAML, agent JSON, source hashes and hardware identity accompany each run.
The first GPU kernel compilation took several minutes; later starts reused cache.

The Go2 task preserves physical primitives/inertias, 1 mm margins, explicit PD
gains, 23.5 Nm limits, 5 ms physics and 20 ms actions. Visual meshes are omitted
from training. Warp's native/multicontact box CCD does not support those margins,
so those backends are disabled in training. This simulator-version/contact-backend
difference is recorded and requires native-MuJoCo evaluation.

An explicit action adapter corrects mjlab's default XML-order lookup to the
deployment FR/FL/RR/RL contract. A 16-environment baseline GPU rollout completed
768 transitions with finite observations/rewards. A 64-environment PPO smoke run
completed 15,360 transitions on flat/ascending/descending 2–6 cm training terrain.

A longer initialized experiment reached checkpoint `model_100.pt` before the
execution timeout. It was resumed for 50 additional iterations (153,600 additional
transitions) and exported in `training_20261012T131949_350461Z`. GPU-to-export parity
passed, followed by parity on **128 saved observations in the native CPU container**:

```bash
docker compose run --rm -T dougie-sim python3 scripts/check_dougie_policy.py \
  results/dougie/training_20261012T131949_350461Z/policy.json

./dougie terrain-qualify --cases flat stairs_up_060 stairs_down_060 \
  ramp_up_10 ramp_down_10 --seeds 1 2 --workers 2 \
  --policy-config results/dougie/training_20261012T131949_350461Z/policy.json
```

### Candidate policy screening

| Case | Traversed | Strict passes |
|---|---:|---:|
| Flat | 2/2 | 2/2 |
| 6 cm ascent | 2/2 | 1/2 |
| 6 cm descent | 2/2 | 0/2 |
| 10° ascent ramp | 2/2 | 2/2 |
| 10° descent ramp | 2/2 | 2/2 |

Failures were non-foot impacts. This checkpoint **does not qualify stairs** and
has not replaced the baseline. Results live in
`terrain_20261012T132245_895958Z`. Training currently uses a proprioceptive actor
and a privileged height/velocity critic. It establishes the GPU training,
resume/export/evaluation loop, not the planned perceptive stair policy.

## Next gates

1. Add the deployable local terrain observation/schema, with the same scanner
   visibility, validity masks, delay and dropout in training and deployment.
2. Add stop/restart and controlled descent objectives; qualify held-out flights
   before widening terrain admission beyond ramps.
3. Build connector state machines and explicit unresolved-frontier accounting;
   add observation-driven floor slicing and fair map allocation across floors.
4. Reduce local sensing/graph-build tail latency and test paced missions under load.
5. Expand from this low gallery to full-height two/three-floor procedural layouts.
6. Add LiDAR/IMU/leg localization and requalify with pose uncertainty and loop closure.
7. Run the sealed, independently scored reliability campaign from the plan.

No new real-hardware or industrial-stair capability is claimed by this package.

## Verification

Final CPU/ROS regression run: **89 tests passed** in 19.88 seconds.

The CPU/ROS regression suite covers policy contracts, numerical export parity,
sensor detachment, ceiling rejection, scalar/batched ramp ray agreement,
slope/ledge separation, stacked floors, reverse-directed costs, occupancy bounds,
shielding, landing witnesses and real DDS publication. Run it with `./dougie test`.
GPU rollout/training and native screening are separate checks documented above.
