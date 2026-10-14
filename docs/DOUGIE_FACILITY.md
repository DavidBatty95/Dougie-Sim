# Full-height facility, perceptive training and the RViz Go2 model

Development record: 12 October 2026. This extends the
[multi-floor plan](DOUGIE_MULTIFLOOR_PLAN.txt) and the earlier
[low-mezzanine milestone](DOUGIE_MULTIFLOOR_PROGRESS.md).

**14 October update:** [smooth/spatial full-height results and discovery targets](DOUGIE_FULLHEIGHT_RESULTS.md)
now include two completed round trips, the retained initial failure, a strict
`--require-round-trip` evaluator gate, and optional seeded landmark inspection.
The 12 October results below are retained as the historical grid/global cohort.

## Run

```bash
# Full-height building, cutaway view and the verified Go2 visual meshes.
./dougie radiation --world facility --duration 2400 --dose-budget 5 \
  --gp-capacity 256 --seed 3

# Reproduce the successful physical development run with fixed planner scheduling.
./dougie radiation --world facility --headless --fast --duration 2400 \
  --dose-budget 5 --gp-capacity 256 --seed 3 --fixed-plan-delay .5 --require-clean --path-mode grid

# Inspect the articulated Go2 up close in RViz.
./dougie demo --world room --vx .2 --duration 30
```

The complete facility mission is long: the verified run took about 34 simulated
minutes / 22 wall minutes. Shorter missions may correctly return before reaching
the upper floor. Interactive scheduling changes trajectories and has not received
the same full-height qualification as the fixed-delay development run.

`--require-clean` is an **independent simulation-evaluation gate**, not a new robot
sensor. It aborts a trial when non-foot impact limits are exceeded and returns a
nonzero exit status for an incomplete or unclean return. It prevents a failed
trial from spending minutes continuing after a damaging contact. The autonomous
planner still receives only observations, ideal pose and detector measurements.

## A more realistic room

`dougie_sim/facility.py` builds a 19 × 16 m service building with:

- Two overlapping usable floors, separated by **2.4 m**, with 18 cm slabs.
- Equipment rooms, 1.8 m door openings, headers, columns, drums and cabinets.
- Two 8 m switchback service-ramp flights at **8.53°**, an intermediate 1.2 m
  landing, handrails/posts and a full-height upper access gallery.
- A genuine **16-step, 15 cm-riser staircase** as an unqualified challenge.
  The slope planner rejects this staircase; it is not a disguised ramp.
- Three spatially separated radiation sources and explicit slab/wall attenuation.
- `--layout-seed` for doorway/equipment variation, separate from detector noise.

The default source profile is `facility`, and the default terrain profile is
`layered`. There is no supplied navigation map or floor/connector coordinate list.
The RViz cutaway is explicitly diagnostic simulator geometry and is not an
autonomy input. Radiation remains a synthetic attenuation proxy.

## RViz uses the actual Go2 visual model

The previous default collision-marker display has been replaced by a
`rviz_default_plugins/RobotModel` using `/robot_description`:

- Original checksum-verified Go2 OBJ meshes and material colours.
- **17 links and 12 articulated joints**, with physical model axes and limits.
- Visual origins from the source MJCF, avoiding compiled-mesh recentering errors.
- Link transforms generated from the same joint coordinates as `/joint_states`.
- Transient-local description publication for late-starting viewers.
- Collision primitives remain an optional disabled debug display.

`robot_description.py` generates the description from the pinned assets rather
than loading an unrelated G1 or approximate robot. Tests verify mesh hashes and
compare URDF forward kinematics and published relative transforms with MuJoCo
over randomized joint/root poses. Real DDS tests verify late description delivery
and all limb transforms. An interactive room session loaded RViz without reported
URDF/mesh errors; the host still logged Mesa driver fallback messages.

The ordinary demo now uses a close following camera. The facility uses a wider
cutaway overview so the ground floor remains visible beneath upper slabs.

## Exploration improvements driven by failures

1. **Support cache capacity:** an early mission exhausted 80,000 support entries
   downstairs. Floating-point values just above/below zero duplicated floor voxels.
   Stable quantization fixes that duplication; capacity is explicitly bounded at
   160,000, with saturation reported rather than silently claiming completion.
2. **Incremental fitting:** unchanged regions reuse fitted candidates, while new
   observations invalidate neighbouring regions. Tests compare cached and fresh
   graphs after introducing an obstacle.
3. **Covered supports:** an observed ramp top and the floor beneath it must not be
   fit as one rough surface. Low covered supports are excluded using observed
   returns, while genuinely separated floors remain distinct.
4. **Inspection viewpoints:** the former 0.75 m candidate minimum could prevent
   short moves needed to see a ramp entrance. Layered exploration admits supported
   short inspection moves and uses a tighter follower tolerance.
5. **Footprint clearance:** a development run fell at the intermediate landing.
   Four neighbouring support cells were insufficient. Admission now checks a
   45 cm-radius body/swing-foot footprint, including diagonal support, while plane
   fitting stays local enough to recognize continuous ramp/landing junctions.
6. **Height-aware utility:** observed slopes and unsampled height bands contribute
   to candidate utility without bypassing geometry, dose or return admission.
7. **Bounded visibility work:** visibility rays are evaluated in bounded Boolean
   grids, with exact tests against the prior ray-by-ray computation and a sparse
   fallback for large coordinate extents. A cheaper graph-degree approximation
   missed useful inspection moves and was removed.

## Full-height physical result

Final fixed-delay run: `results/dougie/radiation_20261012T175641_497545Z`.
Baseline walking policy, layout 0, detector seed 3, 2,400 s limit, 5 µSv budget,
256 inducing points, independent clean-contact gate enabled:

| Measure | Result |
|---|---:|
| Upper-floor foot-pose witness | **759.33 s** |
| Subsequent lower-floor witness | **1,961.89 s** |
| Upright home return | **2,021.44 s** |
| Travelled path | **259.15 m** |
| Actual / measured exposure | **0.63646 / 0.63148 µSv** |
| Non-foot impulse / peak | **0 N·s / 0 N** |
| Detector height span | **0.391–2.856 m** |
| Observed surface proxy | **428.36 m²** |
| Real-time factor | **1.52×** |
| Main-loop p95 / p99 | **0.43 / 0.62 ms** |
| Planner p95 | **1.40 s** |

There were still 2,020 steps over the 5 ms budget, including synchronous scan and
fixed-schedule worker waits. Neither tail timing nor a single successful layout
establishes hard-real-time or general navigation reliability. The mission made
repeated visits; route efficiency remains an improvement target.

**This demonstrates a physical, observation-driven full-height ramp round trip.**
It does not establish ordinary stair capability, complete building coverage,
sensor-only localization or superior radiation avoidance versus matched baselines.
`exploration_complete` remains false rather than treating home return as proof
that every room was explored.

Earlier facility trials are retained, including downstairs-only returns, a
landing-edge fall, a partial climb, and a matched 900 s comparison in which both
baseline and a new perceptive policy returned cleanly but stayed downstairs.
The portable report includes those outcomes, not only the final successful run.

## Perceptive training and verification

The new `go2-proprio-lidar-622-v2` actor uses:

- Existing 45 proprioceptive/command values.
- **288 rays sampled from the existing scanner's beam pattern**, from its actual
  origin, including occlusion; no overhead height-map oracle.
- Normalized ranges, hit-validity bits and scan age.
- A 10 Hz sample-and-hold contract, with training noise and dropout.

Training and deployment share the beam layout and feature encoding. Tests check
ray-subset equality, CPU/Torch encoding, hold age/reset behavior and deployment
through the versioned policy adapter. Critics/rewards may use privileged data;
the exported actor does not.

```bash
./dougie training-setup
./dougie train --terrain small-stairs --perceptive --num-envs 128 \
  --iterations 500 --initialize-baseline --quiet

# Preserve the schema when resuming; exported actors remain unqualified.
./dougie train --terrain small-stairs --perceptive --num-envs 128 \
  --iterations 500 --resume results/dougie/TRAINING_RUN/checkpoint.pt --quiet

# Export a retained checkpoint without doing additional learning.
./dougie train --terrain stairs --perceptive --num-envs 16 \
  --export-only --resume results/dougie/TRAINING_RUN/model_1850.pt
```

### What was learned from the experiments

- The first 250-iteration perceptive candidate learned to stand still and failed
  all 14 screening trials. Progress tracking was strengthened and command episodes
  lengthened to exercise actual flights rather than short random motions.
- A 1,000-iteration run restored flat/ramp traversal, but stair qualification still
  failed. A larger continuation reached retained checkpoint 1850 before its wall
  timeout; that checkpoint was exported and independently tested, not discarded.
- Investigation found a concrete **reward-interface bug**: contact sensors expanded
  in FL/FR/RL/RR order, while foot-height frames used FR/FL/RR/RL. Some swing-height
  rewards therefore used another foot's contact state. All per-foot reward tensors
  now align with the actual model order, independently of policy joint ordering.
  A startup assertion and regression test prevent silent recurrence.
- A fresh corrected 750-iteration run completed **4,608,000 transitions**. Another
  fresh 500-iteration small-stair run completed **1,536,000 transitions**, with an
  accumulated non-foot impulse gate and explicit commanded-stall cost. All exported
  candidates passed action parity in the native CPU runtime.

New training runs archive source snapshots, dependency locks, configuration,
checkpoint ancestry hashes and sensor ordering. Earlier trials remain identified
as pre-correction experiments; their successful export checks were not proof that
their rewards were correctly aligned.

### Native-MuJoCo screening, seeds 11 and 12

These seeds are development screening, not a sealed holdout set. Counts are strict
passes out of two, not merely crossings:

| Policy / experiment | Flat | 6 cm up | 6 cm down | 10 cm up/down | 10° ramps up/down |
|---|---:|---:|---:|---:|---:|
| Pinned baseline | 2 | 2 | 0 | 0 / 0 | 2 / 2 |
| Initial perceptive, 250 updates | 0 | 0 | 0 | 0 / 0 | 0 / 0 |
| Progress reward, 1,000 updates | 2 | 0 | 0 | 0 / 0 | 2 / 2 |
| Longer continuation, checkpoint 1850 | 2 | 0 | 0 | 0 / 0 | 2 / 2 |
| Corrected foot ordering, 750 updates | 2 | 0 | 0 | 0 / 0 | 2 / 2 |
| Corrected ordering + impulse gate, 500 updates | 2 | 0 | 0 | Not tested | 2 / 2 |

The final candidate also passed 2/2 3 cm descents and 1/2 3 cm ascents; the other
ascent violated a joint limit. Six-centimetre stair crossings still included
non-foot impacts, penetration or joint-limit excursions. **No candidate passed
the stair promotion gate, and no candidate has replaced the default policy.**

The current GPU/native collision-backend gap, foot-placement objectives and
independent held-out evaluation remain important work. More training updates
alone have not produced robust ordinary stairs or established an optimal policy.

## Evidence and next gates

Final regression verification: **108 tests passed** in 19.59 seconds, including
real DDS description/TF delivery, mesh hashes and kinematics, sparse-ray encoding,
per-foot ordering, contact-budget accumulation, support margins and strict mission
exit status. An eight-second interactive RViz walking smoke run also exited cleanly
(`results/dougie/20261012T183245_263297Z/`).

[Portable facility/training report](results/dougie_facility_training_20261012.json)
contains mission hashes/configurations, policy hashes, all screening summaries,
completed training outcomes and the interrupted continuation.

Next: qualify the full-height route across independent layouts/seeds; improve
frontier completion and route efficiency; calibrate GPU/native contact behavior;
train clean stair ascent/descent before admitting those connectors; and replace
ideal pose with the staged LiDAR/IMU/leg estimator. Radiation-blind and flattened-XY
comparators are still needed to attribute avoidance improvements specifically to
the XYZ radiation model.
