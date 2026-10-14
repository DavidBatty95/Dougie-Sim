# Smooth routes, spatial GP scaling, incline training and radiation highlights

Development checkpoint: 14 October 2026. These changes build on the
[facility milestone](DOUGIE_FACILITY.md). All comparisons below are development
evidence, not a general safety or reliability guarantee.

## 14 October refinement results

**Later full-height update:** two corrected smooth/spatial facility runs completed
upstairs/downstairs/home with no non-foot impacts. The visited-connector reward,
explicit round-trip gate and optional occlusion-aware discovery targets are
documented in [full-height results](DOUGIE_FULLHEIGHT_RESULTS.md). The earlier
tables and qualification gaps below describe their original development cohorts.

The [new portable report](results/dougie_refinements_20261014.json) retains the
following results, model budgets, source hashes and failed policy screenings.
The 13 October tables below remain historical comparisons.

### Replan stability and turn preview

The follower now joins a delayed route within its first **1 m of arc length**,
including later checked spans, rather than always restarting in the first span.
The join must be within **12 cm in XYZ**; rejected joins stop until another route
is supplied. The search window also bounds projections on long segments and
excludes segments beginning outside that window. Span endpoints retain the 6 cm
arrival gate. Curvature preview slows the robot before upcoming rounded bends.

All six physical follower trials reached their goals with zero recorded non-foot
impacts. The obstacle smooth trial took **22.235 s**, travelled **3.405 m**, and
had yaw-command variation **5.947**, down from **7.220** in the historical smooth
trial. The zigzag result remained 16.370 s / 2.774 m. Resubmitting each checked
route every 0.5 s with an approximately 10 cm stale prefix produced the same
physical trajectories: **32 / 44 replans, zero rejections** for zigzag / obstacle.
This controlled resubmission test is not a changing-obstacle latency evaluation.

The combined smooth/spatial mezzanine mission again witnessed ascent and descent
and returned cleanly: **261.545 s**, **28.091 m**, **0.15818 µSv**, zero recorded
non-foot impacts. It used the baseline policy, detector seed 3 and fixed 0.5 s
simulated planning delay. Fixed-delay scheduling and concurrent test workloads
make this a physical regression check, not a real-time latency guarantee.

### Recorded-facility GP replay

`benchmark_dougie_gp_replay.py` fits the first **3,233** recorded count measurements
from the successful full-height facility run and scores the final **809** positions
without assimilating their measurements. Evaluator truth is used only for scoring.
These positions include spatially correlated revisits, not a new-building holdout.

Prepared prediction caches the posterior's mean coefficients and variance
correction matrix until the next update, and bounds query temporaries to 2,048
points per chunk. Both Gaussian and Poisson models retain an uncached reference
path. Spatial queries additionally group expert assignments with lexicographic
sorting and reuse constant halo masks.

| Model | Reference / cached median batch | Cached cold batch | log1p-rate RMSE | Upper-rate coverage |
|---|---:|---:|---:|---:|
| Global-256 | 24.37 / **16.68 ms** | 20.17 ms | 0.601 | 99.63% |
| Global-512 (313 admitted points) | 30.16 / **22.06 ms** | 28.64 ms | **0.402** | 99.38% |
| Spatial-32×128 (40 experts) | 7.33 / **6.07 ms** | 7.21 ms | 0.469 | 99.26% |

Maximum absolute prediction differences between cached and reference paths were
below **1.4e-12**. These comparisons isolate prediction preparation on the current
grouping implementation. They do not measure the grouping change separately.
Prediction caches used an additional **526 / 786 / 121 kB**, respectively, beyond
posterior arrays. Cold measurements include cache construction. Frequent updates
and smaller query batches can amortize that cost differently. Spatial mode is
fastest here, while global-512 is most accurate; empirical upper-rate coverage is
not a simultaneous trajectory guarantee.

```bash
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_gp_replay.py \
  results/dougie/radiation_20261012T175641_497545Z
```

### Bounded-residual incline experiment

`--residual-scale .15` trains a zero-initialized correction around the frozen
baseline actor: `baseline(proprio) + .15*tanh(correction(observation))`. The
deterministic per-joint correction is bounded in policy action units; PPO's sampled
exploration noise and subsequent closed-loop motion are not bounded by this value.
The actor uses the same coherent delayed proprio/LiDAR vector. Resume/export must
preserve the residual scale and observation schema.

```bash
./dougie train --terrain inclines --perceptive --num-envs 128 --iterations 150 \
  --initialize-baseline --residual-scale .15 --seed 44 --quiet
```

One 460,800-transition GPU run and its earlier `model_50.pt` checkpoint passed
native CPU export parity. Development screening used the same nine cases and
seeds 21–23 as the historical baseline:

| Candidate | Strict passes / 27 | Failures |
|---|---:|---|
| Historical baseline | **27 / 27** | None |
| Residual final checkpoint | 25 / 27 | Two 15° downhill stalls |
| Residual `model_50.pt` | 26 / 27 | One delayed-downhill corridor exit |

Both candidates failed the baseline-preservation gate and were rejected before
fresh-seed evaluation. **Baseline remains the deployment default.** Smaller
corrections reduced regression relative to earlier candidates, but superiority
and repeatability remain unestablished. Further tuning must remain on development
seeds before freezing a candidate for independent evaluation.

Final regression suite: **153 tests passed**, including prepared-posterior
equivalence after updates/chunking, bounded route joins and scripted residual
gradient/export checks. Full-height smooth/spatial repeatability, industrial
stairs and sensor-only localization remain open qualification work.

## Run

```bash
# Smoother execution is now the default. Source beacons remain diagnostic/opt-in.
./dougie radiation --world facility --duration 2400 --dose-budget 5 \
  --gp-capacity 256 --show-sources

# Optional local GP experts for larger maps:
./dougie radiation --world facility --duration 2400 --dose-budget 5 \
  --gp-mode spatial --gp-expert-capacity 32 --gp-max-experts 128 \
  --gp-cell-size 4 4 1.5 --show-sources

# Compare the original grid follower or inspect the complete unfiltered posterior:
./dougie radiation --world mezzanine --path-mode grid
./dougie radiation --world mezzanine --field-display all --map-layer uncertainty
```

`--gp-capacity` configures the global GP. Spatial mode has a separate, explicit
per-expert capacity and maximum expert count. The global GP remains the default;
spatial mode trades local-model overhead and memory allocation for scalable
prediction and better spatial coverage.

## Path smoothing is used for movement, not just drawing

The planner still selects goals and admits return/dose costs on the observed
support graph. `path_motion.py` then:

1. Removes grid staircasing with bounded line-of-sight shortcuts.
2. Checks sampled clearance, interpolated support, height consistency and graph
   connectivity, including intermediate cells at diagonal corners and blocked edges.
3. Rounds suitable corners with quadratic curves only after checking their convex
   region, so the follower's lookahead chords stay within that checked region.
4. Re-evaluates predicted upper-rate exposure; a risk-increasing route falls back
   to the original grid route.
5. Tracks arc length with bounded lookahead, curvature-aware speed and velocity/yaw
   command slew limits. Lookahead is restricted at unrounded corners. Sensor/plan
   freshness and high-rate stops still zero commands immediately.

These are discretized checks against observed geometry and a GP estimate, not a
continuous collision proof or calibrated exposure guarantee. Narrow/unknown
corners and hazardous shortcuts may intentionally remain angular. The grid route,
execution route, smoothing decisions and timings are retained in `trace.jsonl`.
`worker_p95_ms` includes observation handling, planning, smoothing and map queries;
the older `planning_p95_ms` covers the planner's replan function only.

### Matched physical follower tests

Same observed route, starting state, walking policy and goal tolerance; one repeat
per mode. All four runs reached the goal with zero recorded non-foot impacts.

| Scenario | Mode | Actual distance | Simulated time | Yaw-command variation |
|---|---|---:|---:|---:|
| Zigzag grid route | Grid | 3.084 m | 16.71 s | 15.258 |
| Zigzag grid route | Smooth | **2.774 m** | **16.37 s** | **4.646** |
| Route around a low obstacle | Grid | 3.997 m | 24.69 s | 8.224 |
| Route around a low obstacle | Smooth | **3.414 m** | **22.28 s** | **7.220** |

The zigzag's actual path was about 10% shorter with 70% lower yaw-command variation.
The obstacle route was about 15% shorter and 10% faster. Yaw variation is the sum
of absolute yaw-rate command changes, not an energy measurement.

### Closed-loop mezzanine comparison

Detector seed 3, 300 s limit, 2 µSv budget, fixed 0.5 s simulated planner delay:

| Execution / GP | Home return | Travel | Turning in place | Actual dose | Observed area proxy |
|---|---:|---:|---:|---:|---:|
| Grid / global | 255.53 s | 29.55 m | 43.63 s | 0.14604 µSv | 98.84 m² |
| Smooth / global | **249.51 s** | **28.02 m** | **29.88 s** | 0.14576 µSv | 99.28 m² |
| Smooth / spatial | 261.54 s | 28.09 m | 32.76 s | 0.15818 µSv | 98.16 m² |

All three witnessed the upper landing and subsequent descent, returned upright,
and recorded zero non-foot impacts. These are matched configurations, not identical
trajectories: motion changes affect later sensing and goal selection. Spatial GP
did not improve mission time/exposure in this small scene; its benefit is scaling.
The earlier full-height facility result used the grid follower and is a separate
cohort; the new combined modes have not received full-height repeatability testing.

## Spatial GP architecture and benchmark

`spatial_gp.py` assigns observations to bounded overlapping XYZ cells. Each expert
has its own sparse posterior. Nearby cells receive halo observations, and queries
are batched by expert. Cells are computational regions, not supplied room maps or
known floor identities.

- Display means and latent variances use mixture moments, including disagreement.
- Correlated overlapping experts are **not** multiplied as independent likelihoods.
- Planning uses an envelope at least as high as every active expert's upper rate;
  a locally predicted hotspot is not averaged away. This envelope can have steps
  where experts enter/leave support, even though blended means are continuous.
- Missing regions retain prior uncertainty. Queries never allocate models.
- Expert count and local dictionaries are bounded. Exhaustion is reported and
  causes mission return reasoning rather than silent eviction of return-route data.
- Global and spatial numeric archives can be reloaded with `spatial_gp.load_gp`.

Matched 3,000 synthetic count observations, 10,000 held-out queries, 24 × 24 m
domain and three sensor heights; single-threaded BLAS and five timed query batches:

| Model | Median query batch | Posterior arrays | log1p-rate RMSE | Upper-rate coverage |
|---|---:|---:|---:|---:|
| Global, 256 inducing points | 604.8 ms | 1.06 MB | 0.544 | 99.79% |
| Global, 512 inducing points | 3,051.5 ms | 4.21 MB | 0.380 | 99.87% |
| Spatial, up to 32 points × 256 experts | **392.3 ms** | **2.47 MB** | **0.159** | 99.96% |

Spatial prediction was about 1.5× faster than global-256 and 7.8× faster than
global-512 here. It used 252 experts / 5,089 total inducing entries, including halo
duplication; these are different model budgets, not a same-inducing-count claim.
Halo updates were slower than global-256 updates. Memory figures count posterior
NumPy arrays, not whole-process RSS. Empirical coverage is not a calibrated joint
safety guarantee, and the local model is not the exact global GP posterior.

```bash
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_motion.py
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_spatial_gp.py
```

## Above-background visualization

The default `--field-display detected` no longer paints the whole GP prior volume.

- Detector counts accumulate in bounded **0.4 m XYZ cells**.
- A cell is highlighted when its one-sided 99% Poisson lower rate limit exceeds
  the background calibration. Redrawing does not double-count observations.
- Bright warm cubes mark that count evidence. Larger yellow-to-red spheres show
  above-background GP estimates close to those detections: at most 0.8 m
  horizontally / 0.45 m vertically in an ellipsoidal neighbourhood.
- No unmeasured distant room or different floor is coloured solely from a high
  GP prior. Raw posterior arrays and `/radiation/gp_map` remain available for analysis.
- `--display-background RATE` overrides the operator calibration; otherwise the
  synthetic scenario's configured background is used. It is not a planner input.

The significance limit is pointwise; repeated looks and multiple cells are not a
simultaneous confidence claim. Spheres are interpolated predictions, not detector
samples. **Blank space is not evidence of safety.** `--field-display all` restores
the full diagnostic posterior, including uncertainty in unmeasured regions.

With `--show-sources`, true-source markers now have larger beacons, direction
arrows and hotspot/leak labels. An elevated annotation is placed above equipment
enclosing an emitter, with an arrow and label indicating its actual position and
height. These are clearly marked **TRUE SOURCE / diagnostic**; they are never
supplied to autonomy.

## Incline-focused training

The new `--terrain inclines` curriculum includes flat patches and ascending and
inverted sloped terrain, with nominal axis gradients from tan(2°) to tan(14°).
Pyramid cross-slopes can be steeper than a single axis; this range is a training
parameter, not a qualified maximum slope.

Training now includes terrain-relative posture with relaxed pitch preference,
stronger slip cost, zero-command drift cost, and velocity tracking that does not
penalize legitimate vertical motion. The final version delays the **whole actor
vector coherently** by 0–40 ms, rather than mixing positions, velocities and prior
actions from inconsistent timestamps. Critics may use privileged data; actors
still use the deployed proprioception / occluded LiDAR contract.

```bash
./dougie train --terrain inclines --perceptive --num-envs 128 \
  --iterations 500 --initialize-baseline --quiet
```

Three fresh GPU runs were trained and CPU-export parity checked. Independent
native-MuJoCo screening used seeds 21–23 across nine cases: flat, 8 m 10° up/down,
6 m stop/restart up/down, 20 ms delayed up/down, and 15° up/down.

| Policy | Strict passes / 27 |
|---|---:|
| Established baseline | **27 / 27** |
| First incline-specific candidate | 19 / 27 |
| Independent per-term delay experiment | 0 / 27 |
| Corrected coherent-delay candidate | 23 / 27 |

The corrected candidate passed the long 10° ramps and 15° ascent, but still had
two delayed-ascent corridor failures and two stalls. **It has not outperformed
the baseline, so no experimental policy replaces the default.** The curriculum,
latency handling and qualification coverage are improved; improved deployment
reliability of a new policy is not established. These three repeats per case are
screening, not the 60-repeat qualification gate.

## Evidence

Verification: **141 tests passed**, including geometry/risk rejection, expert
overlap uncertainty, both posterior archive formats, coherent actor ordering,
background-only suppression, and real DDS delivery of detections/source labels.
A paced reactor display smoke run completed without simulation error or recorded
non-foot impacts. The updated, larger GP glyph workload still measured **31.25 FPS**
on NVIDIA in `rviz_benchmark_20261013T163929_176288Z` (synthetic rendering benchmark).

[Portable combined report](results/dougie_motion_mapping_20261013.json) retains
the matched motion/GP benchmarks, three closed-loop missions, all three training
experiments and their native screening results, including failures.

The rendering changes are operator-only. Path smoothing is checked against the
same inferred field used for admission, and global GP remains the default until
larger matched closed-loop comparisons justify changing it.
