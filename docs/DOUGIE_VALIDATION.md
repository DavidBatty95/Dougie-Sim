# Dougie-Sim initial validation

[Project home](../README.md) · [Methods](DOUGIE_RADIATION.md) · [Architecture](DOUGIE_SIM.md)

Checkpoint: 7 October 2026. [Portable evidence](results/dougie_initial_20261007.json)
records source/asset hashes and the Docker image identity. Raw artifacts remain
under ignored `results/dougie/`. These are simulation qualification trials,
not a general reliability study, hardware deployment or SOTA comparison.

## Software and interfaces

**25 tests passed**, including the real-DDS radiation publication test.

Tests cover named Go2 joint/actuator mapping, policy observations, unpinned
standing, walking and bidirectional turning, scan walls/people, malformed
commands, command timeouts, known-free frontiers, radiation distance/shielding,
Poisson count integration, exact-versus-streaming Gaussian posterior agreement,
bounded GP state, unknown-height uncertainty, Poisson zero/count-rate behavior,
multi-height graph connectivity, stale-input rejection, dose integration and
low-budget return. A separate process test observes real ROS detector messages
and the 3-D posterior point cloud.

```bash
./dougie test
./dougie demo --headless --duration 8
python3 scripts/check_dougie_run.py
```

The ordinary ROS smoke test recorded odometry, joint states, IMU and scans.
Early planar exploration integration attempts encountered a launch boolean
expression and an rclpy reserved-name collision; both were corrected. Planar
frontier admission was slow in the first office trial. No completed planar
coverage claim is made. Radiation mode uses its separate 3-D graph planner.

## Closed-loop radiation missions

Frozen Poisson-GP source, one reactor layout, 180 s maximum mission time,
0.2 µSv budget, 100 µSv/h rate threshold, 128-point GP capacity. Seeds change
detector noise, not facility geometry. Trials ran concurrently on the host;
their planning latency includes contention.

| Seed | Return time (s) | Actual dose (µSv) | Budget exceeded | Observed surface-area proxy (m²) | Sampled free volume proxy (m³) |
|---|---:|---:|---|---:|---:|
| 1 | 123.20 | 0.08283 | No | 131.81 | 70.34 |
| 2 | 149.26 | 0.11124 | No | 124.94 | 66.27 |
| 3 | 135.23 | 0.10429 | No | 133.56 | 70.70 |

All three returned within the configured 0.4 m home tolerance and remained
upright. This is three detector-noise repeats of one scene, not statistical
proof of safety. Coverage is discretized observed geometry, not a claim to
have mapped every room or all accessible volume. Surface proxy counts can
include more than one height in a column. Free volume uses sampled ray cells.

Sensor heights in these runs were approximately **0.39–0.46 m**. Thus these runs
validate the 3-D data path and ground-level mission, not independent radiation
reconstruction throughout a multi-storey building. Earlier Gaussian-mode
development runs also returned within budget; they are not pooled with the
frozen Poisson cohort.

```bash
./dougie radiation --headless --fast --duration 180 --dose-budget 0.2 --seed 3
```

## GP accuracy and computational cost

Reproducible synthetic XYZ benchmark: 2,000 noisy training observations and
1,000 held-out query locations throughout the configured facility volume.
The same samples are used for both likelihoods and all capacities. This is
uniform experimental sampling, not the path actually flown/walked by the robot.
Single-thread BLAS timings are hardware-dependent and exclude sensing/navigation.

| Likelihood | Capacity | Update p95 (ms) | 1,000-point batch (ms) | Log1p-rate RMSE | Raw-rate RMSE (µSv/h) |
|---|---:|---:|---:|---:|---:|
| Gaussian approximation | 32 | 0.097 | 2.58 | 1.002 | 99.10 |
| Gaussian approximation | 64 | 0.053 | 5.21 | 0.620 | 48.28 |
| Gaussian approximation | 128 | 0.209 | 11.16 | 0.478 | 37.10 |
| Poisson/Laplace | 32 | 0.089 | 1.60 | 0.976 | 99.39 |
| Poisson/Laplace | 64 | 0.090 | 3.48 | 0.589 | 47.46 |
| Poisson/Laplace | 128 | 0.245 | 9.48 | 0.439 | 36.54 |

At 128 inducing points, posterior arrays occupy **266,240 bytes**, excluding
temporary query arrays and runtime libraries. Nominal 95% latent interval
coverage for the Poisson model was 99.8% on this dataset: conservative, not
perfectly calibrated. The raw-rate RMSE still shows substantial hotspot error;
fast inference does not imply an accurate peak-dose estimate everywhere.

```bash
./dougie benchmark-gp
```

## Physical terrain capability

- **Stairs:** all six ascent and six descent steps contacted. Risers 6 cm,
  tread 40 cm, platform 36 cm high. The untethered Go2 crossed 8 m in 119.4 s,
  reached a maximum base height of 0.673 m and ended at 0.306 m, upright.
- **Uneven plates:** all three 2–4 cm plates contacted. The robot traversed
  from x=-1 m to x=-3.8 m in 22.925 s without falling.

These tests use heading-guided velocity commands on known qualification courses,
not autonomous reactor-wide stair discovery. Full-height industrial staircases,
steep slopes, damaged stairs and multi-floor radiation missions remain unqualified.

```bash
./dougie terrain-test
docker compose run --rm -T dougie-sim python3 scripts/probe_dougie_terrain.py --terrain uneven
```

## Reproduce the portable report

After running explicit output directories for seeds 1–3 and saving the terrain
and GP benchmark outputs:

```bash
python3 scripts/publish_dougie_evidence.py --runs results/dougie/poisson_seed1 results/dougie/poisson_seed2 results/dougie/poisson_seed3 --output docs/results/dougie_initial_20261007.json
```

The report retains each run's source hashes instead of assuming future working
trees are identical. Fresh runs may differ under scheduling and library changes;
retain their failures and update claims from their measured outcomes.
