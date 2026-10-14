# RViz visualisation and simulation performance

## Run

```bash
./dougie radiation --show-sources --dose-budget 2 --rate-limit 100 --duration 600
```

RViz is now the default interactive renderer. MuJoCo advances physics without
creating a viewer or OpenGL context. `demo` and `explore` also use RViz through
their session launcher. Radiation runs retain `--visualizer mujoco` for diagnostic
comparisons; `--headless` suppresses either window. `--headless --fast` continues
to run without ROS or visualisation.

RViz displays:

- Static world primitives on `/dougie/scene`, published once with transient-local
  durability so a late-starting RViz receives them. These are diagnostic simulator
  truth, not geometry supplied to the planner.
- The verified Go2 visual meshes via RViz RobotModel, a retained
  `/robot_description`, and articulated link transforms at 50 Hz. Optional
  collision-debug primitives on `/dougie/robot` remain bounded to 20 Hz and are
  built only when subscribed. The dated 9 October renderer measurements below
  used the earlier collision-primitive display, not the new mesh display.
- GP samples, planned path, travelled path and mission state on
  `/radiation/visualization`, updated when a plan arrives (normally about 0.5 Hz).
  The underlying numeric posterior remains on `/radiation/gp_map`.
- Optional true hotspot/leak markers with `--show-sources`.
- A lightweight `/radiation/dashboard` display with measured dose/budget,
  measured rate/threshold, estimated return reserve, observed surface proxy,
  simulation time and decision reason. Detector updates refresh it at 2 Hz;
  plan changes and completion also refresh it, without resending GP markers.
- A green exploration target and planned route, blue travelled path, and amber
  estimated return route. The legend identifies the selected GP colour scale.
  End-of-run inspection distinguishes return, interruption, error and time limit.

Use RViz's mouse controls for the camera. `--map-layer uncertainty` selects GP
uncertainty colouring; the default is mean dose rate. `--camera` only controls the
legacy MuJoCo viewer/offscreen recording. When a radiation mission finishes,
RViz stays open for inspection; use `--exit-on-complete` to close automatically.

The RViz configuration requests **30 rendering FPS**. Physics stays at **200 Hz**,
policy inference and robot TF at **50 Hz**, and optional debug markers at **20 Hz**. These are distinct
rates. Actual viewport presentation FPS is measured in the graphics section below;
the older 9 October measurements describe physics throughput.

## GPU graphics fix and measured RViz FPS — 13 October 2026

The normal container launch was using **Mesa llvmpipe**, a CPU software renderer,
even on the RTX 4080 laptop. The GPU Compose override existed but was not selected
by `./dougie`; hybrid-laptop GLX selection also required NVIDIA PRIME offload.

Interactive commands now automatically select NVIDIA graphics when both a GPU
and the NVIDIA Docker runtime are available. The GPU override exposes graphics
and display driver capabilities and selects the NVIDIA GLX vendor. Intel/AMD
systems can use `/dev/dri` with the host's numeric render/video permission groups.
Headless `auto` execution does not probe or request graphics devices.

```bash
# Actual renderer, not merely whether nvidia-smi can see a device:
./dougie graphics-check --require-hardware

# Normal interactive launch now selects acceleration automatically:
./dougie radiation --world facility --duration 2400 --dose-budget 5 --gp-capacity 256 --seed 3

# Explicit selection when needed:
./dougie demo --graphics nvidia --world room --vx .2
./dougie graphics-check --graphics dri --require-hardware
```

`--graphics` accepts `auto`, `nvidia`, `dri`, `software`, or `system`; the default
can also be set with the host environment variable `DOUGIE_GRAPHICS`. `system`
uses the plain container configuration, while `software` intentionally forces
Mesa CPU rendering for comparison. NVIDIA mode needs NVIDIA Container Toolkit
configured for Docker. It worked with this host's existing installation.

**Restart the session to apply the fix:** devices and driver libraries are selected
when the container starts. Setting the flag inside an already-running software
renderer does not retrofit GPU access.

Each new RViz session records its OpenGL vendor, renderer and version in
`rviz_graphics.json` alongside `rviz.log`. The probe runs in the RViz child process
before launching RViz, never in the physics process.

### Actual frame-delivery measurement

Same facility cutaway configuration and full Go2 mesh URDF, 450 synthetic GP
points and a 2,000-point trail. Eight-second warmup followed by a 20-second wall
measurement. Both runs verified subscriptions to the robot description, static
scene and mission markers. Main viewport: **613,872 pixels**.

| Renderer | Presented FPS | Frame interval p50 | Frame interval p95 |
|---|---:|---:|---:|
| llvmpipe CPU software rendering | **13.10** | 75.61 ms | 84.20 ms |
| NVIDIA RTX 4080 Laptop GPU | **31.25** | 31.98 ms | 34.02 ms |

The NVIDIA result is approximately the requested 30 FPS. The reported 3 FPS user
view was not reproduced pixel-for-pixel; viewport size, camera and host load vary.
The matched benchmark demonstrates accelerated rendering with the complete model,
not a reduction to collision primitives or an exploration-quality result.

The optional benchmark counts `glXSwapBuffers` presentations, selecting the largest
drawable by pixel area to exclude RViz's tiny auxiliary context. It does not
mistake a ROS topic rate, physics throughput or configured FPS for actual rendering.
The measurement library is loaded only into the benchmark's RViz child.

```bash
./dougie benchmark-rviz --graphics software --world facility
./dougie benchmark-rviz --world facility
```

Raw frame timestamps, renderer identity and results are retained in
`results/dougie/rviz_benchmark_*/`. The measured pair is
`rviz_benchmark_20261013T075417_790509Z` (software) and
`rviz_benchmark_20261013T075318_831610Z` (NVIDIA).
[Portable rendering report](results/dougie_rviz_graphics_20261013.json).
The DRI diagnostic also verified hardware rendering with this host's integrated
Intel RPL-P GPU; its RViz frame rate was not benchmarked.

Verification: **114 tests passed**. Normal walking and facility launch smoke tests
both exited cleanly and recorded the NVIDIA renderer in `rviz_graphics.json`.
The facility RViz log reported OpenGL 4.6 without the earlier Mesa/iris fallback
errors. These short launch tests verify graphics setup, not mission performance.

## Local measurements — 9 October 2026

Reactor scene, clustered planner, conservative terrain profile, seed 3,
hotspots-and-leaks, 2 µSv budget, 120-second simulated mission limit:

| Mode | Simulated / loop wall seconds | Physics steps/wall second | Active loop p95 | MuJoCo viewer cost |
|---|---:|---:|---:|---:|
| MuJoCo viewer, interrupted | 20.96 / 177.76 | 23.58 | 174.47 ms | 172.10 s total; 180.28 ms p95/update |
| RViz, source markers, isolated ROS domain, subscriptions verified | 78.80 / 78.80 | 199.99 | 3.12 ms | Zero calls |

The verified RViz run returned cleanly, with no non-foot collision impulse recorded. The
MuJoCo-viewer comparison reached the wall-clock timeout and was interrupted;
it is not a completed-mission comparison. Approximately **8.5× greater physics
throughput** was observed locally. The two renderers logged Mesa/DRI fallback
errors on this host, so the size of the improvement is machine/driver dependent.
Asynchronous planner completion affects trajectories; these are operational
performance observations, not matched-trajectory planner-quality experiments.

Initial cProfile headless-ROS baseline: 82.74 simulated seconds, 82.97 wall
seconds including finalisation. ROS callback polling consumed 4.59 seconds over
16,548 calls. Polling now runs at the 50 Hz control/publication cadence instead
of every physics step. The follow-up profile recorded 4,240 calls and 1.85 seconds
for 84.79 simulated seconds. The radiation mission also stops computing the
unused 10 Hz horizontal scan; its 3-D sensing remains active. Navigation sessions
retain the horizontal scan and command watchdog.

The verified RViz run had 305/15,760 active steps exceed the 5 ms physics
budget (1.94%); p99 was 7.80 ms, and 3-D LiDAR p95 was 4.78 ms. Average real-time
operation is established here, not hard real-time deadline guarantees. Planner
p95 was 175 ms in its worker. Further latency work should target these remaining
spikes rather than increasing the physics timestep.

## Evidence and reproduction

Local artifacts under `results/dougie/`:

- `before_rviz_ros.prof`: original headless ROS profile.
- `after_rviz_ros.prof`: optimised headless ROS profile.
- `radiation_20261009T073234_472168Z/`: interrupted MuJoCo viewer comparison.
- `radiation_20261009T074232_304805Z/`: verified RViz run. A separate ROS observer
  confirmed RViz subscriptions to all three marker topics before the benchmark
  completed. Earlier RViz launch trials used an incorrect `Marker Topic` config
  key and are excluded from the visualisation results; the Humble MarkerArray
  display uses `Topic`.

Each new `radiation_summary.json` includes a `performance` section with real-time
factor, physics steps/second, active-loop p50/p95/p99, over-budget step counts,
ROS/marker costs, LiDAR costs and MuJoCo viewer costs. Active-loop timing excludes
intentional pacing sleeps. It includes main-thread work/waits, not worker CPU
time or RViz rendering time. Loop wall timing excludes final archive generation
and post-mission inspection. Viewer startup handling differs between modes, so
these are sustained-run rather than startup benchmarks.

```bash
# RViz, normal interactive timing
./dougie radiation --duration 120 --seed 3 --exit-on-complete

# Diagnostic legacy comparison (can be substantially slower than real time)
./dougie radiation --duration 120 --seed 3 --exit-on-complete --visualizer mujoco

# No window, ROS enabled
./dougie radiation --duration 120 --seed 3 --headless

# Regression checks, including real DDS delivery to a late scene subscriber
./dougie test
```

Verification: **68 tests passed**. The DDS integration test checks latched scene
delivery, articulated robot updates bounded to 20 Hz, GP marker data, fixed frame,
and absence of MuJoCo viewer calls. A three-second room walking session also
completed with RViz and no simulation error.

## Sensor hot-path update — 12 October 2026

The static radiation field now precomputes source/shield pairs and evaluates
segment/box intersections in NumPy batches. The horizontal scanner uses MuJoCo's
batched ray API instead of 180 Python-to-native calls. Source strengths, shielding,
scan angles, range filtering, physics timestep and detector integration are unchanged.
GP visualisation is now strictly bounded to 150 support locations / 450 XYZ samples.

Local reactor microbenchmark, default **six-source hotspots-and-leaks** profile,
three shields, 1,000 timed calls after 20 warm-up calls per implementation:

| Operation | Scalar median / p95 | Batched median / p95 | Median speedup |
|---|---:|---:|---:|
| Radiation rate at one detector position | 102.47 / 110.65 µs | 19.70 / 21.75 µs | **5.20×** |
| 180-ray horizontal scan | 300.22 / 352.00 µs | 45.36 / 49.04 µs | **6.62×** |

These are same-process sensor comparisons at the initial reactor pose, not
whole-mission or rendering speedups. Radiation missions use the 3-D scanner;
the horizontal scan improvement benefits ordinary walking/navigation sessions.

A complete headless fixed-planner-delay smoke mission returned upright with
**0.24044 µSv actual dose against a 2 µSv budget**, zero recorded non-foot contact
impulse, 8.82 m travelled, and 111.69 m² observed surface proxy. It ran 71.07
simulated seconds in 9.99 loop wall seconds (**7.12× real time**); active-loop p95
was 0.81 ms. Its p99 was 10.93 ms and 358 steps exceeded 5 ms: fixed-delay fast
mode can wait for planner work, so this is throughput evidence rather than a
deadline guarantee. No graphical viewer was exercised in this smoke run.
Local artifacts: `results/dougie/radiation_20261012T121147_531013Z/`.

Reproduce in the existing container:

```bash
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_sensors.py
docker compose run --rm -T dougie-sim python3 -m dougie_sim.radiation_run \
  --headless --fast --duration 120 --seed 3 --dose-budget 2 --fixed-plan-delay .5
docker compose run --rm -T dougie-sim python3 -m pytest tests/dougie -q
```

Verification: **76 tests passed**, including scalar/batched equivalence across
three worlds and varied ray headings/ranges; radiation tests include empty
fields, parallel rays, shield boundaries and source-coincident positions.
Real-DDS tests exercise the dashboard, route/target messages and map point cap.
