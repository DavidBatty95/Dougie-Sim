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
- Articulated Go2 physical primitives on `/dougie/robot`, at at most 20 Hz in
  simulation time, and only when subscribed. These represent collision geometry,
  not the high-resolution visual meshes.
- GP samples, planned path, travelled path and mission state on
  `/radiation/visualization`, updated when a plan arrives (normally about 0.5 Hz).
  The underlying numeric posterior remains on `/radiation/gp_map`.
- Optional true hotspot/leak markers with `--show-sources`.

Use RViz's mouse controls for the camera. `--map-layer uncertainty` selects GP
uncertainty colouring; the default is mean dose rate. `--camera` only controls the
legacy MuJoCo viewer/offscreen recording. When a radiation mission finishes,
RViz stays open for inspection; use `--exit-on-complete` to close automatically.

The RViz configuration requests **30 rendering FPS**. Physics stays at **200 Hz**,
policy inference at **50 Hz**, and robot markers at **20 Hz**. These are distinct
rates: physics throughput is measured below; actual RViz display FPS was not
instrumented.

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
