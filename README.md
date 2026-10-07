# Dougie-Sim

[![Go2 verification](https://github.com/DavidBatty95/Dougie-Sim/actions/workflows/verification.yml/badge.svg)](https://github.com/DavidBatty95/Dougie-Sim/actions/workflows/verification.yml)

**Unitree Go2 · MuJoCo · ROS 2 Humble · Dose-constrained radiation mapping**

![Dougie's reactor environment](docs/media/dougie-reactor.png)

**Mission:** map as much of an unknown facility as possible while managing
cumulative radiation exposure and retaining an estimated return budget.
Dougie combines physical quadruped walking, 3-D range sensing, a counting
radiation detector, online sparse Gaussian-process inference, and dose-aware
exploration through a reactor-themed environment.

Dougie is the quadruped sibling of [Geoff-Sim](https://github.com/DavidBatty95/Geoff-Sim).
It preserves Geoff's source history at `d1d54ff974e44546e260d3ccf6e3cb1f8f3a7568`
and adds an independent Go2 runtime: twelve torque-controlled joints, a pinned
pretrained walking policy, ROS interfaces, a three-dimensional observed-surface
planner and a radiation mission. A planar SLAM/Nav2 comparison profile is also available.

## Quick start

Linux with Docker Engine and Docker Compose v2:

```bash
git clone git@github.com:DavidBatty95/Dougie-Sim.git
cd Dougie-Sim
./dougie setup
./dougie radiation --dose-budget 2 --duration 600
```

Setup downloads checksum-verified public Go2 meshes and the Go2 `robot_lab`
policy from `fan-ziqi/rl_sar`, then builds `dougie-sim:humble`. Recursive private
submodule checkout is **not required** for Dougie's runtime. Assets are cached
under `.dougie/` and are not committed. First setup requires internet access.

### One-line demos

```bash
./dougie radiation --dose-budget 2 --rate-limit 100 --duration 600
./dougie radiation --headless --fast --dose-budget 0.2 --duration 180 --seed 3
./dougie demo --world flat --vx 0.3 --duration 30
./dougie demo --world room
./dougie explore --world office --duration 300
./dougie explore --world office --people 1 --duration 300
```

Radiation sources are MuJoCo sites, with shielding properties on scene walls.
The 2 Hz detector produces Poisson counts and dose estimates. The default GP
preserves the Poisson likelihood through streaming sparse Laplace inference;
`--gp-likelihood gaussian` selects a log-Gaussian comparison approximation.
`--gp-capacity 128` bounds the posterior dictionary.

The MuJoCo viewer opens by default. Colored spheres in radiation mode show the
**learned GP mean**, not the hidden source field. Ordinary exploration also opens RViz. Add `--headless`
to run without either GUI. `./dougie worlds` lists scenes. Ctrl+C stops the
session and its child processes. Results are written under `results/dougie/`.

Budgets are in **µSv**, rate limits in **µSv/h**: synthetic dose-equivalent
proxies, not an electronics damage calibration. Radiation mode owns motion and
does not accept external `/cmd_vel`. The zero-command ordinary demo balances in place until it receives `/cmd_vel`. From a
second terminal, enter `./dougie shell`, then:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Stopping publication triggers the 0.5-second command watchdog. Use one command
source at a time; the exploration session owns navigation commands. A basic
scan-based forward stop assists the demo; it is not Geoff's human-aware planner.

On X11, pass your existing `DISPLAY` and `XAUTHORITY`. If the local X server
requires explicit permission for the container user, `xhost +si:localuser:$(id -un)`
grants that user access. Headless runs need no display server or GPU.

## Runtime and evidence boundaries

- Physics runs at 200 Hz; the policy at 50 Hz, using its documented 45-value
  observation and 12-action contract. MuJoCo integrates the free-floating base;
  neither base teleportation nor a stabilizing tether is used during execution.
- ROS publishes `/clock`, `/odom`, `/imu`, `/joint_states`, `/scan`, and `/tf`.
  **Odometry is simulator ground truth**, and the scan is an ideal horizontal
  raycast. This is a simulation integration baseline, not sensor-only autonomy
  or a reproduction of the Go2 L1 LiDAR.
- Radiation mode builds a multi-height support graph from ideal 3-D range
  returns. It checks observed footprint support and rejects steps over 7.5 cm.
  GP queries are XYZ; unobserved heights retain uncertainty.
- Ordinary exploration uses observed SLAM occupancy, inflated known-free connectivity,
  bounded retries, and Nav2 action results. Moving people are simplified
  kinematic capsules observed in scans. Crowd-safe performance is not established.
- The standard Go2 has no arms: G1 pickup, hand control, and placement are
  inherited research references, not Go2 capabilities.
- Physical trials traversed **6 cm stair risers**, a **36 cm inspection deck**,
  and **2–4 cm floor plates**. Full-height industrial stairs and complete
  multi-storey autonomous coverage are not yet qualified.
- Dose planning uses upper GP estimates, route exposure, cumulative counts,
  return routes and reserves. It is a heuristic, not a guarantee of globally
  optimal coverage or probabilistic dose safety.
- Existing Geoff results and the literature PDF describe the humanoid. They
  are retained with their original identity, not relabeled as Dougie evidence.

See [architecture and migration](docs/DOUGIE_SIM.md),
[radiation methods and research](docs/DOUGIE_RADIATION.md),
[validation](docs/DOUGIE_VALIDATION.md), and [asset provenance](config/dougie_assets.json).

### Mission outputs

`results/dougie/radiation_*/` contains measured samples, decision traces,
source/configuration hashes, the predicted 3-D field, serialized GP posterior,
observed geometry, and independent actual-exposure scoring. The planner never
receives source locations or true field values.

ROS adds `/radiation/measurement` and `/radiation/status` (JSON `std_msgs/String`),
and `/radiation/gp_map` (`sensor_msgs/PointCloud2`, with mean, upper rate and
log-rate uncertainty). ROS is an optional publication interface for radiation
mode; headless accelerated trials can run without DDS.

## Tests

```bash
./dougie test
./dougie terrain-test
./dougie benchmark-gp
```

The default CI builds the Go2 image and tests policy/model ordering, physical
standing/walking/turning, scans, watchdogs, frontier planning, count statistics,
GP posteriors, dose-return behavior, stairs, uneven ground and ROS interfaces.
The Go2 profile does not build inherited G1 ROS packages.

## Repository layout

| Path | Role |
|---|---|
| `dougie`, `scripts/dougie_cli.py` | Go2 command-line entry point |
| `dougie_sim/` | Physics, policy adapter, ROS bridge, exploration, lifecycle |
| `config/dougie_*` | Go2 configuration and verified upstream assets |
| `docker/Dockerfile.dougie` | Reproducible Humble/Python 3.10 container |
| `tests/dougie/` | Go2-specific validation |
| `README-GEOFF.md`, `ros2_ws/`, `vendor/`, existing `docs/` | Inherited Geoff source and research history |

Upstream notices are retained; see [licensing inventory](LICENSES.md) and
[Dougie attribution](docs/DOUGIE_SIM.md#attribution). No blanket relicensing of
the inherited repository is implied.
