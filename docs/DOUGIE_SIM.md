# Dougie-Sim architecture and migration

[Project home](../README.md) · [Radiation methods](DOUGIE_RADIATION.md) · [Validation](DOUGIE_VALIDATION.md)

## Independent quadruped profile

Dougie starts from Geoff-Sim commit `d1d54ff974e44546e260d3ccf6e3cb1f8f3a7568`.
Git history, package notices, research and prior results are retained.
The active entry point is `dougie`; `install.sh` delegates to `dougie setup`.
The default Compose service/image is `dougie-sim` / `dougie-sim:humble`.
Loopback-only DDS defaults to domain **43**, separate from Geoff's 42.

Go2 assets are downloaded from immutable public commits and checked against
SHA-256 values in `config/dougie_assets.json`. Private Geoff submodules are not
initialized by Dougie setup or CI. No local `.env`, private training checkpoint
or uncommitted Geoff experiment was copied.

## Data flow

```text
MuJoCo free-body Go2 + reactor structures + hidden radiation source sites
    ├─ ideal pose, joint state, IMU and 3-D range returns
    ├─ field transport proxy → Poisson detector → count / exposure observations
    └─ independent exposure evaluator (not a planner input)
                  ↓
observed multi-height surface graph + online XYZ radiation GP
                  ↓
coverage / uncertainty / route-dose scoring + return-dose and time reserves
                  ↓
known-support route follower → Go2 walking policy → joint PD torques → MuJoCo
```

The policy runs at 50 Hz and torque-driven physics at 200 Hz. There are no root
position actuators, post-start pose resets, fixed-base welds or stabilization
tethers. Joint names explicitly map policy order to MuJoCo addresses.
The policy can drift at zero command; route following supplies heading feedback.

Radiation planning runs in one background worker with detached scan/sample
inputs. The main loop consumes returned paths, checks sensor/plan freshness and
continues physics during planning. BLAS threads are bounded. Accelerated mode
waits at bounded simulated intervals for its worker; its speed is not a
real-time guarantee.

## Navigation profiles

- **Radiation:** observed 3-D support graph, GP risk, dose-aware route selection
  and return behavior. Independent of planar Nav2.
- **Ordinary exploration:** horizontal scans → SLAM Toolbox → Nav2 → frontier
  goals. Initial planar runs exposed slow frontier admission. Startup success
  is not completed coverage or evidence of 3-D radiation exploration.

Pose comes from MuJoCo truth, explicitly tagged in results. LiDAR/inertial/leg
odometry is needed before claiming onboard-only autonomy. The virtual scanner
is not a calibrated Go2 L1 model. Terrain retains multiple support heights per
XY cell and cannot connect floors by vertical teleportation. It recognizes
near-horizontal surfaces and small steps, not arbitrary slopes or large risers.
Old range hits are cached conservatively; dynamic clearing is not implemented
in radiation mode.

The [terrain qualification](DOUGIE_TERRAIN_QUALIFICATION.md) distinguishes
geometry admission from executable locomotion. The 7.5 cm graph-edge threshold
is a geometry heuristic, not a validated stair capability: the current policy
stalls or strikes stair edges under conditions that pass that geometric test.

## Artifact meanings

| Artifact | Meaning |
|---|---|
| `measurements.jsonl` | Noisy counts, integration duration, measured dose, rate and XYZ |
| `trace.jsonl` | Planner state, coverage and timing |
| `run_config.json` | CLI configuration and source/asset-manifest hashes |
| `radiation_map_3d.npz` | XYZ queries, mean rate, log-rate uncertainty and upper rate |
| `gp_posterior.npz` | Inducing coordinates, posterior, kernel factorization and parameters |
| `observed_geometry.npz` | Range hits and sampled observed-free voxels |
| `radiation_summary.json` | Outcome, estimated/actual dose, return state and height coverage |
| `evaluator_field.json` | Post-run truth for evaluation; not an autonomy input |

A volumetric plot is not evidence that every height was measured. Inspect
`sampled_height_range_m` and uncertainty before interpreting reconstruction.

## Attribution

- Geoff-Sim: history and package-specific notices retained.
- Go2 model: `unitreerobotics/unitree_mujoco`; its pinned BSD-3-Clause license
  is downloaded and recorded in the asset manifest.
- Policy/observation contract: `fan-ziqi/rl_sar`, Go2 `robot_lab` deployment,
  copyright 2024–2025 Ziqi Fan, Apache-2.0. Configuration/license are downloaded
  beside the policy.
- MuJoCo, ROS 2, Nav2, SLAM Toolbox, NumPy, SciPy and PyTorch retain their licenses.

`README-GEOFF.md`, `install-geoff.sh`, `docker-compose.geoff.yml` and
`.github/legacy/verification-geoff.yml` preserve the historical profile. G1
results and research have not been relabeled as Go2 evidence.
