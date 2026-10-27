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
    ├─ noisy body-frame IMU, encoders, foot forces and LiDAR → fused pose
    ├─ field transport proxy → Poisson detector → count + estimated motion window
    └─ independent exposure evaluator (not a planner input)
                  ↓
observed multi-height support + observed-geometry/window radiation GP
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

Default radiation missions use `odometry.py` and detached observations from
`onboard_sensors.py`. IMU prediction and stance-leg updates are corrected by
observed-cloud registration. The policy uses measured joints/gyro and estimated
gravity. Scans retain capture-time transforms; no true base pose or field metadata
enters the planner. `--localization ground-truth` retains the historical comparator,
and the ordinary planar profile still has simulator-truth odometry. RViz/world
geometry and physics summaries remain diagnostic. The virtual scanner is not a
calibrated Go2 L1 model. Robust navigation uses quality-gated keyframes and
confirmed loop/anchor matches. Accepted corrections are distributed in time and
replayed consistently through support and count histories; this is bounded loop
correction, not a fully qualified pose-graph SLAM system.

Recovery has explicit sensing/support/tracking reasons, bounded waits, checked
retreats and exposure accounting. Unrecoverable stops retain dose integration to
the original mission deadline. Robust smooth routing also reserves the follower's
tracking-error allowance in its planned clearance. The information/cost scorer is
available as an experimental alternative; legacy utility remains the default
after mixed matched-task dose results. [Implementation and evidence](DOUGIE_ROBUST_MISSIONS.md).

Terrain retains multiple support heights per XY cell and cannot connect floors
by vertical teleportation. The layered mapper supports observed shallow slopes;
industrial stairs and noisy sensor-only multi-floor missions remain unqualified.
Old range hits are cached conservatively; dynamic clearing is not implemented
in radiation mode.

The [terrain qualification](DOUGIE_TERRAIN_QUALIFICATION.md) distinguishes
geometry admission from executable locomotion. The 7.5 cm graph-edge threshold
is a geometry heuristic, not a validated stair capability: the current policy
stalls or strikes stair edges under conditions that pass that geometric test.

## Artifact meanings

| Artifact | Meaning |
|---|---|
| `measurements.jsonl` | Noisy counts, integration duration, estimated trajectory bins, measured dose/rate and observation map revision |
| `map_corrections.jsonl` | Onboard-derived transforms and revision intervals; apply to older observation epochs before combining them with the final map |
| `localization_evaluation.jsonl` | Estimated pose and explicitly evaluator-only physical pose/error |
| `trace.jsonl` | Planner state, coverage and timing |
| `run_config.json` | CLI configuration and source/asset-manifest hashes |
| `radiation_map_3d.npz` | XYZ queries, mean rate, log-rate uncertainty and upper rate |
| `gp_posterior.npz` | Inducing posterior/kernel; geometry models also retain observed features and bounded window history |
| `observed_geometry.npz` | Range hits and sampled observed-free voxels |
| `radiation_summary.json` | Outcome, estimated/actual dose, return state and height coverage |
| `evaluator_field.json` | Post-run truth for evaluation; not an autonomy input |
| `circuit_evaluation.json` | Evaluator-only frozen physical targets and dwell witnesses for matched-work circuit experiments |

A volumetric plot is not evidence that every height was measured. Inspect
`sampled_height_range_m` and uncertainty before interpreting reconstruction.
The final GP/terrain archives use `radiation_summary.json`'s
`map_consistency.map_revision`. Raw measurement and localization logs preserve
their capture-time estimates; they are not silently rewritten after a loop correction.

## Attribution

- Geoff-Sim: history and package-specific notices retained.
- Go2 model: `unitreerobotics/unitree_mujoco`; its pinned BSD-3-Clause license
  is downloaded and recorded in the asset manifest.
- Policy/observation contract: `fan-ziqi/rl_sar`, Go2 `robot_lab` deployment,
  copyright 2024–2025 Ziqi Fan, Apache-2.0. Configuration/license are downloaded
  beside the policy.
- MuJoCo, ROS 2, Nav2, SLAM Toolbox, NumPy, SciPy and PyTorch retain their licenses.

The historical G1 runtime, launchers and CI are available through the
[pre-cleanup history](DOUGIE_HISTORY.md). They are no longer bundled in the Go2
checkout; their results and research have not been relabeled as Go2 evidence.
