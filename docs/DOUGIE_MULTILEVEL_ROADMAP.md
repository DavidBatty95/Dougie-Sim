# Stair training and multi-level radiation demo roadmap

Assessment: 12 October 2026. **This is a proposed development plan, not a trained
policy or a completed multi-floor mission.** The intended extension is 3-D
radiation inference with navigation on connected floor/stair surfaces, rather
than unconstrained free-space 3-D motion.

For subsequent implementation and measured full-height ramp missions, see the
[facility development record](DOUGIE_FACILITY.md). This document preserves the
original planning assessment.

## Recommendation

Build the demo around a visible decision: **the shortest route is radiologically
expensive, so Dougie surveys an alternative floor, descends by a different route,
and returns with an explicit exposure reserve.** Keep the walking controller and
the radiation executive separate: train terrain locomotion, then use measured
locomotion capability and timing in dose-aware planning.

Develop two milestones:

1. A small ramp-connected mezzanine, with two routes and height-separated sources,
   to exercise vertical sensing, mapping, shielding and route selection early.
2. A two-storey facility with independently qualified stair ascent, descent,
   landing turns and stopping. Add a third storey after two-floor repeatability.

## What the robot can do today

The [680-trial terrain assessment](DOUGIE_TERRAIN_QUALIFICATION.md) is the relevant
evidence, rather than isolated successful clips:

| Capability | Existing evidence | Demo implication |
|---|---|---|
| Flat walking | 60/60 strict holdout passes | Established baseline |
| Tested 10-degree ramps, each direction | 60/60 each | Best candidate for an early raised-level course; autonomous ramp navigation still needs qualification |
| Tested 2 cm random tiles | 60/60 | Useful minor floor damage/roughness |
| 3 cm stair descent | 60/60 | Small transition only |
| 3 cm stair ascent | 59/60 | Does not meet the study's repeatability target |
| 6 cm stairs | Ascent 2/5 strict passes; descent 0/5 despite 5/5 crossings | Calf impacts and stalls remain important |
| 15, 18 and 20 cm stairs | All 30 screening trials failed to complete | Requires a new controller and qualification |
| Radiation-aware exploration | Completed single-layout missions with detector/GP/return accounting | Good mission foundation; no established multi-floor reliability |
| Autonomous door opening, manipulation, self-righting | No Go2 implementation established in this assessment | Later capabilities, with substantial additional control/model work |

The ramp holdouts used centre-line velocity guidance. They do not establish that
the conservative radiation planner can currently find or follow ramp routes.

## Why replacing the policy alone is insufficient

| Component | Current implementation | Required extension |
|---|---|---|
| Locomotion | `simulation.py`: 45 proprioceptive/command values, no terrain input, 12 joint actions, 50 Hz policy | Versioned terrain-aware observation and policy interface |
| Sensing cadence | `radiation_run.py`: 3-D scan normally submitted every 2 simulated seconds | A faster, small local terrain observation for foot placement, independent of the global GP worker |
| Support detection | `simulation.py`: absolute vertical normal component accepts horizontal surfaces without distinguishing top from underside | Reject ceilings/undersides as support; retain confidence and unknown regions |
| Terrain admission | `terrain.py`: conservative mode uses 2 cm edge/relief limits; body-height hits at 12–70 cm are obstacles | Separate slope, stair, drop, body-clearance and foothold reasoning tied to measured controller capability |
| Resolution | 25 cm XY support cells | Locally finer stair/landing geometry; retain coarse long-range graph for performance |
| Graph | Multiple heights per XY already retained, but symmetric adjacent-cell step admission | Directed connector edges with ascent/descent capability, time, uncertainty and landing requirements |
| Return estimates | `clustered_planner.py`: searches outward from home to obtain candidate return costs | Reverse-graph search for candidate-to-home costs once edges become directed; ascent and descent costs need not match |
| Route following | `radiation_run.py`: yaw/forward-speed following and 22 cm waypoint tolerance | Approach/alignment, committed flight traversal, landing verification, turn and controlled descent states |
| Progress/freshness | Fixed stall/freshness stops during ordinary walking | Train stopping on steps; distinguish a supported hold from an unstable interrupted transition |
| Radiation prediction | XYZ GP with a fixed-capacity dictionary and Euclidean kernel | Evaluate cross-slab errors and height coverage; investigate observed-geometry/floor-aware correlations |
| GP capacity | New inducing points stop being admitted after capacity is reached | Measure upper-floor coverage after lower-floor saturation; add replacement or spatial allocation if needed |
| Map storage | Fixed support/hit/free-space cache limits | Ensure first-floor observations do not consume the capacity needed for later floors |
| Novelty | Clustered gain is locally height-aware; baseline novelty merges XY columns | Explicit floor/connector frontier objectives; avoid declaring unvisited upper floors explored |
| Visualization | Global decimated GP points and whole static scene | Floor slices, cutaway view, per-floor sampling quotas and floor/connector status |

For example, a 10-degree ramp rises about 4.4 cm across a 25 cm grid cell, already
above conservative mode's 2 cm edge threshold. Raising the threshold globally
would also admit unqualified abrupt steps. A slope-aware capability model is needed.

## Training approach

### Preferred first experiment: MuJoCo-native GPU training

Use **mjlab / MuJoCo Warp with PPO**, porting its quadruped rough-terrain task to
the pinned Go2 model and the deployment actuator contract. This keeps the model
family close to the native MuJoCo evaluation runtime, although GPU/CPU simulator
agreement must still be measured.

The upstream velocity configurations inspected contain Go1 and G1, not a ready
Go2 configuration. Its terrain library supplies ascending/inverted stairs, ramps
and roughness. The named stair preset tops out at 10 cm; the lower-level generator
supports larger ranges. Neither availability establishes Go2 stair capability.

The checked host reports an **RTX 4080 Laptop GPU, 12,282 MiB VRAM**, driver
580.178.04. Start with 128–256 environments as a proposed memory/throughput probe,
then measure 512/1024 if feasible. Choose parallelism from VRAM use and rollout
throughput, not an inherited 8,192-environment humanoid default. Record laptop
power/thermal conditions; training duration cannot be estimated reliably before
this benchmark.

The current Dougie container uses CPU PyTorch, `vendor/mjlab` is unpopulated, and
the inherited training launchers target G1. Add a separately pinned Go2 training
environment with CUDA dependencies, leaving CPU inference lightweight.

### Existing-Go2 alternative

`robot_lab` exposes `RobotLab-Isaac-Velocity-Rough-Unitree-Go2-v0` and a known
deployment path through `rl_sar`. This avoids building a Go2 asset task from a
Go1 template, but adds Isaac Sim/PhysX setup and sim-to-sim qualification.

Its inspected Go2 configuration explicitly removes the actor's `height_scan`
and base linear velocity. Simply retraining that default is not the proposed
perceptive stair controller. The shipped Dougie `.pt` is an inference artifact;
it is not a complete PPO checkpoint with critic/optimizer/training state.
Actor initialization or distillation can be investigated, but cannot be treated
as a turnkey training resume, especially after changing observations.

### Observation and deployment contract

- Actor: IMU/angular velocity, projected gravity, joint positions/velocities,
  previous actions, velocity command, and a local gravity-aligned elevation patch
  with validity/confidence masks. Add short history or recurrent state if justified.
- Unknown support must remain unknown; no filling missing terrain with flat ground.
- Train the actor with noisy/delayed terrain observations and dropout that can be
  reproduced by deployment sensing. Privileged terrain/velocity information may
  help a training critic or teacher, but must not leak into deployment inputs.
- Preserve the existing 20 ms action period, nominal 5 ms native evaluation step,
  joint ordering, observation scaling, PD gains and torque limits for the first
  experiment. Any intentional changes must be versioned and evaluated together.
- Export policy weights **and** input normalization, action scales, joint mapping,
  history/reset rules, model hash, terrain configuration, seeds and framework pins.
- Verify exported actions against training inference on saved observations before
  testing native MuJoCo walking. Store new artifacts beside the baseline policy.

### Curriculum and objective

Proposed progression (advance by held-out results, not elapsed training time):

1. Flat walking, turning, start/stop and quiet stance using the exact Go2 model.
2. 2–6 cm steps, gentle ramps and roughness; both travel directions.
3. 6–10 cm stair flights, approaching at varied headings and gait phases.
4. 10–15 cm flights, first/last-step transitions and stop/restart.
5. Target 15–18 cm closed-riser stairs with varied treads, landings and turns.
6. Longer flights, descent, interrupted commands, payload, friction and latency
   variation. Treat 20 cm as an additional target, not an assumed capability.

Train descent as a first-class task with its own samples and scoring. Useful
objectives include commanded progress, terrain-relative swing clearance,
controlled touchdown, stable support, stance-foot slip, action smoothness,
joint/torque limits and non-foot impacts. Avoid a reward that merely incentivizes
throwing the robot over a finish line or remaining still to avoid contact penalties.
Add actual one-way flights and landings rather than relying only on pyramid stairs.

Keep radiation source locations and dose decisions out of the locomotion actor.
The mission executive should decide where to go; the locomotion controller should
execute admissible terrain transitions reliably.

## Proposed level and radiation experiment

### Early vertical slice

A roughly 0.7 m mezzanine reached by a roughly 4 m horizontal-run, 10-degree ramp
is close to the tested ramp scale. Add a second connection so avoidance changes
route choice. Qualify autonomous ascent/descent and landing turns on this new
layout before using it as a reliable demonstration.

### Full two-storey target

Use a proposed 2.88 m floor separation, 16 risers at 18 cm divided into two flights
with an intermediate landing, and a second staircase on the other side. Final
widths, treads and landings must follow measured Go2 turning/clearance needs.
Use floor slabs rather than solid blocks extending to the ground; leave usable
space beneath the upper floor and ensure stair openings really connect levels.

Provide a lower pump room, an upper service gallery, overlapping XY corridors,
and source profiles that make route choice genuinely depend on height:

- **Vertical aliasing:** the same XY is hazardous upstairs and comparatively
  shielded downstairs; a flattened map should be a poor model.
- **Short/hot versus long/cool:** a source near one connector makes the longer
  route preferable at one budget, while both remain plausible at another.
- **Return pressure:** accumulated exposure and remaining time force a return
  decision before entering the next flight.
- **Unknown upper floor:** reaching a landing reveals previously unobserved
  support and field structure rather than following a supplied global map.

Floor slabs/walls must explicitly carry attenuation parameters. Currently only
three named shielding geometries receive them in `simulation.py`; visible
collision geometry alone is radiation-transparent. `RadiationField.from_mujoco`
assumes axis-aligned shielding boxes, so rotated shielding requires an oriented
intersection implementation or an axis-aligned design. Derive runtime and archive
shield definitions from the same scene to avoid inconsistent source metadata.

Keep the initial experiment static. Dynamic leaks require changing the static
field cache, detector scenario and GP temporal assumptions, not just an animation.

## Qualification and honest comparison

Use the current contact-aware harness as the independent native-MuJoCo evaluator.
Extend it for longer flights, turns, floor transitions, stopping, sensor errors
and actual deployable policy selection. Separate train/validation/test geometry,
not only counting-noise seeds. Freeze the selected policy before final evaluation.

For each claimed stair condition, target the existing **60 fresh runs** with
strict completion/contact/settling criteria. 60/60 gives a one-sided 95% lower
bound of approximately 95.13% for that condition's sampled distribution; it is
not an all-terrain guarantee. Retain failed runs and qualify ascent and descent
separately. Also run full multi-floor missions: isolated flight success does not
establish an entire mission's reliability.

Compare radiation-blind, flattened-XY hazard and XYZ dose-aware planners under
matched scenes, detector seeds, locomotion, sensing and compute scheduling.
Do not call the existing `baseline` planner a radiation-blind baseline: it already
uses dose. New comparator modes are required. Match task progress/coverage as
well as dose so staying home cannot win the comparison.

Report actual and estimated dose, return outcome, flight completion in each
direction, falls/contact impulses, per-floor observed area, detector height span,
held-out field error and uncertainty calibration, route duration and planning
latency. Use runtime measurements to replace the current generic stair-speed and
return-time heuristics with direction-dependent transition estimates.

## Demo priorities and implementation order

| Priority | Deliverable | Completion evidence |
|---|---|---|
| 1 | Versioned Go2 policy adapter and GPU training smoke task | Correct ordering/scaling; bounded memory; exported-policy parity and native flat walking |
| 2 | Ramp mezzanine plus slope-aware graph and explicit slab shielding | Autonomous up/down course; no ceiling supports or cross-floor graph shortcuts |
| 3 | Stair curriculum and contact-aware qualification | Frozen policy passes held-out ascent/descent, landing and stop/restart cases |
| 4 | Directed stair connectors and transition executor | Asymmetric return-cost tests and witnessed landing/flight execution |
| 5 | Two-floor dose experiment with two connectors | Held-out matched comparisons showing route choice, progress, exposure and return |
| 6 | Presentation and larger layout | Repeatable full mission with floor cutaway, clear decisions and bounded rendering cost |

High-impact presentation additions: a floor selector/cutaway, a height profile of
the travelled route, measured-dose and return-reserve timeline, named mission
checkpoints, and synchronized comparison replays. Reveal hidden source truth
only in an explicitly labelled diagnostic/post-run view. Keep the live policy
demo and accelerated data replay visually distinct.

Other promising capabilities are deliberate stop-and-measure surveys, selecting
measurement viewpoints, retreat/replanning after a sensed obstruction, and
multi-height source localization evaluated independently of GP heatmap appearance.
Blocked-route handling needs occupancy clearing and stale-map handling; these are
not supplied by the current static hit cache. Prioritize these mission-relevant
behaviours ahead of jumping, running, self-righting or manipulation.

## Sources inspected

- Local implementation and [terrain evidence](DOUGIE_TERRAIN_QUALIFICATION.md).
- [Pinned deployment observation/action contract](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/policy/go2/robot_lab/config.yaml).
- mjlab upstream main inspected at `371e3b7ef1c32cbbf1f0527ec04d03a28a0873f1`:
  [framework](https://github.com/mujocolab/mjlab/blob/371e3b7ef1c32cbbf1f0527ec04d03a28a0873f1/README.md),
  [Go1 task](https://github.com/mujocolab/mjlab/blob/371e3b7ef1c32cbbf1f0527ec04d03a28a0873f1/src/mjlab/tasks/velocity/config/go1/env_cfgs.py),
  [terrain presets](https://github.com/mujocolab/mjlab/blob/371e3b7ef1c32cbbf1f0527ec04d03a28a0873f1/src/mjlab/terrains/config.py).
- robot_lab upstream main inspected at `500399ed75f510aeaff28705a8ce736c514dbec3`:
  [framework and training entry points](https://github.com/fan-ziqi/robot_lab/blob/500399ed75f510aeaff28705a8ce736c514dbec3/README.md),
  [Go2 rough task](https://github.com/fan-ziqi/robot_lab/blob/500399ed75f510aeaff28705a8ce736c514dbec3/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/quadruped/unitree_go2/rough_env_cfg.py).

External code was inspected, not installed or trained in this assessment.
