# Native payload contract and dual-delivery checkpoint — 6 October 2026

The subsequent [hardware-interface/performance pass](HARDWARE_FIDELITY.md) adds
two further unchanged-code dual-delivery passes (`145814Z`, `150306Z`), bringing
cycle-03 development attempts to 6 complete passes among 11 evolving integrations.
The historical slices below retain their own denominators and source identities.

## Result and scope

**Two consecutive complete dual-cube native runs passed**, with the final code
and cycle-03 checkpoint unchanged between them:

| Run | Seed | Verified deliveries | Falls/drops | Evaluator |
|---|---:|---|---|---|
| `20261006T073223Z` | 42 | red and blue | none recorded | valid |
| `20261006T073631Z` | 43 | red and blue | none recorded | valid |

The second run also cancelled navigation, waited for fresh localization and
resumed delivery after a measured odometry interruption. These are same-layout,
zero-person development repeats, not held-out environment or crowd acceptance.
The seed changes the pedestrian generator; with zero people it is primarily a
repeat under asynchronous runtime variation, not a new geometric scenario.
Reported zero contact samples do not establish complete robot/table contact
coverage. Grasp remains a capture-radius weld, not physical BrainCo force closure.

## Diagnosed contract mismatches

1. **Result collection latency.** Completed inference waited for the next 50 Hz
   control tick. A separate 2 ms result poll now uses the same lowstate, startup,
   tilt, stale-result, deadline and stop-generation checks without advancing
   observation history or submitting extra inference. A native follow-up measured
   median completion-to-publication latency 1.076 ms (p95 4.374 ms), and median
   submission-to-publication latency 8.059 ms (p95 17.181 ms). This is upstream of
   arm composition and motor application; it is not a hard real-time guarantee.
2. **Rigid-body mismatch.** Historical MJLab torso mass was 7.818 kg versus native
   9.598 kg, with different waist offsets/inertias. Historical effort limits also
   differed for hip roll and waist yaw. `--dynamics native_inertial` copies native
   matching body inertials/transforms and motor limits. Training retains its
   original collision assets and position-actuator formulation; complete physics
   equivalence has not been established.
3. **Untrained application delay.** `--max-delay-steps 2` samples 0/20/40 ms
   per-episode delay. Actor history uses the applied policy action before external
   arm composition. Historical defaults remain available with zero delay.

The new candidate trained for **300 additional PPO iterations, 1,024 environments,
seed 53**, resuming cycle 01, with the fluid proxy objective, native inertials,
0–40 ms delay exposure and arm transitions. Artifact:
`vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/`.
ONNX SHA-256: `25bab45eda339e314c7b15156ba9e3695cfdcfffa157b990a720e59d3d3e65cc`.

## Isolated diagnostics

`scripts/probe_native_policy.py` runs the native geometry in synchronous CPU
MuJoCo without ROS/camera scheduling. It supports explicit delayed actions and
prescribed empty-hand arm composition. This is a dynamics diagnostic only.

- Previous fluid candidate: 20 s immediate standing survived; +20 ms delayed
  standing fell at 18.18 s with 1.389 m displacement.
- Cycle 03: +20 ms delayed standing survived 20 s with 0.046 m displacement.
- Immediate 0.6 rad/s turning: displacement fell from 4.109 m (fluid cycle 02)
  to 0.481 m (cycle 03); both survived. These are displacement comparisons, not
  complete tracking/comfort scores.
- Held-empty cycle-03 probes at +0.35/-0.35 rad/s survived, with 0.069/0.110 m
  displacement and +0.992/-1.488 rad net rotation over a 20 s trial that begins
  with 5 s standing. Native loaded motion had stalled at ~0.18 rad/s, so ordinary
  loaded-turn taper now has a 0.35 floor and retains its 0.6 cap/aligned stop.

Raw probes: `vendor/mjlab/logs/native_contract_20261006/`.
Earlier held-empty probe JSON scope text predates the optional-arm wording;
their `held_empty: true` field identifies the actual prescribed-arm condition.

Matched seed-71, 32-environment MJLab loaded-turn evaluations with native
inertials and 0–40 ms delay exposure: baseline **23/32 survivors**, cycle 03
**32/32 survivors**. Right-pivot p95 jerk was 124.25 versus 91.04 m/s³, with net
yaw -2.980 versus -2.665 rad. However, cycle-03 left-pivot net rotation was only
0.432 rad against baseline 1.355 rad; some stop/arc/restart proxy metrics worsened.
Thus smoother motion is not universally demonstrated at equivalent tracking.
The rigid virtual-cup proxy remains neither liquid simulation nor spill-proof
acceptance. Reports are in `native_delay_{baseline,cycle_03}_eval71_20261006/`
under the payload training logs.

## Mission corrections

- Absolute navigation/dock/rotate goals retry at most twice following localization
  interruption, after action termination and three distinct fresh odometry stamps.
  Recovery waits at most 5 s and retains the original motion deadline.
- Previously confirmed RGB-D positions may guide viewing direction after expiry;
  they cannot satisfy discovery/grasp/placement freshness.
- The large pregrasp arm transition occurs at staging distance before precision
  docking, so docking can correct transition-induced base motion.
- Keep the retracted arm under continuous ownership between deliveries; restore
  the measured rest pose and release authority after the final delivery.

## Retained attempts

All raw runs are under
`ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/`.

| Run | Candidate/configuration | Outcome |
|---|---|---|
| `20261006T065645Z` | fluid 02, prompt result polling | Odometry interruption before grasp; no recorded fall |
| `20261006T065806Z` | baseline, prompt polling | Lifted red; odometry interruption; no recorded fall/drop |
| `20261006T070237Z` | baseline, bounded localization recovery | Red verified; blue navigation aborted from lethal start space |
| `20261006T070740Z` | baseline, viewing hints | Fell during initial survey; recovery timed out |
| `20261006T071945Z` | native-delay cycle 03 | Red verified; blue carry-posture tolerance failed |
| `20261006T072348Z` | cycle 03, pre-dock arm transition | Red lifted; retreat stalled below yaw response region |
| `20261006T072827Z` | cycle 03, turn floor | Red verified; blue navigation aborted |
| `20261006T073223Z` | cycle 03, continuous inter-delivery ownership | Both verified, complete |
| `20261006T073631Z` | unchanged candidate | Both verified, complete; localization recovery exercised |

Cycle 03 therefore has **2 complete passes among 5 evolving integration attempts**,
and **2/2 on the final repeated configuration**. Keep both denominators.

Portable ledger: [transport_native_contract_20261006.json](results/transport_native_contract_20261006.json).

## Subsequent S2 navigation gate

The original navigation policy with prompt inference-result collection was tested
in a synchronized four-person, three-goal pilot (seed 42, 120 s allowance).
Small-open reached 3/3 with zero collisions but scored **67.933**, below the
unchanged 70-point threshold. Medium-doorway reached 2/3 with **one collision**
and -0.022 m minimum body clearance. Strict result: **0/2**. The delayed-training
payload candidate was not substituted into this navigation pilot.

Retained cohort: `avoidance_matrix/20261006T074551Z/` under benchmark results;
portable [pilot report](results/avoidance_native_contract_pilot_20261006.json).
Trace inspection places the doorway collision after lateral clearance recovery
and an emergency stop as a sensed person approached at about 0.93 m/s. Root-cause
isolation of tracking, prediction and physical stopping remains the next S2 work;
the timing improvement alone did not close the navigation gate.

Verification: **417 bridge, 95 benchmark, 18 bringup, 19 focused script, and
9 Torch/training tests passed**. Native training/evaluation and diagnostic probes
are additional execution evidence, not substitutes for stage acceptance.

## Reproduction

```bash
python3 scripts/run_transport_demo.py --world transport_dual --people 0 --seed 42 \
  --duration 420 \
  --policy vendor/mjlab/logs/payload_training/native_delay_cycle_03_20261006/policy.onnx \
  --carry-posture payload_v1 --no-rviz --fast-start
```

## Phase gates still open

This checkpoint advances S1/S4/S5 components. It does not complete S0–S7.
S0 still needs clean rebuild/reproduction; S1 needs complete contact coverage and
acquisition-stamped object evaluation; S2 needs the strict six-world/three-seed
navigation matrix and held-out encounters; S3 needs contact-only mobile grasping;
S4 needs broader injected-fault coverage; S5 needs repeated objects/layouts and
declared loaded tracking/comfort acceptance; S6 needs held-out crowd deliveries.
S7 requires access to a physical G1 and calibrated BrainCo/sensor interfaces.

## Subsequent acquisition/approach slice

The simulator was rebuilt with `OBJECT_V2`/`SURFACE_V2` packets carrying native
Unix acquisition nanoseconds. ROS preserves those stamps per detection; the
cache rejects replay/out-of-order refresh and the evaluator rejects stale/future
acquisitions. Repeated samples cannot increase placement dwell. Legacy packet
support is explicitly labelled as receipt-time evidence. All ten simulator
CTest targets passed, including the native packet timestamp test.

Four more runs (seeds 44 and 45) were retained:

| Run | Change | Outcome |
|---|---|---|
| `20261006T075719Z` | Native acquisition timestamps | Red verified; distant blue destination not reacquired |
| `20261006T080200Z` | Bounded Nav2 observation-viewpoint search | Red verified; blue arm execution failed tolerance |
| `20261006T080844Z` | Measured support-edge pickup approach | Both verified; complete; native acquisition evidence valid |
| `20261006T081830Z` | Bounded support-plane processing, seed 45 | Both verified; complete; native acquisition evidence valid |

Pickup now estimates a face-normal approach from the visible RGB-D support
rectangle near the cube, requiring two distinct consistent support-angle views.
It does not use scene table coordinates or global preset orientations. Missing
support estimates retain the prior box-bearing fallback. A historical confirmed
position can request a Nav2-checked observation viewpoint, but fresh visual
confirmation is still mandatory. The final run exercised this closer-view search
for blue and used measured support-edge approaches for both cubes.

Overall cycle-03 development ledger is now **4 complete passes among 9 evolving
integration attempts**. The earlier 2/2 repeated configuration is distinct from
the acquisition/approach follow-up (2/4 while evolving, 1/1 on its latest code).
Neither denominator constitutes a held-out reliability estimate. The `080844Z`
run lifted red/blue 0.220/0.223 m and transported them 3.244/3.218 m;
both placement verifications passed, with no recorded fall/drop. Its many docking
corrections remain an efficiency and precision-control concern.

Support-plane processing now rejects unrelated bounding boxes before rectangle
fitting and uses a decimated support mask; cube/pad detection retains full image
resolution. On the next native run, retained acquisition-interval p95 returned
from 1.50 s to 1.00 s, and processed-image-age p95 from 2.02 s to 1.67 s. That
run lifted red/blue 0.210/0.218 m and transported them 3.183/3.168 m. These runs
had different perception source hashes, so they are consecutive successes, not
an unchanged-code two-trial cohort.

Portable follow-up ledger:
[transport_acquisition_approach_20261006.json](results/transport_acquisition_approach_20261006.json).
Latest checks: **426 bridge, 95 benchmark, 18 bringup, 19 focused script,
9 Torch/training tests and 10 native CTests passed**.

Native acquisition timestamps are implemented; complete physical contact
accounting, broader recovery qualification and the other phase gates remain open.
