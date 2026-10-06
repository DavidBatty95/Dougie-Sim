# Dual transport: acquisition diagnosis and native validation (2026-10-05)

Historical checkpoint. The [6 October native-contract follow-up](NATIVE_PAYLOAD_CONTRACT.md)
records the subsequent two complete dual-delivery passes and retained failures.

## Status

The two-colour/four-table scene now obtains distinct-frame confirmations for
both cubes and both destinations. One diagnostic run completed a verified red
delivery. **No complete two-cube delivery is validated.** Native standing and
turning stability remain blockers for both the cycle-01 baseline and experimental
fluid-cycle-02 policy.

## Measured perception problem and changes

At 384×216, baseline run `20261005T174903Z` received images with a median first
processing-attempt age of **3.11 s**. Distinct processed acquisition stamps were
**2.50 s apart (median)**, exceeding the existing 1.5 s geometry-confirmation
interval. This run confirmed no objects. The delay was already present when the
colour callback first attempted pairing, before depth geometry processing.

Reducing the native raycast camera to 192×108 restored median acquisition
intervals to **0.50 s**, with p95 approximately **1.00 s**. Baseline run
`20261005T175131Z` had median first-attempt age **0.82 s** and processed age
**0.89 s** (p95 **1.38 s**). Both cubes and both targets acquired confirmation.
This resolution is now the launcher default; width and height remain explicit
CLI options and are recorded in the manifest. Geometry/confirmation thresholds
are unchanged. This addresses the observed acquisition bottleneck; it does not
establish a hard sensor-latency bound.

Additional changes:

- Ignore replayed/out-of-order frames globally in consensus. Previously, an old
  ambiguous frame could revoke newer confirmation, or an old empty frame could
  prune future-stamped memory.
- Skip already-processed acquisitions before geometry/TF work.
- Record transformed odometry-frame detections, synchronized robot-state stamps,
  acquisition intervals, processing times, and bounded processing-attempt history.
- Preserve attempt/history diagnostics even when no image was processed.
- Record torso rotation for offline geometry profiling.

History is bounded (180 successful frames, 600 processing attempts); reported
distributions describe retained samples, not necessarily an entire long run.

## Retained native results

All runs used `transport_dual`, zero people, seed 42, `payload_v1`, and the built
ROS verification overlay with `--fast-start`. Evaluator data remained valid.
Seed equality does not make asynchronous native runs deterministic.

| Run (UTC) | Policy | Camera | Allowance | Result |
|---|---|---|---|---|
| `20261005T174836Z` | fluid cycle 02 | 384×216 | 120 s | Fell during zero-command startup, before survey; no saved image history |
| `20261005T174903Z` | cycle 01 | 384×216 | 120 s | No visual confirmations; drifted/fell during survey |
| `20261005T175131Z` | cycle 01 | 192×108 | 120 s | Red delivery verified (0.192 m lift, 3.247 m travel); deadline during blue survey; no fall/drop |
| `20261005T175356Z` | cycle 01 | 192×108 | 420 s | All four IDs confirmed; fell and dropped red during delivery navigation |
| `20261005T175516Z` | fluid cycle 02 | 192×108 | 420 s | All four IDs confirmed; fell during destination survey before grasp |

Raw evidence lives under
`ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/<run>/`.
Portable timing/outcome comparison:
[`results/transport_dual_perception_20261005.json`](results/transport_dual_perception_20261005.json).

The fluid candidate's matched MJLab probe improvements do not qualify it for
native deployment. Next work must compare training/deployment observations,
previous-action semantics, arm composition and actuator dynamics, then require
stable empty-hand and loaded native turn probes before another full mission.
Pickup remains RGB-D/measured-state driven with a capture-radius simulated weld.

## Reproduction

Verification: **412 bridge, 95 benchmark, 18 bringup, 8 focused script and 8
Torch/training tests passed**. ROS package suites were run separately: a combined
invocation hit the existing `test` package-name collision during collection.
The host lacked Torch, so the eight training tests were subsequently run and
passed in the GPU-enabled MJLab environment.

```bash
python3 scripts/run_transport_demo.py --world transport_dual --people 0 --seed 42 \
  --duration 420 --policy vendor/mjlab/logs/payload_training/cycle_01/policy.onnx \
  --carry-posture payload_v1 --camera-width 192 --camera-height 108 \
  --no-rviz --fast-start
```

This is a diagnostic command, not a reliable-success demonstration. Omit
`--fast-start` if the verification overlay/native simulator needs rebuilding.
Use `scripts/analyze_transport_perception.py RUN [RUN ...] --output REPORT.json`
for portable timing summaries. In the sourced ROS environment,
`scripts/profile_transport_perception.py RUN` profiles saved RGB-D geometry.
