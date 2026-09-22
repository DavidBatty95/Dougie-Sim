# G1 Command-Response Calibration Protocol

> Archived pre-collection protocol. This is not evidence of a fitted response envelope.
> Current tooling and limitations are documented in `docs/TRAINING_AND_CALIBRATION.md`.

## Purpose

This protocol measures the delay and motion envelope between a planar velocity
command and the resulting Unitree G1 body motion. The resulting model will bound
the robot rollout used by the uncertainty-aware navigation safety filter.

Calibration is not a navigation benchmark. Run it without pedestrians, Nav2
goals, social planning, or collision filtering. Do not fit response parameters
from final social-navigation evaluation trials.

The machine-readable summary must validate against
`docs/schemas/g1_response_calibration.schema.json`. Raw time series remain in a
rosbag or equivalent immutable log referenced by the summary.

## Research Quantities

Measure:

- command-to-motion latency;
- 10% and 90% rise times;
- steady-state velocity and tracking error;
- acceleration and deceleration envelopes;
- stop time and longitudinal stopping distance;
- lateral and heading drift during forward motion;
- residual displacement 1, 3, and 5 seconds after stop;
- yaw-rate response and yaw settling;
- maximum tilt, minimum base height, and falls;
- variation across repeated trials, initial speed, simulation, and hardware.

The first model should remain simple: command delay followed by bounded planar
acceleration/deceleration and a residual position-error bound. Introduce a more
complex model only if held-out coverage demonstrates that this model is
insufficient.

## Configuration Freeze

Before collection, record and freeze:

- locomotion policy path and SHA-256;
- policy profile, observation/action dimensions, and command scaling;
- command acceleration limits and deadband;
- startup, action-ramp, and action-rate-limit settings;
- MuJoCo scene, timestep, keyframe, and simulator revision;
- robot URDF/MJCF and bridge configuration hashes;
- ROS workspace revision and dirty state;
- container image ID;
- hardware firmware and network/DDS settings when applicable;
- test-surface description and approximate friction when known.

Changing any item requires a new dataset ID. Do not merge trials collected under
different response-critical configurations into one fitted envelope.

## Test Environment

### Simulation

Use the bundled empty, flat scene and the checksum-pinned official Unitree G1
velocity policy. Disable:

- Nav2 and SLAM goals;
- pedestrian animation and tracking;
- human-aware planner and collision supervisor;
- random, waypoint, frontier, and patrol command sources;
- camera publication and unrelated high-cost diagnostics.

Keep only one calibrated command publisher. Record the command consumed by the
locomotion policy rather than assuming it equals the requested command.

### Hardware

Hardware calibration is optional and follows simulation. It requires:

- an independent emergency stop;
- a dedicated safety operator;
- a level, restricted test area;
- conservative speed limits;
- verified command-source exclusivity;
- a pre-test standing and low-speed check;
- a clearly marked stopping corridor;
- no nearby participants or unprotected obstacles.

Begin at `0.10 m/s`. Stop testing after any fall, repeated large tilt, unstable
settling, unexpected command ownership, or response outside the current safety
corridor.

## Command Matrix

Use separate calibration and held-out repetitions. Recommended initial matrix:

| Test family | Command levels | Hold before | Command duration | Hold after | Simulation repetitions | Hardware repetitions |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Forward step-stop | 0.10, 0.15, 0.20, 0.25, 0.30, 0.35 m/s | 3 s | 8 s | 8 s | 10 | 5 |
| Yaw step-stop | -0.30, -0.20, -0.10, 0.10, 0.20, 0.30 rad/s | 3 s | 8 s | 8 s | 10 | 3 |
| Forward transitions | 0.10 to 0.20, 0.20 to 0.35, 0.35 to 0.15 m/s | 3 s | 6 s per level | 5 s | 8 | 3 |
| Emergency stop | 0.15, 0.25, 0.35 m/s to zero | 3 s | reach steady state | 8 s | 10 | 5 |

Do not add lateral tests until the selected locomotion policy passes independent
lateral tracking, settling, and balance gates. Existing evidence indicates that
lateral response is not reliable enough to assume as a safety action.

Randomize command-level order within a platform and split, while retaining a
fixed recorded order file. Insert a complete upright reset and settle period
between trials. Simulation calibration and held-out seeds must be disjoint.

## Required Topics And Signals

Record at a common or reconstructable time base:

- requested calibration command;
- command consumed by the locomotion policy, published as stamped
  `/g1/policy_cmd_vel` after deadband, saturation, axis isolation, and rate
  limiting;
- `/odom` pose and twist;
- `/g1/base_height` exact simulation base height or a hardware-equivalent source;
- base orientation, angular velocity, and height;
- `/lowstate` or equivalent joint/base state;
- locomotion diagnostics and fall state;
- `/tf` and `/tf_static`;
- calibration phase and trial ID;
- simulator time and wall/monotonic timing metadata.

Recommended minimum effective rate is 100 Hz for command and body-state response.
If `/odom` is slower, record its actual rate and timestamp age. Do not interpolate
across missing intervals without marking them.

## Phase Protocol

Each trial follows these phases:

1. **Reset:** place the robot at the defined upright start state.
2. **Settle:** require low velocity and acceptable tilt for at least 3 seconds.
3. **Pre-command:** publish zero and record the baseline noise distribution.
4. **Step:** publish the selected command with a recorded command timestamp.
5. **Steady hold:** continue until the prescribed duration is complete.
6. **Stop:** command exact zero, bypassing nonzero deadband substitutions.
7. **Post-stop:** continue recording for at least 5 seconds.
8. **Validate:** mark falls, saturation, stale data, command conflicts, and
   incomplete phases before calculating metrics.

A trial is infrastructure-invalid only for predefined acquisition failures such
as missing required topics, timestamp reversal, command publisher loss, or reset
failure. Falls, failure to move, poor tracking, and excessive drift are valid
response outcomes and must remain in the dataset.

## Metric Definitions

Let `t_command` be the timestamp at which the nonzero command is observed on the
topic consumed by the locomotion policy. Use measured planar body velocity after
low-pass filtering with a frozen, reported filter.

### Motion Onset

Motion onset is the first time velocity in the commanded direction exceeds
`0.03 m/s` continuously for at least `0.20 s`. Command latency is:

```text
latency_s = t_motion_onset - t_command
```

If motion never begins, latency is null and `no_motion` is true. Do not replace a
missing latency with the trial duration.

### Rise And Steady State

Estimate steady velocity as the median commanded-direction velocity over the
last 2 seconds before stop. Report:

- first crossing of 10% steady velocity;
- first sustained crossing of 90% steady velocity;
- steady velocity;
- absolute and relative tracking error.

Use the frozen `0.10 s` trailing mean for onset, rise, and stopping metrics. Test
steady-window stability with a separate `0.80 s` trailing mean matching the
configured gait cycle, so expected step-to-step body-velocity oscillation is not
misclassified as a missing steady response.

Estimate effective acceleration as `0.8 * steady_response / (t90 - t10)` and
effective deceleration as `steady_response / stop_time`. These rise/stop-based
quantities are authoritative for envelope fitting; do not use a peak derivative
of gait-oscillating pose velocity as locomotion capability.

Flag trials where a stable steady interval is unavailable. For negative yaw,
calculate thresholds in the signed commanded direction.

### Stopping

Let `t_stop_command` be the first consumed exact-zero command after steady hold.
Stop time is the first subsequent time commanded-direction speed remains below
`0.03 m/s` for `0.50 s`.

Longitudinal stopping distance is displacement along the body heading at
`t_stop_command`, integrated until stop time. Also report absolute planar
displacement so turning or lateral drift cannot hide motion.

Post-stop drift is planar displacement from stop time to 1, 3, and 5 seconds
after stop time. If the recording does not cover a horizon, report null and mark
the horizon unavailable.

### Stability

Report maximum absolute roll/pitch tilt, minimum base height, any fall, and the
time of first fall. A fall is a valid failed response trial, not an exclusion.

## Fitting The Conservative Envelope

Fit only on calibration trials. Evaluate once on held-out trials.

The estimator, interpolation, confidence calculation, sample-size requirement,
and failure policy are frozen in
`docs/G1_RESPONSE_ENVELOPE_FITTING_SPEC.md` and
`g1_response_envelope_fit_v1.yaml`. Validate both collection and fitting
specifications before acquisition with:

```bash
python3 scripts/validate_g1_response_fitting_spec.py
```

For each command range, estimate:

- upper command-delay bound;
- lower acceleration and deceleration capability;
- upper stopping-time and stopping-distance bounds;
- upper longitudinal, lateral, and yaw residual-error bounds;
- upper post-stop drift bounds.

Version 1 uses binwise calibration extrema plus a leave-one-seed-out extreme-
spacing reserve. This is a declared engineering estimator, not a probabilistic
guarantee. Joint held-out trial coverage is the inferential check; do not call a
sample maximum or its spacing reserve a probabilistic bound.

The deployed rollout should use the least favorable bound in the applicable
command bin. Interpolate only in a conservative direction. Extrapolation beyond
the measured command range is prohibited.

## Acceptance Gates

The response model is ready for planner integration when:

- all required configurations and raw artifacts are traceable;
- at least 90% of planned calibration and held-out trials are evaluable;
- held-out stopping trajectories remain inside the claimed envelope at the
  predeclared empirical coverage target;
- each command family has at least 29 evaluable held-out trials and its one-sided
  95% Clopper-Pearson lower bound on joint trial coverage is at least 0.90;
- no command level used by navigation lies outside the calibrated range;
- latency and stopping bounds do not decrease unexpectedly with higher speed;
- simulation and hardware results are reported separately;
- hardware response is contained by the selected deployment envelope, if
  hardware is claimed;
- failures and outliers have documented causes without post-hoc removal.

If held-out coverage fails, widen or reformulate the envelope before modifying
the navigation filter. Do not tune the envelope on social-navigation outcomes.

## Artifact Layout

```text
response_calibration/<dataset-id>/
  manifest.json
  protocol.yaml
  command_order.csv
  trials/
    <trial-id>/
      trial.json
      samples.csv
      rosbag/
      stdout.log
      stderr.log
  aggregate/
    response_summary.json
    response_trials.csv
    fitted_envelope.yaml
    held_out_coverage.csv
    figures/
```

`response_summary.json` is the schema-validated index. Large raw bags may be
stored externally if their stable location and SHA-256 are retained.

## Immediate Implementation Tasks

Acquisition foundations are now implemented in `measure_g1_fixed_command.py` and
`run_g1_fixed_command_test.sh`: the node owns the isolated `/cmd_vel` source,
labels settle/command/stop phases, records each new odometry sample with source
and arrival timing, derives body twist over a frozen 0.10-second pose window,
retains reported odometry twist as a diagnostic, and records stamped effective
policy commands from `/g1/policy_cmd_vel`. It checks stream freshness, emits a repeated
exact-zero cleanup burst, and saves a hashed CSV sample artifact.

Offline extraction and assembly are implemented by:

```bash
python3 scripts/extract_g1_response_metrics.py raw_samples.csv \
  --metadata trial_metadata.json --output trial_response.json
python3 scripts/assemble_g1_response_dataset.py trial_response*.json \
  --metadata dataset_metadata.json --output response_summary.json
python3 scripts/summarize_g1_response_dataset.py response_summary.json \
  --output repeatability_summary.json
```

The metadata JSON contains `dataset_id`, `platform`, `collected_at`, and the
schema provenance object. Extraction metadata also contains a `trial` object
with `trial_id`, `split`, `seed`, `repetition`, and `command_family`. The
extractor uses the pose-derived body twist, freezes a 0.10-second trailing-mean
response filter and 0.80-second gait-cycle stability filter, and validates its output against
`docs/schemas/g1_response_calibration.schema.json`.

Remaining tasks:

1. Collect calibration simulation data and freeze the resulting envelope hash.
2. Collect the still-unopened held-out simulation data.
3. Run the one-shot joint held-out coverage analysis.
4. Only then expose response-model parameters to the navigation supervisor.

The frozen three-command live pilot now has two repetitions per command. All six
trials are valid, fall-free, and have complete drift horizons. The repeatability
artifact is
`results/response_pilot_empty_v1/repeatability_summary.json` in the benchmark
package. Persistent under-tracking at `0.10 m/s` and `0.10 rad/s` is a measured
policy response and must be represented by the fitted envelope. The initial
forward stability failures were attributable to expected gait-cycle velocity
oscillation under the onset filter; all six pass the separately declared gait-
cycle stability test. This pilot validates collection and extraction but is too
small for envelope fitting or a coverage claim.
