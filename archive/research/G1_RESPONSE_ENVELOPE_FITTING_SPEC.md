# G1 Response-Envelope Fitting Specification

> Archived pre-collection design. A validated fitted envelope has not been produced.
> See `docs/TRAINING_AND_CALIBRATION.md` for maintained implementation status.

## Status And Scope

This document freezes `g1_response_envelope_fit_v1` before collection of the
full response dataset. The machine-readable authority is
`g1_response_envelope_fit_v1.yaml`. Pilot observations motivated completing this
specification but are not inputs to the fitted envelope.

The result is an empirical engineering envelope, not a distribution-free safety
guarantee. Held-out coverage and its exact confidence bound must be reported with
that limitation. Simulation and hardware envelopes remain separate.

## Data Separation

Use only the seven declared calibration seeds to estimate bounds. Do not inspect
the five held-out seeds until the envelope artifact and its SHA-256 have been
frozen. Each seed contributes one trial at every command knot. A trial remains an
outcome when tracking is poor, drift is large, motion is absent, or the robot
falls; only predefined infrastructure-invalid trials are unevaluable.

The two command families are fitted and accepted separately. Positive and
negative yaw are separate branches because symmetry is not assumed.

## Binwise Estimator

Group calibration trials by exact signed command. At each command knot and for
each declared metric, require at least six evaluable calibration values.

For an adverse upper metric with values `y_1 ... y_n`:

1. Compute the full-sample maximum `M`.
2. Leave out each seed in turn and compute the maximum of the remaining values.
3. Compute each positive held-out violation above that remaining maximum.
4. Let `s_upper` be the largest violation, including zero.
5. Set the raw upper bound to `M + s_upper`.

For an adverse lower metric, repeat the construction around the minimum. Let
`s_lower` be the largest positive amount by which a left-out value falls below
the remaining minimum. Set the raw lower bound to
`max(0, full_minimum - s_lower)`.

For distinct values, these slacks equal the spacing between the two largest or
two smallest observations. They provide a declared calibration-only engineering
reserve without pretending that bootstrap resampling can infer values beyond a
small-sample maximum. They do not create a probabilistic guarantee.

`steady_response` and `effective_acceleration` receive both lower and upper
bounds because unexpectedly weak and unexpectedly fast response can each be
adverse. Deceleration capability and minimum base height receive lower bounds.
Delay, stopping, drift, tilt, and displacement quantities receive upper bounds.

After fitting knot values, upper bounds use a cumulative maximum with increasing
command magnitude. Lower capability bounds use a cumulative minimum. This
prevents a higher requested speed from producing a less conservative envelope.

Any calibration fall or no-motion result makes that command knot unsupported.
The same applies when a critical metric is null or fewer than six evaluable
calibration trials remain. Unsupported knots are retained in reports and cannot
be silently removed or bridged by interpolation.

## Command Queries

No extrapolation is allowed. At a command between two supported knots, use the
larger adjacent upper bound and smaller adjacent lower bound. Do not linearly
interpolate toward a less adverse value. Do not interpolate across zero between
negative and positive yaw.

Zero is an exact stationary command. A nonzero command below the smallest
calibrated magnitude is mapped to zero by the navigation safety filter. Commands
above the largest calibrated magnitude or adjacent to an unsupported knot are
uncertified and must be rejected or clamped into a supported domain before
rollout.

## Held-Out Coverage

A held-out trial passes only when it is evaluable, has motion, has no fall, has a
stable steady interval and all required drift horizons, and every upper and lower
metric for its family is inside the frozen envelope. This is a joint per-trial
test, not separate metric-wise coverage that could hide multiple-comparison
failures.

For each command family separately, report observed joint coverage and the
one-sided 95% Clopper-Pearson lower confidence bound. Acceptance requires:

```text
lower_confidence_bound >= 0.90
```

At least 29 evaluable held-out trials are required per family. If every trial
passes, 29 trials give the exact lower bound
`0.05^(1/29) = 0.9019`. Five held-out seeds provide 30 planned trials per family,
allowing one infrastructure-invalid trial but no algorithmic coverage failure at
the minimum sample size. Confidence is stated per family; no simultaneous 95%
familywise claim is made.

## Failure Rules

If calibration quality makes a knot unsupported, reduce the deployable command
domain or collect a new protocol version. Do not impute a favorable response.

If held-out coverage fails, reject the envelope. Held-out observations may be
used to diagnose the failure but not to tune and re-test the same artifact. Any
changed estimator, margin, command domain, filter, or policy requires a new
fitting version and new unopened held-out seeds. Falls, no-motion, poor tracking,
and bound violations cannot be reclassified as infrastructure failures.

The response model may be integrated into the navigation supervisor only after
both command families pass, all provenance hashes are complete, and the fitted
artifact and held-out report are immutable.
