# Planned-stop deceleration screening — 26 October 2026

[Project home](../README.md) · [Traction baseline](DOUGIE_TRACTION_SCREEN.md) · [Active plan](DOUGIE_ONBOARD_PLAN.md)

A two-second command taper improves some simulation outcomes, but fails fresh-seed
validation as a general stopping remedy. It remains a harness experiment. The
production mission controller and its terminal-entry gates are unchanged.

## Experiment

The approach command decreases linearly from .15 m/s to zero over the final one
or two seconds **before** the prescribed stop at 13 s. Commands remain zero after
that request; the production `GravityStanceHold` then attempts its existing quiet
entry. This requires advance knowledge of the stop time. It is not reactive
emergency braking, measured-velocity feedback, or a certified physical deceleration.

Each cohort includes flat, 8.53° uphill and downhill fixtures, effective friction
.4/.25, and paired zero-force / 15 N lateral pushes for .5 s. Post-stop scoring
continues for 45 s. The .05 m drift gate, contact gates, motor limit and .2 s quiet
dwell/3 s entry deadline are unchanged. Four new tests verify bounded monotone
commands, zero commands after terminal request and the original step comparator;
the full suite passes **324 tests**.

## Results, including regressions

| Cohort | Step-stop baseline | Taper | Candidate passes | Recovered / regressed cases |
|---|---:|---:|---:|---:|
| Development seeds 0, 1 | 18/24 | 1 s | 18/24 | 2 / 2 |
| Development seeds 0, 1 | 18/24 | 2 s | 22/24 | 4 / 0 |
| Fresh seeds 2, 3 | 17/24 | 2 s | 19/24 | 3 / 1 |

The 1 s taper introduces two uphill push failures at friction .4. The 2 s
development result retains two downhill entry failures at .25. On fresh seeds,
all four low-friction downhill cases still fail entry, with drift up to 1.625 m;
one .4 uphill push case loses supported stance and drifts .452 m. The fresh-seed
comparison includes a newly failing case, even though its aggregate count improves.
All cases pass the non-foot-contact gates. Passing those gates does not establish
stationary stopping.

### Reproducible evidence

The development implementation is frozen in `cb64d77`; fresh seed selection is
declared in `4bc21ce`. All comparisons verify identical runtime source hashes,
scene/policy identity and sampled physical motion before the taper begins.
Each cohort audit independently checks completeness, full-duration scoring,
effective contact friction, applied impulse, torque limit and matched pre-push
physics. Raw reports preserve noisy entry measurements and all failed trials.

| Cohort | Raw report | Audit | Matched comparison |
|---|---|---|---|
| 1 s development | [24 cases](results/dougie_brake_one_20261026.json) | [Audit](results/dougie_brake_one_audit_20261026.json) | [2 gains, 2 regressions](results/dougie_brake_one_comparison_20261026.json) |
| 2 s development | [24 cases](results/dougie_brake_two_20261026.json) | [Audit](results/dougie_brake_two_audit_20261026.json) | [4 gains](results/dougie_brake_two_comparison_20261026.json) |
| Fresh step comparator | [24 cases](results/dougie_brake_holdout_control_20261026.json) | [Audit](results/dougie_brake_holdout_control_audit_20261026.json) | — |
| Fresh 2 s taper | [24 cases](results/dougie_brake_holdout_20261026.json) | [Audit](results/dougie_brake_holdout_audit_20261026.json) | [3 gains, 1 regression](results/dougie_brake_holdout_comparison_20261026.json) |

```bash
docker compose run --rm -T dougie-sim python3 scripts/qualify_dougie_stance_disturbance.py \
  --duration 45 --brake-seconds 2 --sensor-seeds 2 3 --output /tmp/planned-stop.json
python3 scripts/check_dougie_stance_disturbance.py \
  --report docs/results/dougie_brake_holdout_20261026.json --output /tmp/planned-stop-audit.json
python3 scripts/compare_dougie_planned_stop.py \
  --baseline docs/results/dougie_brake_holdout_control_20261026.json \
  --candidate docs/results/dougie_brake_holdout_20261026.json \
  --output /tmp/planned-stop-comparison.json
```

## Measured-yaw feedback follow-up

The [entry diagnosis](results/dougie_reserve_entry_20261026.json) found adequate
foot forces and joint speeds in the failed downhill fixtures, but gyro magnitude
remained above the .15 rad/s quiet threshold. A
[component probe](results/dougie_settling_gyro_probe_20261026.json) measured gyro
RMS components of .0659/.0471/.1477 rad/s: yaw was dominant. The threshold was
not relaxed.

An opt-in harness feedback experiment now damps measured body yaw during the
two-second **pre-terminal** taper: gain 1, command magnitude at most .2 rad/s,
active command slew at most .5 rad/s². Stale/nonfinite observations, insufficient
foot support or excessive gyro inhibit the feedback. Commands remain zero after
the terminal request. The posture-gated version additionally requires the
estimated normalized body-up vector's forward component to be below -.05. This
uses estimated attitude, not the fixture's uphill/downhill label or simulator pose.

| Feedback experiment | Seeds | Taper-only passes | Feedback passes | Recovered / regressed |
|---|---|---:|---:|---:|
| All postures | 0, 1 | 22/24 | 20/24 | 2 / 4 |
| Estimated downhill posture | 0, 1 | 22/24 | 22/24 | 2 / 2 |
| Estimated downhill posture, retained failures | 2, 3 | 19/24 | 21/24 | 2 / 0 |
| Estimated downhill posture, fresh seeds | 4, 5 | 18/24 | 20/24 | 2 / 0 |

These are **120 additional trials**, including the new fresh-seed comparator.
Seeds 2/3 were used in diagnosis and are explicitly not a fresh holdout here.
On fresh seeds, worst drift at friction .25 fell from **1.640533 m to .069365 m**.
All four fresh low-friction downhill cases entered and retained stance; both
pushed cases still exceeded the .05 m drift gate (.060323/.069365 m). Existing
uphill push failures at .4 remain. On development seeds, gating exchanged which
downhill seed failed entry, so equal aggregate counts conceal two regressions.
All cohort contact gates passed. Neither feedback version is enabled in missions.

The all-posture implementation is frozen in `4a49363`, and the estimated-posture
gate in `e346913`. The current full suite passes **338 tests**, including feedback
limits, fault inhibition, posture gating and the separate loop-history option.

| Cohort | Raw report | Audit | Matched comparison |
|---|---|---|---|
| All postures | [Report](results/dougie_yaw_settling_20261026.json) | [Audit](results/dougie_yaw_settling_audit_20261026.json) | [Comparison](results/dougie_yaw_settling_comparison_20261026.json) |
| Gated development | [Report](results/dougie_yaw_downhill_20261026.json) | [Audit](results/dougie_yaw_downhill_audit_20261026.json) | [Comparison](results/dougie_yaw_downhill_comparison_20261026.json) |
| Gated retained failures | [Report](results/dougie_yaw_retained_20261026.json) | [Audit](results/dougie_yaw_retained_audit_20261026.json) | [Comparison](results/dougie_yaw_retained_comparison_20261026.json) |
| Fresh taper comparator | [Report](results/dougie_yaw_fresh_control_20261026.json) | [Audit](results/dougie_yaw_fresh_control_audit_20261026.json) | — |
| Gated fresh seeds | [Report](results/dougie_yaw_fresh_20261026.json) | [Audit](results/dougie_yaw_fresh_audit_20261026.json) | [Comparison](results/dougie_yaw_fresh_comparison_20261026.json) |

```bash
docker compose run --rm -T dougie-sim python3 scripts/qualify_dougie_stance_disturbance.py \
  --duration 45 --brake-seconds 2 --settling-yaw-gain 1 --settling-yaw-mode downhill \
  --sensor-seeds 4 5 --output /tmp/yaw-settling.json
```

Remaining work is reliable support acquisition across the retained failed seeds,
then resistance to pushes after entry. Lower gyro at one instant is insufficient:
the full quiet dwell, joint-speed limits and physical drift gates remain mandatory.
