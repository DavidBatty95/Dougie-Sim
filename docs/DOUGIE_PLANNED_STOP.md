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

The next controller investigation needs measured settling/support dynamics rather
than further tuning a fixed stopping time against these same seeds. Preserve the
fresh-seed regressions when evaluating any future transition controller.
