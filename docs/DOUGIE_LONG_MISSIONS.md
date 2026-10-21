# Five-times dose-allowance experiment

This page retains the earlier navigation cohort. The subsequent
[robust-navigation stages and revised default checks](DOUGIE_ROBUST_MISSIONS.md)
add quality-gated localization, consistent map corrections, bounded recovery and
tracking clearance. Use `--navigation-profile legacy --utility-mode legacy` to
select the earlier comparator with current code; source identities remain in the
original report.

[Project home](../README.md) · [Onboard methods](DOUGIE_ONBOARD_MISSION.md) ·
[Portable paired report](results/dougie_budget_5x_600.json)

## Protocol

The requested larger allowance is **10 µSv**, five times the current **2 µSv**
mission budget. The experiment uses a **600 s simulated-time cap** for both
budgets, compared with the previous short 90 s development trials. Keeping the
time cap equal isolates the budget effect within each seed pair.

- Reactor, fused IMU/leg/LiDAR localization, observed-geometry GP, moving-window
  Poisson counts and smooth path following.
- Detector/sensor seed pairs **3/0 and 4/1**, each at **2 and 10 µSv**.
- Identical **100 µSv/h** rate ceiling and **0.5 s fixed simulated plan delay**.
- The clean-return gate checks budget, contacts and physical home distance.
- Runs execute sequentially; failures and logs are retained. Fixed scheduling
  isolates trajectory comparisons but does not establish paced performance.

## Efficiency measures

The report scores unique **0.5 m × 0.5 m physical XY visitation cells** from the
independent evaluator trajectory. It reports cells per actual µSv and cells per
simulated minute, progression through the mission, path length, actual dose,
return outcome, localization error and worker latency.

Visitation measures where the robot travelled, not the complete area surveyed or
radiation-map reconstruction quality. It does not give credit for estimator
drift or noisy duplicate support-height cells. Evaluator positions are never
inputs to planning. A failed return or lower dose achieved by doing less work
cannot be interpreted as an efficiency improvement.

## Measured outcomes

All four runs completed evidence collection under `results/dougie/onboard_budget_5x_600`.
The matrix exited **1**, correctly preserving the failed clean-return gate.

| Detector/sensor seed | Budget (µSv) | Duration (s) | Actual dose (µSv) | Travel (m) | Visited cells | Cells/µSv | Cells/min | Outcome |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 3/0 | 2 | 466.055 | 1.137097 | 54.37 | 84 | 73.87 | 10.81 | Clean return |
| 3/0 | 10 | 554.275 | 1.509889 | 72.37 | 139 | 92.06 | 15.05 | Clean return |
| 4/1 | 2 | 393.875 | 1.171521 | 48.01 | 85 | 72.56 | 12.95 | Clean return |
| 4/1 | 10 | 600.005 | 1.242306 | 32.15 | 59 | 47.49 | 5.90 | **Did not return** |

All four recorded zero non-foot impulse/peak force. The failed run was still
**7.615 m from home** at the time limit, so it is not a successful low-dose mission.

### What the larger allowance achieved

- **Seed pair 3/0:** 65.5% more physical visitation cells for 32.8% more dose and
  18.9% more time. Visitation per dose improved **24.6%**, and visitation per minute
  improved **39.1%**. Both runs returned, although their completed work differs.
- **Seed pair 4/1:** the 10 µSv run initially visited more cells (55 versus 41 at
  180 s), but ended with fewer cells and no return. It reached 58 cells by 300 s
  and only 59 by 600 s. The final state was
  `waiting_for_fresh_sensors_or_support`; its last decision reason was
  `no_rate_compliant_return_route`.
- The 10 µSv trials used only **12.4–15.1% of their allowance**. Higher admission
  reserves changed routes, but did not remove sensing, support, deadline or
  return-route constraints. A five-times allowance does not imply five-times
  useful mission duration.

### Localization and compute

| Seed / budget | Position RMSE (m) | Position p95 (m) | True home distance (m) | Worker p95 (ms) |
|---|---:|---:|---:|---:|
| 3/0 / 2 µSv | .229 | .372 | .082 | 684.8 |
| 3/0 / 10 µSv | .372 | .644 | .347 | 757.2 |
| 4/1 / 2 µSv | .132 | .200 | .218 | 675.2 |
| 4/1 / 10 µSv | .340 | .395 | 7.615 | 546.5 |

The long runs show substantially larger errors than the short 90 s-limit
screening. Registration and mapped support need stronger long-duration
qualification; the present covariance/drift allowance is not a reliable bound on
these observed errors. The recorded wait state identifies a support/sensing
limitation, but these results do not isolate its full root cause.

**Conclusion:** the higher budget can improve visitation efficiency, but it is
not a consistently reliable longer-mission setting yet. Two seed pairs on one
layout are development evidence, not a statistical reliability estimate or
matched-coverage dose-reduction claim. Prioritize localization consistency,
support-map recovery and stalled-exposure handling before extending duration
further. The comparator defaults and safety gates were not relaxed to obtain passes.

Verification includes two new scoring tests: estimator drift cannot inflate the
physical-visitation metric, and missing/failed runs cannot be counted as success.

## Reproduce

One larger-budget mission:

```bash
./dougie radiation --headless --fast --dose-budget 10 --duration 600 \
  --seed 3 --sensor-seed 0 --fixed-plan-delay .5 --require-clean
```

The paired four-run experiment, with a fresh output directory:

```bash
docker compose run --rm -T dougie-sim python3 scripts/benchmark_dougie_budget.py \
  --duration 600 --seed-pairs 3:0 4:1
```

The harness writes a summary after each trial and exits nonzero if any required
clean-return gate fails. The original 2 µSv setting remains available for the
matched comparison; the larger allowance is selected explicitly per mission.
