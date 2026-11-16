# Experimental 3-D campaign agenda

`--campaign-strategy agenda` is an opt-in development policy. The default
`baseline` campaign is unchanged. This policy persists observed 3-D region
attempts, newly mapped support counts, transitions between spatial/height bins,
and marginal unknown-view signatures across replacement robots. It receives no
evaluator floor labels, true radiation or hidden world geometry. Old campaign
memory without these fields remains readable.

Candidate goals still pass existing support, return, radiation exclusion and
survival-dose admission. Region scoring penalizes repeated unproductive
attempts and considers estimated round-trip dose. A bounded two-frontier graph
lookahead uses distinct observed unknown cells, predicted upper exposure and a
return reserve. Only the first already-admitted goal is dispatched; a future
goal must pass fresh observations and admission. The lookahead is advisory,
not a continuous-motion or probabilistic safety guarantee. A new guard confines
second views to the same 3-D region and prevents a tour from overriding a much
better immediate goal. No evaluator coverage is used by the planner.

The persisted attempt productivity is a **global newly mapped support proxy**,
not a proven causal attribution to the assigned region. Connector transitions
are coarse observed region changes, not yet a robust directed connector model
with calibrated traversal dose. The optional marginal-view policy can change
which unresolved frontiers are considered productive; unresolved frontiers stay
visible and do not become planner-completed by being deprioritized.

## Development screens: retain negative outcomes

These are single detector seeds on fixed layouts, not independent environment
validation. Each comparison uses a separate blind/aware physical campaign.

| Revision / scenario | Baseline aware | Experimental aware | Outcome |
|---|---|---|---|
| Agenda only, reactor seed 201, two sorties | 51.93% / 1.044 µSv | 52.07% / 0.811 µSv | Both recovered two robots; small development signal only |
| Agenda + unguarded two-goal lookahead, reactor seed 202, two sorties | 52.13% / 1.332 µSv | 51.62% / 0.698 µSv | Both recovered two; lower dose with less exploration, not equal-work improvement |
| Same unguarded revision, facility seed 203, one sortie, 600 simulated seconds cap | 51.47% / 0.425 µSv | 29.00% / 0.128 µSv | Both returned; neither reached upper landing. **Material exploration regression** |
| Guarded lookahead, facility seed 204, one sortie, 600 simulated seconds cap | 45.08% / 0.644 µSv | 45.78% / 0.358 µSv | Both returned; neither reached upper landing. One fixed-layout seed; not validation of multilevel access |

Facility tests have an explicit simulated-time cap, not a frontier-completion
result. Both guarded robots only physically reached the ground-floor reference
height; their return reasons differed. One screen cannot establish general
performance, and the earlier regression remains visible. No revision is promoted
solely on these results.

Raw data and per-run source hashes are retained under
`results/dougie/campaign_agenda_{baseline,pilot}_201/`,
`results/dougie/campaign_tour_{baseline,pilot}_202/` and
`results/dougie/campaign_tour_facility_{baseline,pilot}_203/` and
`results/dougie/campaign_guard_facility_{baseline,pilot}_204/`.

## Remaining work

The latest experimental revision additionally records **directional** coarse
3-D edge time and detector-count dose rates between nearby valid detector windows.
Pending experience is committed only when the robot returns alive; it is retained
across subsequent replacements without replaying the measurements. Two observed
transitions are required before use. A sample mean plus twice the sample standard
deviation is a conservative empirical heuristic, **not a calibrated confidence
bound**. Route search and graph forecasts use the greater of the empirical cost
and the existing model forecast; experience cannot discount predicted dose or
waive support, obstacle, hotspot or return-reserve checks. The blind team only
receives its sanitized zero-radiation measurements.

When no ordinary admissible frontier/connector remains, the aware policy may
consider up to eight distinct observed-support stand-offs within 1.25 m of an
uncertain budget/radiation-blocked frontier. They must be on the same height
band, locally unmeasured and pass the normal outbound, return and radiation
admission. Such a probe is a request for new observations, not an explored
frontier, safe-route certificate or completion claim. The blind ablation does
not receive radiation-driven probes.

In a two-sortie reactor seed-205 development screen, the baseline aware team
observed **50.86% / 1.123 µSv**, and the experimental aware team **51.71% /
0.861 µSv**; both recovered two robots. The second experimental sortie loaded
36 learned directed edges from the first. **No uncertainty probe was selected**,
so this run does not test whether probing improves outcomes. Raw data is under
`results/dougie/campaign_experience_{baseline,pilot}_205/`.

In a two-sortie **facility** seed-206 screen (600 simulated-second cap per
sortie), the baseline aware team reached **58.06% at 0.276 µSv**, while the
experimental team reached **54.67% at 0.466 µSv**. Both recovered two robots;
neither physically reached the upper landing. The second experimental sortie
loaded 65 directed edges from the first; **no uncertainty probe was selected**.
This is a negative development result: more dose and less exploration, with
incomplete missions. Raw traces are under
`results/dougie/campaign_experience_facility_{baseline,pilot}_206/`.

At the first ≥50% crossing, cumulative aware dose was **0.1835 µSv baseline
versus 0.1715 µSv experimental** (actual observation 50.30% / 50.01%). At ≥54%,
it was **0.1898 versus 0.2314 µSv** (actual 54.05% / 54.09%). In the second
sortie, the recorded trace had 17 versus 27 target changes and 6 versus 12
`route_tracking_lost` planning records, respectively. These counts are
descriptive, not independent recovery episodes or proof of causation. Both
robots returned under the simulated-time reserve. The late-stage exploration
and route-tracking regression needs investigation before any policy promotion.

A goal-switch hysteresis was separately screened on the same facility seed 206
and two-sortie protocol. It never activated, and the experimental outcome
remained **54.67% / 0.466 µSv** versus **58.06% / 0.276 µSv** baseline, with
six versus three distinct second-sortie tracking-loss episodes. Neither aware
robot reached the upper landing. The ineffective hysteresis was removed; raw
records remain under `results/dougie/campaign_stability_facility_{baseline,pilot}_206/`.

A separate 0.25 µSv-equivalent reactor stress package with a 1 µSv/h exclusion
threshold produced one probe candidate but rejected its execution route; no
probe was visited. Raw records remain under `results/dougie/campaign_probe_physics_250/`.

The [controlled physical probe fixture](results/dougie_probe_physics_20261114.json)
deliberately injects a blocked-frontier classification to exercise the rare
branch. The selected stand-off and support come from real range sensing; the Go2
moves under MuJoCo physics and receives real Poisson detector counts. The robot
reached within 25 cm, received 10 detector windows, and its local GP log-rate
standard deviation decreased from 1.033 to 0.720. The artificial blocked status
makes this a functional intervention, not a naturally occurring campaign event,
navigation efficacy comparison or calibrated safety result. Reproduce inside the
runtime with:

```bash
python3 scripts/check_probe_physics.py --output /tmp/dougie-probe-check.json
```

An uncapped facility seed-208 screen was externally interrupted after both aware
robots crossed 70% to permit the next source revision. **Both physically reached
the upper landing**: baseline at 518.95 simulated seconds and experimental at
659.01 seconds. At the first ≥70% crossing, actual observation was 70.10% /
70.20% and cumulative dose was **0.18204 / 0.27288 µSv** respectively. The
experimental robot used more dose at that matched milestone. The interrupted
endpoints (80.55% and 71.50%) are censored, not completion or equal-work dose
results. Records: `results/dougie/campaign_uncapped_facility_{baseline,pilot}_208/`.

The live seed-251 reactor stress run (0.5 µSv robot lifetime, 5 µSv/h exclusion)
selected a natural uncertainty probe but abandoned it 0.388 m from its goal after
new nearby evidence made it fail the initial "unmeasured" filter. The updated
policy revalidates an already admitted probe while its blocked target remains
uncertain; it never overrides dose, radiation, route or time checks. In the
matched repeat, the robot came within **0.032 m** of the original probe at
6.5 s, then returned with unresolved frontiers. Endpoint observation was 41.50%
and dose 0.07290 µSv, versus 40.68% and 0.01225 µSv in the earlier run. That is
more exploration **and** more dose, not evidence of better efficiency. Raw
records: `results/dougie/campaign_probe_v3_251/` and
`results/dougie/campaign_probe_commitment_fixed_251/`.

The probe selector now considers all nearby height-compatible uncertain targets
rather than only the nearest, which could be ineligible even when a suitable
target was nearby. A full suite exposed an uninitialized local variable in the
first implementation affecting non-survey objectives; that bug was fixed and the
entire suite rerun. The corresponding partial facility screens were stopped and
are retained as interrupted under `campaign_commit_facility_{baseline,pilot}_209`.

A second-layout facility screen (`layout_seed=1`, detector seed 207, one sortie,
600 simulated-second cap) produced **44.11% / 0.1388 µSv baseline aware** and
**47.14% / 0.2013 µSv experimental aware**; both robots returned without an
upper-landing witness. At the common ≥40% crossing, dose was **0.06589 versus
0.07722 µSv**, with actual observation 40.16% versus 40.03%. This is more area
at the endpoint but more dose at matched progress, not an unqualified win. Raw
results are under `results/dougie/campaign_layout_{baseline,pilot}_l1_207/`.
This is one sortie on one additional layout, not a multi-layout statistical
holdout or successful multilevel qualification.

Observed region transitions are coarse 3-D bins, not verified directional
connector certifications. The revision has not yet demonstrated reliable
upper-floor access or passed multi-layout, multi-seed, multi-sortie facility
holdouts. A natural probe was visited in one deliberately tight-budget stress
case, but its overall efficacy remains unestablished. These remain necessary
before default promotion.

The subsequent bounded, spatially diverse budget-frontier screening revision
was screened on facility layout 2 / seed 212 without a simulated-time cap. At
the first ≥70% crossing, baseline aware accumulated **0.28657 µSv at 70.04%**;
experimental aware accumulated **0.26657 µSv at 70.38%**. The baseline aware
robot returned at 80.63% / 0.42108 µSv. The experimental aware robot had reached
the upper landing and 87.00% / 0.41495 µSv when the paired diagnostic was
**externally interrupted during return** to start occupancy-index development.
Both packages are censored; the experimental endpoint is not a recovered robot
or an equal-work dose comparison. Neither mission declared frontier completion.
Raw records: `results/dougie/campaign_screen_{baseline,pilot}_l2_s212/`.

For a new test session:

```bash
DOUGIE_COMPARE_CAMPAIGN_STRATEGY=agenda ./demo
```

Use a new campaign after changing runtime code: campaign protocol hashes guard
against mixing source revisions. Compare actual coverage, floor witnesses,
losses, dose and completion; endpoint dose after less exploration is not a win.

## Uncapped multi-layout development holdout

A later experimental revision was screened against the baseline using
facility layout seeds **1 and 2**, detector seeds **210 and 212** respectively,
two sorties per policy, 2 µSv-equivalent robot lifetimes and **no simulated-time
cap**. Both strategies run a matching blind and aware mission. These two layouts
are a small development holdout, not a statistically robust generalization or
hardware claim. Each second sortie uses its campaign's next detector seed and
retains only that policy's own map, radiation knowledge and wrecks.

### Completed holdout and return-route development

The initial holdout packages and baseline retries have now ended. The complete
[retained report](results/dougie_holdout_20261115.json) includes both failed
original baselines, both retries and both experimental packages. On layout 1,
the experimental aware team recovered both robots and observed **90.82% at
0.842 µSv**, versus **87.41% at 1.071 µSv** for the clean baseline retry. On
layout 2, the experimental package halted after its blind worker had a process
pool error; its aware robot returned at **64.28%**, while the clean baseline
retry reached **80.76%**. All packages remained incomplete: a sortie limit or
halt is censored for frontier completion. No natural probe was selected in the
reported first sorties. This is two layouts and not a generalization claim.

In the layout-2 experimental first sortie, the aware planner began returning
with a **1.116 µSv upper return forecast**. Actual dose accrued during that
return was approximately **0.320 µSv**. These are different model/evaluator
quantities, not proof that a lower bound would have been safe. The experimental
planner now compares its original, geometric and upper-dose-weighted observed
return paths using the **same GP upper exposure estimates** and current-dose
return reserve. A lower predicted-upper-dose alternative can be chosen without
changing the lifetime budget, GP bounds, exclusion threshold or physical
admission. Candidate routes are re-evaluated by the existing smoothing contract;
any recovered-route empirical cost can only increase the forecast. This is a
model-based choice, not a guarantee of lower true dose.

The first return-route screen was **interrupted and invalidated** after review
found that execution might re-smooth a different raw path from the one whose
upper dose had been evaluated. Both `campaign_return_{baseline,pilot}_l2_s212`
directories retain `validation_notes.json` and must not be used for efficacy or
safety claims. The corrected planner hands the exact evaluated `MotionRoute` to
the worker only while its raw path still matches; a recovery stop or replacement
invalidates that certificate. Focused tests cover both ordinary return and the
completed-survey return branch.

The corrected revision is screened separately on **layout 2 / detector seed
212**, one uncapped sortie per strategy, at
`results/dougie/campaign_return_v2_{baseline,pilot}_l2_s212/`. Its retained
report is available at <http://localhost:8778> while the local report service is
running. This is a targeted
development repeat of a known difficult case, not a fresh independent holdout.
Do not promote the strategy or claim improved survival/coverage without its
actual return outcome and further unseen-seed/layout tests.

Both corrected-screen aware workers have now returned alive. The experimental
aware robot ended at **64.06% / 0.55090 µSv**, with 864 unresolved frontiers;
baseline ended at **80.63% / 0.42108 µSv**, with 269 unresolved frontiers.
Both new aware summaries have confirmed upper-landing witnesses. The experimental robot's
first return decision had upper-dose estimates **1.120 µSv** for the original
route and **0.878 µSv** for its chosen upper-dose alternative, but the resulting
actual endpoint was worse on dose and exploration than baseline. This does not
isolate the return-route change from the rest of the experimental agenda policy.
Both packages later reached their configured one-sortie limit: the baseline blind
robot returned, while the experimental blind robot was destroyed by radiation.
Neither aware mission completed its frontier survey. This is a negative targeted
development outcome, not a basis for default promotion.

While the local monitoring service is running, see
<http://localhost:8777> for active and finalized coverage, dose, recoveries,
losses, upper-landing witnesses and selected/reached probes. Its JSON is also
written to `results/dougie/holdout_progress.json`. The four raw directories are
`results/dougie/holdout_l1_s210_{baseline,agenda}/` and
`results/dougie/holdout_l2_s212_{baseline,agenda}/`. Source/protocol hashes,
individual outcomes, actual milestone overshoots and interrupted attempts
remain retained. A package ending at its sortie count is not frontier completion.

During the four-way concurrent run, the layout-1 baseline's second blind worker
exited with signal 9, and the layout-2 baseline's second aware worker reported
an abruptly terminated process pool. Their exact causes were not established.
Both affected packages were then stopped and remain **halted/censored**; their
partial data is not overwritten or counted as clean completion. Same-seed baseline retries were run at
`results/dougie/holdout_l1_s210_baseline_retry1/` and
`results/dougie/holdout_l2_s212_baseline_retry1/` after stopping the failed
packages. The live report lists the failed originals and retries separately;
failures must not be silently removed from the final interpretation.

The [final retained report](results/dougie_holdout_20261115.json) includes those
failed originals, both retries and both experimental packages. In the first
sortie, both aware strategies on both layouts had an upper-landing witness and
returned with unresolved frontiers; no natural uncertainty probe was selected.
The layout-2 experimental package also halted after its blind worker reported a
process-pool error. No package demonstrated complete frontier exhaustion.

The report and monitor can be reproduced with:

```bash
python3 scripts/report_campaign_holdout.py --studies \
  results/dougie/holdout_l1_s210_baseline results/dougie/holdout_l1_s210_agenda \
  results/dougie/holdout_l2_s212_baseline results/dougie/holdout_l2_s212_agenda \
  results/dougie/holdout_l1_s210_baseline_retry1 results/dougie/holdout_l2_s212_baseline_retry1 \
  --output results/dougie/holdout_progress.json
```

If a package is still active or halts unexpectedly, the report marks it censored;
it must not be interpreted as a completed mission. An uncapped run can take a
long time. Do not edit runtime sources while the cohort runs: campaign protocol
fingerprints prevent a later sortie from mixing source revisions.
