# Pathwise exposure improvement policy

`--exposure-policy pathwise` separates exploration progress from radiation-based
route choice. It is an experimental uncapped, smooth-execution profile; it does
not promise lower actual dose in every partially observed world.

## Shared progress scheduler

Both blind and aware workers rank frontier/connector goals using observed
geometry, neighbourhood revisit time and the predicted execution time of the
certified geometric route. Radiation is not used to make a low-progress region
look attractive merely because the GP predicts it cold. Region scheduling uses
the same geometric timing. Actual routes can reveal different geometry, so later
goal schedules can still diverge naturally.

The shared frontier calculation follows observed support heights, uses the
layered mapper's retained returns, and distinguishes known drops from unknown
support. A known, unvisited connector can be probed to improve access to existing
frontiers. A probe does not fabricate a frontier or change the completion rule.

## Route improvement gate

For each bounded lookahead goal, the aware planner considers:

1. The certified geometric baseline route.
2. A time-plus-exposure route.
3. An exposure-first route with a small time tie breaker.

The planner evaluates turning, transit and dwell with the GP and the count-only
near-field guard. An alternative must first pass an evidence gate: every evaluated
route point more than **0.15 m** from the nearest geometric-baseline route point
must have GP log-rate standard deviation **≤1.0** and at least **0.25 s** of recorded
neighbourhood exposure observations at detector height. These are local evidence
requirements, not proof that the route is safe. An unmeasured cold prior alone
cannot justify a detour. Diagnostics retain `detour_evidence_supported`,
`novel_route_points` and `unobserved_detour_points` for the selected route.

It deviates from the geometric baseline only if the evidence-supported alternative
predicts **at least 5% lower total exposure** and its policy upper exposure estimate
is **no more than 5% worse**. Otherwise it uses the geometric certificate. If all
radiation inputs are zero, no spurious exposure benefit can justify a detour.

This is a model-based dominance check, not a true-field or calibrated safety
guarantee. A wrong model can still make an unhelpful choice. No true source,
reference coverage or hidden field enters route selection.

## Responsive observation guards

The optional guards use received detector counts only and never edit GP map
values. A lower-confidence local measurement charges turning/dwell when the GP
has not represented a sharp peak. Statistically significant rises over a bounded
recent window fit a directional, short-range warning; unknown distant regions
are not globally inflated. Blind observations are sanitized, so these guards do
not leak the evaluator's real radiation into the ablation.

Pathwise goal commitment uses the geometric route's normalized progress cost;
destination radiation does not independently preempt that commitment. Radiation
alternatives are reevaluated during route refinement, without letting fluctuating
goal-rate predictions repeatedly switch the exploration target. The `responsive`
profile retains its separate whole-route exposure preemption mechanism.
Neighbourhood dwell penalties work across voxel boundaries. A hot viewpoint
making no mapping, positional or angular progress can be temporarily deferred;
necessary turns and clearly cooler escape goals are exempt. This is not a
cumulative dose cutoff, permanent hotspot ban or completion claim.

## Certificate and recovery contract

The execution worker uses the certificate actually evaluated by lookahead. It may
do so **only when the post-recovery path exactly matches the certified raw path**.
If recovery clears or replaces the path, the cached certificate is invalidated.
This preserves the original stop/retreat semantics and geometric constraints.

The earlier `facility_pathwise_validation` pilot exposed a certificate-handoff
regression and is explicitly marked invalid for performance claims in its
`validation_notes.json`. Its raw records are retained. It must not be used as a
successful awareness comparison.

## Validation protocol

The corrected, pre-evidence-gate pilot used detector seeds **14–17**, fixed sensor/layout seed
0, the full-height facility, ideal-pose localization and online windowed GPs. Both
workers have no dose budget or simulated-time cap. Two pairs run concurrently;
a **600-wall-second guard per pair** is external censoring, not robot completion.

All four pairs were censored, with no confirmed frontier completions. At ≥25%
observation, mean dose was 0.0268 µSv blind versus 0.0208 µSv aware; at ≥50%,
it was **0.1382 versus 0.2368 µSv**, respectively. This did not demonstrate reliable
improvement. A separate pre-evidence-gate reactor pilot (seeds 18–19, 300 wall
seconds per pair) reached ≥75% with mean doses **1.3054 versus 0.9469 µSv**, but
both pairs were censored and neither reached ≥90% or frontier completion.

The evidence-gated revision is evaluated separately on fresh facility detector
seeds **20–23**, with the same fixed layout/sensor seeds, two concurrent pairs
and a 600-wall-second guard. Its output directory is
`results/dougie/facility_pathwise_evidence_validation`. Results from different
revisions and seed sets are development evidence, not a paired before/after test.

This latest facility pilot also failed to establish lower dose: at ≥50%
observation, blind mean dose was **0.138199 µSv** and aware mean dose was
**0.147520 µSv** (four eligible pairs, approximately **6.7% higher** for awareness).
The paired aware-minus-blind mean was 0.009321 µSv, with a percentile-bootstrap
95% mean interval of [0.000000, 0.018642] µSv. All four pairs were censored;
neither mode completed exploration or reached ≥75%. Endpoint observation was
55.9575% for both modes, with no upper-landing physical witnesses and no process,
missing-summary or artifact errors. The smaller penalty than the previous pilot
is not a causal improvement estimate because seeds and revision both changed.

See the [complete retained pilot table](results/dougie_policy_pilots_20261110.md)
and [portable statistics and input hashes](results/dougie_policy_pilots_20261110.json).
`responsive` remains the default; these results do not qualify `pathwise` for a
performance claim or default promotion.

```bash
export COMPOSE_PROJECT_NAME=dougie-pathwise-evidence-validation
export DOUGIE_BATCH_OUTPUT=results/dougie/facility_pathwise_evidence_validation
export DOUGIE_BATCH_SEEDS=4 DOUGIE_BATCH_SEED_START=20
export DOUGIE_BATCH_WORKERS=2 DOUGIE_BATCH_WALL_TIMEOUT=600
export DOUGIE_BATCH_EXPOSURE_POLICY=pathwise
./dougie-mac batch
./dougie-mac batch-report
```

All requested cases, censoring and failures stay visible. Effectiveness is judged
using paired dose at common observation milestones together with physical
observation, floor-level witnesses and frontier completion. Endpoint dose alone
does not establish better exploration. Four counting-noise repeats on a fixed
layout are a development pilot, not a 30-seed or multi-environment claim.

For a live run:

```bash
DOUGIE_COMPARE_WORLD=facility DOUGIE_COMPARE_EXPOSURE_POLICY=pathwise ./demo
```

Use a new output directory after runtime/source changes. See
[multi-seed studies](DOUGIE_SEED_STUDY.md), [online windowed GPs](DOUGIE_WINDOWED_GP.md)
and the [retained policy pilot table](results/dougie_policy_pilots_20261110.md).
The preceding standard/responsive pilots used fresh detector seeds 2–9 and
external wall guards; their negative and censored results remain in that table.
