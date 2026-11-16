# Portable Dougie results

[Project home](../../README.md) · [Documentation](../DOUGIE_INDEX.md) · [Current comparison](../DOUGIE_COMPARISON_RESULTS.md)

This directory contains compact Go2 evidence referenced by maintained guides,
reproduction tools, calibration artifacts and regression tests. Raw simulation
outputs belong in the ignored root `results/dougie/` directory.

| Start here | Scope |
|---|---|
| [Retained reactor comparison](dougie_frontier_comparison.json) | Earlier single-pair global-GP result, matched milestones and incomplete outcomes |
| [Recent policy pilots](dougie_policy_pilots_20261110.md) | Censored and invalid revisions alongside matched milestone statistics |
| [Campaign checks](dougie_campaign_checks_20261114.json) | Simulated lifetime, recovery and persistent-wreck checks |
| [Multilevel holdout](dougie_holdout_20261115.json) | Retained campaigns, errors, incomplete outcomes and reference milestones |
| [Historical smoke cohort](dougie_comparison_20261027.json) | Six earlier blind/aware smoke runs under a different protocol |
| [Recorded mezzanine round trip](dougie_loop_budget_baseline_20261026.json) | Explicit 8° sensor-driven navigation milestone |
| [Terrain qualification](dougie_terrain_qualification_20261007.json) | Contact-aware scored courses and repeatability |
| [Support replay inputs](support_replay_20261024/) | Retained inputs for reproducible mapping diagnosis |

Dated reports remain when they substantiate a published finding, including
negative results and comparators. Their metrics and configuration scopes are not
interchangeable with the current comparison's physical-arrival coverage metric.

When publishing a new report, retain the complete cohort, seeds, source hashes,
metric definitions and outcomes. Use the corresponding publisher/checker in
[`scripts/`](../../scripts/README.md). The old G1 avoidance/transport version series
is available through [repository history](../DOUGIE_HISTORY.md).
