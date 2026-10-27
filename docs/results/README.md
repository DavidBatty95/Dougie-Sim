# Portable Dougie results

[Project home](../../README.md) · [Documentation](../DOUGIE_INDEX.md) · [Current comparison](../DOUGIE_COMPARISON_RESULTS.md)

This directory contains compact Go2 evidence referenced by maintained guides,
reproduction tools, calibration artifacts and regression tests. Raw simulation
outputs belong in the ignored root `results/dougie/` directory.

| Start here | Scope |
|---|---|
| [Current comparison cohort](dougie_comparison_20261027.json) | All six blind/aware smoke runs, 3D coverage, configurations and source identities |
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
