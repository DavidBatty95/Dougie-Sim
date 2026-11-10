# Dougie scripts

[Project home](../README.md) · [Documentation](../docs/DOUGIE_INDEX.md) · [Contributing](../CONTRIBUTING.md)

Use [`../dougie`](../dougie) for normal setup, comparison, radiation, walking,
exploration, testing and training commands. `dougie_cli.py` implements that entry point.

| Script family | Purpose |
|---|---|
| `benchmark_dougie_*.py` | GP, sensing, planning, mission and rendering experiments |
| `check_dougie_*.py` | Documentation, artifact, media and outcome verification |
| `diagnose_dougie_*.py`, `probe_dougie_*.py` | Support geometry, registration, contacts and runtime diagnostics |
| `qualify_dougie_*.py` | Declared uncertainty and stance/stop qualification protocols |
| `report_dougie_*.py` | Portable evidence and mission/terrain summaries |
| `render_dougie_*.py`, `publish_dougie_*.py` | Figures and recorded-media publication from retained inputs |
| `setup_dougie_training.py` | Isolated, pinned optional CUDA training environment |

Common checks and publication commands:

```bash
python3 scripts/check_dougie_docs.py
docker compose run --rm -T dougie-sim python3 scripts/publish_frontier_comparison.py --check
docker compose run --rm -T dougie-sim python3 scripts/publish_frontier_comparison.py
node scripts/test_comparison_metrics.cjs
```

The [comparison report](../docs/DOUGIE_COMPARISON_RESULTS.md) and
[media guide](../docs/DOUGIE_MEDIA.md) record reproduction commands. Consult each
script's `--help` and the relevant protocol before running an experiment.

`check_comparison_browser.mjs` optionally uses Playwright to verify the real page,
matched metrics, both videos, rate maps, endpoint labels and mobile layout. Pass
`--report docs/results/dougie_frontier_comparison.json` when serving the retained
experiment, and `--screenshot <path>` to capture the current desktop viewport.
Set `PLAYWRIGHT_MODULE` only if Playwright is installed outside the normal Node
module search path. The runtime container does not require a browser dependency.

Retired G1/transport/avoidance launchers and publishers are retained in
[Git history](../docs/DOUGIE_HISTORY.md), not alongside the current Go2 tooling.
