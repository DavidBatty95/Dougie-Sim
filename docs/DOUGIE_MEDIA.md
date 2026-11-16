# Media and reproducible figures

[Project home](../README.md) · [Documentation](DOUGIE_INDEX.md) ·
[Retained comparison](DOUGIE_COMPARISON_RESULTS.md)

| Asset | Meaning |
|---|---|
| [Browser screenshot](media/dougie/frontier-dashboard.png) | Actual retained reactor comparison, not a mockup |
| [Dose and exploration curves](media/dougie/frontier-results.svg) | Earlier single-pair, ideal-pose experiment; both surveys incomplete |
| [Truth and GP field](media/dougie/frontier-radiation.png) | Earlier global-GP field snapshot, not a current windowed-GP qualification |
| [Figure manifest](media/dougie/frontier-manifest.json) | Retained input and output identities |

Regenerate or verify the published figures from their retained inputs:

```bash
docker compose run --rm -T dougie-sim python3 scripts/publish_frontier_comparison.py --check
docker compose run --rm -T dougie-sim python3 scripts/publish_frontier_comparison.py
```

The current browser page streams actual independent MuJoCo renders. A completed
run can be replayed with `./dougie-mac replay results/dougie/comparison_<timestamp>`.
The optional `scripts/check_comparison_browser.mjs` checks actual feeds, plots,
labels, field maps and mobile layout. It can capture a screenshot with
`--screenshot <path>`.

Older recorded GIFs, manifests and portable inputs remain available in the
repository; their dated descriptions and regeneration commands are in the
[Go2 development archive](DOUGIE_HISTORY.md#go2-development-archive). They are
historical evidence, not demonstrations of current campaign completion.
