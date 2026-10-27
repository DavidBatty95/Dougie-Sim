# Current comparison results — 27 October 2026

[Project home](../README.md) · [Comparison guide](DOUGIE_COMPARISON.md) · [Portable data](results/dougie_comparison_20261027.json)

## Protocol

Three paired development smoke runs, one pair per world, 60 s configured cap. Not full exploration or a matched-coverage dose-reduction study.

All pairs use detector seed 1, sensor seed 0, layout seed 0, a 2 µSv dose budget, a 100 µSv/h rate limit, fused localization, and fixed 0.5 s simulated planning latency. The 60 s cap includes return reserve; runs can finish earlier.

Coverage uses `physically_reached_3d_floor_v2`. All six outcomes are included. No curve is extended past the end of its run. These results verify comparison integration and coverage semantics; they do not establish a radiation-awareness advantage at matched work.

## Per-run results

| World | Mode | End (s) | Dose (µSv) | Explored (%) | Path (m) | End state / survey outcome |
|---|---|---:|---:|---:|---:|---|
| reactor | blind | 19.025 | 0.050949 | 42.15 | 2.487 | returned / time_limit |
| reactor | aware | 25.040 | 0.072218 | 43.64 | 2.655 | returned / time_limit |
| mezzanine | blind | 17.020 | 0.004635 | 34.73 | 1.805 | returned / time_limit |
| mezzanine | aware | 13.015 | 0.003433 | 34.62 | 1.256 | returned / time_limit |
| facility | blind | 41.055 | 0.007024 | 14.25 | 2.449 | returned / time_limit |
| facility | aware | 19.025 | 0.003365 | 13.21 | 2.065 | returned / time_limit |

![Dose and exploration traces](media/dougie/comparison-results.svg)

![Per-level exploration](media/dougie/comparison-levels.svg)

## Verification and provenance

`./dougie test`: **355 passed** on 27 October 2026 (full local Go2 suite). This is not a new remote-CI or field-hardware result.

The portable JSON retains time-series samples, configurations, outcomes, slice counts and SHA-256 identities of raw inputs. The media manifest records the exported JSON and image hashes. The preview is a composition of actual final renders, not a browser screenshot or a fabricated trajectory.

Source comparisons:

- `comparison_20261027T164147_444153Z` — reactor
- `comparison_20261027T153520_048381Z` — mezzanine
- `comparison_20261027T153520_239161Z` — facility

Regenerate both SVG graphs and this report from the committed portable data:

```bash
docker compose run --rm -T dougie-sim python3 scripts/publish_dougie_comparison.py
```

To import fresh data and regenerate the preview, add `--runs` followed by exactly one complete comparison directory for each world. The publisher rejects old coverage metrics and mismatched protocol settings. See the script for the declared cohort.
