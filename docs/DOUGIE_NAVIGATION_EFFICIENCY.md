# Navigation efficiency development

## Stage 1: repeated work

Branch: `improve-navigation-efficiency`. The existing planning update was first
verified with 190 passing tests and committed as `e60d168`.

Changes reuse smoothing geometry and exact checked routes within one planning
cycle, skip live display predictions in headless non-ROS runs (the final map is
still archived), and invalidate layered support-footprint checks only around
changed candidate surfaces. Global spatial preprocessing and adjacency generation
remain; this is not a fully incremental mapper.

Verification: 188 tests passed initially; four archive checks detected a missing
headless final map. After restoring final-map generation, all 36 affected
mapping/motion/archive checks passed. The other 156 checks had already passed.

Matched reactor development runs: seed 3, 180 s limit, 2 uSv, fixed .5 s plan delay,
smooth/global defaults, clean-return gate. Run folders under `results/dougie/`:
`navigation_baseline_reactor` and `navigation_cached_reactor`.

Both returned at 131.220 s, travelled 15.6059 m, observed 123.875 m² surface proxy,
used 0.346027 uSv and had no non-foot contacts. Execution-smoothing p95 fell from
15.11 to 5.57 ms; total worker p95 increased from 424.79 to 463.84 ms. Wall times
were 44.74 and 43.76 s. One matched run establishes unchanged trajectory, not a
general latency improvement. Fixed scheduling is not paced real-time evidence.

Reproduce either revision with:

```bash
docker compose run --rm -T dougie-sim python3 -m dougie_sim.radiation_run \
  --world reactor --headless --fast --duration 180 --dose-budget 2 --seed 3 \
  --fixed-plan-delay .5 --require-clean
docker compose run --rm -T dougie-sim python3 -m pytest tests/dougie -q
```
