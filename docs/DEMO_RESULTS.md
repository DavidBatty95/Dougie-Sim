# Published demo evidence

The portable source-linked records are in [demo_results.json](results/demo_results.json).
The newer multi-world cohort is documented in [avoidance experiments](AVOIDANCE_EXPERIMENTS.md)
and published separately as [avoidance_dataset.json](results/avoidance_dataset.json).

| Earlier run | Observed outcome | Interpretation |
| --- | --- | --- |
| Natural eight-person visual, seed 44 | 75 s; 2 goals; 3 contacts; 4 near misses | Unscored visualization, not a pass |
| Natural eight-person scored, seed 44 | 0/3 goals; 1 contact; 2 near misses; 34.637 s | Failed onboard trial |
| Mock quick, seed 1 | 2/3 goals; 1 contact | Mechanics diagnostic; acceptance failed |
| Historical oracle seeds 42/43/44 | 8/8 goals each, zero recorded contacts/near misses | Oracle diagnostic, not onboard acceptance |

The JSON additionally includes a fixed-forward response run and its measurements.
Source artifact paths and SHA-256 hashes establish which local run supplied each
record. Raw logs are intentionally outside maintained documentation.

The recorded eight-person GUI run had local-planner p95 58.921 ms (median
46.164 ms, maximum 70.073 ms), with perception-stale stops. Earlier four-person
timings must not be substituted for this configuration. Goal handoffs were
logged at 0.494 m and 0.438 m using prepared next goals.

Use `python3 scripts/avoidance_results_gui.py --dataset docs/results/demo_results.json`
to inspect this evidence separately from the new multi-world experiment.
