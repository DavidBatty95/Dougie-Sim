# Results-dashboard frontend

[Project home](../README.md) · [Published datasets](../docs/results/README.md) · [Results review](../docs/RESULTS_REVIEW.md)

[`avoidance_dashboard.html`](avoidance_dashboard.html) is the local browser UI
served by `scripts/avoidance_results_gui.py`. It provides dataset selection,
filtering, per-run detail, charts, CSV export and recorded-trajectory replay where
the dataset supplies it.

## Launch

From the repository root:

```bash
./geoff results
# Open http://127.0.0.1:8765
```

Use the Python server rather than opening the HTML as a standalone file: the UI
loads normalized data from its local endpoints. Inspecting published results
requires Python 3 and a browser, not ROS or a running simulator.

## Development and interpretation

Keep missing values distinct from zero and retain invalid attempts in the UI.
Replay samples are asynchronous/downsampled visual aids; scored contact metrics
come from the evaluator. The [results guide](../docs/RESULTS_REVIEW.md) explains
cohort comparisons and strict-validator semantics.

Dashboard regression tests live in `scripts/test_avoidance_dashboard.py`.
The optional Firefox/geckodriver check is described in
[development](../docs/DEVELOPMENT.md). When adding metrics, update both the
normalization layer and UI labels, then verify existing datasets remain readable.
