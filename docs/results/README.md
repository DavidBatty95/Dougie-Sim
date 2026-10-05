# Published experiment records

[Project home](../../README.md) · [Results-review guide](../RESULTS_REVIEW.md) · [Current status](../PROJECT_STATUS.md)

This directory holds portable measurements and provenance for inspection without
a running simulator. It includes failures and invalid attempts, as well as
successful runs. Raw logs and local policies live in the artifact locations
described by the [repository guide](../REPOSITORY_GUIDE.md#where-results-go).

## Start with a dataset

| Record | Purpose |
| --- | --- |
| [avoidance_dataset.json](avoidance_dataset.json) | Published v12 representative-world cohort used by the default dashboard |
| [demo_results.json](demo_results.json) | Earlier visual, mock, oracle and locomotion records |
| [avoidance_v82_matrix.json](avoidance_v82_matrix.json) | Consolidated investigation snapshot with recorded replay data |
| [avoidance_v84_matrix.json](avoidance_v84_matrix.json) | Later complete matrix: 10/18 strict passes, six contacts |
| [avoidance_v85_alignment_audit.json](avoidance_v85_alignment_audit.json) | Targeted evaluation-alignment diagnostic, not a complete release matrix |
| [payload_training_cycle_01.json](payload_training_cycle_01.json) | Payload-training evaluation and native candidate attempts |
| [visual_goal_handoffs.json](visual_goal_handoffs.json) | Unscored goal-cycling observations |

Version numbers identify development datasets, not software releases or rankings.
Files named `pilot`, `validation`, `smoke` and `matrix` may contain different
worlds and trial counts. Consult the [investigation ledger](../NAVIGATION_RELIABILITY_INVESTIGATION.md)
and [transport history](../TRANSPORT_DEMO_PLAN.md) before comparing them.

## Inspect and export

From the repository root:

```bash
python3 scripts/avoidance_results_gui.py
# Open http://127.0.0.1:8765
python3 scripts/avoidance_results_gui.py --export docs/results/avoidance_dataset.csv
```

Use the dataset selector to keep cohorts separate. The dashboard supports the
avoidance/demo datasets; specialized payload and audit records have their own
report structures and should be read with their corresponding runbooks.

## Publishing another result

Use a fresh output name for a new experiment. Include its question, exact command,
source/configuration/policy identity, world version, seed, budget, data validity
and validator outcome. Retain all attempts and state how the record was selected.
Update the relevant experiment report and [status page](../PROJECT_STATUS.md) when
the evidence changes the project assessment. The default dataset is a deliberate
publication snapshot, not automatically the newest local experiment.
