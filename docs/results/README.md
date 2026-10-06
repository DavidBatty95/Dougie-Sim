# Published experiment records

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../README.md"><img src="../media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../README.md#quick-start"><img src="../media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../DEMO_GUIDE.md"><img src="../media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../INDEX.md"><img src="../media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../REPOSITORY_GUIDE.md"><img src="../media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../PROJECT_STATUS.md"><img src="../media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../PROJECT_PLAN.md"><img src="../media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../RESULTS_REVIEW.md"><img src="../media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../literature_review/README.md"><img src="../media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../DEVELOPMENT.md"><img src="../media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

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
| [hardware_fidelity_20261006.json](hardware_fidelity_20261006.json) | Nominal model and live articulated-hand interface audits; no physical calibration |
| [avoidance_hardware_fidelity_smoke_20261006.json](avoidance_hardware_fidelity_smoke_20261006.json) | Updated-interface native one-person smoke: 3/3 goals, zero contacts, strict pass |
| [transport_hardware_fidelity_20261006.json](transport_hardware_fidelity_20261006.json) | Two unchanged-code dual-delivery passes following interface/sensor changes |
| [transport_native_contract_20261006.json](transport_native_contract_20261006.json) | Native-inertial/delay development attempts and earlier dual-delivery passes |
| [transport_acquisition_approach_20261006.json](transport_acquisition_approach_20261006.json) | Acquisition-stamp/viewpoint/support-edge follow-up, including failures |
| [visual_goal_handoffs.json](visual_goal_handoffs.json) | Unscored goal-cycling observations |
| [avoidance_r1_baseline_20261005.json](avoidance_r1_baseline_20261005.json) | Instrumented synchronized courtyard baseline; failed trial retained |
| [avoidance_r3_profile_20261005.json](avoidance_r3_profile_20261005.json) | Scoring-phase profiling pilot; failed trial retained |
| [avoidance_r3_bounded_20261005.json](avoidance_r3_bounded_20261005.json) | Bounded priority pilot: one of two strict passes |
| [avoidance_r2_stop_generation_20261005.json](avoidance_r2_stop_generation_20261005.json) | Stop-generation follow-up: both crowd cases failed |
| [transport_runtime_capture_20261005.json](transport_runtime_capture_20261005.json) | Two deliberately short startup/deadline diagnostics, not delivery acceptance |
| [avoidance_r2_atomic_20261005.json](avoidance_r2_atomic_20261005.json) | Atomic planner-command handoff pilot; seed-44 failure retained |
| [avoidance_r2_alignment_20261005.json](avoidance_r2_alignment_20261005.json) | Checked alignment-priority pilot; one of two strict passes |
| [avoidance_r3_supervisor_timing_20261005.json](avoidance_r3_supervisor_timing_20261005.json) | Supervisor timing baseline; failed crowd case |
| [avoidance_r3_prepared_20261005.json](avoidance_r3_prepared_20261005.json) | Immutable prediction preparation; one of two strict passes |
| [avoidance_r3_batched_20261005.json](avoidance_r3_batched_20261005.json) | Batched static checks; both crowd cases failed |
| [avoidance_r2_protocol_smoke_20261005.json](avoidance_r2_protocol_smoke_20261005.json) | One-person, one-goal native protocol smoke passed |
| [avoidance_r2_protocol_handoffs_20261005.json](avoidance_r2_protocol_handoffs_20261005.json) | Full-pose envelope: one person, three goals, strict pass; includes lineage summary |
| [avoidance_r3_async20_20261005.json](avoidance_r3_async20_20261005.json) | Asynchronous search at 20 Hz: both crowd cases failed |
| [avoidance_r3_async50_20261005.json](avoidance_r3_async50_20261005.json) | 50 Hz candidate: no strict passes; seed 42 completed without contact but below score threshold |
| [avoidance_r3_async_handoffs_20261005.json](avoidance_r3_async_handoffs_20261005.json) | Idle-drain follow-up: one person, three goals, strict pass |

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
