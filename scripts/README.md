# Script entry points

[Project home](../README.md) · [Folder guide](../docs/REPOSITORY_GUIDE.md) · [Demo walkthrough](../docs/DEMO_GUIDE.md) · [Project plan](../docs/PROJECT_PLAN.md)

For normal use, run `./geoff` from the repository root:

```bash
./geoff worlds
./geoff demo plaza
./geoff demo cafe --people 4 --duration 300 --fast-start
./geoff demo clinic --people 4 --duration 300 --fast-start
./geoff benchmark --worlds small_clutter medium_doorway --seeds 42 43 44
./geoff results
```

Every generated world and the original plaza can run as a continuous demo. The
demo launcher handles goal handoff, session duration, episode restart and logs.
Use `./geoff demo --help`, `./geoff benchmark --help` or `./geoff results --help`
for the underlying tool options.

## Choose a workflow

| Goal | Recommended entry point | Guide |
| --- | --- | --- |
| Visual exploration | `./geoff demo WORLD` | [Demo walkthrough](../docs/DEMO_GUIDE.md) |
| Scored navigation | `./scripts/run_demo_suite.sh --case onboard-waypoints` | [Suite reference](../docs/DEMO_SUITE.md) |
| Repeated world/seed trials | `./geoff benchmark` | [Experiment methodology](../docs/AVOIDANCE_EXPERIMENTS.md) |
| Existing-result inspection | `./geoff results` | [Results review](../docs/RESULTS_REVIEW.md) |
| Experimental pickup/carry | `./geoff transport --help` | [Payload policy and fixture](../docs/PAYLOAD_POLICY_TRAINING.md) |
| Training/calibration | Explicit checkpoint and measurement tools | [Training guide](../docs/TRAINING_AND_CALIBRATION.md) |

Run host commands from the repository root. The launchers arrange the supported
container environment; individual ROS modules require its sourced workspace.
Use the exact policy and configuration recorded by the relevant experiment.

## Maintained internal tools

| Area | Entry points |
| --- | --- |
| Installation/build | `setup_sources.sh`, `build_ws.sh`, `build_unitree_mujoco.sh` |
| Demo orchestration | `onboard_visual_session.py`, `demo_suite.py` |
| Worlds and experiments | `avoidance_worlds.py`, `world_appearance.py`, `run_avoidance_matrix.py` |
| Results | `avoidance_results_gui.py`, `analyze_avoidance_dataset.py`, `collect_demo_results.py` |
| Diagnostics | `summarize_social_run.py`, `inspect_social_trace.py`, `evaluate_onboard_perception.py` |
| General research batches | `benchmark_batch.py`, `aggregate_benchmark_batch.py` |
| Response calibration | `measure_g1_fixed_command.py`, `process_g1_response_trial.py`, `assemble_g1_response_dataset.py` |
| Transport | `run_transport_demo.py`, `transport_worlds.py`, `transport_evidence.py`, `summarize_payload_results.py` |
| Payload/hand research | `g1_payload_training.py`, `brainco_hand_model.py`, `measure_brainco_contact_grasp.py` |
| Evaluation alignment | `audit_evaluation_alignment.py`, `test_evaluation_audit.py` |
| Source/docs maintenance | `source_patches.py`, `check_docs.py`, `generate_readme_graphics.py` |
| Verification | `test_workspace.sh`, `test_workspace_inside.sh`, `test_*.py` |

The low-level shell wrappers remain because the supported orchestration uses
them. MJLab training/play/evaluation helpers are optional development tools, not
the demo interface; see [training](../docs/TRAINING_AND_CALIBRATION.md).

## Outputs and maintenance

Demo/suite/matrix launchers save artifacts under the benchmark package's ignored
`results/` tree. Training uses `vendor/mjlab/logs/`; selected portable evidence
is published under `docs/results/`. See the
[artifact map](../docs/REPOSITORY_GUIDE.md#where-results-go).

Scripts named `test_*.py` provide regression coverage. Training-specific tests
need MJLab/Torch rather than the ROS Python environment. The
[development guide](../docs/DEVELOPMENT.md) records the current invocation and
verification scope. Changes to public CLI flags should also update the demo
guide, affected package README and reproducibility examples.

README diagrams are generated from `generate_readme_graphics.py`; use `--check`
to detect stale SVGs. `check_docs.py` checks maintained documentation links.

## Historical tools

Removed unused legacy tools:

- `mjlab_continue_g1_from_5597.sh`: tied to one private numbered checkpoint;
  use explicit checkpoint arguments with the maintained training helpers.
- `auto_tune_human_aware_navigation.py` and `tune_extreme_lab_survival.py`:
  old ad hoc Extreme Lab parameter sweeps; use versioned benchmark matrices and
  current source/provenance checks instead.

Their prior contents remain available in Git history.
