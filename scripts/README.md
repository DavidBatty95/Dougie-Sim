# Script entry points

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
| Source/docs maintenance | `source_patches.py`, `check_docs.py`, `generate_readme_graphics.py` |
| Verification | `test_workspace.sh`, `test_workspace_inside.sh`, `test_*.py` |

The low-level shell wrappers remain because the supported orchestration uses
them. MJLab training/play/evaluation helpers are optional development tools, not
the demo interface; see [training](../docs/TRAINING_AND_CALIBRATION.md).

Removed unused legacy tools:

- `mjlab_continue_g1_from_5597.sh`: tied to one private numbered checkpoint;
  use explicit checkpoint arguments with the maintained training helpers.
- `auto_tune_human_aware_navigation.py` and `tune_extreme_lab_survival.py`:
  old ad hoc Extreme Lab parameter sweeps; use versioned benchmark matrices and
  current source/provenance checks instead.

Their prior contents remain available in Git history.
