# Contributing to Dougie-Sim

[Project home](README.md) · [Documentation](docs/DOUGIE_INDEX.md) · [Architecture](docs/DOUGIE_SIM.md)

The maintained project is the Unitree Go2 simulation in `dougie_sim/`. Start with
the [current milestones](README.md#research-and-design), choose a focused change,
and describe its acceptance criterion in the issue or pull request.

1. Keep simulator truth out of onboard localization, mapping and route selection.
   Label evaluator/display inputs and changes to the comparison's completion
   criterion explicitly.
2. Run checks appropriate to the changed component. For runtime changes, use
   `./dougie test`; for documentation, use `python3 scripts/check_dougie_docs.py`.
3. Record commands, seeds, policy/configuration, source identity and outcomes for
   experiments. Include failures as well as passes; do not weaken scoring gates.
4. Keep local assets, raw runs, build products, credentials and checkpoints out of
   commits. Publish compact, reproducible evidence under `docs/results/` when it
   supports a maintained claim, test or figure.
5. Update the affected guide and link new guides from `docs/DOUGIE_INDEX.md`.
   Regenerate figures from their recorded inputs rather than editing data by hand.
6. Stage only intended changes and retain upstream copyright/license notices.

The browser comparison, radiation missions, planar navigation and optional Go2
training use the current `dougie` CLI. Retired G1 tools are available through the
[history reference](docs/DOUGIE_HISTORY.md), rather than as a second runtime in this checkout.

See [LICENSES.md](LICENSES.md) for the licensing inventory and [SECURITY.md](SECURITY.md)
for the supported deployment scope.
