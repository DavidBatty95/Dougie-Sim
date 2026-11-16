# Repository history and retained evidence

[Project home](../README.md) · [Documentation](DOUGIE_INDEX.md)

The maintained checkout contains the **Go2 Dougie runtime**, its tests, pinned
configuration, optional training setup and the evidence used by its published
results. The 27 October 2026 cleanup retired the unused Geoff/G1 workspace,
submodule declarations, source patches, transport/avoidance tools, old CI,
versioned G1 datasets and duplicated documentation/media navigation.

## Access the previous tree

All removed material remains in Git history at the pre-cleanup commit:

**[`cc4a3006b886f2e9ee465418b92fced2feeb304f`](https://github.com/DavidBatty95/Dougie-Sim/tree/cc4a3006b886f2e9ee465418b92fced2feeb304f)**

- [Historical Geoff README](https://github.com/DavidBatty95/Dougie-Sim/blob/cc4a3006b886f2e9ee465418b92fced2feeb304f/README-GEOFF.md)
- [G1 documentation and research](https://github.com/DavidBatty95/Dougie-Sim/tree/cc4a3006b886f2e9ee465418b92fced2feeb304f/docs)
- [G1 workspace](https://github.com/DavidBatty95/Dougie-Sim/tree/cc4a3006b886f2e9ee465418b92fced2feeb304f/ros2_ws)
- [Older G1 experiment datasets](https://github.com/DavidBatty95/Dougie-Sim/tree/cc4a3006b886f2e9ee465418b92fced2feeb304f/docs/results)

Inspect any historical file locally without changing branches:

```bash
git show cc4a3006b886f2e9ee465418b92fced2feeb304f:README-GEOFF.md
```

No history rewrite is required. Original source revisions and their license
notices remain together. The upstream [Geoff-Sim project](https://github.com/DavidBatty95/Geoff-Sim)
is a separate humanoid project; its results are not Go2 qualification evidence.

## Why some dated Dougie reports remain

The current README, calibration/configuration, reproduction scripts and research
guides depend on dated Go2 evidence. Those inputs—including unsuccessful trials—
remain inspectable under [`docs/results/`](results/README.md) and in the
[media catalogue](DOUGIE_MEDIA.md). They are supporting evidence, not competing
runtime versions. One unreferenced, byte-identical registration-report duplicate
was removed; the canonical `dougie_registration_probe_670_20261025.json` remains.

Raw runs and downloaded/training assets remain local and ignored under
`results/` and `.dougie/`. The runtime downloads its own checksum-verified assets;
optional Go2 training installs pinned MJLab packages rather than using the retired
humanoid submodule.

## Go2 development archive

The current documentation index focuses on the maintained campaign, inference,
mapping and runbooks. Dated October 2026 Go2 plans, intermediate development
reports and their one-off benchmark/audit scripts were removed from the working
tree because they describe superseded configurations and repeated status updates.
Their original contents remain at the last published pre-cleanup commit:

**[Go2 tree at `db22a5ff`](https://github.com/DavidBatty95/Dougie-Sim/tree/db22a5ff01f92c74482072de9e03ac26e1416d87)** ·
[Archived documentation](https://github.com/DavidBatty95/Dougie-Sim/tree/db22a5ff01f92c74482072de9e03ac26e1416d87/docs) ·
[Archived scripts](https://github.com/DavidBatty95/Dougie-Sim/tree/db22a5ff01f92c74482072de9e03ac26e1416d87/scripts)

This includes the planned-stop, traction, stance, localization, full-height,
navigation-recovery and multifloor development records. Their failed trials are
not reclassified as successes. Portable result files, current-pilot summaries and
raw local run outputs remain retained. `git show db22a5ff:<path>` retrieves a
tracked historical file without changing branches.
