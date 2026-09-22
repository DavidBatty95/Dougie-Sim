# Team handoff

## First session

1. Read the [README](../README.md), [architecture](ARCHITECTURE.md), and
   [dataset methodology](AVOIDANCE_EXPERIMENTS.md).
2. Install on a Linux graphics host, verify fork access and apply the pinned
   [source patches](../patches/README.md).
3. Run fast checks, then one short visual demonstration and one scored trial.
4. Inspect JSON/CSV in the results GUI, including invalid and failed trials.

## Current engineering priorities

- Reliable sensor-only avoidance: existing natural-crowd trials include contacts.
- Latest representative cohort: 4/18 strict passes, 19/54 goals and 12 contacts;
  crowd interactions and task progress remain the main acceptance blockers.
- Perception latency/dropouts under eight-person rendering and GUI load.
- RViz indexed-map GLSL sampler warnings on this host.
- Longer endurance evidence: the recorded short sessions do not establish an
  uninterrupted 30-minute run.
- Deterministic startup/reset orchestration and command-source exclusivity.
- Broader paired seeds/counts/world comparisons, with uncertainty estimates.

The navigation input contract, goal criteria and scoring definitions must remain
explicit. Historical oracle passes cannot establish onboard acceptance. Visual
sessions may restart, log contacts and remain unscored.

## Publication and ownership

- Preserve existing license terms. Confirm root-license policy and pedestrian
  asset/policy provenance before broader redistribution: [inventory](../LICENSES.md).
- Confirm incoming team access to all pinned GitHub forks. Private forks need a
  read-only CI credential; the workflow falls back to the repository token.
- Assign maintainers and configure GitHub branch protection, required reviews,
  required checks, private vulnerability reporting and organization contacts.
- Review Dependabot PRs; actions are pinned by commit and workflow permissions
  are read-only. Do not grant write credentials to untrusted pull requests.
- Run a fresh image/build on the incoming team's hardware and retain its output.

The development-host verification includes a fresh Docker image build, 196
bridge tests, 81 benchmark tests, 18 bringup tests, 69 script tests and four
Torch-specific tests; seven simulator and two costmap/BT CTest targets also passed.
All six generated worlds were loaded by MuJoCo in the simulator tests. The GUI
was checked in Firefox for dataset selection, filtering, details and replay.
These software checks do not turn failed navigation trials into passes.

The repository's maintained documentation is indexed [here](INDEX.md); research
drafts are kept in [archive](../archive/README.md).
