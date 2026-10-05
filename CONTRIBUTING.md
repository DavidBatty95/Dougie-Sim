# Contributing

Start with [the handoff](docs/HANDOFF.md), [architecture](docs/ARCHITECTURE.md)
and [development guide](docs/DEVELOPMENT.md). This is a simulation research
prototype; reproducible failures are useful evidence.

Use the [repository guide](docs/REPOSITORY_GUIDE.md) to locate components and the
[project plan](docs/PROJECT_PLAN.md) to choose a bounded work package. Link its
S0–S7 or R1–R6 identifier in the issue/PR, state the expected evidence, and assign
an owner when work begins. Update [project status](docs/PROJECT_STATUS.md) when a
published milestone changes the assessment; keep historical experiment reports dated.

1. Work on a focused branch and describe the acceptance criterion.
2. Keep simulator truth out of onboard localization, tracking, planning and goal
   selection. Visualization/scoring may use explicitly isolated diagnostics.
3. Run the checks appropriate to the changed component. Record exact live
   commands, seed, policy, scene and artifacts. Report failures as well as passes.
4. If changing patched sources under a submodule, review/export the patch set
   and verify pristine application before including it in the parent change.
5. Update maintained documentation and generated diagrams. Put research drafts
   in `archive/research/`, clearly marked historical, rather than `docs/`.
6. Stage only intended files. Keep local `.env`, build outputs, checkpoints,
   generated walk meshes and raw logs out of commits.

Do not weaken validators, redefine success, or switch to oracle input to make a
sensor-only experiment pass. Visual sessions are explicitly unscored. Avoid
changing locomotion limits as a side effect of visualization work.

The existing package/submodule licenses remain authoritative; see [LICENSES.md](LICENSES.md).
The incoming team must designate reviewers/owners before enabling mandatory
CODEOWNERS review. No placeholder team identity is enforced in this repository.
