# Reviewed source patches

The parent repository pins upstream/fork revisions through Git submodules.
These patches carry the local simulator and MJLab modifications needed by the
current workspace, including new files that a parent-repository commit alone
would otherwise omit. Generated walk meshes and machine-specific
`simulate/config.yaml` changes are intentionally excluded.

`manifest.json` records each base commit, patch checksum and resulting source
file checksum. `scripts/setup_sources.sh` applies them after source checkout.

```bash
python3 scripts/source_patches.py apply             # Fresh checkout; idempotent
python3 scripts/source_patches.py check             # Detect snapshot drift
python3 scripts/source_patches.py verify-pristine   # Reproduce files from pinned base
```

After intentionally changing patched sources, review the submodule diff, run its
tests, then regenerate and commit the patch snapshot with the parent changes:

```bash
python3 scripts/source_patches.py export
python3 scripts/source_patches.py verify-pristine
```

The exporter uses an explicit source allowlist. Add new source files to it when
needed. It never commits, stages, resets or discards working-tree changes.
Application fails on a conflicting checkout rather than overwriting local work.
When updating submodule base revisions, review/rebase the patch set explicitly.
