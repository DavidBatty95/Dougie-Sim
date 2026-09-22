# Development and verification

Use the Docker ROS Humble environment for ROS tests. Host-only tooling such as
the results GUI, world generator and source-patch manager uses Python 3.

```bash
python3 scripts/check_docs.py
python3 scripts/generate_readme_graphics.py --check
python3 scripts/source_patches.py check
python3 scripts/source_patches.py verify-pristine
python3 -m pytest scripts/test_avoidance_dashboard.py scripts/test_avoidance_worlds.py
docker compose run --rm unitree-g1-sim /workspace/scripts/test_workspace_inside.sh fast
```

After changing C++ simulator code:

```bash
docker compose run --rm unitree-g1-sim /workspace/scripts/build_unitree_mujoco.sh
docker compose run --rm unitree-g1-sim ctest --test-dir /workspace/ros2_ws/src/unitree_mujoco/simulate/build --output-on-failure
python3 scripts/source_patches.py export
python3 scripts/source_patches.py verify-pristine
```

Do not run Compose tests concurrently with a live matrix: benchmark cleanup uses
the simulator service's one-off container names. The full `ci` test mode builds
the ROS workspace and runs package tests plus a strict mock benchmark gate.
That benchmark's acceptance failure must remain visible.

Patched submodules intentionally appear dirty. Their pinned gitlinks plus
`patches/manifest.json` and the reviewed patches reproduce the maintained sources;
machine-specific `simulate/config.yaml` changes are not exported. Do not commit
generated meshes, build trees, local environments, policies or raw logs.

The Docker image copies only `docker/` runtime files; sources are mounted at
runtime. Direct Python runtime packages are pinned in `docker/requirements.txt`,
and the verified ROS base image is pinned by digest. Apt repositories and
transitive Python dependencies are not bit-for-bit frozen. Cohort manifests
record the actual Docker image ID used for new runs.

The optional real-browser check uses installed Firefox/geckodriver:

```bash
./geoff results
# In another terminal:
python3 scripts/check_dashboard_browser.py
```

It verifies dataset selection, filtering, full run details and recorded-trajectory
replay. Torch-specific training tests use the MJLab environment rather than the
ROS Python interpreter; disable ROS pytest plugin autoload for that isolated test.
