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

## Publication verification — 5 October 2026

The source publication was checked with the existing Docker environment:

| Check | Result |
| --- | --- |
| ROS workspace build, `.verification` overlay | All 15 packages built |
| MuJoCo build and CTest | Build succeeded; 10/10 tests passed |
| Bridge tests | 333 passed |
| Benchmark package tests | 83 passed |
| Bringup tests | 18 passed |
| General script tests in ROS environment | 79 passed; 3 payload tests skipped for missing training dependencies |
| Payload and stability-schedule tests in MJLab | All 7 passed, including the 3 payload tests skipped above |
| Source patch snapshot and pristine replay | Both simulator and MJLab snapshots reproduced |

Run package test directories in separate pytest invocations: their shared `test`
package names collide when collected together by ROS launch-testing. Run
`scripts/test_g1_stability_schedule.py` in MJLab rather than the ROS interpreter.
For the isolated training tests:

```bash
docker compose run --rm -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 unitree-g1-sim \
  /workspace/scripts/run_mjlab.sh --no-sync python -m pytest -q \
  /workspace/scripts/test_g1_payload_training.py \
  /workspace/scripts/test_g1_stability_schedule.py
```

This verifies builds and regression coverage. No new live navigation matrix,
transport mission or hardware acceptance run was collected for this publication.
