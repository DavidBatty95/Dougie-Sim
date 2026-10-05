# Container environment

[Project home](../README.md) · [Installation](../README.md#quick-start) · [Development](../docs/DEVELOPMENT.md)

The supported runtime puts ROS 2 Humble, MuJoCo dependencies and project tooling
inside the `unitree-g1-sim` Compose service. The host provides Docker, display
access, graphics devices and the source checkout.

## Configuration map

| File | Role |
| --- | --- |
| [`Dockerfile`](Dockerfile) | Image build and runtime dependencies |
| [`requirements.txt`](requirements.txt) | Pinned direct Python runtime dependencies |
| [`scripts/`](scripts/) | Container-side setup/entrypoint helpers |
| [`../docker-compose.yml`](../docker-compose.yml) | Default service, source mounts, DDS and display settings |
| [`../docker-compose.gpu.yml`](../docker-compose.gpu.yml) | Optional NVIDIA configuration used by GPU workflows |
| [`../.env.example`](../.env.example) | Documented local environment defaults |

The default working directory is `/workspace/ros2_ws`. Sources and artifacts are
mounted under `/workspace`; the image is not a frozen copy of the current code.
Many tooling/documentation mounts are read-only, while ROS build and training
artifact locations are writable.

## Common commands

From the repository root after installation:

```bash
docker compose config --quiet
docker compose run --rm unitree-g1-sim /workspace/scripts/build_ws.sh
docker compose run --rm unitree-g1-sim /workspace/scripts/build_unitree_mujoco.sh
docker compose run --rm unitree-g1-sim /workspace/scripts/test_workspace_inside.sh fast
```

Use `./geoff` for ordinary demos. GPU training uses the
[MJLab wrapper](../docs/PAYLOAD_POLICY_TRAINING.md#reproduce). Host networking and
the shared DDS domain mean independent containers can still share robot topics;
follow the one-stack-at-a-time runbooks.

## Reproducibility

The ROS base image and direct Python dependencies are pinned. Apt/transitive
dependency resolution is not a bit-for-bit freeze. Experiment records should
retain the actual image ID, source/patch hashes, policy identity and effective
configuration. See [S0 of the project plan](../docs/PROJECT_PLAN.md).
