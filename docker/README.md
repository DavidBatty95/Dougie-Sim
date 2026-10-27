# Dougie container

[Project home](../README.md) · [Quick start](../README.md#quick-start) · [Graphics guide](../docs/DOUGIE_PERFORMANCE.md)

The maintained image is built from [Dockerfile.dougie](Dockerfile.dougie), a pinned
ROS 2 Humble base plus [requirements-dougie.txt](requirements-dougie.txt) and CPU
PyTorch. The checkout and local `.dougie/` assets are mounted into `/workspace`.

```bash
./dougie setup
./dougie compare --world reactor
./dougie test
```

Compose files at the project root:

- `docker-compose.yml`: Go2 runtime, user mapping and loopback DDS configuration.
- `docker-compose.gpu.yml`: NVIDIA graphics overlay.
- `docker-compose.dri.yml`: DRM/DRI graphics overlay.

Use `./dougie ... --graphics auto` to select hardware graphics where available,
or `--graphics software` for CPU rendering. The browser comparison uses offscreen
EGL; interactive desktop viewers need the session's display credentials.

Optional GPU training is a separate pinned environment installed by
`./dougie training-setup`; it does not change the runtime image or deployed policy.
