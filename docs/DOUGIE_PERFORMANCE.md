# Graphics and performance

[Project home](../README.md) · [Mac launcher](DOUGIE_MAC.md) · [Architecture](DOUGIE_SIM.md)

## Browser campaign on Apple Silicon

```bash
./demo
./dougie-mac compare-status
./dougie-mac compare-logs --tail 50
```

The Mac container uses ARM64, CPU PyTorch and Mesa software rendering. The
browser streams independent MuJoCo frames; a slow or disconnected feed does not
change the independent simulation clocks or accumulated simulated dose. The
active campaign and reports use separate processes and may require significant
CPU and memory. Prefer a single paired campaign at a time on an 8 GB Docker VM;
concurrent multilevel experiments previously encountered worker failures.

The browser page is at <http://localhost:8770>. If it is disconnected, check
`compare-status` and `compare-logs`; `./dougie-mac compare-stop` stops the
dashboard without deleting its saved results. The
[Mac guide](DOUGIE_MAC.md) documents configuration and replay.

## Linux interactive rendering

`./dougie radiation` uses RViz by default; `--visualizer mujoco` selects the
diagnostic MuJoCo viewer and `--headless` suppresses the window. The physics
loop, policy inference, ROS publication and frame presentation are distinct
rates; a configured FPS is not a measurement of delivered frames.

```bash
./dougie graphics-check --require-hardware
./dougie demo --graphics nvidia --world room --vx .2
./dougie graphics-check --graphics dri --require-hardware
./dougie benchmark-rviz --world facility
```

`--graphics` accepts `auto`, `nvidia`, `dri`, `software` or `system`.
NVIDIA mode requires the NVIDIA Container Toolkit; DRI requires appropriate
device permissions. Restart the container session after changing graphics mode.
The benchmark counts delivered viewport frames rather than inferring FPS from
ROS topic or physics rates. `rviz_graphics.json` records the renderer identity.

## Mapping and inference measurements

- [Occupancy lookup](DOUGIE_OCCUPANCY_EFFICIENCY.md) reports exact old/new
  frontier equivalence, dense/sparse timings and memory trade-offs.
- [Windowed GP](DOUGIE_WINDOWED_GP.md) describes bounded resident experts,
  persistent paging, continuous field queries and a retained microbenchmark.
- `scripts/benchmark_dougie_sensors.py` and `scripts/benchmark_dougie_rviz.py`
  measure sensing and rendering separately. Their results are not mission-dose
  or exploration-efficacy comparisons.

Older machine-specific FPS and sensor timing reports are retained in the
[Go2 development archive](DOUGIE_HISTORY.md#go2-development-archive); they
should not be presented as performance of the current campaign.
