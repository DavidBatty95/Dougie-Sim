# Native rendering probe

[`rviz_frame_probe.c`](rviz_frame_probe.c) measures real RViz buffer swaps for
[`benchmark_dougie_rviz.py`](../scripts/benchmark_dougie_rviz.py). Run it through:

```bash
./dougie benchmark-rviz --world facility
```

See the [graphics/performance guide](../docs/DOUGIE_PERFORMANCE.md). The live
comparison dashboard is implemented in [`dougie_sim/comparison.html`](../dougie_sim/comparison.html).
