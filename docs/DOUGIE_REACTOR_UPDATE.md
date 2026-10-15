# Reactor detail and prediction-display update — 15 October 2026

[Project home](../README.md) · [Radiation display methods](DOUGIE_SMOOTH_MAPPING.md)

## More detailed reactor room

![Actual expanded MuJoCo reactor model](media/dougie-reactor.png)

The reactor now includes a domed vessel head, flanges, top nozzle and instrument
branch; twin pump/motor assemblies and coupling guards; coolant downcomers,
pipe-support racks and valve handles; cabinet screens, status lamps and vents;
an overhead cable tray, spare-parts crates, a shielded service alcove, work lights,
and aisle/hazard markings. This remains an illustrative industrial environment,
not an engineering model of a specific reactor.

Large equipment, racks and the new shielding wall have physical collision
geometry. Small cabinet details and painted markings are visual-only group-2
geometry and are excluded from range sensing. The initial staging floor remains
clear. The additional axis-aligned alcove shield is included in both the nominal
and resolved radiation descriptions; the reactor now has four attenuation boxes.
Added metallic equipment is not assigned an invented material attenuation model.

## Clearer prediction visualization

![Archived GP prediction and recorded detector trajectory in the expanded reactor](media/dougie/reactor-prediction.png)

The cloud now uses a **world-aligned 0.18 × 0.18 × 0.12 m display grid** instead
of repeating a separate set of overlapping blobs for every query. Duplicate
queries do not increase cloud density or opacity. Up to four nearby samples blend
in log-rate space, within a local XYZ support envelope, so colour transitions are
smoother without filling the gaps between different floors. The uncertainty
display also includes local disagreement.

Display opacity is reduced to **0.09**, with glossy highlights removed in MuJoCo,
to keep the robot, equipment and route visible. The payload remains bounded at
1,200 glyphs. Evidence filtering is reapplied to cloud centres; small glyphs can
overlap geometric boundaries and are not precise hazard contours. Blank areas
are not evidence of safety.

This is **display interpolation**, not additional GP measurements or a new
posterior. Raw predictions, planner admission and radiation avoidance logic are
unchanged by the renderer. The screenshot is a replay of archived final GP/count
evidence and actual detector positions, not a live video or a true-field map.

```bash
./dougie radiation --world reactor --duration 300 --dose-budget 2 --show-sources
# Native MuJoCo viewer instead of RViz:
./dougie radiation --world reactor --duration 300 --dose-budget 2 \
  --show-sources --visualizer mujoco
```

The larger source cores, labelled elevated beacons and rings remain available
with `--show-sources`. These are explicitly diagnostic true-source annotations.

## Physical checks

Baseline gait, smooth execution, global Poisson GP, default six-source
`hotspots-and-leaks` profile, fixed 0.5 s simulated planner delay:

| Detector seed | Limit / budget | Home return | Travel | Actual dose | Non-foot impacts |
|---|---|---:|---:|---:|---:|
| 3 | 300 s / 2 µSv | 265.555 s | 33.80 m | 0.79568 µSv | 0 |
| 4 | 300 s / 2 µSv | 261.545 s | 32.69 m | 0.83956 µSv | 0 |

Both completed clean returns. An earlier 0.2 µSv smoke returned after 17.015 s
and only 1.49 m, so it is retained as a short smoke rather than coverage evidence.
These are development checks on one scene, not full exploration certification or
proof of improved avoidance. Geometry and shielding changed, so old reactor
trajectories are a separate cohort. [Portable mission evidence](results/dougie_reactor_detail_20261015.json).

**180 tests passed**, including local cloud gradients, duplicate-density invariance,
cross-floor separation, physical/decorative geometry contracts and shielding
consistency. The expanded reactor with a 1,200-point cloud and source annotations
also ran near the requested 30 FPS in the NVIDIA RViz rendering check; that is a
synthetic rendering benchmark, not a mission-time performance guarantee.
The final check measured [30.2 presented FPS](results/dougie_reactor_rendering_20261015.json).

## Image regeneration

```bash
docker compose run --rm -T -e MUJOCO_GL=egl dougie-sim python3 scripts/render_dougie_reactor.py
docker compose run --rm -T -e MUJOCO_GL=egl dougie-sim python3 scripts/render_dougie_reactor.py \
  --run results/dougie/radiation_20261015T070542_993410Z \
  --output docs/media/dougie/reactor-prediction.png
```

[Scene-image manifest](media/dougie-reactor.json) and
[prediction-image manifest](media/dougie/reactor-prediction.json) retain the image,
scene, renderer and input identities. Replaying the prediction requires the raw
ignored mission archive; the published images and compact result report remain portable.
