# Dougie-Sim media and regeneration

[Project home](../README.md) · [Documentation](DOUGIE_INDEX.md) · [Validation](DOUGIE_VALIDATION.md)

## Gallery provenance

### Reactor/prediction refresh — 15 October 2026

The [reactor still](media/dougie-reactor.png) now shows the expanded industrial
scene. The new [prediction replay](media/dougie/reactor-prediction.png) overlays
archived GP estimates and detector positions from the updated reactor's seed-3
mission. It is not a true-field map or live recording. Both were rendered with
native MuJoCo; their adjacent JSON manifests preserve input/image identities.
See [regeneration and measured scope](DOUGIE_REACTOR_UPDATE.md).

### Full-height update — 14 October 2026

- [Facility cutaway](media/dougie/facility-cutaway.png): actual MuJoCo facility
  model after one second of baseline-policy settling. Layout 0, eight targets,
  landmark seed 7. Transparent walls/slabs and the all-target view are diagnostic,
  not a robot observation. [Image/scene/source manifest](media/dougie/facility-cutaway.json).
- [Facility results SVG](media/dougie/facility-results.svg): actual detector-height
  and XY telemetry from the failed pre-correction seed-3 mission and successful
  corrected seeds 3/4. Not a video, GP heatmap or coverage certificate. The figure
  is regenerated from [portable downsampled measurements and hashes](media/dougie/facility-telemetry.json).

```bash
docker compose run --rm -T -e MUJOCO_GL=egl dougie-sim python3 scripts/render_dougie_facility.py
python3 scripts/render_dougie_facility_results.py
```

The SVG needs no ignored archives to regenerate. To deliberately replace its
portable input, pass `--runs` with the desired three retained mission directories
and update the captions. See [the full-height report](DOUGIE_FULLHEIGHT_RESULTS.md)
for measured results and limitations.

### Original gallery — 7 October 2026

Created for the Go2 README on **7 October 2026**. The shared dark-blue, teal and
amber presentation belongs to Dougie; no G1 footage is used in its gallery.

| Asset | Origin and interpretation |
|---|---|
| [Walking GIF](media/dougie/walking.gif) | New six-second MuJoCo `Simulation('room')` run. Command `(0.3, 0, 0)` m/s, m/s, rad/s; startup settling is part of the clip. 120 frames at 20 fps, nominal 1× simulated-time playback. Camera follows actual base position. Twelve-joint policy/torque physics; no pose interpolation. |
| [Radiation replay GIF](media/dougie/radiation-replay.gif) | Seed-3 initial Poisson cohort. 270 integration-window detector positions from 0.5–135 s shown in 90 frames over nine seconds, approximately 15× playback. The full run returned at 135.23 s. XY projection, metre grid, home square, measured dose/rate and latest available planner state. This is a log visualization, not the live MuJoCo or RViz view and not a GP heatmap. |
| [Reactor still](media/dougie-reactor.png) | Existing actual reactor-model render, produced by [the reactor renderer](../scripts/render_dougie_reactor.py). |
| [Title banner](media/dougie/banner.svg) | Original Go2-inspired vector illustration: rounded silver chassis, dark front sensor housing, circular hip motors and four articulated legs with a diagonal-pair trot, body bob and rotating scan ring. Decorative SVG animation, not recorded gait or telemetry. |
| [Architecture](media/dougie/architecture.svg) | Original static system schematic. |

Both GIFs are 640 × 420 and loop. Restarting a loop resets the displayed time;
there is no interpolated transition back to the first state. The walking command
is a target, not a measured velocity. Its final base position and upright metric
are recorded in the [manifest](media/dougie/manifest.json). The replay budget bar
uses the noisy count-derived dose, which differs from independent actual exposure
in the validation table. Blank areas of the XY plot make no free-space claim.

## Portable inputs

[radiation-replay.json](media/dougie/radiation-replay.json) retains the original
measurements, planner trace, summary and run configuration from
`results/dougie/poisson_seed3/`, plus SHA-256 identities for those four raw files.
The ignored raw directory is not needed to rebuild the committed replay.

[manifest.json](media/dougie/manifest.json) records GIF hashes, frame counts,
dimensions, sizes, renderer/source/asset-manifest hashes and library versions.
The capture used `dougie-sim:humble` image
`sha256:e62530ae7848f15cbd44fd0bf8e7f149fdedc812f9121a4d5588f6ae8ebc21f6`.
Future source, font, Pillow or rendering-library changes can change encoded bytes.
Simulation integration and the replay are checked independently from encoding.

## Regenerate

From the repository root after `./dougie setup`:

```bash
docker compose run --rm -T -e MUJOCO_GL=egl dougie-sim python3 scripts/render_dougie_readme.py
```

The [renderer](../scripts/render_dougie_readme.py) runs fresh walking physics,
reads the committed replay input and rewrites both GIFs and their manifest.
It uses the container's Pillow and DejaVu Sans font and an EGL offscreen context.
No graphical desktop is needed for this command. Software EGL availability is
separate from the non-rendering headless simulator.

To deliberately replace the portable replay with another retained **seed-3** run:

```bash
docker compose run --rm -T -e MUJOCO_GL=egl dougie-sim python3 scripts/render_dougie_readme.py --run results/dougie/poisson_seed3
```

The caption identifies seed 3; update labels and this provenance page when
publishing a different configuration. Keep the dated initial validation cohort
distinct from fresh results. Rebuilding media does not rerun its radiation mission.

To regenerate the overview still:

```bash
docker compose run --rm -T -e MUJOCO_GL=egl dougie-sim python3 scripts/render_dougie_reactor.py
```

For a static alternative, use the reactor image, the architecture diagram and
the [validation tables](DOUGIE_VALIDATION.md). All information-bearing animations
have text descriptions and underlying data links.
