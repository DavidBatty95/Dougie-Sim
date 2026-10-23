# Dougie-Sim media and regeneration

[Project home](../README.md) · [Documentation](DOUGIE_INDEX.md) · [Validation](DOUGIE_VALIDATION.md)

## Gallery provenance

### Range-hold replay — 23 October 2026

[Range-hold GIF](media/dougie/range-hold.gif) and
[static comparison](media/dougie/range-hold.png) visualize the full
[20-case healthy-range screen](results/dougie_range_hold_20261023.json).
The same 81-frame, 800 × 590, 100 ms/frame format uses evaluator telemetry and a
common 0–2 m scale. This is not video or evidence of facility completion.
The [manifest](media/dougie/range-hold.json) pins the inputs and renderer.

The animation is explicitly labelled **healthy-range**. The separate
[eight dropout trials](results/dougie_range_hold_dropout_20261023.json) are retained
and linked next to it in the README: withholding correction commands still left
1.32–1.52 m of physical drift. Their outcomes must not be inferred from the
healthy-range animation. All earlier comparator animations remain available.

```bash
docker compose run --rm -T dougie-sim python3 scripts/render_dougie_stop_transitions.py \
  --report docs/results/dougie_range_hold_20261023.json --prefix range-hold
```

Generation decodes and verifies all 81 frames. The shared renderer and manifests
for the earlier two stop-screen animations were refreshed at this stage.

### Active position-hold replay — 23 October 2026

The [active-hold GIF](media/dougie/position-hold.gif) and
[static comparison](media/dougie/position-hold.png) use all 20 cases in the
[active-feedback report](results/dougie_position_hold_20261023.json).
The 81-frame, 800 × 590 animation is downsampled **evaluator telemetry**, not
physics video. It shows the same common 0–2 m physical-drift scale as the earlier
frozen-target screen. Zero contact failures do not imply successful stationary
hold: the residual .37–.60 m ramp drift is labelled explicitly.

The [manifest](media/dougie/position-hold.json) pins the report, renderer, GIF,
Pillow version and frame timing. Regenerate and decode all frames with:

```bash
docker compose run --rm -T dougie-sim python3 scripts/render_dougie_stop_transitions.py \
  --report docs/results/dougie_position_hold_20261023.json --prefix position-hold
```

The prior frozen-target replay is retained and its renderer manifest refreshed.
Native mission footage remains the separately qualified 8° round trip below.

### Stop-transition replay — 22 October 2026

[Stop-transition GIF](media/dougie/stop-transitions.gif) and
[static final comparison](media/dougie/stop-transitions.png) visualize all 20
trials in the [paired walking-to-stop report](results/dougie_stop_transitions_20261022.json).
The GIF has 81 frames at 800 × 590, 100 ms/frame, approximately 15× playback.
It is evaluator telemetry, not physics video. Bars show maximum displacement so
far across both seeds, relative to the first 1 Hz sample at the stop request;
the report retains exact full-control-rate maxima. Final contact outcomes remain
labelled throughout the animation. All four downhill hold failures are shown.

The [manifest](media/dougie/stop-transitions.json) records report/renderer/GIF
hashes, Pillow version, timing, dimensions and the existing contact thresholds.
Regenerate and decode every frame with:

```bash
docker compose run --rm -T dougie-sim python3 scripts/render_dougie_stop_transitions.py
```

The earlier native mission GIFs below retain their runtime identities: this stage
changes the screening harness, not the mission controller.

### Pose-graph mission refresh — 21 October 2026

The README leads with a fresh **pose-graph / fused-localization / geometry-GP /
moving-window** recording of the explicit **8° mezzanine** variant. The original
10° fixture and full-height facility remain unqualified.

| Asset | Provenance and interpretation |
|---|---|
| [Onboard mission GIF](media/dougie/onboard-mission.gif) | Actual native MuJoCo run `pose_graph_mezz8_recorded`, 73 frames, 640 × 490, 100 ms/frame. Approximately 33× simulated-time playback. Close route-only recording view keeps Go2 visible; geometry-aware inference still runs. The full rendered world is diagnostic, not robot perception. |
| [Fusion/window GIF](media/dougie/onboard-fusion.gif) | Same run's downsampled pose/count telemetry, 100 frames, 640 × 490, 100 ms/frame. Approximately 24× playback. Teal estimated pose, white independent evaluator pose, amber motion-window quadrature positions; the height readout is estimate/evaluator. Not video, a coverage map, or a GP field. |
| [Mission poster](media/dougie/onboard-mission.png) / [telemetry poster](media/dougie/onboard-fusion.png) | Static alternatives extracted/rendered at approximately two-thirds of the respective animation. |
| [Portable telemetry](media/dougie/onboard-replay.json) | Estimated/evaluator poses, counts and motion windows, run settings, selected outcomes, capture metadata and source/input hashes. |
| [Manifest](media/dougie/onboard-manifest.json) | GIF hashes, exact frame times, dimensions, byte sizes, renderer/Pillow identity and runtime source identities. |

The recording reproduced the development probe's **237.490 s** return,
**23.166 m** travel and **.164563 uSv** actual dose. It passed the strict ordered
upper/lower/home witness gate, with .0958 m home error and no non-foot contacts.
The upper witness occurred at 85.170 s; lower return at 192.825 s. Measured dose
on the HUD is .16582 uSv. Sensor seed 0, detector seed 3, 8° mezzanine,
10 uSv budget and 600 s limit. This recording uses the diagnostic runtime without
the historical source-pinned calibration profile. It is not proof of complete
coverage, real-time performance or lower dose at matched work.

The two GIFs loop; the transition back to the beginning resets displayed time.
The native recording is copied unchanged from its checksum-verified capture.
No hidden pose is supplied to the estimator; evaluator truth in the replay is
only a visual comparison. [Methods, failures and evidence](DOUGIE_SLAM_QUALIFICATION.md).

Generate a fresh recording and publish both animations:

```bash
docker compose run --rm -T -e MUJOCO_GL=egl dougie-sim \
  python3 -m dougie_sim.radiation_run --world mezzanine --mezzanine-angle 8 --headless --fast \
  --duration 600 --dose-budget 10 --seed 3 --sensor-seed 0 --fixed-plan-delay .5 \
  --slam-backend pose-graph --record-demo --record-view routes --require-round-trip \
  --output results/dougie/NEW_RUN
docker compose run --rm -T dougie-sim python3 scripts/render_dougie_onboard.py \
  --run results/dougie/NEW_RUN
```

`NEW_RUN` must not already exist. To regenerate the telemetry GIF from committed
inputs, omit `--run`. Regenerating native physics footage requires a fresh mission;
it is not reconstructed from downsampled telemetry.

Verify every GIF frame, checksum and portable detector window:

```bash
docker compose run --rm -T dougie-sim python3 scripts/check_dougie_onboard_media.py
```

### Historical onboard checkpoint — 19 October 2026

The previous GIF pair showed `onboard_recorded_routes_seed3`: a reactor mission
returning at 47.050 s / .126188 uSv. Its files were superseded by the 21 October
refresh above. The earlier [portable results](results/dougie_onboard_20261019.json)
and repository history retain that checkpoint; it is distinct from the new
vertical recording and pose-graph calibration cohort.

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
