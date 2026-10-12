# Point-source hotspots and leaks

[Project home](../README.md) · [Radiation model](DOUGIE_RADIATION.md)

The radiation demo now defaults to **six localized point emitters**: three
hotspots and three leak locations. The field is a sum of source contributions
with the existing concrete shielding and background. Leak scenarios represent
steady localized radiation from contaminated joints, seals or drains; fluid
transport and dispersal are separate physical processes.

## Launch

From the repository root:

```bash
./dougie radiation --source-profile hotspots-and-leaks --show-sources --dose-budget 2 --duration 600
```

`--show-sources` enables diagnostic MuJoCo site markers: **red = hotspot**,
**amber = leak**. Markers are at the exact emitter locations and may be occluded
by equipment containing them. They are separate from the translucent GP-map
spheres. Sources have no collision geometry, and markers do not affect LiDAR,
the counting detector, the GP or the planner. Omit the flag for the normal
hidden-source view; autonomy always receives only detector observations.

| Profile | Emitters |
|---|---|
| `hotspots-and-leaks` (default) | All six new point sources |
| `hotspots` | Vessel, waste drum, floor contamination |
| `leaks` | Coolant pipe joint, pump seal, drain |
| `baseline` | Original three-source field for reproducing earlier results |

For a single family:

```bash
./dougie radiation --source-profile leaks --show-sources --duration 600
./dougie radiation --source-profile hotspots --show-sources --duration 600
```

The combined scenario has greater exposure than the original baseline. A small
0.2 µSv budget can lead to an early return with little travel; this is not a
guarantee of full facility coverage. Prior qualification results belong to the
`baseline` field, not these new source profiles.

## Edit sources

Configuration: [`config/dougie_radiation_sources.json`](../config/dougie_radiation_sources.json).

| Source ID | XYZ (m) | Strength (µSv·m²/h) | Core radius (m) |
|---|---|---:|---:|
| `vessel_hotspot` | `[5.8, 3.8, 1.2]` | 450 | 0.06 |
| `waste_drum_hotspot` | `[-5.2, 4.8, 0.6]` | 65 | 0.05 |
| `floor_contamination` | `[2.6, -1.8, 0.12]` | 20 | 0.04 |
| `coolant_pipe_leak` | `[-1.5, 2.8, 2.8]` | 60 | 0.04 |
| `pump_seal_leak` | `[-5.0, 2.5, 0.65]` | 40 | 0.05 |
| `drain_leak` | `[6.5, -4.4, 0.12]` | 50 | 0.04 |

Example source entry:

```json
"pump_seal_leak": {
  "kind": "leak",
  "position": [-5.0, 2.5, 0.65],
  "strength_usv_h_m2": 40.0,
  "core_radius_m": 0.05,
  "enabled": true
}
```

`enabled` is optional and defaults to `true`. Set it to `false` to disable a
source. Add new IDs to a profile's `sources` list to activate them. IDs must use
lowercase letters, digits and underscores, beginning with a letter. A profile
can contain up to 64 sources, or an empty list for a background-only control.
Each profile sets its own `background_usv_h`.

Without shielding, one emitter contributes:

```text
rate = strength_usv_h_m2 / (distance_m² + core_radius_m²)
```

The finite core prevents a singularity at the emitter. Beyond that small core,
falloff is approximately inverse-square. Strength is approximately the
unshielded rate at one metre, not a constant rate throughout the room. Fields
add together; background is added once. Only explicitly tagged shield geometries
attenuate the field. This remains a gamma dose-equivalent simulation proxy.

To use a separate file, put it inside the checkout and pass:

```bash
./dougie radiation --source-config config/my_sources.json --source-profile hotspots-and-leaks --show-sources
```

Unknown IDs/profiles, duplicate profile IDs, malformed coordinates, nonfinite
values, negative strengths and core radii below 1 mm are rejected before launch.
The field is static during each run, consistent with the current spatial GP.

## Reproducibility

Every mission records the selected profile and configuration checksum in
`run_config.json`, and writes the normalized simulator input to
`source_scenario.json`. `evaluator_field.json` records the instantiated source
IDs, kinds, strengths, radii and shielding for post-run scoring. These files
are operator/evaluator artifacts and are not published as detector measurements.

To reproduce the earlier three-source missions:

```bash
./dougie radiation --source-profile baseline --headless --fast --dose-budget 0.2 --duration 180 --seed 3
```
