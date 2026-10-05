# Demo media catalogue

[README gallery](../README.md#recorded-demo-gallery) · [Demo walkthrough](DEMO_GUIDE.md)

The README features all five project demo GIFs under `docs/media/`. These are
recordings of visual sessions, not generated illustrations or scored acceptance
summaries. Upstream training-library example media belongs to its own documentation.

## Recordings

| Asset | View | Context |
| --- | --- | --- |
| [Plaza crowd](media/onboard-crowd.gif) | MuJoCo | Eight-person onboard visual demonstration |
| [Plaza decisions](media/onboard-decisions.gif) | RViz | Onboard sensing, tracking and planning visualization |
| [Courtyard crowd](media/courtyard/onboard-crowd.gif) | MuJoCo | Representative residential courtyard |
| [Courtyard decisions](media/courtyard/onboard-decisions.gif) | RViz | Courtyard perception and navigation visualization |
| [Station crowd](media/station/onboard-crowd.gif) | MuJoCo | Representative station entrance |

The plaza pair has a [recording manifest](media/recordings.json) containing the
source run path, source-result SHA-256, recording timestamp, frame counts and
frame rate. Each clip has 32 frames at 4 fps. The visual runbook identifies the
[courtyard session and handoff observation](ONBOARD_VISUAL_DEMO.md#results-and-verification).
The station preview is presented as a scene illustration; the plaza manifest
does not establish its provenance.

Short recordings do not establish endurance, collision-free behavior or the
behavior of later controller revisions. For numerical claims, cite a scored
dataset and its configuration using the [results-review guide](RESULTS_REVIEW.md).

## Dashboard and schematics

| Asset | Purpose |
| --- | --- |
| [Results dashboard](media/results-dashboard.png) | Screenshot of dataset filtering and recorded-trajectory replay |
| [Banner](media/banner.svg) | Project identity and simulation theme |
| [Installation pipeline](media/install-pipeline.svg) | Setup stages and dependency flow |
| [Architecture](media/architecture.svg) | High-level system organization |
| [Command safety](media/command-safety.svg) | Command supervision concept |
| [Social navigation](media/social-navigation.svg) | Perception/planning concept |
| [Verification tiers](media/verification-tiers.svg) | Layers of implementation and integration checks |
| [Repository map](media/repository-map.svg) | Task-oriented folder overview; the [folder guide](REPOSITORY_GUIDE.md) provides the text equivalent |
| [Project progression](media/project-progression.svg) | Dated development milestones, with source and evidence in [project status](PROJECT_STATUS.md) |
| [Forward roadmap](media/project-roadmap.svg) | Eight acceptance-driven stages; detailed deliverables are in [the plan](PROJECT_PLAN.md) |

The SVGs are schematics, not sensor output. The dashboard screenshot is a static
example; launch `./geoff results` to inspect the checked-in datasets interactively.

The folder, progression and roadmap graphics use a shared two-column layout with
readable static content, descriptive SVG titles and reduced-motion support.
Their source text lives in `BOARDS` in the graphics generator. Status graphics
are dated snapshots: update their text alongside the status/plan pages rather
than treating animation or a colored label as a live progress indicator.

## Updating the gallery

Keep asset paths relative so the README works on GitHub and in a local checkout.
Use descriptive alt text and captions explaining the scene and evidence category.
When replacing a recording, record the command, world, seed, source configuration,
source-result hash and whether the session was scored. Update the associated
manifest rather than letting old provenance describe new footage.

Keep clips short enough for a practical GitHub landing page and retain full raw
logs in the experiment artifact location. Review the rendered README when changing
image dimensions or table layout. Check maintained links and generated schematics:

```bash
python3 scripts/check_docs.py
python3 scripts/generate_readme_graphics.py --check
```
