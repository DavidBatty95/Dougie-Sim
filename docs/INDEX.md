# Documentation

Geoff-Sim combines Unitree G1 locomotion, MuJoCo simulation, ROS 2 onboard
navigation and source-linked evaluation. Choose a reading path below, or browse
the reference tables.

## Start here

- **Run your first demo:** [installation](../README.md#quick-start) →
  [demo walkthrough](DEMO_GUIDE.md) → [visual runbook](ONBOARD_VISUAL_DEMO.md).
- **Inspect the research:** [results review](RESULTS_REVIEW.md) →
  [published evidence](DEMO_RESULTS.md) → [reliability investigation](NAVIGATION_RELIABILITY_INVESTIGATION.md).
- **Understand the implementation:** [architecture](ARCHITECTURE.md) →
  [source boundaries](ONBOARD_SOCIAL_NAVIGATION.md) → [route planning](DYNAMIC_ROUTE_PLANNING.md).
- **Contribute:** [development checks](DEVELOPMENT.md) →
  [contribution guide](../CONTRIBUTING.md) → [handoff](HANDOFF.md).

## Demos and experiments

| Guide | What it covers |
| --- | --- |
| [Demo walkthrough](DEMO_GUIDE.md) | Choosing a mode, launching viewers, observing behavior and locating artifacts |
| [Visual runbook](ONBOARD_VISUAL_DEMO.md) | Viewer overlays, goal handoffs, episode restarts and session semantics |
| [Demo suite reference](DEMO_SUITE.md) | Presets, individual cases, prerequisites, validators and policy selection |
| [Avoidance experiments](AVOIDANCE_EXPERIMENTS.md) | Six worlds, dataset collection, browser inspection and methodology |
| [General benchmark batches](BENCHMARK_EXPERIMENTS.md) | Broader research batch execution and aggregation |
| [Media catalogue](MEDIA.md) | All README GIFs, recording provenance and diagram descriptions |
| [Script entry points](../scripts/README.md) | Daily CLI and lower-level maintenance tools |

## Results and research reviews

| Guide | What it covers |
| --- | --- |
| [Reviewing results and evidence](RESULTS_REVIEW.md) | Evidence categories, metrics, provenance and reproducible reporting |
| [Published demo evidence](DEMO_RESULTS.md) | Published v12 cohort and separately labelled historical demos |
| [Navigation reliability investigation](NAVIGATION_RELIABILITY_INVESTIGATION.md) | Retained corrections, failed experiments and the iteration ledger |
| [Literature review](literature_review/README.md) | PDF, 50-work evidence map, research gaps, reading audit and bibliography verification |
| [Training and calibration](TRAINING_AND_CALIBRATION.md) | Policy training, response measurements and calibration tools |
| [Payload-aware training](PAYLOAD_POLICY_TRAINING.md) | Training cycle, native transport results and explicit checkpoint reproduction |
| [Transport implementation plan](TRANSPORT_DEMO_PLAN.md) | Mission design, integration requirements and acceptance goals |
| [Social locomanipulation roadmap](SOCIAL_LOCOMANIPULATION_ROADMAP.md) | Active staged plan for BrainCo-equipped indoor delivery, implementation ledger and promotion gates |

## Architecture and maintenance

| Guide | What it covers |
| --- | --- |
| [Architecture](ARCHITECTURE.md) | Components and end-to-end data flow |
| [Onboard source boundaries](ONBOARD_SOCIAL_NAVIGATION.md) | Sensor-driven inputs, simulator-truth isolation and validation scope |
| [Dynamic route planning](DYNAMIC_ROUTE_PLANNING.md) | Replanning triggers, timing and route execution |
| [Development and verification](DEVELOPMENT.md) | Host checks, Docker tests and source-patch maintenance |
| [Team handoff](HANDOFF.md) | Operational context and remaining work |
| [Source patches](../patches/README.md) | Reproducing changes to pinned upstream sources |
| [Contributing](../CONTRIBUTING.md) | Contribution workflow and review expectations |
| [Security](../SECURITY.md) / [Licenses](../LICENSES.md) | Reporting scope and existing licensing information |
| [GitHub profile package](../profile/SETUP.md) | Optional companion profile setup |

Portable measurements live in [`docs/results/`](results/). Raw simulation logs,
generated scenes and locally trained policies follow the artifact locations in
their respective runbooks. The [calibration schemas](schemas/) are machine
contracts. [Archived research drafts](../archive/README.md) preserve historical
context; use maintained guides for current entry points.
