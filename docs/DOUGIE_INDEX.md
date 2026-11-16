# Dougie-Sim documentation

[Project home](../README.md) · [Architecture](DOUGIE_SIM.md) · [History](DOUGIE_HISTORY.md)

## Run and operate

| Task | Guide |
|---|---|
| Start the Apple Silicon demo | [Mac setup and lifecycle](DOUGIE_MAC.md) |
| Read the blind/aware dashboard and configure missions | [Comparison runbook](DOUGIE_COMPARISON.md) |
| Configure robot lifetimes, swaps and wrecks | [Multi-sortie campaigns](DOUGIE_CAMPAIGNS.md) |
| Run resumable paired-seed studies | [Seed-study protocol and reports](DOUGIE_SEED_STUDY.md) |
| Diagnose graphics and rendering | [Performance and graphics](DOUGIE_PERFORMANCE.md) |

## Methods and active development

| Topic | Guide |
|---|---|
| Simulator, sensors and autonomy boundaries | [Architecture](DOUGIE_SIM.md) |
| Radiation field and detector | [Radiation methods](DOUGIE_RADIATION.md) · [Source presets](DOUGIE_SOURCES.md) |
| Online inference and continuous fields | [Windowed GP](DOUGIE_WINDOWED_GP.md) · [Radiation display](DOUGIE_RADIATION_DISPLAY.md) |
| Multilevel support and constraints | [Facility](DOUGIE_FACILITY.md) · [Terrain qualification](DOUGIE_TERRAIN_QUALIFICATION.md) |
| Occupancy lookup | [Exact compact lookup and benchmarks](DOUGIE_OCCUPANCY_EFFICIENCY.md) |
| Experimental frontier scheduling | [Campaign agenda and retained negative results](DOUGIE_CAMPAIGN_AGENDA.md) |
| Experimental route choice | [Pathwise exposure policy](DOUGIE_PATHWISE_POLICY.md) |
| Tune aware planning weights | [Run-to-run learning](DOUGIE_PLANNING_LEARNING.md) |
| Research context | [Reading guide](DOUGIE_RESEARCH.md) |

## Evidence and reproduction

| Resource | Scope |
|---|---|
| [Retained reactor comparison](DOUGIE_COMPARISON_RESULTS.md) | Earlier single-pair, ideal-pose, global-GP result; not validation of the current campaign |
| [Recent policy pilots](results/dougie_policy_pilots_20261110.md) | Matched dose, censored outcomes and invalid revisions |
| [Portable results](results/README.md) | Evidence, inputs and source identities |
| [Media and regeneration](DOUGIE_MEDIA.md) | Current figures and retained comparison provenance |
| [History](DOUGIE_HISTORY.md) | Removed dated plans, reports and tools in the pinned prior Git tree |

The current browser demo is a finite-lifetime reactor campaign. Reference-floor
observation is evaluation-only; completion requires the planner's three fresh
zero-frontier confirmations. Incomplete returns, blocked frontiers and robot
losses remain explicit. The multilevel facility remains experimental.
