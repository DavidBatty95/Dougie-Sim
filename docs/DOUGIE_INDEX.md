# Dougie-Sim documentation

[Project home](../README.md) · [Research](DOUGIE_RESEARCH.md) · [Validation](DOUGIE_VALIDATION.md)

The active project is the Unitree **Go2 quadruped**. Start here for its runtime,
research and evidence; the larger inherited documentation tree also contains
Geoff's G1 humanoid development history.

| I want to… | Start here |
|---|---|
| Install and run a first mission | [Quick start](../README.md#quick-start) |
| See the robot and recorded mission | [Animated gallery](../README.md#recorded-demo-gallery) |
| Set budgets, likelihood and mission duration | [Radiation runbook](../README.md#run-a-radiation-mission) |
| Configure point-source hotspots and leaks | [Source presets, markers and configuration](DOUGIE_SOURCES.md) |
| Drive Go2 or inspect ROS | [Walking, exploration and ROS](../README.md#walking-exploration-and-ros) |
| Understand autonomy and simulator truth | [Architecture](DOUGIE_SIM.md) |
| Understand detector, GP and route-dose assumptions | [Radiation methods](DOUGIE_RADIATION.md) |
| Read the papers and plan comparisons | [Research reading guide](DOUGIE_RESEARCH.md) |
| Inspect measured outcomes and limitations | [Validation](DOUGIE_VALIDATION.md) · [Portable report](results/dougie_initial_20261007.json) |
| Check stair and uneven-ground capability | [680-trial terrain qualification](DOUGIE_TERRAIN_QUALIFICATION.md) · [Per-run data](results/dougie_terrain_qualification_20261007.json) |
| Plan stair training and multi-floor radiation missions | [Capability assessment and implementation roadmap](DOUGIE_MULTILEVEL_ROADMAP.md) |
| Implement multi-floor exploration | [Complete TXT plan](DOUGIE_MULTIFLOOR_PLAN.txt) · [Runnable milestone and training results](DOUGIE_MULTIFLOOR_PROGRESS.md) |
| Run the full-height facility and inspect Go2 in RViz | [Facility, perceptive training, model verification and results](DOUGIE_FACILITY.md) |
| Check the latest full-height runs or add discovery targets | [Round-trip results, landmark sensing and graphics](DOUGIE_FULLHEIGHT_RESULTS.md) · [Portable evidence](results/dougie_fullheight_20261014.json) |
| Fix slow RViz rendering or measure real FPS | [GPU graphics setup and frame-delivery benchmark](DOUGIE_PERFORMANCE.md) |
| Smooth routes, scale the GP, or highlight detected radiation | [Motion, spatial mapping, incline training and display update](DOUGIE_SMOOTH_MAPPING.md) |
| Inspect the detailed reactor and improved cloud rendering | [Reactor update, prediction replay and physical checks](DOUGIE_REACTOR_UPDATE.md) |
| Improve mission completion, return prediction and map capacity | [Planning priorities, inspection mode and measured tradeoffs](DOUGIE_PLANNING_PRIORITIES.md) |
| Review staged navigation optimizations and their tests | [Navigation efficiency and retained regressions](DOUGIE_NAVIGATION_EFFICIENCY.md) |
| Rebuild or inspect the GIFs | [Media provenance](DOUGIE_MEDIA.md) |
| Find implementation files | [Repository map](../README.md#repository-map) |
| Diagnose setup problems | [Troubleshooting](../README.md#troubleshooting) |
| Inspect upstream asset identities | [Asset manifest](../config/dougie_assets.json) · [Attribution](DOUGIE_SIM.md#attribution) |

## Historical resources

- [Preserved Geoff README](../README-GEOFF.md): G1 navigation and transport.
- [Geoff documentation index](INDEX.md): inherited guides and experiments.
- [Geoff literature review](literature_review/README.md): humanoid research and PDF.

These historical resources retain their original robot and evidence scope.
