# Research and paper archive

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../README.md"><img src="../media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../README.md#quick-start"><img src="../media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../DEMO_GUIDE.md"><img src="../media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../INDEX.md"><img src="../media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../REPOSITORY_GUIDE.md"><img src="../media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../PROJECT_STATUS.md"><img src="../media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../PROJECT_PLAN.md"><img src="../media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../RESULTS_REVIEW.md"><img src="../media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../literature_review/README.md"><img src="../media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../DEVELOPMENT.md"><img src="../media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[← Resource library](README.md) · [Experiments and evidence](EXPERIMENTS.md) · [Forward plan](../PROJECT_PLAN.md)

[![Research and paper collection](../media/resources/research.svg)](#complete-paper-index)

## Review package and reading paths

| Resource | Use it for |
| --- | --- |
| [Literature-review guide](../literature_review/README.md) | Scope, main argument, build commands and update procedure |
| [Review PDF](../literature_review/literature_review.pdf) | Critical synthesis, research gaps, hypotheses, ablations and evaluation guidance |
| [Editable LaTeX](../literature_review/literature_review.tex) | Maintaining the review document |
| [Reading audit](../literature_review/READING_AUDIT.md) | What was inspected for each paper, beyond identity verification |
| [Search log](../literature_review/SEARCH_LOG.md) | Search scope, cutoff, screening and unresolved follow-up |
| [Verified metadata](../literature_review/verified_sources.json) / [BibTeX](../literature_review/references.bib) | Source identities, recorded retrieval evidence and citations |
| [Grove-G1 review](../GROVE_G1_REVIEW.md) | A separate implementation review informing integration experiments |
| [Historical research drafts](HISTORY.md#earlier-research-notes) | Earlier proposals and their relationship to the current plan |

**Suggested paths:** begin with surveys for orientation; compare classical and
learned navigation for baselines; use perception/prediction for observation
assumptions; use locomotion/transfer for policy response; finish with benchmarks
and evaluation principles when designing an experiment.

## Complete paper index

This index groups all **50 works** in the checked-in bibliography. Titles, years,
URLs and inspection labels come from the existing verified metadata snapshot;
the topic grouping is an editorial navigation aid. The review's search cutoff is
**22 September 2026**. This archive does not claim a new literature search or new
full-text review. A verified identity is not an experimental replication.

Use browser Find to search a title or citation key. DOI links resolve to publisher
records; arXiv links retain the versions recorded by the review. Access to full
texts may depend on the publisher. See the reading audit for inspection depth.

### Surveys, evaluation principles and social behavior

Start here to frame research questions, human-aware behavior and evaluation criteria.

| Year | Paper | Citation key | Recorded inspection |
| --- | --- | --- | --- |
| 2024 | [A survey on socially aware robot navigation: Taxonomy and future challenges](https://doi.org/10.1177/02783649241230562) | `survey2024` | metadata/abstract |
| 2013 | [Human-aware robot navigation: A survey](https://doi.org/10.1016/j.robot.2013.05.007) | `survey2013` | metadata/abstract |
| 2025 | [Social robot navigation: a review and benchmarking of learning-based methods](https://doi.org/10.3389/frobt.2025.1658643) | `survey2023` | metadata/abstract |
| 2025 | [Principles and Guidelines for Evaluating Social Robot Navigation Algorithms](https://doi.org/10.1145/3700599) | `guidelines` | metadata/abstract |
| 2026 | [Advancing a taxonomy of proxemics for socially aware robot navigation](https://doi.org/10.3389/frobt.2026.1800762) | `proxemics2026` | metadata/abstract |

[Back to the paper index](#complete-paper-index) · [Resource library](README.md)

### Classical avoidance and control foundations

Motion constraints, crowd interactions and safety-oriented control foundations.

| Year | Paper | Citation key | Recorded inspection |
| --- | --- | --- | --- |
| 1997 | [The dynamic window approach to collision avoidance](https://doi.org/10.1109/100.580977) | `dwa` | metadata/abstract |
| 1998 | [Motion Planning in Dynamic Environments Using Velocity Obstacles](https://doi.org/10.1177/027836499801700706) | `vo` | metadata/abstract |
| 2011 | [Reciprocal n-Body Collision Avoidance](https://doi.org/10.1007/978-3-642-19457-3_1) | `orca` | metadata/abstract |
| 2013 | [Efficient trajectory optimization using a sparse model](https://doi.org/10.1109/ecmr.2013.6698833) | `teb` | metadata/abstract |
| 1995 | [Social force model for pedestrian dynamics](https://doi.org/10.1103/physreve.51.4282) | `socialforce` | metadata/abstract |
| 2013 | [Robot navigation in dense human crowds: the case for cooperation](https://doi.org/10.1109/icra.2013.6630866) | `freezing` | metadata/abstract |
| 2016 | [Safe navigation strategies for a biped robot walking in a crowd](https://doi.org/10.1109/humanoids.2016.7803304) | `biped2016` | metadata/abstract |
| 2017 | [Control Barrier Function Based Quadratic Programs for Safety Critical Systems](https://doi.org/10.1109/tac.2016.2638961) | `cbf` | metadata/abstract |
| 2022 | [Safe Learning in Robotics: From Learning-Based Control to Safe Reinforcement Learning](https://doi.org/10.1146/annurev-control-042920-020211) | `safecontrol` | metadata/abstract |

[Back to the paper index](#complete-paper-index) · [Resource library](README.md)

### Learned crowd and dynamic navigation

Learned interaction models and policies for navigating dynamic environments.

| Year | Paper | Citation key | Recorded inspection |
| --- | --- | --- | --- |
| 2017 | [Decentralized non-communicating multiagent collision avoidance with deep reinforcement learning](https://doi.org/10.1109/icra.2017.7989037) | `cadrl` | metadata/abstract |
| 2019 | [Crowd-Robot Interaction: Crowd-Aware Robot Navigation With Attention-Based Deep Reinforcement Learning](https://doi.org/10.1109/icra.2019.8794134) | `sarl` | metadata/abstract |
| 2020 | [Relational Graph Learning for Crowd Navigation](https://doi.org/10.1109/iros45743.2020.9340705) | `rgl` | metadata/abstract |
| 2021 | [Decentralized Structural-RNN for Robot Crowd Navigation with Deep Reinforcement Learning](https://doi.org/10.1109/icra48506.2021.9561595) | `dsrnn` | metadata/abstract |
| 2025 | [End-to-End Humanoid Robot Safe and Comfortable Locomotion Policy](https://arxiv.org/abs/2508.07611v1) | `safehumanoid` | full-text: methods and evaluation |
| 2026 | [FLUX: Accelerating Cross-Embodiment Generative Navigation Policies via Rectified Flow and Static-to-Dynamic Learning](https://arxiv.org/abs/2603.12806v1) | `flux` | full-text: methods and evaluation |
| 2025 | [Omni-Perception: Omnidirectional Collision Avoidance for Legged Locomotion in Dynamic Environments](https://arxiv.org/abs/2505.19214) | `omni` | abstract |

[Back to the paper index](#complete-paper-index) · [Resource library](README.md)

### Perception, localization and motion prediction

Human observations, odometry and forecasts that navigation decisions depend on.

| Year | Paper | Citation key | Recorded inspection |
| --- | --- | --- | --- |
| 2023 | [JRDB: A Dataset and Benchmark of Egocentric Robot Visual Perception of Humans in Built Environments](https://doi.org/10.1109/tpami.2021.3070543) | `jrdb` | metadata/abstract |
| 2017 | [DROW: Real-Time Deep Learning-Based Wheelchair Detection in 2-D Range Data](https://doi.org/10.1109/lra.2016.2645131) | `drow` | metadata/abstract |
| 2020 | [DR-SPAAM: A Spatial-Attention and Auto-regressive Model for Person Detection in 2D Range Data](https://doi.org/10.1109/iros45743.2020.9341689) | `drspaam` | metadata/abstract |
| 2022 | [FAST-LIO2: Fast Direct LiDAR-Inertial Odometry](https://doi.org/10.1109/tro.2022.3141876) | `fastlio` | metadata/abstract |
| 2021 | [ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual–Inertial, and Multimap SLAM](https://doi.org/10.1109/tro.2021.3075644) | `orbslam` | metadata/abstract |
| 2020 | [Human motion trajectory prediction: a survey](https://doi.org/10.1177/0278364920917446) | `prediction` | metadata/abstract |
| 2016 | [Social LSTM: Human Trajectory Prediction in Crowded Spaces](https://doi.org/10.1109/cvpr.2016.110) | `sociallstm` | metadata/abstract |
| 2018 | [Social GAN: Socially Acceptable Trajectories with Generative Adversarial Networks](https://doi.org/10.1109/cvpr.2018.00240) | `socialgan` | metadata/abstract |
| 2020 | [Trajectron++: Dynamically-Feasible Trajectory Forecasting with Heterogeneous Data](https://doi.org/10.1007/978-3-030-58523-5_40) | `trajectron` | metadata/abstract |

[Back to the paper index](#complete-paper-index) · [Resource library](README.md)

### Legged locomotion, whole-body control and transfer

Locomotion learning, dynamics adaptation and whole-body skills; embodiments differ.

| Year | Paper | Citation key | Recorded inspection |
| --- | --- | --- | --- |
| 2021 | [RMA: Rapid Motor Adaptation for Legged Robots](https://doi.org/10.15607/rss.2021.xvii.011) | `rma` | metadata/abstract |
| 2021 | [Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning](https://arxiv.org/abs/2109.11978) | `rudin` | abstract |
| 2022 | [Learning robust perceptive locomotion for quadrupedal robots in the wild](https://doi.org/10.1126/scirobotics.abk2822) | `rough` | metadata/abstract |
| 2024 | [ANYmal parkour: Learning agile navigation for quadrupedal robots](https://doi.org/10.1126/scirobotics.adi7566) | `parkour` | metadata/abstract |
| 2024 | [Learning robust autonomous navigation and locomotion for wheeled-legged robots](https://doi.org/10.1126/scirobotics.adi9641) | `wheeledleg` | metadata/abstract |
| 2024 | [Expressive Whole-Body Control for Humanoid Robots](https://doi.org/10.15607/rss.2024.xx.107) | `expressive` | metadata/abstract |
| 2018 | [DeepMimic](https://doi.org/10.1145/3197517.3201311) | `deepmimic` | metadata/abstract |
| 2018 | [Sim-to-Real Transfer of Robotic Control with Dynamics Randomization](https://doi.org/10.1109/icra.2018.8460528) | `domain` | metadata/abstract |
| 2025 | [ASAP: Aligning Simulation and Real-World Physics for Learning Agile Humanoid Whole-Body Skills](https://arxiv.org/abs/2502.01143v3) | `asap` | full-text: methods and evaluation |

[Back to the paper index](#complete-paper-index) · [Resource library](README.md)

### Benchmarks, datasets and embodied environments

Resources for designing comparisons; inclusion does not mean Geoff-Sim integrates them.

| Year | Paper | Citation key | Recorded inspection |
| --- | --- | --- | --- |
| 2022 | [SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation](https://doi.org/10.1145/3476413) | `socnavbench` | metadata/abstract |
| 2024 | [SOCIALGYM 2.0: Simulator for Multi-Robot Learning and Navigation in Shared Human Spaces](https://doi.org/10.1609/aaai.v38i21.30562) | `socialgym` | metadata/abstract |
| 2022 | [Socially CompliAnt Navigation Dataset (SCAND): A Large-Scale Dataset of Demonstrations for Social Navigation](https://doi.org/10.1109/lra.2022.3184025) | `scand` | metadata/abstract |
| 2026 | [Beyond Isolation: A Unified Benchmark for General-Purpose Navigation](https://arxiv.org/abs/2605.09441v1) | `omninav` | full-text: methods and evaluation |
| 2024 | [MetaUrban: An Embodied AI Simulation Platform for Urban Micromobility](https://arxiv.org/abs/2407.08725) | `metaurban` | abstract |
| 2023 | [Habitat 3.0: A Co-Habitat for Humans, Avatars and Robots](https://arxiv.org/abs/2310.13724) | `habitat3` | abstract |
| 2026 | [DPed-VLN: A Benchmark for Socially Compliant Vision-and-Language Navigation in Dynamic Pedestrian Environments](https://arxiv.org/abs/2609.21504) | `dped` | abstract |

[Back to the paper index](#complete-paper-index) · [Resource library](README.md)

### Core software and simulation foundations

Primary papers behind the simulation and ROS navigation ecosystem.

| Year | Paper | Citation key | Recorded inspection |
| --- | --- | --- | --- |
| 2012 | [MuJoCo: A physics engine for model-based control](https://doi.org/10.1109/iros.2012.6386109) | `mujoco` | metadata/abstract |
| 2022 | [Robot Operating System 2: Design, architecture, and uses in the wild](https://doi.org/10.1126/scirobotics.abm6074) | `ros2` | metadata/abstract |
| 2020 | [The Marathon 2: A Navigation System](https://doi.org/10.1109/iros45743.2020.9341207) | `nav2` | metadata/abstract |
| 2021 | [SLAM Toolbox: SLAM for the dynamic world](https://doi.org/10.21105/joss.02783) | `slam` | metadata/abstract |

[Back to the paper index](#complete-paper-index) · [Resource library](README.md)

## Maintaining the index

This page is generated by `scripts/resource_library.py` through the shared
graphics/documentation generator. Update the reviewed bibliography using its
[verification procedure](../literature_review/README.md#verification-and-updating),
then update the topic key groups if papers were added or removed. Regenerate with
`python3 scripts/generate_readme_graphics.py`; `--check` detects stale output and
rejects missing, duplicate or unclassified paper keys.
