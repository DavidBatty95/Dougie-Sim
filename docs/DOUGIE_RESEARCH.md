# Dougie-Sim research reading guide

[Project home](../README.md) · [Methods](DOUGIE_RADIATION.md) · [Evidence](DOUGIE_VALIDATION.md) · [Documentation](DOUGIE_INDEX.md)

## Research question

**Can a physically walking quadruped collect useful geometric and radiation
information in an unknown facility while retaining an exposure-aware route home?**

The current contribution is an inspectable simulation integration: counting
observations, a bounded XYZ GP, observed-surface routes and cumulative-exposure
reasoning all run in one closed loop. Novel algorithmic superiority has not been
established. The strongest current evidence is the small initial cohort and
its retained artifacts, rather than a demonstration image.

## Concepts before papers

### Rate is not dose

A rate threshold controls a local observation; a dose budget concerns the entire
trajectory. For a rate in µSv/h and time in seconds:

```text
dose_usv = integral(rate_usv_h(t) dt) / 3600
```

For example, remaining at 10 µSv/h for 72 seconds accumulates 0.2 µSv even though
the rate stays far below a 100 µSv/h threshold. A robot also consumes exposure
while returning, waiting for a plan or moving more slowly than expected.
Dougie therefore combines cumulative counts with predicted outbound/return costs.
The proxy units describe the simulated experiment, not electronics survival.

### A zero count does not prove a zero field

Radioactive detection is modeled as a counting process. If latent log-rate is
`f(x)`, sensitivity is `s` counts/µSv and the integration window is `dt` seconds:

```text
counts ~ Poisson(s * dt / 3600 * exp(f(x)))
```

Low counts are asymmetric and noisy. A log field keeps rates positive, while a
Poisson likelihood retains the integer-count interpretation. Dougie integrates
the simulated intensity along motion, then associates the count observation with
the window's mean position; this spatial approximation matters near sharp edges.

### A GP is a field estimate with assumptions

A Gaussian process describes correlated latent values and uncertainty at query
locations. Dougie uses separate vertical/horizontal correlation scales in a
Matérn-3/2 kernel. Nearby data can reduce uncertainty, but a smooth mean can hide
an abrupt shielding transition. Fixed hyperparameters and approximate inference
need empirical checking; credible bounds are not automatically trajectory-wide
safety guarantees.

### Bounded inference and bounded missions are different

The inducing dictionary is capped at 128 locations by default. Once full, it
stays fixed; subsequent updates do not revisit the entire sample history.
Between admissions, posterior update work is O(M²). This bounds the inference
state, not all geometry storage or the complete planner's runtime. The published
GP benchmark excludes sensor processing and navigation.

### XYZ prediction is not volumetric observation

The robot walks on connected support surfaces embedded in 3-D. It cannot move
freely through an arbitrary volume. Likewise, ground-height detector samples do
not independently constrain all upper floors. The initial runs sampled around
0.4 m height; height coverage must accompany any volumetric reconstruction claim.

## Annotated reading path

This supplements the [methods review's inspection ledger](DOUGIE_RADIATION.md#research-findings-and-design-choices).
For this documentation update, West's primary article and POAM's primary HTML
were retrieved and their relevant methods inspected on **7 October 2026**.
Other entries below use the inspection scope already recorded in that ledger;
this is a focused reading path, not an exhaustive or systematic review.

### 1. Radiation reconstruction from real surveys

**West et al. (2021), “Use of Gaussian process regression for radiation mapping
of a nuclear reactor with a mobile robot.” Scientific Reports 11, 13975.**
[Primary open-access article](https://doi.org/10.1038/s41598-021-93474-4)

- **Problem:** irregular, repeated, noisy low-count measurements collected by a
  mobile robot do not fit assumptions of many simple interpolation methods.
- **Method/evidence:** a Jackal-based platform surveyed a neutron laboratory and
  the JSI TRIGA reactor. The work uses positive log-space estimates, a Poisson
  likelihood and a Matérn-3/2 covariance with background bias; transport modeling
  supports the laboratory comparison.
- **Important distinction:** the paper notes its detector was not calibrated and
  uses count rate, with dose estimates requiring conversion. It also identifies
  the O(N³) GP cost as a limitation for large-data online use. Detector height was
  fixed, making its regression spatially 2-D.
- **Dougie connection:** motivates the count likelihood and positive field.
  Dougie adds its own bounded streaming approximation and synthetic XYZ setting;
  it does not reproduce the reactor survey or its radiation-transport validation.

Read the gamma monitoring and reconstruction sections before comparing a
colorful radiation map to calibrated dosimetry.

### 2. Hazard-aware exploration

**Stephens et al. (2024), safe exploration using Gaussian-process prediction.**
[Primary record](https://doi.org/10.1007/s10514-024-10172-6) ·
[Oxford Robotics Institute field account](https://ori.ox.ac.uk/groups/goals/news-goals/exploring-hazardous-environments-with-mobile-robots)

- **Relevance:** decisions must account for uncertain hazardous regions and
  future reachability, rather than merely seeking the most informative sample.
- **Inspection boundary:** the existing review inspected bibliographic/abstract
  material and the institutional account; Springer full-text access was challenged.
  The reported Spot field trial used simulated radiation.
- **Dougie connection:** a SafeEstMDP-style method is a proposed comparator.
  Dougie's upper-rate route costs and reserves are heuristic and should not be
  attributed the guarantees of another planner. Local rate constraints and
  cumulative dose constraints need separate evaluation.

### 3. Efficient online nonstationary mapping

**Chen, Liu and Khardon (2024), “POAM: Probabilistic Online Attentive Mapping
for Efficient Robotic Information Gathering.” RSS 2024.**
[Primary HTML, version 1](https://arxiv.org/html/2406.03669v1) ·
[Authors' implementation](https://github.com/Weizhe-Chen/POAM)

- **Problem:** online information gathering needs both accurate uncertainty and
  manageable computation as observations accumulate.
- **Method:** an attentive nonstationary kernel, pivoted-Cholesky inducing
  selection and variational expectation-maximization coordinate model updates.
  Sections III–IV distinguish sparse inference, inducing updates and
  hyperparameter optimization. Mini-batch hyperparameter optimization can access
  the accumulated dataset; constant-time update work is not a claim that no
  historical data is retained.
- **Evidence:** active bathymetric mapping, with accuracy, uncertainty and runtime
  comparisons. This is not count-based nuclear mapping or dose-return validation.
- **Dougie connection:** bounded computation is shared motivation. Dougie uses
  fixed ARD Matérn hyperparameters and a dictionary that freezes at capacity,
  rather than POAM's full update scheme. Attentive or shielding-aware kernels are
  prospective comparisons near radiation discontinuities.

### 4. Streaming mapping during motion

**Sudha, Jose and Coates (ECMR 2025), streaming GP mapping and planning.**
[Primary HTML, version 2](https://arxiv.org/html/2507.13053v2)

The existing review inspected the methods, Tables I–II and limitations. Its
static 2-D field setting motivates continual mapping while moving; inducing
count can grow with the observations, while 3-D/dynamic extensions remain future
work. Dougie's fixed-capacity XYZ formulation offers a different engineering
tradeoff. A fair comparison must match memory and compute budgets as well as
measurement trajectories.

### Follow-up screening

[Psomiadis, Maity and Tsiotras, β-sparse GPs (2026)](https://arxiv.org/abs/2605.26304v2)
is retained in the existing review as **abstract/bibliographic screening only**.
Task-aware inducing selection under cardinality constraints is potentially useful,
but its multi-robot setting does not validate a single Go2 radiation mission.
Full appraisal and reproduction would be required before selecting it as a baseline.

## From papers to implementation

| Design issue | Active source | What to examine |
|---|---|---|
| Counting and exposure | [radiation.py](../dougie_sim/radiation.py) | Window integration, count variance, measured versus evaluator dose |
| Sparse positive field | [gp.py](../dougie_sim/gp.py) | Poisson/Laplace update, capacity, prediction residual uncertainty |
| Observed support | [terrain.py](../dougie_sim/terrain.py) | Multiple heights, connectivity and footprint support |
| Information versus return cost | [dose_planner.py](../dougie_sim/dose_planner.py) | Candidate score, travel assumptions, reserve and return triggers |
| Closed-loop execution | [radiation_run.py](../dougie_sim/radiation_run.py) | Worker lifecycle, stale-input gate, traces and evaluation |
| Physical embodiment | [simulation.py](../dougie_sim/simulation.py) | Policy contract, actual joint torques and ideal-sensor assumptions |

## Proposed evaluation programme

These experiments are **planned**, beyond the initial three noise-seed runs.

| Question | Controlled comparison | Report |
|---|---|---|
| Does radiation awareness help? | Blind frontiers / mean risk / upper risk, same layout and detector seeds | Coverage–actual-dose curves, return success, exceedances and falls |
| Does the likelihood matter? | Poisson versus Gaussian approximation, identical observations and capacity | Held-out count prediction, rate error, calibration and update latency |
| Can the model handle shielding? | Fixed Matérn / nonstationary or shielding-aware alternative, held-out layouts | Hotspot underestimation, error near boundaries, compute/memory |
| Are reserves realistic? | Fixed versus empirically fitted travel times, controlled delays/obstructions | Return prediction error, deadline misses, dose overshoot |
| Does the system generalize in Z? | Distinct sampling heights and unseen-height queries | Error/uncertainty by height, actual sampled support and coverage |
| How much does ideal pose help? | Ground truth versus noisy/estimated pose, matched trajectories | Localization error, field degradation, return reliability |

Keep source/configuration hashes, report all attempted runs and separate noise
repeats from changed layouts. Predefine evaluation conditions and retain failures.
GP-only latency is not end-to-end response time; separate synthetic held-out
benchmarks from the measurements collected by the robot's policy.

## Practical reference library

- [MuJoCo documentation](https://mujoco.readthedocs.io/en/stable/): physics, rendering and model APIs.
- [ROS 2 Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html): topics, transforms and tooling.
- [Nav2 documentation](https://docs.nav2.org/): the separate planar navigation profile.
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox): planar scan mapping.
- [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco): upstream model provenance.
- [rl_sar](https://github.com/fan-ziqi/rl_sar): Go2 policy deployment contract.
- [Gaussian Processes for Machine Learning](https://gaussianprocess.org/gpml/): background on kernels, inference and likelihoods.

For implemented equations and settings, use [Dougie's methods](DOUGIE_RADIATION.md).
For claims supported by measurements, use [Dougie's validation](DOUGIE_VALIDATION.md).
