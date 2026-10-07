# Radiation-aware unknown-environment exploration

[Project home](../README.md) · [Architecture](DOUGIE_SIM.md) · [Validation](DOUGIE_VALIDATION.md)

Research/implementation checkpoint: **7 October 2026**.

## Objective and scope

Maximize useful geometric coverage and radiation information while managing
cumulative exposure and retaining an estimated return budget. Finding the
strongest source is not itself the objective. Neither coverage alone nor a
smooth GP plot establishes mission success.

This is a **research-informed baseline**, not a claim to reproduce or outperform
the latest published system. It combines counting measurements, bounded GP
inference, observed 3-D traversability and dose-aware receding-horizon decisions.
It does not implement SafeEstMDP's MDP synthesis or POAM's attentive kernel/EM.

## Research findings and design choices

| Source | Inspected material | Relevance and boundary |
|---|---|---|
| [West et al., 2021: reactor radiation mapping](https://doi.org/10.1038/s41598-021-93474-4) | Primary methods, likelihood/kernel discussion and experimental setup | Irregular/repeated low-count observations, Matérn-3/2, positive log fields and Poisson likelihood. Jackal data from an actual reactor/laboratory, but fixed detector height and post-survey large-data fitting. Adopt the counting model, not claims of autonomous 3-D dose safety. |
| [Stephens et al., 2024: safe exploration with GP prediction](https://doi.org/10.1007/s10514-024-10172-6) | Crossref identity/abstract and [ORI field account](https://ori.ox.ac.uk/groups/goals/news-goals/exploring-hazardous-environments-with-mobile-robots) | Information, predicted hazards and return reasoning. Springer full text was access-challenged; Spot field trial used simulated radiation. Local rate thresholds differ from cumulative dose constraints. Required future comparator, not an implemented guarantee. |
| [Chen, Liu and Khardon, POAM, RSS 2024](https://arxiv.org/html/2406.03669v1) | Primary Sections III–IV and online-update assumptions | Bounded posterior work, inducing selection and nonstationary modeling. Bathymetric experiments, not reactor dosimetry. Attentive kernels and pivoted-Cholesky replacement are promising for shielding transitions; current kernel is fixed ARD Matérn. |
| [Sudha, Jose and Coates, ECMR 2025](https://arxiv.org/html/2507.13053v2) | Primary methods, Tables I–II and limitations | Streaming GP and planning while moving. Their study is static 2-D fields; inducing count grows with data and 3-D/dynamic environments remain future work. Dougie caps the dictionary. |
| [Psomiadis, Maity and Tsiotras, β-sparse GPs, 2026](https://arxiv.org/abs/2605.26304v2) | Bibliographic record and abstract only | Task-aware inducing selection under cardinality constraints. Multi-robot communication/navigation setting; screened as follow-up, not single-Go2 radiation validation. |

Searches used newest-first arXiv queries `radiation mapping Gaussian process`
and `online sparse Gaussian process robot mapping`, direct primary records and
citation tracing. Unrelated astronomy/medical results were excluded. This is
a targeted single-workflow search, not an exhaustive review. A newer date alone
does not establish a better radiation mapper. No external algorithm was replicated.

## Radiation and sensor model

The facility includes a vessel/hot-cell area, waste drums, pipes, cabinets,
concrete shielding, aisle markings, uneven plates and an inspection deck.
Source strength/core radius are MuJoCo site user fields; attenuation is stored
on shield box geometries. The simulator reads these to evaluate:

```text
rate(x) = background + Σ strength_i / (||x-source_i||² + core_radius_i²)
                         × exp(-Σ attenuation_j × length_inside_shield_j)
```

This is finite point-source falloff and straight-line exponential attenuation,
not scattering, neutron transport, activation, energy spectra or detector dead
time. MuJoCo integrates mechanics; the field module evaluates a radiation proxy.
It is not MCNP/Geant4 transport.

The detector integrates expected counts along actual motion, samples Poisson
counts every 0.5 s and records the integration-window mean position. Default
sensitivity is 50,000 counts/µSv. Rate is µSv/h; elapsed seconds are divided by
3,600 for µSv integration. These are illustrative dose-equivalent proxies, not
absorbed Gy or a calibration of Go2 electronics damage. The evaluator separately
integrates the underlying field; autonomy receives only detector observations.

## Online 3-D GP

Default: `PoissonOnlineGP` in `dougie_sim/gp.py`.

- Input `(x,y,z)` metres; latent `log(rate_usv_h)`.
- `counts ~ Poisson(sensitivity × integration_s / 3600 × exp(f))`.
- Matérn-3/2 ARD kernel, with separate vertical/horizontal correlation scales.
- Gaussian posterior over at most 128 inducing values by default. Conditional
  prior variance admits new locations; after capacity is reached the dictionary
  stays fixed. It is not an unbounded sample buffer.
- Each batch is used once: a scalar Laplace approximation is projected into the
  inducing posterior. Between admissions, work is O(M²), independent of total
  sample count. Prediction retains conditional prior residual uncertainty.
- Positive rate mean, log-rate uncertainty and upper credible rate are exported
  at XYZ query locations. A `--gp-likelihood gaussian` approximation is retained
  for comparison, not silently equated with Poisson inference.

Fixed hyperparameters and approximate inference can be misspecified near shield
boundaries or changing sources. Credible intervals are not simultaneous safety
guarantees along trajectories. Benchmark empirical coverage is reported rather
than assumed. Unsupported heights remain uncertain even after ground-floor data.

## Dose-aware exploration

Organized 3-D range returns and adjacent-ray normals provide near-horizontal
support hypotheses. The graph retains multiple Z levels per XY cell. Unknown
support is not traversable; large steps and known body-height obstacles are
rejected. It is a surface navigation graph embedded in 3-D, not a flying robot's
arbitrary free-space planner.

Candidate utility combines nearby unsensed support columns and GP uncertainty
with travel/dose costs. It is a heuristic, not globally optimal coverage.
Route exposure integrates upper GP rates with 0.1 m/s flat and 0.06 m/s step
travel assumptions. Admission requires measured-dose upper estimate + outbound
dose + known-route return dose + 15% budget reserve + a dwell allowance to fit
the budget. A return-time estimate also triggers retreat before the deadline.
High measured rate or 80% budget use triggers return. Stale sensors/plans gate motion.

Return can fail through blockage, a fall or model error. A hidden sharp hotspot
can expose the robot before the next sensor sample. All evaluator-detected
budget exceedances produce a failed exit and remain recorded.

## Next research gates

1. Compare radiation-blind frontiers, mean-only risk, upper-bound risk and a
   SafeEstMDP-style method under matched sensing/compute. Report coverage versus
   actual dose, return success and all failures.
2. Compare POAM-style attentive kernels, local GP experts and shielding-aware
   kernels with count likelihoods on held-out source/shield layouts.
3. Replace fixed travel assumptions with measured progress distributions and
   qualify a perceptive Go2 controller for normal industrial stair risers.
4. Replace ideal pose with LiDAR/inertial/leg odometry and propagate pose error
   into GP uncertainty and return costs.
5. Excite the Z dimension experimentally. Current dose trials mostly sample
   around 0.4 m height; volumetric prediction is not validated volumetric coverage.
6. Compare attenuation-only fields with transport-generated references and
   calibrated detector responses.

These are explicit future gates, not completed capabilities.
