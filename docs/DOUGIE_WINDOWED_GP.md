# Online, spatially windowed radiation GP

The paired demo and seed-study runner now default to `--gp-mode windowed`.
Instead of one mission-wide matrix/history, overlapping local geometry GPs update
only where observations arrive. Inactive posteriors are paged to disk and restored
when needed; they are never silently replaced with an untrained prior.

## Working set and retained map

| Setting | Default |
|---|---|
| XYZ cell size / overlap halo | 4 × 4 × 1.5 m / 20% |
| Inducing capacity | 32 per local GP |
| Resident working set | At most 16 local GPs |
| Exact local geometry replay | 256 detector windows |
| Local observed plane features | At most 8 |
| Global sample or expert cutoff | None in this mode |

The global window catalogue and disk storage grow with explored space; active
matrix/history arrays stay bounded. This is not constant total storage or an
unlimited-resource guarantee. Navigation, geometry mapping and localization still
have independent limits. Unknown queries retain an uncertainty-bearing prior and
do not allocate new experts or fabricate zero radiation.

Cold prediction queries stream one compact, read-only posterior at a time. They
do not load detector replay histories, allocate regions, change the training LRU
or evict active observation windows. Exact histories are loaded only when a
region actually needs an update. There is one transient prediction workspace in
addition to the configured resident training set.

## Online observations

A moving detector window stays one integrated Poisson count across its trajectory
quadrature. Each overlapping expert receives that observation once; counts are
not split into invented independent measurements. Experts are correlated, so
prediction uses mixture moments and disagreement variance, not a product of
independent precisions. The conservative upper envelope remains available.

Only geometry around the sensor's active windows updates their kernels. When an
expert's exact replay history fills, its geometry-feature kernel is fixed but
**count/posterior updates continue indefinitely**. It does not stop learning or
trigger return. This avoids rebuilding a kernel from incomplete history. Raw
detector observations remain in the mission archive for offline analysis.

## Initial calibration

The old lognormal prior's median was 2 µSv/h, but its variance made the unobserved
mean approximately **6.16 µSv/h**. This can cause costly detours in low-rate scenes.
The windowed model estimates initial intensity from the first eight count windows
(or fewer for a smaller history), and sets the prior's expected rate to that
measured intensity. Variance is retained. Only the small initial set is rebuilt,
from complete local histories; subsequent observations are local streaming updates,
not recurring global refits. No true sources, field or reference coverage enter
this calibration. The initial prior can still be wrong in another room or floor.

## Continuous field interpolation

New spatial/windowed GPs use **quintic partition-of-unity blending** instead of
linear overlap weights. The transition `6t^5 - 15t^4 + 10t^3` has zero first and
second derivatives at each support edge. Local posterior means and uncertainty
therefore join smoothly across compute-window boundaries; the windows do not
become physical radiation cells. Every XYZ query still evaluates the GP kernels,
including the observed-geometry features, rather than snapping to a measurement.

Rate means remain correlated-expert mixture means. Log variance includes both
expert variance and disagreement. Overlap never multiplies independent precision
or declares an unmeasured region certain. The upper estimate keeps the maximum
of learned active experts; its influence tapers smoothly in an expanded support
outside the ordinary mixture halo. This removes the previous upper-envelope jump
when a neighbour entered with an infinitesimal weight, while retaining the hot
expert's full upper estimate throughout its ordinary overlap. These remain
model-based planning estimates, not calibrated mission-wide safety bounds.
Unallocated windows contribute their uncertainty-bearing mixture prior, not a
full untrained upper envelope at zero-weight measured locations. This prevents
spurious return-reserve floors without discarding uncertainty in unseen space.

The live comparison exports **up to 8,192 direct GP queries** over observed
support, sampled at approximately 25 cm horizontally with separate height bins.
Actual support returns supply the query positions; evaluator floor references
and hidden world geometry are not passed to planning. Real detector positions
are retained in the display budget to resolve measured local peaks. These are
prediction queries, not additional observations or GP training updates.

The dashboard and 3D cloud interpolate exported predictions using compact,
smooth Wendland–Shepard weights. All supported neighbours contribute, avoiding
nearest-neighbour patches and a hard k-nearest switch. Display uncertainty retains
local disagreement instead of combining correlated predictions as independent
evidence. Floor-map interpolation has 35 cm horizontal and 18 cm vertical support;
uncovered gaps remain null/grey, never zero. The planner uses the GP itself, not
these display interpolations. The display identifies interpolation versus a
retained legacy nearest-query snapshot explicitly.

New spatial/windowed checkpoints use schema 3 and store both blending and upper
rules. Schema-1 checkpoints retain linear blending; schema-2 checkpoints retain
their recorded expanded-prior envelope. Reloading a new checkpoint preserves the
continuous field, evidence-aware bounds and lazy paging behavior.

### Read-only reconstruction check

The [retained benchmark](results/dougie_continuous_gp_20261112.json) reconstructs
one completed facility posterior. It is **not a new navigation trial, dose benefit
or accuracy calibration**. Dense interpolation provides estimates for 8,168 of
8,292 reference cells versus 2,889 in the retained sparse display. This is
**prediction availability**, not newly explored floor or measured radiation.

Across 288 seam/overlap probes separated by 2 micrometres, the largest upper-rate
change was 16.9674 µSv/h with the old envelope and 0.0003283 µSv/h with continuous
blending. The mean and log-rate uncertainty also remained continuous. In this
Linux ARM64 container, 8,192 predictions took about 0.112 s, and map interpolation
about 0.094 s. Prediction left zero training experts resident in the lazy-loaded
model; it did not fill the training working set with display queries.

```bash
python3 scripts/benchmark_continuous_gp.py \
  results/dougie/comparison_20261111T214218_843825Z/aware
```

## Why the facility comparison accumulated more aware dose

The retained local trial `comparison_20261110T094741_064137Z` reached its first
≥50% reference-floor observation at 277.34 s / 0.17010 µSv (blind) and
373.79 s / 0.20334 µSv (aware). The aware robot experienced an approximately
11% lower trajectory-average rate but spent 35% longer reaching that threshold:
integrated dose therefore increased by 19.5%. The frontier survey branch had
bypassed the revisit-aware utility and region scheduler, allowing persistent
frontier loops. Its exposure objective also penalized a GP upper envelope rather
than expected dose. Finally, the global GP's 8,192-window replay limit forced an
aware return, while the blind GP had no equivalent limit. Those unequal endpoint
totals cannot measure an awareness advantage.

In these `--fast` trials, dose is integrated over simulated time: slower GP wall
time does not by itself add dose. The additional simulated motion, turning and
repeated observations do. The corrections above address computation, model
calibration and planning separately. Matched-work, per-floor and multi-seed
outcomes must still be examined; lower actual dose is not guaranteed in every
noisy, partially observed mission.

The read-only `scripts/diagnose_dougie_exposure.py` summarizes retained count
windows, planning decisions, repeated targets and matched milestones. The
synthetic `scripts/benchmark_windowed_gp.py` checks streaming updates and retained
paged beliefs separately from physics, geometry fitting and navigation workloads.

## Dose objective and exploration efficiency

Uncapped `minimize-dose` routing and smoothing use the **posterior mean** as
expected exposure. Hard-budget/rate-limited profiles retain upper-bound admission,
and uncertainty is still displayed. Minimizing an uncertainty envelope is a
different objective from minimizing expected accumulated dose.

Soft route costs use an observed-rate reference rather than a fixed 50 µSv/h
scale. Frontier scoring includes travel, expected dwell exposure, useful unknown
support, observed connectors and time already spent at a viewpoint. Region/height
candidates remain present during multilevel frontier sampling.

Repeated cycles among the same frontier regions without new observed support can
trigger `frontier_progress_stalled`. Remaining frontiers are retained and the
outcome is **incomplete**, not finished. Completion still requires zero frontiers
on three fresh scans. No method forces awareness to win: model error, necessary
hotspot visits and extra travel can still yield higher actual dose in a seed.

## Configuration and evidence

```bash
./demo
./dougie compare --world facility --gp-mode windowed --gp-expert-capacity 32 --gp-max-experts 16
```

Here `--gp-max-experts` means **resident** windows, not a global map limit.
`geometry`, `spatial` and `global` remain available as declared ablations with
their legacy limits. Old seed cohorts cannot be resumed under changed source
hashes: use a new output directory for this profile.

GP metrics include total/resident windows and inducing points, active matrix and
history bytes, affected experts per update, paging counts, update timings and
calibration state. `frontier_decision` exposes prediction type, observed rate
reference and shortlisted costs/revisit weights.

`.gp_windows/` holds inactive regional models. `gp_posterior.npz` is a
self-contained checkpoint, written one window at a time. `load_gp()` in
`dougie_sim.spatial_gp` restores it lazily without pickle or loading the entire
map into RAM. Tests cover local-only updates, unsplit counts, bounded residency,
retained predictions after eviction, learning past history capacity, lazy
checkpoint restoration and count-only prior calibration.
