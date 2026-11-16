# Compact occupancy lookup for multilevel exploration

Layered missions maintain fine 20 cm occupancy evidence plus compact confirmed-free
2×2×2 blocks. Frontier evaluation needs to ask whether space at an expected support
height was observed free. Previously, every frontier replan expanded each compact
block into eight Python tuple entries, then converted the resulting set into a
dense query grid when the local region was small enough. This repeatedly allocated
large temporary collections even though the underlying map was already compact.

`dougie_sim/occupancy_access.py` provides exact compact-map access:

- For bounded dense queries, build the boolean query grid directly from confirmed
  fine cells and coarse blocks, without constructing the expanded Python set.
- For sparse or very large extents, retain immutable sets of confirmed fine keys
  and coarse block keys. A query checks the fine key or its floor-divided block;
  it does not expand every block into children.
- Negative coordinates use floor division, and Z remains part of every key.
  Unknown, weak-evidence and occupied cells are not treated as observed free.
  A known lower floor cannot clear an unobserved upper floor.

This change is an occupancy **lookup/index optimization**. It does not merge
independently localized robot maps, replace the layered terrain graph, reduce the
resolution of stored geometry, modify the GP or alter frontier/return/safety rules.
The dense and sparse representations are snapshots for a planning cycle.

## Retained multilevel-map benchmark

The [occupancy index benchmark](results/dougie_occupancy_access_20261115.json)
uses a retained facility snapshot with **124,654 fine evidence cells** and
**5,748 compact free blocks**. The old and new dense grids matched exactly.
Untraced median index-build time was **65.5 ms → 17.5 ms**; peak traced index
allocation was **18.0 MB → 4.8 MB**. Sparse snapshot construction also matched
exactly across 50,000 sampled queries and reduced peak traced allocation from
10.3 MB to 2.6 MB. In this sample, those sparse queries took **24.5 ms versus
21.1 ms** for the already-expanded set: the sparse memory/build gain has a
measured query-time trade-off.

The separate [full frontier calculation](results/dougie_occupancy_planner_20261115.json)
reconstructed **7,483 observed support nodes** from the same map. Old/new produced
identical frontier gains **and identical unknown-cell witnesses for every node**
(190 frontiers). Frontier-index setup took 0.228 → 0.109 s and used 64.7 →
51.3 MB peak traced allocation. The subsequent frontier evaluation itself was
approximately unchanged (0.689 versus 0.694 s). These are single-machine,
read-only measurements, not a guarantee of end-to-end speedup or mission-dose
improvement. Graph reconstruction, GP and path planning have other costs.

```bash
python3 scripts/benchmark_occupancy_access.py \
  results/dougie/holdout_l2_s212_baseline_retry1/aware/layered_geometry.npz
python3 scripts/benchmark_occupancy_planner.py \
  results/dougie/holdout_l2_s212_baseline_retry1/aware
```

A [same-seed physical replay comparison](results/dougie_occupancy_mission_20261115.json)
is retained under `results/dougie/campaign_occupancy_equivalence_l2_s212/` and
compared with the pre-change `campaign_screen_pilot_l2_s212`. At the first ≥60%
crossing, both had identical simulated time **491.965 s**, observation **60.0217%**,
true dose **0.215172 µSv**, physical distance **67.930 m** and **288 frontiers**.
All **976 matching time samples** up to that crossing had zero differences in
those four metrics. Only `occupancy_access.py` (new) and `region_clustering.py`
changed between runtime source hashes. Both runs were externally interrupted
and are not completion outcomes. This is a behavior-equivalence diagnostic for
one seed/layout, not an independent navigation or dose-efficiency study.

```bash
python3 scripts/compare_occupancy_missions.py \
  results/dougie/campaign_screen_pilot_l2_s212/aware \
  results/dougie/campaign_occupancy_equivalence_l2_s212/aware --milestone 60
```
