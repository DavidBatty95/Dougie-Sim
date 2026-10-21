# Robust onboard mission stages

## Stage 1 — registration quality and consistent maps

`--navigation-profile robust` is the new development profile; `legacy` retains
the previous fusion/mission comparator. Both consume onboard observations only.

- Atomic point-to-plane registration rejects insufficient correspondences,
  inconsistent normals, excessive residuals and large pose innovations. A failed
  fit cannot partially move the estimate or contaminate the next registration map.
- Up to 64 fixed observed keyframes replace continuously overwritten registration
  voxels. Initial anchors are retained. Weak geometry is reported by observable
  rank rather than supplying fictitious motion constraints.
- Stance-velocity disagreement excludes individually slipping feet. Pose allowance
  now includes travelled distance and registration innovations, and local residuals
  add clearance margin. This is a more conservative heuristic, **not calibrated
  probabilistic uncertainty**.
- Revisits require two consistent, well-conditioned matches against an older
  observed keyframe. At most 16 loop corrections are accepted. Corrections are
  distributed in time from an unchanged anchor; this is approximate bounded
  loop correction, not a full pose-graph optimizer.
- A versioned observation journal reprojects terrain scans and detector motion
  windows and rebuilds the GP before executing routes in the new frame. Partial
  detector windows, display evidence and trails use the same correction. Counts,
  exposure durations and measured cumulative dose are never changed. Pending
  old-revision plans are withheld.
- History is bounded at 1,024 planning scans / 8,192 count windows. Capacity
  exhaustion triggers return; corrections cannot silently drop old observations.
- Severe pose-quality failures withhold spatial GP updates but retain measured
  dose accounting. Home approach uses a tighter .2 m estimated tolerance with a
  separately support-checked final segment inside the last graph cell.

Tests cover known-transform recovery, unobservable motion on a plane, rejected
scan isolation, confirmed loop correction, shared scan/count revisions, partial
window conservation, single-assimilation worker replay and the final home segment.
The full suite passed 222 checks before adding the home-segment regression; all
18 affected mapping/odometry/archive checks then passed.

Physical screening (90 s cap, 2 uSv, detector/sensor 3/0, fixed .5 s plan delay):
`robust_fusion_stage1_seed3_homefix` returned cleanly at **43.050 s**, **4.976 m**,
**.109965 uSv**, no non-foot contacts, **.1452 m** physical home error. Position
RMSE/p95: **.0510/.0924 m**. No real loop closure occurred in this short run;
loop/reprojection evidence at this stage is synthetic. Concurrent test workloads
make the recorded wall timings unsuitable as an isolated performance comparison.

The preceding `robust_fusion_stage1_seed3` is retained as a failed return: the
tighter home tolerance exposed a coarse-cell stopping deadlock, fixed by the
checked final approach. Longer missions and matched completed work remain the
acceptance gates for the following stages.
