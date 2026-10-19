# Onboard-only radiation mission development

## Sensor-fusion stage

Radiation missions now default to `--localization fused`. The explicit
`--localization ground-truth` comparator preserves the historical ideal-pose
experiments; ideal semantic landmarks are only available in that comparator.

The simulator sensor front-end emits body-frame gyroscope/specific acceleration,
joint encoders/velocities, foot touch forces, and a body-mounted organized LiDAR
scan. Separate seeded sensor noise includes gyro/accelerometer biases, encoder
noise, 2 mm range noise and .5% dropped beams. No frame-pose/quaternion/velocity
sensors, object IDs, source positions or shielding coefficients cross into fusion.
These are synthetic sensor models, not a calibrated reproduction of Go2 hardware.

The estimator combines IMU prediction, stationary-foot kinematics and stance
anchors with a bounded observed-cloud point-to-plane registration map. Registration
updates only observable eigendirections; it does not invent heading information
on an empty plane. It has no loop closure or global position reference. Reported
conditional filter covariance is supplemented by a separate drift allowance;
neither is a validated probability bound. The walking policy receives measured
proprioception and estimated gravity, not simulator orientation/velocity.

LiDAR is transformed with the estimate at capture time. Applying a later attitude
to an old scan caused mapping failure during development and is now rejected.
Physics pose and the hidden field remain inside sensor generation and independent
evaluation. `localization_evaluation.jsonl` is evaluator-only. Clean return now
also requires true physical home distance <= .4 m; an estimated return alone
cannot pass. ROS/world geometry displays remain diagnostic.

Sensor-only reactor screening: seed 3, sensor seed 0, 90 s limit, 2 uSv budget,
smooth/global, fixed .5 s planning delay. `onboard_fusion_reactor_v3` returned
cleanly at **43.050 s**, travelled **4.569 m**, used **.118689 uSv**, and had no
non-foot contacts. Position error RMSE/p95 was **.0715/.0922 m**, true home error
**.0693 m**. This is a short development pass, not a dose improvement comparison.
The earlier v1/v2/layered attempts are retained failures; snapshot timing and
in-motion gyro-bias calibration were corrected. No mission success is inferred
from standing still at a lower exposure.

Verification: **206 tests passed**, including physical walking with fused inputs,
URDF forward kinematics, body-frame sensor isolation, capture-time snapshots and
policy invariance to changed hidden base pose/velocity.

## Observed-geometry and moving-window inference

`--gp-mode geometry` is the new default. Up to 16 localized planar surface patches
are fitted from estimated-world LiDAR returns. The Matérn kernel is multiplied by
an RBF kernel in smooth, localized plane-side features. This product is positive
semidefinite; arbitrary ray-obstruction multipliers generally are not. It weakens
correlation across observed surfaces without assuming a wall's material,
attenuation coefficient, hidden radiation source, or a safe far-side dose.

When new geometry changes the kernel, the inducing posterior is rebuilt from
retained count windows (bounded at 8,192). Geometry processing is throttled to
10 simulated seconds and saturates at 16 patches. Count-history saturation stops
new exploration through the existing model-capacity return gate. Patches are
local statistical features, not a complete material map or radiation transport.

The default `--detector-likelihood window` preserves up to eight duration-weighted
estimated positions per detector integration interval. One Poisson likelihood
uses `lambda = counts_per_uSv / 3600 * sum(dt_i * exp(f_i))`. A joint-window
Laplace approximation is projected into the sparse posterior; counts are never
divided into independent pseudo-measurements. The likelihood is non-concave and
the approximation uses a conservative curvature floor. Quadrature approximates
motion inside each time bin. `--detector-likelihood point` and `--gp-mode global`
provide explicit ablations. Global, geometry and spatial models support windows.

Geometry archives retain the actual kernel features and count replay history;
`load_gp` restores both prediction and subsequent geometry rebuilds. Unit checks
cover PSD, uncertainty across an observed wall, replay equivalence, scalar-limit
equivalence, midpoint bias, and archive round trips. **212 tests passed**.

The combined sensor-only reactor smoke (`onboard_geometry_window_reactor_v1`)
returned cleanly at 47.050 s, .126188 uSv, 5.693 m, no non-foot contacts. Its
position RMSE/p95 was .0739/.1066 m. The earlier sensor-only global/point run had
less travel and less dose; these unequal survey trajectories do not establish a
dose reduction. Matched-objective comparisons are required below.
