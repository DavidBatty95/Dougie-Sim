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
