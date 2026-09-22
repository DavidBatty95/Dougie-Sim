# Stable identifier assigned by the tracker.
uint64 track_id

# Pose and velocity are expressed in the frame of the containing message.
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity

# Detection or tracking confidence in [0, 1].
float32 confidence

# Approximate horizontal collision radius in meters.
float32 body_radius

uint8 TRACK_STATE_UNKNOWN=0
uint8 TRACK_STATE_TENTATIVE=1
uint8 TRACK_STATE_CONFIRMED=2
uint8 TRACK_STATE_LOST=3
uint8 track_state

# Time since the last accepted observation, measured at the containing header.
# A propagated LOST state is not a new observation.
builtin_interfaces/Duration observation_age
