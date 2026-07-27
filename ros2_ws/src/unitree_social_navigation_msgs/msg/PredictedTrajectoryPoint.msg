# Non-negative offset from the containing message's header stamp.
builtin_interfaces/Duration time_offset

# Predicted state in the frame of the containing message.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# Row-major covariance of position (x, y, z), in square meters.
float64[9] position_covariance

# Confidence in this prediction point in [0, 1].
float32 confidence
