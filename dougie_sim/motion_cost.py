"""Shared conservative route-cost assumptions, distinct from command limits.

Forecast speed is an effective speed including gait tracking loss. Retain the
existing conservative value until held-out travel-time calibration supports a
change; faster commands alone do not justify reducing the return reserve.
"""
FORECAST_SPEED = .06
FORECAST_YAW_RATE = .35


def turn_duration(a, b, yaw_rate=FORECAST_YAW_RATE):
    import math
    import numpy as np
    if np.linalg.norm(a)*np.linalg.norm(b) <= 1e-12:
        return 0.
    return abs(math.atan2(a[0]*b[1]-a[1]*b[0], float(a@b)))/yaw_rate
