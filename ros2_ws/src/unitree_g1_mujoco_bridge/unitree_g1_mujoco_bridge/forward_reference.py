"""Route-following preference; collision checking remains with the planner."""
import math


def forward_reference(local_x, local_y, speed, max_yaw=.35):
    """Turn toward route lookahead, then walk forward without sustained strafing.

    A 20 degree alignment gate avoids traversing corners while facing across
    the route. Close-person escape candidates remain independently available.
    """
    if not all(math.isfinite(v) for v in (local_x, local_y, speed, max_yaw)):
        return 0., 0., 0.
    if math.hypot(local_x, local_y) < 1e-6:
        return 0., 0., 0.
    heading = math.atan2(local_y, local_x)
    angular = max(-max_yaw, min(max_yaw, 1.2*heading))
    alignment = max(0., math.cos(heading))
    forward = max(0., speed)*alignment*alignment if abs(heading) < math.pi/9 else 0.
    return forward, 0., angular
