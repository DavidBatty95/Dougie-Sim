"""Linear-time directed route-tree forecasts including turning exposure."""
import math

import numpy as np


def forecast_polyline(points, rates, speed=.06, yaw_rate=.35):
    """Time/exposure for an already geometry-checked execution polyline."""
    points, rates = np.asarray(points), np.asarray(rates)
    if len(points) < 2:
        return 0., 0.
    delta = np.diff(points, axis=0)
    edge_time = np.linalg.norm(delta, axis=1)/speed
    duration = float(edge_time.sum())
    dose = float(np.sum(np.maximum(rates[:-1], rates[1:])*edge_time))/3600.
    for i, (a, b) in enumerate(zip(delta[:-1, :2], delta[1:, :2]), 1):
        if np.linalg.norm(a)*np.linalg.norm(b) > 1e-12:
            turn = abs(math.atan2(a[0]*b[1]-a[1]*b[0], float(a@b)))/yaw_rate
            duration += turn
            dose += rates[i]*turn/3600.
    return duration, dose


def forecast_tree(terrain, parent, rates, reverse=False, speed=.06, yaw_rate=.35):
    """Forecast selected paths, not alternative-path optimization or a guarantee.

    Reverse-tree edges execute child -> parent. Each parent's turn is included
    when a child is appended; its angle is unchanged by reversing the whole path.
    Start heading is unknown here and covered by the executive's dwell reserve.
    """
    children = {key: [] for key in parent}
    queue = []
    for key, previous in parent.items():
        if previous is None:
            queue.append(key)
        else:
            children[previous].append(key)
    duration, exposure, peak = {}, {}, {}
    for key in queue:
        previous = parent[key]
        if previous is None:
            duration[key], exposure[key], peak[key] = 0., 0., rates[key]
        else:
            turn = 0.
            before = parent[previous]
            if before is not None:
                a = terrain.nodes[previous][:2]-terrain.nodes[before][:2]
                b = terrain.nodes[key][:2]-terrain.nodes[previous][:2]
                if np.linalg.norm(a)*np.linalg.norm(b) > 1e-12:
                    turn = abs(math.atan2(a[0]*b[1]-a[1]*b[0], float(a@b)))/yaw_rate
            edge = terrain.edge_time(key, previous, speed) if reverse else terrain.edge_time(previous, key, speed)
            duration[key] = duration[previous]+edge+turn
            exposure[key] = exposure[previous]+(max(rates[key], rates[previous])*edge+rates[previous]*turn)/3600.
            peak[key] = max(peak[previous], rates[key])
        queue.extend(children[key])
    return duration, exposure, peak
