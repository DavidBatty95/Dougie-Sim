"""Revalidate retained routes against an evolving observed occupancy map."""
import math

import numpy as np


def people_clear_corridor(start, finish, people, robot_radius, margin=.4, horizon=3.):
    """Conservative admission for a shortcut, not a replacement collision test."""
    dx, dy = finish[0]-start[0], finish[1]-start[1]
    norm = max(1e-12, dx*dx+dy*dy)
    for x, y, vx, vy, radius in people:
        for time in np.linspace(0., horizon, max(2, math.ceil(horizon/.5)+1)):
            px, py = x+vx*time, y+vy*time
            fraction = max(0., min(1., ((px-start[0])*dx+(py-start[1])*dy)/norm))
            if math.hypot(px-start[0]-fraction*dx, py-start[1]-fraction*dy) < robot_radius+radius+margin:
                return False
    return True


def route_is_free(grid, path, radius):
    if grid is None or not path.poses or path.header.frame_id != grid.header.frame_id:
        return False
    info = grid.info
    if info.resolution <= 0 or len(grid.data) != info.width * info.height:
        return False
    points = np.asarray([(p.pose.position.x, p.pose.position.y) for p in path.poses])
    if not np.isfinite(points).all():
        return False
    samples = [points[:1]]
    for a, b in zip(points[:-1], points[1:]):
        count = max(1, math.ceil(np.linalg.norm(b - a) / (info.resolution * .5)))
        if count > 10000:
            return False
        samples.append(a + np.linspace(0, 1, count + 1)[1:, None] * (b - a))
    points = np.concatenate(samples)
    q = info.origin.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    points -= (info.origin.position.x, info.origin.position.y)
    cells = np.floor(points @ np.array(((c, -s), (s, c))) / info.resolution).astype(int)
    extent = math.ceil(radius / info.resolution)
    offsets = np.array([(x, y) for x in range(-extent, extent + 1) for y in range(-extent, extent + 1)
                        if math.hypot(x, y) * info.resolution <= radius + info.resolution * .71])
    cells = (cells[:, None, :] + offsets).reshape(-1, 2)
    if np.any(cells < 0) or np.any(cells[:, 0] >= info.width) or np.any(cells[:, 1] >= info.height):
        return False
    values = np.asarray(grid.data)[cells[:, 1] * info.width + cells[:, 0]]
    return bool(np.all((values >= 0) & (values < 20)))
