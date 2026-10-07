"""Known-free frontier standoffs. Unknown space is never traversable.

Uses the same conservative grid-inflation principle as Geoff-Sim's observed-map
search; the Go2 executive has no simulator layout or object-coordinate inputs.
"""
from collections import deque
import math

import numpy as np


def frontier_goals(message, robot, excluded=(), radius=.38):
    info = message.info
    w, h, res = info.width, info.height, info.resolution
    if w <= 0 or h <= 0 or w*h > 500000 or not math.isfinite(res) or res <= 0 or len(message.data) != w*h:
        raise ValueError('invalid or oversized occupancy grid')
    q = info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = info.origin.position.x, info.origin.position.y
    if not all(math.isfinite(v) for v in (*robot, ox, oy, radius)) or radius < 0:
        raise ValueError('invalid grid geometry')
    cells = np.asarray(message.data).reshape(h, w)
    free = (cells >= 0) & (cells < 20)
    reserve = math.ceil(radius/res)
    size = 2*reserve+1
    integral = np.pad(np.pad(free.astype(np.int32), reserve), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    clear = integral[size:, size:] - integral[:-size, size:] - integral[size:, :-size] + integral[:-size, :-size] == size*size
    dx, dy = robot[0]-ox, robot[1]-oy
    start = math.floor((c*dx+s*dy)/res), math.floor((-s*dx+c*dy)/res)
    if not (0 <= start[0] < w and 0 <= start[1] < h and clear[start[1], start[0]]):
        return []
    distances = {start: 0}
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        for nx, ny in ((x+1, y), (x-1, y), (x, y+1), (x, y-1)):
            if 0 <= nx < w and 0 <= ny < h and clear[ny, nx] and (nx, ny) not in distances:
                distances[nx, ny] = distances[x, y]+1
                queue.append((nx, ny))
    goals = []
    stride = max(1, round(.4/res))
    for (x, y), steps in distances.items():
        if x % stride or y % stride or steps*res < .8:
            continue
        # Count first-hit unknown cells, excluding rays blocked by furniture.
        unknown = set()
        for angle in np.linspace(-math.pi, math.pi, 16, endpoint=False):
            for step in range(1, math.ceil(2./res)+1):
                nx, ny = round(x+step*math.cos(angle)), round(y+step*math.sin(angle))
                if not (0 <= nx < w and 0 <= ny < h) or cells[ny, nx] >= 20:
                    break
                if cells[ny, nx] < 0:
                    unknown.add((nx, ny))
                    break
        if not unknown:
            continue
        lx, ly = (x+.5)*res, (y+.5)*res
        point = ox+c*lx-s*ly, oy+s*lx+c*ly
        if any(math.dist(point, old) < .7 for old in excluded):
            continue
        heading = math.atan2(point[1]-robot[1], point[0]-robot[0])
        goals.append((len(unknown)/(1.+steps*res), *point, heading))
    return sorted(goals, reverse=True)
