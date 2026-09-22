"""Goal admission using observed occupancy only; unknown is not traversable."""
import math


def observed_free(grid, x, y, radius):
    if grid is None or grid.info.resolution <= 0:
        return False
    origin = grid.info.origin
    q = origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    dx, dy = x - origin.position.x, y - origin.position.y
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = c * dx + s * dy, -s * dx + c * dy
    resolution = grid.info.resolution
    cx, cy = math.floor(x / resolution), math.floor(y / resolution)
    cells = math.ceil(radius / resolution)
    for iy in range(cy - cells, cy + cells + 1):
        for ix in range(cx - cells, cx + cells + 1):
            if not (0 <= ix < grid.info.width and 0 <= iy < grid.info.height):
                return False
            index = iy * grid.info.width + ix
            if index >= len(grid.data) or not 0 <= grid.data[index] < 20:
                return False
    return True
