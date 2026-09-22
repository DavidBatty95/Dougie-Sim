"""Early 360-degree crossing escape proposals; callers must collision-check them."""
import math


def crossing_escape_directions(pose, people, horizon=3., clearance=1.4):
    threats = []
    for x, y, vx, vy in people:
        speed2 = vx*vx + vy*vy
        if speed2 < .04:
            continue
        dx, dy = x-pose[0], y-pose[1]
        time = -(dx*vx+dy*vy)/speed2
        if not 0 <= time <= horizon:
            continue
        distance = math.hypot(dx+vx*time, dy+vy*time)
        if distance < clearance:
            threats.append((time, distance, vx, vy))
    if not threats:
        return []
    _, _, vx, vy = min(threats)
    speed = math.hypot(vx, vy)
    nx, ny = -vy/speed, vx/speed
    c, s = math.cos(pose[2]), math.sin(pose[2])
    bx, by = c*nx+s*ny, -s*nx+c*ny
    return [(bx, by), (-bx, -by)]
