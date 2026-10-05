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


def velocity_obstacle_command(pose, velocity, preferred, people, limits,
                              gains=(1., 1.), horizon=3., delay=.35, side_hint=0.):
    """Project a preferred command into non-reciprocal velocity-obstacle planes.

    People do not share avoidance responsibility. The result is only a proposal:
    the caller must still check actuator-response rollouts and static geometry.
    `people` contains (x,y,vx,vy,combined_radius) in the planning frame.
    """
    c, s = math.cos(pose[2]), math.sin(pose[2])
    def world(x, y):
        return c*x-s*y, s*x+c*y
    def lateral_gain(y):
        return gains[2] if y < 0. and len(gains)>2 else gains[1]
    robot_v = world(velocity[0], velocity[1])
    desired = world(preferred[0]*gains[0], preferred[1]*lateral_gain(preferred[1]))
    forward, reverse, lateral = limits
    polygon = [world(x*gains[0], y*lateral_gain(y)) for x,y in
               ((-reverse,-lateral),(forward,-lateral),(forward,lateral),(-reverse,lateral))]
    for x,y,vx,vy,radius in people:
        rvx, rvy = robot_v[0]-vx, robot_v[1]-vy
        px, py = x-pose[0]-rvx*delay, y-pose[1]-rvy*delay
        distance2, radius2 = px*px+py*py, radius*radius
        if distance2 < 1e-12:
            return None
        tau = horizon if distance2 > radius2 else .5
        wx, wy = rvx-px/tau, rvy-py/tau
        dot = wx*px+wy*py
        length2 = wx*wx+wy*wy
        if distance2 <= radius2 or (dot < 0 and dot*dot > radius2*length2):
            length = math.sqrt(length2)
            if length < 1e-9:
                length = max(1e-9, math.sqrt(distance2))
                nx, ny = -px/length, -py/length
            else:
                nx, ny = wx/length, wy/length
            ux, uy = (radius/tau-length)*nx, (radius/tau-length)*ny
        else:
            leg = math.sqrt(max(0., distance2-radius2))
            side = px*wy-py*wx
            if side_hint and abs(side) < .15*math.sqrt(distance2):
                side = side_hint*(px*c+py*s)
            if side > 0:
                dx, dy = (px*leg-py*radius)/distance2, (px*radius+py*leg)/distance2
            else:
                dx, dy = -(px*leg+py*radius)/distance2, -(-px*radius+py*leg)/distance2
            projection = rvx*dx+rvy*dy
            ux, uy = projection*dx-rvx, projection*dy-rvy
            nx, ny = -dy, dx
        bound = nx*(robot_v[0]+ux) + ny*(robot_v[1]+uy)
        clipped = []
        for a,b in zip(polygon, polygon[1:]+polygon[:1]):
            fa, fb = nx*a[0]+ny*a[1]-bound, nx*b[0]+ny*b[1]-bound
            if fa >= -1e-9:
                clipped.append(a)
            if (fa >= 0) != (fb >= 0):
                fraction = fa/(fa-fb)
                clipped.append((a[0]+fraction*(b[0]-a[0]), a[1]+fraction*(b[1]-a[1])))
        polygon = clipped
        if not polygon:
            return None
    best, score = None, math.inf
    # Include the desired velocity only if it is inside the convex polygon.
    signs = [(b[0]-a[0])*(desired[1]-a[1])-(b[1]-a[1])*(desired[0]-a[0])
             for a,b in zip(polygon,polygon[1:]+polygon[:1])]
    area = sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(polygon,polygon[1:]+polygon[:1]))
    if len(polygon) >= 3 and area > 1e-12 and all(value >= -1e-9 for value in signs):
        best = desired
    else:
        for a,b in zip(polygon, polygon[1:]+polygon[:1]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            fraction = max(0., min(1., ((desired[0]-a[0])*dx+(desired[1]-a[1])*dy)/max(1e-12,dx*dx+dy*dy)))
            point = a[0]+fraction*dx, a[1]+fraction*dy
            distance = (point[0]-desired[0])**2+(point[1]-desired[1])**2
            if distance < score:
                best, score = point, distance
    body_y = -s*best[0]+c*best[1]
    return ((c*best[0]+s*best[1])/max(1e-6,gains[0]),body_y/max(1e-6,lateral_gain(body_y)))
