"""ROS-independent event policy for bounded, dynamic global replanning."""
import math


def distance_to_route(point, route):
    if not route:
        return math.inf
    if len(route) == 1:
        return math.hypot(point[0]-route[0][0], point[1]-route[0][1])
    best = math.inf
    for a, b in zip(route, route[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        fraction = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy) / max(1e-12, dx*dx+dy*dy)))
        best = min(best, math.hypot(point[0]-a[0]-fraction*dx, point[1]-a[1]-fraction*dy))
    return best


def forward_route(route, robot, lookahead=6.0):
    if not route:
        return []
    index = min(range(len(route)), key=lambda i: math.hypot(route[i][0]-robot[0], route[i][1]-robot[1]))
    result = [route[index]]
    length = 0.0
    for point in route[index+1:]:
        length += math.hypot(point[0]-result[-1][0], point[1]-result[-1][1])
        result.append(point)
        if length >= lookahead:
            break
    return result


def corridor_people(people, route, horizon=1.5, padding=.65):
    """Snapshot people whose observed CV sweep intersects the upcoming route.

    Samples are spaced by at most 0.10 m and the radius includes half that step,
    so a fast crossing between samples is conservatively retained.
    """
    relevant = {}
    for identity, x, y, vx, vy, radius in people:
        count = max(1, math.ceil(math.hypot(vx, vy)*horizon/.10))
        if any(distance_to_route((x+vx*horizon*i/count, y+vy*horizon*i/count), route)
               <= radius+padding+.05 for i in range(count+1)):
            relevant[identity] = (x, y, vx, vy)
    return relevant


class ReplanPolicy:
    def __init__(self, minimum_interval=.2, maximum_interval=.5, movement=.20, velocity_change=.25):
        if not 0 < minimum_interval <= maximum_interval:
            raise ValueError("Replan intervals must satisfy 0 < minimum <= maximum")
        self.minimum_interval = minimum_interval
        self.maximum_interval = maximum_interval
        self.movement = movement
        self.velocity_change = velocity_change
        self.last_request = -math.inf
        self.last_plan = -math.inf
        self.people_at_plan = {}
        self.pending_people = {}
        self.map_was_blocked = False

    def accept_plan(self, now):
        self.last_plan = now
        self.people_at_plan = dict(self.pending_people)

    def reason(self, now, people, has_path, map_blocked=False, off_route=False):
        self.pending_people = dict(people)
        if now - self.last_request < self.minimum_interval:
            return None
        was_blocked = self.map_was_blocked
        self.map_was_blocked = map_blocked
        if not has_path:
            reason = "missing_route"
        elif map_blocked:
            reason = "observed_map_blocked"
        elif was_blocked:
            reason = "observed_map_cleared"
        elif off_route:
            reason = "off_route"
        elif people.keys() != self.people_at_plan.keys():
            reason = "corridor_occupancy_changed"
        elif any(math.hypot(value[0]-self.people_at_plan[key][0], value[1]-self.people_at_plan[key][1]) >= self.movement
                 or math.hypot(value[2]-self.people_at_plan[key][2], value[3]-self.people_at_plan[key][3]) >= self.velocity_change
                 for key, value in people.items()):
            reason = "corridor_motion_changed"
        elif now - max(self.last_plan, self.last_request) >= self.maximum_interval:
            reason = "periodic_refresh"
        else:
            return None
        self.last_request = now
        return reason
