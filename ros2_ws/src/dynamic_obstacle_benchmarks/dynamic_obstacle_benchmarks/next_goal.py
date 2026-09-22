"""Prepare an observed-grid-reachable visual waypoint without preempting Nav2."""
import copy
import heapq
import math
import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from .geometry import distance_xy
from .goal_sampler import GoalSampler
from .observed_space import observed_free
from .person_motion_models import min_person_distance


def grid_path(grid, start, goal, radius, free=None):
    """Bounded A* over known-free footprint cells. Unknown is never traversable."""
    if grid is None or grid.info.resolution <= 0:
        return None, free
    info = grid.info
    if free is None:
        cells = np.asarray(grid.data).reshape((info.height, info.width))
        raw = (cells >= 0) & (cells < 20)
        padding = math.ceil(radius/info.resolution)
        padded = np.pad(raw, padding, constant_values=False)
        free = raw.copy()
        for dy in range(2*padding+1):
            for dx in range(2*padding+1):
                free &= padded[dy:dy+info.height, dx:dx+info.width]
    q = info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    def cell(pose):
        x, y = pose.x-info.origin.position.x, pose.y-info.origin.position.y
        return math.floor((c*x+s*y)/info.resolution), math.floor((-s*x+c*y)/info.resolution)
    def allowed(point):
        x, y = point
        return 0 <= x < info.width and 0 <= y < info.height and free[y, x]
    first, last = cell(start), cell(goal)
    if not allowed(first) or not allowed(last):
        return None, free
    queue, costs, previous = [(0., first)], {first: 0.}, {}
    visited = set()
    while queue and len(visited) < 2500:
        _, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        if current == last:
            cells = [last]
            while cells[-1] != first:
                cells.append(previous[cells[-1]])
            path = Path()
            path.header.frame_id = grid.header.frame_id
            for x, y in reversed(cells):
                px, py = (x+.5)*info.resolution, (y+.5)*info.resolution
                pose = PoseStamped()
                pose.pose.orientation.w = 1.
                pose.pose.position.x = info.origin.position.x+c*px-s*py
                pose.pose.position.y = info.origin.position.y+s*px+c*py
                path.poses.append(pose)
            return path, free
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            target = current[0]+dx, current[1]+dy
            if not allowed(target) or (dx and dy and (not allowed((current[0]+dx, current[1])) or not allowed((current[0], current[1]+dy)))):
                continue
            cost = costs[current] + math.hypot(dx, dy)
            if cost >= costs.get(target, math.inf):
                continue
            costs[target], previous[target] = cost, current
            heapq.heappush(queue, (cost+math.hypot(target[0]-last[0], target[1]-last[1]), target))
    return None, free


class NextGoalPreparer:
    def __init__(self, node, rng):
        self.node, self.rng = node, rng
        self.ready = None
        self.last_attempt = -math.inf
        self.grid_message, self.free = None, None

    def clear(self):
        self.ready = None

    def prepare(self, index, anchor):
        now = time.monotonic()
        if self.ready is not None or now-self.last_attempt < .3 or self.node.latest_robot_pose is None:
            return
        self.last_attempt = now
        grid = self.node.observed_map
        if grid is None or grid.header.frame_id != self.node.config.map_frame:
            return
        if grid is not self.grid_message:
            self.grid_message, self.free = grid, None
        config = copy.copy(self.node.config)
        # Keep the configured sampling annulus. Shrinking both ends by arrival
        # tolerance collapsed the default 1.5–2.5 m range to only 2.00–2.02 m,
        # starving prefetch in crowded/partly observed spaces. take() rechecks
        # actual separation from the robot at handoff.
        sampler = GoalSampler(config, self.rng, lambda start, goal:
            observed_free(grid, goal.x, goal.y, config.robot_radius_m+.15))
        try:
            goal = sampler.sample_random_goal(index, anchor, self.node.goal_people())
        except RuntimeError:
            return
        path, self.free = grid_path(grid, self.node.latest_robot_pose, goal.pose, config.robot_radius_m, self.free)
        if path is not None:
            self.ready = goal, path, now

    def take(self, index):
        if self.ready is None:
            return None
        goal, path, stamp = self.ready
        config = self.node.config
        valid = (goal.index == index and time.monotonic()-stamp <= 5.
                 and observed_free(self.node.observed_map, goal.pose.x, goal.pose.y, config.robot_radius_m+.15)
                 and min_person_distance(goal.pose, self.node.goal_people()) >= config.spawn_min_distance_from_goal_m)
        if valid:
            pose = self.node.latest_robot_pose
            valid = pose is not None and distance_xy(pose, goal.pose) >= config.min_goal_distance_m
            if valid:
                nearest = min(range(len(path.poses)), key=lambda i: math.hypot(path.poses[i].pose.position.x-pose.x, path.poses[i].pose.position.y-pose.y))
                valid = all(observed_free(self.node.observed_map, p.pose.position.x, p.pose.position.y,
                                         config.robot_radius_m) for p in path.poses[nearest:])
        self.clear()
        return goal if valid else None
