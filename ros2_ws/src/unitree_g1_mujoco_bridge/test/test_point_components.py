import math
import random

from unitree_g1_mujoco_bridge.point_components import connected_components


def test_adaptive_connectivity_matches_exhaustive_graph():
    rng = random.Random(51)
    for z_gate in (0., .2, .6):
        points = [(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0, 1.8)) for _ in range(180)]
        radii = [rng.uniform(.1, .5) for _ in points]
        remaining, expected = set(range(len(points))), []
        while remaining:
            start = remaining.pop()
            queue, component = [start], {start}
            while queue:
                i = queue.pop()
                x, y, z = points[i]
                matches = {j for j in remaining if
                           (z_gate <= 0 or abs(points[j][2]-z) <= z_gate) and
                           math.hypot(points[j][0]-x, points[j][1]-y) <= max(radii[i], radii[j])}
                remaining -= matches
                component |= matches
                queue.extend(matches)
            expected.append(frozenset(component))
        actual = connected_components(points, radii, .2, z_gate)
        assert {frozenset(c) for c in actual} == set(expected)
        assert sum(map(len, actual)) == len(points)


def test_duplicates_and_empty_cloud():
    assert connected_components([], [], .1, .2) == []
    assert len(connected_components([(0, 0, 0)]*100, [.2]*100, .2, .5)) == 1
