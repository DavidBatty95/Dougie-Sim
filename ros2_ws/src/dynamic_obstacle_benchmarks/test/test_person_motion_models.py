import random
import unittest

from dynamic_obstacle_benchmarks.config import Bounds
from dynamic_obstacle_benchmarks.person_motion_models import build_person, update_person


class PersonMotionModelTests(unittest.TestCase):
    def test_required_behaviours_build_and_move(self) -> None:
        behaviours = [
            "straight_line_crossing",
            "random_waypoint",
            "corridor_bottleneck",
            "opposing_flow",
            "stop_start",
            "occlusion_emergence",
        ]
        for index, behaviour in enumerate(behaviours):
            person = build_person(index, behaviour, random.Random(index), Bounds(), 0.3, 0.5)
            before = person.pose
            update_person(person, 0.1, 0.1, 0.5, Bounds())
            self.assertGreaterEqual(len(person.waypoints), 2)
            self.assertNotEqual((before.x, before.y), (person.pose.x, person.pose.y))


if __name__ == "__main__":
    unittest.main()
