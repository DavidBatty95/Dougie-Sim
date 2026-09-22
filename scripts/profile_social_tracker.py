#!/usr/bin/env python3
"""Bounded oracle-tracker CPU profile in a sourced ROS environment."""
import cProfile
import argparse
import os
import pstats

import rclpy
from unitree_g1_mujoco_bridge.dynamic_obstacle_tracker import DynamicObstacleTracker, PersonDetection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-domain-id", type=int, default=93)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    rclpy.init()
    node = DynamicObstacleTracker()
    node.publish_safety_grid = False
    node._lookup_robot_xy = lambda *args: (0.0, 0.0)
    start = node.get_clock().now().nanoseconds * 1e-9
    profiler = cProfile.Profile()
    try:
        profiler.enable()
        for step in range(40):
            stamp = start + step * 0.05
            node.mujoco_detections = [PersonDetection(1.5 + i, -2.0 + step * 0.03, 0.8, 1.0, 0.42, stamp, 0.0, 0.6) for i in range(4)]
            node._publish_points([], stamp, "world")
        profiler.disable()
        pstats.Stats(profiler).sort_stats("cumulative").print_stats(25)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
