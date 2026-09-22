#!/usr/bin/env python3
"""Diagnose a blocked snapshot with CV people and a denser translation search.

This isolates command feasibility: static obstacles and full runtime prediction
state are not replayed. It is not a replacement for live acceptance testing.
"""
import argparse
import json
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from unitree_g1_mujoco_bridge.human_aware_motion_planner import Candidate, HumanAwareMotionPlanner, HumanTrack, align_track_to_time
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--ros-domain-id", type=int, default=93)
    parser.add_argument("--acceleration", type=float, default=1.2)
    args = parser.parse_args()
    odom = tracker = None
    snapshot = None
    with open(args.log, encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if row["topic"] == "odom":
                odom = row["data"]
            elif row["topic"] == "tracker_debug":
                tracker = row["data"]
            elif row["topic"] == "planner_debug" and odom and tracker:
                data = row["data"]
                if data["state"] == "EMERGENCY_STOP" and data.get("candidate_results") and data.get("track_count", 0) > 0:
                    snapshot = row
                    break
    if snapshot is None:
        raise SystemExit("No blocked planner snapshot found")
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    rclpy.init()
    planner = HumanAwareMotionPlanner()
    try:
        planner.motion_envelope = MotionEnvelope(max_x=0.65, max_y=0.45, accel_x=args.acceleration, accel_y=args.acceleration)
        planner.horizon = 6.0
        planner.dt = 0.15
        planner.hard_collision_horizon = 4.0
        planner.holonomic_avoidance = True
        planner.max_forward_near_human = 0.65
        planner.max_lateral_near_human = 0.45
        planner.max_reverse_velocity = 0.45
        planner._static_obstacle_cost = lambda trajectory: (0.0, False)
        data = snapshot["data"]
        planner.measured_velocity = tuple(data["measured_velocity"])
        planner.policy_command = data.get("policy_command")
        planner.policy_command_stamp = planner.get_clock().now().nanoseconds * 1e-9
        planner._simulate = lambda pose, c: planner.motion_envelope.rollout(pose, (c.vx, c.vy, c.wz), planner.measured_velocity, planner.horizon, planner.dt, planner.response_delay, c.switch_time if c.follow_vx is not None else math.inf, (c.follow_vx, 0.0, 0.0) if c.follow_vx is not None else None, planner.policy_command)
        planner.latest_cmd = Twist()
        planner.latest_cmd.linear.x = data["raw_cmd"]["x"]
        planner.latest_cmd.angular.z = data["raw_cmd"]["z"]
        yaw = 2.0 * math.atan2(odom["qz"], odom["qw"])
        pose = (odom["x"], odom["y"], yaw)
        tracks = [align_track_to_time(HumanTrack(t["track_id"], t["state"], t["x"], t["y"], t["vx"], t["vy"], t["confidence"], t["person_score"], t["missed_count"], t["cov_xx"], t["cov_yy"], tracker["stamp"], 0.42), snapshot["stamp"]) for t in tracker["tracks"]]
        results = []
        for vx in (-0.65, -0.45, -0.30, 0.0, 0.15, 0.30, 0.45, 0.65):
            for vy in (-0.45, -0.30, -0.15, 0.0, 0.15, 0.30, 0.45):
                for duration in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0):
                    candidate = Candidate("replay", vx, vy, 0.0, "", 0.0, duration)
                    result = planner._score_candidate(candidate, pose, tracks, [], [], {}, 0.0)
                    results.append((result.min_human_distance, vx, vy, duration, result.reason))
        generated = planner._executable_candidates(planner._candidate_commands(planner.latest_cmd, "HUMAN_CAUTION", pose, tracks))
        scored = [planner._score_candidate(c, pose, tracks, [], [], {}, 0.0) for c in generated]
        print(json.dumps({"static_obstacles_evaluated": False, "prediction_model": "constant_velocity_debug_snapshot", "snapshot": snapshot["stamp"], "raw": data["raw_cmd"], "pose": pose, "velocity": planner.measured_velocity, "people": tracker["tracks"], "best_clearance": sorted(results, reverse=True)[:10], "generated": [(r.candidate.vx, r.candidate.vy, r.candidate.switch_time, r.reason, r.min_human_distance) for r in scored]}, indent=2))
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
