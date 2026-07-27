#!/usr/bin/env python3
"""Offline evaluator for LiDAR person tracking and avoidance bags.

The script intentionally does not use MuJoCo ground-truth identities for runtime
perception. Optional ground-truth CSV input is only used here for offline metrics.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


def _duration(stamps: list[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    return max(0.0, max(stamps) - min(stamps))


def _load_ground_truth(path: str | None) -> dict[float, list[tuple[float, float]]]:
    if not path:
        return {}
    frames: dict[float, list[tuple[float, float]]] = defaultdict(list)
    with open(path, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            stamp = float(row["stamp"])
            frames[stamp].append((float(row["x"]), float(row["y"])))
    return frames


def _nearest_gt(stamp: float, gt: dict[float, list[tuple[float, float]]], tolerance: float) -> list[tuple[float, float]]:
    if not gt:
        return []
    best_stamp = min(gt, key=lambda candidate: abs(candidate - stamp))
    if abs(best_stamp - stamp) > tolerance:
        return []
    return gt[best_stamp]


def _read_bag_messages(bag_path: str):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:  # pragma: no cover - depends on ROS environment
        raise SystemExit("rosbag2_py/rclpy are required inside a sourced ROS 2 environment") from exc

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    msg_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg = deserialize_message(data, msg_types[topic])
        yield topic, msg, timestamp * 1e-9


def _point_count(msg) -> int:
    return int(getattr(msg, "width", 0)) * int(getattr(msg, "height", 1))


def evaluate(args: argparse.Namespace) -> dict[str, float | int]:
    gt = _load_ground_truth(args.ground_truth_csv)
    stamps: list[float] = []
    confirmed_frames = 0
    predicted_frames = 0
    false_confirmed_frames = 0
    false_predicted_frames = 0
    missed_person_frames = 0
    stale_prediction_duration = 0.0
    cmd_overrides = 0
    false_cmd_overrides = 0
    debug_tracks = []
    last_prediction_stamp: float | None = None
    last_prediction_nonempty = False

    for topic, msg, stamp in _read_bag_messages(args.bag):
        stamps.append(stamp)
        if topic == args.debug_topic:
            try:
                debug_tracks.append(json.loads(msg.data))
            except json.JSONDecodeError:
                pass
        elif topic == args.tracked_topic:
            count = _point_count(msg)
            has_track = count > 0
            if has_track:
                confirmed_frames += 1
            gt_people = _nearest_gt(stamp, gt, args.gt_tolerance)
            if has_track and gt and not gt_people:
                false_confirmed_frames += 1
            if gt_people and not has_track:
                missed_person_frames += 1
        elif topic == args.predicted_topic:
            count = _point_count(msg)
            has_prediction = count > 0
            if has_prediction:
                predicted_frames += 1
            gt_people = _nearest_gt(stamp, gt, args.gt_tolerance)
            if has_prediction and gt and not gt_people:
                false_predicted_frames += 1
            if last_prediction_stamp is not None and last_prediction_nonempty and not has_prediction:
                stale_prediction_duration += max(0.0, stamp - last_prediction_stamp)
            last_prediction_stamp = stamp
            last_prediction_nonempty = has_prediction
        elif topic == args.safe_cmd_topic:
            raw_speed = math.hypot(float(msg.linear.x), float(msg.linear.y)) + abs(float(msg.angular.z))
            if raw_speed > args.override_motion_threshold:
                cmd_overrides += 1
                gt_people = _nearest_gt(stamp, gt, args.gt_tolerance)
                if gt and not gt_people:
                    false_cmd_overrides += 1

    minutes = max(_duration(stamps) / 60.0, 1e-6)
    id_switches = 0
    track_jumps = 0
    fragmented_tracks = defaultdict(int)
    last_track_positions: dict[int, tuple[float, float, float, str]] = {}
    for payload in debug_tracks:
        for track in payload.get("tracks", []):
            track_id = int(track.get("track_id", -1))
            state = str(track.get("state", ""))
            x = float(track.get("x", 0.0))
            y = float(track.get("y", 0.0))
            stamp = float(payload.get("stamp", 0.0))
            if track_id in last_track_positions:
                last_x, last_y, last_stamp, last_state = last_track_positions[track_id]
                dt = max(1e-3, stamp - last_stamp)
                speed = math.hypot(x - last_x, y - last_y) / dt
                if speed > args.max_plausible_speed:
                    track_jumps += 1
                if last_state != "CONFIRMED" and state == "CONFIRMED":
                    fragmented_tracks[track_id] += 1
            last_track_positions[track_id] = (x, y, stamp, state)

    return {
        "duration_min": round(minutes, 3),
        "false_positive_confirmed_tracks_per_min": round(false_confirmed_frames / minutes, 3),
        "false_predicted_risk_publications_per_min": round(false_predicted_frames / minutes, 3),
        "missed_person_frames": missed_person_frames,
        "confirmed_track_frames": confirmed_frames,
        "predicted_risk_frames": predicted_frames,
        "track_fragmentation_count": sum(max(0, count - 1) for count in fragmented_tracks.values()),
        "id_switches_estimate": id_switches,
        "track_jumps_over_plausible_speed": track_jumps,
        "stale_prediction_duration_sec": round(stale_prediction_duration, 3),
        "cmd_vel_safe_override_samples": cmd_overrides,
        "false_cmd_vel_safe_override_samples": false_cmd_overrides,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Path to a ROS 2 sqlite3 rosbag directory")
    parser.add_argument("--ground-truth-csv", help="Optional offline-only CSV with columns stamp,person_id,x,y")
    parser.add_argument("--tracked-topic", default="/tracked_dynamic_obstacles")
    parser.add_argument("--predicted-topic", default="/predicted_dynamic_obstacles")
    parser.add_argument("--debug-topic", default="/dynamic_obstacle_tracker/debug_tracks")
    parser.add_argument("--safe-cmd-topic", default="/cmd_vel_safe")
    parser.add_argument("--gt-tolerance", type=float, default=0.15)
    parser.add_argument("--max-plausible-speed", type=float, default=3.0)
    parser.add_argument("--override-motion-threshold", type=float, default=0.03)
    parser.add_argument("--output", help="Optional JSON output path")
    args = parser.parse_args()
    result = evaluate(args)
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
