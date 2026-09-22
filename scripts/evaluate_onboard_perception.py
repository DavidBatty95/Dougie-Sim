#!/usr/bin/env python3
"""Offline-only alignment of estimated odometry/tracks with scoring truth."""
import argparse
from bisect import bisect_left
import json
import math

import numpy as np


def yaw(pose):
    return math.atan2(2*(pose["qw"]*pose["qz"]+pose["qx"]*pose["qy"]),
                      1-2*(pose["qy"]**2+pose["qz"]**2))


def rotate(xy, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s), (s, c))) @ xy


def nearest(series, stamps, stamp):
    index = bisect_left(stamps, stamp)
    candidates = [i for i in (index-1, index) if 0 <= i < len(series)]
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(stamps[i] - stamp))
    return series[best] if abs(stamps[best] - stamp) <= .12 else None


def summary(values):
    return {"count": len(values), "p50": float(np.median(values)),
            "p95": float(np.quantile(values, .95)), "max": float(max(values))} if values else {}


def evaluate(path):
    streams = {name: [] for name in ("odom", "evaluation_odom", "mujoco_people", "tracker_debug")}
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            if row["topic"] in streams and isinstance(row["data"], dict):
                data = row["data"]
                stamp = data.get("source_stamp", data.get("stamp", row["stamp"]))
                streams[row["topic"]].append((stamp, data))
    streams = {name: sorted(values, key=lambda item: item[0]) for name, values in streams.items()}
    stamps = {name: [item[0] for item in values] for name, values in streams.items()}
    drift, residuals = [], []
    alignment = None
    for stamp, pose in streams["odom"]:
        truth = nearest(streams["evaluation_odom"], stamps["evaluation_odom"], stamp)
        if truth is None:
            continue
        actual = truth[1]
        observed_xy, actual_xy = np.array((pose["x"], pose["y"])), np.array((actual["x"], actual["y"]))
        if alignment is None:
            angle = yaw(pose) - yaw(actual)
            alignment = (angle, observed_xy - rotate(actual_xy, angle))
        drift.append(np.linalg.norm(observed_xy - rotate(actual_xy, alignment[0]) - alignment[1]))
    for stamp, data in streams["tracker_debug"]:
        samples = {name: nearest(streams[name], stamps[name], stamp) for name in ("odom", "evaluation_odom", "mujoco_people")}
        if any(value is None for value in samples.values()):
            continue
        pose, root, people = (samples[name][1] for name in ("odom", "evaluation_odom", "mujoco_people"))
        angle = yaw(pose) - yaw(root)
        offset = np.array((pose["x"], pose["y"])) - rotate(np.array((root["x"], root["y"])), angle)
        observed = np.array([(t["x"], t["y"]) for t in data["tracks"] if t["state"] == "CONFIRMED" and t["missed_count"] == 0]).reshape(-1, 2)
        for person in people.get("people", []):
            if math.hypot(person["x"]-root["x"], person["y"]-root["y"]) > 5.0:
                continue
            position = rotate(np.array((person["x"], person["y"])), angle) + offset
            # Includes occlusions and misses: this is not a visibility-conditioned recall metric.
            residuals.append(float(np.min(np.linalg.norm(observed-position, axis=1))) if len(observed) else 10.)
    return {"odometry_drift_m": summary(drift), "nearby_truth_to_confirmed_track_m": summary(residuals),
            "note": "Offline diagnostic only; nearby people can be occluded. Unmatched observations are assigned 10 m."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    args = parser.parse_args()
    print(json.dumps(evaluate(args.log), indent=2))
