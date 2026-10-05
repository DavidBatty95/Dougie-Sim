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


def evaluate(path, person_id=None, last_seconds=10.):
    streams = {name: [] for name in ("odom", "evaluation_odom", "mujoco_people", "tracker_debug", "planner_debug", "safety_debug")}
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            if row["topic"] in streams and isinstance(row["data"], dict):
                data = row["data"]
                data['_logged_at'] = row['stamp']
                stamp = data.get("source_stamp", data.get("stamp", row["stamp"]))
                streams[row["topic"]].append((stamp, data))
    streams = {name: sorted(values, key=lambda item: item[0]) for name, values in streams.items()}
    stamps = {name: [item[0] for item in values] for name, values in streams.items()}
    drift, residuals, close_residuals, velocity_errors, radial_bias = [], [], [], [], []
    close_missed = close_count = 0
    trace = []
    previous_trace = -math.inf
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
        tracks = [t for t in data['tracks'] if t['state'] == 'CONFIRMED' and t['missed_count'] == 0]
        observed = np.array([(t['x'], t['y']) for t in tracks]).reshape(-1,2)
        for person in people.get("people", []):
            if math.hypot(person["x"]-root["x"], person["y"]-root["y"]) > 5.0:
                continue
            position = rotate(np.array((person["x"], person["y"])), angle) + offset
            # Includes occlusions and misses: this is not a visibility-conditioned recall metric.
            distances = np.linalg.norm(observed-position, axis=1)
            residual = float(np.min(distances)) if len(observed) else 10.
            residuals.append(residual)
            if (person_id is not None and person['id'] == person_id and
                    stamp >= stamps['tracker_debug'][-1]-last_seconds and stamp-previous_trace >= .4):
                previous_trace = stamp
                matched = tracks[int(np.argmin(distances))] if len(tracks) and residual < .5 else None
                decision = nearest(streams['planner_debug'], stamps['planner_debug'], data['_logged_at'])
                safety = nearest(streams['safety_debug'], stamps['safety_debug'], data['_logged_at'])
                trace.append(dict(stamp=round(stamp,3), distance=round(math.hypot(person['x']-root['x'],person['y']-root['y']),3),
                    truth_velocity=[round(v,3) for v in rotate(np.array((person['vx'],person['vy'])),angle)],
                    track=matched, decision=None if decision is None else {k:decision[1].get(k) for k in ('state','selected_cmd','policy_command','min_human_distance','reference_cmd')},
                    safety=None if safety is None else {k:safety[1].get(k) for k in ('mode','output_cmd')}))
            if math.hypot(person['x']-root['x'], person['y']-root['y']) <= 2.:
                close_count += 1
                close_residuals.append(residual)
                if residual > .5:
                    close_missed += 1
                else:
                    matched = tracks[int(np.argmin(distances))]
                    radial_bias.append(math.hypot(matched['x']-pose['x'],matched['y']-pose['y'])-
                                       math.hypot(person['x']-root['x'],person['y']-root['y']))
                    truth_velocity = rotate(np.array((person['vx'],person['vy'])),angle)
                    velocity_errors.append(float(np.linalg.norm(np.array((matched['vx'],matched['vy']))-truth_velocity)))
    return {"odometry_drift_m": summary(drift), "nearby_truth_to_confirmed_track_m": summary(residuals),
            "within_2m_position_error_m": summary(close_residuals),
            "within_2m_velocity_error_mps": summary(velocity_errors),
            "within_2m_radial_bias_m": summary(radial_bias),
            "within_2m_unmatched_samples": close_missed, "within_2m_samples": close_count,
            **({'person_trace':trace} if person_id is not None else {}),
            "note": "Offline diagnostic only; nearby people can be occluded. Unmatched observations are assigned 10 m."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument('--person', type=int)
    parser.add_argument('--last', type=float, default=10.)
    args = parser.parse_args()
    report = evaluate(args.log,args.person,args.last)
    trace = report.pop('person_trace', [])
    print(json.dumps(report, indent=2))
    for item in trace:
        track = item['track']
        velocity = None if track is None else (track['vx'],track['vy'])
        decision = item['decision'] or {}
        safety = item['safety'] or {}
        print(f"{item['stamp']:.3f} distance={item['distance']:.3f} true_v={item['truth_velocity']} track_v={velocity} "
              f"state={decision.get('state')} cmd={decision.get('selected_cmd')} predicted_min={decision.get('min_human_distance')} "
              f"safety={safety.get('mode')} output={safety.get('output_cmd')}")
