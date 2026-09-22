#!/usr/bin/env python3
"""Summarize command-chain behavior and latency from a human-aware JSONL run."""

import argparse
from collections import Counter, defaultdict
import json
import math


def summarize(path, last_seconds=0.0, include_tracker=False):
    counts = defaultdict(Counter)
    samples = defaultdict(list)
    trace = []
    last_trace = -math.inf
    pose = None
    tracker_trace = []
    people = None
    for line in open(path, encoding="utf-8"):
        row = json.loads(line)
        topic, data = row.get("topic"), row.get("data")
        if not isinstance(data, dict):
            continue
        if topic == "odom":
            pose = data
        if topic == "mujoco_people":
            people = data.get("people")
        if topic == "planner_debug":
            if data.get("static_error"):
                counts["static_errors"][data["static_error"].split(".")[0]] += 1
            if last_seconds > 0.0 and row["stamp"] - last_trace >= 0.5:
                last_trace = row["stamp"]
                trace.append({"stamp": row["stamp"], "state": data.get("state"), "selected": data.get("selected"), "cmd": data.get("selected_cmd"), "policy_command": data.get("policy_command"), "pose": pose, "velocity": data.get("measured_velocity"), "person": data.get("closest_track"), "nearest_static": data.get("nearest_static"), "static_error": data.get("static_error")})
                trace[-1]["evaluation_people_world"] = people
                trace[-1]["planning_ms"] = data.get("planning_ms")
                trace[-1]["tracks_age_s"] = data.get("tracks_age_s")
            counts["planner_states"][data.get("state", "unknown")] += 1
            counts["selected_candidates"][data.get("selected", "unknown")] += 1
            counts["reference_status"][data.get("path_reference_status", "not_recorded")] += 1
            for key in ("planning_ms", "planning_thread_ms", "tracks_age_s", "prediction_age_s"):
                if isinstance(data.get(key), (float, int)) and math.isfinite(data[key]):
                    samples[key].append(data[key])
            for key, value in data.get("planning_phases_ms", {}).items():
                samples[key].append(value)
            for candidate in data.get("candidate_results", []):
                if candidate.get("rejected"):
                    counts["rejection_reasons"][candidate.get("reason", "unknown")] += 1
        elif topic == "tracker_debug":
            for key, value in data.get("processing_phases_ms", {}).items():
                samples["tracker_" + key + "_ms"].append(value)
            for event in data.get("events", []):
                reason = event.get("reason") or event.get("rejection_reason")
                if reason:
                    category = "tracker_events" if event.get("state") == "partial" else "tracker_rejections"
                    counts[category][reason] += 1
            if include_tracker and last_seconds > 0 and (not tracker_trace or row["stamp"] - tracker_trace[-1]["stamp"] >= .5):
                tracker_trace.append({"stamp": row["stamp"], "tracks": data.get("tracks"), "events": data.get("events")})
            for key in ("previous_processing_ms", "source_age_s"):
                if isinstance(data.get(key), (float, int)) and math.isfinite(data[key]):
                    samples[key].append(data[key])
        elif topic == "safety_debug":
            counts["supervisor_modes"][data.get("mode", "unknown")] += 1
            counts["supervisor_reasons"][data.get("reason", "unknown")] += 1
        elif topic == "localization_status":
            counts["localization_status"]["accepted" if data.get("accepted") else data.get("reason", "rejected")] += 1
            for source, target in (("processing_ms", "localization_ms"), ("input_age_s", "localization_age_s")):
                if isinstance(data.get(source), (float, int)) and math.isfinite(data[source]):
                    samples[target].append(data[source])
        elif topic == "replanning_status":
            counts["route_replanning"][data.get("reason", "unknown")] += 1
            value = data.get("request_to_route_ms")
            if isinstance(value, (int, float)) and math.isfinite(value):
                samples["request_to_matching_route_ms"].append(value)
    return {
        **({"trace": [row for row in trace if row["stamp"] >= last_trace - last_seconds]} if trace else {}),
        **({"tracker_trace": [row for row in tracker_trace if row["stamp"] >= last_trace - last_seconds]} if tracker_trace else {}),
        **{key: dict(value.most_common()) for key, value in counts.items()},
        "timing": {key: {
            "samples": len(values),
            "p50": sorted(values)[int((len(values) - 1) * 0.50)],
            "p95": sorted(values)[int((len(values) - 1) * 0.95)],
            "max": max(values),
        } for key, values in samples.items() if values},
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--last", type=float, default=0.0, help="Include the last N seconds of planner decisions")
    parser.add_argument("--tracker", action="store_true", help="Include detailed tracker events with --last")
    args = parser.parse_args()
    print(json.dumps(summarize(args.log, args.last, args.tracker), indent=2))
