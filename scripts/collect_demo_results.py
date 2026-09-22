#!/usr/bin/env python3
"""Publish compact, checksum-linked records without copying large runtime logs."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIELDS = ("run_id", "timestamp", "seed", "backend", "benchmark_name", "pedestrian_scenario",
          "onboard_only", "visual_session", "session_end_reason", "data_sources", "people_count",
          "goals_requested", "goals_reached", "valid_trial", "invalid_reason", "success", "failure_reason",
          "total_time_s", "actual_path_length_m", "minimum_body_clearance_m", "collisions", "near_misses",
          "fallen", "final_score", "metric_role", "provenance", "command", "duration_s",
          "net_displacement_m", "average_path_speed_mps", "reactivity", "max_tilt_rad",
          "accumulated_yaw_change_rad", "average_yaw_rate_radps")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", nargs=3, action="append", required=True, metavar=("ID", "ROLE", "JSON"))
    parser.add_argument("--output", type=Path, default=ROOT/"docs/results/demo_results.json")
    args = parser.parse_args()
    records = []
    for identity, role, filename in args.run:
        path = Path(filename).resolve()
        raw = path.read_bytes()
        data = json.loads(raw)
        record = {"id": identity, "role": role, "source_artifact": str(path.relative_to(ROOT)),
                  "source_sha256": hashlib.sha256(raw).hexdigest(),
                  "result": {name: data[name] for name in FIELDS if name in data}}
        status = path.parent/"suite_status.json"
        if status.exists():
            record["suite_status"] = json.loads(status.read_text())
        records.append(record)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"schema_version": 1,
        "scope": "Observed development runs; not pooled performance across configurations",
        "records": records}, indent=2)+"\n")
    print(args.output)


if __name__ == "__main__":
    main()
