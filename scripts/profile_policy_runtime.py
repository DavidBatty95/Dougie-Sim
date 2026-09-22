#!/usr/bin/env python3
"""Compare ONNX deployment latency and CPU duty in a sourced ROS environment."""

import argparse
import json
import time

import numpy as np
import onnxruntime as ort

from unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller import OnnxPolicy


def measure(call, cycles, period):
    for _ in range(20):
        call()
    start, cpu = time.perf_counter(), time.process_time()
    latencies = []
    for index in range(cycles):
        tick = time.perf_counter()
        call()
        latencies.append((time.perf_counter() - tick) * 1000.0)
        time.sleep(max(0.0, start + (index + 1) * period - time.perf_counter()))
    return {
        "wall_seconds": time.perf_counter() - start,
        "process_cpu_seconds": time.process_time() - cpu,
        "median_ms": float(np.median(latencies)),
        "p95_ms": float(np.percentile(latencies, 95)),
        "max_ms": max(latencies),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policy")
    parser.add_argument("--cycles", type=int, default=100)
    parser.add_argument("--period", type=float, default=0.02)
    args = parser.parse_args()
    if args.cycles < 1 or args.period <= 0.0:
        parser.error("cycles and period must be positive")
    session = ort.InferenceSession(args.policy, providers=["CPUExecutionProvider"])
    entry = session.get_inputs()[0]
    obs = np.zeros((1, int(entry.shape[1])), dtype=np.float32)
    default = measure(lambda: session.run(None, {entry.name: obs}), args.cycles, args.period)
    del session
    policy = OnnxPolicy(args.policy)
    bounded = measure(lambda: policy(obs), args.cycles, args.period)
    print(json.dumps({"policy": args.policy, "default_pool": default, "single_worker": bounded}, indent=2))


if __name__ == "__main__":
    main()
