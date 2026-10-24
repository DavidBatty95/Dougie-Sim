#!/usr/bin/env python3
"""Evaluator-only static observability probe, NOT an autonomy or mission test.

An archived physical pose initializes a sensor fixture. Registration gets a
same-pose reference and ideal initial alignment to isolate measurement geometry.
These oracle-initialized results must never be used as onboard pose estimates.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from threadpoolctl import threadpool_limits

from dougie_sim.onboard_sensors import OnboardSensors
from dougie_sim.registration import register
from dougie_sim.scan_geometry import registration_cloud, sample_registration_cloud
from dougie_sim.simulation import Simulation, rotation
from dougie_sim.pose_uncertainty import core_fingerprint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--stamp', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sampling', choices=('spatial', 'normal-balanced'), default='spatial')
    args = parser.parse_args()
    with np.load(args.run/'motion_trace.npz', allow_pickle=False) as archive:
        rows = archive['rows']
    row = rows[np.argmin(np.abs(rows[:, 0]-args.stamp))]
    if abs(row[0]-args.stamp) > .051:
        raise ValueError('requested pose is not in the physical trace')
    sim = Simulation(controller='stand', model_xml=(args.run/'scene.xml').read_text())
    sim.data.qpos[:] = row[1:20]
    mujoco.mj_forward(sim.model, sim.data)
    R, p = rotation(sim.data.qpos[3:7]), sim.data.qpos[:3].copy()
    def cloud(seed):
        points, normals = registration_cloud(OnboardSensors(seed).scan(sim))
        raw_information = np.linalg.eigvalsh((normals@R.T).T@(normals@R.T)).tolist()
        raw_count = len(points)
        points, normals = sample_registration_cloud(points, normals, args.sampling)
        return points, normals, raw_count, raw_information
    reference, normal, _, _ = cloud(10000)
    results = []
    with threadpool_limits(limits=1):
        for seed in range(4):
            points, normals, raw_count, raw_information = cloud(seed)
            fit = register(points, normals, reference@R.T+p, normal@R.T, p, R)
            world_normals = normals@R.T
            results.append(dict(seed=seed, points=len(points), rank=fit.observable_rank,
                raw_points=raw_count, raw_normal_information_eigenvalues=raw_information,
                accepted=fit.accepted, residual_m=fit.rmse if np.isfinite(fit.rmse) else None,
                normal_information_eigenvalues=np.linalg.eigvalsh(world_normals.T@world_normals).tolist(),
                translation_error_m=float(np.linalg.norm(fit.position-p))))
    report = dict(scope=__doc__, stamp_s=float(row[0]), sampling=args.sampling, results=results, source_sha256=core_fingerprint(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        input_sha256={name: hashlib.sha256((args.run/name).read_bytes()).hexdigest() for name in ('scene.xml', 'motion_trace.npz')})
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
