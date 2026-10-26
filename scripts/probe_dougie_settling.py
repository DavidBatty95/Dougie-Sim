#!/usr/bin/env python3
"""Replay one retained stopping fixture and record onboard gyro components at entry."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from threadpoolctl import threadpool_limits

from dougie_sim.onboard_sensors import OnboardSensors
from scripts.qualify_dougie_stance_disturbance import trial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    original = OnboardSensors.proprioception
    measurements = []
    def observe(self, sim):
        measured = original(self, sim)
        if 13. <= measured.stamp_s <= 16.:
            measurements.append(dict(time_s=measured.stamp_s, gyro=measured.gyro.tolist(),
                minimum_foot_force_n=float(np.min(measured.foot_force)),
                maximum_joint_speed_rad_s=float(np.max(np.abs(measured.joint_velocity)))))
        return measured
    OnboardSensors.proprioception = observe
    try:
        with threadpool_limits(limits=1):
            result = trial(8.53, 'down', .25, 0., 2, 20., 2.)
    finally:
        OnboardSensors.proprioception = original
    gyro = np.array([row['gyro'] for row in measurements])
    report = dict(scope=__doc__, result=result, entry_measurements=measurements,
        gyro_component_rms_rad_s=np.sqrt(np.mean(gyro**2, axis=0)).tolist(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(gyro_component_rms_rad_s=report['gyro_component_rms_rad_s'], stance=result['stance']), indent=2))


if __name__ == '__main__':
    main()
