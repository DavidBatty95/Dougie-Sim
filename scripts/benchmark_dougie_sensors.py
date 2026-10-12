#!/usr/bin/env python3
"""Compare batched sensor paths with their scalar reference implementations."""
import argparse
import json
import math
import time

import mujoco
import numpy as np

from dougie_sim.radiation import RadiationField, segment_box_length
from dougie_sim.reactor import radiation_description
from dougie_sim.simulation import Simulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=1000)
    parser.add_argument('--source-profile', default='hotspots-and-leaks')
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error('--iterations must be positive')
    sim = Simulation('reactor', controller='stand', radiation_sources=radiation_description(args.source_profile))
    field = RadiationField.from_mujoco(sim.model, sim.data)
    point = sim.data.qpos[:3]+[0., 0., .12]
    groups = np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8)
    geom = np.empty(1, dtype=np.int32)

    def scalar_field():
        return field.background+sum(
            s['strength_usv_h_m2']/(np.sum((point-np.asarray(s['position']))**2)+s['core_radius_m']**2)
            * math.exp(-sum(b['attenuation_per_m']*segment_box_length(s['position'], point, b['lower'], b['upper'])
                           for b in field.shields)) for s in field.sources)

    def scalar_scan():
        ranges = []
        for angle in np.linspace(-math.pi, math.pi, 180, endpoint=False):
            direction = np.array([math.cos(angle), math.sin(angle), 0.])
            d = mujoco.mj_ray(sim.model, sim.data, point, direction, groups, 1, sim.base, geom)
            ranges.append(d if .05 <= d <= 8. else math.inf)
        return ranges

    def timing(fn):
        for _ in range(20):
            fn()
        times = []
        for _ in range(args.iterations):
            start = time.perf_counter()
            fn()
            times.append((time.perf_counter()-start)*1e6)
        return dict(p50_us=float(np.median(times)), p95_us=float(np.percentile(times, 95)))

    result = dict(iterations=args.iterations, world='reactor', source_profile=args.source_profile,
                  sources=len(field.sources), shields=len(field.shields))
    for name, reference, optimized in (('radiation_field', scalar_field, lambda: field.rate(point)),
                                       ('horizontal_scan', scalar_scan, sim.scan)):
        np.testing.assert_allclose(reference(), optimized(), rtol=1e-12, atol=1e-12)
        before, after = timing(reference), timing(optimized)
        result[name] = dict(scalar=before, batched=after, median_speedup=before['p50_us']/after['p50_us'])
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
