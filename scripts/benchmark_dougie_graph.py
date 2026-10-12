#!/usr/bin/env python3
"""Same-scan geometry and routing microbenchmark; checks numerical equivalence."""
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from threadpoolctl import threadpool_limits

from dougie_sim.reference_terrain import ReferenceSurfaceMap
from dougie_sim.simulation import Simulation
from dougie_sim.terrain import SurfaceMap


def main():
    with threadpool_limits(limits=1):
        sim = Simulation('reactor')
        reference, optimized = ReferenceSurfaceMap(), SurfaceMap()
        for i in range(2400):
            sim.step((.2, 0., 0.))
            if i % 400 == 0:
                points, supports = sim.lidar3d()
                for grid in (reference, optimized):
                    grid.update(points, supports)
        reference.build_graph()
        optimized.build_graph()
        assert reference.graph == optimized.graph
        assert reference.nodes.keys() == optimized.nodes.keys()
        root = reference.nearest([0., 0., 0.])
        rates = {key: 1.+abs(point[0]) for key, point in reference.nodes.items()}
        old, new = reference.routes(root, rates), optimized.routes(root, rates)
        for a, b in zip(old[:3], new[:3]):
            assert a.keys() == b.keys()
            np.testing.assert_allclose([a[key] for key in a], [b[key] for key in a], rtol=1e-10, atol=1e-10)
        candidates = sorted(reference.nodes)[::max(1, len(reference.nodes)//160)]
        assert [reference.novelty(key) for key in candidates] == [optimized.novelty(key) for key in candidates]
        timings = {}
        for name, grid in (('reference', reference), ('optimized', optimized)):
            values = dict(graph=[], routes=[], novelty=[])
            for _ in range(9):
                start = time.perf_counter(); grid.build_graph(); values['graph'].append((time.perf_counter()-start)*1000.)
                start = time.perf_counter(); grid.routes(root, rates); values['routes'].append((time.perf_counter()-start)*1000.)
                start = time.perf_counter(); [grid.novelty(key) for key in candidates]; values['novelty'].append((time.perf_counter()-start)*1000.)
            timings[name] = {key: dict(median_ms=float(np.median(v)), p95_ms=float(np.percentile(v, 95))) for key, v in values.items()}
    result = dict(scope='same observed reactor scan sequence; experimental geometry settings; not full mission speed',
        nodes=len(reference.nodes), edges=sum(map(len, reference.graph.values())), candidates=len(candidates),
        graph_equal=True, path_costs_equal_to_tolerance=1e-10, novelty_equal=True, timings=timings,
        source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in
            (Path('dougie_sim/terrain.py'), Path('dougie_sim/reference_terrain.py'))})
    path = Path('results/dougie/ori_graph_benchmark.json')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
