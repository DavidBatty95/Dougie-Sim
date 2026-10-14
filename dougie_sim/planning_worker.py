"""Single-owner mission planning service for thread or spawned-process execution."""
import math
import time

import numpy as np
from threadpoolctl import threadpool_limits

from .clustered_planner import ClusteredDoseExplorer
from .dose_planner import DoseExplorer
from .terrain import SurfaceMap

_explorer = None
_planner = None
_limits = None
_options = {}


def create_explorer(parameters, planner, terrain_profile, options=None):
    options = options or {}
    explorer = (DoseExplorer(*parameters) if planner == 'baseline' else
                ClusteredDoseExplorer(*parameters, commitment=planner == 'clustered'))
    if terrain_profile == 'conservative':
        explorer.terrain = SurfaceMap(radius=.4, max_step=.02, max_local_relief=.02)
    elif terrain_profile == 'layered':
        from .layered_terrain import LayeredSurfaceMap
        explorer.terrain = LayeredSurfaceMap()
    if options.get('gp_mode') == 'spatial':
        from .spatial_gp import SpatialGP
        explorer.gp = SpatialGP(capacity=options.get('gp_expert_capacity', 32),
            max_experts=options.get('gp_max_experts', 128), cell_size=options.get('gp_cell_size', (4., 4., 1.5)),
            likelihood=parameters[4])
    return explorer


def initialize(parameters, planner, terrain_profile, limit_threads=True, options=None):
    global _explorer, _planner, _limits, _options
    # Never reconfigure native thread pools in a Python worker thread while the
    # main thread is executing Torch. The parent already limits the thread mode.
    if limit_threads:
        _limits = threadpool_limits(limits=1)
    _planner = planner
    _options = options or {}
    _explorer = create_explorer(parameters, planner, terrain_profile, _options)


def plan(batch, packet, position, stamp, stalled, landmarks=()):
    started = time.perf_counter()
    explorer = _explorer
    if hasattr(explorer, 'observe_landmarks'):
        explorer.observe_landmarks(landmarks)
    for sample in batch:
        explorer.observe_radiation(sample)
        key = tuple(np.floor(np.asarray(sample.position)/.4).astype(int))
        explorer.display_samples[key] = (np.asarray(sample.position), sample.rate_usv_h)
        if len(explorer.display_samples) > 2048:
            del explorer.display_samples[next(iter(explorer.display_samples))]
    if getattr(explorer.gp, 'saturated', False):
        explorer.returning = True
        explorer.last_reason = 'radiation_model_capacity_limit'
    if hasattr(explorer.terrain, 'observe_packet'):
        explorer.terrain.observe_packet(packet)
        if explorer.terrain.support_saturated or explorer.terrain.occupancy.saturated:
            explorer.returning = True
            explorer.last_reason = 'mapping_capacity_limit'
    else:
        explorer.observe_geometry(packet.points, packet.supports, packet.origin)
    if stalled and explorer.target is not None:
        if hasattr(explorer, 'report_stall'):
            explorer.report_stall(position, stamp)
        else:
            explorer.failed.append((explorer.target.copy(), stamp))
    explorer.replan(position, stamp)
    execution_path = [p.copy() for p in explorer.path]
    motion = None
    if _options.get('path_mode') == 'smooth' and execution_path:
        from .path_motion import smooth_route
        motion = smooth_route([position, *execution_path], explorer.terrain, explorer.gp,
                              explorer.rate_limit, explorer.state.startswith('return'))
        execution_path = motion.points
    nodes = np.asarray(list(explorer.terrain.nodes.values())).reshape(-1, 3)
    volume = None
    if len(nodes):
        nodes = nodes[::max(1, math.ceil(len(nodes)/100))]
        queries = [nodes+[0., 0., z] for z in (.25, .75, 1.5)]
        # Spend part of the display budget at real detector locations, including
        # strong readings, instead of losing narrow detections in a large grid.
        samples = list(explorer.display_samples.values())
        if samples:
            strongest = [p for p, _ in sorted(samples, key=lambda item: -item[1])[:75]]
            spread = [samples[i][0] for i in np.linspace(0, len(samples)-1, min(75, len(samples))).astype(int)]
            queries.append(np.asarray(strongest+spread).reshape(-1, 3))
        query = np.unique(np.concatenate(queries), axis=0)
        volume = (query, *explorer.gp.predict(query))
    metrics = explorer.metrics()
    metrics.update(planner=_planner, selected_target=None if explorer.target is None else explorer.target.tolist(),
                   planned_path=[p.tolist() for p in execution_path])
    if motion is not None:
        metrics.update(grid_path=[p.tolist() for p in explorer.path], path_spans=motion.spans, motion=motion.metrics)
        if explorer.state.startswith('return'):
            metrics['return_path'] = metrics['planned_path']
        elif metrics.get('return_path') and _options.get('smooth_return_overlay', True):
            return_motion = smooth_route([position, *metrics['return_path']], explorer.terrain, explorer.gp,
                                         explorer.rate_limit, returning=True)
            metrics['grid_return_path'] = metrics['return_path']
            metrics['return_path'] = [p.tolist() for p in return_motion.points]
            metrics['return_motion'] = return_motion.metrics
    if hasattr(explorer.terrain, 'metrics'):
        metrics['terrain'] = explorer.terrain.metrics()
    explorer.worker_ms.append((time.perf_counter()-started)*1000.)
    if len(explorer.worker_ms) > 4096:
        explorer.worker_ms = explorer.worker_ms[-2048:]
    metrics['worker_p95_ms'] = float(np.percentile(explorer.worker_ms, 95))
    return execution_path, explorer.state, metrics, volume


def finalize(pending_samples):
    """Drain every remaining count sample and transfer the final archive state."""
    for sample in pending_samples:
        _explorer.observe_radiation(sample)
    return _explorer
