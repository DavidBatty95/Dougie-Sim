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
_journal = None
_parameters = _terrain_profile = None
_recovery = None
_breadcrumbs = []
_support_captured = False
_checkpoints = None


def create_explorer(parameters, planner, terrain_profile, options=None):
    options = options or {}
    explorer = (DoseExplorer(*parameters) if planner == 'baseline' else
                 ClusteredDoseExplorer(*parameters, commitment=planner == 'clustered'))
    explorer.detector_likelihood = options.get('detector_likelihood', 'window')
    explorer.home_tolerance = .2 if options.get('consistent_mapping') else .4
    explorer.recovery_reserve_s = 3. if options.get('recovery_enabled') else 0.
    explorer.calibration_enabled = options.get('navigation_profile') == 'robust'
    if hasattr(explorer, 'objective'):
        explorer.objective = options.get('mission_objective', 'survey')
        explorer.smooth_execution = options.get('path_mode') == 'smooth'
        explorer.region_scheduling = options.get('region_scheduling', 'hierarchical')
        explorer.utility_mode = options.get('utility_mode', 'legacy')
        explorer.execution_admission_refinement = options.get('navigation_profile') == 'robust'
        from .observed_circuit import ObservedCircuit
        explorer.circuit = ObservedCircuit(options.get('circuit_radius', 1.5))
    if terrain_profile == 'conservative':
        explorer.terrain = SurfaceMap(radius=.4, max_step=.02, max_local_relief=.02)
    elif terrain_profile == 'layered':
        from .layered_terrain import LayeredSurfaceMap
        explorer.terrain = LayeredSurfaceMap(support_fusion=options.get('support_fusion', 'maximum'))
    if options.get('consistent_mapping') and options.get('path_mode') == 'smooth':
        from .path_motion import TRACKING_ERROR_LIMIT
        # The planned clearance must cover permitted execution error, not just
        # the robot footprint and localization residual at the path centreline.
        explorer.terrain.tracking_margin = TRACKING_ERROR_LIMIT
        explorer.terrain.base_radius += TRACKING_ERROR_LIMIT
        explorer.terrain.radius = explorer.terrain.base_radius
    if options.get('gp_mode') == 'spatial':
        from .spatial_gp import SpatialGP
        explorer.gp = SpatialGP(capacity=options.get('gp_expert_capacity', 32),
            max_experts=options.get('gp_max_experts', 128), cell_size=options.get('gp_cell_size', (4., 4., 1.5)),
            likelihood=parameters[4])
    elif options.get('gp_mode') == 'geometry':
        from .geometry_gp import GeometryPoissonGP
        if parameters[4] != 'poisson':
            raise ValueError('observed-geometry GP requires Poisson likelihood')
        explorer.gp = GeometryPoissonGP(capacity=parameters[2])
    return explorer


def initialize(parameters, planner, terrain_profile, limit_threads=True, options=None):
    global _explorer, _planner, _limits, _options, _journal, _parameters, _terrain_profile, _recovery, _breadcrumbs, _support_captured, _checkpoints
    # Never reconfigure native thread pools in a Python worker thread while the
    # main thread is executing Torch. The parent already limits the thread mode.
    if limit_threads:
        _limits = threadpool_limits(limits=1)
    _planner = planner
    _options = options or {}
    _parameters, _terrain_profile = parameters, terrain_profile
    from .map_corrections import ObservationJournal
    _journal = ObservationJournal() if _options.get('consistent_mapping') else None
    from .recovery import BudgetedRecovery
    _recovery = BudgetedRecovery() if _options.get('recovery_enabled') else None
    _breadcrumbs = []
    _support_captured = False
    from .localization_checkpoint import LocalizationCheckpoints
    _checkpoints = LocalizationCheckpoints(_options.get('localization_checkpoints', 0))
    _explorer = create_explorer(parameters, planner, terrain_profile, _options)


def observe_packet(explorer, packet):
    if hasattr(explorer.terrain, 'observe_packet'):
        explorer.terrain.observe_packet(packet)
    else:
        explorer.observe_geometry(packet.points, packet.supports, packet.origin)


def rebuild_explorer(old, new_events):
    """Reconstruct support and radiation in one revision before any route is used."""
    fresh = create_explorer(_parameters, _planner, _terrain_profile, _options)
    fresh.terrain.set_localization_margin(old.terrain.localization_margin)
    fresh.home = None if old.home is None else old.home.copy()
    fresh.home_stamp = getattr(old, 'home_stamp', 0.)
    for event in new_events:
        if fresh.home is not None:
            fresh.home = event.point(fresh.home, fresh.home_stamp)
    for packet in _journal.scans:
        observe_packet(fresh, packet)
        if hasattr(fresh.gp, 'observe_geometry'):
            fresh.gp.observe_geometry(packet)
    for sample in _journal.samples:
        fresh.observe_radiation(sample, replay=True)
        fresh.display_samples[tuple(np.floor(np.asarray(sample.position)/.4).astype(int))] = (np.asarray(sample.position), sample.rate_usv_h)
    fresh.display_samples = dict(list(fresh.display_samples.items())[-2048:])
    fresh.terrain.build_graph()
    valid_visits = [(stamp, point, key) for stamp, point in _journal.visits
                    if (key := fresh.terrain.nearest(point)) is not None]
    fresh.visited = {key for _, _, key in valid_visits}
    if getattr(fresh.terrain, 'active_viewpoints', False):
        # Restore the same observed body-neighbour visit proxy used online,
        # rather than turning old ramp footprints into novel tasks after a loop.
        roots = list(fresh.visited)
        for key in roots:
            fresh.visited.update(other for other in fresh.terrain.graph.get(key, [])
                                 if np.linalg.norm(fresh.terrain.nodes[other]-fresh.terrain.nodes[key]) <= .3)
        fresh.sampled_height_bands = {round(float(point[2])/.2) for _, point, _ in valid_visits}
        if _journal.visits:
            last_stamp, last_point = _journal.visits[-1]
            fresh.region_agenda.update(fresh.terrain.nodes, fresh.terrain.graph, fresh.terrain.nearest(last_point), last_stamp, fresh.visited)
            fresh.region_agenda.current = None
            fresh.region_agenda.last_served.clear()
            for visited_at, point, key in valid_visits:
                region = fresh.region_agenda.membership.get(key)
                if region is not None:
                    if fresh.region_agenda.current is not None and fresh.region_agenda.current != region:
                        fresh.region_agenda.transitions[fresh.region_agenda.current, region] += 1
                    fresh.region_agenda.current = region
                    fresh.region_agenda.last_served[region] = visited_at
    if hasattr(old, 'goal_switches'):
        fresh.goal_switches = old.goal_switches
    fresh.returning |= old.returning
    fresh.events = old.events
    fresh.worker_ms = old.worker_ms
    fresh.worker_phase_history = getattr(old, 'worker_phase_history', {})
    if hasattr(old, 'plan_delays'):
        fresh.plan_delays, fresh.wait_allowance = old.plan_delays, old.wait_allowance
    fresh.failed = old.failed
    fresh.calibration = old.calibration
    if hasattr(old, 'circuit'):
        import copy
        fresh.circuit = copy.deepcopy(old.circuit)
    for event in new_events:
        fresh.failed = [(event.point(point, stamp), stamp) for point, stamp in fresh.failed]
        if hasattr(fresh, 'circuit'):
            fresh.circuit.correct(event)
    fresh.last_reason = old.last_reason if old.returning else 'map_revision_revalidated'
    return fresh


def plan(batch, packet, position, stamp, stalled, landmarks=(), plan_delay_s=0., yaw=None, corrections=(), localization=None, execution_status=None):
    global _explorer, _breadcrumbs, _support_captured
    started = time.perf_counter()
    if _journal is not None:
        previous = _journal.revision
        if _journal.correct(corrections):
            new_events = [e for e in corrections if e.revision > previous]
            _explorer = rebuild_explorer(_explorer, new_events)
            for event in new_events:
                _breadcrumbs = [(t, event.point(p, t)) for t, p in _breadcrumbs]
        packet = _journal.align_packet(packet)
        batch = [_journal.align_sample(sample) for sample in batch]
        _journal.append(batch, packet, position, stamp)
    explorer = _explorer
    explorer.map_consistency = _journal.metrics() if _journal else dict(map_revision=0)
    explorer.localization = localization or {}
    if ((localization or {}).get('pose_graph') or {}).get('saturated', False):
        explorer.returning, explorer.last_reason = True, 'pose_graph_capacity'
    if _options.get('consistent_mapping') and max((localization or {}).get('position_std_m', [0.])) >= .18:
        if not explorer.returning:
            _checkpoints.request(localization or {})
        explorer.returning, explorer.last_reason = True, 'localization_return_reserve'
    if _journal and _journal.saturated:
        explorer.returning, explorer.last_reason = True, 'observation_journal_capacity'
    explorer.start_yaw = yaw
    if hasattr(explorer, 'observe_plan_delay'):
        explorer.observe_plan_delay(plan_delay_s)
    if hasattr(explorer, 'observe_landmarks'):
        explorer.observe_landmarks(landmarks)
    if hasattr(explorer.gp, 'observe_geometry'):
        explorer.gp.observe_geometry(packet)
    for sample in batch:
        explorer.observe_radiation(sample)
        key = tuple(np.floor(np.asarray(sample.position)/.4).astype(int))
        explorer.display_samples[key] = (np.asarray(sample.position), sample.rate_usv_h)
        if len(explorer.display_samples) > 2048:
            del explorer.display_samples[next(iter(explorer.display_samples))]
    if getattr(explorer.gp, 'saturated', False):
        explorer.returning = True
        explorer.last_reason = 'radiation_model_capacity_limit'
    observed = time.perf_counter()
    if hasattr(explorer.terrain, 'observe_packet'):
        # Repeated recovery plans may reference the same capture; do not count
        # its occupancy evidence repeatedly.
        if packet.stamp_s != getattr(explorer, 'last_packet_stamp', None):
            explorer.terrain.observe_packet(packet)
        if explorer.terrain.support_saturated or explorer.terrain.occupancy.saturated:
            explorer.returning = True
            explorer.last_reason = 'mapping_capacity_limit'
    else:
        if packet.stamp_s != getattr(explorer, 'last_packet_stamp', None):
            explorer.observe_geometry(packet.points, packet.supports, packet.origin)
    explorer.last_packet_stamp = packet.stamp_s
    explorer.last_packet_sequence = packet.sequence
    if _journal is not None:
        local_std = max((localization or {}).get('conditional_position_std_m', [.02]))
        residual = (localization or {}).get('scan_rmse_m') or 0.
        margin = min(.1, math.ceil(2.*math.hypot(local_std, residual)/.02)*.02)
        explorer.terrain.set_localization_margin(margin)
    if stalled and explorer.target is not None and explorer.path and execution_status != 'waiting_for_route_reacquisition':
        if hasattr(explorer, 'report_stall'):
            explorer.report_stall(position, stamp)
        else:
            explorer.failed.append((explorer.target.copy(), stamp))
    mapped = time.perf_counter()
    explorer.replan(position, stamp)
    if _checkpoints.limit:
        at_home = explorer.state == 'returned'
        capacity_ok = (not (_journal and (_journal.saturated or len(_journal.scans) >= 992))
            and not getattr(explorer.gp, 'saturated', False)
            and not getattr(explorer.terrain, 'support_saturated', False)
            and not getattr(getattr(explorer.terrain, 'occupancy', None), 'saturated', False)
            and not (((localization or {}).get('pose_graph') or {}).get('saturated', False)))
        admissible = capacity_ok and (_recovery is None or (_recovery.started is None and not _recovery.terminal))
        if at_home:
            key = explorer.terrain.nearest(explorer.home)
            plane = getattr(explorer.terrain, 'planes', {}).get(key)
            sample = explorer.sample
            rate = sample.rate_usv_h+3.*math.sqrt(sample.variance_usv_h2) if sample else math.inf
            admissible &= (plane is not None and np.linalg.norm(plane[:2]) <= math.tan(math.radians(2.))
                and sample is not None and 0 <= stamp-sample.stamp_s <= 1.5
                and sample.dose_upper_usv < .8*explorer.budget and rate <= explorer.rate_limit
                and sample.dose_upper_usv+explorer.return_reserve+rate*_checkpoints.wait_s/3600. < explorer.budget
                and stamp+_checkpoints.wait_s+15. < explorer.mission_duration)
        if _checkpoints.step(stamp, localization or {}, at_home, admissible, explorer.last_reason):
            explorer.returning = False
            explorer.target, explorer.path, explorer.return_path = None, [], []
            explorer.state, explorer.last_reason = 'checkpoint_revalidated', 'checkpoint_revalidated'
            explorer.route_forecast = {}
        elif at_home and _checkpoints.waiting:
            explorer.state, explorer.path = 'returning_checkpoint', []
    support_diagnosis = None
    if explorer.state in ('no_observed_return_route', 'waiting_for_home_support', 'waiting_for_support') and hasattr(explorer.terrain, 'support_diagnosis'):
        support_diagnosis = dict(state=explorer.state, reason=getattr(explorer, 'last_reason', explorer.state),
            current=explorer.terrain.support_diagnosis(position),
            home=None if explorer.home is None else explorer.terrain.support_diagnosis(explorer.home))
        if _options.get('support_capture_path') and not _support_captured and explorer.home is not None:
            save_support_archive(_options['support_capture_path'], 'first_support_failure')
            _support_captured = True
    if _journal is not None and stamp-packet.stamp_s > .75:
        explorer.path, explorer.state, explorer.last_reason = [], 'waiting_for_range', 'stale_range_packet'
    if _recovery is not None and explorer.state != 'returned':
        apply_recovery(explorer, packet, position, stamp, stalled, execution_status, localization or {})
    planned = time.perf_counter()
    execution_path = [p.copy() for p in explorer.path]
    motion = None
    smoother = getattr(explorer, 'smoother', None)
    if _options.get('path_mode') == 'smooth' and execution_path:
        from .path_motion import RouteSmoother
        smoother = smoother or RouteSmoother(explorer.terrain, explorer.gp, explorer.rate_limit)
        motion = smoother([position, *execution_path], explorer.state.startswith('return'))
        execution_path = motion.points
    smoothed = time.perf_counter()
    nodes = np.asarray(list(explorer.terrain.nodes.values())).reshape(-1, 3)
    volume = None
    if len(nodes) and _options.get('display_prediction', True):
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
    displayed = time.perf_counter()
    metrics = explorer.metrics()
    metrics['localization_checkpoints'] = _checkpoints.metrics()
    if support_diagnosis is not None:
        metrics['support_diagnosis'] = support_diagnosis
    metrics.update(planner=_planner, selected_target=None if explorer.target is None else explorer.target.tolist(),
                   planned_path=[p.tolist() for p in execution_path], map_revision=_journal.revision if _journal else 0)
    if motion is not None:
        metrics.update(grid_path=[p.tolist() for p in explorer.path], path_spans=motion.spans, motion=motion.metrics)
        if explorer.state.startswith('return'):
            metrics['return_path'] = metrics['planned_path']
        elif metrics.get('return_path') and _options.get('smooth_return_overlay', True):
            return_motion = smoother([position, *metrics['return_path']], returning=True)
            metrics['grid_return_path'] = metrics['return_path']
            metrics['return_path'] = [p.tolist() for p in return_motion.points]
            metrics['return_motion'] = return_motion.metrics
    if hasattr(explorer.terrain, 'metrics'):
        metrics['terrain'] = explorer.terrain.metrics()
    metrics['smoothing_cache_hits'] = smoother.hits if smoother else 0
    finished = time.perf_counter()
    phases = dict(observations=(observed-started)*1000., mapping=(mapped-observed)*1000.,
        planning=(planned-mapped)*1000., execution_smoothing=(smoothed-planned)*1000.,
        display_prediction=(displayed-smoothed)*1000., metrics_and_return_overlay=(finished-displayed)*1000.)
    metrics['worker_phase_ms'] = phases
    explorer.worker_phase_history = getattr(explorer, 'worker_phase_history', {})
    for name, value in phases.items():
        explorer.worker_phase_history[name] = (explorer.worker_phase_history.get(name, [])+[value])[-256:]
    metrics['worker_phase_p95_ms'] = {name: float(np.percentile(values, 95))
                                     for name, values in explorer.worker_phase_history.items()}
    explorer.worker_ms.append((time.perf_counter()-started)*1000.)
    if len(explorer.worker_ms) > 4096:
        explorer.worker_ms = explorer.worker_ms[-2048:]
    metrics['worker_p95_ms'] = float(np.percentile(explorer.worker_ms, 95))
    return execution_path, explorer.state, metrics, volume


def apply_recovery(explorer, packet, position, stamp, stalled, execution_status, localization):
    from .recovery import failure_reason
    from .path_motion import RouteGeometry
    from .route_forecast import forecast_polyline
    reason = failure_reason(explorer, packet, stamp, localization, stalled, execution_status)
    sample = explorer.sample
    measured, upper = (sample.dose_usv, sample.dose_upper_usv) if sample else (0., 0.)
    rate = sample.rate_usv_h+3.*math.sqrt(sample.variance_usv_h2) if sample else explorer.rate_limit
    permitted_sensing = reason not in ('localization_unreliable', 'stale_range', 'stale_radiation')
    valid_route = bool(explorer.path) and permitted_sensing
    forecast = getattr(explorer, 'route_forecast', {})
    decision = _recovery.step(reason, stamp, measured, upper, rate, explorer.budget,
        explorer.return_reserve, explorer.mission_duration, forecast.get('return_duration_s', 0.), valid_route)
    if decision.terminal:
        explorer.path, explorer.state, explorer.last_reason = [], 'stopped_unrecoverable', _recovery.reason
    elif decision.phase == 'resensing':
        explorer.path, explorer.state, explorer.last_reason = [], 'recovery_resensing', reason or _recovery.reason
    elif decision.force_return:
        known_return = explorer.path if explorer.state.startswith('return') and explorer.path else getattr(explorer, 'return_path', [])
        explorer.returning = True
        if permitted_sensing and known_return:
            explorer.path = [p.copy() for p in known_return]
            explorer.state, explorer.last_reason = 'returning_recovery', 'recovery_known_return_route'
        elif permitted_sensing and _recovery.allow_attempt(stamp):
            geometry = RouteGeometry(explorer.terrain)
            explorer.path = []
            for observed_at, point in reversed(_breadcrumbs):
                if not 3. <= stamp-observed_at <= 60. or not .2 <= np.linalg.norm(point-position) <= .8:
                    continue
                checked = geometry.line(np.asarray(position), point)
                if checked is None:
                    continue
                _, _, rates = explorer.gp.predict(checked+[0., 0., .44])
                seconds, dose = forecast_polyline(checked, rates, start_yaw=getattr(explorer, 'start_yaw', None))
                if (float(np.max(rates)) > explorer.rate_limit or upper+dose+explorer.return_reserve >= explorer.budget
                        or stamp+seconds+15. >= explorer.mission_duration):
                    continue
                explorer.path, explorer.state = list(checked), 'returning_recovery'
                explorer.last_reason = 'checked_breadcrumb_retreat'
                break
            if not explorer.path:
                explorer.state, explorer.last_reason = 'recovery_resensing', 'no_checked_recovery_motion'
        else:
            explorer.path, explorer.state, explorer.last_reason = [], 'recovery_resensing', reason or 'recovery_attempt_limit'
    if decision.phase in ('idle', 'recovered') and explorer.terrain.nearest(position) is not None:
        if not _breadcrumbs or np.linalg.norm(np.asarray(position)-_breadcrumbs[-1][1]) > .2:
            _breadcrumbs.append((stamp, np.asarray(position).copy()))
            del _breadcrumbs[:-128]
    explorer.recovery = _recovery.metrics()


def save_support_archive(path, scope='final'):
    if _journal is None:
        raise ValueError('support replay capture requires an observation journal')
    terrain = _explorer.terrain
    points = [packet.points for packet in _journal.scans]
    supports = [packet.supports for packet in _journal.scans]
    np.savez_compressed(path, schema_version=1, map_revision=_journal.revision,
        capture_scope=scope, planning_state=_explorer.state,
        planning_reason=getattr(_explorer, 'last_reason', None) or _explorer.state,
        journal_saturated=_journal.saturated,
        stamps_s=np.array([packet.stamp_s for packet in _journal.scans]),
        origins=np.array([packet.origin for packet in _journal.scans]).reshape(-1, 3),
        point_offsets=np.r_[0, np.cumsum([len(p) for p in points], dtype=np.int64)],
        support_offsets=np.r_[0, np.cumsum([len(p) for p in supports], dtype=np.int64)],
        points=np.concatenate(points) if points else np.empty((0, 3)),
        support_points=np.concatenate(supports) if supports else np.empty((0, 3)),
        home=np.array([]) if _explorer.home is None else _explorer.home,
        last_position=np.array([]) if not _journal.visits else _journal.visits[-1][1],
        resolution=terrain.resolution, base_radius=terrain.base_radius,
        localization_margin=terrain.localization_margin, max_slope_deg=math.degrees(math.atan(terrain.max_slope)),
        residual_limit=terrain.residual_limit, support_capacity=terrain.support_capacity,
        support_fusion=terrain.support_fusion, fusion_span_m=terrain.fusion_span_m,
        archived_nodes=len(terrain.nodes))


def finalize(pending_samples, corrections=(), support_archive=None):
    """Drain every remaining count sample and transfer the final archive state."""
    global _explorer
    if _journal is not None:
        previous = _journal.revision
        if _journal.correct(corrections):
            _explorer = rebuild_explorer(_explorer, [e for e in corrections if e.revision > previous])
        pending_samples = [_journal.align_sample(sample) for sample in pending_samples]
    for sample in pending_samples:
        if _journal is not None:
            if _journal.samples and sample.stamp_s <= _journal.samples[-1].stamp_s:
                raise ValueError('final count observations must not be assimilated twice')
            if len(_journal.samples) < _journal.sample_capacity:
                _journal.samples.append(sample)
            else:
                _journal.saturated = True
        _explorer.observe_radiation(sample)
    if _journal is not None:
        _explorer.map_consistency = _journal.metrics()
    if support_archive is not None:
        save_support_archive(support_archive)
    _explorer.localization_checkpoints = _checkpoints.metrics()
    return _explorer
