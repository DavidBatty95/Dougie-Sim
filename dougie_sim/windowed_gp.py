"""Online XYZ-local geometry GPs with a bounded, disk-paged working set.

Only experts overlapping an observation are updated. Counts spanning a cell
boundary remain one integrated Poisson observation in each correlated expert;
predictions use mixture moments, never a product of independent precisions.
Inactive beliefs are saved, not discarded, and no global sample/expert count
ends the mission. RAM and matrix sizes are bounded; retained map storage grows.
"""
from collections import OrderedDict
import io
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import zipfile

import numpy as np

from .geometry_gp import GeometryPoissonGP
from .spatial_gp import SpatialGP


class WindowedGP(SpatialGP):
    def __init__(self, capacity=32, max_experts=16, cell_size=(4., 4., 1.5), halo=.2,
                 replay_capacity=256, max_planes=8, store_dir=None, blend='quintic',upper_rule='evidence'):
        if replay_capacity < 1 or max_planes < 0:
            raise ValueError('invalid local GP history/geometry capacity')
        self.replay_capacity, self.max_planes = int(replay_capacity), int(max_planes)
        super().__init__(capacity, max_experts, cell_size, halo, 'poisson',blend=blend,upper_rule=upper_rule)
        self.experts = OrderedDict()
        self.catalog = {}
        self.dirty = set()
        self.store_dir = Path(store_dir or tempfile.mkdtemp(prefix='dougie-gp-windows-'))
        self.store_dir.mkdir(parents=True, exist_ok=True)
        self.archive_path = None
        self.page_reads = self.page_writes = self.evictions = 0
        self.prediction_page_reads = 0
        self.last_window_experts = 0
        self.last_geometry_packet = None
        self.prior_calibration_windows = min(8, self.replay_capacity)
        self.prior_calibrated = False
        self.calibration_counts = 0
        self.calibration_exposure = 0.
        self.calibration_windows = 0
        self.new_epoch=False
        self.epoch_reset_keys=set()

    def start_new_mission(self):
        self.new_epoch=True
        self.epoch_reset_keys=set()

    def _mission_clock(self,key,expert):
        if self.new_epoch and key not in self.epoch_reset_keys:
            expert.last_geometry_stamp=-math.inf
            self.epoch_reset_keys.add(key)
        return expert

    def _new_expert(self):
        expert = GeometryPoissonGP(self.capacity, self.max_planes, self.replay_capacity,
                                   streaming_after_capacity=True)
        if hasattr(self, 'prior'):
            expert.prior, expert.variance = self.prior, self.variance
            expert.lengthscales = self.lengthscales.copy()
        return expert

    @staticmethod
    def name(key):
        return '_'.join(str(int(v)) for v in key)+'.npz'

    def _remember(self, key, expert):
        self.catalog[key] = dict(updates=expert.updates, inducing_points=len(expert.Z),
                                 geometry_frozen=expert.geometry_frozen)

    def _store(self, key, expert):
        temporary = self.store_dir/(self.name(key)+'.tmp')
        with temporary.open('wb') as stream:
            expert.save(stream)
        temporary.replace(self.store_dir/self.name(key))
        self._remember(key, expert)
        self.dirty.discard(key)
        self.page_writes += 1

    def _get(self, key, create=False):
        key = tuple(map(int, key))
        if key in self.experts:
            expert = self.experts.pop(key)
            self.experts[key] = expert
            return self._mission_clock(key,expert)
        if key not in self.catalog and not create:
            return None
        while len(self.experts) >= self.max_experts:
            old_key, old = self.experts.popitem(last=False)
            if old_key in self.dirty:
                self._store(old_key, old)
            self.evictions += 1
        if key in self.catalog:
            from .spatial_gp import load_gp
            cached = self.store_dir/self.name(key)
            if cached.exists():
                expert = load_gp(cached)
            elif self.archive_path is not None:
                with zipfile.ZipFile(self.archive_path) as archive:
                    expert = load_gp(io.BytesIO(archive.read('experts/'+self.name(key))))
            else:
                raise RuntimeError(f'Missing stored GP window {key}; refusing to replace learned beliefs with a prior')
            self.page_reads += 1
        else:
            expert = self._new_expert()
            self.dirty.add(key)
            self._remember(key, expert)
        self.experts[key] = expert
        return self._mission_clock(key,expert)

    def _prediction_expert(self, key):
        # Unknown query regions keep an uncertainty-bearing prior; queries never
        # allocate a GP or evict a training/history window. A cold prediction
        # streams only compact posterior/geometry arrays, not replay records.
        if key in self.experts:
            return self.experts[key]
        if key not in self.catalog:
            return None
        from .spatial_gp import load_gp
        cached = self.store_dir/self.name(key)
        if cached.exists():
            expert = load_gp(cached, include_history=False)
        elif self.archive_path is not None:
            with zipfile.ZipFile(self.archive_path) as archive:
                with archive.open('experts/'+self.name(key)) as stream:
                    expert = load_gp(stream, include_history=False)
        else:
            raise RuntimeError(f'Missing stored GP prediction window {key}')
        self.page_reads += 1
        self.prediction_page_reads += 1
        return expert

    def _observe_local_geometry(self, key, expert, packet):
        if (expert.geometry_frozen or len(expert.planes) >= expert.max_planes
                or packet.stamp_s-expert.last_geometry_stamp < 10.):
            return
        points = packet.origin+packet.directions*packet.distances[..., None]
        lower = (np.asarray(key)-self.halo)*self.cell_size
        upper = (np.asarray(key)+1.+self.halo)*self.cell_size
        valid = packet.valid & np.all((points >= lower) & (points <= upper), axis=-1)
        local = SimpleNamespace(stamp_s=packet.stamp_s, origin=packet.origin,
            directions=packet.directions, distances=packet.distances, valid=valid)
        expert.observe_geometry(local)
        self.dirty.add(key)

    def observe_geometry(self, packet):
        self.last_geometry_packet = packet
        keys, weights = self.assignments(np.asarray(packet.origin)[None])
        for key in set(map(tuple, keys[weights > 0.])):
            expert = self._get(key)
            if expert is not None:
                self._observe_local_geometry(key, expert, packet)

    def update_counts(self, point, counts, integration_s, counts_per_usv=50000.):
        self.update_counts_window([point], [integration_s], counts, counts_per_usv)

    def update_counts_window(self, points, durations, counts, counts_per_usv=50000.):
        started = time.perf_counter()
        points, durations = np.asarray(points, dtype=float), np.asarray(durations, dtype=float)
        if (points.shape != (len(durations), 3) or not 1 <= len(durations) <= 8
                or not np.isfinite(points).all() or not np.isfinite(durations).all() or np.any(durations <= 0)
                or not math.isfinite(counts) or counts < 0 or counts != int(counts)
                or not math.isfinite(counts_per_usv) or counts_per_usv <= 0):
            raise ValueError('invalid windowed Poisson observation')
        if not self.prior_calibrated:
            self.calibration_counts += int(counts)
            self.calibration_exposure += float(durations.sum())*counts_per_usv/3600.
            self.calibration_windows += 1
            if self.calibration_windows >= self.prior_calibration_windows:
                self._calibrate_prior()
        keys, weights = self.assignments(points)
        active = sorted(set(map(tuple, keys[weights > 0.])))
        for key in active:
            expert = self._get(key, create=True)
            if self.last_geometry_packet is not None:
                self._observe_local_geometry(key, expert, self.last_geometry_packet)
            expert.update_counts_window(points, durations, counts, counts_per_usv)
            self.dirty.add(key)
            self._remember(key, expert)
        self.last_window_experts = len(active)
        self.updates += 1
        self.update_ms = (self.update_ms+[(time.perf_counter()-started)*1000.])[-4096:]

    def update(self, *args, **kwargs):
        raise ValueError('WindowedGP requires Poisson detector counts')

    def _calibrate_prior(self):
        # Empirical Bayes bootstrap from the initial count windows. The prior's
        # expected rate, not its lognormal median, matches the observed intensity.
        # Variance is retained: an unmeasured region is not declared safe.
        rate = max(.05, self.calibration_counts/max(self.calibration_exposure, 1e-12))
        self.prior = math.log(rate)-.5*self.variance
        self.prototype = self._new_expert()
        # Calibration happens before any expert can lose exact history:
        # calibration_windows <= replay_capacity, and this window is not yet
        # assimilated. Rebuild only this small bootstrap set, then fix the prior.
        for key in list(self.catalog):
            old = self._get(key)
            if old.updates != len(old.records):
                raise RuntimeError('Cannot recalibrate GP from incomplete local history')
            fresh = self._new_expert()
            fresh.planes = [plane.copy() for plane in old.planes]
            fresh.last_geometry_stamp = old.last_geometry_stamp
            fresh.geometry_rebuilds = old.geometry_rebuilds
            for positions, durations, counts, sensitivity in old.records:
                fresh.update_counts_window(positions, durations, counts, sensitivity)
            self.experts[key] = fresh
            self.dirty.add(key)
            self._remember(key, fresh)
        self.prior_calibrated = True

    def metrics(self):
        return dict(model='paged_windowed_geometry_gp', likelihood='poisson',
            experts=len(self.catalog), resident_experts=len(self.experts), max_resident_experts=self.max_experts,
            capacity_per_expert=self.capacity, inducing_points=sum(v['inducing_points'] for v in self.catalog.values()),
            resident_inducing_points=sum(len(e.Z) for e in self.experts.values()),
            capacity=self.capacity*len(self.catalog), resident_inducing_capacity=self.capacity*self.max_experts,
            total_window_limit=None, updates=self.updates,
            expert_updates=sum(v['updates'] for v in self.catalog.values()),
            last_window_experts=self.last_window_experts, local_replay_capacity=self.replay_capacity,
            frozen_geometry_windows=sum(v['geometry_frozen'] for v in self.catalog.values()),
            posterior_bytes=sum(e.Z.nbytes+e.L.nbytes+e.mean.nbytes+e.cov.nbytes for e in self.experts.values()),
            resident_history_bytes=sum(p.nbytes+d.nbytes for e in self.experts.values() for p,d,_,_ in e.records),
            prediction_cache_bytes=sum(e.prediction_cache_bytes for e in self.experts.values()),
            cell_size_m=self.cell_size.tolist(), halo_fraction=self.halo,
            page_reads=self.page_reads, page_writes=self.page_writes, evictions=self.evictions,
            prediction_page_reads=self.prediction_page_reads,
            saturated=False, unallocated_updates=0,
            update_p50_ms=float(np.median(self.update_ms)) if self.update_ms else None,
            update_p95_ms=float(np.quantile(self.update_ms,.95)) if self.update_ms else None,
            history_policy='bounded local exact history; fixed local geometry kernel after it fills; posterior updates continue',
            prior_calibrated=self.prior_calibrated, prior_expected_rate_usv_h=math.exp(self.prior+.5*self.variance),
            prior_calibration_windows=self.calibration_windows,
            storage_policy='LRU resident windows; inactive posteriors retained on disk; no global expert-count cutoff',
            kernel='local Matern32-ARD x observed-plane-side RBF', coordinates='xyz metres',blending=self.blend,upper_rule=self.upper_rule,
            uncertainty='correlated expert mixture moments and conservative upper envelope')

    def save(self, path):
        for key, expert in list(self.experts.items()):
            if key in self.dirty:
                self._store(key, expert)
        metadata = dict(capacity=self.capacity, max_experts=self.max_experts, cell_size=self.cell_size.tolist(),
            halo=self.halo, replay_capacity=self.replay_capacity, max_planes=self.max_planes,
            prior=self.prior, variance=self.variance, lengthscales=self.lengthscales.tolist(), updates=self.updates,blend=self.blend,upper_rule=self.upper_rule,
            prior_calibrated=self.prior_calibrated, calibration_counts=self.calibration_counts,
            calibration_exposure=self.calibration_exposure, calibration_windows=self.calibration_windows,
            windows=[dict(key=list(key), **value) for key, value in sorted(self.catalog.items())])
        path = Path(path)
        temporary = path.with_suffix(path.suffix+'.tmp')
        with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_STORED) as output:
            for name, value in (('model_kind', 'windowed'), ('schema_version', 3), ('likelihood', 'poisson'),
                                ('metadata', json.dumps(metadata, allow_nan=False))):
                with output.open(name+'.npy', 'w') as stream:
                    np.lib.format.write_array(stream, np.asarray(value), allow_pickle=False)
            # Each compressed local posterior is copied individually, never
            # constructing a global matrix or loading the full map into RAM.
            for key in sorted(self.catalog):
                cached = self.store_dir/self.name(key)
                if cached.exists():
                    payload = cached.read_bytes()
                else:
                    with zipfile.ZipFile(self.archive_path) as source:
                        payload = source.read('experts/'+self.name(key))
                output.writestr('experts/'+self.name(key), payload)
        temporary.replace(path)
        self.archive_path = str(path.resolve())

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(str(archive['metadata'].item()))
        gp = cls(**{k:metadata[k] for k in ('capacity', 'max_experts', 'cell_size', 'halo', 'replay_capacity', 'max_planes')},
            blend=metadata.get('blend','linear'),upper_rule=metadata.get('upper_rule','expanded_prior'))
        gp.prior, gp.variance = metadata['prior'], metadata['variance']
        gp.lengthscales, gp.updates = np.asarray(metadata['lengthscales']), metadata['updates']
        gp.prototype = gp._new_expert()
        gp.prior_calibrated = metadata.get('prior_calibrated', True)
        gp.calibration_counts, gp.calibration_exposure = metadata.get('calibration_counts', 0), metadata.get('calibration_exposure', 0.)
        gp.calibration_windows = metadata.get('calibration_windows', gp.prior_calibration_windows)
        gp.catalog = {tuple(item['key']):{k:v for k,v in item.items() if k != 'key'} for item in metadata['windows']}
        gp.archive_path = str(Path(path).resolve())
        return gp
