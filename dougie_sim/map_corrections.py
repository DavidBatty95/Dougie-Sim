"""Versioned, time-distributed SE(3) corrections for onboard observation history.

Loop registration supplies the endpoint transform. Distributing it between an
unchanged anchor and the current scan is an approximation, not pose-graph SLAM.
Every consumer uses the same transform and revision; counts/durations never change.
"""
from dataclasses import dataclass, replace

import numpy as np
from scipy.spatial.transform import Rotation

from .sensing import RangePacket, support_mask


@dataclass(frozen=True)
class MapCorrection:
    revision: int
    start_s: float
    end_s: float
    rotation_vector: tuple
    translation: tuple

    def __post_init__(self):
        if (self.revision < 1 or not np.isfinite([self.start_s, self.end_s, *self.rotation_vector, *self.translation]).all()
                or self.end_s <= self.start_s or len(self.rotation_vector) != 3 or len(self.translation) != 3):
            raise ValueError('invalid map correction')

    def transform(self, stamp):
        fraction = float(np.clip((stamp-self.start_s)/(self.end_s-self.start_s), 0., 1.))
        return Rotation.from_rotvec(np.asarray(self.rotation_vector)*fraction).as_matrix(), np.asarray(self.translation)*fraction

    def point(self, point, stamp):
        R, t = self.transform(stamp)
        return R@np.asarray(point)+t

    def packet(self, packet):
        if packet.map_revision >= self.revision:
            return packet
        R, t = self.transform(packet.stamp_s)
        origin, directions = R@packet.origin+t, packet.directions@R.T
        return RangePacket(packet.stamp_s, packet.sequence, origin, directions, packet.distances,
            packet.max_range_m, support_mask(origin, directions, packet.distances, packet.max_range_m), self.revision)

    def sample(self, sample):
        if sample.map_revision >= self.revision:
            return sample
        if sample.window_positions:
            durations = np.asarray(sample.window_durations_s)
            stamps = sample.stamp_s-sample.integration_s+np.cumsum(durations)-durations/2.
            points = np.array([self.point(p, t) for p, t in zip(sample.window_positions, stamps)])
            return replace(sample, window_positions=tuple(map(tuple, points)),
                           position=tuple(np.average(points, axis=0, weights=durations)), map_revision=self.revision)
        return replace(sample, position=tuple(self.point(sample.position, sample.stamp_s-sample.integration_s/2.)),
                       map_revision=self.revision)


class ObservationJournal:
    """Bounded worker-side replay history; overflow is explicit, never silent eviction."""
    def __init__(self, scan_capacity=1024, sample_capacity=8192):
        self.scans, self.samples, self.visits = [], [], []
        self.scan_capacity, self.sample_capacity = scan_capacity, sample_capacity
        self.revision = self.rebuilds = 0
        self.saturated = False
        self.corrections = []

    def align_packet(self, packet):
        for event in self.corrections:
            packet = event.packet(packet)
        return packet

    def align_sample(self, sample):
        for event in self.corrections:
            sample = event.sample(sample)
        return sample

    def correct(self, events):
        new = [event for event in events if event.revision > self.revision]
        if not new:
            return False
        if self.saturated:
            raise RuntimeError('cannot correct an incomplete observation journal')
        if [event.revision for event in new] != list(range(self.revision+1, self.revision+1+len(new))):
            raise ValueError('map correction revisions must be contiguous')
        for event in new:
            if event.revision != self.revision+1:
                raise ValueError('map correction revisions must be contiguous')
            self.scans = [event.packet(packet) for packet in self.scans]
            self.samples = [event.sample(sample) for sample in self.samples]
            self.visits = [(stamp, event.point(point, stamp)) for stamp, point in self.visits]
            self.corrections.append(event)
            self.revision = event.revision
        self.rebuilds += 1
        return True

    def append(self, batch, packet, position, stamp):
        if (len(self.scans) >= self.scan_capacity or len(self.samples)+len(batch) > self.sample_capacity):
            self.saturated = True
            return False
        if not self.scans or packet.stamp_s > self.scans[-1].stamp_s:
            self.scans.append(packet)
            self.visits.append((stamp, np.asarray(position).copy()))
        for sample in batch:
            if self.samples and sample.stamp_s <= self.samples[-1].stamp_s:
                raise ValueError('count observations must be assimilated once in timestamp order')
            self.samples.append(sample)
        return True

    def metrics(self):
        return dict(map_revision=self.revision, map_rebuilds=self.rebuilds, retained_scans=len(self.scans),
                    retained_count_windows=len(self.samples), observation_journal_saturated=self.saturated)
