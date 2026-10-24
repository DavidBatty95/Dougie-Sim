"""Normals for non-uniform organized LiDAR, including sparse distant wall rows."""
import numpy as np


def registration_cloud(packet):
    points = packet.origin+packet.directions*packet.distances[..., None]
    left, right = points-np.roll(points, 1, axis=1), np.roll(points, -1, axis=1)-points
    down, up = points-np.roll(points, 1, axis=0), np.roll(points, -1, axis=0)-points
    across, radial = left+right, down+up
    normals = np.cross(across, radial)
    lengths = np.linalg.norm(normals, axis=2)
    valid = packet.valid & np.roll(packet.valid, 1, axis=0) & np.roll(packet.valid, -1, axis=0)
    valid &= np.roll(packet.valid, 1, axis=1) & np.roll(packet.valid, -1, axis=1)
    # A fixed .6 m vertical-neighbour cutoff discards real wall normals when
    # upper beam elevations are widely spaced. Scale only by measured range and
    # known beam angles; reject inconsistent one-sided planes at occlusion edges.
    angular_span = np.linalg.norm(np.roll(packet.directions, -1, axis=0)-np.roll(packet.directions, 1, axis=0), axis=2)
    range_scale = np.maximum.reduce((np.maximum(packet.distances, 0.),
        np.maximum(np.roll(packet.distances, -1, axis=0), 0.), np.maximum(np.roll(packet.distances, 1, axis=0), 0.)))
    radial_limit = np.maximum(.6, 1.5*range_scale*angular_span+.05)
    n1, n2 = np.cross(right, up), np.cross(left, down)
    product = np.linalg.norm(n1, axis=2)*np.linalg.norm(n2, axis=2)
    agreement = np.abs(np.sum(n1*n2, axis=2))/np.maximum(product, 1e-12)
    valid &= (lengths > 1e-5) & (product > 1e-10) & (agreement > .8)
    valid &= (np.linalg.norm(across, axis=2) < .6) & (np.linalg.norm(radial, axis=2) < radial_limit)
    valid[[0, -1]] = False
    cloud, normal = points[valid], normals[valid]/lengths[valid, None]
    # Preserve the observed side of a surface. Opposite faces of a thin wall
    # must not become compatible merely because their normals are antiparallel.
    normal[np.sum(normal*(cloud-packet.origin), axis=1) > 0.] *= -1.
    return cloud, normal


def sample_registration_cloud(points, normals, mode='spatial', capacity=1500):
    """Retain actual observations, optionally preserving rare normal directions."""
    if mode not in ('spatial', 'normal-balanced') or capacity < 1:
        raise ValueError('invalid registration sampling policy or capacity')
    if not len(points):
        return points.copy(), normals.copy()
    cells = np.floor(points/.12).astype(int)
    if mode == 'spatial':
        _, indices = np.unique(cells, axis=0, return_index=True)
        indices = indices[::max(1, int(np.ceil(len(indices)/capacity)))]
        return points[indices], normals[indices]
    directions = np.rint(normals/.25).astype(int)
    _, indices = np.unique(np.c_[cells, directions], axis=0, return_index=True)
    if len(indices) > capacity:
        _, groups, counts = np.unique(directions[indices], axis=0, return_inverse=True, return_counts=True)
        if len(counts) > capacity:
            raise ValueError('registration capacity cannot retain each observed normal group')
        quotas = np.minimum(counts, min(4, max(1, capacity//len(counts))))
        remaining = capacity-int(quotas.sum())
        spare = counts-quotas
        allocation = remaining*spare/spare.sum()
        extra = np.floor(allocation).astype(int)
        quotas += extra
        for group in np.argsort(-(allocation-extra), kind='stable')[:capacity-int(quotas.sum())]:
            quotas[group] += 1
        selected = []
        for group, quota in enumerate(quotas):
            members = indices[groups == group]
            selected.extend(members[np.linspace(0, len(members)-1, int(quota), dtype=int)])
        indices = np.sort(selected)
    return points[indices], normals[indices]
