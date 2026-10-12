"""Numerical contracts for the batched sensor hot paths."""
import math

import mujoco
import numpy as np
import pytest

from dougie_sim.radiation import RadiationField, segment_box_length
from dougie_sim.simulation import Simulation


def scalar_rate(field, point):
    return field.background+sum(
        source['strength_usv_h_m2']/(np.sum((point-np.asarray(source['position']))**2)+source['core_radius_m']**2)
        * math.exp(-sum(shield['attenuation_per_m']*segment_box_length(
            source['position'], point, shield['lower'], shield['upper']) for shield in field.shields))
        for source in field.sources)


@pytest.mark.parametrize('source_count,shield_count', [(0, 0), (0, 3), (1, 0), (1, 1), (8, 5)])
def test_field_matches_scalar_intersections(source_count, shield_count):
    rng = np.random.default_rng(43)
    origins = rng.uniform(-4, 4, (source_count, 3))
    lowers = rng.uniform(-3, 2, (shield_count, 3))
    field = RadiationField(dict(background_usv_h=.2, sources=[
        dict(position=p, strength_usv_h_m2=float(rng.uniform(1, 100)), core_radius_m=.2)
        for p in origins], shields=[dict(lower=p, upper=p+1, attenuation_per_m=2.) for p in lowers]))
    points = list(rng.uniform(-5, 5, (100, 3)))+list(origins)+list(lowers)+list(lowers+1)
    # Parallel, nearly parallel, and source-coincident ray/box intersections.
    for p in origins:
        for axis in range(3):
            q = p.copy()
            q[axis] += 3.
            points.extend((q, q+1e-13))
    for p in points:
        assert field.rate(p) == pytest.approx(scalar_rate(field, p), rel=2e-13, abs=1e-12)


@pytest.mark.parametrize('world', ['room', 'office', 'reactor'])
def test_batched_scan_matches_individual_rays(world):
    sim = Simulation(world, controller='stand')
    groups = np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8)
    geom = np.empty(1, dtype=np.int32)
    for yaw in (0., .73, -2.1):
        sim.data.qpos[3:7] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
        mujoco.mj_forward(sim.model, sim.data)
        origin = sim.data.qpos[:3]+[0, 0, .12]
        for count, cutoff in ((180, 8.), (37, 2.)):
            expected = []
            for angle in np.linspace(-math.pi, math.pi, count, endpoint=False)+yaw:
                direction = np.array([math.cos(angle), math.sin(angle), 0.])
                distance = mujoco.mj_ray(sim.model, sim.data, origin, direction, groups, 1, sim.base, geom)
                expected.append(distance if .05 <= distance <= cutoff else math.inf)
            np.testing.assert_allclose(sim.scan(count, cutoff), expected, rtol=1e-12, atol=1e-12)
