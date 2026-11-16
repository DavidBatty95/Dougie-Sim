from types import SimpleNamespace

import numpy as np

from dougie_sim.region_clustering import ObservedGain


def scene(height, right_height=None, unknown_right=False):
    resolution = .2
    points = []
    for x in range(-16,17):
        for y in range(-16,17):
            if unknown_right and x >= 0:
                continue
            px, py = (x+.5)*resolution, (y+.5)*resolution
            z = height(px,py) if x < 0 or right_height is None else right_height
            points.append(np.array([px,py,z]))
    key = (-1,0,0)
    node = np.array([-.1,.1,height(-.1,.1)])
    return SimpleNamespace(resolution=resolution,max_slope=np.tan(np.deg2rad(10.)),
        nodes={key:node}, hits={}, surfaces={},
        support_returns={i:point for i,point in enumerate(points)}),key


def test_observed_continuous_ramp_is_not_a_horizontal_height_frontier():
    terrain,key = scene(lambda x,y:.1*x)
    # The authoritative fine returns are complete; the inherited flat cache is
    # intentionally empty, as can happen after a cache/capacity mismatch.
    gain = ObservedGain(terrain)
    assert gain.value(key) == 0
    np.testing.assert_array_equal(gain.values([key]),[0])


def test_known_drop_is_a_boundary_but_unobserved_support_remains_frontier():
    drop,key = scene(lambda x,y:2.4,right_height=0.)
    drop.occupancy = SimpleNamespace(resolution=.2,coarse_free={},
        evidence={(x,y,z):-3 for x in range(17) for y in range(-16,17) for z in (11,12)})
    assert ObservedGain(drop).value(key) == 0
    unknown,key = scene(lambda x,y:2.4,unknown_right=True)
    assert ObservedGain(unknown).value(key) > 0


def test_previously_observed_lower_floor_does_not_erase_upper_frontiers():
    terrain,key = scene(lambda x,y:2.4,right_height=0.)
    assert ObservedGain(terrain).value(key) > 0
    # Free space above the slab is not evidence that the slab is absent.
    terrain.occupancy = SimpleNamespace(resolution=.2,coarse_free={},
        evidence={(x,y,12):-8 for x in range(17) for y in range(-16,17)})
    assert ObservedGain(terrain).value(key) > 0


def test_drop_evidence_survives_compaction_and_sparse_fallback():
    terrain,key = scene(lambda x,y:2.4,right_height=0.)
    terrain.occupancy = SimpleNamespace(resolution=.2,evidence={},
        coarse_free={(x,y,z):-3 for x in range(9) for y in range(-8,9) for z in (5,6)})
    assert ObservedGain(terrain).value(key) == 0
    terrain.nodes[(1000000,1000000,0)] = np.array([200000.1,200000.1,0.])
    assert ObservedGain(terrain).value(key) == 0


def test_ray_cleared_edge_needs_no_observed_lower_floor():
    terrain,key = scene(lambda x,y:2.4,unknown_right=True)
    terrain.occupancy = SimpleNamespace(resolution=.2,coarse_free={},
        evidence={(x,y,z):-3 for x in range(17) for y in range(-16,17) for z in (11,12)})
    assert ObservedGain(terrain).value(key) == 0


def test_air_below_a_possible_thin_ramp_does_not_resolve_support():
    terrain,key = scene(lambda x,y:2.4,unknown_right=True)
    terrain.occupancy = SimpleNamespace(resolution=.2,coarse_free={},
        evidence={(x,y,11):-8 for x in range(17) for y in range(-16,17)})
    assert ObservedGain(terrain).value(key) > 0
    terrain.nodes[(1000000,1000000,0)] = np.array([200000.1,200000.1,0.])
    assert ObservedGain(terrain).value(key) > 0


def test_an_observed_upper_floor_does_not_claim_unknown_ground_support():
    terrain,key = scene(lambda x,y:0.,right_height=2.4)
    assert ObservedGain(terrain).value(key) > 0


def test_sparse_and_dense_height_tracing_agree():
    terrain,key = scene(lambda x,y:.1*x,unknown_right=True)
    dense = ObservedGain(terrain).value(key)
    terrain.nodes[(1000000,1000000,0)] = np.array([200000.1,200000.1,0.])
    sparse = ObservedGain(terrain)
    assert not sparse.use_grid
    assert sparse.value(key) == dense
