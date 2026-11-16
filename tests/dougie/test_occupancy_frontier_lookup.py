from types import SimpleNamespace

import numpy as np

from dougie_sim.region_clustering import ObservedGain


def terrain():
    rng=np.random.default_rng(402)
    fine={tuple(map(int,p)):int(v) for p,v in zip(rng.integers(-5,6,(500,3)),rng.integers(-8,9,500))}
    coarse={tuple(map(int,p)):-4 for p in rng.integers(-3,4,(100,3))}
    nodes={(-3,-3,0):np.array([-.5,-.5,0.]),(3,3,120):np.array([.7,.7,2.4])}
    return SimpleNamespace(resolution=.2,max_slope=.18,nodes=nodes,hits={},surfaces={},support_returns={},
        occupancy=SimpleNamespace(resolution=.2,evidence=fine,coarse_free=coarse))


def test_dense_and_sparse_free_queries_match_exact_compacted_occupancy_in_3d():
    t=terrain()
    dense=ObservedGain(t)
    assert dense.free_grid is not None
    rng=np.random.default_rng(21)
    cells=rng.integers(-3,4,(300,2))
    height=rng.integers(-3,4,300)*.2+.01
    voxels=np.column_stack((cells,np.floor(height/.2).astype(int)))
    expected=np.asarray([t.occupancy.evidence.get(tuple(v),0)<=-3 or tuple(v//2) in t.occupancy.coarse_free for v in voxels])
    np.testing.assert_array_equal(dense.cleared_at(cells,height),expected)
    t.nodes[(1000000,1000000,0)]=np.array([200000.1,200000.1,0.])
    sparse=ObservedGain(t)
    assert sparse.free_grid is None
    np.testing.assert_array_equal(sparse.cleared_at(cells,height),expected)


def test_unobserved_higher_floor_is_not_cleared_by_lower_free_block():
    t=terrain()
    t.occupancy.evidence={}
    t.occupancy.coarse_free={(0,0,0):-4}
    gain=ObservedGain(t)
    assert gain.cleared_at(np.array([[0,0]]),np.array([.01]))[0]
    assert not gain.cleared_at(np.array([[0,0]]),np.array([2.41]))[0]


def test_sparse_lookup_is_a_snapshot_when_occupancy_changes():
    t=terrain()
    t.nodes[(1000000,1000000,0)]=np.array([200000.1,200000.1,0.])
    t.occupancy.evidence={(0,0,0):-4}
    t.occupancy.coarse_free={(1,0,0):-4}
    gain=ObservedGain(t)
    t.occupancy.evidence[(0,0,0)]=8
    t.occupancy.coarse_free.clear()
    np.testing.assert_array_equal(gain.cleared_at(np.array([[0,0],[2,0]]),np.array([.01,.01])),[True,True])
