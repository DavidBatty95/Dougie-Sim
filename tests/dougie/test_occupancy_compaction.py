import itertools

from dougie_sim.layered_terrain import VoxelEvidence


def test_only_complete_confirmed_free_blocks_are_compacted():
    occupancy = VoxelEvidence(capacity=40)
    complete = list(itertools.product((-2, -1), repeat=3))
    for key in complete:
        occupancy.evidence[key] = -4
    partial = list(itertools.product((0, 1), repeat=3))[:-1]
    occupancy.evidence.update({key: -4 for key in partial})
    occupancy.evidence[(3, 0, 0)] = 8
    assert occupancy.compact_free() == 1
    assert occupancy.coarse_free == {(-1, -1, -1): -4}
    assert all(key in occupancy.evidence for key in partial)
    assert occupancy.evidence[(3, 0, 0)] == 8
    assert occupancy.storage_cells == 9
    occupancy.add_evidence((-2, -2, -2), 3)
    assert not occupancy.coarse_free
    assert occupancy.evidence[(-2, -2, -2)] == -1
    assert all(key in occupancy.evidence for key in complete)


def test_hit_at_capacity_discards_coarse_free_claim_before_recording_hit():
    occupancy = VoxelEvidence(capacity=2)
    occupancy.coarse_free[(0, 0, 0)] = -4
    occupancy.evidence[(10, 0, 0)] = 8
    occupancy.add_evidence((0, 0, 0), 3)
    assert not occupancy.coarse_free
    assert occupancy.evidence[(0, 0, 0)] == 3
    assert occupancy.evidence[(10, 0, 0)] == 8
    assert occupancy.storage_cells == 2
