import numpy as np

from dougie_sim.region_agenda import RegionAgenda


def test_disconnected_and_stacked_support_never_share_region():
    nodes = {k: np.array(p) for k, p in enumerate(((.2, .2, 0.), (.4, .2, 0.), (.6, .2, 0.), (.2, .2, 2.4)))}
    agenda = RegionAgenda()
    agenda.update(nodes, {0: [1], 1: [], 2: [], 3: []}, 0, 0.)
    assert agenda.membership[0] == agenda.membership[1]
    assert agenda.membership[0] != agenda.membership[2]
    assert agenda.membership[0] != agenda.membership[3]


def test_distant_productive_region_can_beat_local_work_and_repeats_are_counted():
    nodes = {k: np.array([float(k), .2, 0.]) for k in range(8)}
    graph = {k: [j for j in (k-1, k+1) if j in nodes] for k in nodes}
    agenda = RegionAgenda()
    for root, stamp in ((1, 0.), (4, 10.), (1, 20.), (4, 30.), (1, 40.)):
        agenda.update(nodes, graph, root, stamp)
    selected = agenda.choose([(1., 2, 1), (4., 7, 8)], {2: 10., 7: 80.}, 40.)
    assert selected[1] == 7
    assert agenda.metrics()['repeated_connector_crossings'] == 2
    assert agenda.metrics()['pending_regions'] == 2


def test_visited_connector_region_yields_to_fresh_landing():
    nodes = {k: np.array([float(k), .2, .7]) for k in range(3, 9)}
    graph = {k: [j for j in (k-1, k+1) if j in nodes] for k in nodes}
    agenda = RegionAgenda()
    agenda.update(nodes, graph, 5, 65., visited=(3, 4, 5))
    # The local ramp's repeated visibility proxy is larger, but it has already
    # been traversed; the connected fresh landing should now be explored.
    assert agenda.choose([(5., 4, 7), (3., 7, 10)], {4: 5., 7: 30.}, 65.)[1] == 7
