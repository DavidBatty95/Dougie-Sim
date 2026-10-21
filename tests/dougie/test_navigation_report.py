from scripts.report_dougie_navigation import navigation_transitions


def test_navigation_report_retains_recovery_attempts_and_map_revisions():
    def row(stamp, phase='resensing', attempts=0, revision=0):
        return dict(time_s=stamp, state='recovery_resensing',
            decision_reason='return_route_disconnected',
            recovery=dict(phase=phase, reason='return_route_disconnected', retreat_attempts=attempts),
            map_consistency=dict(map_revision=revision), planned_path=[],
            localization=dict(position=[1., 2., .5]))

    events = navigation_transitions([row(10), row(12), row(14, 'checked_retreat', 1),
                                    row(16, 'checked_retreat', 2), row(18, 'checked_retreat', 2, 1)])
    assert [event['time_s'] for event in events] == [10, 14, 16, 18]
    assert events[0]['estimated_position'] == [1., 2., .5]
    assert events[-1]['map_revision'] == 1
    assert events[-1]['planned_path_points'] == 0


def test_navigation_report_supports_legacy_trace_without_localization():
    events = navigation_transitions([dict(time_s=1., state='exploring'),
                                    dict(time_s=2., state='returned')])
    assert [event['state'] for event in events] == ['exploring', 'returned']
    assert events[0]['estimated_position'] is None
    assert events[0]['return_path_points'] == 0
    assert navigation_transitions([]) == []


def test_support_diagnosis_distinguishes_disconnected_overlapping_levels():
    from types import SimpleNamespace
    import numpy as np
    from scripts.diagnose_dougie_support import connectivity

    terrain = SimpleNamespace(nodes={0: np.array([0., 0., 0.]), 1: np.array([1., 0., 0.]),
                                     2: np.array([0., 0., 2.4])},
                              graph={0: [1], 1: [0], 2: []})
    components = connectivity(terrain)
    assert [component['nodes'] for component in components] == [2, 1]
    assert components[0]['maximum_xyz'] == [1., 0., 0.]
    assert components[1]['minimum_xyz'] == [0., 0., 2.4]
    assert connectivity(SimpleNamespace(nodes={}, graph={})) == []
