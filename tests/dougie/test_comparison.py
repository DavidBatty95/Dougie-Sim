from functools import partial
from http.server import ThreadingHTTPServer
import json
import threading
from types import SimpleNamespace
from urllib.request import urlopen

import mujoco
import numpy as np

from dougie_sim.comparison import DashboardHandler, mission_command, read_rows
from dougie_sim.comparison_stream import FloorCoverage
from dougie_sim.comparison_stream import planning_snapshot, ComparisonStream
from dougie_sim.planning_worker import create_explorer
from dougie_sim.radiation import RadiationSample
from dougie_sim.radiation_blind import blind_sample


def test_blind_ablation_ignores_exposure_but_aware_retreats():
    hot = RadiationSample(1., (0., 0., .4), 100000, .5, 1000., 10., 5., 6.)
    sanitized = blind_sample(hot)
    assert hot.rate_usv_h == 1000. and sanitized.stamp_s == hot.stamp_s
    parameters = (2., 100., 32, 600., 'poisson')
    blind = create_explorer(parameters, 'clustered', 'conservative', {'radiation_awareness': 'blind'})
    aware = create_explorer(parameters, 'clustered', 'conservative')
    blind.observe_radiation(sanitized)
    aware.observe_radiation(hot)
    assert not blind.returning and aware.returning
    for prediction in blind.gp.predict([[0, 0, .4], [20, 10, 2]]):
        np.testing.assert_array_equal(prediction, [0., 0.])
    # No radiation information gain in previously unobserved space either.
    blind.gp.update_counts([1, 2, 3], 10**9, .5)
    assert all(np.all(value == 0.) for value in blind.gp.predict([[1, 2, 3]]))


def test_budget_free_aware_robot_still_learns_radiation_and_avoids_high_rates():
    parameters = (2., 100., 32, 1e12, 'poisson')
    aware = create_explorer(parameters, 'clustered', 'conservative', {'no_dose_budget': True})
    sample = RadiationSample(1., (0., 0., .44), 50, .5, 7.2, 1., 500., 600.)
    aware.observe_radiation(sample)
    assert not aware.returning and aware.metrics()['budget_usv'] is None
    assert aware.gp.predict([[0., 0., .44]])[0][0] > 0.
    from dataclasses import replace
    aware.observe_radiation(replace(sample, stamp_s=2., rate_usv_h=150.))
    assert aware.returning and aware.last_reason == 'measured_rate_limit'
    minimizing = create_explorer(parameters, 'clustered', 'conservative', {
        'no_dose_budget': True, 'radiation_policy': 'minimize-dose'})
    minimizing.observe_radiation(replace(sample, rate_usv_h=150.))
    assert not minimizing.returning
    assert minimizing.gp.predict([[0., 0., .44]])[0][0] > 0.
    assert minimizing.metrics()['radiation_policy'] == 'minimize-dose'


def test_fixed_floor_denominator_obstacles_levels_and_monotonic_coverage():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom name="floor" type="plane" size="0 0 .1"/>
      <geom name="upper_floor_test" type="box" pos="0 0 2.3" size="1 1 .1"/>
      <geom name="equipment" type="box" pos="3 3 .2" size="1 1 .2"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    coverage = FloorCoverage(SimpleNamespace(model=model, data=data), 'facility')
    points = coverage.points
    assert np.any(points[:, 2] > 2.)
    assert not np.any((abs(points[:, 0]-3) < 1) & (abs(points[:, 1]-3) < 1) & (points[:, 2] == 0))
    assert coverage.percent == 0.
    coverage.observe(points[:10], support_position=[0., 0., 0.])
    first = coverage.percent
    coverage.observe(points[:10], support_position=[0., 0., 0.])
    assert coverage.percent == first
    coverage.observe(points, support_position=[0., 0., 0.])
    assert not coverage.seen[points[:, 2] > 2.].any()
    assert coverage.percent < 100.
    coverage.observe(points, support_position=[0., 0., 2.4])
    assert coverage.percent == 100.
    coverage.observe([[1000., 1000., 0.]], support_position=[0., 0., 2.4])
    assert coverage.percent == 100.


def test_pair_commands_match_except_ablation_and_output(tmp_path):
    args = SimpleNamespace(world='reactor', duration=60., seed=3, sensor_seed=4,
                           layout_seed=5, dose_budget=2., rate_limit=100., feed_fps=30)
    blind = mission_command(args, 'blind', tmp_path)
    aware = mission_command(args, 'aware', tmp_path)
    differences = [(a, b) for a, b in zip(blind, aware) if a != b]
    assert differences == [('blind', 'aware'), (str(tmp_path/'blind'), str(tmp_path/'aware'))]
    assert '--ros' not in blind and '--comparison-stream' in blind
    assert blind[blind.index('--feed-fps')+1] == '30'


def test_open_ended_pair_has_no_dose_or_time_cap_and_matches_pose_source(tmp_path):
    args = SimpleNamespace(world='reactor', duration=0., seed=1, sensor_seed=0,
                           layout_seed=0, dose_budget=None, rate_limit=100., feed_fps=5,
                           localization='ground-truth', radiation_policy='minimize-dose')
    for mode in ('aware', 'blind'):
        command = mission_command(args, mode, tmp_path)
        assert '--no-dose-budget' in command and '--dose-budget' not in command
        assert command[command.index('--duration')+1] == '0.0'
        assert command[command.index('--localization')+1] == 'ground-truth'
        assert command[command.index('--radiation-policy')+1] == 'minimize-dose'


def test_planning_annotations_are_aligned_without_changing_planner_data():
    metrics = dict(planned_path=[[1., 0., 0.], [2., 0., 0.]], return_path=[],
                   selected_target=[2., 0., 0.], clusters=[dict(anchor=[3., 0., 0.])])
    original = json.dumps(metrics)
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    overlay = planning_snapshot(metrics, rotation, np.array([10., 20., 1.]))
    np.testing.assert_allclose(overlay['planned_path'], [[10., 21., 1.], [10., 22., 1.]])
    assert overlay['selected_target'] == [10., 22., 1.]
    assert overlay['clusters'][0]['anchor'] == [10., 23., 1.]
    assert json.dumps(metrics) == original


def test_distance_counts_intermediate_physics_ticks_and_vertical_motion():
    stream = ComparisonStream.__new__(ComparisonStream)
    stream.distance_m, stream.last_position = 0., np.zeros(3)
    stream.support_height = lambda sim: 0.
    stream.minimap = SimpleNamespace(record=lambda *args: None)
    stream.next_metrics = stream.next_frame = float('inf')
    sim = SimpleNamespace(data=SimpleNamespace(qpos=np.array([0., 0., 1.]), time=1.))
    stream.capture(sim, 0., 0., 'exploring')
    sim.data.qpos = np.array([3., 4., 1.])
    stream.capture(sim, 0., 0., 'exploring')
    # Returning to the same position must not double-count or erase distance.
    stream.capture(sim, 0., 0., 'exploring')
    assert stream.distance_m == 6.


def test_reference_percentage_never_decides_mission_completion():
    import io
    stream = ComparisonStream.__new__(ComparisonStream)
    stream.distance_m, stream.last_position = 0., np.zeros(3)
    stream.support_height = lambda sim: 0.
    stream.minimap = SimpleNamespace(record=lambda *args: None)
    stream.next_metrics, stream.next_frame = 0., float('inf')
    stream.coverage = SimpleNamespace(seen=np.ones(2, dtype=bool), percent=100., points=np.zeros((2, 3)),
        metric='test_reference', level_heights=np.array([0.]), reached_levels=np.array([True]))
    stream.admission = np.zeros(2, dtype=np.uint8)
    stream.log = io.StringIO()
    sim = SimpleNamespace(data=SimpleNamespace(qpos=np.zeros(3), time=1.))
    stream.capture(sim, 0., 0., 'exploring', metrics=dict(frontier_count=4, frontier_complete=False))
    assert stream.survey_report['outcome'] == 'surveying'
    assert not stream.survey_report['frontier_complete']
    # Conversely, a planner can finish with a reference score below 100%.
    stream.coverage.percent = 40.
    sim.data.time = 2.
    stream.capture(sim, 0., 0., 'finished', metrics=dict(frontier_count=0, frontier_complete=True))
    assert stream.survey_report['outcome'] == 'frontiers_exhausted'
    assert stream.survey_report['explored_percent'] == 40.


def test_live_api_reads_complete_samples_and_serves_two_views(tmp_path):
    row = dict(time_s=1., true_dose_usv=.1, explored_percent=20., rate_usv_h=360., state='exploring')
    for mode in ('blind', 'aware'):
        directory = tmp_path/mode
        directory.mkdir()
        (directory/'comparison.jsonl').write_text(json.dumps(row)+'\n{"time_s":')
        (directory/'frame.jpg').write_bytes(b'frame-'+mode.encode())
    assert read_rows(tmp_path/'blind/comparison.jsonl') == [row]
    processes = {mode: SimpleNamespace(poll=lambda: None) for mode in ('blind', 'aware')}
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(DashboardHandler, output=tmp_path, processes=processes))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        base = f'http://127.0.0.1:{server.server_port}'
        with urlopen(base+'/api') as response:
            result = json.load(response)
        assert result['blind']['rows'] == result['aware']['rows'] == [row]
        assert result['blind']['exit_code'] is None
        with urlopen(base+'/') as response:
            assert b'coverage' in response.read()
        for mode in processes:
            with urlopen(base+'/'+mode+'/frame.jpg') as response:
                assert response.read() == b'frame-'+mode.encode()
            with urlopen(base+'/'+mode+'/stream.mjpg', timeout=3) as response:
                assert response.headers.get_content_type() == 'multipart/x-mixed-replace'

                def next_frame():
                    assert response.readline() == b'--frame\r\n'
                    assert response.readline() == b'Content-Type: image/jpeg\r\n'
                    size = int(response.readline().split(b':')[1])
                    assert response.readline() == b'\r\n'
                    payload = response.read(size)
                    assert response.read(2) == b'\r\n'
                    return payload

                assert next_frame() == b'frame-'+mode.encode()
                temporary = tmp_path/mode/'frame.tmp'
                temporary.write_bytes(b'new-frame')
                temporary.replace(tmp_path/mode/'frame.jpg')
                # The same HTTP connection delivers the next atomic frame.
                assert next_frame() == b'new-frame'
                processes[mode].poll = lambda: 0
                assert response.readline() == b'--frame--\r\n'
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
