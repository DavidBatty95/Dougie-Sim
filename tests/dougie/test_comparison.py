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
