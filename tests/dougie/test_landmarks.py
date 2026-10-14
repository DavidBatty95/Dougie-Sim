import xml.etree.ElementTree as ET
import time

import mujoco
import numpy as np

from dougie_sim.landmarks import LandmarkObservation, LandmarkSensor, PREFIX
from dougie_sim.simulation import Simulation, scene_xml


def fixture(point=(1., 0., .55), wall=False, slab=False):
    root = ET.fromstring(scene_xml('room'))
    body = root.find('worldbody')
    ET.SubElement(body, 'site', name=PREFIX+'000', pos=' '.join(map(str, point)), size='.08', group='1')
    if wall:
        ET.SubElement(body, 'geom', name='occluder', type='box', pos='.5 0 .6', size='.05 .8 .6')
    if slab:
        ET.SubElement(body, 'geom', name='slab', type='box', pos='1.5 0 1.5', size='3 2 .1')
    return Simulation(model_xml=ET.tostring(root, encoding='unicode'), controller='stand')


def test_semantic_detection_is_occluded_by_walls_and_floors():
    visible = fixture()
    sensor = LandmarkSensor(visible)
    packet = sensor.scan(visible)
    assert len(packet) == 1 and packet[0].inspected
    for sim in (fixture(wall=True), fixture((3., 0., 2.9), slab=True)):
        hidden = LandmarkSensor(sim)
        assert hidden.scan(sim) == ()
        assert hidden.packets() == []


def test_sensor_range_fov_and_latched_inspection():
    for point in ((5., 0., .55), (-1., 0., .55), (0., 2., .55)):
        sim = fixture(point)
        assert LandmarkSensor(sim).scan(sim) == ()
    sim = fixture((3., 0., .55))
    sensor = LandmarkSensor(sim)
    assert not sensor.scan(sim)[0].inspected
    sim.data.qpos[0] = 2.
    mujoco.mj_forward(sim.model, sim.data)
    assert sensor.scan(sim)[0].inspected
    sim.data.qpos[0] = 0.
    mujoco.mj_forward(sim.model, sim.data)
    assert sensor.scan(sim)[0].inspected
    assert sensor.evaluation(sim)['discovered'] == sensor.evaluation(sim)['inspected'] == 1


def test_seeded_targets_exist_in_mujoco_on_both_floors_and_start_hidden():
    def positions(seed):
        root = ET.fromstring(scene_xml('facility', landmark_count=8, landmark_seed=seed))
        return [site.get('pos') for site in root.findall('worldbody/site') if site.get('name').startswith(PREFIX)]
    assert positions(7) == positions(7)
    assert positions(7) != positions(8)
    sim = Simulation('facility', controller='stand', landmark_count=8, landmark_seed=7)
    sensor = LandmarkSensor(sim)
    result = sensor.evaluation(sim)
    assert result['total'] == 8 and result['discovered'] == 0
    assert result['per_floor']['lower']['total'] == result['per_floor']['upper']['total'] == 4
    assert sensor.packets() == []  # The catalogue has not become a sensor observation.


def test_only_observed_uninspected_targets_have_planner_utility():
    from dougie_sim.clustered_planner import ClusteredDoseExplorer
    planner = ClusteredDoseExplorer()
    point = np.array([1., 0., 0.])
    assert planner.landmark_bonus(point) == 0.
    planner.observe_landmarks([LandmarkObservation('visible', (1., 0., .55), 1., False)])
    assert planner.landmark_bonus(point) > 0.
    assert planner.landmark_bonus(point+[0., 0., 2.4]) == 0.
    planner.observe_landmarks([LandmarkObservation('visible', (1., 0., .55), 2., True)])
    planner.observe_landmarks([LandmarkObservation('visible', (1., 0., .55), 1., False)])
    assert planner.landmark_bonus(point) == 0.


def test_discovered_targets_reach_late_rviz_subscribers():
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from visualization_msgs.msg import MarkerArray
    from dougie_sim.ros_bridge import Bridge
    from dougie_sim.rviz_visualization import Visualization
    sim = fixture()
    sensor = LandmarkSensor(sim)
    bridge = Bridge(sim, publish_scan=False)
    observer = None
    try:
        visualization = Visualization(bridge)
        sensor.scan(sim)
        visualization.update_landmarks(sensor.packets())
        observer = rclpy.create_node('landmark_late_observer')
        received = []
        subscription = observer.create_subscription(MarkerArray, '/radiation/landmarks', received.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic()+5.
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=.1)
        assert received
        markers = received[-1].markers
        assert len(markers) == 2 and any('inspected' in m.text for m in markers)
        cube = next(m for m in markers if m.type == m.CUBE)
        np.testing.assert_allclose([cube.pose.position.x, cube.pose.position.y, cube.pose.position.z], [1., 0., .55])
    finally:
        if observer is not None:
            observer.destroy_node()
        bridge.close()
