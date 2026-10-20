"""Exercise real DDS radiation publications, not mocked message constructors."""
import json
import os
import signal
import subprocess
import sys
import time
import pytest

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage


@pytest.mark.parametrize('show_sources', [False, True])
def test_radiation_measurements_and_3d_posterior_reach_ros_subscribers(tmp_path, show_sources):
    domain = 100+os.getpid()%100
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node('radiation_contract_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    samples, clouds = [], []
    scenes, robots, maps, dashboards = [], [], [], []
    descriptions, transforms = [], []
    detections = []
    late_subscribed = False
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(String, '/radiation/measurement', lambda m:samples.append(json.loads(m.data)), 10),
        node.create_subscription(PointCloud2, '/radiation/gp_map', clouds.append, 10),
        node.create_subscription(MarkerArray, '/dougie/robot', robots.append, 10),
        node.create_subscription(MarkerArray, '/radiation/visualization', maps.append, retained),
        node.create_subscription(MarkerArray, '/radiation/dashboard', dashboards.append, retained),
        node.create_subscription(TFMessage, '/tf', transforms.append, 10),
        node.create_subscription(MarkerArray, '/radiation/detections', detections.append, retained),
    ]
    log = (tmp_path/'child.log').open('w')
    process = subprocess.Popen([sys.executable, '-m', 'dougie_sim.radiation_run', '--headless', '--ros',
        '--duration', '30', '--dose-budget', '.2', '--output', str(tmp_path/'run'),
        *(['--show-sources'] if show_sources else [])],
        env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic()+60.
        while process.poll() is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
            # RViz often joins after startup: the static scene must be retained.
            if samples and not late_subscribed:
                subscriptions.append(node.create_subscription(MarkerArray, '/dougie/scene', scenes.append, retained))
                subscriptions.append(node.create_subscription(String, '/robot_description', descriptions.append, retained))
                late_subscribed = True
        assert process.poll() == 0, (tmp_path/'child.log').read_text()
        assert len(samples) >= 3 and clouds
        sample = samples[-1]
        assert len(sample['position']) == 3 and sample['counts'] >= 0
        assert sample['dose_upper_usv'] >= sample['dose_usv'] >= 0.
        assert 'sources' not in sample
        cloud = clouds[-1]
        assert cloud.header.frame_id == 'odom' and cloud.width > 0
        assert {'x', 'y', 'z', 'dose_rate_usv_h', 'upper_rate_usv_h'} <= {f.name for f in cloud.fields}
        assert scenes and robots and maps
        assert all(m.header.frame_id == 'odom' for m in scenes[-1].markers+robots[-1].markers+maps[-1].markers)
        if show_sources:
            assert any(m.ns == 'source_labels' and 'TRUE SOURCE' in m.text for m in scenes[-1].markers)
            assert any(m.ns == 'source_arrows' and len(m.points) == 2 for m in scenes[-1].markers)
            assert any(m.ns == 'sources' and m.scale.x >= .3 for m in scenes[-1].markers)
            assert any(m.ns == 'source_labels' and 'vessel_hotspot' in m.text and m.pose.position.z > 3.
                       for m in scenes[-1].markers)
        else:
            assert all(m.ns != 'sources' for m in scenes[-1].markers)
        assert detections and any(m.points for batch in detections for m in batch.markers)
        # A short mission can have every detected cell inside the robot fade.
        # Its posterior topic above must remain populated; display batches may
        # legitimately delete all local haze rather than obscure the robot.
        gp_markers = [m for m in maps[-1].markers if m.ns == 'gp']
        assert {m.id for m in gp_markers} == set(range(8))
        assert all(len(m.points) == len(m.colors) for m in gp_markers)
        assert all(m.action == m.DELETE for m in gp_markers if not m.points)
        assert all(sum(len(m.points) for m in batch.markers if m.ns == 'gp') <= 1200 for batch in maps)
        assert all(m.scale.x == .48 and 0 < m.color.a <= .018001
                   for batch in maps for m in batch.markers if m.ns == 'gp')
        assert any(m.ns == 'return route' for m in maps[-1].markers)
        assert any(m.ns == 'exploration target' for m in maps[-1].markers)
        assert dashboards
        assert descriptions and 'dougie_unitree_go2' in descriptions[-1].data
        assert '.obj' in descriptions[-1].data
        frames = {tf.child_frame_id for batch in transforms for tf in batch.transforms}
        assert {'base_link', 'FR_hip', 'FL_calf', 'RR_foot', 'RL_foot'} <= frames
        assert any('Measured dose' in m.text and 'uSv/h' in m.text for m in dashboards[-1].markers)
        assert all(m.ns != 'gp' for batch in dashboards for m in batch.markers)
        assert len(robots[-1].markers) > 12  # Articulated physical geometry, not just a base cube.
        stamps = [m.markers[0].header.stamp.sec+m.markers[0].header.stamp.nanosec/1e9 for m in robots]
        assert all(b-a >= .049 for a, b in zip(stamps, stamps[1:]))
        summary = json.loads((tmp_path/'run/radiation_summary.json').read_text())
        assert summary['performance']['mujoco_viewer']['count'] == 0
        assert summary['performance']['active_loop']['count'] > 0
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
