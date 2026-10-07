"""Exercise real DDS radiation publications, not mocked message constructors."""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def test_radiation_measurements_and_3d_posterior_reach_ros_subscribers(tmp_path):
    domain = 100+os.getpid()%100
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node('radiation_contract_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    samples, clouds = [], []
    subscriptions = [
        node.create_subscription(String, '/radiation/measurement', lambda m:samples.append(json.loads(m.data)), 10),
        node.create_subscription(PointCloud2, '/radiation/gp_map', clouds.append, 10),
    ]
    log = (tmp_path/'child.log').open('w')
    process = subprocess.Popen([sys.executable, '-m', 'dougie_sim.radiation_run', '--headless', '--ros',
        '--duration', '30', '--dose-budget', '.2', '--output', str(tmp_path/'run')],
        env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic()+60.
        while process.poll() is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        assert process.poll() == 0, (tmp_path/'child.log').read_text()
        assert len(samples) >= 3 and clouds
        sample = samples[-1]
        assert len(sample['position']) == 3 and sample['counts'] >= 0
        assert sample['dose_upper_usv'] >= sample['dose_usv'] >= 0.
        assert 'sources' not in sample
        cloud = clouds[-1]
        assert cloud.header.frame_id == 'odom' and cloud.width > 0
        assert {'x', 'y', 'z', 'dose_rate_usv_h', 'upper_rate_usv_h'} <= {f.name for f in cloud.fields}
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
