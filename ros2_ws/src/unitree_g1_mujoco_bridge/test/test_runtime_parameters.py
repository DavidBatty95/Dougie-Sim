import json
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from unitree_g1_mujoco_bridge.runtime_parameters import (
    COMMAND_PARAMETERS,RESPONSE_PARAMETERS,capture_parameters,compare_motion_parameters,parameter_record,
)


@pytest.fixture
def graph():
    rclpy.init()
    executor = SingleThreadedExecutor()
    reader = Node('parameter_capture_test')
    server = Node('resolved_parameter_test',parameter_overrides=[Parameter('gain',value=7.5)])
    server.declare_parameter('gain',1.)
    for node in (reader,server):
        executor.add_node(node)
    # Wait for discovery rather than treating an absent graph as a test pass.
    end = time.monotonic()+3.
    while ('resolved_parameter_test','/') not in reader.get_node_names_and_namespaces():
        assert time.monotonic()<end
        executor.spin_once(timeout_sec=.02)
    yield reader,server,lambda timeout:executor.spin_once(timeout_sec=timeout)
    executor.shutdown()
    reader.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


def test_capture_records_resolved_override_and_parameter_types(graph):
    reader,server,pump = graph
    result = capture_parameters(reader,pump,timeout=2.,required_nodes=('/resolved_parameter_test',))
    assert result['required_complete']
    gain = result['nodes']['/resolved_parameter_test']['parameters']['gain']
    assert gain == dict(type=3,value=7.5)
    json.dumps(result,allow_nan=False)


def test_missing_required_node_does_not_get_default_values(graph):
    reader,server,pump = graph
    result = capture_parameters(reader,pump,timeout=.5,required_nodes=('/missing_parameter_test',))
    assert not result['required_complete']
    assert result['nodes']['/missing_parameter_test'] == dict(status='missing_node')


def test_parameter_server_without_services_times_out_within_one_budget(graph):
    reader,server,pump = graph
    silent = Node('silent_parameter_test',start_parameter_services=False)
    try:
        end = time.monotonic()+3.
        while ('silent_parameter_test','/') not in reader.get_node_names_and_namespaces():
            assert time.monotonic()<end
            pump(.02)
        result = capture_parameters(reader,pump,timeout=.2,required_nodes=('/silent_parameter_test',))
        assert not result['required_complete']
        assert result['nodes']['/silent_parameter_test']['status'] == 'timeout'
        assert result['elapsed_monotonic_s'] < 1.
    finally:
        silent.destroy_node()


def test_duplicate_node_name_is_not_ambiguous_provenance(graph):
    reader,server,pump = graph
    duplicate = Node('resolved_parameter_test')
    try:
        end = time.monotonic()+3.
        while reader.get_node_names_and_namespaces().count(('resolved_parameter_test','/'))<2:
            assert time.monotonic()<end
            pump(.02)
        result = capture_parameters(reader,pump,timeout=.2,required_nodes=('/resolved_parameter_test',))
        assert not result['required_complete']
        assert result['nodes']['/resolved_parameter_test']['status'] == 'duplicate_node_name'
    finally:
        duplicate.destroy_node()


def test_nonfinite_and_byte_parameters_are_portable_json():
    nan = parameter_record(Parameter('value',value=float('nan')).get_parameter_value())
    assert nan['value'] == {'nonfinite':'nan'}
    byte_array = parameter_record(Parameter('value',type_=Parameter.Type.BYTE_ARRAY,
                                           value=[b'\x00',b'\xff']).get_parameter_value())
    assert byte_array['value'] == [0,255]
    json.dumps([nan,byte_array],allow_nan=False)


def test_optional_unresponsive_node_does_not_consume_required_timeout(graph):
    reader,server,pump = graph
    silent = Node('optional_parameter_test',start_parameter_services=False)
    try:
        end = time.monotonic()+3.
        while ('optional_parameter_test','/') not in reader.get_node_names_and_namespaces():
            assert time.monotonic()<end
            pump(.02)
        result = capture_parameters(reader,pump,timeout=2.,optional_timeout=.1,
                                    required_nodes=('/resolved_parameter_test',))
        assert result['required_complete'] and not result['complete']
        assert result['nodes']['/optional_parameter_test']['status'] == 'optional_timeout'
        assert result['elapsed_monotonic_s'] < 1.
    finally:
        silent.destroy_node()


def test_resolved_motion_contract_detects_override_and_missing_parameter():
    nodes = ('/human_aware_motion_planner','/human_collision_avoidance','/g1_policy_cmd_vel_controller')
    snapshot = dict(nodes={name:dict(status='captured',parameters={
        key:dict(type=3,value=.5) for key in (*COMMAND_PARAMETERS,*RESPONSE_PARAMETERS)}) for name in nodes})
    assert compare_motion_parameters(snapshot)['consistent']
    snapshot['nodes'][nodes[2]]['parameters']['cmd_accel_yaw']['value'] = .9
    del snapshot['nodes'][nodes[1]]['parameters']['response_yaw_gain']
    result = compare_motion_parameters(snapshot)
    assert not result['consistent']
    assert {item['parameter'] for item in result['differences']} == {'cmd_accel_yaw','response_yaw_gain'}
