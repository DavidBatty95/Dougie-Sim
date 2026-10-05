import math
import os
import socket
import threading

import pytest

from unitree_g1_mujoco_bridge.sim_manipulation_io import (
    CommandClient,
    MAX_DATAGRAM_SIZE,
    ProtocolError,
    StateCache,
    format_base_pin_command,
    format_manipulation_command,
    parse_ack,
    parse_state_record,
)


def packet(quaternion: str = "0 0 0 1") -> bytes:
    return f"OBJECT red_cube 1 2 3 {quaternion} 0.2 0.3 0.4".encode("ascii")


def test_parser_preserves_pose_size_and_type() -> None:
    record = parse_state_record(b"SURFACE table_top -1 2.5 0 0 0 0 1 2 3 0.1")
    assert record.record_type == "SURFACE"
    assert record.object_id == "table_top"
    assert record.position == (-1.0, 2.5, 0.0)
    assert record.orientation == (0.0, 0.0, 0.0, 1.0)
    assert record.size == (2.0, 3.0, 0.1)


def test_parser_normalizes_quaternion() -> None:
    record = parse_state_record(packet("0 0 0 2"))
    assert record.orientation == (0.0, 0.0, 0.0, 1.0)
    assert math.sqrt(sum(value * value for value in record.orientation)) == pytest.approx(1.0)


@pytest.mark.parametrize(
    "malformed",
    [
        b"THING red_cube 1 2 3 0 0 0 1 1 1 1",
        b"OBJECT bad/id 1 2 3 0 0 0 1 1 1 1",
        b"OBJECT red_cube 1 2 3 0 0 0 1 1 1",
        b"OBJECT  red_cube 1 2 3 0 0 0 1 1 1 1",
        b"OBJECT red_cube nan 2 3 0 0 0 1 1 1 1",
        b"OBJECT red_cube 1 2 3 0 0 0 inf 1 1 1",
        b"OBJECT red_cube 1 2 3 0 0 0 0 1 1 1",
        b"OBJECT red_cube 1 2 3 0 0 0 1 1 0 1",
        b"OBJECT red_cube 1 2 3 0 0 0 1 1 1 1 extra",
    ],
)
def test_parser_rejects_malformed_packets(malformed: bytes) -> None:
    with pytest.raises(ProtocolError):
        parse_state_record(malformed)


def test_parser_rejects_oversized_datagram() -> None:
    with pytest.raises(ProtocolError, match="oversized"):
        parse_state_record(b"X" * (MAX_DATAGRAM_SIZE + 1))


def test_cache_returns_only_fresh_latest_records() -> None:
    cache = StateCache()
    old = parse_state_record(packet())
    latest = parse_state_record(b"OBJECT red_cube 4 5 6 0 0 0 1 1 1 1")
    surface = parse_state_record(b"SURFACE shelf 0 0 0 0 0 0 1 1 1 1")
    cache.update(old, 10.0)
    cache.update(latest, 11.0)
    cache.update(surface, 9.0)
    assert cache.fresh_records(11.4, 0.5) == (latest,)
    assert cache.fresh_records(11.6, 0.5) == ()


def test_command_formatting_and_validation() -> None:
    assert format_manipulation_command("GRASP", "left", "blue-cube.2") == b"GRASP left blue-cube.2"
    assert format_manipulation_command("RELEASE", "right") == b"RELEASE right"
    with pytest.raises(ProtocolError):
        format_manipulation_command("GRASP", "middle", "red_cube")
    with pytest.raises(ProtocolError):
        format_manipulation_command("GRASP", "left", "red cube")
    with pytest.raises(ProtocolError):
        format_manipulation_command("RELEASE", "left", "red_cube")
    assert format_base_pin_command(True) == b"PIN_BASE"
    assert format_base_pin_command(False) == b"UNPIN_BASE"
    with pytest.raises(ProtocolError):
        format_base_pin_command(1)


def test_object_selectable_service_preserves_requested_identity_and_rejects_bad_ids():
    pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from g1_msgs.srv import GraspObject
    from unitree_g1_mujoco_bridge.sim_manipulation_io import _create_node_class
    cls, _ = _create_node_class()
    node = object.__new__(cls)
    sent = []
    def command(payload):
        sent.append(payload)
        return True, 'grasped box_blue'
    node._send_command = command
    result = node._grasp_object(SimpleNamespace(arm='right',object_id='box_blue'),GraspObject.Response())
    assert result.success and sent == [b'GRASP right box_blue']
    result = node._grasp_object(SimpleNamespace(arm='right',object_id='bad/id'),GraspObject.Response())
    assert not result.success and len(sent) == 1


@pytest.mark.parametrize(
    ("acknowledgement", "expected"),
    [
        (b"OK grasped red_cube", (True, "grasped red_cube")),
        (b"ERROR object unavailable", (False, "object unavailable")),
    ],
)
def test_parse_ack(acknowledgement: bytes, expected: tuple[bool, str]) -> None:
    assert parse_ack(acknowledgement) == expected


@pytest.mark.parametrize(
    "acknowledgement",
    [
        b"OK",
        b"OK ",
        b"ERROR",
        b"ERROR ",
        b"SUCCESS done",
        b"OK  done",
        b"ERROR failed ",
        b"OK done\n",
        b"OK \xff",
    ],
)
def test_parse_ack_rejects_malformed(acknowledgement: bytes) -> None:
    with pytest.raises(ProtocolError):
        parse_ack(acknowledgement)


def test_parse_ack_rejects_oversized_response() -> None:
    with pytest.raises(ProtocolError, match="oversized"):
        parse_ack(b"OK " + b"x" * MAX_DATAGRAM_SIZE)


def test_command_client_waits_for_simulator_ack(tmp_path) -> None:
    server_path = str(tmp_path / "simulator.sock")
    client_path = str(tmp_path / "client.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(server_path)
    client = CommandClient(server_path, client_path, 0.5)
    server.sendto(b"ERROR stale response", client_path)

    def reply() -> None:
        command, sender = server.recvfrom(MAX_DATAGRAM_SIZE)
        assert command == b"GRASP left red_cube"
        server.sendto(b"OK grasped red_cube", sender)

    responder = threading.Thread(target=reply)
    responder.start()
    try:
        assert client.send(b"GRASP left red_cube") == (True, "grasped red_cube")
        responder.join(timeout=1.0)
        assert not responder.is_alive()
    finally:
        client.close()
        server.close()
        os.unlink(server_path)
    assert not os.path.exists(client_path)


def test_command_client_times_out_without_ack(tmp_path) -> None:
    server_path = str(tmp_path / "simulator.sock")
    client_path = str(tmp_path / "client.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(server_path)
    client = CommandClient(server_path, client_path, 0.01)
    try:
        assert client.send(b"RELEASE right") == (False, "command acknowledgement timed out")
    finally:
        client.close()
        server.close()
        os.unlink(server_path)
