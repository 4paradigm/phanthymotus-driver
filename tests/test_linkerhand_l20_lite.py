from __future__ import annotations

import importlib.util
import queue
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20_lite"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


can_client = load("l20_lite_can_client", DRIVER / "can_client.py")
device = load("l20_lite_device", DRIVER / "device.py")


class FakeTransport:
    def __init__(self):
        self.sent = []
        self.incoming = queue.Queue()
        self.closed = False

    def send(self, can_id, data):
        self.sent.append((can_id, bytes(data)))
        command = data[0]
        responses = {
            0x64: bytes([0x64, 0x21, 0x00, 0x22, 0x00, 0x00, 0x21]),
            0xC0: bytes([0xC0, 0x00, 0x4C, 0x48, 0x54, 0x32, 0x30, 0x2D]),
            0xC1: bytes([0xC1, 0x04, 0x00, 0x01]),
            0xC2: bytes([0xC2, 0x01, 0x00, 0x12]),
            0xC4: bytes([0xC4, 0x0A, 0x01, 0x00]),
            0x01: bytes([0x01, 114, 0, 0, 0, 0]),
            0x04: bytes([0x04, 0, 0, 0, 0, 0]),
            0x05: bytes([0x05, 10, 20, 30, 40, 50]),
            0x06: bytes([0x06, 60, 70, 80, 90, 100]),
        }
        if len(data) == 1 and command in responses:
            self.incoming.put((can_id, responses[command]))

    def recv(self, timeout):
        try:
            return self.incoming.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self):
        self.closed = True


def test_right_hand_reads_ten_positions_and_speeds():
    transport = FakeTransport()
    client = can_client.L20LiteCanClient(transport=transport)
    try:
        assert client.can_id == 0x27
        assert client.get_positions() == [114, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        assert client.get_speeds() == [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
        assert [payload for _, payload in transport.sent[:6]] == [
            bytes([0x64]), bytes([0xC0]), bytes([0xC1]), bytes([0xC2]),
            bytes([0xC4]), bytes([0xB1, 0xC6]),
        ]
    finally:
        client.close()


def test_position_write_is_split_six_plus_four():
    transport = FakeTransport()
    client = can_client.L20LiteCanClient(transport=transport)
    try:
        transport.sent.clear()
        client.set_positions(range(10))
        assert transport.sent == [
            (0x27, bytes([0x04, 6, 7, 8, 9])),
            (0x27, bytes([0x01, 0, 1, 2, 3, 4, 5])),
        ]
    finally:
        client.close()


def test_speed_write_repeats_the_five_finger_vector():
    transport = FakeTransport()
    client = can_client.L20LiteCanClient(transport=transport)
    try:
        transport.sent.clear()
        client.set_speeds(range(5))
        assert transport.sent == [
            (0x27, bytes([0x05, 0, 1, 2, 3, 4])),
            (0x27, bytes([0x05, 0, 1, 2, 3, 4])),
        ]
    finally:
        client.close()


def test_torque_write_applies_the_same_five_fingers_to_both_groups():
    transport = FakeTransport()
    client = can_client.L20LiteCanClient(transport=transport)
    try:
        transport.sent.clear()
        client.set_torques([0, 50, 0, 0, 0])
        assert transport.sent == [
            (0x27, bytes([0x02, 0, 50, 0, 0, 0])),
            (0x27, bytes([0x03, 0, 50, 0, 0, 0])),
        ]
    finally:
        client.close()


class FakeClient:
    def __init__(self):
        self.positions = [114, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.speeds = [80] * 10
        self.position_writes = []
        self.torque_writes = []
        self.operations = []

    def get_positions(self):
        return list(self.positions)

    def get_speeds(self):
        return list(self.speeds)

    def set_positions(self, values):
        self.positions = list(values)
        self.position_writes.append(list(values))
        self.operations.append(("positions", list(values)))

    def set_speeds(self, values):
        self.speeds = list(values)
        self.operations.append(("speeds", list(values)))

    def set_torques(self, values):
        self.torques = list(values)
        self.torque_writes.append(list(values))
        self.operations.append(("torques", list(values)))


class FakeConnection:
    side = "right"
    channel = "can0"

    def __init__(self):
        self.client = FakeClient()

    def get(self):
        return self.client

    def close(self):
        pass


def test_plugin_does_not_move_on_start_and_declares_safety_actions():
    connection = FakeConnection()
    plugin = device.HandPlugin(connection, {})
    plugin.start()
    assert connection.client.position_writes == []
    assert plugin.open_pose == [250, 250, 250, 250, 250, 250, 128, 128, 128, 250]
    assert plugin.fist_pose == [0, 250, 0, 0, 0, 0, 128, 128, 128, 250]
    actions = plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
    assert set(actions) == {
        "get_state", "open", "close", "set_joint", "set_positions",
        "set_speed", "set_torque", "stop",
    }


def test_single_joint_move_preserves_the_other_nine_values():
    connection = FakeConnection()
    plugin = device.HandPlugin(connection, {
        "motion_speed": 20,
        "motion_torque": 30,
        "speed_settle_s": 0,
        "poll_interval_s": 0,
    })
    result = plugin.dispatch("set_joint", {"joint": "index_root", "position": 5})
    assert connection.client.position_writes[-1] == [114, 0, 5, 0, 0, 0, 0, 0, 0, 0]
    assert result["positions"]["index_root"] == 5
    assert result["state"] == "completed"
    assert result["torque_released"] is True
    assert connection.client.torque_writes == [
        [0] * 5,
        [0, 30, 0, 0, 0],
        [0] * 5,
    ]
    assert connection.client.operations[0] == ("torques", [0] * 5)
    assert connection.client.operations[-1] == ("torques", [0] * 5)


def test_right_open_stages_four_fingers_before_thumb_and_releases(monkeypatch):
    connection = FakeConnection()
    plugin = device.HandPlugin(connection, {
        "motion_speed": 100,
        "motion_torque": 200,
        "speed_settle_s": 0,
        "open_finger_stage_s": 0.1,
        "open_thumb_stage_s": 0.1,
        "command_period_s": 0.05,
    })
    clock = [0.0]
    monkeypatch.setattr(device.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(
        device.time, "sleep", lambda seconds: clock.__setitem__(0, clock[0] + seconds))

    result = plugin.dispatch("open", {})

    assert connection.client.position_writes[0] == [
        114, 0, 250, 250, 250, 250, 0, 0, 0, 0,
    ]
    assert connection.client.position_writes[-1] == plugin.open_pose
    assert connection.client.torque_writes == [
        [0] * 5,
        [0, 200, 200, 200, 200],
        [200] * 5,
        [0] * 5,
    ]
    assert result["state"] == "completed_released"
    assert result["torque_released"] is True
    assert plugin._commanded_positions is None


def test_single_joint_uses_last_target_when_feedback_is_stale():
    connection = FakeConnection()
    plugin = device.HandPlugin(connection, {
        "motion_speed": 100,
        "motion_torque": 200,
        "speed_settle_s": 0,
        "poll_interval_s": 0,
    })
    # The four fingers physically opened, but this firmware still reports 0.
    plugin._commanded_positions = [251, 0, 250, 250, 250, 250, 0, 0, 0, 0]
    connection.client.positions = [251, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    result = plugin.dispatch(
        "set_joint", {"joint": "thumb_sway", "position": 250})

    assert connection.client.position_writes[-1] == [
        251, 250, 250, 250, 250, 250, 0, 0, 0, 0,
    ]
    assert result["active_joints"] == ["thumb_sway"]
    assert connection.client.torque_writes == [
        [0] * 5,
        [200, 0, 0, 0, 0],
        [0] * 5,
    ]


def test_torque_action_only_configures_future_moves():
    connection = FakeConnection()
    plugin = device.HandPlugin(connection, {})
    result = plugin.dispatch("set_torque", {"torque": 50})
    assert result == {
        "state": "configured", "torque": 50, "applied": False,
    }
    assert plugin.motion_torque == 50
    assert connection.client.torque_writes == []


def test_stop_releases_torque_without_writing_a_position_target():
    connection = FakeConnection()
    plugin = device.HandPlugin(connection, {})
    result = plugin.dispatch("stop", {})
    assert result["state"] == "released"
    assert connection.client.torque_writes == [[0] * 5]
    assert connection.client.position_writes == []


def test_out_of_range_commands_are_rejected_before_can_write():
    connection = FakeConnection()
    plugin = device.HandPlugin(connection, {})
    with pytest.raises(ValueError, match="0..255"):
        plugin.dispatch("set_joint", {"joint": "index_root", "position": 256})
    assert connection.client.position_writes == []
