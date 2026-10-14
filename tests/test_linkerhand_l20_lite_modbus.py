"""Offline contract tests for the validated L20 Lite Modbus transport/card."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20_lite"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))

from modbus_client import L20LiteModbusClient, crc16
from modbus_device import HandPlugin, JOINT_NAMES, RIGHT_FIST_POSE, RIGHT_OPEN_POSE
from common.vendor_runtime import DriverBundle


class FakeSerial:
    def __init__(self, **kwargs):
        self.options = kwargs
        self.registers = {0: [254, 254, 254, 254, 254, 254, 127, 126, 127, 254],
                          10: [100] * 10, 20: [60] * 10}
        self.sent = []
        self.pending = bytearray()
        self.closed = False
        self.corrupt_next = False
        self.timeout_next = False

    def reset_input_buffer(self):
        self.pending.clear()

    def write(self, frame):
        self.sent.append(bytes(frame))
        assert crc16(frame[:-2]) == frame[-2:]
        slave, function = frame[:2]
        address = int.from_bytes(frame[2:4], "big")
        count = int.from_bytes(frame[4:6], "big")
        assert count == 10
        if function == 0x04:
            values = self.registers[address]
            data = b"".join(value.to_bytes(2, "big") for value in values)
            payload = bytes((slave, 0x04, len(data))) + data
        elif function == 0x10:
            assert frame[6] == 20
            data = frame[7:-2]
            self.registers[address] = [int.from_bytes(data[i:i + 2], "big") for i in range(0, 20, 2)]
            payload = bytes((slave, 0x10)) + frame[2:6]
        else:
            raise AssertionError(f"unexpected function {function}")
        if not self.timeout_next:
            reply = payload + crc16(payload)
            self.pending.extend(reply[:-1] + bytes((reply[-1] ^ 0xFF,)) if self.corrupt_next else reply)
        self.timeout_next = False
        self.corrupt_next = False
        return len(frame)

    def flush(self):
        pass

    def read(self, size):
        part = bytes(self.pending[:size])
        del self.pending[:size]
        return part

    def close(self):
        self.closed = True


def make_client():
    transport = FakeSerial(port="/dev/fake", baudrate=115200)
    client = L20LiteModbusClient("/dev/fake", "right", serial_factory=lambda **_: transport)
    return client, transport


def test_read_position_matches_official_sdk_frame_and_decodes_reply():
    client, transport = make_client()
    assert client.get_positions() == RIGHT_OPEN_POSE
    assert transport.sent[-1] == bytes.fromhex("27 04 00 00 00 0a 77 0b")
    assert transport.options["baudrate"] == 115200


def test_writes_match_official_sdk_frames():
    client, transport = make_client()
    client.set_speeds([60] * 10)
    client.set_torques([100] * 10)
    client.set_positions([254, 254, 234, 254, 254, 254, 127, 127, 127, 254])
    assert transport.sent == [
        bytes.fromhex("27 10 00 14 00 0a 14 00 3c 00 3c 00 3c 00 3c 00 3c 00 3c 00 3c 00 3c 00 3c 00 3c 7d 51"),
        bytes.fromhex("27 10 00 0a 00 0a 14 00 64 00 64 00 64 00 64 00 64 00 64 00 64 00 64 00 64 00 64 5b 8e"),
        bytes.fromhex("27 10 00 00 00 0a 14 00 fe 00 fe 00 ea 00 fe 00 fe 00 fe 00 7f 00 7f 00 7f 00 fe df 69"),
    ]
    assert client.get_positions()[2] == 234


def test_bad_crc_and_timeout_never_return_fake_positions():
    client, transport = make_client()
    transport.corrupt_next = True
    with pytest.raises(ValueError, match="CRC"):
        client.get_positions()
    transport.timeout_next = True
    with pytest.raises(TimeoutError):
        client.get_positions()


class FakeConnection:
    side = "right"
    port = "/dev/fake"
    _client = None

    def __init__(self):
        self.client, self.transport = make_client()

    def get(self):
        self._client = self.client
        return self.client

    def close(self):
        self.client.close()


def test_card_starts_without_motion_and_closes_then_opens(monkeypatch):
    assert JOINT_NAMES == [
        "thumb_root", "thumb_sway", "index_root", "middle_root", "ring_root",
        "pinky_root", "index_sway", "middle_sway", "ring_sway", "thumb_roll",
    ]
    assert RIGHT_FIST_POSE == [93, 255, 0, 0, 0, 0, 128, 128, 128, 255]
    connection = FakeConnection()
    plugin = HandPlugin(connection, {"poll_interval_s": 0.02})
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    plugin.start()
    assert connection.transport.sent == []
    assert plugin.dispatch("start", {}) == {"state": "ready"}
    assert connection.transport.sent == []
    closed = plugin.dispatch("close", {"confirm_motion": True})
    assert closed["state"] == "completed"
    assert connection.client.get_positions() == RIGHT_FIST_POSE
    opened = plugin.dispatch("open", {"confirm_motion": True})
    assert opened["state"] == "completed"
    assert connection.client.get_positions() == RIGHT_OPEN_POSE
    assert plugin.dispatch("stop", {})["state"] == "idle"
    assert connection.client.get_torques() == [0] * 10
    plugin.stop()
    assert connection.transport.closed


def test_card_rejects_bad_joint_without_position_write():
    connection = FakeConnection()
    plugin = HandPlugin(connection, {})
    with pytest.raises(ValueError, match="unknown joint"):
        plugin.dispatch("set_joint", {"joint": "nonexistent", "position": 12, "confirm_motion": True})
    assert connection.transport.sent == []


def test_left_hand_needs_explicit_side_specific_poses():
    connection = FakeConnection()
    connection.side = "left"
    with pytest.raises(ValueError, match="left-hand"):
        HandPlugin(connection, {})


def test_canvas_stop_before_connection_does_not_open_serial():
    connection = FakeConnection()
    plugin = HandPlugin(connection, {})
    assert plugin.dispatch("stop", {}) == {"state": "idle", "torque_released": True}
    assert connection.transport.sent == []


def test_canvas_card_schema_and_csv_positions(monkeypatch):
    connection = FakeConnection()
    plugin = HandPlugin(connection, {"poll_interval_s": 0.02})
    card = plugin.get_tool()
    actions = card["inputSchema"]["properties"]["action"]["enum"]
    assert card["title"] == "L20 Lite 灵巧手"
    assert card["type"] == "actuator"
    assert "release" in actions and "set_positions" in actions
    assert card["inputSchema"]["properties"]["positions"]["type"] == "string"
    assert card["inputSchema"]["properties"]["confirm_motion"] == {
        "type": "boolean", "const": True,
        "description": "确认手附近无人和障碍物后，每次运动显式输入 true",
    }
    assert card["inputSchema"]["x-is-dangerous"] is True
    assert card["inputSchema"]["allOf"][0]["then"]["required"] == ["confirm_motion"]
    for action in ("open", "close", "set_joint", "set_positions"):
        assert card["inputSchema"]["x-action-params"][action]["params"][-1] == "confirm_motion"
    for action in ("get_state", "set_speed", "set_torque", "release"):
        assert "confirm_motion" not in card["inputSchema"]["x-action-params"][action]["params"]
    assert plugin.dispatch("info", {})["serial_port"] == "/dev/fake"
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    pose = [254, 254, 234, 254, 254, 254, 127, 126, 127, 254]
    result = plugin.dispatch("set_positions", {"positions": ",".join(map(str, pose)), "confirm_motion": True})
    assert result["state"] == "completed"
    assert connection.client.get_positions() == pose
    assert plugin.dispatch("release", {})["state"] == "released"
    with pytest.raises(ValueError, match="expected 10"):
        plugin.dispatch("set_positions", {"positions": "1,2,3", "confirm_motion": True})


@pytest.mark.parametrize("action,args", [
    ("open", {}),
    ("close", {}),
    ("set_joint", {"joint": "index_root", "position": 100}),
    ("set_positions", {"positions": "100,100,100,100,100,100,100,100,100,100"}),
])
@pytest.mark.parametrize("confirmation", [None, False, 1, "true"])
def test_motion_requires_explicit_boolean_confirmation_before_serial_access(action, args, confirmation):
    connection = FakeConnection()
    plugin = HandPlugin(connection, {})
    request = dict(args)
    if confirmation is not None:
        request["confirm_motion"] = confirmation
    with pytest.raises(ValueError, match="confirm_motion must be true"):
        plugin.dispatch(action, request)
    assert connection.transport.sent == []
    assert connection._client is None


def test_mcp_bundle_discovers_hand_card_without_opening_serial():
    connection = FakeConnection()
    bundle = DriverBundle([HandPlugin(connection, {})])
    assert bundle.get_all_tools()[0]["name"] == "hand"
    assert bundle.dispatch("hand", {"action": "start"}) == {"state": "ready"}
    assert connection.transport.sent == []
