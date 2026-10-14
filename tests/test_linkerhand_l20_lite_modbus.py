"""Offline contract tests for the validated L20 Lite Modbus transport/card."""

from __future__ import annotations

import sys
import threading
import io
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20_lite"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))

from modbus_client import L20LiteModbusClient, crc16
from modbus_device import HandPlugin, JOINT_NAMES, RIGHT_FIST_POSE, RIGHT_OPEN_POSE
from common.vendor_runtime import DriverBundle, load_config


def calibrated_config(**overrides):
    return {"fist_pose": list(RIGHT_FIST_POSE), **overrides}


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


def capture_completions(plugin):
    events = []
    done = threading.Event()

    def callback(action_id, status, result):
        events.append((action_id, status, result))
        done.set()

    plugin._acp_callback = callback
    return events, done


def completed_motion(plugin, action, args, events, done):
    done.clear()
    accepted = plugin.dispatch(action, args)
    assert accepted["state"] == "running"
    assert accepted["action_id"].startswith(f"hand_{action}_")
    assert done.wait(2), "ACP completion was not delivered"
    action_id, status, result = events[-1]
    assert action_id == accepted["action_id"]
    assert status == "completed"
    for _ in range(100):
        if plugin.dispatch("info", {})["active_action_id"] is None:
            break
        threading.Event().wait(0.001)
    assert plugin.dispatch("info", {})["active_action_id"] is None
    return result


def test_card_starts_without_motion_and_closes_then_opens(monkeypatch):
    assert JOINT_NAMES == [
        "thumb_root", "thumb_sway", "index_root", "middle_root", "ring_root",
        "pinky_root", "index_sway", "middle_sway", "ring_sway", "thumb_roll",
    ]
    assert RIGHT_FIST_POSE == [93, 255, 0, 0, 0, 0, 128, 128, 128, 255]
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config(poll_interval_s=0.02))
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    plugin.start()
    assert connection.transport.sent == []
    assert plugin.dispatch("start", {}) == {"state": "ready"}
    assert connection.transport.sent == []
    closed = completed_motion(plugin, "close", {"confirm_motion": True}, events, done)
    assert closed["state"] == "completed"
    assert connection.client.get_positions() == RIGHT_FIST_POSE
    opened = completed_motion(plugin, "open", {"confirm_motion": True}, events, done)
    assert opened["state"] == "completed"
    assert connection.client.get_positions() == RIGHT_OPEN_POSE
    assert plugin.dispatch("stop", {})["state"] == "idle"
    assert connection.client.get_torques() == [0] * 10
    plugin.stop()
    assert connection.transport.closed


def test_card_rejects_bad_joint_without_position_write():
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    with pytest.raises(ValueError, match="unknown joint"):
        plugin.dispatch("set_joint", {"joint": "nonexistent", "position": 12, "confirm_motion": True})
    assert connection.transport.sent == []


def test_left_hand_needs_explicit_side_specific_poses():
    connection = FakeConnection()
    connection.side = "left"
    with pytest.raises(ValueError, match="left-hand"):
        HandPlugin(connection, {})


def test_right_hand_needs_explicit_calibrated_fist_pose():
    connection = FakeConnection()
    with pytest.raises(ValueError, match="fist_pose must be explicitly calibrated"):
        HandPlugin(connection, {})
    assert connection.transport.sent == []


def test_bundled_right_hand_config_explicitly_sets_calibrated_fist_pose():
    config = load_config(str(DRIVER / "main.py"))
    assert config["side"] == "right"
    assert config["plugins"]["hand"]["fist_pose"] == RIGHT_FIST_POSE


def test_canvas_stop_before_connection_does_not_open_serial():
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    assert plugin.dispatch("stop", {}) == {"state": "idle", "torque_released": True}
    assert connection.transport.sent == []


def test_canvas_card_schema_and_csv_positions(monkeypatch):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config(poll_interval_s=0.02))
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
    assert plugin.PREFIX == "hand"
    assert card["inputSchema"]["x-completion"]["actions"] == ["open", "close", "set_joint", "set_positions"]
    assert card["inputSchema"]["allOf"][0]["then"]["required"] == ["confirm_motion"]
    for action in ("open", "close", "set_joint", "set_positions"):
        assert card["inputSchema"]["x-action-params"][action]["params"][-1] == "confirm_motion"
    for action in ("get_state", "set_speed", "set_torque", "release"):
        assert "confirm_motion" not in card["inputSchema"]["x-action-params"][action]["params"]
    assert plugin.dispatch("info", {})["serial_port"] == "/dev/fake"
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    pose = [254, 254, 234, 254, 254, 254, 127, 126, 127, 254]
    result = completed_motion(plugin, "set_positions", {"positions": ",".join(map(str, pose)), "confirm_motion": True}, events, done)
    assert result["state"] == "completed"
    assert connection.client.get_positions() == pose
    assert plugin.dispatch("release", {})["state"] == "released"
    with pytest.raises(ValueError, match="expected 10"):
        plugin.dispatch("set_positions", {"positions": "1,2,3", "confirm_motion": True})


def test_motion_error_notifies_acp_and_allows_later_request():
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    events, done = capture_completions(plugin)
    connection.transport.corrupt_next = True
    accepted = plugin.dispatch("close", {"confirm_motion": True})
    assert accepted["state"] == "running"
    assert done.wait(2)
    assert events[0][0] == accepted["action_id"]
    assert events[0][1] == "error"
    assert "CRC" in events[0][2]["message"]
    for _ in range(100):
        if plugin.dispatch("info", {})["active_action_id"] is None:
            break
        threading.Event().wait(0.001)
    assert plugin.dispatch("info", {})["active_action_id"] is None


def test_acp_callback_posts_terminal_payload_and_checks_acknowledgement(monkeypatch):
    plugin = HandPlugin(FakeConnection(), calibrated_config())
    requests = []

    def fake_urlopen(request, **kwargs):
        requests.append((request, kwargs))
        return io.BytesIO(b'{"ok": true, "action_id": "hand-test"}')

    monkeypatch.setattr("modbus_device.urllib.request.urlopen", fake_urlopen)
    plugin._acp_callback("hand-test", "completed", {"state": "completed"})
    request, kwargs = requests[0]
    assert request.full_url == "https://localhost:15678/api/acp/complete"
    assert kwargs["timeout"] == 5
    assert json.loads(request.data)["tool"] == "hand"
    assert json.loads(request.data)["status"] == "completed"


def test_failed_acp_delivery_blocks_new_motion():
    plugin = HandPlugin(FakeConnection(), calibrated_config())
    done = threading.Event()

    def reject(*_args):
        done.set()
        raise RuntimeError("Core did not acknowledge")

    plugin._acp_callback = reject
    accepted = plugin.dispatch("open", {"confirm_motion": True})
    assert accepted["state"] == "running"
    assert done.wait(2)
    for _ in range(100):
        if plugin._completion_delivery_failed:
            break
        threading.Event().wait(0.001)
    with pytest.raises(RuntimeError, match="ACP completion delivery failed"):
        plugin.dispatch("close", {"confirm_motion": True})


def test_overlapping_motion_is_rejected_until_completion(monkeypatch):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    events, done = capture_completions(plugin)
    entered, proceed = threading.Event(), threading.Event()

    def pause(_seconds):
        entered.set()
        assert proceed.wait(2)

    monkeypatch.setattr("modbus_device.time.sleep", pause)
    accepted = plugin.dispatch("close", {"confirm_motion": True})
    assert entered.wait(2)
    with pytest.raises(RuntimeError, match="another hand motion"):
        plugin.dispatch("open", {"confirm_motion": True})
    proceed.set()
    assert done.wait(2)
    assert events[0][0] == accepted["action_id"]


def test_stop_notifies_cancelled_without_followup_target(monkeypatch):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    events, done = capture_completions(plugin)
    entered, proceed = threading.Event(), threading.Event()

    def pause(_seconds):
        entered.set()
        assert proceed.wait(2)

    monkeypatch.setattr("modbus_device.time.sleep", pause)
    accepted = plugin.dispatch("close", {"confirm_motion": True})
    assert entered.wait(2)
    stopped = threading.Thread(target=lambda: plugin.dispatch("stop", {}))
    stopped.start()
    assert plugin._stop_requested.wait(2)
    proceed.set()
    stopped.join(2)
    assert done.wait(2)
    assert events[0][0] == accepted["action_id"]
    assert events[0][1] == "cancelled"
    assert connection.client.get_positions() != RIGHT_FIST_POSE
    assert connection.client.get_torques() == [0] * 10


@pytest.mark.parametrize("action,args", [
    ("open", {}),
    ("close", {}),
    ("set_joint", {"joint": "index_root", "position": 100}),
    ("set_positions", {"positions": "100,100,100,100,100,100,100,100,100,100"}),
])
@pytest.mark.parametrize("confirmation", [None, False, 1, "true"])
def test_motion_requires_explicit_boolean_confirmation_before_serial_access(action, args, confirmation):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    request = dict(args)
    if confirmation is not None:
        request["confirm_motion"] = confirmation
    with pytest.raises(ValueError, match="confirm_motion must be true"):
        plugin.dispatch(action, request)
    assert connection.transport.sent == []
    assert connection._client is None


def test_mcp_bundle_discovers_hand_card_without_opening_serial():
    connection = FakeConnection()
    bundle = DriverBundle([HandPlugin(connection, calibrated_config())])
    assert bundle.get_all_tools()[0]["name"] == "hand"
    assert bundle.dispatch("hand", {"action": "start"}) == {"state": "ready"}
    assert connection.transport.sent == []
