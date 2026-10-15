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
import modbus_device
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


class SelectiveFakeSerial(FakeSerial):
    def __init__(self, responding_ids):
        super().__init__(port="/dev/fake", baudrate=115200)
        self.responding_ids = responding_ids

    def write(self, frame):
        if frame[0] not in self.responding_ids:
            self.sent.append(bytes(frame))
            return len(frame)
        return super().write(frame)


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


def test_probe_reads_both_known_addresses_without_changing_target():
    transport = SelectiveFakeSerial({0x28})
    client = L20LiteModbusClient("/dev/fake", "right", serial_factory=lambda **_: transport)
    with pytest.raises(TimeoutError):
        client.probe_positions("right")
    assert client.probe_positions("left") == RIGHT_OPEN_POSE
    assert client.side == "right" and client.slave_id == 0x27
    assert [frame[0] for frame in transport.sent] == [0x27, 0x28]
    assert all(frame[1] == 0x04 for frame in transport.sent)


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


def make_detecting_connection(monkeypatch, responding_ids):
    transports = []

    def client_factory(port, side, **kwargs):
        transport = SelectiveFakeSerial(responding_ids)
        transports.append(transport)
        return L20LiteModbusClient(
            port, side, baudrate=kwargs["baudrate"], timeout_s=kwargs["timeout_s"],
            serial_factory=lambda **_: transport,
        )

    monkeypatch.setattr(modbus_device, "L20LiteModbusClient", client_factory)
    connection = modbus_device.L20LiteConnection({"serial_port": "/dev/fake", "side": "right"})
    return connection, transports


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


@pytest.mark.parametrize("side,slave_id", [("right", 0x27), ("left", 0x28)])
def test_auto_detected_hand_receives_all_device_actions(monkeypatch, side, slave_id):
    connection, transports = make_detecting_connection(monkeypatch, {slave_id})
    plugin = HandPlugin(connection, calibrated_config(auto_detect_gestures=True, poll_interval_s=0.02))
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)

    info = plugin.dispatch("info", {})
    assert info["side"] == side
    assert info["detected_sides"] == [side]
    assert info["detected_count"] == 1
    assert ("左手" if side == "left" else "右手") in info["description"]
    assert all(frame[1] == 0x04 for transport in transports for frame in transport.sent)

    closed = completed_motion(plugin, "close", {"confirm_motion": True}, events, done)
    assert closed["side"] == side
    assert closed["requested_positions"] == dict(zip(JOINT_NAMES, RIGHT_FIST_POSE))
    opened = completed_motion(plugin, "open", {"confirm_motion": True}, events, done)
    assert opened["side"] == side
    assert opened["requested_positions"] == dict(zip(JOINT_NAMES, RIGHT_OPEN_POSE))
    state = plugin.dispatch("get_state", {})
    assert state["side"] == side
    assert state["positions"] == dict(zip(JOINT_NAMES, RIGHT_OPEN_POSE))
    positions = completed_motion(plugin, "set_positions", {
        "index_flex": 220, "confirm_motion": True,
    }, events, done)
    assert positions["side"] == side
    assert positions["positions"]["index_root"] == 220
    assert positions["positions"]["middle_root"] == RIGHT_OPEN_POSE[3]
    before_settings = sum(len(transport.sent) for transport in transports)
    assert plugin.dispatch("set_torque", {"torque": 80})["applied"] is False
    assert sum(len(transport.sent) for transport in transports) == before_settings
    released = plugin.dispatch("release", {})
    assert released["side"] == side and released["torque_released"] is True
    writes = [frame for transport in transports for frame in transport.sent if frame[1] == 0x10]
    assert writes and all(frame[0] == slave_id for frame in writes)


@pytest.mark.parametrize("responding_ids,expected_sides", [
    (set(), []), ({0x27, 0x28}, ["right", "left"]),
])
def test_auto_actions_refuse_zero_or_two_hands(monkeypatch, responding_ids, expected_sides):
    connection, transports = make_detecting_connection(monkeypatch, responding_ids)
    plugin = HandPlugin(connection, calibrated_config(auto_detect_gestures=True))
    events, done = capture_completions(plugin)
    info = plugin.dispatch("info", {})
    assert info["detected_sides"] == expected_sides
    assert info["detected_count"] == len(expected_sides)
    with pytest.raises(RuntimeError, match="expected exactly one hand"):
        plugin.dispatch("get_state", {})
    with pytest.raises(RuntimeError, match="expected exactly one hand"):
        plugin.dispatch("release", {})
    accepted = plugin.dispatch("close", {"confirm_motion": True})
    assert accepted["state"] == "running"
    assert done.wait(2)
    assert events[-1][1] == "error"
    assert "expected exactly one hand" in events[-1][2]["message"]
    done.clear()
    for _ in range(100):
        if plugin._active is None:
            break
        threading.Event().wait(0.001)
    accepted = plugin.dispatch("set_positions", {"index_flex": 234, "confirm_motion": True})
    assert accepted["state"] == "running" and done.wait(2)
    assert events[-1][1] == "error"
    assert "expected exactly one hand" in events[-1][2]["message"]
    assert all(frame[1] == 0x04 for transport in transports for frame in transport.sent)


def test_info_recognizes_hot_swap_before_next_gesture(monkeypatch):
    responding_ids = {0x27}
    connection, transports = make_detecting_connection(monkeypatch, responding_ids)
    plugin = HandPlugin(connection, calibrated_config(auto_detect_gestures=True))
    assert plugin.dispatch("info", {})["side"] == "right"
    responding_ids.clear()
    assert plugin.dispatch("info", {})["side"] is None
    responding_ids.add(0x28)
    assert plugin.dispatch("info", {})["side"] == "left"
    assert all(frame[1] == 0x04 for transport in transports for frame in transport.sent)


def test_card_rejects_bad_structured_position_without_serial_access():
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    with pytest.raises(ValueError, match="expected 1 integer"):
        plugin.dispatch("set_positions", {"index_flex": 256, "confirm_motion": True})
    assert connection.transport.sent == []


@pytest.mark.parametrize("side,slave_id", [("right", 0x27), ("left", 0x28)])
def test_colleague_card_actions_keep_auto_detection_and_confirmation(monkeypatch, side, slave_id):
    connection, transports = make_detecting_connection(monkeypatch, {slave_id})
    plugin = HandPlugin(connection, calibrated_config(auto_detect_gestures=True, poll_interval_s=0.02))
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)

    with pytest.raises(ValueError, match="confirm_motion"):
        plugin.dispatch("set_positions", {"index_flex": 220, "hold_torque": False})
    assert all(frame[1] == 0x04 for transport in transports for frame in transport.sent)

    result = completed_motion(plugin, "set_positions", {
        "index_flex": 234, "speed_ratio": "1.5x",
        "hold_torque": False, "confirm_motion": True,
    }, events, done)
    assert result["side"] == side
    assert result["positions"]["index_root"] == 234
    assert result["speed"] == 90
    assert result["torque_released"] is True

    result = completed_motion(plugin, "set_positions", {
        "thumb_flex": 220, "index_flex": 210, "speed_ratio": "2.0x",
        "hold_torque": False, "confirm_motion": True,
    }, events, done)
    assert result["side"] == side
    assert result["positions"]["thumb_root"] == 220
    assert result["positions"]["index_root"] == 210
    assert result["positions"]["middle_root"] == RIGHT_OPEN_POSE[3]
    assert result["speed"] == 120 and result["torque_released"] is True
    assert all(frame[0] == slave_id for transport in transports for frame in transport.sent if frame[1] == 0x10)


def test_csv_and_invalid_options_fail_before_serial(monkeypatch):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config(poll_interval_s=0.02))
    for arguments in (
        {"speed_ratio": "4.0x"}, {"hold_torque": "maybe"},
        {"positions": ""}, {"index_flex": 256},
        {"positions": ",".join(map(str, RIGHT_OPEN_POSE)), "index_flex": 220},
    ):
        with pytest.raises(ValueError):
            plugin.dispatch("set_positions", {**arguments, "confirm_motion": True})
    assert connection.transport.sent == [] and connection._client is None

    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    result = completed_motion(plugin, "set_positions", {
        "index_flex": 220,
        "confirm_motion": True,
    }, events, done)
    assert result["positions"]["index_root"] == 220
    assert result["torque_released"] is False


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
    assert config["plugins"]["hand"]["open_pose"] == RIGHT_OPEN_POSE
    assert config["plugins"]["hand"]["auto_detect_gestures"] is True


def test_canvas_stop_before_connection_does_not_open_serial():
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    assert plugin.dispatch("stop", {}) == {"state": "idle", "torque_released": True}
    assert connection.transport.sent == []


def test_canvas_card_schema_and_structured_positions(monkeypatch):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config(poll_interval_s=0.02))
    card = plugin.get_tool()
    actions = card["inputSchema"]["properties"]["action"]["enum"]
    assert card["title"] == "L20 Lite 灵巧手"
    assert card["type"] == "actuator"
    assert "info" in actions and "release" in actions and "set_positions" in actions
    assert "set_speed" not in actions and "set_joint" not in actions
    assert card["inputSchema"]["properties"]["torque"]["default"] == 100
    assert "初始值 100" in card["inputSchema"]["properties"]["torque"]["description"]
    assert "初始值 100" in card["inputSchema"]["x-action-params"]["set_torque"]["description"]
    assert card["inputSchema"]["x-action-params"]["info"]["params"] == []
    assert "positions" not in card["inputSchema"]["properties"]
    assert "positions" not in card["inputSchema"]["x-action-params"]["set_positions"]["params"]
    assert card["inputSchema"]["properties"]["index_flex"]["type"] == "integer"
    assert card["inputSchema"]["properties"]["confirm_motion"] == {
        "type": "boolean", "const": True,
        "description": "确认手附近无人和障碍物后，每次运动显式输入 true",
    }
    assert card["inputSchema"]["x-is-dangerous"] is True
    assert plugin.PREFIX == "hand"
    assert card["inputSchema"]["x-completion"]["actions"] == ["open", "close", "set_positions"]
    assert card["inputSchema"]["allOf"][0]["then"]["required"] == ["confirm_motion"]
    for action in ("open", "close", "set_positions"):
        assert card["inputSchema"]["x-action-params"][action]["params"][-1] == "confirm_motion"
    for action in ("info", "get_state", "set_torque", "release"):
        assert "confirm_motion" not in card["inputSchema"]["x-action-params"][action]["params"]
    assert plugin.dispatch("info", {})["serial_port"] == "/dev/fake"
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    pose = [254, 254, 234, 254, 254, 254, 127, 126, 127, 254]
    result = completed_motion(plugin, "set_positions", {"index_flex": 234, "confirm_motion": True}, events, done)
    assert result["state"] == "completed"
    assert connection.client.get_positions() == pose
    assert plugin.dispatch("release", {})["state"] == "released"
    with pytest.raises(ValueError, match="CSV is no longer supported"):
        plugin.dispatch("set_positions", {"positions": "1,2,3", "confirm_motion": True})
    assert plugin.dispatch("set_speed", {"speed": 50}) is None
    assert plugin.dispatch("set_joint", {"joint": "index_root", "position": 100,
                                          "confirm_motion": True}) is None


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
    ("set_positions", {"index_flex": 100}),
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
