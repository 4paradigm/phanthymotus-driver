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
import hardware_smoke
from modbus_device import HandPlugin, JOINT_NAMES, RIGHT_FIST_POSE, RIGHT_OPEN_POSE
from common.vendor_runtime import DriverBundle, load_config


def calibrated_config(**overrides):
    return {"open_pose": list(RIGHT_OPEN_POSE),
            "fist_pose": list(RIGHT_FIST_POSE), **overrides}


def both_sides_config(**overrides):
    return calibrated_config(**{
        "left_open_pose": list(RIGHT_OPEN_POSE),
        "left_fist_pose": list(RIGHT_FIST_POSE),
        **overrides,
    })


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
        "pinky_root", "index_sway", "ring_sway", "pinky_sway", "thumb_roll",
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


def test_open_close_torque_is_per_action_and_reset_opens_then_releases(monkeypatch):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config(poll_interval_s=0.02))
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)

    closed = completed_motion(plugin, "close", {
        "set_torque": 80, "torque": 100, "confirm_motion": True,
    }, events, done)
    assert closed["torque"] == 80
    assert connection.client.get_torques() == [80] * 10
    opened = completed_motion(plugin, "open", {
        "set_torque": 120, "confirm_motion": True,
    }, events, done)
    assert opened["torque"] == 120
    assert connection.client.get_torques() == [120] * 10
    closed_again = completed_motion(plugin, "close", {
        "confirm_motion": True,
    }, events, done)
    assert closed_again["torque"] == 100
    assert connection.client.get_torques() == [100] * 10

    reset = completed_motion(plugin, "reset", {
        "confirm_motion": True, "torque": 80, "set_torque": 200,
    }, events, done)
    assert reset["action"] == "reset"
    assert reset["requested_positions"] == dict(zip(JOINT_NAMES, RIGHT_OPEN_POSE))
    assert reset["torque"] == 0 and reset["torque_released"] is True
    assert connection.client.get_positions() == RIGHT_OPEN_POSE
    assert connection.client.get_torques() == [0] * 10
    position_writes = [i for i, frame in enumerate(connection.transport.sent)
                       if frame[1] == 0x10 and frame[2:4] == b"\x00\x00"]
    zero_torque_writes = [i for i, frame in enumerate(connection.transport.sent)
                          if frame[1] == 0x10 and frame[2:4] == b"\x00\x0a"
                          and frame[7:-2] == bytes(20)]
    assert position_writes[-1] < zero_torque_writes[-1]


def test_reset_already_open_releases_torque_without_position_write():
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    events, done = capture_completions(plugin)
    reset = completed_motion(plugin, "reset", {"confirm_motion": True}, events, done)
    assert reset["state"] == "already_at_target"
    assert reset["torque_released"] is True
    assert connection.client.get_torques() == [0] * 10
    assert not any(frame[1] == 0x10 and frame[2:4] == b"\x00\x00"
                   for frame in connection.transport.sent)


def test_open_already_at_target_restores_default_or_explicit_torque():
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    events, done = capture_completions(plugin)
    released = completed_motion(plugin, "reset", {"confirm_motion": True}, events, done)
    assert released["state"] == "already_at_target"
    assert connection.client.get_torques() == [0] * 10

    opened = completed_motion(plugin, "open", {"confirm_motion": True}, events, done)
    assert opened["state"] == "already_at_target"
    assert opened["torque"] == 100 and opened["torque_released"] is False
    assert connection.client.get_torques() == [100] * 10
    assert not any(frame[1] == 0x10 and frame[2:4] == b"\x00\x00"
                   for frame in connection.transport.sent)

    opened = completed_motion(plugin, "open", {
        "set_torque": None, "confirm_motion": True,
    }, events, done)
    assert opened["state"] == "already_at_target"
    assert connection.client.get_torques() == [100] * 10
    assert not any(frame[1] == 0x10 and frame[2:4] == b"\x00\x00"
                   for frame in connection.transport.sent)

    opened = completed_motion(plugin, "open", {
        "set_torque": 85, "confirm_motion": True,
    }, events, done)
    assert opened["state"] == "already_at_target"
    assert opened["torque"] == 85
    assert connection.client.get_torques() == [85] * 10
    assert not any(frame[1] == 0x10 and frame[2:4] == b"\x00\x00"
                   for frame in connection.transport.sent)


def test_close_already_at_target_restores_torque_after_stop(monkeypatch):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config(poll_interval_s=0.02))
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    completed_motion(plugin, "close", {"confirm_motion": True}, events, done)
    assert plugin.dispatch("stop", {})["torque_released"] is True
    assert connection.client.get_torques() == [0] * 10
    closed = completed_motion(plugin, "close", {"confirm_motion": True}, events, done)
    assert closed["state"] == "already_at_target"
    assert closed["torque"] == 100 and closed["torque_released"] is False
    assert connection.client.get_torques() == [100] * 10


def test_per_action_torque_validation_rejects_bad_values_before_serial():
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    for action in ("open", "close"):
        for torque in (0, 256, True, "80"):
            with pytest.raises(ValueError, match="set_torque"):
                plugin.dispatch(action, {"set_torque": torque, "confirm_motion": True})
    with pytest.raises(ValueError, match="confirm_motion"):
        plugin.dispatch("reset", {})
    assert connection.transport.sent == []


@pytest.mark.parametrize("action,arguments", [
    ("open", {}), ("close", {}),
    ("set_positions", {"index_flex": 220}),
])
def test_removed_hold_torque_is_rejected_before_serial(action, arguments):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    with pytest.raises(ValueError, match="hold_torque is no longer supported"):
        plugin.dispatch(action, {**arguments, "hold_torque": False, "confirm_motion": True})
    assert connection._client is None
    assert connection.transport.sent == []


def test_set_positions_uses_official_ring_and_pinky_sway_slots(monkeypatch):
    assert modbus_device.JOINT_FIELDS["ring_sway"] == 7
    assert modbus_device.JOINT_FIELDS["pinky_sway"] == 8
    assert "middle_sway" not in modbus_device.JOINT_FIELDS
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config(poll_interval_s=0.02))
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    result = completed_motion(plugin, "set_positions", {
        "ring_sway": 200, "pinky_sway": 30, "confirm_motion": True,
    }, events, done)
    assert result["positions"]["ring_sway"] == 200
    assert result["positions"]["pinky_sway"] == 30
    assert connection.client.get_positions()[7:9] == [200, 30]


@pytest.mark.parametrize("side,slave_id", [("right", 0x27), ("left", 0x28)])
def test_auto_detected_hand_receives_all_device_actions(monkeypatch, side, slave_id):
    connection, transports = make_detecting_connection(monkeypatch, {slave_id})
    plugin = HandPlugin(connection, both_sides_config(auto_detect_gestures=True, poll_interval_s=0.02))
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
    reset = completed_motion(plugin, "reset", {"confirm_motion": True}, events, done)
    assert reset["side"] == side and reset["torque_released"] is True
    assert connection._client.get_positions() == RIGHT_OPEN_POSE
    assert connection._client.get_torques() == [0] * 10
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
    with pytest.raises(ValueError, match="confirm_motion"):
        plugin.dispatch("reset", {})
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
    done.clear()
    for _ in range(100):
        if plugin._active is None:
            break
        threading.Event().wait(0.001)
    accepted = plugin.dispatch("reset", {"confirm_motion": True})
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


def test_hardware_smoke_motion_uses_current_card_api_without_hardware(monkeypatch, capsys):
    connection, transports = make_detecting_connection(monkeypatch, {0x27})
    monkeypatch.setattr(hardware_smoke, "L20LiteConnection", lambda _config: connection)
    monkeypatch.delenv("CONFIG_PATH", raising=False)
    monkeypatch.setattr(sys, "argv", ["hardware_smoke.py", "--move-index-and-restore"])
    monkeypatch.setattr("modbus_device.time.sleep", lambda _seconds: None)

    assert hardware_smoke.main() == 0
    output = capsys.readouterr().out
    assert "move: completed" in output
    assert "restore: completed" in output
    assert connection._client is None
    writes = [frame for transport in transports for frame in transport.sent if frame[1] == 0x10]
    position_writes = [frame for frame in writes if frame[2:4] == b"\x00\x00"]
    assert len(position_writes) >= 4
    assert all(frame[0] == 0x27 for frame in writes)


def test_hardware_smoke_read_only_detects_left_without_writes(monkeypatch, capsys):
    connection, transports = make_detecting_connection(monkeypatch, {0x28})
    monkeypatch.setattr(hardware_smoke, "L20LiteConnection", lambda _config: connection)
    monkeypatch.delenv("CONFIG_PATH", raising=False)
    monkeypatch.setattr(sys, "argv", ["hardware_smoke.py", "--read-only"])

    assert hardware_smoke.main() == 0
    assert "side: left" in capsys.readouterr().out
    assert all(frame[1] == 0x04 for transport in transports for frame in transport.sent)


@pytest.mark.parametrize("value", [-1, 256, 1.5, True, "256"])
def test_card_rejects_bad_structured_position_without_serial_access(value):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config())
    with pytest.raises(ValueError, match=r"index_flex must be an integer in 0\.\.255"):
        plugin.dispatch("set_positions", {"index_flex": value, "confirm_motion": True})
    assert connection.transport.sent == [] and connection._client is None


@pytest.mark.parametrize("side,slave_id", [("right", 0x27), ("left", 0x28)])
def test_colleague_card_actions_keep_auto_detection_and_confirmation(monkeypatch, side, slave_id):
    connection, transports = make_detecting_connection(monkeypatch, {slave_id})
    plugin = HandPlugin(connection, both_sides_config(auto_detect_gestures=True, poll_interval_s=0.02))
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)

    with pytest.raises(ValueError, match="confirm_motion"):
        plugin.dispatch("set_positions", {"index_flex": 220})
    assert all(frame[1] == 0x04 for transport in transports for frame in transport.sent)

    result = completed_motion(plugin, "set_positions", {
        "index_flex": 234, "speed_ratio": "1.5x", "confirm_motion": True,
    }, events, done)
    assert result["side"] == side
    assert result["positions"]["index_root"] == 234
    assert result["speed"] == 90
    assert result["torque_released"] is False

    result = completed_motion(plugin, "set_positions", {
        "thumb_flex": 220, "index_flex": 210, "speed_ratio": "2.0x", "confirm_motion": True,
    }, events, done)
    assert result["side"] == side
    assert result["positions"]["thumb_root"] == 220
    assert result["positions"]["index_root"] == 210
    assert result["positions"]["middle_root"] == RIGHT_OPEN_POSE[3]
    assert result["speed"] == 120 and result["torque_released"] is False
    assert all(frame[0] == slave_id for transport in transports for frame in transport.sent if frame[1] == 0x10)


def test_csv_and_invalid_options_fail_before_serial(monkeypatch):
    connection = FakeConnection()
    plugin = HandPlugin(connection, calibrated_config(poll_interval_s=0.02))
    for arguments in (
        {"speed_ratio": "4.0x"}, {"hold_torque": False},
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
        HandPlugin(connection, calibrated_config())


@pytest.mark.parametrize("action,args", [
    ("open", {}), ("close", {}), ("reset", {}),
    ("set_positions", {"index_flex": 220}),
])
@pytest.mark.parametrize("left_pose_config", [
    {}, {"left_open_pose": list(RIGHT_OPEN_POSE)},
    {"left_fist_pose": list(RIGHT_FIST_POSE)},
])
def test_detected_left_without_complete_left_calibration_refuses_motion(
    monkeypatch, action, args, left_pose_config,
):
    connection, transports = make_detecting_connection(monkeypatch, {0x28})
    plugin = HandPlugin(connection, calibrated_config(
        auto_detect_gestures=True, **left_pose_config,
    ))
    events, done = capture_completions(plugin)
    assert plugin.dispatch("info", {})["side"] == "left"
    assert plugin.dispatch("get_state", {})["side"] == "left"
    accepted = plugin.dispatch(action, {**args, "confirm_motion": True})
    assert accepted["state"] == "running"
    assert done.wait(2)
    assert events[-1][0] == accepted["action_id"]
    assert events[-1][1] == "error"
    assert "left-hand motion requires explicit calibrated" in events[-1][2]["message"]
    assert all(frame[1] == 0x04 for transport in transports for frame in transport.sent)


def test_detected_left_uses_its_own_calibrated_poses(monkeypatch):
    connection, transports = make_detecting_connection(monkeypatch, {0x28})
    left_open = list(RIGHT_OPEN_POSE)
    left_fist = list(RIGHT_FIST_POSE)
    left_open[0] = 250
    left_fist[0] = 90
    plugin = HandPlugin(connection, both_sides_config(
        auto_detect_gestures=True, poll_interval_s=0.02,
        left_open_pose=left_open, left_fist_pose=left_fist,
    ))
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    closed = completed_motion(plugin, "close", {"confirm_motion": True}, events, done)
    assert closed["requested_positions"] == dict(zip(JOINT_NAMES, left_fist))
    opened = completed_motion(plugin, "open", {"confirm_motion": True}, events, done)
    assert opened["requested_positions"] == dict(zip(JOINT_NAMES, left_open))
    assert all(frame[0] == 0x28 for transport in transports for frame in transport.sent if frame[1] == 0x10)


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
    assert config["plugins"]["hand"]["left_fist_pose"] == RIGHT_FIST_POSE
    assert config["plugins"]["hand"]["left_open_pose"] == RIGHT_OPEN_POSE
    assert config["plugins"]["hand"]["auto_detect_gestures"] is True


def make_two_port_plugin(monkeypatch):
    responders = {"/dev/ttyUSB0": {0x27}, "/dev/ttyUSB1": {0x28}}
    transports = []

    def client_factory(port, side, **kwargs):
        transport = SelectiveFakeSerial(responders[port])
        transports.append((port, transport))
        return L20LiteModbusClient(port, side, baudrate=kwargs["baudrate"],
                                   timeout_s=kwargs["timeout_s"],
                                   serial_factory=lambda **_: transport)

    monkeypatch.setattr(modbus_device, "L20LiteModbusClient", client_factory)
    plugin = modbus_device.MultiHandPlugin(
        {"serial_port": "/dev/ttyUSB0", "side": "right", "baudrate": 115200},
        both_sides_config(auto_detect_gestures=True, poll_interval_s=0.02),
    )
    monkeypatch.setattr(plugin, "_ports", lambda: list(responders))
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    return plugin, responders, transports


def test_two_cards_bind_distinct_hands_and_route_motion(monkeypatch):
    plugin, _, transports = make_two_port_plugin(monkeypatch)
    card = plugin.get_tool()
    assert card["multiInstance"] is True
    assert card["configSchema"]["properties"]["bound_side"]["scope"] == "instance"
    assert card["configSchema"]["properties"]["bound_side"]["enum"] == ["left", "right"]
    unbound_info = plugin.dispatch("info", {"instance_id": "card-left"})
    assert unbound_info["detected_count"] == 2
    assert unbound_info["side"] is None
    assert set(unbound_info["detected_sides"]) == {"left", "right"}
    assert plugin.dispatch("config", {"instance_id": "card-left", "bound_side": "left"})["state"] == "configured"
    assert plugin.dispatch("config", {"instance_id": "card-right", "bound_side": "right"})["state"] == "configured"
    assert all(frame[1] == 0x04 for _, transport in transports for frame in transport.sent)
    with pytest.raises(ValueError, match="already bound"):
        plugin.dispatch("config", {"instance_id": "card-third", "bound_side": "left"})
    for instance_id, side, port in (("card-left", "left", "/dev/ttyUSB1"),
                                     ("card-right", "right", "/dev/ttyUSB0")):
        info = plugin.dispatch("info", {"instance_id": instance_id})
        assert info["bound_side"] == side
        assert info["serial_port"] == port
        assert info["detected_count"] == 2
        controller = plugin._controllers[instance_id]
        events, done = capture_completions(controller)
        accepted = plugin.dispatch("close", {"instance_id": instance_id,
                                             "confirm_motion": True})
        assert accepted["state"] == "running" and done.wait(2)
        assert events[-1][1] == "completed"
        writes = [(path, frame) for path, transport in transports
                  for frame in transport.sent if frame[1] == 0x10]
        assert all(frame[0] == (0x28 if path == "/dev/ttyUSB1" else 0x27)
                   for path, frame in writes)
    assert any(path == "/dev/ttyUSB0" and frame[1] == 0x10
               for path, transport in transports for frame in transport.sent)
    assert any(path == "/dev/ttyUSB1" and frame[1] == 0x10
               for path, transport in transports for frame in transport.sent)
    plugin.stop()


def test_bound_card_refuses_missing_hand_and_unbound_card_cannot_move(monkeypatch):
    plugin, responders, transports = make_two_port_plugin(monkeypatch)
    plugin.dispatch("config", {"instance_id": "left-card", "bound_side": "left"})
    with pytest.raises(RuntimeError, match="not bound"):
        plugin.dispatch("open", {"instance_id": "other-card", "confirm_motion": True})
    responders["/dev/ttyUSB1"].clear()
    assert plugin.dispatch("info", {"instance_id": "left-card"})["state"] == "offline"
    with pytest.raises(RuntimeError, match="not connected"):
        plugin.dispatch("close", {"instance_id": "left-card", "confirm_motion": True})
    assert all(frame[1] == 0x04 for _, transport in transports for frame in transport.sent)


def test_stopped_card_can_be_replaced_without_duplicate_side_control(monkeypatch):
    plugin, _, _ = make_two_port_plugin(monkeypatch)
    plugin.dispatch("config", {"instance_id": "old", "bound_side": "left"})
    assert plugin.dispatch("get_state", {"instance_id": "old"})["side"] == "left"
    with pytest.raises(ValueError, match="already bound"):
        plugin.dispatch("config", {"instance_id": "new", "bound_side": "left"})
    assert plugin.dispatch("stop", {"instance_id": "old"})["state"] == "idle"
    assert plugin.dispatch("config", {"instance_id": "new", "bound_side": "left"})["state"] == "configured"
    with pytest.raises(RuntimeError, match="not bound"):
        plugin.dispatch("get_state", {"instance_id": "old"})
    assert plugin.dispatch("get_state", {"instance_id": "new"})["side"] == "left"


def test_removed_offline_card_without_controller_releases_its_binding(monkeypatch):
    plugin, responders, transports = make_two_port_plugin(monkeypatch)
    plugin.dispatch("config", {"instance_id": "old", "bound_side": "left"})
    responders["/dev/ttyUSB1"].clear()
    with pytest.raises(RuntimeError, match="not connected"):
        plugin.dispatch("start", {"instance_id": "old"})
    assert "old" not in plugin._controllers
    with pytest.raises(ValueError, match="already bound"):
        plugin.dispatch("config", {"instance_id": "new", "bound_side": "left"})
    # Canvas removal sends stop even though startup never created a client.
    assert plugin.dispatch("stop", {"instance_id": "old"})["state"] == "idle"
    assert plugin.dispatch("config", {"instance_id": "new", "bound_side": "left"})["state"] == "configured"
    responders["/dev/ttyUSB1"].add(0x28)
    assert plugin.dispatch("get_state", {"instance_id": "new"})["side"] == "left"
    assert all(frame[1] == 0x04 for _, transport in transports for frame in transport.sent)


def test_stopped_binding_reactivated_by_same_card_cannot_be_stolen(monkeypatch):
    plugin, _, _ = make_two_port_plugin(monkeypatch)
    plugin.dispatch("config", {"instance_id": "left", "bound_side": "left"})
    plugin.dispatch("stop", {"instance_id": "left"})
    assert plugin.dispatch("start", {"instance_id": "left"})["state"] == "ready"
    with pytest.raises(ValueError, match="already bound"):
        plugin.dispatch("config", {"instance_id": "other", "bound_side": "left"})


def test_bound_card_follows_side_after_usb_port_swap(monkeypatch):
    plugin, responders, transports = make_two_port_plugin(monkeypatch)
    plugin.dispatch("config", {"instance_id": "left-card", "bound_side": "left"})
    first = plugin.dispatch("info", {"instance_id": "left-card"})
    assert first["serial_port"] == "/dev/ttyUSB1"
    # Exercise a previously opened client, not merely a cached info result.
    assert plugin.dispatch("get_state", {"instance_id": "left-card"})["side"] == "left"
    responders["/dev/ttyUSB0"].clear()
    responders["/dev/ttyUSB0"].add(0x28)
    responders["/dev/ttyUSB1"].clear()
    responders["/dev/ttyUSB1"].add(0x27)
    second = plugin.dispatch("info", {"instance_id": "left-card"})
    assert second["serial_port"] == "/dev/ttyUSB0"
    controller = plugin._controllers["left-card"]
    events, done = capture_completions(controller)
    accepted = plugin.dispatch("open", {"instance_id": "left-card", "confirm_motion": True})
    assert accepted["state"] == "running" and done.wait(2)
    assert events[-1][1] == "completed"
    assert all(path == "/dev/ttyUSB0" and frame[0] == 0x28
               for path, transport in transports for frame in transport.sent
               if frame[1] == 0x10)


def test_bound_motion_rechecks_slave_before_any_write(monkeypatch):
    plugin, responders, transports = make_two_port_plugin(monkeypatch)
    plugin.dispatch("config", {"instance_id": "left-card", "bound_side": "left"})
    assert plugin.dispatch("info", {"instance_id": "left-card"})["serial_port"] == "/dev/ttyUSB1"
    controller = plugin._controllers["left-card"]
    events, done = capture_completions(controller)
    # Simulate unplug/replug after read-only discovery but before worker read.
    real_controller = plugin._controller

    def swapped_after_selection(instance_id, side, port):
        selected = real_controller(instance_id, side, port)
        responders[port].clear()
        responders[port].add(0x27)
        return selected

    monkeypatch.setattr(plugin, "_controller", swapped_after_selection)
    accepted = plugin.dispatch("close", {"instance_id": "left-card", "confirm_motion": True})
    assert accepted["state"] == "running" and done.wait(2)
    assert events[-1][1] == "error"
    assert all(frame[1] == 0x04 for _, transport in transports for frame in transport.sent)


def test_unbound_legacy_card_still_works_but_second_unbound_card_is_rejected(monkeypatch):
    connection, transports = make_detecting_connection(monkeypatch, {0x27})
    plugin = modbus_device.MultiHandPlugin(
        {"serial_port": "/dev/fake", "side": "right"},
        both_sides_config(auto_detect_gestures=True),
    )
    # The legacy path uses the same configured serial port and old sole-hand
    # auto-detection; a second canvas instance cannot silently share it.
    assert plugin.dispatch("start", {"instance_id": "first"}) == {"state": "ready"}
    assert plugin.dispatch("get_state", {"instance_id": "first"})["side"] == "right"
    events, done = capture_completions(plugin._legacy)
    accepted = plugin.dispatch("open", {"instance_id": "first", "confirm_motion": True})
    assert accepted["state"] == "running" and done.wait(2)
    assert events[-1][1] == "completed"
    with pytest.raises(RuntimeError, match="another unbound card"):
        plugin.dispatch("start", {"instance_id": "second"})
    with pytest.raises(RuntimeError, match="another unbound card"):
        plugin.dispatch("close", {"instance_id": "second", "confirm_motion": True})
    assert all(frame[0] == 0x27 for transport in transports for frame in transport.sent
               if frame[1] == 0x10)
    assert plugin.dispatch("stop", {"instance_id": "first"})["state"] == "idle"
    assert plugin.dispatch("start", {"instance_id": "second"}) == {"state": "ready"}


def test_build_plugins_exposes_one_multi_instance_hand_tool():
    config = load_config(str(DRIVER / "main.py"))
    config["plugins"]["hand_gesture"]["enabled"] = False
    plugins = modbus_device.build_plugins(config, "", None)
    assert len(plugins) == 1
    tools = DriverBundle(plugins).get_all_tools()
    assert len(tools) == 1
    assert tools[0]["name"] == "hand"
    assert tools[0]["multiInstance"] is True


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
    assert "info" in actions and "reset" in actions and "set_positions" in actions
    assert "set_speed" not in actions and "set_joint" not in actions
    assert "set_torque" not in actions and "release" not in actions
    assert card["inputSchema"]["properties"]["set_torque"]["default"] == 100
    assert card["inputSchema"]["properties"]["set_torque"]["title"] == "SET_TORQUE"
    assert (card["inputSchema"]["properties"]["set_torque"]["minimum"],
            card["inputSchema"]["properties"]["set_torque"]["maximum"]) == (1, 255)
    assert "整数 1–255" in card["inputSchema"]["properties"]["set_torque"]["description"]
    assert "本次 open 或 close 运动的 10 个关节扭矩上限" in card["inputSchema"]["properties"]["set_torque"]["description"]
    assert "torque" not in card["inputSchema"]["properties"]
    assert "hold_torque" not in card["inputSchema"]["properties"]
    assert "到位后保留本次力矩" in card["inputSchema"]["x-action-params"]["open"]["description"]
    assert "到位后保留本次力矩" in card["inputSchema"]["x-action-params"]["close"]["description"]
    for action in ("open", "close", "set_positions"):
        assert "hold_torque" not in card["inputSchema"]["x-action-params"][action]["params"]
    assert "set_torque" in card["inputSchema"]["x-action-params"]["open"]["params"]
    assert "set_torque" in card["inputSchema"]["x-action-params"]["close"]["params"]
    assert "set_torque" not in card["inputSchema"]["x-action-params"]["set_positions"]["params"]
    assert "越界会报错且不发送动作" in card["inputSchema"]["x-action-params"]["set_positions"]["description"]
    assert "布尔值 confirm_motion=true" in card["inputSchema"]["x-action-params"]["set_positions"]["description"]
    assert card["inputSchema"]["x-action-params"]["reset"]["params"] == ["confirm_motion"]
    assert card["inputSchema"]["x-action-params"]["info"]["params"] == []
    assert "positions" not in card["inputSchema"]["properties"]
    assert "positions" not in card["inputSchema"]["x-action-params"]["set_positions"]["params"]
    assert card["inputSchema"]["properties"]["index_flex"]["type"] == "integer"
    assert len(modbus_device.STRUCTURED_POSITION_FIELDS) == 10
    for field in modbus_device.STRUCTURED_POSITION_FIELDS:
        property_schema = card["inputSchema"]["properties"][field]
        assert property_schema["type"] == "integer"
        assert (property_schema["minimum"], property_schema["maximum"]) == (0, 255)
        assert "0–255" in property_schema["description"]
    assert "无名指侧摆" in card["inputSchema"]["properties"]["ring_sway"]["description"]
    assert "小指侧摆" in card["inputSchema"]["properties"]["pinky_sway"]["description"]
    assert "middle_sway" not in card["inputSchema"]["properties"]
    assert card["inputSchema"]["properties"]["confirm_motion"] == {
        "type": "boolean", "const": True,
        "description": "确认手附近无人和障碍物后，每次请求传入布尔值 true（不是字符串）；若画布保留上次勾选，执行前仍须重新检查",
    }
    assert card["inputSchema"]["x-is-dangerous"] is True
    assert plugin.PREFIX == "hand"
    assert card["inputSchema"]["x-completion"]["actions"] == ["open", "close", "reset", "set_positions"]
    assert card["inputSchema"]["allOf"][0]["then"]["required"] == ["confirm_motion"]
    for action in ("open", "close", "reset", "set_positions"):
        assert card["inputSchema"]["x-action-params"][action]["params"][-1] == "confirm_motion"
    for action in ("info", "get_state"):
        assert "confirm_motion" not in card["inputSchema"]["x-action-params"][action]["params"]
    assert plugin.dispatch("info", {})["serial_port"] == "/dev/fake"
    events, done = capture_completions(plugin)
    monkeypatch.setattr("modbus_device.time.sleep", lambda _: None)
    pose = [254, 254, 234, 254, 254, 254, 127, 126, 127, 254]
    result = completed_motion(plugin, "set_positions", {"index_flex": 234, "confirm_motion": True}, events, done)
    assert result["state"] == "completed"
    assert connection.client.get_positions() == pose
    reset = completed_motion(plugin, "reset", {"confirm_motion": True}, events, done)
    assert reset["state"] == "completed" and reset["torque_released"] is True
    assert connection.client.get_positions() == RIGHT_OPEN_POSE
    assert connection.client.get_torques() == [0] * 10
    assert plugin.dispatch("release", {}) is None
    assert plugin.dispatch("set_torque", {"torque": 80}) is None
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
