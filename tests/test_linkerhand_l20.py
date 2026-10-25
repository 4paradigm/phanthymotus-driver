"""Comprehensive unit tests for LinkerHand L20 CAN transport and three canvas cards.

Covers:
  - Vector and naming conversions (16 active joints <-> 20-element CAN frames)
  - L20CanClient mock CAN send/receive and tactile frame parsing
  - HandPlugin (16-joint position control, safety gate, explicit left/right selection)
  - GesturePlugin (predefined gestures, clearance logic, counting)
  - GraspPlugin (position-only grasp, persistent motor holding, release)
"""

from __future__ import annotations

import queue
import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))

from can_client import (
    ACTIVE_JOINT_COUNT,
    VECTOR_JOINT_COUNT,
    FINGER_COUNT,
    RIGHT_HAND_CAN_ID,
    LEFT_HAND_CAN_ID,
    JOINT_NAMES_16,
    FT_PITCH_R,
    FT_YAW_R,
    FT_ROLL_R,
    FT_TIP_R,
    FT_PITCH_NR,
    FT_YAW_NR,
    FT_ROLL_NR,
    FT_TIP_NR,
    FT_NORMAL_FORCE,
    FT_TOUCH_B1,
    L20CanClient,
    validate_position_vector,
    vector20_to_named16,
    named16_to_vector20,
)
from hand_device import HandPlugin, _SingleHandController, build_plugins, validate_hold_torque
from gesture_device import GesturePlugin
from grasp_device import GraspPlugin

# Expected directions from the user's real-hand collision report, including
# the previously verified 0<->4 fix. This is independent of the driver's table.
THREE_STAGE_PAIRS = [
    (0, 4), (4, 0), (0, 3), (3, 0), (0, 7), (7, 0),
    (4, 1), (1, 4), (1, 3), (3, 1), (7, 1), (7, 2),
    (6, 3), (8, 3), (9, 3), (6, 4), (8, 4), (9, 4),
]
THUMB_RELEASE_PAIRS = [(7, 0), (7, 1), (7, 2)]


# ──────────────────────────────────────────────────────────────────────
# Mock CAN Bus for offline testing (Zero-Latency Queue)
# ──────────────────────────────────────────────────────────────────────

class MockCanMessage:
    def __init__(self, arbitration_id: int, data: bytes | list[int]):
        self.arbitration_id = arbitration_id
        self.data = bytearray(data)


class MockCanBus:
    def __init__(self, channel: str = "can0", baudrate: int = 1000000, can_id: int = 0x28):
        self.channel = channel
        self.baudrate = baudrate
        self.can_id = can_id
        self.sent_messages: list[MockCanMessage] = []
        self._inbox: queue.Queue = queue.Queue()
        self._closed = False
        self._lock = threading.Lock()

        # Simulated joint positions
        self.sim_pitch = [254, 254, 254, 254, 254]
        self.sim_yaw   = [128, 128, 128, 128, 128]
        self.sim_roll  = [254, 0, 0, 0, 0]
        self.sim_tip   = [254, 254, 254, 254, 254]

        # Simulated tactile
        self.sim_touch = [50, 60, 55, 65, 45]
        self.sim_normal_force = [12.0, 15.0, 14.0, 16.0, 10.0]

    def send(self, msg):
        with self._lock:
            self.sent_messages.append(msg)
            data = list(msg.data)
            if not data:
                return
            ft = data[0]
            payload = data[1:]

            # Respond immediately to read queries
            if ft == FT_PITCH_R:
                self._inbox.put(MockCanMessage(self.can_id, [FT_PITCH_R] + self.sim_pitch))
            elif ft == FT_YAW_R:
                self._inbox.put(MockCanMessage(self.can_id, [FT_YAW_R] + self.sim_yaw))
            elif ft == FT_ROLL_R:
                self._inbox.put(MockCanMessage(self.can_id, [FT_ROLL_R] + self.sim_roll))
            elif ft == FT_TIP_R:
                self._inbox.put(MockCanMessage(self.can_id, [FT_TIP_R] + self.sim_tip))
            elif ft == 0x41 and not payload:
                self._inbox.put(MockCanMessage(self.can_id, [0x41, self.sim_roll[0], self.sim_yaw[0], self.sim_pitch[0], 0, 0, self.sim_tip[0]]))
            elif ft == 0x42 and not payload:
                self._inbox.put(MockCanMessage(self.can_id, [0x42, self.sim_yaw[1], 0, self.sim_pitch[1], 0, 0, self.sim_tip[1]]))
            elif ft == 0x43 and not payload:
                self._inbox.put(MockCanMessage(self.can_id, [0x43, self.sim_yaw[2], 0, self.sim_pitch[2], 0, 0, self.sim_tip[2]]))
            elif ft == 0x44 and not payload:
                self._inbox.put(MockCanMessage(self.can_id, [0x44, self.sim_yaw[3], 0, self.sim_pitch[3], 0, 0, self.sim_tip[3]]))
            elif ft == 0x45 and not payload:
                self._inbox.put(MockCanMessage(self.can_id, [0x45, self.sim_yaw[4], 0, self.sim_pitch[4], 0, 0, self.sim_tip[4]]))
            elif ft == FT_NORMAL_FORCE:
                self._inbox.put(MockCanMessage(self.can_id, [FT_NORMAL_FORCE] + [int(v) for v in self.sim_normal_force]))
            elif FT_TOUCH_B1 <= ft <= FT_TOUCH_B1 + 4:
                idx = ft - FT_TOUCH_B1
                self._inbox.put(MockCanMessage(self.can_id, [ft, 1, self.sim_touch[idx]]))

            # Handle writes
            elif ft == FT_PITCH_NR and len(payload) >= 5:
                self.sim_pitch = list(payload[:5])
            elif ft == FT_YAW_NR and len(payload) >= 5:
                self.sim_yaw = list(payload[:5])
            elif ft == FT_ROLL_NR and len(payload) >= 5:
                self.sim_roll = list(payload[:5])
            elif ft == FT_TIP_NR and len(payload) >= 5:
                self.sim_tip = list(payload[:5])
            elif ft == 0x41 and len(payload) >= 6:
                self.sim_roll[0]  = payload[0]
                self.sim_yaw[0]   = payload[1]
                self.sim_pitch[0] = payload[2]
                self.sim_tip[0]   = payload[5]
            elif ft == 0x42 and len(payload) >= 6:
                self.sim_yaw[1]   = payload[0]
                self.sim_pitch[1] = payload[2]
                self.sim_tip[1]   = payload[5]
            elif ft == 0x43 and len(payload) >= 6:
                self.sim_yaw[2]   = payload[0]
                self.sim_pitch[2] = payload[2]
                self.sim_tip[2]   = payload[5]
            elif ft == 0x44 and len(payload) >= 6:
                self.sim_yaw[3]   = payload[0]
                self.sim_pitch[3] = payload[2]
                self.sim_tip[3]   = payload[5]
            elif ft == 0x45 and len(payload) >= 6:
                self.sim_yaw[4]   = payload[0]
                self.sim_pitch[4] = payload[2]
                self.sim_tip[4]   = payload[5]

    def recv(self, timeout: float = 0.05) -> MockCanMessage | None:
        try:
            return self._inbox.get(timeout=timeout)
        except queue.Empty:
            return None

    def shutdown(self):
        self._closed = True


def make_mock_client(side="right", can_id=0x28):
    mock_bus = MockCanBus(can_id=can_id)
    client = L20CanClient(
        can_channel="can0",
        side=side,
        can_id=can_id,
        recv_timeout_s=0.05,
        bus_factory=lambda ch, br, cid: mock_bus,
    )
    return client, mock_bus


# ──────────────────────────────────────────────────────────────────────
# Unit Tests: Protocol & Vector Mappings
# ──────────────────────────────────────────────────────────────────────

def test_joint_conversions():
    vec20 = [
        10, 11, 12, 13, 14,      # pitch
        20, 21, 22, 23, 24,      # yaw
        30, 0, 0, 0, 0,          # roll
        40, 41, 42, 43, 44,      # tip
    ]
    named = vector20_to_named16(vec20)
    assert len(named) == 16
    assert named["thumb_pitch"] == 10
    assert named["index_pitch"] == 11
    assert named["thumb_yaw"] == 20
    assert named["thumb_roll"] == 30
    assert named["thumb_tip"] == 40
    assert named["pinky_tip"] == 44

    restored = named16_to_vector20(named)
    assert restored == vec20


def test_validate_position_vector():
    lst16 = [10] * 16
    vec20 = validate_position_vector(lst16)
    assert len(vec20) == 20
    assert vec20[0] == 10
    assert vec20[10] == 10  # thumb roll
    assert vec20[11:15] == [0, 0, 0, 0]

    lst20 = [5] * 20
    assert validate_position_vector(lst20) == lst20

    d = {"thumb_pitch": 50, "index_pitch": 60}
    vec = validate_position_vector(d)
    assert vec[0] == 50
    assert vec[1] == 60


# ──────────────────────────────────────────────────────────────────────
# Unit Tests: L20CanClient
# ──────────────────────────────────────────────────────────────────────

def test_can_client_motion_and_query():
    client, mock_bus = make_mock_client()
    try:
        new_vec = [100] * 5 + [128] * 5 + [200, 0, 0, 0, 0] + [50] * 5
        client.set_joint_positions(new_vec)

        assert mock_bus.sim_pitch == [100] * 5
        assert mock_bus.sim_tip == [50] * 5

        time.sleep(0.05)
        pos = client.get_positions()
        assert pos["thumb_pitch"] == 100
        assert pos["thumb_tip"] == 50

        client.request_tactile_scalar()
        client.request_normal_force()
        time.sleep(0.05)
        snap = client.get_tactile_snapshot()
        assert snap["touch_scalar"]["thumb"] == 50
        assert snap["normal_force"]["thumb"] == 12.0
    finally:
        client.close()


# ──────────────────────────────────────────────────────────────────────
# Unit Tests: HandPlugin (Card 1)
# ──────────────────────────────────────────────────────────────────────

def test_hand_plugin_lifecycle():
    config = {"baudrate": 1000000, "right_can_id": 0x28, "left_can_id": 0x27}
    plugin = HandPlugin(config, {"enabled": True, "motion_speed": 70, "motion_torque": 120})
    tool_def = plugin.get_tool()
    assert tool_def["name"] == "hand"
    assert tool_def["multiInstance"] is True
    assert "open" in tool_def["inputSchema"]["x-action-params"]
    assert "set_positions" in tool_def["inputSchema"]["x-action-params"]

    # Shared adapter configuration needs no card identity or left/right binding.
    res = plugin.dispatch("config", {"can_channel": "can0"})
    assert res["state"] == "configured" and res["binding_saved"] is False
    assert res["can_channel"] == "can0"
    for args in ({"bound_side": "right"}, {"bound_side": "right", "confirm_motion": False}):
        with pytest.raises(ValueError, match="confirm_motion"):
            plugin.dispatch("open", args)


def test_hand_plugin_motion_execution():
    config = {"baudrate": 1000000}
    hand_config = {"enabled": True, "motion_speed": 70, "motion_torque": 120, "motion_timeout_s": 0.5, "poll_interval_s": 0.01}
    plugin = HandPlugin(config, hand_config)

    client, mock_bus = make_mock_client()
    ctrl = MagicMock()
    ctrl.channel = "can0"
    ctrl._motion_lock = threading.RLock()
    ctrl.get.return_value = client
    ctrl.side = "right"
    ctrl._stop_evt = threading.Event()
    ctrl.target_for_action.return_value = [200] * 20
    ctrl.move.return_value = {
        "state": "completed", "action": "open", "side": "right",
        "positions": vector20_to_named16([200] * 20),
    }

    plugin.get_controller_for_side = lambda *_: ctrl

    callback_calls = []
    plugin._acp_callback = lambda aid, status, res: callback_calls.append((aid, status, res))

    try:
        res = plugin.dispatch("open", {"instance_id": "inst_1", "bound_side": "right", "confirm_motion": True})
        assert res["state"] == "running"
        action_id = res["action_id"]

        # Wait briefly for execution
        time.sleep(0.1)
        assert len(callback_calls) == 1
        assert callback_calls[0][0] == action_id
        assert callback_calls[0][1] == "completed"
    finally:
        client.close()


# ──────────────────────────────────────────────────────────────────────
# Unit Tests: GesturePlugin (Card 2)
# ──────────────────────────────────────────────────────────────────────

def test_gesture_plugin_ok_gesture():
    hand_plugin = MagicMock()
    hand_plugin.config = {}
    client, mock_bus = make_mock_client()
    ctrl = MagicMock()
    ctrl.channel = "can0"
    ctrl._motion_lock = threading.RLock()
    ctrl.get.return_value = client
    ctrl.side = "right"
    hand_plugin.get_controller_for_side.return_value = ctrl

    config = {
        "motion_speed": 85,
        "count_speed": 85,
        "motion_timeout_s": 0.5,
        "poll_interval_s": 0.01,
    }
    gesture = GesturePlugin(hand_plugin, config)
    tool_def = gesture.get_tool()
    assert tool_def["name"] == "hand_gesture"
    assert "ok_gesture" in tool_def["inputSchema"]["x-action-params"]
    assert "count" in tool_def["inputSchema"]["x-action-params"]
    assert "number" in tool_def["inputSchema"]["x-action-params"]

    # Check confirm motion gate
    with pytest.raises(ValueError, match="confirm_motion"):
        gesture.dispatch("ok_gesture", {"instance_id": "inst_g", "bound_side": "right"})

    # Mock ACP
    calls = []
    gesture._acp_callback = lambda aid, status, res: calls.append((aid, status, res))

    try:
        res = gesture.dispatch("ok_gesture", {"instance_id": "inst_g", "bound_side": "right", "confirm_motion": True})
        assert res["state"] == "running"
        time.sleep(0.2)
        assert len(calls) == 1
        assert calls[0][1] == "completed"
    finally:
        client.close()


# ──────────────────────────────────────────────────────────────────────
# Unit Tests: GraspPlugin (Card 3 - Position Control)
# ──────────────────────────────────────────────────────────────────────

def test_grasp_plugin_position_control():
    hand_plugin = MagicMock()
    hand_plugin.config = {}
    client, mock_bus = make_mock_client()
    ctrl = _SingleHandController({}, 'right', 'can0', threading.RLock(), can_id=0x28)
    ctrl._client = client
    hand_plugin.get_controller_for_side.return_value = ctrl
    grasp = GraspPlugin(hand_plugin, {'approach_speed': 35, 'poll_interval_s': 0.01})
    definition = grasp.get_tool()
    assert definition['name'] == 'hand_grasp'
    assert set(definition['inputSchema']['x-action-params']) == {'side_grasp', 'top_grasp', 'release'}
    assert grasp.dispatch('info', {'instance_id': 'inst_grasp', "bound_side": "right"})['state'] == 'ready'
    calls, done = [], threading.Event()
    def callback(aid, status, result):
        calls.append((aid, status, result))
        done.set()
    grasp._acp_callback = callback
    try:
        result = grasp.dispatch('side_grasp', {'wait_for_result': False,
            'instance_id': 'inst_grasp', "bound_side": "right", 'confirm_motion': True})
        assert result['state'] == 'running' and done.wait(2)
        assert len(calls) == 1 and calls[0][1] == 'completed'
        assert calls[0][2]['command_sent'] and calls[0][2]['holding']
        assert mock_bus.sim_tip == [0] * 5
        assert not grasp._active
    finally:
        grasp.stop()
        client.close()


def test_position_grasp_does_not_depend_on_tactile_values():
    hand_plugin = MagicMock()
    hand_plugin.config = {}
    client, mock_bus = make_mock_client()
    ctrl = _SingleHandController({}, 'right', 'can0', threading.RLock(), can_id=0x28)
    ctrl._client = client
    hand_plugin.get_controller_for_side.return_value = ctrl
    grasp = GraspPlugin(hand_plugin, {})
    mock_bus.sim_touch = [255] * 5
    client.get_fresh_tactile = MagicMock(side_effect=AssertionError('unexpected tactile read'))
    try:
        result = grasp.dispatch('side_grasp', {'instance_id': 'inst_position', "bound_side": "right", 'confirm_motion': True})
        assert result['success'] and result['command_sent']
        client.get_fresh_tactile.assert_not_called()
        assert mock_bus.sim_tip == [0] * 5
    finally:
        grasp.stop()
        client.close()


def test_build_plugins_all_three_cards():
    config = {
        "plugins": {
            "hand": {"enabled": True},
            "hand_gesture": {"enabled": True},
            "hand_grasp": {"enabled": True},
        }
    }
    plugins = build_plugins(config, "", None)
    assert len(plugins) == 3
    names = [p.get_tool()["name"] for p in plugins]
    assert names == ["hand", "hand_gesture", "hand_grasp"]


def test_all_plugins_start_action():
    config = {"baudrate": 1000000, "side": "left"}
    hand_p = HandPlugin(config, {"enabled": True})
    gesture_p = GesturePlugin(hand_p, {"motion_speed": 85}, global_config=config)
    grasp_p = GraspPlugin(hand_p, {}, global_config=config)
    for card in (hand_p, gesture_p, grasp_p):
        # Project lifecycle startup must not require a device selection or CAN.
        assert card.dispatch("start", {"instance_id": "canvas-card"})["state"] == "ready"
        result = card.dispatch("stop", {"instance_id": "canvas-card"})
        assert result["state"] in ("idle", "unmatched")
        assert not card._active


def test_gesture_and_grasp_standalone_without_hand():
    """State inspection and startup work with the basic hand card disabled."""
    config = {"side": "left", "can_channel": "can0", "left_can_id": 0x28, "right_can_id": 0x27}
    gesture_p = GesturePlugin(None, {"motion_speed": 85}, global_config=config)
    grasp_p = GraspPlugin(None, {}, global_config=config)
    for card in (gesture_p, grasp_p):
        assert card.dispatch("start", {})["state"] == "ready"
        for side in ("left", "right"):
            info = card.dispatch("info", {"bound_side": side})
            assert info["state"] == "ready"
            assert info["bound_side"] == side
        assert not card._controllers


def test_gesture_plugin_count_collision_avoidance():
    client, mock_bus = make_mock_client()
    ctrl = MagicMock()
    ctrl.channel = "can0"
    ctrl._motion_lock = threading.RLock()
    ctrl.get.return_value = client
    ctrl.side = "left"
    hand_plugin = MagicMock()
    hand_plugin.config = {}
    hand_plugin.get_controller_for_side.return_value = ctrl

    config = {
        "motion_speed": 85,
        "count_speed": 85,
        "motion_timeout_s": 0.5,
        "poll_interval_s": 0.01,
    }
    gesture = GesturePlugin(hand_plugin, config)

    done = threading.Event()
    results = []

    def cb(aid, status, res):
        results.append((aid, status, res))
        done.set()

    gesture._acp_callback = cb

    try:
        res = gesture.dispatch("count", {
            "instance_id": "inst_count", "bound_side": "left",
            "start_num": 1,
            "end_num": 3,
            "hold_time_s": 0.1,
            "confirm_motion": True,
        })
        assert res["state"] == "running"
        assert done.wait(5.0), "Count completion callback not received"
        assert len(results) == 1
        assert results[0][1] == "completed"
        assert results[0][2]["final_number"] == 3
    finally:
        client.close()


def test_gesture_plugin_number_action():
    client, mock_bus = make_mock_client()
    ctrl = MagicMock()
    ctrl.channel = "can0"
    ctrl._motion_lock = threading.RLock()
    ctrl.get.return_value = client
    ctrl.side = "left"
    hand_plugin = MagicMock()
    hand_plugin.config = {}
    hand_plugin.get_controller_for_side.return_value = ctrl

    config = {
        "motion_speed": 85,
        "count_speed": 85,
        "motion_timeout_s": 0.5,
        "poll_interval_s": 0.01,
    }
    gesture = GesturePlugin(hand_plugin, config)

    done = threading.Event()
    results = []

    def cb(aid, status, res):
        results.append((aid, status, res))
        done.set()

    gesture._acp_callback = cb

    try:
        # 1. Trigger number 2
        res = gesture.dispatch("number", {
            "instance_id": "inst_num", "bound_side": "left",
            "number": 2,
            "confirm_motion": True,
        })
        assert res["state"] == "running"
        assert done.wait(5.0), "Number completion callback not received"
        assert len(results) == 1
        assert results[0][1] == "completed"
        assert results[0][2]["number"] == 2
        assert results[0][2]["action"] == "number"

        # 2. Transition from number 2 to number 3
        done.clear()
        res2 = gesture.dispatch("number", {
            "instance_id": "inst_num", "bound_side": "left",
            "number": 3,
            "confirm_motion": True,
        })
        assert res2["state"] == "running"
        assert done.wait(5.0)
        assert len(results) == 2
        assert results[1][2]["number"] == 3
    finally:
        client.close()


@pytest.mark.parametrize("from_num", [*range(10), "natural_gesture", "ok_gesture"])
@pytest.mark.parametrize("target_num", range(10))
def test_number_commands_only_selected_digit(from_num, target_num, monkeypatch):
    gesture = GesturePlugin(None, {})
    poses = gesture._calibration["left"]
    numbers = poses["count"]
    current = list(numbers[from_num] if isinstance(from_num, int) else poses[from_num])
    source = list(current)
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        current[:] = target
        commands.append(list(target))

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    # The actual hand may have moved through count or another card since the
    # last number call. A remembered digit must not control the next transition.
    gesture._current_number["left"] = (target_num + 5) % 10
    result = gesture._number(client, "left", poses, {"number": target_num, "speed": 85})

    assert result["number"] == target_num
    assert vector20_to_named16(commands[-1]) == vector20_to_named16(numbers[target_num])
    thumb_last = (from_num, target_num) in THREE_STAGE_PAIRS
    release_first = (from_num, target_num) in THUMB_RELEASE_PAIRS
    assert len(commands) <= (4 if release_first else 3 if thumb_last else 2), "number must not perform a counting sequence"
    # Every intermediate command may move only the thumb. Depending on the
    # unchanged finger posture, that can coincide with digit 5 or the source.
    other = (1, 2, 3, 4, 6, 7, 8, 9, 16, 17, 18, 19)
    for stage, command in enumerate(commands[:-1]):
        if thumb_last and stage == (2 if release_first else 1):
            assert all(command[i] == numbers[target_num][i] for i in other)
            assert all(command[i] == numbers[5][i] for i in (0, 5, 10, 15))
        else:
            assert all(command[i] == source[i] for i in other)


def test_number_retains_thumb_clearance_between_adjacent_digits(monkeypatch):
    gesture = GesturePlugin(None, {})
    poses = gesture._calibration["left"]
    current = list(poses["count"][1])
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        current[:] = target
        commands.append(list(target))

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    gesture._number(client, "left", poses, {"number": 2, "speed": 85})
    assert len(commands) == 2
    assert all(commands[0][i] == poses["count"][5][i] for i in (0, 5, 10, 15))
    assert commands[0][2] == poses["count"][1][2]
    assert commands[-1] == poses["count"][2]


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("from_num", range(10))
@pytest.mark.parametrize("target_num", range(10))
def test_blocking_thumb_clears_before_other_fingers_move(side, from_num, target_num, monkeypatch):
    gesture = GesturePlugin(None, {})
    numbers = gesture._calibration[side]["count"]
    # Distinct synthetic calibrations ensure the driver reads this side's
    # digit-5 table instead of embedding fixed clearance coordinates.
    numbers[5] = list(numbers[5])
    thumb = (0, 5, 10, 15)
    for idx, val in zip(thumb, (250, 65, 252, 250) if side == "left" else (245, 75, 240, 245)):
        numbers[5][idx] = val
    current = list(numbers[from_num])
    # Use real feedback rather than assuming the source pose was reached exactly.
    current[0] -= 3
    current[6] -= 2
    source = list(current)
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        current[:] = target
        commands.append(list(target))

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    clearance = MagicMock(wraps=gesture._clear_thumb_transition)
    monkeypatch.setattr(gesture, "_clear_thumb_transition", clearance)
    gesture._transition_to_number(client, from_num, target_num, numbers, 85)

    thumb_last = (from_num, target_num) in THREE_STAGE_PAIRS
    needs_clearance = thumb_last or (from_num != target_num and (from_num in (1, 2, 9) or target_num in (1, 2, 3, 4, 9)))
    assert clearance.call_count == int(needs_clearance)
    assert commands[-1] == numbers[target_num]
    if not needs_clearance:
        return
    release_first = (from_num, target_num) in THUMB_RELEASE_PAIRS
    assert len(commands) == (4 if release_first else 3 if thumb_last else 2)
    other = (1, 2, 3, 4, 6, 7, 8, 9, 16, 17, 18, 19)
    clearance_stage = 1 if release_first else 0
    for command in commands[:clearance_stage + 1]:
        assert all(command[i] == source[i] for i in other)
    if release_first:
        expected_tip = list(source)
        expected_tip[0] = 255
        expected_tip[15] = 255
        assert commands[0] == expected_tip
    assert all(commands[clearance_stage][i] == numbers[5][i] for i in thumb)
    if thumb_last:
        assert all(commands[-2][i] == numbers[5][i] for i in thumb)
        assert all(commands[-2][i] == numbers[target_num][i] for i in other)
        assert all(commands[-1][i] == commands[-2][i] for i in other)


@pytest.mark.parametrize("source_gesture", ["natural_gesture", "ok_gesture"])
@pytest.mark.parametrize("target_num", [1, 2, 3, 4, 9])
def test_non_numeric_posture_clears_thumb_before_entering_digit(source_gesture, target_num, monkeypatch):
    gesture = GesturePlugin(None, {})
    poses = gesture._calibration["left"]
    current = list(poses[source_gesture])
    source = list(current)
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        current[:] = target
        commands.append(list(target))

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    gesture._number(client, "left", poses, {"number": target_num, "speed": 85})
    assert len(commands) == 2
    assert all(commands[0][i] == poses["count"][5][i] for i in (0, 5, 10, 15))
    assert all(commands[0][i] == source[i] for i in (1, 2, 3, 4, 6, 7, 8, 9, 16, 17, 18, 19))
    assert commands[-1] == poses["count"][target_num]


@pytest.mark.parametrize("from_num,start_num,end_num", [
    (4, 1, 0), (1, 4, 1), (9, 3, 5), (4, 1, 4),
    (3, 0, 1), (7, 0, 1), (7, 2, 1), (6, 4, 3), (8, 4, 3), (9, 4, 3),
])
def test_count_entry_uses_clearance_and_preserves_displayed_sequence(from_num, start_num, end_num, monkeypatch):
    gesture = GesturePlugin(None, {})
    poses = gesture._calibration["left"]
    numbers = poses["count"]
    current = list(numbers[from_num])
    source = list(current)
    commands = []
    displayed = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        current[:] = target
        commands.append(list(target))

    def wait(seconds):
        gesture._check_stop()
        if seconds == 0.4:
            displayed.append(next(n for n, pose in numbers.items() if vector20_to_named16(pose) == vector20_to_named16(current)))

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", wait)
    gesture._count(client, poses, {"speed": 85, "start_num": start_num, "end_num": end_num, "hold_time_s": 0.4})
    step = 1 if start_num <= end_num else -1
    assert displayed == list(range(start_num, end_num + step, step))
    initial_target = start_num if start_num > end_num else 0
    clearance_stage = 1 if (from_num, initial_target) in THUMB_RELEASE_PAIRS else 0
    if from_num in (1, 2, 4, 9) or (from_num, 0) in THREE_STAGE_PAIRS or start_num > end_num:
        assert all(commands[clearance_stage][i] == numbers[5][i] for i in (0, 5, 10, 15))
        assert all(commands[0][i] == source[i] for i in (1, 2, 3, 4, 6, 7, 8, 9, 16, 17, 18, 19))
    if (from_num, initial_target) in THREE_STAGE_PAIRS:
        assert all(commands[clearance_stage + 1][i] == numbers[5][i] for i in (0, 5, 10, 15))
        assert all(commands[clearance_stage + 1][i] == numbers[initial_target][i] for i in (1, 2, 3, 4, 6, 7, 8, 9, 16, 17, 18, 19))
        assert commands[clearance_stage + 2] == numbers[initial_target]


@pytest.mark.parametrize("action,from_num,target_num", [
    ("number", 1, 4), ("count", 1, 4), ("number", 0, 4), ("number", 4, 0), ("count", 4, 4),
    ("number", 9, 3), ("number", 8, 4), ("number", 9, 4),
    ("number", 0, 7), ("number", 7, 0), ("number", 7, 1), ("number", 7, 2),
])
@pytest.mark.parametrize("speed_ratio,expected_speed", [
    ("1.0x", 85), ("1.5x", 128), ("2.0x", 170), ("2.5x", 212), ("3.0x", 255),
])
def test_digit_actions_keep_selected_speed_through_clearance(action, from_num, target_num, speed_ratio, expected_speed):
    import yaml

    config_path = Path(sys.modules[GesturePlugin.__module__].__file__).with_name("config.yaml")
    config = yaml.safe_load(config_path.read_text())
    client, bus = make_mock_client()
    ctrl = MagicMock()
    ctrl.channel = "can0"
    ctrl._motion_lock = threading.RLock()
    ctrl.get.return_value = client
    hand = MagicMock()
    hand.config = {}
    hand.get_controller_for_side.return_value = ctrl
    gesture = GesturePlugin(hand, config["plugins"]["hand_gesture"], global_config=config)
    client.set_joint_positions(gesture._calibration["left"]["count"][from_num])
    done = threading.Event()
    statuses = []

    def completed(aid, status, result):
        statuses.append(status)
        done.set()

    gesture._acp_callback = completed
    bus.sent_messages.clear()
    try:
        gesture.dispatch(action, {
            "instance_id": "speed-check", "bound_side": "left", "confirm_motion": True, "speed_ratio": speed_ratio,
            "number": target_num, "start_num": 1, "end_num": 2, "hold_time_s": 0.05,
        })
        assert done.wait(5)
        assert statuses == ["completed"]
        speed_frames = [list(msg.data) for msg in bus.sent_messages if 0x49 <= msg.data[0] <= 0x4D]
        assert speed_frames == [[ft] + [expected_speed] * 6 for ft in range(0x49, 0x4E)]
        if action == "number" and (from_num, target_num) in THUMB_RELEASE_PAIRS:
            thumb_frames = [list(msg.data) for msg in bus.sent_messages if msg.data[0] == 0x41 and len(msg.data) > 1]
            assert len(thumb_frames) == 4
            # Pitch and tip share the first CAN command; yaw/roll still hold 7.
            assert thumb_frames[0][3] == thumb_frames[0][6] == 255
            source = gesture._calibration["left"]["count"][7]
            assert thumb_frames[0][1:3] == [source[10], source[5]]
    finally:
        client.close()


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("from_num,target_num", [(0, 3), (3, 0), (0, 4), (4, 0), (0, 7), (7, 0)])
def test_zero_transitions_complete_with_recorded_folded_finger_feedback(side, from_num, target_num, monkeypatch):
    gesture = GesturePlugin(None, {"motion_timeout_s": 0.02})
    poses = gesture._calibration[side]
    numbers = poses["count"]
    # Recorded digit-0 feedback: pitch/tip reach the curled target, while
    # index yaw remains 159 rather than the configured 193.
    measured_zero = named16_to_vector20({
        "thumb_pitch": 111, "thumb_yaw": 139, "thumb_roll": 254, "thumb_tip": 120,
        "index_pitch": 8, "index_yaw": 159, "index_tip": 2,
        "middle_pitch": 4, "middle_yaw": 141, "middle_tip": 1,
        "ring_pitch": 0, "ring_yaw": 126, "ring_tip": 1,
        "pinky_pitch": 4, "pinky_yaw": 108, "pinky_tip": 1,
    })
    current = list(measured_zero if from_num == 0 else numbers[from_num])
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        current[:] = target
        commands.append(list(target))
        if target[1:5] == numbers[0][1:5] and target[16:20] == numbers[0][16:20]:
            for i in (1, 2, 3, 4, 6, 7, 8, 9, 16, 17, 18, 19):
                current[i] = measured_zero[i]

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    assert gesture._detect_current_number(client, numbers) == from_num
    result = gesture._number(client, side, poses, {"number": target_num, "speed": 85})

    assert result["number"] == target_num
    assert len(commands) == (4 if (from_num, target_num) in THUMB_RELEASE_PAIRS else 3)
    assert all(commands[-2][i] == numbers[5][i] for i in (0, 5, 10, 15))
    assert commands[-1] == numbers[target_num]
    assert gesture._detect_current_number(client, numbers) == target_num


@pytest.mark.parametrize("joint", [1, 2, 3, 4, 16, 17, 18, 19])
def test_zero_to_four_waits_for_full_finger_extension_before_thumb_return(joint, monkeypatch):
    gesture = GesturePlugin(None, {"motion_timeout_s": 0.02})
    numbers = gesture._calibration["right"]["count"]
    current = list(numbers[0])
    commands = []
    stage_reads = 0
    client = MagicMock()

    def set_positions(target):
        if len(commands) == 2:
            assert stage_reads >= 3, "thumb returned before finger extension was confirmed"
        commands.append(list(target))
        current[:] = target

    def get_positions():
        nonlocal stage_reads
        feedback = list(current)
        if len(commands) == 2:
            stage_reads += 1
            # A 15-count error passed the old 20-count threshold but must
            # not release the thumb into the index finger's path yet.
            if stage_reads < 3:
                feedback[joint] -= 15
        return feedback

    client.set_joint_positions.side_effect = set_positions
    client.get_positions_vector.side_effect = get_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    gesture._number(client, "right", gesture._calibration["right"], {"number": 4, "speed": 85})
    assert len(commands) == 3


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("target_num,joint,error,tolerance,completes", [
    (7, 4, 15, 20, True),  # Recorded 0->7 pinky feedback must allow thumb return.
    (7, 4, 21, 20, False),  # Still stop beyond the configured tolerance.
    (7, 4, 15, 12, False),  # Respect a stricter user configuration.
    (7, 1, 15, 20, False),  # Do not relax the index gate.
    (4, 4, -15, 20, False),  # Do not relax the verified 0->4 gate.
])
def test_zero_to_seven_pinky_feedback_does_not_skip_thumb_finish(
    side, target_num, joint, error, tolerance, completes, monkeypatch,
):
    gesture = GesturePlugin(None, {"motion_timeout_s": 0.01, "position_tolerance": tolerance})
    numbers = gesture._calibration[side]["count"]
    current = list(numbers[0])
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        current[:] = target
        commands.append(list(target))
        if len(commands) >= 2:
            current[joint] += error

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    if completes:
        result = gesture._number(client, side, gesture._calibration[side], {"number": target_num, "speed": 85})
        assert result["positions"]["pinky_pitch"] == 15
        assert len(commands) == 3
        assert commands[-1] == numbers[target_num]
    else:
        with pytest.raises(TimeoutError, match="四指到位"):
            gesture._number(client, side, gesture._calibration[side], {"number": target_num, "speed": 85})
        assert len(commands) == 2


@pytest.mark.parametrize("target_num", [0, 1, 2])
@pytest.mark.parametrize("joint", [0, 15])
def test_digit_seven_waits_for_both_released_joints_before_yaw_roll_move(target_num, joint, monkeypatch):
    gesture = GesturePlugin(None, {})
    poses = gesture._calibration["right"]
    numbers = poses["count"]
    current = list(numbers[7])
    commands = []
    release_reads = 0
    client = MagicMock()

    def set_positions(target):
        if len(commands) == 1:
            assert release_reads >= 3, "yaw/roll moved before pitch and tip both reached 255"
        commands.append(list(target))
        current[:] = target

    def get_positions():
        nonlocal release_reads
        feedback = list(current)
        if len(commands) == 1:
            release_reads += 1
            if release_reads < 3:
                feedback[joint] = 225
        return feedback

    client.set_joint_positions.side_effect = set_positions
    client.get_positions_vector.side_effect = get_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    gesture._number(client, "right", poses, {"number": target_num, "speed": 85})
    assert len(commands) == 4
    assert commands[0][0] == commands[0][15] == 255
    assert all(commands[0][i] == numbers[7][i] for i in (5, 10))


@pytest.mark.parametrize("target_num", [0, 1, 2])
@pytest.mark.parametrize("release_step", [1])
def test_digit_seven_release_can_be_interrupted_before_thumb_sweeps(target_num, release_step, monkeypatch):
    gesture = GesturePlugin(None, {})
    numbers = gesture._calibration["right"]["count"]
    current = list(numbers[7])
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        commands.append(list(target))
        current[:] = target

    def wait(_):
        if len(commands) == release_step:
            gesture._stop.set()
        gesture._check_stop()

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", wait)
    with pytest.raises(InterruptedError):
        gesture._transition_to_number(client, 7, target_num, numbers, 85)
    assert len(commands) == release_step
    assert all(command[i] == numbers[7][i] for command in commands for i in (5, 10))


@pytest.mark.parametrize("joint", [0, 5, 10, 15, 1, 16])
def test_zero_recognition_still_rejects_displaced_thumb_or_unclosed_index(joint):
    gesture = GesturePlugin(None, {})
    numbers = gesture._calibration["right"]["count"]
    current = list(numbers[0])
    current[6] = 159
    current[joint] += 40 if current[joint] <= 215 else -40
    client = MagicMock()
    client.get_positions_vector.return_value = current
    assert gesture._detect_current_number(client, numbers) is None


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("from_num,target_num", THREE_STAGE_PAIRS)
def test_three_stage_routing_survives_folded_finger_yaw_offsets(side, from_num, target_num, monkeypatch):
    gesture = GesturePlugin(None, {})
    poses = gesture._calibration[side]
    numbers = poses["count"]
    current = list(numbers[from_num])
    # Exercise source recognition with lateral feedback displaced by 40 counts
    # on curled fingers. Thumb and all flexion joints remain at the source pose.
    for finger in range(1, 5):
        if current[finger] == 0 and current[finger + 15] == 0:
            yaw = finger + 5
            current[yaw] += 40 if current[yaw] <= 215 else -40
    source = list(current)
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        commands.append(list(target))
        current[:] = target

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    assert gesture._detect_current_number(client, numbers) == from_num
    gesture._number(client, side, poses, {"number": target_num, "speed": 85})
    assert len(commands) == (4 if (from_num, target_num) in THUMB_RELEASE_PAIRS else 3)
    assert all(commands[0][i] == source[i] for i in (1, 2, 3, 4, 6, 7, 8, 9, 16, 17, 18, 19))
    assert all(commands[-2][i] == numbers[5][i] for i in (0, 5, 10, 15))
    assert commands[-1] == numbers[target_num]


@pytest.mark.parametrize("from_num,target_num,joint,error,name", [
    (0, 4, 6, -35, "index_yaw"),  # Extended finger yaw must still reach target.
    (4, 0, 1, 15, "index_pitch"),  # Curl must meet the stricter 10-count gate.
])
def test_zero_four_incomplete_finger_stage_reports_joint_and_does_not_return_thumb(
    from_num, target_num, joint, error, name, monkeypatch,
):
    gesture = GesturePlugin(None, {"motion_timeout_s": 0.01})
    numbers = gesture._calibration["right"]["count"]
    current = list(numbers[from_num])
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        commands.append(list(target))
        current[:] = target
        if len(commands) == 2:
            current[joint] += error

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    with pytest.raises(TimeoutError) as failed:
        gesture._transition_to_number(client, from_num, target_num, numbers, 85)
    assert "四指到位，拇指保持让位" in str(failed.value)
    assert f"{name}={current[joint]}/{numbers[target_num][joint]}" in str(failed.value)
    assert len(commands) == 2


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("from_num,target_num,blocked_stage", [
    (source, target, stage)
    for source, target in THREE_STAGE_PAIRS
    for stage in range(1, 5 if (source, target) in THUMB_RELEASE_PAIRS else 4)
])
def test_three_stage_stall_blocks_following_stages(side, from_num, target_num, blocked_stage, monkeypatch):
    gesture = GesturePlugin(None, {"motion_timeout_s": 0.01, "poll_interval_s": 0.001})
    numbers = gesture._calibration[side]["count"]
    current = list(numbers[from_num])
    source = list(current)
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        commands.append(list(target))
        if len(commands) < blocked_stage:
            current[:] = target
        elif blocked_stage == 1:
            # Digit 6 already has a clear thumb; simulate a real displacement
            # at the blocked waypoint so that arrival must fail there too.
            current[0] = target[0] - 40

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    with pytest.raises(TimeoutError, match="避碰.*未到位"):
        gesture._transition_to_number(client, from_num, target_num, numbers, 85)
    assert len(commands) == blocked_stage
    clearance_steps = 2 if (from_num, target_num) in THUMB_RELEASE_PAIRS else 1
    assert all(command[i] == source[i] for command in commands[:clearance_steps] for i in (1, 2, 3, 4, 6, 7, 8, 9, 16, 17, 18, 19))
    if blocked_stage == clearance_steps + 1:
        assert all(commands[-1][i] == numbers[5][i] for i in (0, 5, 10, 15))


@pytest.mark.parametrize("blocked_stage", [1, 2])
@pytest.mark.parametrize("from_num", [1, 2, 9])
def test_clearance_stall_aborts_remaining_stages(from_num, blocked_stage, monkeypatch):
    gesture = GesturePlugin(None, {"motion_timeout_s": 0.01, "poll_interval_s": 0.001})
    numbers = gesture._calibration["left"]["count"]
    current = list(numbers[from_num])
    commands = []
    client = MagicMock()
    client.get_positions_vector.side_effect = lambda: list(current)

    def set_positions(target):
        commands.append(list(target))
        if len(commands) < blocked_stage:
            current[:] = target

    client.set_joint_positions.side_effect = set_positions
    monkeypatch.setattr(gesture, "_wait", lambda _: gesture._check_stop())
    with pytest.raises(TimeoutError, match="避碰.*未到位"):
        gesture._transition_to_number(client, from_num, 5, numbers, 85)
    assert len(commands) == blocked_stage


def test_clearance_can_be_interrupted_before_other_fingers_move(monkeypatch):
    gesture = GesturePlugin(None, {})
    numbers = gesture._calibration["left"]["count"]
    client = MagicMock()
    client.get_positions_vector.return_value = list(numbers[9])

    def request_stop(_):
        gesture._stop.set()
        gesture._check_stop()

    monkeypatch.setattr(gesture, "_wait", request_stop)
    with pytest.raises(InterruptedError):
        gesture._transition_to_number(client, 9, 5, numbers, 85)
    assert client.set_joint_positions.call_count == 1


def test_failed_thumb_clearance_reports_error_and_releases_torque(monkeypatch):
    client, bus = make_mock_client()
    ctrl = MagicMock()
    ctrl.channel = "can0"
    ctrl._motion_lock = threading.RLock()
    ctrl.get.return_value = client
    hand = MagicMock()
    hand.config = {}
    hand.get_controller_for_side.return_value = ctrl
    gesture = GesturePlugin(hand, {"motion_timeout_s": 0.02, "poll_interval_s": 0.001})
    client.set_joint_positions(gesture._calibration["left"]["count"][9])
    original_send = bus.send

    def stalled_positions(msg):
        if 0x41 <= msg.data[0] <= 0x45 and len(msg.data) > 1:
            bus.sent_messages.append(msg)
            return
        original_send(msg)

    monkeypatch.setattr(bus, "send", stalled_positions)
    done = threading.Event()
    terminal = {}

    def completed(aid, status, result):
        terminal.update(status=status, result=result)
        done.set()

    gesture._acp_callback = completed
    bus.sent_messages.clear()
    try:
        gesture.dispatch("number", {"instance_id": "blocked-thumb", "bound_side": "left", "number": 5, "confirm_motion": True})
        assert done.wait(2)
        assert terminal["status"] == "error"
        assert "避碰过渡姿态未到位" in terminal["result"]["message"]
        assert client._currents == [0] * 5
        # Only the first waypoint's five position frames were sent.
        assert len([msg for msg in bus.sent_messages if 0x41 <= msg.data[0] <= 0x45 and len(msg.data) > 1]) == 5
    finally:
        client.close()


def test_validate_hold_torque_boundaries_and_types():
    # Valid values
    assert validate_hold_torque(None) == 0
    assert validate_hold_torque("") == 0
    assert validate_hold_torque(0) == 0
    assert validate_hold_torque(1) == 1
    assert validate_hold_torque(128) == 128
    assert validate_hold_torque(255) == 255
    assert validate_hold_torque(None, default=50) == 50

    # Invalid types and out-of-range values
    for invalid in [True, False, -1, 256, 1000, 3.14, "0", "100", [], {}]:
        with pytest.raises(ValueError, match="hold_torque 必须是 0–255 之间的整数"):
            validate_hold_torque(invalid)


def test_hand_and_gesture_hold_torque_dispatch_and_rejection(monkeypatch):
    client, mock_bus = make_mock_client()
    ctrl = MagicMock()
    ctrl.channel = "can0"
    ctrl._motion_lock = threading.RLock()
    ctrl.get.return_value = client
    ctrl.side = "left"
    hand_p = HandPlugin({"baudrate": 1000000, "side": "left"}, {"enabled": True})
    hand_p._controllers[("left", "can0")] = ctrl
    hand_p.get_controller_for_side = lambda *_: ctrl
    ctrl._stop_evt = threading.Event()
    # This test inspects accepted options only. Keep workers from racing those
    # assertions or creating hardware connections outside the injected mock bus.
    monkeypatch.setattr(threading.Thread, "start", lambda self: None)

    gesture_p = GesturePlugin(hand_p, {"motion_speed": 85}, global_config={"side": "left"})

    try:
        # 1. HandPlugin: non-integer / boolean hold_torque must be rejected
        for bad_val in [True, False, -1, 256, "100", 2.5]:
            with pytest.raises(ValueError, match="hold_torque 必须是 0–255 之间的整数"):
                hand_p.dispatch("open", {"instance_id": "h1", "bound_side": "left", "hold_torque": bad_val, "confirm_motion": True})

        # 2. GesturePlugin: non-integer / boolean hold_torque must be rejected
        for bad_val in [True, False, -1, 256, "50", 1.0]:
            with pytest.raises(ValueError, match="hold_torque 必须是 0–255 之间的整数"):
                gesture_p.dispatch("ok_gesture", {"instance_id": "g1", "bound_side": "left", "hold_torque": bad_val, "confirm_motion": True})

        # 3. Valid integer hold_torque accepted in HandPlugin
        res_h = hand_p.dispatch("open", {"instance_id": "h1", "bound_side": "left", "hold_torque": 80, "confirm_motion": True})
        assert res_h["state"] == "running"
        with hand_p._lock:
            assert hand_p._active[("left", "can0")]["hold_torque"] == 80
            hand_p._active.clear()
            vars(ctrl).pop("_command_owner", None)

        # 4. Omitted hold_torque defaults to 0 in HandPlugin
        res_h2 = hand_p.dispatch("open", {"instance_id": "h1", "bound_side": "left", "confirm_motion": True})
        assert res_h2["state"] == "running"
        with hand_p._lock:
            assert hand_p._active[("left", "can0")]["hold_torque"] == 0
            hand_p._active.clear()
            vars(ctrl).pop("_command_owner", None)

        # 5. Valid integer hold_torque accepted in GesturePlugin
        res_g = gesture_p.dispatch("ok_gesture", {"instance_id": "g1", "bound_side": "left", "hold_torque": 150, "confirm_motion": True})
        assert res_g["state"] == "running"
        with gesture_p._lock:
            assert gesture_p._active[("left", "can0")]["options"]["hold_torque"] == 150
            gesture_p._active.clear()
            vars(ctrl).pop("_command_owner", None)

        # 6. Omitted hold_torque defaults to 0 in GesturePlugin
        res_g2 = gesture_p.dispatch("ok_gesture", {"instance_id": "g1", "bound_side": "left", "confirm_motion": True})
        assert res_g2["state"] == "running"
        with gesture_p._lock:
            assert gesture_p._active[("left", "can0")]["options"]["hold_torque"] == 0
            gesture_p._active.clear()
            vars(ctrl).pop("_command_owner", None)
    finally:
        client.close()


def test_gesture_defensive_validation():
    client, mock_bus = make_mock_client()
    ctrl = MagicMock()
    ctrl.channel = "can0"
    ctrl._motion_lock = threading.RLock()
    ctrl.get.return_value = client
    ctrl.side = "left"
    hand_p = HandPlugin({"baudrate": 1000000, "side": "left"}, {"enabled": True})
    hand_p.get_controller_for_side = lambda *_: ctrl

    gesture_p = GesturePlugin(hand_p, {"motion_speed": 85}, global_config={"side": "left"})

    try:
        # 1. number action validations
        for bad_num in [True, False, -1, 10, "10", "-1", "4.0", "4e0", "04", "", " 4 ", "４", 4.0, 5.5, [], {}]:
            with pytest.raises(ValueError, match="number 必须是 0–9 之间的整数"):
                gesture_p.dispatch("number", {"instance_id": "g1", "bound_side": "left", "number": bad_num, "confirm_motion": True})

        # 2. count action validations
        for bad_s in [True, False, -1, 10, "1"]:
            with pytest.raises(ValueError, match="start_num 必须是 0–9 之间的整数"):
                gesture_p.dispatch("count", {"instance_id": "g1", "bound_side": "left", "start_num": bad_s, "confirm_motion": True})

        for bad_e in [True, False, -1, 15, "5"]:
            with pytest.raises(ValueError, match="end_num 必须是 0–9 之间的整数"):
                gesture_p.dispatch("count", {"instance_id": "g1", "bound_side": "left", "end_num": bad_e, "confirm_motion": True})

        for bad_ht in [True, False, 0.01, 20.0, "1.0"]:
            with pytest.raises(ValueError, match="hold_time_s 必须是 0.05–10.0 之间的数值"):
                gesture_p.dispatch("count", {"instance_id": "g1", "bound_side": "left", "hold_time_s": bad_ht, "confirm_motion": True})

        # 3. grasp and custom_grasp must NOT be in gesture_p
        tool_actions = gesture_p.get_tool()["inputSchema"]["properties"]["action"]["enum"]
        assert "grasp" not in tool_actions
        assert "custom_grasp" not in tool_actions
        assert gesture_p.dispatch("grasp", {"instance_id": "g1", "bound_side": "left", "confirm_motion": True}) is None
        assert gesture_p.dispatch("custom_grasp", {"instance_id": "g1", "bound_side": "left", "confirm_motion": True}) is None

        # 4. speed_ratio type & enum validation
        for bad_ratio in [True, 2.0, "5.0x", "fast"]:
            with pytest.raises(ValueError, match="speed_ratio must be one of"):
                gesture_p.dispatch("ok_gesture", {"instance_id": "g1", "bound_side": "left", "speed_ratio": bad_ratio, "confirm_motion": True})
    finally:
        client.close()


@pytest.mark.parametrize("number", [*range(10), *map(str, range(10))])
def test_number_accepts_integer_and_canvas_dropdown_values(number, monkeypatch):
    client, mock_bus = make_mock_client()
    ctrl = MagicMock()
    ctrl.channel = "can0"
    ctrl._motion_lock = threading.RLock()
    ctrl.get.return_value = client
    hand = MagicMock()
    hand.config = {}
    hand.get_controller_for_side.return_value = ctrl
    gesture = GesturePlugin(hand, {})
    # Inspect accepted options without running a motion worker.
    monkeypatch.setattr(threading.Thread, "start", lambda self: None)
    try:
        reply = gesture.dispatch("number", {
            "instance_id": "number-card", "bound_side": "left", "number": number, "confirm_motion": True,
        })
        assert reply["state"] == "running"
        assert type(gesture._active[("left", "can0")]["options"]["number"]) is int
        assert gesture._active[("left", "can0")]["options"]["number"] == int(number)
        assert all(len(msg.data) == 1 or msg.data[0] >= 0xC0 for msg in mock_bus.sent_messages)
    finally:
        client.close()


def test_grasp_defensive_validation_and_conflict_protection():
    grasp = GraspPlugin(None, {})
    for ratio in [True, False, 0, 256, '50', 1.0]:
        with pytest.raises(ValueError, match='speed_ratio'):
            grasp.dispatch('side_grasp', {'instance_id': 'gr1', "bound_side": "left", 'speed_ratio': ratio, 'confirm_motion': True})
    for parameter in ('target_force', 'max_force', 'hold_duration_s', 'active_fingers', 'approach_speed'):
        with pytest.raises(ValueError, match='已移除参数'):
            grasp.dispatch('side_grasp', {'instance_id': 'gr1', "bound_side": "left", parameter: 30, 'confirm_motion': True})
    for confirm in (None, False, 'true', 1):
        with pytest.raises(ValueError, match='confirm_motion'):
            grasp.dispatch('side_grasp', {'instance_id': 'gr1', "bound_side": "left", 'confirm_motion': confirm})
    assert grasp.dispatch('hold', {'instance_id': 'gr1', "bound_side": "left", 'confirm_motion': True}) is None
    assert grasp._controllers == {}


def test_cards_require_explicit_target_without_inheriting_old_configuration():
    """An optional ID or legacy config never substitutes for this call's side."""
    config = {"baudrate": 1000000, "side": "left", "can_channel": "can0",
              "left_can_id": 0x28, "right_can_id": 0x27}
    hand_p = HandPlugin(config, {"enabled": True})
    gesture_p = GesturePlugin(hand_p, {"motion_speed": 85}, global_config=config)
    grasp_p = GraspPlugin(hand_p, {}, global_config=config)
    for card, action in ((hand_p, "open"), (gesture_p, "ok_gesture"), (grasp_p, "side_grasp")):
        assert card.dispatch("start", {"instance_id": "new-card"})["state"] == "ready"
        card.dispatch("config", {"instance_id": "old-card", "bound_side": "left"})
        for identity in ("", "old-card", "new-card"):
            for invalid_side in (None, "", "some_unbound_instance", "LEFT", True, [], {}):
                args = {"instance_id": identity, "confirm_motion": True}
                if invalid_side is not None:
                    args["bound_side"] = invalid_side
                with pytest.raises(ValueError, match="bound_side"):
                    card.dispatch(action, args)
        assert not card._controllers
        assert not card._active
        assert not hasattr(card, "_bindings")

    # The direct controller APIs accept physical sides, never canvas IDs.
    for invalid in ("", "some_unbound_instance"):
        with pytest.raises(ValueError, match="bound_side"):
            hand_p._get_controller(invalid)
        with pytest.raises(ValueError, match="bound_side"):
            gesture_p._get_client_and_ctrl(invalid)
        with pytest.raises(ValueError, match="bound_side"):
            grasp_p._get_client_and_ctrl(invalid)
