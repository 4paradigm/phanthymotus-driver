"""Contract and control-profile tests for the Bumi ``spin`` card.

``Spin.py`` keeps ROS and the vendor SDK out of module scope, so these tests
run with fake HighController/LocoPlugin objects and no rclpy.
"""

import importlib.util
import math
import threading
import types
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / "Spin.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("bumi_spin_card", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def module():
    return _load_module()


class _FakeHighCtrl:
    def __init__(self, mode=2, yaw=0.0):
        self.mode = mode
        self.yaw = yaw
        self.commands = []

    def get_mode(self):
        return self.mode

    def get_imu_data(self):
        half = math.radians(self.yaw) / 2.0
        return types.SimpleNamespace(ori=[0.0, 0.0, math.sin(half), math.cos(half)])

    def publish_cmd(self, vx, vy, vyaw, command, index):
        self.commands.append(vyaw)


class _FakeLoco:
    def __init__(self, moving=False):
        self.moving = moving
        self.published = []
        self.preempted = []
        self.owner = None
        self._rotate_lock = threading.RLock()
        self._body_action = None
        self._rotate_state = {"state": "idle"}

    def is_moving(self):
        return self.moving

    def _preempt_servo(self, reason):
        self.preempted.append(reason)
        return True

    def _set_servo_velocity_owner(self, owned):
        self.owner = owned

    def publish_velocity(self, vx, vy, vyaw):
        self.published.append(vyaw)


# ── Pure helpers ─────────────────────────────────────────────────────────────

@pytest.mark.parametrize("value, expected", [
    (1, 1), (5, 5), ("3", 3), (2.0, 2), (5.0, 5),
])
def test_valid_turns(module, value, expected):
    assert module.validate_turns(value) == expected


@pytest.mark.parametrize("value", [
    0, 6, True, False, None, 2.5, "two", "0", "6", [], {},
])
def test_invalid_turns_are_rejected(module, value):
    with pytest.raises(module.SpinInputError):
        module.validate_turns(value)


@pytest.mark.parametrize("value, expected", [
    ("clockwise", "clockwise"), ("CW", "clockwise"), ("right", "clockwise"),
    ("顺时针", "clockwise"),
    ("counterclockwise", "counterclockwise"),
    ("counter-clockwise", "counterclockwise"), ("ccw", "counterclockwise"),
    ("left", "counterclockwise"), ("逆时针", "counterclockwise"),
])
def test_direction_aliases(module, value, expected):
    assert module.normalize_direction(value) == expected


@pytest.mark.parametrize("value", [None, 3, "up", "", "顺逆"])
def test_unknown_direction_is_rejected(module, value):
    assert module.normalize_direction(value) is None


@pytest.mark.parametrize("value, expected", [
    (None, 0.55), (0.55, 0.55), (0.80, 0.80), ("0.70", 0.70),
])
def test_valid_speed(module, value, expected):
    assert module.validate_speed(value, 0.80) == pytest.approx(expected)


@pytest.mark.parametrize("value", [
    0.54, 0.81, 1.0, 0, -1, True, False, "fast",
])
def test_invalid_speed_is_rejected(module, value):
    with pytest.raises(module.SpinInputError):
        module.validate_speed(value, 0.80)


def test_speed_is_bounded_by_the_configured_limit(module):
    assert module.validate_speed(0.60, 0.65) == pytest.approx(0.60)
    with pytest.raises(module.SpinInputError):
        module.validate_speed(0.80, 0.65)
    with pytest.raises(module.SpinInputError):
        module.validate_speed(0.80, 0.60)


def test_clockwise_is_negative_and_ccw_is_positive(module):
    assert module.spin_target_deg(1, "clockwise") == -360.0
    assert module.spin_target_deg(5, "clockwise") == -1800.0
    assert module.spin_target_deg(1, "counterclockwise") == 360.0
    assert module.spin_target_deg(5, "counterclockwise") == 1800.0


def test_wrap180(module):
    assert module.wrap180(190) == -170
    assert module.wrap180(-190) == 170
    assert module.wrap180(180) == -180


def test_speed_profile_tapers_to_zero(module):
    max_vyaw = 0.55
    assert module.speed_for_remaining(180, max_vyaw) == max_vyaw
    assert module.speed_for_remaining(25, max_vyaw) == max_vyaw
    assert module.speed_for_remaining(20, max_vyaw) == pytest.approx(0.45)
    assert module.speed_for_remaining(10, max_vyaw) == 0.45
    assert module.speed_for_remaining(8, max_vyaw) == 0.0
    assert module.speed_for_remaining(2, max_vyaw) == 0.0
    # The faster setting cruises faster, but the final taper still stops.
    assert module.speed_for_remaining(180, 0.80) == 0.80
    assert module.speed_for_remaining(20, 0.80) == pytest.approx(0.64)
    assert module.speed_for_remaining(8, 0.80) == 0.0


def test_pulse_duration_is_bounded(module):
    assert module.pulse_duration(1) == 0.08
    assert module.pulse_duration(5) == 0.15
    assert module.pulse_duration(100) == 0.25


def test_expected_duration_grows_with_turns(module):
    one = module.expected_duration_s(1, 0.55)
    five = module.expected_duration_s(5, 0.55)
    fast = module.expected_duration_s(1, 0.80)
    assert 10.0 < one < 30.0
    assert five > one * 2
    assert fast < one


# ── Card contract ────────────────────────────────────────────────────────────

def test_tool_contract(module):
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl())
    tool = card.get_tool()
    assert tool["name"] == "spin"
    assert tool["type"] == "actuator"
    assert tool["multiInstance"] is False
    assert tool["topic_out"] == []
    schema = tool["inputSchema"]
    assert schema["properties"]["action"]["enum"] == [
        "spin", "stop", "info", "start"]
    assert schema["properties"]["turns"]["minimum"] == 1
    assert schema["properties"]["turns"]["maximum"] == 5
    assert schema["properties"]["direction"]["enum"] == [
        "clockwise", "counterclockwise"]
    assert schema["properties"]["speed"]["minimum"] == 0.55
    assert schema["properties"]["speed"]["maximum"] == 0.80
    assert schema["properties"]["speed"]["default"] == 0.55
    assert schema["x-completion"] == {"actions": ["spin"], "timeout": 180}
    assert schema["x-hooks"] == {"on_interrupt_motion": {"action": "stop"}}
    assert set(schema["x-action-params"]) == {"spin", "stop", "info", "start"}


@pytest.mark.parametrize("configured, expected", [
    (2.0, 0.80), (0.1, 0.55), (0.55, 0.55), (0.80, 0.80),
    ("bad", 0.80), (None, 0.80),
])
def test_max_vyaw_is_clamped(module, configured, expected):
    plugin = module.SpinPlugin({"max_vyaw": configured}, "robot", None,
                               _FakeHighCtrl())
    assert plugin._max_vyaw == pytest.approx(expected)


def test_start_reports_unavailable_without_controller(module):
    card = module.SpinPlugin({}, "robot", None, None)
    assert card.dispatch("start", {})["state"] == "unavailable"


def test_info_is_idle_before_any_spin(module):
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl())
    info = card.dispatch("info", {})
    assert info["state"] == "idle"
    assert info["max_turns"] == 5
    assert info["speed_range"] == [0.55, 0.80]
    assert info["default_speed"] == 0.55
    assert info["last"] is None


def test_invalid_turns_never_reach_the_robot(module):
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl())
    result = card.dispatch("spin", {"turns": 9, "direction": "clockwise"})
    assert result["state"] == "error"
    assert "turns" in result["error"]


def test_invalid_direction_never_reach_the_robot(module):
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl())
    result = card.dispatch("spin", {"turns": 1, "direction": "up"})
    assert result["state"] == "error"
    assert "direction" in result["error"]


def test_invalid_speed_never_reach_the_robot(module):
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl())
    result = card.dispatch(
        "spin", {"turns": 1, "direction": "clockwise", "speed": 0.9})
    assert result["state"] == "error"
    assert "speed" in result["error"]


def test_speed_is_validated_before_preflight(module):
    # A valid 0.70 must pass validation and reach the workmode check; an
    # invalid speed would have been reported as a speed error instead.
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl(mode=0))
    result = card.dispatch(
        "spin", {"turns": 1, "direction": "clockwise", "speed": 0.70})
    assert result["state"] == "error"
    assert "workmode=2" in result["error"]


def test_spin_requires_walking_mode(module):
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl(mode=0))
    result = card.dispatch("spin", {"turns": 1, "direction": "clockwise"})
    assert result["state"] == "error"
    assert "workmode=2" in result["error"]


def test_spin_refuses_when_another_motion_is_running(module):
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl(),
                             loco_plugin=_FakeLoco(moving=True))
    result = card.dispatch("spin", {"turns": 1, "direction": "clockwise"})
    assert result["state"] == "error"
    assert "其他移动动作" in result["error"]


def test_publish_uses_loco_rate_limiter_when_available(module):
    loco = _FakeLoco()
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl(), loco_plugin=loco)
    card._publish(0.25)
    assert loco.published == [0.25]


def test_yaw_and_advance_follow_the_imu(module):
    high = _FakeHighCtrl(yaw=0.0)
    card = module.SpinPlugin({}, "robot", None, high)
    traveled, last = card._advance(0.0, high.yaw)
    assert traveled == pytest.approx(0.0)
    high.yaw = 10.0
    traveled, last = card._advance(traveled, last)
    assert traveled == pytest.approx(10.0)
    high.yaw = 20.0
    traveled, _ = card._advance(traveled, last)
    assert traveled == pytest.approx(20.0)


def test_stop_without_active_spin_is_a_no_op(module):
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl())
    assert card.dispatch("stop", {}) == {"state": "idle", "stopped": False}


def test_reserve_loco_marks_and_releases_the_chassis(module):
    loco = _FakeLoco()
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl(), loco_plugin=loco)
    assert card._reserve_loco() is True
    assert loco._body_action == "spin.spin"
    card._release_loco()
    assert loco._body_action is None


def test_reserve_loco_refuses_an_already_reserved_chassis(module):
    loco = _FakeLoco()
    loco._body_action = "loco.move"
    card = module.SpinPlugin({}, "robot", None, _FakeHighCtrl(), loco_plugin=loco)
    with pytest.raises(module.SpinInputError):
        card._reserve_loco()
    assert loco._body_action == "loco.move"
