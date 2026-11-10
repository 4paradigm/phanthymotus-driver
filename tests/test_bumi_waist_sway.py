"""Waist sway card tests — no robot, no ROS, no Noetix SDK at module level.

The card is call-shaped (like `loco.move`): parameters ride on the `start`
action. The tests fake `lowcontrol_py` (the SDK module the card imports
lazily inside its stream thread) and drive the state machine through the
same entry points the sibling cards use.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_bumi_waist_sway.py -q
"""

from __future__ import annotations

import importlib.util
import math
import sys
import time
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / "noetix" / "bumi"

# The vendor example tables, copied verbatim from example_policy_walk.cpp /
# test_policy.py. `test_source_tables_match_vendor_example` pins the card's
# copies to these, and FakeLowController reuses the order for its index map.
VENDOR_JOINT_NAMES = [
    "leg_l1_joint", "leg_r1_joint", "waist_1_joint", "leg_l2_joint",
    "leg_r2_joint", "arm_l1_joint", "arm_r1_joint", "leg_l3_joint",
    "leg_r3_joint", "arm_l2_joint", "arm_r2_joint", "leg_l4_joint",
    "leg_r4_joint", "arm_l3_joint", "arm_r3_joint", "leg_l5_joint",
    "leg_r5_joint", "arm_l4_joint", "arm_r4_joint", "leg_l6_joint",
    "leg_r6_joint",
]
VENDOR_STAND_ANGLES = [
    -0.1495, -0.1495, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2618, -0.2618,
    0.3215, 0.3215, 0.0, 0.0, -0.1720, -0.1720, 0.0, 0.0, 0.0, 0.0,
]


@pytest.fixture(scope="module")
def ws():
    spec = importlib.util.spec_from_file_location(
        "bumi_waist_sway", BUMI / "waist_sway.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys.modules["bumi_waist_sway"] = module
    yield module
    sys.modules.pop("bumi_waist_sway", None)


def stand_table_by_hw():
    """The vendor stand pose in hardware-motor order under the fake's mapping.

    The fake maps policy order (waist at position 2) onto hw indices: the
    waist is motor 20, every other name keeps a unique index below 20.
    """
    table = [None] * 21
    for i, name in enumerate(VENDOR_JOINT_NAMES):
        hw = 20 if name == "waist_1_joint" else (i if i < 2 else i - 1)
        table[hw] = VENDOR_STAND_ANGLES[i]
    return table


class FakeMotorCmd:
    def __init__(self):
        self.pos = 0.0
        self.vel = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.tau = 0.0
        self.motor_id = 0


class FakeLowController:
    """Stands in for `lowcontrol_py.LowController` (a singleton).

    `waist_pos=None` makes the measured waist follow the last commanded value
    (a perfectly tracking robot); a float pins it (a robot ignoring commands).
    """

    _the = None

    def __init__(self, *, init_error=None, waist_pos=None):
        self.cmds = []
        self.init_error = init_error
        self.waist_pos = waist_pos
        FakeLowController._the = self

    @classmethod
    def instance(cls):
        return cls._the

    def init(self):
        if self.init_error:
            raise RuntimeError(self.init_error)
        return True

    def getJointsIndex(self, name):
        if name == "waist_1_joint":
            return 20
        try:
            i = VENDOR_JOINT_NAMES.index(name)
        except ValueError:
            return -1
        return i if i < 2 else i - 1

    def set_joint(self, cmds):
        self.cmds.append(list(cmds))

    def get_joint_state(self):
        waist = self.waist_pos
        if waist is None:
            waist = self.cmds[-1][20].pos if self.cmds else 0.0
        return [
            types.SimpleNamespace(
                pos=(waist if i == 20 else 0.0), vel=0.0, tau=0.0,
                motor_id=i, error=0, temperature=25)
            for i in range(21)
        ]


class FakeHighCtrl:
    def __init__(self, mode=2):
        self.mode = mode

    def get_mode(self):
        return self.mode


class FakeLoco:
    def __init__(self):
        self.prepare_calls = []

    def prepare_walk_mode(self, requester):
        self.prepare_calls.append(requester)
        return {"state": "completed", "workmode": 2}


@pytest.fixture
def make_card(ws, monkeypatch):
    """Build a started card plus its fakes; every card is stopped on teardown."""
    made = []

    def _make(*, high=None, loco=None, low=None):
        low = low or FakeLowController()
        fake_module = types.ModuleType("lowcontrol_py")
        fake_module.LowController = FakeLowController
        fake_module.MotorCmd = FakeMotorCmd
        monkeypatch.setitem(sys.modules, "lowcontrol_py", fake_module)
        high = high or FakeHighCtrl()
        card = ws.WaistSwayPlugin({}, "bumi", None, high, loco_plugin=loco)
        card.start()
        made.append(card)
        return card, low, high, loco

    yield _make
    for card in made:
        card.stop()


def _wait(predicate, timeout=2.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def _start(card, **params):
    return card.dispatch("start", params)


# ── the vendor tables the whole stream rests on ──────────────────────────────

def test_source_tables_match_vendor_example(ws):
    assert ws._SDK_JOINT_NAMES == VENDOR_JOINT_NAMES
    assert ws._STAND_ANGLES == VENDOR_STAND_ANGLES
    assert len(ws._STAND_ANGLES) == 21
    assert ws._SPEED_PERIODS == {"slow": 8.0, "medium": 4.0}
    assert ws._RATE_HZ == 500.0


# ── the stream itself ────────────────────────────────────────────────────────

def test_start_streams_stand_hold_with_waist_sine(make_card):
    card, low, _high, _loco = make_card()
    assert _start(card, amplitude_deg=30.0, speed="medium")["state"] == "streaming"

    # ~2.3 s of ticks at 500 Hz: past the 1 s blend-in and into the sine's
    # second half-cycle, so both waist directions have been commanded.
    assert _wait(lambda: len(low.cmds) > 1150, timeout=3.5), "stream did not run"
    table = stand_table_by_hw()

    for cmd in low.cmds:
        assert len(cmd) == 21
        assert [c.motor_id for c in cmd] == list(range(21))

    last = low.cmds[-1]
    for i, c in enumerate(last):
        expected_kp = 10.0
        if i == 20:
            assert c.kp == pytest.approx(expected_kp)
            assert c.kd == pytest.approx(0.5)
        else:
            # Past the blend window the hold targets the vendor stand pose.
            assert c.pos == pytest.approx(table[i]), f"joint {i}"
            assert c.kp == pytest.approx(expected_kp)
            assert c.kd == pytest.approx(0.5)
        assert c.vel == 0.0 and c.tau == 0.0

    waist = [cmd[20].pos for cmd in low.cmds]
    assert max(waist) > 0.05, "waist never swung one way"
    assert min(waist) < -0.05, "waist never swung back"
    amplitude = math.radians(30.0)
    assert max(waist) <= amplitude + 1e-9
    assert min(waist) >= -amplitude - 1e-9
    assert card._info()["waist_tracking"] == "ok"
    assert card._info()["amplitude_deg"] == pytest.approx(30.0)
    assert card._info()["speed"] == "medium"


def test_start_parameter_validation(make_card):
    card, low, _high, _loco = make_card()
    assert _start(card, speed="fast")["state"] == "error"
    assert _start(card, amplitude_deg=70.0)["state"] == "error"
    assert _start(card, amplitude_deg=2.0)["state"] == "error"
    assert _start(card, duration_s=-1.0)["state"] == "error"
    assert _start(card, duration_s="soon")["state"] == "error"
    assert _start(card, amplitude_deg=True)["state"] == "error"
    assert low.cmds == []


# ── engage gates and timing ──────────────────────────────────────────────────

def test_engage_refused_when_not_standing(make_card):
    card, low, _high, _loco = make_card(high=FakeHighCtrl(mode=5))
    result = _start(card)
    assert result["state"] == "error"
    assert "workmode" in result["error"]
    info = card._info()
    assert info["state"] == "error"
    assert low.cmds == []


def test_untimed_start_runs_until_stop(make_card):
    card, low, _high, _loco = make_card()
    assert _start(card, duration_s=0)["state"] == "streaming"
    assert card._info()["remaining_s"] is None
    assert _wait(lambda: len(low.cmds) > 100)
    time.sleep(0.3)   # several supervisor ticks: still streaming
    assert card._info()["state"] == "streaming"
    assert card.dispatch("stop", {})["state"] == "off"


def test_timed_start_auto_stops_and_hands_back(make_card):
    card, low, _high, _loco = make_card()
    assert _start(card, duration_s=1.0)["state"] == "streaming"
    assert card._info()["remaining_s"] is not None
    assert _wait(lambda: len(low.cmds) > 50)

    # duration (1 s) + supervisor tick + wind-down (1 s) — then it must be off.
    assert _wait(lambda: card._info()["state"] == "off", timeout=4.0), \
        "timed sway did not stop itself"
    stopped_at = len(low.cmds)
    time.sleep(0.2)
    assert len(low.cmds) == stopped_at
    assert card._info()["last_restore"]["reason"] == "duration_elapsed"


def test_restart_replaces_the_running_sway(make_card):
    card, low, _high, _loco = make_card()
    assert _start(card, amplitude_deg=20.0, speed="slow")["state"] == "streaming"
    assert _wait(lambda: len(low.cmds) > 100)

    assert _start(card, amplitude_deg=40.0, speed="medium")["state"] == "streaming"
    info = card._info()
    assert info["amplitude_deg"] == pytest.approx(40.0)
    assert info["speed"] == "medium"
    # The new stream keeps running past the replacement window.
    assert _wait(lambda: len(low.cmds) > 1200, timeout=4.0)
    assert card._info()["state"] == "streaming"


# ── chassis coexistence ──────────────────────────────────────────────────────

def test_chassis_notify_keeps_streaming_and_gates_the_self_check(make_card):
    # A waist that never follows (pinned at 0) with a 60 deg amplitude: the
    # self-check would abort within a few seconds — unless the chassis label
    # says the robot is walking, where being ignored is expected.
    low = FakeLowController(waist_pos=0.0)
    card, low, _high, _loco = make_card(low=low)
    assert _start(card, amplitude_deg=60.0, speed="medium")["state"] == "streaming"
    card.notify_chassis_active("loco.move")
    assert _wait(lambda: len(low.cmds) > 1400, timeout=4.0), \
        "default policy must keep streaming while the chassis walks"
    assert card._info()["state"] == "streaming"
    assert card._info()["chassis_active"] == "loco.move"

    card.notify_chassis_idle("loco.move")
    assert _wait(lambda: card._info()["state"] == "error", timeout=3.0), \
        "self-check must resume once the chassis goes idle"


# ── auto-disable on body actions ─────────────────────────────────────────────

def test_body_action_auto_disables_and_restores(make_card):
    loco = FakeLoco()
    card, low, high, _unused = make_card(loco=loco)
    assert _start(card)["state"] == "streaming"
    assert _wait(lambda: len(low.cmds) > 10)

    high.mode = 1   # force the restore through the loco card's mode prep
    assert card.disable_for_body_action("semantic_action.wave") is True
    info = card._info()
    assert info["state"] == "off"
    assert info["preempted_by"] == "semantic_action.wave"
    stopped_at = len(low.cmds)
    time.sleep(0.2)
    assert len(low.cmds) == stopped_at
    assert loco.prepare_calls, "release must hand the robot back via loco"

    assert card.disable_for_body_action("semantic_action.wave") is False


def test_supervisor_disables_on_external_workmodes(make_card):
    card, low, high, _loco = make_card()
    assert _start(card)["state"] == "streaming"
    assert _wait(lambda: len(low.cmds) > 5)

    high.mode = 26   # protection, e.g. after a fall nobody reported
    assert _wait(lambda: card._info()["state"] == "off"), \
        "supervisor missed the protection mode"
    assert "workmode_26" in card._info()["preempted_by"]
    stopped_at = len(low.cmds)
    time.sleep(0.2)
    assert len(low.cmds) == stopped_at

    high.mode = 2
    assert _start(card)["state"] == "streaming"
    high.mode = 8   # greet, e.g. started from the vendor App
    assert _wait(lambda: "workmode_8" in (card._info()["preempted_by"] or ""))


# ── failure paths ────────────────────────────────────────────────────────────

def test_tracking_abort_when_firmware_ignores_the_stream(make_card):
    low = FakeLowController(waist_pos=0.0)   # the waist never moves
    card, low, _high, _loco = make_card(low=low, )
    assert _start(card, amplitude_deg=60.0, speed="medium")["state"] == "streaming"
    assert _wait(lambda: card._info()["state"] == "error", timeout=6.0), \
        "an ignored stream must tear itself down"
    info = card._info()
    assert "not tracking" in info["last_error"]
    stopped_at = len(low.cmds)
    time.sleep(0.2)
    assert len(low.cmds) == stopped_at


def test_low_controller_init_failure_is_reported(make_card):
    low = FakeLowController(init_error="boom")
    card, _low, _high, _loco = make_card(low=low)
    result = _start(card)
    assert result["state"] == "streaming"   # the failure surfaces in the thread
    assert _wait(lambda: card._info()["state"] == "error")
    info = card._info()
    assert "boom" in info["low_controller"]
    assert info["last_error"]


# ── operator stop ────────────────────────────────────────────────────────────

def test_stop_action_winds_down_and_releases(make_card):
    card, low, _high, _loco = make_card()
    assert _start(card)["state"] == "streaming"
    assert _wait(lambda: len(low.cmds) > 10)

    result = card.dispatch("stop", {})
    assert result["state"] == "off"
    info = card._info()
    assert info["state"] == "off"
    deadline = time.monotonic() + 2.0   # wind-down is 1 s
    count = len(low.cmds)
    while time.monotonic() < deadline:
        time.sleep(0.05)
        stopped = len(low.cmds)
        if stopped == count and count > 10:
            time.sleep(0.15)
            assert len(low.cmds) == stopped, "stream kept running after stop"
            return
        count = stopped
    pytest.fail("stream did not stop after the wind-down")
