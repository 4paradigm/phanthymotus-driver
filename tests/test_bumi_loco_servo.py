"""The Bumi `loco_servo` card — `motus.control/1` twist onto a normalised SDK.

`ControlSink` is tested on its own in test_control_sink.py and the shared
chassis-shaped concerns in test_r1_loco_servo.py. This file covers what is
*Bumi*-shaped, which is where a Bumi-shaped mistake would be:

  - **the unit conversion**, because `publish_cmd` takes `[-1, 1]` while the
    protocol is m/s, and a factor error there is a robot that travels at the
    wrong speed while every counter reports success
  - **the calibration being an estimate**, because nothing has measured this
    chassis and the policy upstream has no other way to find that out
  - **holding means republishing zero**, not publishing nothing: a Bumi command
    does not persist the way R1's `Move(..., True)` does, so "stop sending" is
    not a stop
  - **arbitration with four call-shaped cards**, not one — the posture, preset
    and teaching actions all change `workmode` out from under a running stream

No robot, no ROS, no Noetix SDK at module level, so this runs on a laptop —
deliberately, because a test that only runs on a Bumi is a test that runs after
the robot has already walked.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_bumi_loco_servo.py -q
"""

from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _load():
    bundle = ROOT / "noetix" / "bumi"
    sys.path.insert(0, str(bundle))
    try:
        spec = importlib.util.spec_from_file_location(
            "bumi_loco_servo", bundle / "loco_servo.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules["bumi_loco_servo"] = module
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(bundle))


loco_servo = _load()

from common.control import parse_descriptor  # noqa: E402
from common.control.sink import Verdict  # noqa: E402


class FakeHighCtrl:
    """Stands in for `HighController`. Answers the workmode, counts the reads."""

    def __init__(self, mode=loco_servo.WALKING_MODE, raises=False):
        self.mode = mode
        self.mode_reads = 0
        self._raises = raises

    def get_mode(self):
        self.mode_reads += 1
        if self._raises:
            raise RuntimeError("DDS read failed")
        return self.mode


class FakeLoco:
    """Stands in for `LocoPlugin`. Records the normalised commands."""

    def __init__(self, moving=False):
        self._moving = moving
        self.servo = None
        self.published = []

    def attach_servo(self, servo):
        self.servo = servo

    def is_moving(self):
        return self._moving

    def publish_velocity(self, x, y, z):
        self.published.append((round(x, 6), round(y, 6), round(z, 6)))


def _card(high_ctrl=None, loco=None, **config):
    config.setdefault("dry_run", False)
    return loco_servo.LocoServoPlugin(
        config, "bumi", executor=None,
        high_ctrl=high_ctrl if high_ctrl is not None else FakeHighCtrl(),
        loco_plugin=loco)


def _command(values, seq=1, stamp_ms=None, ttl_ms=200):
    # Wall clock, because the sink's ttl and observation-age checks are against
    # the real one unless a clock is injected — a fixed literal here is simply
    # an expired command, which tests the wrong thing.
    stamp_ms = int(time.time() * 1000) if stamp_ms is None else stamp_ms
    return {"schema": "motus.control/1", "seq": seq, "stamp_ms": stamp_ms,
            "obs_stamp_ms": stamp_ms, "ttl_ms": ttl_ms, "source": "test",
            "priority": 50, "mode": "twist", "dof": 6, "values": list(values)}


def _sink(card):
    from common.control import ControlSink
    return ControlSink(card._descriptor, card._apply,
                       on_watchdog=card._hold, on_abort=card._hold)


# ── the descriptor ───────────────────────────────────────────────────────────

def test_the_descriptor_is_valid():
    parsed = parse_descriptor(loco_servo.build_descriptor(loco_servo.Calibration()))
    assert parsed.mode == "twist"
    assert parsed.dof == 6
    assert parsed.joint_names == ("vx", "vy", "vz", "wx", "wy", "wz")


def test_axis_order_matches_the_odometry_format():
    """Commanded and measured have to line up index by index, or every
    comparison a consumer makes is between two different axes."""
    from common.odom import AXES

    assert tuple(loco_servo.AXIS_NAMES) == AXES


def test_the_declared_ceilings_are_the_calibration():
    """The limits a policy is clamped against and the numbers its commands are
    divided by have to be the same, or it is fitted to a ceiling that does not
    exist and every full-scale command comes back REJECTED."""
    calibration = loco_servo.Calibration({"full_scale_vx_mps": 0.7,
                                          "full_scale_wz_rads": 1.4})
    limits = loco_servo.build_descriptor(calibration)["limits"]
    assert limits["upper"][0] == 0.7 and limits["lower"][0] == -0.7
    assert limits["upper"][5] == 1.4 and limits["lower"][5] == -1.4


def test_unactuatable_axes_are_pinned_to_zero():
    limits = loco_servo.build_descriptor(loco_servo.Calibration())["limits"]
    for i in (2, 3, 4):          # vz, wx, wy
        assert limits["lower"][i] == 0.0 and limits["upper"][i] == 0.0


def test_max_velocity_is_not_declared():
    """In a velocity space `max_velocity` would be jerk — a limit nobody can
    interpret is worse than an absent one."""
    limits = loco_servo.build_descriptor(loco_servo.Calibration())["limits"]
    assert "max_velocity" not in limits


def test_the_step_cap_is_never_finer_than_the_deadband_on_the_same_axis():
    """An acceleration cap below the floor is not a cap, it is dead time: the
    ramp's first steps are all below the deadband and the robot executes none
    of them, then starts at full speed. See `_step_limit`."""
    calibration = loco_servo.Calibration({"min_wz_rads": 0.9,
                                          "full_scale_wz_rads": 1.5})
    limits = loco_servo.build_descriptor(calibration)["limits"]
    assert limits["max_delta_per_step"][5] >= 0.9


def test_force_torque_is_declared_null_rather_than_omitted():
    """A missing protection has to be visible; parse_descriptor requires it."""
    descriptor = loco_servo.build_descriptor(loco_servo.Calibration())
    assert "force_torque" in descriptor and descriptor["force_torque"] is None


def test_the_footprint_says_it_is_an_estimate():
    """navi reads `source` and repeats the caveat in its `degraded` list. The
    meshes are stripped out of `bumi_model.urdf`, so nothing in this repo knows
    how wide this robot really is, and a footprint that did not say so would be
    believed."""
    footprint = loco_servo.build_descriptor(loco_servo.Calibration())["footprint"]
    assert footprint["source"] == "estimate"
    assert footprint["half_width"] > 0


# ── the calibration ──────────────────────────────────────────────────────────

def test_the_calibration_defaults_to_estimate_and_says_so_in_the_descriptor():
    """The whole velocity space is a guess until somebody walks the robot, and
    the policy upstream has no other way to find that out."""
    descriptor = loco_servo.build_descriptor(loco_servo.Calibration())
    assert descriptor["calibration"]["calibration_source"] == "estimate"


def test_metric_commands_become_normalised_stick_values():
    """The conversion this whole file exists for. `publish_cmd` takes the
    joystick's `[-1, 1]`, the protocol is m/s, and getting the factor wrong is a
    robot travelling at the wrong speed with every counter reporting success."""
    calibration = loco_servo.Calibration({"full_scale_vx_mps": 0.5,
                                          "full_scale_vy_mps": 0.4,
                                          "full_scale_wz_rads": 2.0})
    assert calibration.to_normalised(0.25, -0.2, 1.0) == (0.5, -0.5, 0.5)


def test_normalised_output_is_clamped_to_full_stick():
    """A value past 1.0 would mean the descriptor and the conversion disagree,
    and sending it on lets the SDK decide what an out-of-range stick means."""
    calibration = loco_servo.Calibration({"full_scale_vx_mps": 0.5})
    assert calibration.to_normalised(9.0, 0.0, 0.0)[0] == 1.0
    assert calibration.to_normalised(-9.0, 0.0, 0.0)[0] == -1.0


def test_a_zero_calibration_value_is_refused_rather_than_read_as_no_limit():
    """A zero full scale divides by zero; a zero deadband claims this legged
    robot can creep, which is the claim that makes small corrections vanish."""
    with pytest.raises(loco_servo.CalibrationError):
        loco_servo.Calibration({"full_scale_vx_mps": 0})
    with pytest.raises(loco_servo.CalibrationError):
        loco_servo.Calibration({"min_wz_rads": 0})


def test_a_deadband_at_or_above_its_ceiling_is_refused():
    """It would leave one commandable speed, which is a switch and not a
    controller — and the policy's lifted value would then be rejected on the
    hard limit, which reads as 'every command refused'."""
    with pytest.raises(loco_servo.CalibrationError) as excinfo:
        loco_servo.Calibration({"min_vx_mps": 0.6, "full_scale_vx_mps": 0.5})
    assert "min_vx_mps" in str(excinfo.value)


def test_an_unknown_calibration_source_is_refused():
    with pytest.raises(loco_servo.CalibrationError):
        loco_servo.Calibration({"calibration_source": "probably fine"})


# ── the command path ─────────────────────────────────────────────────────────

def test_an_ordinary_command_reaches_the_sdk_on_three_axes():
    loco = FakeLoco()
    card = _card(loco=loco, full_scale_vx_mps=0.5, full_scale_vy_mps=0.5,
                 full_scale_wz_rads=1.0)
    outcome = _sink(card).submit(_command([0.1, 0.05, 0.0, 0.0, 0.0, -0.3]))
    assert outcome.verdict is Verdict.APPLIED
    assert loco.published == [(0.2, 0.1, -0.3)]


def test_wz_is_taken_from_index_five_not_index_two():
    """An off-by-one here turns a turn command into a vertical one and back,
    which on a chassis reads as 'the robot ignores yaw' rather than as a bug."""
    loco = FakeLoco()
    card = _card(loco=loco, full_scale_wz_rads=1.0)
    _sink(card).submit(_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.5]))
    assert loco.published == [(0.0, 0.0, 0.5)]


def test_a_command_on_a_pinned_axis_is_rejected_not_ignored():
    """The reason the pinned axes are declared at all. A policy that believes it
    commands vertical motion must be told, not quietly two-thirds obeyed."""
    loco = FakeLoco()
    card = _card(loco=loco)
    outcome = _sink(card).submit(_command([0.1, 0.0, 0.5, 0.0, 0.0, 0.0]))
    assert outcome.verdict is Verdict.REJECTED
    assert loco.published == []


def test_an_over_speed_command_is_rejected():
    loco = FakeLoco()
    card = _card(loco=loco, full_scale_vx_mps=0.5)
    outcome = _sink(card).submit(_command([5.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert outcome.verdict is Verdict.REJECTED
    assert loco.published == []


def test_dry_run_reaches_neither_the_sdk_nor_the_repeat_thread():
    loco = FakeLoco()
    card = _card(loco=loco, dry_run=True)
    _sink(card).submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    card._hold()
    assert loco.published == []
    assert card._applied == 1          # it still counted, so info() is truthful


def test_dry_run_is_on_by_default_because_nothing_has_measured_this_chassis():
    """The opposite of R1's default, and the difference is the reason.

    R1's `dry_run` defaults off because a deployed chassis that silently refuses
    to move is worse than one that moves — `applied: 33, refused: 0` on screen
    while the robot stood still cost an afternoon there. That holds once the
    numbers are right. Here `x = 1.0` has never been measured on a Bumi, so the
    first connection has to be an observation of what the card wants to send.
    """
    card = loco_servo.LocoServoPlugin({}, "bumi", None, FakeHighCtrl())
    assert card._dry_run is True
    assert card.get_tool()["configSchema"]["properties"]["dry_run"]["default"] is True


# ── holding is an action, and it is a repeated one ───────────────────────────

def test_hold_commands_zero_rather_than_doing_nothing():
    """An arm holds by being left alone; a chassis given a velocity keeps
    travelling."""
    loco = FakeLoco()
    card = _card(loco=loco)
    card._hold()
    assert loco.published == [(0.0, 0.0, 0.0)]


def test_hold_also_clears_the_standing_target():
    """The Bumi-specific half. R1's `StopMove` latches; here a repeat thread is
    still sending whatever was accepted last, so a hold that only stopped
    *accepting* would leave the robot walking at its last commanded speed."""
    card = _card(loco=FakeLoco(), full_scale_vx_mps=0.5)
    _sink(card).submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert card._target != (0.0, 0.0, 0.0)
    card._hold()
    assert card._target == (0.0, 0.0, 0.0)


def test_the_repeat_thread_keeps_sending_the_standing_target():
    """A Bumi command is one DDS message and does not persist — the vendor's own
    example republishes at 50 Hz. Without this the robot takes one step per
    policy tick."""
    loco = FakeLoco()
    card = _card(loco=loco, full_scale_vx_mps=0.5)
    card._target = (0.4, 0.0, 0.0)
    card._start_repeat()
    try:
        deadline = time.monotonic() + 1.0
        while len(loco.published) < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        card._stop_repeat()
    assert len(loco.published) >= 3
    assert set(loco.published) == {(0.4, 0.0, 0.0)}


def test_the_repeat_thread_publishes_nothing_in_dry_run():
    loco = FakeLoco()
    card = _card(loco=loco, dry_run=True)
    card._target = (0.4, 0.0, 0.0)
    card._start_repeat()
    try:
        time.sleep(0.15)
    finally:
        card._stop_repeat()
    assert loco.published == []


def test_a_refused_command_stops_the_robot_rather_than_leaving_it_walking():
    """The failure that only exists because the target persists: if a posture
    refusal merely declined to update the target, the repeat thread would keep
    driving the robot at the last speed it accepted."""
    loco = FakeLoco()
    high_ctrl = FakeHighCtrl()
    card = _card(high_ctrl=high_ctrl, loco=loco, full_scale_vx_mps=0.5)
    sink = _sink(card)
    sink.submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], seq=1))
    assert card._target == (0.2, 0.0, 0.0)

    high_ctrl.mode = 30                      # disabled
    card._mode_checked_at = float("-inf")    # expire the cache, as time would
    sink.submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], seq=2))
    assert card._target == (0.0, 0.0, 0.0)
    assert loco.published[-1] == (0.0, 0.0, 0.0)


# ── posture ──────────────────────────────────────────────────────────────────

def test_starting_does_not_depend_on_posture():
    """Starting is a wiring event, not a motion one. Refusing while the robot
    happens to be lying down blocks the whole canvas over a fact that says
    nothing about whether the wiring is right."""
    card = _card(high_ctrl=FakeHighCtrl(mode=30))
    result = card.dispatch("start", {"input_topic": "/nav/cmd"})
    assert "workmode" not in (result.get("message") or "")


def test_a_command_is_refused_while_the_robot_is_not_walking():
    loco = FakeLoco()
    card = _card(high_ctrl=FakeHighCtrl(mode=30), loco=loco)
    _sink(card).submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert card._refused == 1
    assert "stand_up" in card.dispatch("info", {})["posture_problem"]
    assert (0.0, 0.0, 0.0) in loco.published     # and the chassis was stopped


def test_protection_mode_says_protection_rather_than_just_the_number():
    """26 is the mode an operator has to recover from differently, and being
    told 'workmode=26' sends them looking for a stand_up that will be refused."""
    card = _card(high_ctrl=FakeHighCtrl(mode=loco_servo.PROTECTION_MODE))
    _sink(card).submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert "保护模式" in card.dispatch("info", {})["posture_problem"]


def test_a_command_is_refused_when_the_workmode_cannot_be_read():
    """Acting on a failed read is how a safe call becomes a fall."""
    loco = FakeLoco()
    card = _card(high_ctrl=FakeHighCtrl(raises=True), loco=loco)
    _sink(card).submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert card._refused == 1
    assert loco.published == [(0.0, 0.0, 0.0)]


def test_walking_later_lets_commands_through():
    """The case a start-time gate gets wrong: posture changes after start."""
    loco = FakeLoco()
    high_ctrl = FakeHighCtrl(mode=30)
    card = _card(high_ctrl=high_ctrl, loco=loco, full_scale_vx_mps=0.5)
    sink = _sink(card)
    sink.submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], seq=1))
    loco.published.clear()

    high_ctrl.mode = loco_servo.WALKING_MODE
    card._mode_checked_at = float("-inf")
    sink.submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], seq=2))
    assert loco.published == [(0.2, 0.0, 0.0)]
    assert card._refused == 0


def test_the_workmode_is_not_read_once_per_command():
    """At 10 Hz a DDS read per command puts a round trip to the robot's own
    controller in the path of every velocity."""
    high_ctrl = FakeHighCtrl()
    card = _card(high_ctrl=high_ctrl, loco=FakeLoco())
    sink = _sink(card)
    for seq in range(1, 6):
        sink.submit(_command([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seq=seq))
    assert high_ctrl.mode_reads == 1


def test_require_standing_off_skips_the_check_entirely():
    high_ctrl = FakeHighCtrl(mode=30)
    loco = FakeLoco()
    card = _card(high_ctrl=high_ctrl, loco=loco, require_standing=False,
                 full_scale_vx_mps=0.5)
    _sink(card).submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert loco.published == [(0.2, 0.0, 0.0)]
    assert high_ctrl.mode_reads == 0


# ── arbitration with the call-shaped cards ───────────────────────────────────

def test_starting_is_refused_while_loco_is_driving():
    card = _card(loco=FakeLoco(moving=True))
    result = card.dispatch("start", {"input_topic": "/nav/cmd"})
    assert result["state"] == "error"
    assert "stop_move" in result["message"]


def test_the_card_registers_itself_with_loco_so_it_can_be_preempted():
    loco = FakeLoco()
    card = _card(loco=loco)
    assert loco.servo is card


def test_being_preempted_stops_the_chassis_immediately():
    """Pausing without commanding zero would leave the repeat thread's last
    target standing — the robot keeps walking while the card reports paused."""
    loco = FakeLoco()
    card = _card(loco=loco, full_scale_vx_mps=0.5)
    _sink(card).submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    card._running = True
    assert card.pause_for_explicit_command("loco.move") is True
    assert card._target == (0.0, 0.0, 0.0)
    assert loco.published[-1] == (0.0, 0.0, 0.0)


def test_pause_for_explicit_command_reports_whether_it_did_anything():
    """The caller uses the return value to say it preempted something; a card
    that was already idle must not claim it was interrupted."""
    card = _card(loco=FakeLoco())
    assert card.pause_for_explicit_command("x") is False
    card._running = True
    assert card.pause_for_explicit_command("x") is True
    assert card.pause_for_explicit_command("x") is False   # already paused


# ── the operator's switches, settable at runtime ─────────────────────────────

def test_the_three_toggles_are_declared_in_the_config_schema():
    schema = _card().get_tool()["configSchema"]["properties"]
    assert set(schema) == {"dry_run", "rotate_only", "require_standing"}
    assert all(field["type"] == "boolean" for field in schema.values())


def test_config_takes_effect_while_the_card_is_streaming():
    """A toggle that reports success and changes nothing until a restart is the
    shape of failure this bundle keeps hitting — a setting that looks applied."""
    loco = FakeLoco()
    card = _card(loco=loco)
    card.dispatch("config", {"dry_run": True})
    _sink(card).submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert loco.published == []


def test_clearing_dry_run_hands_a_live_stream_to_the_motors():
    """The direction to be careful about, honoured deliberately: nothing else
    stands between this flag and the chassis once a stream is subscribed, so the
    operator's act is the authorisation."""
    loco = FakeLoco()
    card = _card(loco=loco, dry_run=True)
    card.dispatch("config", {"dry_run": False})
    _sink(card).submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert len(loco.published) == 1


def test_config_only_touches_keys_that_were_sent():
    """A form rendering an unchecked box for a field nobody set would otherwise
    send `require_standing: false` and silently drop a posture check."""
    card = _card(require_standing=True, rotate_only=True)
    card.dispatch("config", {"dry_run": True})
    assert card._require_standing is True and card._rotate_only is True


def test_config_ignores_keys_that_are_not_toggles():
    """`config` and `start` share an argument dict, so an unfiltered assignment
    would turn a stray `input_topic` into an attribute."""
    card = _card()
    card.dispatch("config", {"input_topic": "/x", "action": "config",
                             "dry_run": True})
    assert card._dry_run is True and card._input_topic == ""


def test_rotate_only_clamps_translation_instead_of_rejecting_the_command():
    """Pinning vx/vy in the descriptor would make the sink reject the *whole*
    command whenever the policy asked to move forward, taking the yaw with it.
    The point of this switch is that the robot still turns."""
    loco = FakeLoco()
    card = _card(loco=loco, rotate_only=True, full_scale_vx_mps=0.5,
                 full_scale_wz_rads=1.0)
    outcome = _sink(card).submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.4]))
    assert outcome.verdict is Verdict.APPLIED
    assert loco.published == [(0.0, 0.0, 0.4)]


# ── plumbing ─────────────────────────────────────────────────────────────────

def test_start_without_a_topic_says_what_to_wire():
    card = _card()
    assert "control/velocity" in card.dispatch("start", {})["message"]


def test_info_exposes_the_descriptor_the_dry_run_flag_and_the_calibration():
    """agent-core reads `control_interface` from here to hand it upstream. A
    card in dry run and a card with a dead chassis look identical from outside,
    and so do a measured calibration and a guessed one."""
    card = _card(dry_run=True)
    info = card.dispatch("info", {})
    assert info["control_interface"]["mode"] == "twist"
    assert info["control_interface"]["calibration"]["calibration_source"] == "estimate"
    assert info["dry_run"] is True
    assert info["state"] == "idle"


def test_the_card_declares_a_control_velocity_input():
    tool = _card().get_tool()
    assert tool["topic_in"][0]["format"] == "control/velocity"
    assert tool["inputSchema"]["x-resource"] == ["base"]
    assert tool["inputSchema"]["x-hooks"]["on_interrupt_motion"]["action"] == "pause"


def test_start_and_stop_are_not_offered_to_the_llm():
    """`start` needs an input topic the model does not have, so a model that
    stopped this card could not start it again."""
    params = _card().get_tool()["inputSchema"]["x-action-params"]
    assert set(params) == {"pause", "resume"}
