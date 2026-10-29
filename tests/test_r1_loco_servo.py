"""The R1 loco_servo card — the repo's first `motus.control/1` twist sink.

`ControlSink` is tested on its own in test_control_sink.py. This file covers what
is chassis-shaped, which is where a chassis-shaped mistake would be:

  - **the axes R1 does not have**, because `lower == upper == 0` is what turns a
    policy commanding vertical motion into a loud rejection instead of a third
    of its output vanishing
  - **the arbitration with `loco`**, because two cards writing `Move` on one
    `RpcProxy` is a stutter on the robot and nothing at all in the logs
  - **holding means commanding zero**, which is the one way a chassis differs
    from every arm card here: an arm holds by being left alone, a chassis
    keeps going
  - `dry_run` genuinely not reaching the SDK

No robot, no ROS, no Unitree SDK at module level, so this runs on a laptop —
deliberately, because a test that only runs on an R1 is a test that runs after
the robot has already moved.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_r1_loco_servo.py -q
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
    bundle = ROOT / "unitree" / "r1"
    sys.path.insert(0, str(bundle))
    try:
        spec = importlib.util.spec_from_file_location(
            "r1_loco_servo", bundle / "loco_servo.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules["r1_loco_servo"] = module
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(bundle))


loco_servo = _load()

from common.control import parse_descriptor  # noqa: E402
from common.control.sink import Verdict  # noqa: E402


class FakeClient:
    """Stands in for RpcProxy. Records instead of moving."""

    def __init__(self, fsm=loco_servo.STANDING_FSM, fsm_code=0):
        self.moves = []
        self.stops = 0
        self.fsm_reads = 0
        self._fsm = fsm
        self._fsm_code = fsm_code

    def Move(self, vx, vy, vyaw, _flag=True):
        self.moves.append((vx, vy, vyaw))
        return 0

    def StopMove(self):
        self.stops += 1
        return 0

    def GetFsmId(self):
        self.fsm_reads += 1
        return self._fsm_code, self._fsm


class FakeLoco:
    def __init__(self, moving=False):
        self._moving = moving
        self.servo = None
        self.paused_for = []

    def attach_servo(self, servo):
        self.servo = servo

    def is_moving(self):
        return self._moving


def _card(client=None, **config):
    config.setdefault("dry_run", False)
    return loco_servo.LocoServoPlugin(
        config, "r1", executor=None, loco_client=client or FakeClient(),
        loco_plugin=config.pop("loco_plugin", None))


def _command(values, seq=1, stamp_ms=None, ttl_ms=200):
    # Wall clock, because the sink's ttl and observation-age checks are against
    # the real one unless a clock is injected — a fixed literal here is simply
    # an expired command, which tests the wrong thing.
    stamp_ms = int(time.time() * 1000) if stamp_ms is None else stamp_ms
    return {"schema": "motus.control/1", "seq": seq, "stamp_ms": stamp_ms,
            "obs_stamp_ms": stamp_ms, "ttl_ms": ttl_ms, "source": "test",
            "priority": 50, "mode": "twist", "dof": 6, "values": list(values)}


# ── the descriptor ───────────────────────────────────────────────────────────

def test_the_descriptor_is_valid():
    parsed = parse_descriptor(loco_servo.build_descriptor())
    assert parsed.mode == "twist"
    assert parsed.dof == 6
    assert parsed.joint_names == ("vx", "vy", "vz", "wx", "wy", "wz")


def test_axis_order_matches_the_odometry_format():
    """Commanded and measured have to line up index by index, or every
    comparison a consumer makes is between two different axes."""
    from common.odom import AXES

    assert tuple(loco_servo.AXIS_NAMES) == AXES


def test_max_velocity_is_not_declared():
    """In a velocity space `max_velocity` would be jerk — a limit nobody can
    interpret is worse than an absent one. `max_delta_per_step` is the
    acceleration cap and is the field that does the work here."""
    limits = loco_servo.build_descriptor()["limits"]
    assert "max_velocity" not in limits
    assert limits["max_delta_per_step"][0] == loco_servo._step_limit(
        loco_servo.VX_ACCEL, loco_servo.MIN_VX)


def test_unactuatable_axes_are_pinned_to_zero():
    limits = loco_servo.build_descriptor()["limits"]
    for i in (2, 3, 4):          # vz, wx, wy
        assert limits["lower"][i] == 0.0 and limits["upper"][i] == 0.0


def test_force_torque_is_declared_null_rather_than_omitted():
    """A missing protection has to be visible; parse_descriptor requires it."""
    assert "force_torque" in loco_servo.build_descriptor()
    assert loco_servo.build_descriptor()["force_torque"] is None


# ── what the sink does with this descriptor ──────────────────────────────────

def _sink(card):
    from common.control import ControlSink
    return ControlSink(card._descriptor, card._apply,
                       on_watchdog=card._hold, on_abort=card._hold)


def test_a_command_on_a_pinned_axis_is_rejected_not_ignored():
    """The reason the pinned axes are declared at all. A policy that believes it
    commands vertical motion must be told, not quietly two-thirds obeyed."""
    client = FakeClient()
    card = _card(client)
    outcome = _sink(card).submit(_command([0.2, 0.0, 0.5, 0.0, 0.0, 0.0]))
    assert outcome.verdict is Verdict.REJECTED
    assert client.moves == []


def test_an_ordinary_command_reaches_the_chassis_on_three_axes():
    client = FakeClient()
    card = _card(client)
    outcome = _sink(card).submit(_command([0.2, 0.05, 0.0, 0.0, 0.0, -0.3]))
    assert outcome.verdict is Verdict.APPLIED
    assert client.moves == [(0.2, 0.05, -0.3)]


def test_wz_is_taken_from_index_five_not_index_two():
    """An off-by-one here turns a turn command into a vertical one and back,
    which on a chassis reads as 'the robot ignores yaw' rather than as a bug."""
    client = FakeClient()
    card = _card(client)
    _sink(card).submit(_command([0.0, 0.0, 0.0, 0.0, 0.0, 1.5]))
    assert client.moves == [(0.0, 0.0, 1.5)]


def test_an_over_velocity_command_is_rejected():
    client = FakeClient()
    card = _card(client)
    outcome = _sink(card).submit(_command([5.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert outcome.verdict is Verdict.REJECTED
    assert client.moves == []


# ── holding is an action here ────────────────────────────────────────────────

def test_hold_commands_a_stop_rather_than_doing_nothing():
    """The one way a chassis differs from every arm card in this repo: an arm
    holds by being left alone, a chassis given a velocity keeps travelling."""
    client = FakeClient()
    card = _card(client)
    card._hold()
    assert client.stops == 1


def test_dry_run_reaches_neither_move_nor_stop():
    client = FakeClient()
    card = _card(client, dry_run=True)
    _sink(card).submit(_command([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]))
    card._hold()
    assert client.moves == [] and client.stops == 0
    assert card._applied == 1          # it still counted, so info() is truthful


def test_dry_run_is_not_the_default():
    """This test used to assert the opposite, on the argument that a newly
    wired card must not drive a chassis the first time it is connected.

    The argument does not hold for this card. `start()` is inert; it acts only
    on a command stream somebody wired up, started a policy on, and gave a
    target to — three deliberate acts, each with its own gate. What the old
    default bought instead was a silently dead chassis on every freshly
    deployed robot, reporting `applied: 33, refused: 0, sdk_errors: 0` while
    standing still. That is indistinguishable from broken, and it cost an
    afternoon on r1_sz immediately after a deploy."""
    card = loco_servo.LocoServoPlugin({}, "r1", None, FakeClient())
    assert card._dry_run is False


# ── the operator's switches, settable at runtime ─────────────────────────────
#
# These lived only in config.yaml, so changing one meant editing a file inside a
# container and restarting the bundle. `dry_run` in particular is reached for in
# the second before a new policy is tried on a real robot, not after a redeploy.

def test_the_three_toggles_are_declared_in_the_config_schema():
    """Declared, or the frontend renders no form and they stay unreachable."""
    schema = _card().get_tool()["configSchema"]["properties"]
    assert set(schema) == {"dry_run", "rotate_only", "require_standing"}
    assert all(field["type"] == "boolean" for field in schema.values())


def test_the_declared_defaults_leave_the_robot_movable():
    """A default that cannot move is the failure `dry_run` already caused once.

    The schema's defaults are also what a form sends for a field nobody touched,
    so a `dry_run: true` default here would silently kill every freshly wired
    chassis — exactly what the old constructor default did.
    """
    schema = _card().get_tool()["configSchema"]["properties"]
    assert schema["dry_run"]["default"] is False
    assert schema["rotate_only"]["default"] is False
    assert schema["require_standing"]["default"] is True


def test_config_takes_effect_while_the_card_is_streaming():
    """A toggle that reports success and changes nothing until a restart is the
    shape of failure this bundle keeps hitting — a setting that looks applied."""
    client = FakeClient()
    card = _card(client)
    card.dispatch("config", {"dry_run": True})
    _sink(card).submit(_command([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert client.moves == []
    assert card._dry_run is True


def test_clearing_dry_run_hands_a_live_stream_to_the_motors():
    """The direction to be careful about, and it is honoured deliberately.

    Nothing else stands between this flag and the chassis once a stream is
    already subscribed, so the operator's act is the authorisation. It is logged
    for the same reason — a robot that starts moving with nothing in the log
    saying why is the worse outcome.
    """
    client = FakeClient()
    card = _card(client, dry_run=True)
    card.dispatch("config", {"dry_run": False})
    _sink(card).submit(_command([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert len(client.moves) == 1


def test_config_only_touches_keys_that_were_sent():
    """A form rendering an unchecked box for a field nobody set would otherwise
    send `require_standing: false` and silently drop a posture check."""
    card = _card(None, require_standing=True, rotate_only=True)
    card.dispatch("config", {"dry_run": True})
    assert card._require_standing is True
    assert card._rotate_only is True


def test_config_ignores_keys_that_are_not_toggles():
    """`config` and `start` share an argument dict on this card, so an
    unfiltered assignment would turn a stray `input_topic` into an attribute."""
    card = _card()
    card.dispatch("config", {"input_topic": "/x", "action": "config",
                             "_tool_name": "loco_servo", "dry_run": True})
    assert card._dry_run is True
    assert not hasattr(card, "_input_topic_") and card._input_topic == ""


def test_config_reports_what_is_now_in_force():
    card = _card()
    out = card.dispatch("config", {"rotate_only": True})
    assert out["ok"] is True
    assert out["rotate_only"] is True
    assert out["dry_run"] is False
    assert out["require_standing"] is True


def test_info_reports_all_three_so_the_card_is_readable():
    """What this card is enforcing right now must not have to be inferred from
    a config file that may no longer be what is in force."""
    card = _card()
    card.dispatch("config", {"require_standing": False})
    info = card.dispatch("info", {})
    assert info["require_standing"] is False
    assert "dry_run" in info and "rotate_only" in info


# ── arbitration with the call-shaped card ────────────────────────────────────

def test_starting_is_refused_while_loco_is_driving():
    loco = FakeLoco(moving=True)
    card = _card(FakeClient(), loco_plugin=loco)
    result = card.dispatch("start", {"input_topic": "/nav/cmd"})
    assert result["state"] == "error"
    assert "stop_move" in result["message"]


def test_the_card_registers_itself_with_loco_so_it_can_be_preempted():
    loco = FakeLoco()
    card = _card(FakeClient(), loco_plugin=loco)
    assert loco.servo is card


def test_pause_for_explicit_command_reports_whether_it_did_anything():
    """`loco` uses the return value to say it preempted something; a card that
    was already idle must not claim it was interrupted."""
    card = _card(FakeClient())
    assert card.pause_for_explicit_command("x") is False
    card._running = True
    assert card.pause_for_explicit_command("x") is True
    assert card._paused is True
    assert card.pause_for_explicit_command("x") is False   # already paused


# ── posture ──────────────────────────────────────────────────────────────────

def test_starting_does_not_depend_on_posture():
    """Starting is a wiring event, not a motion one.

    A project comes up when someone opens the canvas, and the robot is very
    often lying down at that moment. Refusing to start then blocks the whole
    canvas — every other card with it — over a posture that says nothing about
    whether the wiring is right, and that will very likely have changed by the
    time the first command arrives.
    """
    card = _card(FakeClient(fsm=1))          # damp, on the ground
    result = card.dispatch("start", {"input_topic": "/nav/cmd"})
    assert "FSM" not in (result.get("message") or "")


def test_a_command_is_refused_while_the_robot_is_lying_down():
    """The gate moved here — to the moment it can actually be answered."""
    client = FakeClient(fsm=1)
    card = _card(client)
    _sink(card).submit(_command([0.3, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert client.moves == []
    assert card._refused == 1
    assert "lie2standup" in card.dispatch("info", {})["posture_problem"]


def test_a_command_is_refused_when_the_fsm_cannot_be_read():
    """Acting on a failed read is how a safe call becomes a collapse — the same
    rule `switch_mode` already follows."""
    client = FakeClient(fsm_code=-1)
    card = _card(client)
    _sink(card).submit(_command([0.3, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert client.moves == []


def test_standing_up_later_lets_commands_through():
    """The case the start-time gate got wrong: posture changes after start."""
    client = FakeClient(fsm=1)
    card = _card(client)
    sink = _sink(card)
    sink.submit(_command([0.3, 0.0, 0.0, 0.0, 0.0, 0.0], seq=1))
    assert client.moves == []

    client._fsm = loco_servo.STANDING_FSM
    card._fsm_checked_at = float("-inf")     # expire the cache, as time would
    sink.submit(_command([0.3, 0.0, 0.0, 0.0, 0.0, 0.0], seq=2))
    assert client.moves == [(0.3, 0.0, 0.0)]
    assert card._refused == 0                # cleared once a command lands


def test_the_fsm_is_not_read_once_per_command():
    """At 10 Hz an RPC per command puts a round trip to the robot's own
    controller in the path of every velocity."""
    client = FakeClient()
    card = _card(client)
    sink = _sink(card)
    for seq in range(1, 6):
        sink.submit(_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], seq=seq))
    assert client.fsm_reads == 1
    assert len(client.moves) == 5


def test_require_standing_off_skips_the_check_entirely():
    client = FakeClient(fsm=1)
    card = _card(client, require_standing=False)
    _sink(card).submit(_command([0.3, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert client.moves == [(0.3, 0.0, 0.0)]
    assert client.fsm_reads == 0


# ── plumbing ─────────────────────────────────────────────────────────────────

def test_start_without_a_topic_says_what_to_wire():
    card = _card(FakeClient())
    assert "control/velocity" in card.dispatch("start", {})["message"]


def test_info_exposes_the_descriptor_and_the_dry_run_flag():
    """agent-core reads `control_interface` from here to hand it upstream, and
    an operator needs `dry_run` visible — a card in dry run and a card with a
    dead chassis look identical from outside."""
    card = _card(FakeClient(), dry_run=True)
    info = card.dispatch("info", {})
    assert info["control_interface"]["mode"] == "twist"
    assert info["dry_run"] is True
    assert info["state"] == "idle"


def test_the_prefix_has_no_underscore():
    """dispatch routes on partition("_"), so a prefix with one never matches."""
    assert "_" not in loco_servo.LocoServoPlugin.PREFIX


def test_start_and_stop_are_not_offered_to_the_model():
    """`start` needs an input topic the model does not have, so a model that
    stopped this card could not start it again."""
    params = _card(FakeClient()).get_tool()["inputSchema"]["x-action-params"]
    assert "start" not in params and "stop" not in params
    assert set(params) == {"pause", "resume"}


def test_the_interrupt_hooks_are_bound():
    hooks = _card(FakeClient()).get_tool()["inputSchema"]["x-hooks"]
    assert hooks["on_interrupt_motion"]["action"] == "pause"
    assert hooks["on_interrupt_all"]["action"] == "pause"


# ── rotate_only ──────────────────────────────────────────────────────────────

def test_rotate_only_zeroes_translation_but_keeps_the_turn():
    """The whole point of the switch. Pinning vx/vy in the descriptor instead
    would make the sink reject the entire command — yaw included — so the robot
    would not even turn, which is the opposite of what was asked for."""
    client = FakeClient()
    card = _card(client, rotate_only=True)
    _sink(card).submit(_command([0.3, 0.1, 0.0, 0.0, 0.0, -0.4]))
    assert client.moves == [(0.0, 0.0, -0.4)]


def test_rotate_only_counts_what_it_suppressed():
    """A card quietly dropping two thirds of every command is the failure this
    driver keeps warning about; it has to be visible from info()."""
    card = _card(FakeClient(), rotate_only=True)
    sink = _sink(card)
    sink.submit(_command([0.3, 0.0, 0.0, 0.0, 0.0, 0.0], seq=1))
    sink.submit(_command([0.3, 0.0, 0.0, 0.0, 0.0, 0.0], seq=2))
    info = card.dispatch("info", {})
    assert info["rotate_only"] is True
    assert info["suppressed_translations"] == 2


def test_rotate_only_does_not_count_commands_that_were_already_pure_yaw():
    card = _card(FakeClient(), rotate_only=True)
    _sink(card).submit(_command([0.0, 0.0, 0.0, 0.0, 0.0, -0.4]))
    assert card.dispatch("info", {})["suppressed_translations"] == 0


def test_rotate_only_is_off_by_default():
    assert _card(FakeClient()).dispatch("info", {})["rotate_only"] is False


def test_the_descriptor_declares_the_robots_deadband():
    """Measured on r1_sz one axis at a time: below these the robot does nothing,
    the SDK still returns 0, and every layer reports success. A policy that
    does not know about it emits a smooth ramp and never actuates."""
    limits = loco_servo.build_descriptor()["limits"]
    assert limits["min_magnitude"] == [0.4, 0.4, 0.0, 0.0, 0.0, 1.0]


def test_the_deadband_does_not_break_descriptor_parsing():
    parse_descriptor(loco_servo.build_descriptor())


def test_the_step_clamp_is_never_finer_than_the_deadband():
    """An acceleration cap below the floor is dead time, not a cap.

    The sink clamps the command the policy sends, so a 0.30 rad/s cap against a
    1.0 rad/s floor ramps 0.30 → 0.60 → 0.90 → 1.0 and the robot executes none of
    the first three. Three ticks of silence and then full speed, while every
    layer reports success — that is the lurch.
    """
    limits = loco_servo.build_descriptor()["limits"]
    for index, floor in enumerate(limits["min_magnitude"]):
        assert limits["max_delta_per_step"][index] >= floor, (
            f"axis {index} can be commanded in steps the robot cannot execute")


def test_a_ramp_from_rest_reaches_an_executable_speed_on_its_first_step():
    """The property the test above is really about, as the sink actually runs it."""
    from common.control.sink import ControlSink

    descriptor = loco_servo.build_descriptor()
    floor = descriptor["limits"]["min_magnitude"][5]
    applied = []
    sink = ControlSink(parse_descriptor(descriptor),
                       lambda values, _g=None: applied.append(values[5]))
    for seq in range(1, 4):
        sink.submit(_command([0.0, 0.0, 0.0, 0.0, 0.0, 1.5], seq=seq))
    assert abs(applied[0]) >= floor, (
        f"first step was {applied[0]:.2f} rad/s, under the {floor} floor — "
        "the robot would stand still for it")


# ── the footprint ────────────────────────────────────────────────────────────

def test_the_footprint_is_declared_in_metres_and_names_its_provenance():
    """A navigation card cannot build a corridor out of an angular camera slice:
    a fixed slice covers a different width at every distance, and at R1's
    numbers it is narrower than the robot below ~1.1 m — which is exactly where
    stopping matters. So the chassis declares its own envelope, same reasoning
    as `min_magnitude`.

    `source` is part of the contract, not decoration. A datasheet box and a
    measured one deserve different margins, and a consumer that cannot tell
    them apart will pick one number for both.
    """
    footprint = loco_servo.build_descriptor()["footprint"]
    assert footprint["shape"] == "box"
    assert footprint["source"] in ("vendor-spec", "measured", "estimate")
    for field in ("half_width", "front", "rear", "height"):
        assert footprint[field] > 0, field
    # 357 mm across, per Unitree's spec sheet.
    assert footprint["half_width"] == pytest.approx(0.1785, abs=0.002)


def test_the_footprint_does_not_pretend_to_cover_the_arms():
    """The problem that prompted this is a shoulder clipping a doorframe, and a
    raised arm leaves the torso box entirely. Declaring the box as if it were a
    clearance would hand the consumer a number that is wrong in the one
    direction that hurts."""
    assert loco_servo.build_descriptor()["footprint"]["arms"] == "at-rest"


def test_the_descriptor_still_parses_with_the_footprint_on_it():
    """It is an addition to `motus.control/1`, so every existing consumer has to
    keep working without knowing about it."""
    parse_descriptor(loco_servo.build_descriptor())


def test_the_yaw_deadband_is_declared_twice_because_it_is_not_one_number():
    """Standing, R1 does nothing under 1.0 rad/s. Walking, 0.05 rad/s is
    visible — twenty times smaller, because a gait cycle that is already
    running can be steered a little per step and one that has to be started
    cannot.

    A consumer that treats the standing figure as a constant overshoots every
    small correction mid-approach by that factor, reverses, and overshoots
    again. On r1_sz that looked like the robot weaving left and right on its
    way to a target it was already facing.
    """
    limits = loco_servo.build_descriptor()["limits"]
    standing = limits["min_magnitude"]
    moving = limits["min_magnitude_moving"]
    assert moving[5] < standing[5] / 10, "the whole point is that it collapses"
    assert moving[:2] == standing[:2], (
        "nothing has measured whether the translation floors move; inventing a "
        "smaller one would be the same mistake in the other direction")


def test_the_moving_floors_are_an_addition_the_sink_still_accepts():
    parse_descriptor(loco_servo.build_descriptor())


def test_a_deployed_chassis_can_move():
    """`dry_run` used to default on, so every freshly deployed robot had a
    silently dead chassis: commands arrive, pass every check, report APPLIED,
    and nothing moves — `applied: 33, refused: 0, sdk_errors: 0` while the robot
    stands there. Indistinguishable from broken, and it cost an afternoon on
    r1_sz right after a deploy.

    The argument for the old default does not hold for this card: `start()` is
    inert, and it acts only on a command stream somebody wired up, started a
    policy on, and gave a target to."""
    card = _card()
    assert card._dry_run is False
    assert card._rotate_only is False


def test_dry_run_is_still_available_and_still_announced():
    card = _card(dry_run=True)
    assert card._dry_run is True
    assert card._info()["dry_run"] is True


def test_a_swallowed_command_stream_is_visible_to_the_card_upstream():
    """A policy whose commands are being dropped looks exactly like one that is
    working. The descriptor is the only channel back, so it carries the fact."""
    plain = _card()._info()["control_interface"]
    assert "dry_run" not in plain and "rotate_only" not in plain

    muted = _card(dry_run=True, rotate_only=True)._info()["control_interface"]
    assert muted["dry_run"] is True and muted["rotate_only"] is True
    assert muted["mode"] == "twist", "still a valid descriptor"
