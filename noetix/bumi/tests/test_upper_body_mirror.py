"""The mirror card's rules, exercised off a robot.

`upper_body_mirror_spec` imports no ROS and no SDK, so most of this runs with
nothing installed. What is worth pinning down is not that the list has 21 items
but the four decisions that would fail *silently* on real hardware:

  * a vector that is not exactly 21 commands is rejected by the SDK, so the
    count and the `motor_id` sequence are a contract;
  * `MotorCmd.pos` is radians while every limit is degrees — converting twice
    or not at all still produces a plausible-looking number;
  * the fifteen joints the card does not track must hold their measured
    position, because "hold" is the only thing keeping the robot upright;
  * `-999` means "never measured", and commanding it would be a jump to an
    impossible angle.
"""
import math
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import upper_body_mirror_spec as spec  # noqa: E402


def _measured(*arm_radians):
    """21 measured positions, with the six arm axes set from the arguments.

    Order (matching `device.py`'s `_BUMI_JOINT_NAMES`): left pitch, roll, yaw,
    elbow at 0-3; right pitch, roll, yaw, elbow at 10-13. Everything else stays
    at a non-zero constant so a command that accidentally zeroes a leg shows up.
    """
    values = [0.5] * spec.JOINT_COUNT
    order = (0, 1, 3, 10, 11, 13)
    for index, value in zip(order, arm_radians):
        values[index] = float(value)
    return values


def _targets(**kwargs):
    targets = dict(spec.ZERO_TARGETS_DEG)
    targets.update(kwargs)
    return targets


# ── the 21-command contract ──────────────────────────────────────────────────

def test_the_vector_is_always_exactly_twenty_one_commands():
    """`set_joint` throws on anything else, so a short vector is a dead card."""
    commands = spec.build_joint_commands(
        _measured(*([0.0] * 6)), _targets(),
        keep_kp=10.0, keep_kd=0.5, arm_kp=5.0, arm_kd=0.5)
    assert len(commands) == 21
    assert [c["motor_id"] for c in commands] == list(range(21))


def test_a_measured_vector_of_the_wrong_length_is_rejected():
    with pytest.raises(ValueError):
        spec.build_joint_commands([0.0] * 20, _targets(),
                                  keep_kp=10.0, keep_kd=0.5,
                                  arm_kp=5.0, arm_kd=0.5)


def test_the_fifteen_untracked_joints_hold_their_measured_position():
    """A leg sent to zero would fold the robot; it must be sent where it is."""
    measured = _measured(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    commands = spec.build_joint_commands(
        measured, _targets(),
        keep_kp=10.0, keep_kd=0.5, arm_kp=5.0, arm_kd=0.5)
    for index in range(21):
        if index in spec.ARM_INDEXES:
            continue
        assert commands[index]["pos"] == measured[index]
        assert commands[index]["kp"] == 10.0
        assert commands[index]["kd"] == 0.5


def test_shoulder_yaw_is_never_written_even_when_a_target_exists():
    """Humeral rotation is unobservable; writing an invented value turns the arm.

    The tracker reports yaw as zero because it cannot see the elbow crease. A
    zero written here would be a real command to a joint nobody measured, so the
    two yaw indices must keep the position the motor reports.
    """
    measured = _measured(0.1, 0.2, 0.7, 0.4, 0.5, 0.6)
    targets = _targets(l_pitch=45.0, r_pitch=45.0)
    # Even if an upstream were to send a yaw field, the spec has no index for it.
    targets["l_yaw"] = 88.0
    targets["r_yaw"] = 88.0
    commands = spec.build_joint_commands(
        measured, targets,
        keep_kp=10.0, keep_kd=0.5, arm_kp=5.0, arm_kd=0.5)
    # `_measured` writes the six observable axes; yaw keeps the filler value.
    assert commands[2]["pos"] == pytest.approx(0.5)
    assert commands[12]["pos"] == pytest.approx(0.5)
    assert commands[2]["kp"] == 10.0


def test_the_six_arm_joints_use_the_tracking_gains():
    commands = spec.build_joint_commands(
        _measured(*([0.0] * 6)), _targets(l_pitch=30.0),
        keep_kp=10.0, keep_kd=0.5, arm_kp=5.0, arm_kd=0.4)
    assert commands[0]["kp"] == 5.0
    assert commands[0]["kd"] == 0.4
    assert commands[10]["kp"] == 5.0


# ── units ────────────────────────────────────────────────────────────────────

def test_degrees_become_radians_exactly_once():
    """A double conversion is off by 57x and still looks like a joint angle."""
    commands = spec.build_joint_commands(
        _measured(*([0.0] * 6)), _targets(l_pitch=90.0),
        keep_kp=10.0, keep_kd=0.5, arm_kp=5.0, arm_kd=0.5)
    assert commands[0]["pos"] == pytest.approx(math.pi / 2.0, abs=1e-9)


def test_measured_radians_come_back_as_degrees():
    angles = spec.measured_arm_deg(_measured(math.pi / 2.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0))
    assert angles["l_pitch"] == pytest.approx(90.0, abs=1e-9)
    assert angles["l_roll"] == pytest.approx(0.0, abs=1e-9)


# ── limits ───────────────────────────────────────────────────────────────────

def test_the_elbow_never_fully_straightens():
    """The upper stop is 0 deg (straight); the margin keeps it slightly bent.

    A fully extended elbow is the pose closest to a singularity, and it is also
    where an overshoot has no structure left to absorb it.
    """
    assert spec.clamp_arm("left", "elbow", 0.0) == pytest.approx(-10.0)
    assert spec.clamp_arm("left", "elbow", 40.0) == pytest.approx(-10.0)
    assert spec.clamp_arm("left", "elbow", -200.0) == pytest.approx(-119.0)


def test_the_two_arms_have_mirrored_roll_limits():
    """-100 is out of range on the left and legal on the right.

    The margin is applied *inward*: the left roll range is (-8, 111), so the
    lower stop becomes 2, not -18. Reading the sign of the margin wrong here is
    how a clamp silently allows the arm to cross its own body.
    """
    assert spec.clamp_arm("left", "roll", -100.0) == pytest.approx(2.0)
    assert spec.clamp_arm("right", "roll", -100.0) == pytest.approx(-100.0)
    assert spec.clamp_arm("right", "roll", 100.0) == pytest.approx(-2.0)


def test_a_margin_wide_enough_to_invert_a_range_collapses_to_the_midpoint():
    low, high = spec._effective_range("right", "roll", 80.0)
    assert low == high == pytest.approx((-111.0 + 80.0 + 8.0 - 80.0) / 2.0)


# ── missing and unmeasured axes ──────────────────────────────────────────────

def test_an_unmeasured_arm_holds_instead_of_driving_to_zero():
    """-999 is the tracker's 'never measured'; it is not a target angle."""
    measured = _measured(0.2, 0.3, 0.4, 0.5, 0.6, 0.7)
    targets = _targets(l_pitch=spec.UNMEASURED_DEG, r_pitch=20.0)
    commands = spec.build_joint_commands(
        measured, targets,
        keep_kp=10.0, keep_kd=0.5, arm_kp=5.0, arm_kd=0.5)
    assert commands[0]["pos"] == measured[0]
    assert commands[0]["kp"] == 10.0
    assert commands[10]["pos"] == pytest.approx(math.radians(20.0))


def test_a_missing_field_reads_as_unmeasured_not_as_zero():
    targets = spec.targets_from_payload({"tracked": True, "l_pitch_deg": 12.0})
    assert targets["l_pitch"] == pytest.approx(12.0)
    assert targets["l_roll"] == spec.UNMEASURED_DEG
    assert targets["r_elbow"] == spec.UNMEASURED_DEG


def test_a_non_numeric_field_is_unmeasured_rather_than_an_exception():
    targets = spec.targets_from_payload({"l_pitch_deg": None, "r_pitch_deg": "abc"})
    assert targets["l_pitch"] == spec.UNMEASURED_DEG
    assert targets["r_pitch"] == spec.UNMEASURED_DEG


def test_a_payload_that_is_not_an_object_is_rejected():
    with pytest.raises(ValueError):
        spec.targets_from_payload([1, 2, 3])


# ── rate limiting ────────────────────────────────────────────────────────────

def test_the_rate_limit_steps_toward_the_target_without_overshooting():
    assert spec.step_toward(0.0, 30.0, 0.6) == pytest.approx(0.6)
    assert spec.step_toward(0.0, 0.2, 0.6) == pytest.approx(0.2)
    assert spec.step_toward(0.0, -30.0, 0.6) == pytest.approx(-0.6)


def test_thirty_degrees_a_second_at_fifty_hertz_is_point_six_a_tick():
    assert spec.max_step_for(30.0, 50.0) == pytest.approx(0.6)


def test_the_same_rate_limit_moves_less_per_tick_at_a_higher_rate():
    slow = spec.max_step_for(30.0, 10.0)
    fast = spec.max_step_for(30.0, 50.0)
    assert slow > fast
    assert slow == pytest.approx(3.0)
    assert fast == pytest.approx(0.6)


def test_a_zero_rate_limit_is_rejected_rather_than_free_running():
    with pytest.raises(ValueError):
        spec.max_step_for(0.0, 50.0)


def test_the_limit_is_applied_to_every_axis_at_once():
    current = dict(spec.ZERO_TARGETS_DEG)
    desired = {key: 100.0 for key in spec.ZERO_TARGETS_DEG}
    stepped = spec.rate_limited_targets(current, desired, 0.6)
    assert all(value == pytest.approx(0.6) for value in stepped.values())


# ── first command ────────────────────────────────────────────────────────────

def test_the_first_command_is_the_pose_the_arm_is_already_in():
    """`start` must not move anything: enabling a card is not an instruction.

    The pose is chosen inside every effective range, because a measured angle
    that sits outside a limit *should* be clamped on the way out — that is the
    clamp doing its job, not a violation of this rule.
    """
    measured = _measured(0.3, 0.4, -0.5, 0.5, -0.4, -0.7)
    start_pose = spec.measured_arm_deg(measured)
    commands = spec.build_joint_commands(
        measured, start_pose,
        keep_kp=10.0, keep_kd=0.5, arm_kp=5.0, arm_kd=0.5)
    for index in spec.ARM_INDEXES:
        assert commands[index]["pos"] == pytest.approx(measured[index])


# ── the payload the tracker actually publishes ───────────────────────────────

def test_the_flattened_payload_fields_are_the_ones_the_card_reads():
    """Guards against the tracker renaming a field and the mirror silently idle."""
    payload = {
        "tracked": True,
        "l_pitch_deg": 1.0, "l_roll_deg": 2.0, "l_elbow_deg": 3.0,
        "r_pitch_deg": 4.0, "r_roll_deg": 5.0, "r_elbow_deg": 6.0,
    }
    targets = spec.targets_from_payload(payload)
    assert targets == {
        "l_pitch": 1.0, "l_roll": 2.0, "l_elbow": 3.0,
        "r_pitch": 4.0, "r_roll": 5.0, "r_elbow": 6.0,
    }


# ── the card itself, with ROS and the SDK stubbed out ────────────────────────

def _load_card_module(monkeypatch):
    """Import the plugin with `rclpy` and `std_msgs` stubbed.

    The spec tests above cannot see a bug in the control loop: a lost operator
    handled by freezing instead of drifting, or a stop that sends one more
    command. Those live in the plugin, so it has to be importable on a laptop.
    """
    import importlib.util

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _FakeNode
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = lambda **kw: None
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos_mod.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos_mod.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    std_msgs = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = lambda data="": types.SimpleNamespace(data=data)
    for name, module in (
        ("rclpy", types.ModuleType("rclpy")),
        ("rclpy.node", node_mod), ("rclpy.qos", qos_mod),
        ("std_msgs", std_msgs), ("std_msgs.msg", std_msg),
    ):
        monkeypatch.setitem(sys.modules, name, module)

    path = Path(__file__).resolve().parent.parent / "upper_body_mirror_card.py"
    module_spec = importlib.util.spec_from_file_location("upper_body_mirror_card", path)
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    return module


class _FakeNode:
    """Records subscriptions; enough behaviour for the plugin's lifecycle."""

    def __init__(self, name):
        self.name = name
        self.subscriptions = []
        self.destroyed = []
        self.errors = []

    def create_subscription(self, _type, topic, _callback, _qos):
        sub = types.SimpleNamespace(topic=topic)
        self.subscriptions.append(sub)
        return sub

    def destroy_subscription(self, sub):
        self.destroyed.append(sub.topic)
        if sub in self.subscriptions:
            self.subscriptions.remove(sub)

    def get_logger(self):
        return types.SimpleNamespace(error=self.errors.append)


class _FakeHighCtrl:
    """21 motors at a fixed pose, plus a workmode the test can change."""

    def __init__(self, mode=2, pose=None):
        self.mode = mode
        self.pose = pose if pose is not None else ([0.0] * 21)
        self.joint_reads = 0

    def get_mode(self):
        return self.mode

    def get_joint_state(self):
        self.joint_reads += 1
        return [types.SimpleNamespace(pos=value) for value in self.pose]


class _FakeLowCtrl:
    def __init__(self):
        self.calls = []

    def set_joint(self, commands):
        self.calls.append(commands)


def _plugin(monkeypatch, *, mode=2, pose=None, config=None):
    card = _load_card_module(monkeypatch)
    high = _FakeHighCtrl(mode=mode, pose=pose)
    low = _FakeLowCtrl()
    plugin = card.UpperBodyMirrorPlugin(
        config or {}, "bumi", types.SimpleNamespace(add_node=lambda *_: None),
        high_ctrl=high)
    # The SDK probe would try to import the ARM64 bindings; the fake replaces
    # both halves of `_ensure_low_level`.
    card._load_low_level = lambda: (lambda: low, _FakeMotorCmd)
    card._probe_low_level = lambda *_a, **_k: (True, "")
    plugin._low_ctrl = low
    plugin._motor_cmd_cls = _FakeMotorCmd
    return card, plugin, high, low


class _FakeMotorCmd:
    def __init__(self):
        self.pos = 0.0
        self.vel = 0.0
        self.tau = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.motor_id = 0


def test_the_tool_is_an_actuator_that_declares_the_motors_it_takes(monkeypatch):
    """`actuator` is what makes Agent Core ask for confirmation before moving.

    Bumi's other cards are processors and sensors, which Agent Core runs without
    a confirmation step. The type is therefore load-bearing, not cosmetic: it is
    the difference between "mirror my arms" prompting the operator and simply
    happening. `x-resource` names the physical hardware the card takes over so
    two cards cannot quietly fight for the same motors.
    """
    card = _load_card_module(monkeypatch)
    plugin = card.UpperBodyMirrorPlugin(
        {}, "bumi", types.SimpleNamespace(add_node=lambda *_: None))
    tool = plugin.get_tool()
    assert tool["name"] == "upper_body_mirror"
    assert tool["type"] == "actuator"
    assert tool["topic_out"] == []
    assert tool["inputSchema"]["x-resource"] == ["base", "arm_l", "arm_r"]
    assert set(tool["inputSchema"]["x-action-params"]) == {"start", "stop", "info"}
    assert tool["inputSchema"]["required"] == ["action"]


def test_start_refuses_in_protection_mode(monkeypatch):
    """Workmode 26 is the robot's own fault call; taking the motors undoes it."""
    _card, plugin, _high, low = _plugin(monkeypatch, mode=26)
    result = plugin.dispatch("start", {})
    assert result["state"] == "error"
    assert "protection" in result["error"]
    assert low.calls == []


def test_start_refuses_without_a_controller(monkeypatch):
    card = _load_card_module(monkeypatch)
    plugin = card.UpperBodyMirrorPlugin(
        {}, "bumi", types.SimpleNamespace(add_node=lambda *_: None), high_ctrl=None)
    result = plugin.dispatch("start", {})
    assert result["state"] == "error"
    assert "controller" in result["error"]


def test_start_begins_from_the_current_pose_without_moving(monkeypatch):
    """The first command must equal the measured pose, not the zero target.

    Each axis gets a value inside its own effective range, because a measured
    angle outside a limit is meant to be clamped — that is the clamp working.
    """
    pose = [0.5] * 21
    inside_range = {
        0: math.radians(20.0),    # l_pitch, +-125 after margin
        1: math.radians(20.0),    # l_roll, 2..101
        3: math.radians(-30.0),   # l_elbow, -119..-10
        10: math.radians(20.0),   # r_pitch
        11: math.radians(-20.0),  # r_roll, -101..-2
        13: math.radians(-30.0),  # r_elbow
    }
    for index, value in inside_range.items():
        pose[index] = value
    _card, plugin, _high, low = _plugin(monkeypatch, pose=pose)
    result = plugin.dispatch("start", {})
    try:
        assert result["state"] == "ready"
        import time
        deadline = 2.0
        started = time.monotonic()
        while not low.calls and time.monotonic() - started < deadline:
            time.sleep(0.01)
        assert low.calls, "control loop never sent a command"
        first = low.calls[0]
        assert len(first) == 21
        for index in spec.ARM_INDEXES:
            assert first[index].pos == pytest.approx(pose[index], abs=1e-3)
    finally:
        plugin.dispatch("stop", {})


def test_a_lost_operator_drifts_back_toward_zero(monkeypatch):
    """Freezing an arm in mid-air is the failure this replaces.

    Drift-back starts only after mirroring has actually begun — a card started
    with nobody in frame holds instead of lowering the arms. So the operator is
    delivered first, then the stream goes silent and the timeout expires.
    """
    import json
    import time

    _card, plugin, _high, low = _plugin(
        monkeypatch, pose=[0.0] * 21, config={"lost_timeout_s": 0.2})
    plugin.dispatch("start", {})
    try:
        payload = json.dumps({
            "tracked": True,
            "l_pitch_deg": 60.0, "l_roll_deg": 20.0, "l_elbow_deg": -30.0,
            "r_pitch_deg": 60.0, "r_roll_deg": -20.0, "r_elbow_deg": -30.0,
        })
        plugin._on_angles(types.SimpleNamespace(data=payload))
        time.sleep(0.2)
        assert low.calls, "control loop never sent a command"
        tracked_pose = low.calls[-1][0].pos
        assert tracked_pose > math.radians(1.0), "never tracked toward the operator"

        # No further frames: the operator has left. Wait past the 0.2 s timeout.
        time.sleep(0.6)
        settled = low.calls[-1][0].pos
        assert settled < tracked_pose, "arm did not drift back after the operator left"
        assert plugin.dispatch("info", {})["lost"] is True
    finally:
        plugin.dispatch("stop", {})


def test_a_card_started_with_nobody_in_frame_holds_its_pose(monkeypatch):
    """Lowering the arms before mirroring began is movement nobody asked for."""
    import time

    pose = [0.0] * 21
    pose[0] = math.radians(40.0)   # left shoulder held up by whatever came before
    _card, plugin, _high, low = _plugin(
        monkeypatch, pose=pose, config={"lost_timeout_s": 0.1})
    plugin.dispatch("start", {})
    try:
        time.sleep(0.3)  # well past the lost timeout, but nothing was ever tracked
        assert low.calls, "control loop never sent a command"
        assert low.calls[-1][0].pos == pytest.approx(math.radians(40.0), abs=1e-3)
        info = plugin.dispatch("info", {})
        assert info["lost"] is True
    finally:
        plugin.dispatch("stop", {})


def test_stop_releases_the_subscription_and_the_loop(monkeypatch):
    _card, plugin, _high, low = _plugin(monkeypatch)
    plugin.dispatch("start", {})
    node = plugin._node
    assert len(node.subscriptions) == 1
    plugin.dispatch("stop", {})
    assert node.destroyed == [plugin._default_angles_topic]
    assert plugin._angles_sub is None
    assert plugin.dispatch("info", {})["state"] == "idle"
    count = len(low.calls)
    import time
    time.sleep(0.1)
    assert len(low.calls) == count, "loop kept sending after stop"


def test_a_malformed_angles_frame_does_not_kill_the_loop(monkeypatch):
    _card, plugin, _high, _low = _plugin(monkeypatch)
    plugin.dispatch("start", {})
    try:
        plugin._on_angles(types.SimpleNamespace(data="not json"))
        plugin._on_angles(types.SimpleNamespace(data="[1,2,3]"))
        assert plugin.dispatch("info", {})["state"] == "running"
    finally:
        plugin.dispatch("stop", {})


def test_a_valid_frame_sets_the_targets(monkeypatch):
    _card, plugin, _high, _low = _plugin(monkeypatch)
    plugin.dispatch("start", {})
    try:
        payload = (
            '{"tracked": true, "l_pitch_deg": 10.0, "l_roll_deg": 10.0,'
            ' "l_elbow_deg": -10.0, "r_pitch_deg": 10.0, "r_roll_deg": 10.0,'
            ' "r_elbow_deg": -10.0}')
        plugin._on_angles(types.SimpleNamespace(data=payload))
        assert plugin._latest_tracked is True
        assert plugin._latest_targets["l_pitch"] == pytest.approx(10.0)
    finally:
        plugin.dispatch("stop", {})


def test_range_checks_raise_instead_of_clamping(monkeypatch):
    """A 300 deg/s limit is not 'fast', it is the limiter doing nothing."""
    card = _load_card_module(monkeypatch)
    executor = types.SimpleNamespace(add_node=lambda *_: None)
    with pytest.raises(ValueError):
        card.UpperBodyMirrorPlugin({"rate_limit_deg_s": 300.0}, "bumi", executor)
    with pytest.raises(ValueError):
        card.UpperBodyMirrorPlugin({"control_hz": 5.0}, "bumi", executor)
    with pytest.raises(ValueError):
        card.UpperBodyMirrorPlugin({"keep_kp": 0.0}, "bumi", executor)


# ── non-finite input and feedback ────────────────────────────────────────────

def test_a_nan_payload_axis_is_treated_as_unmeasured():
    """NaN compares False against every clamp bound, so it would reach the motor.

    `clamp_arm` uses `<` and `>`, both of which are False for NaN, so the value
    passes through untouched and lands in `MotorCmd.pos`. Treating it like
    "never measured" holds the joint instead.
    """
    targets = spec.targets_from_payload(
        {"l_pitch_deg": float("nan"), "r_pitch_deg": 10.0})
    assert targets["l_pitch"] == spec.UNMEASURED_DEG
    assert targets["r_pitch"] == pytest.approx(10.0)


def test_an_infinite_payload_axis_is_treated_as_unmeasured():
    targets = spec.targets_from_payload(
        {"l_pitch_deg": float("inf"), "r_pitch_deg": float("-inf")})
    assert targets["l_pitch"] == spec.UNMEASURED_DEG
    assert targets["r_pitch"] == spec.UNMEASURED_DEG


def test_non_finite_measured_positions_are_rejected():
    """A dead encoder or a torn DDS sample must stop the tick, not be held.

    Holding NaN would put it into all 21 commands, including the legs.
    """
    measured = _measured(*([0.0] * 6))
    measured[4] = float("nan")
    with pytest.raises(ValueError):
        spec.build_joint_commands(measured, _targets(),
                                  keep_kp=10.0, keep_kd=0.5,
                                  arm_kp=5.0, arm_kd=0.5)


def test_a_non_finite_target_holds_instead_of_reaching_the_motor():
    measured = _measured(0.2, 0.3, -0.4, 0.5, -0.6, -0.7)
    targets = _targets(l_pitch=float("nan"), r_pitch=float("inf"))
    commands = spec.build_joint_commands(
        measured, targets,
        keep_kp=10.0, keep_kd=0.5, arm_kp=5.0, arm_kd=0.5)
    assert commands[0]["pos"] == measured[0]
    assert commands[0]["kp"] == 10.0
    assert commands[10]["pos"] == measured[10]


# ── protection mode is not a start-time check ────────────────────────────────

def test_entering_protection_during_a_run_stops_commanding(monkeypatch):
    """Workmode 26 can arrive mid-run (a fall, a motor fault).

    Re-reading it only at `start` means the loop keeps writing set_joint after
    the robot has said stop — overriding the one safety decision it made alone.
    """
    import time

    _card, plugin, high, low = _plugin(monkeypatch, mode=2)
    plugin.dispatch("start", {})
    try:
        time.sleep(0.08)
        assert low.calls, "control loop never sent a command"
        high.mode = 26
        time.sleep(0.2)
        info = plugin.dispatch("info", {})
        assert info["state"] == "error"
        assert "protection" in info["last_error"]
        after = len(low.calls)
        time.sleep(0.2)
        assert len(low.calls) == after, "kept commanding after protection mode"
    finally:
        plugin.dispatch("stop", {})


def test_an_unreadable_workmode_stops_commanding(monkeypatch):
    """Same rule when the mode cannot be read: unknown is not permission."""
    import time

    _card, plugin, high, low = _plugin(monkeypatch, mode=2)
    plugin.dispatch("start", {})
    try:
        time.sleep(0.08)

        def _boom():
            raise RuntimeError("dds gone")

        high.get_mode = _boom
        time.sleep(0.2)
        info = plugin.dispatch("info", {})
        assert info["state"] == "error"
        assert "unreadable" in info["last_error"]
        after = len(low.calls)
        time.sleep(0.2)
        assert len(low.calls) == after
    finally:
        plugin.dispatch("stop", {})
