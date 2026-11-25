"""The mirror card's rules, exercised off a robot.

`upper_body_mimic_spec` imports no ROS and no SDK, so most of this runs with
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

import upper_body_mimic_spec as spec  # noqa: E402


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
    elbow_low, elbow_high = spec.HARD_LIMIT_DEG["left"]["elbow"]
    bent = elbow_high - spec.LIMIT_MARGIN_DEG
    stop = elbow_low + spec.LIMIT_MARGIN_DEG
    assert spec.clamp_arm("left", "elbow", 0.0) == pytest.approx(bent)
    assert spec.clamp_arm("left", "elbow", 40.0) == pytest.approx(bent)
    assert spec.clamp_arm("left", "elbow", -200.0) == pytest.approx(stop)


def test_the_two_arms_have_mirrored_roll_limits():
    """-100 is out of range on the left and legal on the right.

    The margin is applied *inward*: the left roll lower stop moves up by the
    margin, not down. Reading the sign wrong lets the arm cross its own body.
    """
    left_low = spec.HARD_LIMIT_DEG["left"]["roll"][0] + spec.LIMIT_MARGIN_DEG
    right_high = spec.HARD_LIMIT_DEG["right"]["roll"][1] - spec.LIMIT_MARGIN_DEG
    assert spec.clamp_arm("left", "roll", -100.0) == pytest.approx(left_low)
    assert spec.clamp_arm("right", "roll", -100.0) == pytest.approx(-100.0)
    assert spec.clamp_arm("right", "roll", 100.0) == pytest.approx(right_high)


def test_a_margin_wide_enough_to_invert_a_range_collapses_to_the_midpoint():
    low_raw, high_raw = spec.HARD_LIMIT_DEG["right"]["roll"]
    low, high = spec._effective_range("right", "roll", 80.0)
    assert low == high == pytest.approx((low_raw + high_raw) / 2.0)


# ── missing and unmeasured axes ──────────────────────────────────────────────

def test_an_unmeasured_arm_returns_to_the_safe_pose():
    """-999 means 'never measured'; keeping the old command would freeze an
    arm in a pose the camera can no longer see."""
    measured = _measured(0.2, 0.3, 0.4, 0.5, 0.6, 0.7)
    targets = _targets(l_pitch=spec.UNMEASURED_DEG, r_pitch=20.0)
    commands = spec.build_joint_commands(
        measured, targets,
        keep_kp=10.0, keep_kd=0.5, arm_kp=5.0, arm_kd=0.5)
    assert commands[0]["pos"] == pytest.approx(math.radians(0.0))
    assert commands[0]["kp"] == 5.0
    assert commands[10]["pos"] == pytest.approx(math.radians(20.0))


def test_a_missing_field_returns_to_the_safe_pose():
    targets = spec.targets_from_payload({"tracked": True, "l_pitch_deg": 12.0})
    assert targets["l_pitch"] == pytest.approx(12.0)
    assert targets["l_roll"] == pytest.approx(0.0)
    assert targets["r_elbow"] == pytest.approx(0.0)


def test_a_non_numeric_field_returns_to_the_safe_pose():
    targets = spec.targets_from_payload({"l_pitch_deg": None, "r_pitch_deg": "abc"})
    assert targets["l_pitch"] == pytest.approx(0.0)
    assert targets["r_pitch"] == pytest.approx(0.0)


def test_a_payload_that_is_not_an_object_is_rejected():
    with pytest.raises(ValueError):
        spec.targets_from_payload([1, 2, 3])


# ── dynamics: PR #485's 2 rad/s and 6 rad/s² ─────────────────────────────────

def test_the_preview_bounds_are_used_in_degrees():
    assert spec.MAX_VELOCITY_RAD_S == pytest.approx(2.0)
    assert spec.MAX_ACCELERATION_RAD_S2 == pytest.approx(6.0)
    assert spec.MAX_VELOCITY_DEG_S == pytest.approx(math.degrees(2.0))
    assert spec.MAX_ACCELERATION_DEG_S2 == pytest.approx(math.degrees(6.0))


def test_acceleration_is_limited_before_velocity():
    dt = 0.02
    position, velocity = spec.step_with_accel(
        0.0, 0.0, 1000.0, dt, spec.MAX_VELOCITY_DEG_S,
        spec.MAX_ACCELERATION_DEG_S2)
    assert velocity == pytest.approx(spec.MAX_ACCELERATION_DEG_S2 * dt)
    # Trapezoidal integration: from rest, the tick's mean speed is v/2, not v.
    assert position == pytest.approx(0.5 * velocity * dt)


def test_velocity_never_exceeds_the_bound():
    velocity = 0.0
    position = 0.0
    for _ in range(200):
        position, velocity = spec.step_with_accel(
            position, velocity, 1000.0, 0.02,
            spec.MAX_VELOCITY_DEG_S, spec.MAX_ACCELERATION_DEG_S2)
        assert abs(velocity) <= spec.MAX_VELOCITY_DEG_S + 1e-9
    assert velocity == pytest.approx(spec.MAX_VELOCITY_DEG_S)


def test_the_axis_brakes_instead_of_overshooting_the_target():
    """A trapezoid must decelerate in time; a naive limiter slams the stop."""
    position, velocity = 0.0, spec.MAX_VELOCITY_DEG_S
    for _ in range(30):
        position, velocity = spec.step_with_accel(
            position, velocity, 30.0, 0.02,
            spec.MAX_VELOCITY_DEG_S, spec.MAX_ACCELERATION_DEG_S2)
        assert position <= 30.0 + 1e-9
    assert position == pytest.approx(30.0)


def test_reversing_direction_pays_the_acceleration_cost():
    """Instant reversal at the top of a swing is the jerk the bound removes."""
    position, velocity = 0.0, 0.0
    for _ in range(20):
        position, velocity = spec.step_with_accel(
            position, velocity, 1000.0, 0.02,
            spec.MAX_VELOCITY_DEG_S, spec.MAX_ACCELERATION_DEG_S2)
    assert velocity > 0.0, "the axis never got moving"
    back_position, back_velocity = spec.step_with_accel(
        position, velocity, 0.0, 0.02, spec.MAX_VELOCITY_DEG_S,
        spec.MAX_ACCELERATION_DEG_S2)
    assert back_velocity > 0.0, "velocity must not flip in one tick"
    assert back_position > position, "direction must not reverse instantly"


def test_the_limit_is_applied_to_every_axis_at_once():
    current = dict(spec.ZERO_TARGETS_DEG)
    desired = {key: 1000.0 for key in spec.ZERO_TARGETS_DEG}
    positions, velocities = spec.accel_limited_targets(current, {}, desired, 0.02)
    assert len(positions) == 6
    assert len(velocities) == 6
    assert all(value > 0.0 for value in positions.values())


def test_bounds_are_read_from_the_urdf_not_a_guessed_table():
    assert spec.URDF_LIMITS_LOADED is True, spec.URDF_LIMITS_ERROR
    assert spec.HARD_LIMIT_DEG["left"]["pitch"][0] == pytest.approx(
        math.degrees(-2.36), abs=0.1)
    assert spec.HARD_LIMIT_DEG["right"]["roll"][1] == pytest.approx(
        math.degrees(0.14), abs=0.1)
    # Shoulder yaw is one of the eight URDF joints even though the card holds it.
    assert "yaw" in spec.HARD_LIMIT_DEG["left"]


def test_trajectory_targets_are_clamped_inside_the_urdf_stops():
    clamped = spec.clamp_targets({"l_pitch": 1000.0, "l_roll": -1000.0})
    assert clamped["l_pitch"] == pytest.approx(
        spec.HARD_LIMIT_DEG["left"]["pitch"][1] - spec.LIMIT_MARGIN_DEG)
    assert clamped["l_roll"] == pytest.approx(
        spec.HARD_LIMIT_DEG["left"]["roll"][0] + spec.LIMIT_MARGIN_DEG)


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

    path = Path(__file__).resolve().parent.parent / "upper_body_mimic_card.py"
    module_spec = importlib.util.spec_from_file_location("upper_body_mimic_card", path)
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
    # Every ordinary test has already done the operator sign-off; the gate
    # itself has its own explicit regression below.
    settings = {"dds_contention_verified": True, **(config or {})}
    plugin = card.UpperBodyMimicPlugin(
        settings, "bumi", types.SimpleNamespace(add_node=lambda *_: None),
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
    plugin = card.UpperBodyMimicPlugin(
        {}, "bumi", types.SimpleNamespace(add_node=lambda *_: None))
    tool = plugin.get_tool()
    assert tool["name"] == "upper_body_mimic"
    assert tool["type"] == "actuator"
    assert tool["topic_out"] == []
    assert tool["inputSchema"]["x-resource"] == ["base", "arm_l", "arm_r"]
    assert set(tool["inputSchema"]["x-action-params"]) == {"start", "stop", "info"}
    assert tool["inputSchema"]["required"] == ["action"]


def test_start_refuses_outside_walking_mode(monkeypatch):
    """Only workmode=2 is permitted — protection and disabled are not 'close'."""
    _card, plugin, _high, low = _plugin(monkeypatch, mode=26)
    result = plugin.dispatch("start", {"confirm": True})
    assert result["state"] == "error"
    assert "only workmode=2" in result["error"]
    assert low.calls == []


def test_start_refuses_without_the_dds_sign_off(monkeypatch):
    """The operator must confirm Low/High coexistence before any motion."""
    card = _load_card_module(monkeypatch)
    high = _FakeHighCtrl(mode=2, pose=[0.0] * 21)
    plugin = card.UpperBodyMimicPlugin(
        {}, "bumi", types.SimpleNamespace(add_node=lambda *_: None), high_ctrl=high)
    result = plugin.dispatch("start", {"confirm": True})
    assert result["state"] == "error"
    assert "DDS contention not verified" in result["error"]


def test_start_refuses_when_the_urdf_limits_are_missing(monkeypatch):
    """Guessed mechanical stops are not acceptable on a real robot."""
    card = _load_card_module(monkeypatch)
    monkeypatch.setattr(card.spec, "URDF_LIMITS_LOADED", False)
    high = _FakeHighCtrl(mode=2, pose=[0.0] * 21)
    plugin = card.UpperBodyMimicPlugin(
        {"dds_contention_verified": True}, "bumi",
        types.SimpleNamespace(add_node=lambda *_: None), high_ctrl=high)
    result = plugin.dispatch("start", {"confirm": True})
    assert result["state"] == "error"
    assert "URDF limits unavailable" in result["error"]


def test_start_refuses_without_a_controller(monkeypatch):
    card = _load_card_module(monkeypatch)
    plugin = card.UpperBodyMimicPlugin(
        {}, "bumi", types.SimpleNamespace(add_node=lambda *_: None), high_ctrl=None)
    result = plugin.dispatch("start", {"confirm": True})
    assert result["state"] == "error"
    assert "controller" in result["error"]


def test_start_begins_from_the_current_pose_without_a_jump(monkeypatch):
    """The first command must start at the measured pose, not at zero.

    With a 6 rad/s² bound the first tick advances only one acceleration step;
    that is the smooth start the bound exists for, not the old "snap to zero".
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
    result = plugin.dispatch("start", {"confirm": True})
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
        one_step = spec.MAX_ACCELERATION_RAD_S2 / (plugin._control_hz ** 2)
        for index in spec.ARM_INDEXES:
            assert first[index].pos == pytest.approx(pose[index], abs=one_step + 1e-3)
    finally:
        plugin.dispatch("stop", {})


def test_a_lost_operator_drifts_back_toward_zero(monkeypatch):
    """Freezing an arm in mid-air is the failure this replaces.

    Deliver the operator first, then let the stream go silent and the timeout
    expire; the command must walk back to the safe pose rather than hold.
    """
    import json
    import time

    _card, plugin, _high, low = _plugin(
        monkeypatch, pose=[0.0] * 21, config={"lost_timeout_s": 0.2})
    plugin.dispatch("start", {"confirm": True})
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


def test_a_card_started_with_nobody_in_frame_returns_to_the_safe_pose(monkeypatch):
    """No tracking means no human command to keep: return to the safe pose."""
    import time

    pose = [0.0] * 21
    pose[0] = math.radians(40.0)   # left shoulder held up by whatever came before
    _card, plugin, _high, low = _plugin(
        monkeypatch, pose=pose, config={"lost_timeout_s": 0.1})
    plugin.dispatch("start", {"confirm": True})
    try:
        time.sleep(0.3)  # well past the lost timeout, nothing was ever tracked
        assert low.calls, "control loop never sent a command"
        assert low.calls[-1][0].pos < math.radians(30.0), \
            "arm did not move toward the safe pose"
        info = plugin.dispatch("info", {})
        assert info["lost"] is True
    finally:
        plugin.dispatch("stop", {})


def test_stop_releases_the_subscription_and_the_loop(monkeypatch):
    _card, plugin, _high, low = _plugin(monkeypatch)
    plugin.dispatch("start", {"confirm": True})
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
    plugin.dispatch("start", {"confirm": True})
    try:
        plugin._on_angles(types.SimpleNamespace(data="not json"))
        plugin._on_angles(types.SimpleNamespace(data="[1,2,3]"))
        assert plugin.dispatch("info", {})["state"] == "running"
    finally:
        plugin.dispatch("stop", {})


def test_a_valid_frame_sets_the_targets(monkeypatch):
    _card, plugin, _high, _low = _plugin(monkeypatch)
    plugin.dispatch("start", {"confirm": True})
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
    """A 100 rad/s bound is not 'fast', it is the limiter doing nothing."""
    card = _load_card_module(monkeypatch)
    executor = types.SimpleNamespace(add_node=lambda *_: None)
    with pytest.raises(ValueError):
        card.UpperBodyMimicPlugin({"max_velocity_rad_s": 100.0}, "bumi", executor)
    with pytest.raises(ValueError):
        card.UpperBodyMimicPlugin({"max_acceleration_rad_s2": 100.0}, "bumi", executor)
    with pytest.raises(ValueError):
        card.UpperBodyMimicPlugin({"control_hz": 5.0}, "bumi", executor)
    with pytest.raises(ValueError):
        card.UpperBodyMimicPlugin({"keep_kp": 0.0}, "bumi", executor)


# ── non-finite input and feedback ────────────────────────────────────────────

def test_a_nan_payload_axis_returns_to_the_safe_pose():
    """NaN compares False against every clamp bound, so it would reach the motor.

    `clamp_arm` uses `<` and `>`, both of which are False for NaN, so the value
    passes through untouched and lands in `MotorCmd.pos`. Treat it as lost
    tracking and drive to the safe pose instead.
    """
    targets = spec.targets_from_payload(
        {"l_pitch_deg": float("nan"), "r_pitch_deg": 10.0})
    assert targets["l_pitch"] == pytest.approx(0.0)
    assert targets["r_pitch"] == pytest.approx(10.0)


def test_an_infinite_payload_axis_returns_to_the_safe_pose():
    targets = spec.targets_from_payload(
        {"l_pitch_deg": float("inf"), "r_pitch_deg": float("-inf")})
    assert targets["l_pitch"] == pytest.approx(0.0)
    assert targets["r_pitch"] == pytest.approx(0.0)


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


def test_a_non_finite_target_goes_to_the_safe_pose_not_the_motor():
    measured = _measured(0.2, 0.3, -0.4, 0.5, -0.6, -0.7)
    targets = _targets(l_pitch=float("nan"), r_pitch=float("inf"))
    commands = spec.build_joint_commands(
        measured, targets,
        keep_kp=10.0, keep_kd=0.5, arm_kp=5.0, arm_kd=0.5)
    assert commands[0]["pos"] == pytest.approx(0.0)
    assert commands[0]["kp"] == 5.0
    assert commands[10]["pos"] == pytest.approx(0.0)


# ── workmode is not a start-time-only check ──────────────────────────────────

def test_leaving_walking_mode_during_a_run_stops_commanding(monkeypatch):
    """Any exit from workmode 2 can arrive mid-run (a fall, an estop).

    Re-reading it only at `start` means the loop keeps writing set_joint after
    the robot has said stop — overriding the one safety decision it made alone.
    """
    import time

    _card, plugin, high, low = _plugin(monkeypatch, mode=2)
    plugin.dispatch("start", {"confirm": True})
    try:
        time.sleep(0.08)
        assert low.calls, "control loop never sent a command"
        high.mode = 26
        time.sleep(0.2)
        info = plugin.dispatch("info", {})
        assert info["state"] == "error"
        assert "workmode left 2" in info["last_error"]
        after = len(low.calls)
        time.sleep(0.2)
        assert len(low.calls) == after, "kept commanding after workmode exit"
    finally:
        plugin.dispatch("stop", {})


def test_an_unreadable_workmode_stops_commanding(monkeypatch):
    """Same rule when the mode cannot be read: unknown is not permission."""
    import time

    _card, plugin, high, low = _plugin(monkeypatch, mode=2)
    plugin.dispatch("start", {"confirm": True})
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


def test_clamp_arm_rejects_a_non_finite_angle():
    """Returning NaN from a clamp function is worse than raising: it looks clamped."""
    with pytest.raises(ValueError):
        spec.clamp_arm("left", "pitch", float("nan"))
    with pytest.raises(ValueError):
        spec.clamp_arm("left", "pitch", float("inf"))


def test_measured_arm_deg_rejects_non_finite_feedback():
    measured = _measured(*([0.0] * 6))
    measured[0] = float("nan")
    with pytest.raises(ValueError):
        spec.measured_arm_deg(measured)


# ── lifecycle and low-confidence frames ──────────────────────────────────────

def test_repeated_start_stop_never_leaves_two_writers(monkeypatch):
    """A forgotten worker would keep writing set_joint into the next run."""
    import time

    _card, plugin, _high, low = _plugin(monkeypatch)
    workers = []
    for _ in range(5):
        plugin.dispatch("start", {"confirm": True})
        worker = plugin._worker
        assert worker is not None
        workers.append(worker)
        time.sleep(0.03)
        plugin.dispatch("stop", {})
        assert worker.is_alive() is False
        assert plugin._worker is None
    assert len({id(worker) for worker in workers}) == 5
    settled = len(low.calls)
    time.sleep(0.1)
    assert len(low.calls) == settled, "a stale worker kept commanding"


def test_a_low_confidence_frame_drives_back_to_the_safe_pose(monkeypatch):
    """`tracked=false` is not 'hold': it is lost tracking."""
    import json
    import time

    _card, plugin, _high, low = _plugin(monkeypatch)
    plugin.dispatch("start", {"confirm": True})
    try:
        person = json.dumps({
            "tracked": True,
            "l_pitch_deg": 60.0, "l_roll_deg": 20.0, "l_elbow_deg": -30.0,
            "r_pitch_deg": 60.0, "r_roll_deg": -20.0, "r_elbow_deg": -30.0,
        })
        plugin._on_angles(types.SimpleNamespace(data=person))
        time.sleep(0.2)
        assert low.calls
        tracked_pose = low.calls[-1][0].pos
        occluded = json.dumps({
            "tracked": False,
            "l_pitch_deg": 60.0, "l_roll_deg": 20.0, "l_elbow_deg": -30.0,
            "r_pitch_deg": 60.0, "r_roll_deg": -20.0, "r_elbow_deg": -30.0,
        })
        plugin._on_angles(types.SimpleNamespace(data=occluded))
        # The arm must first brake and then reverse; 0.8 s covers both phases
        # under the 6 rad/s² bound instead of mistaking braking for a hold.
        time.sleep(0.8)
        assert low.calls[-1][0].pos < tracked_pose, \
            "low-confidence frame kept the last human command"
    finally:
        plugin.dispatch("stop", {})


def test_info_reports_the_safety_sign_off(monkeypatch):
    _card, plugin, _high, _low = _plugin(monkeypatch)
    info = plugin.dispatch("info", {})
    assert info["dds_contention_verified"] is True
    assert info["urdf_limits_loaded"] is True
    assert info["walking_mode_ok"] is True
    assert info["max_velocity_rad_s"] == pytest.approx(2.0)
    assert info["max_acceleration_rad_s2"] == pytest.approx(6.0)


# ── dangerous-action gate (review High #1) ───────────────────────────────────

def test_the_tool_is_declared_dangerous_with_a_confirm_parameter(monkeypatch):
    """Agent Core only applies its confirmation gate to x-is-dangerous cards."""
    card = _load_card_module(monkeypatch)
    plugin = card.UpperBodyMimicPlugin(
        {}, "bumi", types.SimpleNamespace(add_node=lambda *_: None))
    schema = plugin.get_tool()["inputSchema"]
    assert schema["x-is-dangerous"] is True
    assert schema["properties"]["confirm"]["type"] == "boolean"
    assert "confirm" in schema["x-action-params"]["start"]["params"]
    assert "confirm" not in schema["x-action-params"]["stop"]["params"]


@pytest.mark.parametrize("args", [
    {},
    {"confirm": False},
    {"confirm": 1},
    {"confirm": "true"},
])
def test_start_without_a_literal_confirm_true_refuses_to_arm(monkeypatch, args):
    _card, plugin, _high, low = _plugin(monkeypatch)
    result = plugin.dispatch("start", args)
    assert result["state"] == "error"
    assert "confirm=true" in result["error"]
    assert low.calls == [], "an unconfirmed start must not touch the motors"
    assert plugin._worker is None


# ── trapezoidal braking (review High #2) ─────────────────────────────────────

def test_per_tick_acceleration_stays_within_bound_through_a_short_target():
    """A review found velocity was zeroed on arrival — one huge acceleration.

    Cruise toward a target only 1 degree away and check *every* tick's speed
    change. Snapping to rest on the final tick shows up here as a delta far
    beyond max_acc * dt; a real braking phase never exceeds it.
    """
    dt = 0.02
    limit = spec.MAX_ACCELERATION_DEG_S2
    position, velocity = 0.0, spec.MAX_VELOCITY_DEG_S
    ticks = 0
    for _ in range(200):
        next_position, next_velocity = spec.step_with_accel(
            position, velocity, 1.0, dt,
            spec.MAX_VELOCITY_DEG_S, limit)
        delta_acc = abs(next_velocity - velocity) / dt
        assert delta_acc <= limit + 1e-6, (
            f"tick {ticks}: deceleration {delta_acc:.1f} exceeds {limit:.1f}")
        assert abs(next_velocity) <= spec.MAX_VELOCITY_DEG_S + 1e-9
        assert abs(next_position - 1.0) <= abs(position - 1.0) + 1e-9, \
            "braking must not move away from the target"
        position, velocity = next_position, next_velocity
        ticks += 1
        if abs(position - 1.0) < 1e-9 and abs(velocity) < 1e-9:
            break
    assert position == pytest.approx(1.0, abs=1e-6)
    assert velocity == pytest.approx(0.0, abs=1e-6)


def test_a_long_swing_also_obeys_the_acceleration_bound_to_arrival():
    dt = 0.02
    limit = spec.MAX_ACCELERATION_DEG_S2
    position, velocity = 0.0, 0.0
    for _ in range(600):
        next_position, next_velocity = spec.step_with_accel(
            position, velocity, 40.0, dt,
            spec.MAX_VELOCITY_DEG_S, limit)
        assert abs(next_velocity - velocity) / dt <= limit + 1e-6
        position, velocity = next_position, next_velocity
        if abs(position - 40.0) < 1e-9 and abs(velocity) < 1e-9:
            break
    assert position == pytest.approx(40.0, abs=1e-6)
