"""The RM75 servo card — the first driver to consume motus.control/1.

`ControlSink` is tested on its own in test_control_sink.py; this file covers
only what is specific to putting it on an arm, which is where the mistakes
would be arm-shaped rather than protocol-shaped:

  - the descriptor is derived from the *same* vendor limits `joint_control`
    enforces, so the two paths cannot disagree about what this arm can do
  - the descriptor is in radians and the SDK takes degrees, and the conversion
    happens once, at the boundary
  - the levers a model is given mid-motion are pause and stop, and pause holds
    rather than releases
  - nothing streams because a container restarted

No SDK, no ROS, no arm: the client is a fake that records calls.

Run: python3 -m pytest tests/test_rm75_servo.py -q
"""

from __future__ import annotations

import importlib.util
import math
import sys
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "realman" / "rm75_6f_v"
sys.path.insert(0, str(ROOT))


def _load(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, DRIVER / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


device = _load("realman_rm75_device", "device.py")
sys.modules["device"] = device          # servo.py imports `device`
servo = _load("realman_rm75_servo", "servo.py")

from common.control import Verdict  # noqa: E402


class FakeClient:
    def __init__(self, *, connected=True, motion_enabled=True):
        self.connected = connected
        self.motion_enabled = motion_enabled
        self.motion_gate = threading.Lock()
        self.calls: list[tuple] = []

    def command(self, method, *args):
        self.calls.append((method, args))
        return 0

    @property
    def movej_calls(self):
        return [args for method, args in self.calls if method == "rm_movej_canfd"]

    @property
    def stop_calls(self):
        return [args for method, args in self.calls if method == "rm_set_arm_slow_stop"]


def make_plugin(**client_kwargs):
    client = FakeClient(**client_kwargs)
    plugin = servo.RM75ServoPlugin(client, {}, namespace="rm75", ros2=None)
    return plugin, client


# ── descriptor ───────────────────────────────────────────────────────────────

def test_descriptor_is_derived_from_the_same_limits_joint_control_enforces():
    """Two paths to the same motors must not disagree about their limits."""
    d = servo.build_descriptor()

    assert d["dof"] == len(device.JOINT_NAMES)
    assert d["joint_names"] == list(device.JOINT_NAMES)
    for i, (low, high) in enumerate(device.JOINT_LIMITS_DEG):
        assert d["limits"]["lower"][i] == pytest.approx(math.radians(low))
        assert d["limits"]["upper"][i] == pytest.approx(math.radians(high))
    for i, speed in enumerate(device.JOINT_MAX_SPEED_DEG_S):
        assert d["limits"]["max_velocity"][i] == pytest.approx(math.radians(speed))


def test_descriptor_is_in_radians_matching_the_skeleton_this_driver_publishes():
    d = servo.build_descriptor()
    assert d["units"]["angle"] == "rad"


def test_step_limit_is_one_period_of_travel_at_rated_speed():
    d = servo.build_descriptor(expected_hz=50.0)
    for i, speed in enumerate(device.JOINT_MAX_SPEED_DEG_S):
        assert d["limits"]["max_delta_per_step"][i] == pytest.approx(
            math.radians(speed) / 50.0
        )


def test_force_torque_is_declared_null_not_omitted():
    """A missing protection must be visible, not absent."""
    d = servo.build_descriptor()
    assert "force_torque" in d
    assert d["force_torque"] is None


def test_descriptor_parses():
    from common.control import parse_descriptor
    parsed = parse_descriptor(servo.build_descriptor())
    assert parsed.mode == "joint_position"
    assert parsed.has_force_torque is False


def test_expected_hz_above_the_passthrough_ceiling_is_refused():
    with pytest.raises(ValueError):
        servo.RM75ServoPlugin(FakeClient(), {"servo": {"expected_hz": 500}},
                              namespace="rm75", ros2=None)


# ── the motion gate ──────────────────────────────────────────────────────────
def test_start_refuses_while_the_driver_is_read_only():
    """Frequent commands do not earn an exemption from RM_MOTION_ENABLED."""
    plugin, client = make_plugin(motion_enabled=False)
    result = plugin.dispatch("start", {"input_topic": "/x"})
    assert result["state"] == "error"
    assert "RM_MOTION_ENABLED" in result["message"]
    assert client.calls == []


def test_start_refuses_without_an_input_topic():
    plugin, _ = make_plugin()
    result = plugin.dispatch("start", {})
    assert result["state"] == "error"
    assert "input_topic" in result["message"]


def test_bundle_start_does_not_begin_streaming():
    """A restarted container must not come up moving an arm."""
    plugin, client = make_plugin()
    plugin.start()
    assert plugin.dispatch("info", {})["state"] == "idle"
    assert client.calls == []


# ── the tool definition ──────────────────────────────────────────────────────

def test_tool_consumes_control_joint_and_declares_its_resource():
    plugin, _ = make_plugin()
    definition = plugin.get_tools()[0]

    assert definition["type"] == "actuator"
    assert definition["topic_in"][0]["format"] == "control/joint"
    assert definition["inputSchema"]["x-resource"] == "arm"
    assert definition["inputSchema"]["x-is-dangerous"] is True


def test_interrupt_hooks_pause_this_card():
    plugin, _ = make_plugin()
    hooks = plugin.get_tools()[0]["inputSchema"]["x-hooks"]
    # Pause, not stop: the arm stops either way, but a card that unwired
    # itself could not be resumed by whoever interrupted it.
    assert hooks["on_interrupt_all"]["action"] == "pause"
    assert hooks["on_interrupt_motion"]["action"] == "pause"


def test_no_completion_is_declared():
    """A stream has no end; an open pending action would block every actuator."""
    plugin, _ = make_plugin()
    assert "x-completion" not in plugin.get_tools()[0]["inputSchema"]


def test_info_reports_the_descriptor_and_the_committed_window():
    plugin, _ = make_plugin()
    info = plugin.dispatch("info", {})
    assert info["control_interface"]["control_interface"] == "motus.control/1"
    assert info["committed_window_ms"] == pytest.approx(1000 / servo.DEFAULT_EXPECTED_HZ,
                                                        abs=1)


# ── radians in, degrees out ──────────────────────────────────────────────────

def test_apply_converts_to_degrees_at_the_sdk_boundary():
    plugin, client = make_plugin()
    values = tuple(math.radians(d) for d in (10.0, -20.0, 30.0, 0.0, 0.0, 0.0, 0.0))

    plugin._apply(values, None)

    method_args = client.movej_calls[0]
    degrees, follow = method_args[0], method_args[1]
    assert degrees == pytest.approx([10.0, -20.0, 30.0, 0.0, 0.0, 0.0, 0.0])
    assert follow is False              # low-follow; high follow needs <10 ms


def test_a_command_outside_the_vendor_limits_never_reaches_the_sdk():
    """End to end through the sink: rejection means no passthrough call at all."""
    from common.control import ControlSink

    plugin, client = make_plugin()
    clock = [1_000_000]
    sink = ControlSink(servo.build_descriptor(), plugin._apply,
                       clock=lambda: clock[0])

    beyond = math.radians(device.JOINT_LIMITS_DEG[1][1] + 10.0)   # joint2 over max
    outcome = sink.submit({
        "schema": "motus.control/1", "seq": 1, "stamp_ms": clock[0],
        "obs_stamp_ms": clock[0], "ttl_ms": 100, "source": "test",
        "priority": 50, "mode": "joint_position", "dof": 7,
        "values": [0.0, beyond, 0.0, 0.0, 0.0, 0.0, 0.0],
    })

    assert outcome.verdict is Verdict.REJECTED
    assert client.movej_calls == []


def test_a_valid_command_reaches_the_sdk_in_degrees():
    from common.control import ControlSink

    plugin, client = make_plugin()
    clock = [1_000_000]
    sink = ControlSink(servo.build_descriptor(), plugin._apply,
                       clock=lambda: clock[0])

    outcome = sink.submit({
        "schema": "motus.control/1", "seq": 1, "stamp_ms": clock[0],
        "obs_stamp_ms": clock[0], "ttl_ms": 100, "source": "test",
        "priority": 50, "mode": "joint_position", "dof": 7,
        "values": [math.radians(5.0)] + [0.0] * 6,
    })

    assert outcome.applied
    assert client.movej_calls[0][0][0] == pytest.approx(5.0)


# ── stopping ─────────────────────────────────────────────────────────────────

def test_watchdog_and_abort_both_route_to_the_vendor_slow_stop():
    plugin, client = make_plugin()
    plugin._slow_stop()
    assert len(client.stop_calls) == 1


def test_slow_stop_is_a_noop_when_disconnected():
    plugin, client = make_plugin(connected=False)
    plugin._slow_stop()
    assert client.stop_calls == []


# ── pause is the model's lever, not a start gate ─────────────────────────────

def test_pause_stops_applying_without_dropping_the_subscription():
    """`stop` is not an answer to "wait": only agent-core knows the input topic
    and the downstream descriptor, so a model that stopped this card could not
    start it again."""
    plugin, client = make_plugin()
    plugin._running = True
    plugin._input_topic = "/x"

    assert plugin.dispatch("pause", {})["state"] == "paused"
    assert plugin._input_topic == "/x"
    assert plugin.dispatch("info", {})["state"] == "paused"

    assert plugin.dispatch("resume", {})["state"] == "running"
    assert plugin.dispatch("info", {})["state"] == "running"


def test_a_command_arriving_during_a_pause_is_dropped_not_queued():
    """Applying it at resume would be a jump computed from a stale world."""
    plugin, client = make_plugin()
    plugin._running = True
    plugin._sink = _RecordingSink()
    plugin.dispatch("pause", {})

    plugin._on_message(_Message('{"values": [0, 0, 0, 0, 0, 0, 0]}'))

    assert plugin._sink.submitted == []


def test_pausing_a_card_that_is_not_running_is_not_an_error():
    plugin, _ = make_plugin()
    assert plugin.dispatch("pause", {})["state"] == "idle"


class _Message:
    def __init__(self, data):
        self.data = data


class _RecordingSink:
    def __init__(self):
        self.submitted = []

    def submit(self, payload):
        self.submitted.append(payload)
        raise AssertionError("a paused card must not reach the sink")
