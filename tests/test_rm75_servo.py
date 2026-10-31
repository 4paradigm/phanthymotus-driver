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
import time
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


def _stub_std_msgs():
    """`_publish_state` 里 `from std_msgs.msg import String` 要有东西可 import。

    这个文件的其余部分完全不碰 ROS（这是它能在笔记本上跑的理由），而状态发布
    是唯一需要一个消息类型的地方。装一个最小的替身，不装 ROS —— 和
    `test_tianyi_servo.py` 同一个做法。真有 `std_msgs` 时不覆盖它。
    """
    import types

    if "std_msgs.msg" in sys.modules:
        return
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = type("String", (), {"__init__": lambda self: None})
    std.msg = std_msg
    sys.modules.setdefault("std_msgs", std)
    sys.modules.setdefault("std_msgs.msg", std_msg)


device = _load("realman_rm75_device", "device.py")
sys.modules["device"] = device          # servo.py imports `device`
servo = _load("realman_rm75_servo", "servo.py")
_stub_std_msgs()

from common.control import Verdict  # noqa: E402


# `None` 在 FakeClient 里是「用默认值」，所以「这个 SDK 调用会失败」要另一个
# 哨兵 —— 否则 `force=None` 读起来像「没有力数据」而实际拿到的是默认值。
FAILS = object()


class FakeClient:
    def __init__(self, *, connected=True, motion_enabled=True,
                 gripper_state=None, force=None, arm_state=None):
        self.connected = connected
        self.motion_enabled = motion_enabled
        self.motion_gate = threading.Lock()
        self.calls: list[tuple] = []
        self.reads: list[str] = []
        # 默认：夹爪在线、使能、全开（actpos 1000 = 闭合度 0）。
        self.gripper_state = ({"status": 1, "enable_state": 1,
                               "actpos": servo.GRIPPER_POSITION_MAX, "mode": 1}
                              if gripper_state is None else gripper_state)
        # `force_data` 和 `zero_force_data` 刻意不同，于是一条测试能分辨驱动
        # 读的是哪一个 —— 读错那个会被工具自重触发，而这里的 force_data 就是
        # 一个「看着像撞上了」的大值。
        self.force = ({"force_data": [900.0] * 6,
                       "zero_force_data": [0.5, -0.5, 1.0, 0.0, 0.1, 0.0]}
                      if force is None else force)
        self.arm_state = ({"joint": [10.0, -20.0, 30.0, 0.0, 0.0, 0.0, 0.0],
                           "pose": [0.3, 0.1, 0.5, 0.0, 0.0, 0.0]}
                          if arm_state is None else arm_state)
        self.command_hook = None

    def command(self, method, *args):
        # 先记账再调 hook：hook 用来模拟厂商包装调用**卡住**，而一次已经发出去
        # 但还没返回的调用，在账上必须是「发生过」。顺序反了的话，测试在卡住
        # 期间看到的是一个空列表，于是「发没发」和「返回没返回」被混成一件事。
        self.calls.append((method, args))
        if self.command_hook is not None:
            self.command_hook(method, args)
        return 0

    def call(self, method, *args):
        self.reads.append(method)
        if method == "rm_get_gripper_state":
            if self.gripper_state is FAILS:
                raise RuntimeError("rm_get_gripper_state failed with code -1")
            return self.gripper_state
        if method == "rm_get_force_data":
            if self.force is FAILS:
                raise RuntimeError("rm_get_force_data failed with code -1")
            return self.force
        if method == "rm_get_current_arm_state":
            if self.arm_state is FAILS:
                raise RuntimeError("rm_get_current_arm_state failed with code -1")
            return self.arm_state
        raise AssertionError(f"unexpected SDK call: {method}")

    @property
    def movej_calls(self):
        return [args for method, args in self.calls if method == "rm_movej_canfd"]

    @property
    def stop_calls(self):
        return [args for method, args in self.calls if method == "rm_set_arm_slow_stop"]

    @property
    def gripper_calls(self):
        return [args for method, args in self.calls
                if method == "rm_set_gripper_position"]


def make_plugin(config=None, **client_kwargs):
    client = FakeClient(**client_kwargs)
    plugin = servo.RM75ServoPlugin(client, config or {}, namespace="rm75", ros2=None)
    return plugin, client


# ── descriptor ───────────────────────────────────────────────────────────────

def test_descriptor_is_derived_from_the_same_limits_joint_control_enforces():
    """Two paths to the same motors must not disagree about their limits."""
    d = servo.build_descriptor()

    # 7 臂 + 1 夹爪。这条断言此前是 `== len(JOINT_NAMES)`，也就是把「这张卡没有
    # 夹爪」当成了不变量 —— 那正是它要防的那个 bug。
    assert d["dof"] == len(device.JOINT_NAMES) + 1
    assert d["joint_names"] == list(device.JOINT_NAMES) + ["gripper"]
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
    # 两个时钟都用这个假时钟：下面的消息 stamp 也取自它。必须显式写 —— sink 默认
    # 用墙钟和 `stamp_ms` 比，因为真实链路上那个字段是另一个进程按 Unix 纪元打的。
    sink = ControlSink(servo.build_descriptor(), plugin._apply,
                       clock=lambda: clock[0], wall_clock=lambda: clock[0])

    beyond = math.radians(device.JOINT_LIMITS_DEG[1][1] + 10.0)   # joint2 over max
    outcome = sink.submit({
        "schema": "motus.control/1", "seq": 1, "stamp_ms": clock[0],
        "obs_stamp_ms": clock[0], "ttl_ms": 100, "source": "test",
        "priority": 50, "mode": "joint_position", "dof": 8,
        "values": [0.0, beyond, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    })

    assert outcome.verdict is Verdict.REJECTED
    assert client.movej_calls == []


def test_a_valid_command_reaches_the_sdk_in_degrees():
    from common.control import ControlSink

    plugin, client = make_plugin()
    clock = [1_000_000]
    # 两个时钟都用这个假时钟：下面的消息 stamp 也取自它。必须显式写 —— sink 默认
    # 用墙钟和 `stamp_ms` 比，因为真实链路上那个字段是另一个进程按 Unix 纪元打的。
    sink = ControlSink(servo.build_descriptor(), plugin._apply,
                       clock=lambda: clock[0], wall_clock=lambda: clock[0])

    outcome = sink.submit({
        "schema": "motus.control/1", "seq": 1, "stamp_ms": clock[0],
        "obs_stamp_ms": clock[0], "ttl_ms": 100, "source": "test",
        "priority": 50, "mode": "joint_position", "dof": 8,
        "values": [math.radians(5.0)] + [0.0] * 7,
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


# ── 夹爪 ─────────────────────────────────────────────────────────────────────
#
# 这一组里最重要的是极性。它在真机上**没有验证过**，而接反的后果是在物体周围
# 执行「张开」变成「夹紧」—— 危险的那个方向。所以这里钉死的是「代码实现的是
# 哪个约定」，让真机上发现接反时，改一处、这些断言一起翻转。

def test_gripper_closure_zero_is_open_and_one_is_closed():
    """SDK 的 position 是**开口度**，方向和闭合度相反。"""
    assert servo.closure_to_position(0.0) == device.GRIPPER_POSITION_MAX   # 张开
    assert servo.closure_to_position(1.0) == device.GRIPPER_POSITION_MIN   # 闭合


def test_gripper_mapping_round_trips():
    """两个换算必须互为逆运算 —— 它们是同一件事的两个方向，不能各自漂走。"""
    for closure in (0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0):
        position = servo.closure_to_position(closure)
        assert servo.position_to_closure(position) == pytest.approx(closure, abs=2e-3)


def test_gripper_mapping_clamps_rather_than_extrapolating():
    assert servo.closure_to_position(-5.0) == device.GRIPPER_POSITION_MAX
    assert servo.closure_to_position(5.0) == device.GRIPPER_POSITION_MIN
    assert servo.position_to_closure(10_000) == 0.0
    assert servo.position_to_closure(-10_000) == 1.0


def test_gripper_group_declares_its_own_unit_and_offset():
    d = servo.build_descriptor()
    groups = {g["name"]: g for g in d["groups"]}
    assert groups["arm"]["offset"] == 0
    assert groups["arm"]["count"] == len(device.JOINT_NAMES)
    assert groups["arm"]["unit"] == "rad"
    assert groups["gripper"]["offset"] == len(device.JOINT_NAMES)
    assert groups["gripper"]["count"] == 1
    # 混合单位靠 groups 说清楚；`units` 的一个映射描述不了两种。
    assert groups["gripper"]["unit"] == "normalized"
    assert d["units"]["normalized"] == "0-1"
    assert d["limits"]["lower"][-1] == 0.0
    assert d["limits"]["upper"][-1] == 1.0


def test_a_card_without_a_gripper_is_genuinely_seven_dimensions():
    """不是 8 维里悄悄忽略一维 —— 带夹爪的模型应当协商失败，而不是被执行一半。"""
    d = servo.build_descriptor(gripper=False)
    assert d["dof"] == len(device.JOINT_NAMES)
    assert "gripper" not in d["joint_names"]
    assert [g["name"] for g in d["groups"]] == ["arm"]
    assert "normalized" not in d["units"]


def test_detect_gripper_requires_both_online_and_enabled():
    """一只在线但没使能的夹爪收下指令之后不会动，所以两个标志都要。"""
    assert servo.detect_gripper({"status": 1, "enable_state": 1}) == "online"
    assert servo.detect_gripper({"status": 0, "enable_state": 1}) == ""
    assert servo.detect_gripper({"status": 1, "enable_state": 0}) == ""
    assert servo.detect_gripper(None) == ""
    assert servo.detect_gripper({}) == ""


def test_start_refuses_when_the_config_claims_a_gripper_the_arm_does_not_have():
    """拒绝，不降级 —— 悄悄降成 7 维会让带夹爪的模型协商通过然后那一维去向虚空。"""
    plugin, client = make_plugin(gripper_state={"status": 0, "enable_state": 0})
    plugin._ros2 = object()           # ROS 的检查在探测之前，这里要越过它
    plugin._subscribe = lambda topic: None
    result = plugin.dispatch("start", {"input_topic": "/x"})

    assert result["state"] == "error"
    assert "servo.gripper" in result["message"]
    # 关键：没有去动手臂。探测失败不该留下一条已经发出去的指令。
    assert client.calls == []


def test_gripper_false_skips_the_probe_entirely():
    plugin, client = make_plugin(config={"servo": {"gripper": False}},
                                gripper_state=FAILS)
    plugin._subscribe = lambda topic: None
    plugin._ros2 = object()
    result = plugin.dispatch("start", {"input_topic": "/x"})

    assert result["state"] == "running"
    assert result["gripper"] is False
    assert "rm_get_gripper_state" not in client.reads
    plugin.dispatch("stop", {})


def test_apply_sends_seven_joints_to_the_arm_and_the_eighth_to_the_gripper():
    plugin, client = make_plugin()
    plugin._gripper_enabled = True

    plugin._apply([math.radians(5.0)] + [0.0] * 6 + [0.75], None)

    # 手臂只拿到 7 个数，第 8 维没有被当成第 8 个关节发下去。
    degrees = client.movej_calls[0][0]
    assert len(degrees) == len(device.JOINT_NAMES)
    assert degrees[0] == pytest.approx(5.0)
    # 夹爪只进了信箱，这一拍还没有 SDK 调用 —— worker 才会发。
    assert client.gripper_calls == []
    assert plugin._gripper_wanted == pytest.approx(0.75)


def test_apply_does_not_block_on_the_gripper():
    """夹爪的 SDK 调用卡住时，手臂回路必须照常走。

    这是 worker 线程存在的全部理由：`hardware.py:command_trajectory` 记着这台
    硬件上 `block=0` 的调用会停在 Python/C 包装里。
    """
    plugin, client = make_plugin(config={"servo": {"gripper_max_hz": 1000}})
    plugin._gripper_enabled = True
    plugin._running = True
    entered = threading.Event()
    stuck = threading.Event()

    def hook(method, args):
        if method == "rm_set_gripper_position":
            entered.set()
            stuck.wait(timeout=5.0)        # 模拟厂商包装调用卡死

    client.command_hook = hook
    plugin._start_workers()
    try:
        plugin._apply([0.0] * 7 + [1.0], None)
        # **先确认前提成立。** worker 真的进了那个卡死的调用 —— 没有这一步，
        # 一个「worker 根本没在发指令」的回归会让这条测试空过，而它看起来
        # 仍然在证明非阻塞。
        assert entered.wait(timeout=2.0), "worker never reached the gripper call"

        # 手臂的下几拍不受影响：`_apply` 没有在夹爪上阻塞。
        for _ in range(3):
            plugin._apply([0.0] * 7 + [1.0], None)
        assert len(client.movej_calls) == 4
        # 而夹爪那一侧仍然只发出过一条、且还没返回。
        assert len(client.gripper_calls) == 1
    finally:
        stuck.set()
        plugin._stop_workers()


def _drain_gripper(plugin, client, *, expect, timeout=2.0):
    """等 worker 把信箱发出去。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if len(client.gripper_calls) >= expect:
            return
        time.sleep(0.005)


def test_worker_sends_the_latest_closure_once():
    plugin, client = make_plugin(config={"servo": {"gripper_max_hz": 1000}})
    plugin._gripper_enabled = True
    plugin._running = True
    plugin._start_workers()
    try:
        plugin._want_gripper(1.0)
        _drain_gripper(plugin, client, expect=1)
        assert client.gripper_calls[0][0] == device.GRIPPER_POSITION_MIN
        # 非阻塞 + timeout 0：厂商文档的「发送后立即返回」。
        assert client.gripper_calls[0][1] is False
        assert client.gripper_calls[0][2] == 0
    finally:
        plugin._stop_workers()


def test_changes_inside_the_deadband_do_not_reach_the_sdk():
    plugin, client = make_plugin(
        config={"servo": {"gripper_deadband": 0.2, "gripper_max_hz": 1000}})
    plugin._gripper_enabled = True
    plugin._running = True
    plugin._start_workers()
    try:
        plugin._want_gripper(0.5)
        _drain_gripper(plugin, client, expect=1)
        assert len(client.gripper_calls) == 1
        # 0.55 和已发的 0.5 差 0.05 < 死区 0.2 —— 夹爪是控制器后面的串行设备，
        # 30 Hz 里的浮点噪声全发下去只会占满那条链路。
        plugin._want_gripper(0.55)
        time.sleep(0.05)
        assert len(client.gripper_calls) == 1
        # 越过死区的变化照常发。
        plugin._want_gripper(0.9)
        _drain_gripper(plugin, client, expect=2)
        assert len(client.gripper_calls) == 2
    finally:
        plugin._stop_workers()


def test_gripper_rate_is_capped_and_the_cap_is_reported():
    """被限掉的频率必须报出来 —— README_dev 的 "no silent caps"。"""
    plugin, _ = make_plugin(
        config={"servo": {"gripper_max_hz": 4.0, "gripper_deadband": 0.3}})
    info = plugin.dispatch("info", {})["gripper"]
    assert info["max_hz"] == 4.0
    assert info["deadband"] == 0.3
    # 极性未验证这件事也要在 info 里看得见。
    assert "UNVERIFIED" in info["polarity"]


def test_watchdog_holds_the_gripper_by_default():
    """松开会把手里的东西扔到下面的任何东西上，而这个文件无从判断哪个更糟。"""
    plugin, client = make_plugin()
    plugin._gripper_enabled = True
    plugin._want_gripper(1.0)

    plugin._on_watchdog()

    assert client.stop_calls              # 手臂停了
    assert plugin._gripper_wanted is None  # 信箱清了，但没有发「张开」


def test_watchdog_releases_the_gripper_when_asked():
    plugin, client = make_plugin(
        config={"servo": {"release_gripper_on_watchdog": True,
                          "gripper_max_hz": 1000}})
    plugin._gripper_enabled = True
    plugin._running = True
    plugin._start_workers()
    try:
        plugin._on_watchdog()
        _drain_gripper(plugin, client, expect=1)
        # 0 = 张开。
        assert client.gripper_calls[0][0] == device.GRIPPER_POSITION_MAX
    finally:
        plugin._stop_workers()


# ── 本体状态 ─────────────────────────────────────────────────────────────────

class _FakePublisher:
    def __init__(self):
        self.published = []

    def publish(self, message):
        self.published.append(message)


def _wire_state_publisher(plugin):
    publisher = _FakePublisher()
    plugin._state_pub = publisher
    return publisher


def test_state_payload_is_the_same_layout_as_the_commands_it_accepts():
    """同序同单位：两者出自同一份 build_descriptor()，想只改一边做不到。"""
    import json

    plugin, _ = make_plugin()
    plugin._gripper_enabled = True
    plugin._rebuild_descriptor()
    publisher = _wire_state_publisher(plugin)

    plugin._publish_state()

    payload = json.loads(publisher.published[0].data)
    assert payload["schema"] == "motus.control/1"
    assert payload["kind"] == "joint_state"
    assert payload["dof"] == len(device.JOINT_NAMES) + 1
    assert payload["joint_names"] == list(device.JOINT_NAMES) + ["gripper"]
    assert len(payload["values"]) == len(device.JOINT_NAMES) + 1
    # 关节是弧度（SDK 报的是度），和 descriptor 的单位一致。
    assert payload["values"][0] == pytest.approx(math.radians(10.0))
    assert payload["values"][1] == pytest.approx(math.radians(-20.0))
    # actpos 1000 = 张开 = 闭合度 0，和指令那一侧同一个约定。
    assert payload["values"][-1] == pytest.approx(0.0)


def test_state_carries_the_current_eef_pose_as_a_unit_quaternion():
    """`eef` 是给增量模型当基准位姿用的：delta「相对于哪儿」只有机器人知道。"""
    import json
    from common.control import rotation

    plugin, _ = make_plugin()
    plugin._gripper_enabled = True
    plugin._rebuild_descriptor()
    publisher = _wire_state_publisher(plugin)

    plugin._publish_state()

    eef = json.loads(publisher.published[0].data)["eef"]
    # 7 维位姿 + 夹爪，和 phanthymotus-cloud 的 SINGLE_ARM_EEF_GROUPS 同构。
    assert len(eef) == 8
    assert eef[:3] == pytest.approx([0.3, 0.1, 0.5])     # 米，原样
    assert rotation.is_unit(eef[3:7])
    assert eef[7] == pytest.approx(0.0)


def test_state_is_not_published_when_a_channel_is_missing():
    """缺一路就整个不发，不补零 —— 补进去的零是看着合理的读数，策略无从分辨。"""
    plugin, _ = make_plugin(arm_state={"joint": [1.0, 2.0], "pose": [0.0] * 6})
    plugin._gripper_enabled = True
    publisher = _wire_state_publisher(plugin)

    plugin._publish_state()

    assert publisher.published == []
    assert "incomplete" in plugin.dispatch("info", {})["state_out"]["error"]


def test_state_is_not_published_when_the_pose_is_not_finite():
    plugin, _ = make_plugin(arm_state={
        "joint": [0.0] * 7, "pose": [0.0, 0.0, float("nan"), 0.0, 0.0, 0.0]})
    plugin._gripper_enabled = True
    publisher = _wire_state_publisher(plugin)

    plugin._publish_state()

    assert publisher.published == []


def test_state_stamp_is_the_sample_time_not_the_publish_time():
    """VLA 用它算观测年龄；报发布时刻等于宣称通道比实际更新鲜。"""
    import json

    plugin, _ = make_plugin()
    plugin._gripper_enabled = True
    publisher = _wire_state_publisher(plugin)

    before = int(time.time() * 1000)
    sample = plugin._sample_state()
    after = int(time.time() * 1000)
    assert before <= sample["stamp_ms"] <= after

    plugin._publish_state()
    stamp = json.loads(publisher.published[0].data)["stamp_ms"]
    assert stamp <= int(time.time() * 1000)


def test_state_width_follows_the_gripper_being_absent():
    import json

    plugin, _ = make_plugin(config={"servo": {"gripper": False}})
    publisher = _wire_state_publisher(plugin)

    plugin._publish_state()

    payload = json.loads(publisher.published[0].data)
    assert payload["dof"] == len(device.JOINT_NAMES)
    assert len(payload["values"]) == len(device.JOINT_NAMES)
    assert len(payload["eef"]) == 7         # 位姿，没有夹爪那一维


def test_the_card_declares_its_state_output_before_it_is_started():
    """画布上 vla → servo →(state)→ vla 是个环，环里没人能从已启动的上游学到话题。"""
    plugin, _ = make_plugin()
    definition = plugin.get_tools()[0]

    assert definition["topic_out"][0]["format"] == "state/joint"
    assert definition["topic_out"][0]["topic"] == "/rm75/servo/state"
    assert plugin.dispatch("info", {})["state"] == "idle"


def test_one_sdk_round_trip_covers_both_joints_and_pose():
    """rm_get_current_arm_state 同时给 joint 和 pose，所以手臂那边只问一次。"""
    plugin, client = make_plugin()
    plugin._gripper_enabled = True
    _wire_state_publisher(plugin)

    plugin._publish_state()

    assert client.reads.count("rm_get_current_arm_state") == 1
    assert client.reads.count("rm_get_gripper_state") == 1


# ── 六维力 ───────────────────────────────────────────────────────────────────

def test_force_reads_the_compensated_vector_not_the_raw_one():
    """`force_data` 含工具自重；对它设阈值会被重力触发，而那看着像阈值没调好。"""
    plugin, client = make_plugin()
    readings = plugin._read_force()
    # fake 里 force_data 是 900（「看着像撞上了」），zero_force_data 是小值。
    assert readings == pytest.approx([0.5, -0.5, 1.0, 0.0, 0.1, 0.0])


def test_force_thresholds_default_to_six_axes_from_two_scalars():
    plugin, _ = make_plugin()
    assert plugin._force_thresholds == [
        servo.DEFAULT_FORCE_N, servo.DEFAULT_FORCE_N, servo.DEFAULT_FORCE_N,
        servo.DEFAULT_TORQUE_NM, servo.DEFAULT_TORQUE_NM, servo.DEFAULT_TORQUE_NM]


def test_force_config_accepts_an_explicit_six_list_and_rejects_a_wrong_width():
    plugin, _ = make_plugin(
        config={"servo": {"force_torque": [1, 2, 3, 4, 5, 6]}})
    assert plugin._force_thresholds == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    with pytest.raises(ValueError):
        make_plugin(config={"servo": {"force_torque": [1, 2, 3]}})


def test_force_can_be_turned_off_and_then_the_descriptor_says_so():
    plugin, _ = make_plugin(config={"servo": {"force_torque": False}})
    assert plugin._force_thresholds is None
    assert plugin._descriptor_raw["force_torque"] is None
    assert plugin._probe_force() == (False, "disabled by servo.force_torque")


def test_an_all_zero_sample_does_not_become_a_working_protection():
    """全零分不清「悬空且刚标零」和「传感器没接」，所以不赌 —— 不启用，并说原因。"""
    plugin, _ = make_plugin(force={"force_data": [1.0] * 6,
                                   "zero_force_data": [0.0] * 6})
    enabled, note = plugin._probe_force()
    assert enabled is False
    assert "zero" in note and "rm_clear_force_data" in note


def test_an_unreadable_sensor_does_not_become_a_working_protection():
    plugin, _ = make_plugin(force=FAILS)
    enabled, note = plugin._probe_force()
    assert enabled is False
    assert "cannot read" in note


def test_the_probe_never_zeroes_the_sensor_itself():
    """标零会把当前状态定为零位；手臂已经碰着东西时等于把接触烘焙成零点。"""
    plugin, client = make_plugin()
    plugin._probe_force()
    assert "rm_clear_force_data" not in [m for m, _ in client.calls]


def test_a_force_over_the_threshold_aborts_and_latches():
    from common.control import ControlSink

    plugin, client = make_plugin()
    plugin._force_enabled = True
    plugin._rebuild_descriptor()
    clock = [1_000_000]
    sink = ControlSink(plugin._descriptor_raw, plugin._apply,
                       clock=lambda: clock[0], wall_clock=lambda: clock[0])
    plugin._sink = sink
    plugin._running = True
    # 刚好越过 Fz 的阈值。
    client.force = {"force_data": [0.0] * 6,
                    "zero_force_data": [0.0, 0.0, servo.DEFAULT_FORCE_N + 1.0,
                                        0.0, 0.0, 0.0]}

    plugin._sample_force()

    assert sink.stats()["aborted"]
    # 锁存：撞到东西之后，下一条指令恰恰是最不该跑的那一条。
    outcome = sink.submit({
        "schema": "motus.control/1", "seq": 1, "stamp_ms": clock[0],
        "obs_stamp_ms": clock[0], "ttl_ms": 100, "source": "test",
        "priority": 50, "mode": "joint_position", "dof": 8,
        "values": [0.0] * 8,
    })
    assert outcome.verdict is Verdict.ABORTED
    assert client.movej_calls == []


def test_reset_is_the_only_way_out_of_a_force_abort():
    from common.control import ControlSink

    plugin, client = make_plugin()
    plugin._force_enabled = True
    plugin._rebuild_descriptor()
    sink = ControlSink(plugin._descriptor_raw, plugin._apply)
    plugin._sink = sink
    plugin._running = True
    client.force = {"force_data": [0.0] * 6,
                    "zero_force_data": [999.0, 0, 0, 0, 0, 0]}
    plugin._sample_force()
    assert sink.stats()["aborted"]

    # resume 恢复不了一个锁存 —— 它们是两回事。
    plugin.dispatch("resume", {})
    assert sink.stats()["aborted"]

    assert plugin.dispatch("reset", {})["state"] == "running"
    assert not sink.stats()["aborted"]


def test_reset_drops_the_gripper_target_held_through_the_abort():
    """锁存期间攒下的目标是撞击之前算出来的，不该在恢复的瞬间被发出去。"""
    from common.control import ControlSink

    plugin, _ = make_plugin()
    plugin._sink = ControlSink(servo.build_descriptor(), plugin._apply)
    plugin._running = True
    plugin._want_gripper(1.0)

    plugin.dispatch("reset", {})

    assert plugin._gripper_wanted is None


def test_reset_on_an_idle_card_is_not_an_error():
    plugin, _ = make_plugin()
    assert plugin.dispatch("reset", {})["state"] == "idle"


def test_force_and_the_threshold_source_are_both_reported():
    plugin, _ = make_plugin()
    report = plugin.dispatch("info", {})["force_torque"]
    assert report["thresholds"] == plugin._force_thresholds
    # 读的是哪个向量要能被读到 —— 这是这一项里最容易弄错的一处。
    assert "zero_force_data" in report["source"]


def test_the_motion_gate_is_taken_before_any_sdk_probe():
    """别人持着手臂时，不该有一次 SDK 查询发出去，报错也不该指向夹爪。

    顺序反了的话失败信息是「读不到夹爪状态」，于是看的人去查夹爪，而真正的
    原因是另一张卡片正在用这条手臂。
    """
    plugin, client = make_plugin()
    plugin._ros2 = object()
    plugin._subscribe = lambda topic: None
    assert client.motion_gate.acquire(blocking=False)
    try:
        result = plugin.dispatch("start", {"input_topic": "/x"})
    finally:
        client.motion_gate.release()

    assert result["state"] == "error"
    assert "another arm operation" in result["message"]
    # 关键：一次 SDK 读都没有发生。
    assert client.reads == []


def test_a_failed_probe_gives_the_motion_gate_back():
    """探测失败不能把共享的门漏掉 —— 漏掉之后这条手臂对所有卡片都不可用了。"""
    plugin, client = make_plugin(gripper_state={"status": 0, "enable_state": 0})
    plugin._ros2 = object()
    plugin._subscribe = lambda topic: None

    assert plugin.dispatch("start", {"input_topic": "/x"})["state"] == "error"

    assert client.motion_gate.acquire(blocking=False), "motion gate was leaked"
    client.motion_gate.release()


def test_a_probe_that_raises_is_reported_rather_than_propagated():
    plugin, client = make_plugin(gripper_state=FAILS)
    plugin._ros2 = object()
    plugin._subscribe = lambda topic: None

    result = plugin.dispatch("start", {"input_topic": "/x"})

    assert result["state"] == "error"
    assert "gripper" in result["message"]
    assert client.motion_gate.acquire(blocking=False), "motion gate was leaked"
    client.motion_gate.release()


def test_pause_also_stops_the_gripper():
    """「暂停」不能变成「手臂停了但手还在合」。

    `pause` 是模型的 halt，也是中断钩子打的那个。一个还压在信箱里的目标会在
    暂停之后被 worker 发出去 —— 那个目标出自一个已经过去的世界。
    """
    plugin, client = make_plugin(config={"servo": {"gripper_max_hz": 1000}})
    plugin._gripper_enabled = True
    plugin._running = True
    plugin._start_workers()
    try:
        plugin._want_gripper(1.0)
        plugin.dispatch("pause", {})
        assert plugin._gripper_wanted is None
        time.sleep(0.05)
        assert client.gripper_calls == []
    finally:
        plugin._stop_workers()


def test_the_worker_itself_refuses_to_act_while_paused():
    """清信箱不够 —— worker 自己也要查暂停。

    手臂那条路是同步的：暂停之后就没有新的 `_apply`。夹爪这条是异步的，清信箱
    和 worker 取出目标之间有一个窗口。这条测试绕开清信箱、直接往信箱里写，于是
    它**只**验证 worker 自己那道闸门 —— 把闸门去掉它就会红，而
    `test_pause_also_stops_the_gripper` 不会。
    """
    plugin, client = make_plugin(config={"servo": {"gripper_max_hz": 1000}})
    plugin._gripper_enabled = True
    plugin._running = True
    plugin._paused = True
    plugin._start_workers()
    try:
        # 绕过 `_want_gripper`/`_clear_gripper_target` 的配对，直接造出那个窗口。
        with plugin._gripper_lock:
            plugin._gripper_wanted = 1.0
        plugin._gripper_wake.set()
        time.sleep(0.05)
        assert client.gripper_calls == []

        # resume 之后同一个目标照常发下去。
        plugin._paused = False
        plugin._gripper_wake.set()
        _drain_gripper(plugin, client, expect=1)
        assert len(client.gripper_calls) == 1
    finally:
        plugin._stop_workers()
