"""Contract tests for the GaitPlugin and MotionRecorderPlugin (from device.py)."""

import importlib.util
import json
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from control import T800_JOINT_POSITION_LIMITS  # noqa: E402


def safe_positions(*, torso_yaw=0.0):
    positions = [
        (lower + upper) / 2.0
        for lower, upper in T800_JOINT_POSITION_LIMITS
    ]
    positions[12] = float(torso_yaw)
    return positions


class Message:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')
        self.position = []
        self.velocity = []


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self, name, context=None):
        self.name = name
        self.publishers = []
        self.subscriptions = []

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher()
        publisher.topic = topic
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        import types as t
        subscription = t.SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        if subscription in self.subscriptions:
            self.subscriptions.remove(subscription)
        return True

    def create_timer(self, period, callback):
        return types.SimpleNamespace(period=period, callback=callback)

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 'stamp'))

    def destroy_node(self):
        pass


class FakeExecutor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class FakeRos:
    def __init__(self):
        self.ctx_robot = object()
        self.ctx_core = object()
        self.executor_robot = FakeExecutor()
        self.executor_core = FakeExecutor()


class AgentCoreBarrierContractHarness:
    """Faithful model of Agent Core's device-global, serialized ACP gate."""

    def __init__(self):
        self._dispatch = {}
        self._completion_actions = set()
        self._interrupt_actions = set()
        self._lock = threading.Lock()
        self._pending: set[str] = set()

    def register(self, tool: dict, dispatch) -> None:
        schema = tool["inputSchema"]
        tool_name = str(tool["name"])
        self._dispatch[tool_name] = dispatch
        self._completion_actions.update(
            (tool_name, action)
            for action in schema.get("x-completion", {}).get("actions", [])
        )
        self._interrupt_actions.update(
            (tool_name, binding.get("action"))
            for hook_id, binding in schema.get("x-hooks", {}).items()
            if hook_id.startswith("on_interrupt")
        )

    def call(self, tool_name: str, action: str, args: dict) -> dict:
        action_key = (tool_name, action)
        with self._lock:
            barrier_blocks = (
                bool(self._pending)
                and action_key not in self._interrupt_actions
            )
        if barrier_blocks:
            return {
                "state": "barrier_waiting",
                "tool": tool_name,
                "action": action,
            }

        result = self._dispatch[tool_name](action, args)
        with self._lock:
            if action_key in self._completion_actions and result.get("action_id"):
                self._pending.add(str(result["action_id"]))
            if action_key in self._interrupt_actions:
                self._pending.clear()
        return result

    def pending_actions(self) -> set[str]:
        with self._lock:
            return set(self._pending)


def install_ros_stubs():
    if 'rclpy.node' in sys.modules:
        return

    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType('rclpy.qos')
    rclpy_qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    sys.modules['rclpy.node'] = rclpy_node
    sys.modules['rclpy.qos'] = rclpy_qos

    std_msgs = types.ModuleType('std_msgs.msg')
    std_msgs.Header = type('Header', (Message,), {})
    std_msgs.String = type('String', (Message,), {'__init__': lambda self: setattr(self, 'data', '')})
    sys.modules['std_msgs.msg'] = std_msgs

    sensor_msgs = types.ModuleType('sensor_msgs.msg')
    sensor_msgs.PointCloud2 = type('PointCloud2', (Message,), {})
    sensor_msgs.CompressedImage = type('CompressedImage', (Message,), {})
    sensor_msgs.Image = type('Image', (Message,), {})
    sys.modules['sensor_msgs.msg'] = sensor_msgs

    nav_msgs = types.ModuleType('nav_msgs.msg')
    nav_msgs.Odometry = type('Odometry', (Message,), {})
    sys.modules['nav_msgs.msg'] = nav_msgs

    audio_msgs = types.ModuleType('audio_msgs.msg')
    audio_msgs.AudioChunk = type('AudioChunk', (Message,), {})
    sys.modules['audio_msgs.msg'] = audio_msgs

    protocol_msg = types.ModuleType('interface_protocol.msg')
    for name in ('BodyVelCmd', 'GamepadKeys', 'ImuInfo', 'JointCommand', 'JointOverrideCommand', 'JointState', 'LedControl', 'MotionState', 'MotionStateRequest', 'Heartbeat', 'LinkInfo', 'MotorDebug', 'NodeControl', 'PowerInfo', 'Tts', 'JointMotionPlanRequest', 'JointMotionPlanState'):
        setattr(protocol_msg, name, type(name, (Message,), {}))
    protocol_srv = types.ModuleType('interface_protocol.srv')
    protocol_srv.EnableMotor = type('EnableMotor', (), {})
    protocol = types.ModuleType('interface_protocol')
    protocol.msg = protocol_msg
    protocol.srv = protocol_srv
    sys.modules['interface_protocol'] = protocol
    sys.modules['interface_protocol.msg'] = protocol_msg
    sys.modules['interface_protocol.srv'] = protocol_srv


def load_device():
    install_ros_stubs()
    spec = importlib.util.spec_from_file_location('t800_device', ROOT / 'device.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ── GaitPlugin tests ────────────────────────────────────────────────────────

class FakeMotionState:
    def __init__(self, current="pd_stand", available=None):
        self._current = current
        self._generation = 0
        self.available = list(available or [])

    @property
    def current(self):
        return self._current

    @current.setter
    def current(self, value):
        self._current = value
        self._generation += 1

    def current_motion(self):
        return self.current, list(self.available)

    def motion_snapshot(self):
        return self.current, list(self.available), self._generation


class FakeMotionMode:
    def __init__(self, state):
        self.state = state
        self.calls = []

    def dispatch(self, action, args):
        self.calls.append((action, dict(args)))
        target = args["target"]
        if self.state.available and target not in self.state.available and not args.get("force", False):
            return {"error": f"{target} is not available", "available": self.state.available}
        previous = self.state.current
        self.state.current = target
        if args.get("wait", True):
            return {"state": "completed", "current": target, "available": self.state.available}
        return {"state": "requested", "target": target, "previous": previous}


class GaitPluginContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dev = load_device()

    def setUp(self):
        self.state = FakeMotionState(available=["rl_basic", "lower_body_balance"])
        self.motion_mode = FakeMotionMode(self.state)
        self.plugin = self.dev.GaitPlugin(
            {"plugins": {"gait": {"basic_motion_states": ["rl_basic", "walk"]}}},
            self.motion_mode,
            self.state,
        )

    def test_tool_schema_declares_real_gait_profiles(self):
        tool = self.plugin.get_tool()
        self.assertEqual("gait", tool["name"])
        self.assertEqual("actuator", tool["type"])
        actions = tool["inputSchema"]["properties"]["action"]["enum"]
        self.assertEqual(["start", "stop", "info", "status", "list", "select"], actions)
        self.assertEqual(
            [
                {"const": "basic", "title": "拟人步态"},
                {"const": "balanced", "title": "下肢平衡"},
            ],
            tool["inputSchema"]["properties"]["gait"]["oneOf"],
        )
        self.assertNotIn("force", tool["inputSchema"]["properties"])
        self.assertNotIn("wait", tool["inputSchema"]["properties"])
        self.assertEqual(
            ["gait"],
            tool["inputSchema"]["x-action-params"]["select"]["params"],
        )
        self.assertEqual(
            ["select"], tool["inputSchema"]["x-completion"]["actions"]
        )

    def test_start_is_actuator_lifecycle_ready(self):
        self.assertEqual("ready", self.plugin.dispatch("start", {})["state"])

    def test_list_reports_runtime_availability(self):
        result = self.plugin.dispatch("list", {})
        self.assertEqual("ready", result["state"])
        profiles = {item["name"]: item for item in result["profiles"]}
        self.assertEqual("拟人步态", profiles["basic"]["title"])
        self.assertEqual("rl_basic", profiles["basic"]["resolved_motion_state"])
        self.assertTrue(profiles["basic"]["available"])
        self.assertNotIn("terrain", profiles)

    def test_select_anthropomorphic_gait_uses_fixed_safe_options_and_acp(self):
        completions = []
        self.plugin._acp_notify = lambda *args: completions.append(args)
        result = self.plugin.dispatch("select", {"gait": "basic"})
        self.assertEqual("switching", result["state"])
        self.assertTrue(result["action_id"].startswith("t800_gait_"))
        self.plugin._selection_thread.join(timeout=1.0)
        self.assertEqual(
            ("switch", {"target": "rl_basic", "force": False, "wait": False}),
            self.motion_mode.calls[-1],
        )
        self.assertEqual(result["action_id"], completions[0][0])
        self.assertEqual("completed", completions[0][1])

    def test_select_anthropomorphic_adapts_to_legacy_walk_state(self):
        self.state.available = ["walk"]
        self.plugin._acp_notify = lambda *_args: None
        result = self.plugin.dispatch("select", {"gait": "basic"})
        self.assertEqual("switching", result["state"])
        self.plugin._selection_thread.join(timeout=1.0)
        self.assertEqual("walk", self.motion_mode.calls[-1][1]["target"])

    def test_concurrent_select_claim_is_atomic_before_thread_assignment(self):
        class BlockingMotionMode(FakeMotionMode):
            def __init__(self, state):
                super().__init__(state)
                self.release = threading.Event()

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                self.release.wait(timeout=1.0)
                self.state.current = args["target"]
                return {
                    "state": "completed",
                    "current": args["target"],
                    "available": self.state.available,
                }

        motion_mode = BlockingMotionMode(self.state)
        plugin = self.dev.GaitPlugin(
            {"plugins": {"gait": {
                "basic_motion_states": ["rl_basic", "walk"]
            }}},
            motion_mode,
            self.state,
        )
        plugin._acp_notify = lambda *_args: None
        original_thread = self.dev.threading.Thread
        constructor_entered = threading.Event()
        release_constructor = threading.Event()
        constructor_calls = 0

        def pausing_thread(*args, **kwargs):
            nonlocal constructor_calls
            constructor_calls += 1
            if constructor_calls == 1:
                constructor_entered.set()
                release_constructor.wait(timeout=1.0)
            return original_thread(*args, **kwargs)

        results = [{}, {}]
        self.dev.threading.Thread = pausing_thread
        try:
            callers = [
                original_thread(
                    target=lambda index=index: results[index].update(
                        plugin.dispatch("select", {"gait": "basic"})
                    )
                )
                for index in range(2)
            ]
            callers[0].start()
            self.assertTrue(constructor_entered.wait(timeout=1.0))
            callers[1].start()
            time.sleep(0.02)
            release_constructor.set()
            for caller in callers:
                caller.join(timeout=1.0)
        finally:
            self.dev.threading.Thread = original_thread
            release_constructor.set()
            motion_mode.release.set()

        switching = [result for result in results if result.get("state") == "switching"]
        rejected = [result for result in results if "error" in result]
        self.assertEqual(1, len(switching))
        self.assertEqual(1, len(rejected))
        self.assertEqual(
            switching[0]["action_id"], rejected[0]["action_id"]
        )

    def test_legacy_walk_selected_by_gait_is_accepted_by_loco(self):
        self.state.available = ["walk"]
        self.plugin._acp_notify = lambda *_args: None
        selected = self.plugin.dispatch("select", {"gait": "basic"})
        self.assertEqual("switching", selected["state"])
        self.plugin._selection_thread.join(timeout=1.0)
        self.assertEqual("walk", self.state.current)
        loco = self.dev.LocomotionPlugin(
            {
                "control": {
                    "max_vx": 1.0,
                    "max_vy": 1.0,
                    "max_vyaw": 1.0,
                    "velocity_rate_hz": 100.0,
                    "stream_watchdog_period_sec": 0.5,
                },
                "topics": {"body_velocity": "/motion/body_vel_cmd"},
            },
            "t800",
            FakeRos(),
            self.state,
        )
        loco.start()
        moved = loco.dispatch("move", {"vx": 0.1, "duration": 0.01})
        self.assertEqual("completed", moved["state"])
        loco.dispatch("stop_move", {})

    def test_loco_stop_move_bypasses_acp_barrier_and_cancels_timed_action(self):
        self.state.current = "rl_basic"
        loco = self.dev.LocomotionPlugin(
            {
                "control": {
                    "max_vx": 2.0,
                    "max_vy": 1.0,
                    "max_vyaw": 2.0,
                    "velocity_rate_hz": 100.0,
                    "stream_watchdog_period_sec": 0.5,
                    "locomotion_prepare_duration_sec": 0.2,
                },
                "topics": {"body_velocity": "/motion/body_vel_cmd"},
            },
            "t800",
            FakeRos(),
            self.state,
        )
        loco.start()
        loco._ACP_MIN_DURATION_SEC = 0.1
        completions = []
        loco._acp_notify = lambda *args: completions.append(args)
        group = self.dev.MotionInterruptGroup()
        group.register("locomotion", loco.halt, loco.motion_active)
        loco.set_interrupt_group(group)
        harness = AgentCoreBarrierContractHarness()
        harness.register(loco.get_tool(), loco.dispatch)

        moving = harness.call("loco", "move", {"vx": 0.2, "duration": 1.0})
        self.assertEqual({moving["action_id"]}, harness.pending_actions())
        stopped = harness.call("loco", "stop_move", {})

        self.assertEqual(set(), harness.pending_actions())
        self.assertEqual("stopped", stopped["state"])
        self.assertFalse(loco.motion_active())
        deadline = time.monotonic() + 1.0
        while not completions and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(moving["action_id"], completions[0][0])
        self.assertEqual("cancelled", completions[0][1])

    def test_stop_hook_cancels_gait_and_supersedes_pending_transition(self):
        class PendingMotionMode:
            def __init__(self, state):
                self.state = state
                self.calls = []
                self.requested = threading.Event()

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                if args["target"] == "rl_basic":
                    self.requested.set()
                else:
                    self.state.current = args["target"]
                return {
                    "state": "requested",
                    "target": args["target"],
                }

        self.state.current = "pd_stand"
        motion_mode = PendingMotionMode(self.state)
        plugin = self.dev.GaitPlugin(
            {
                "control": {"mode_transition_timeout_sec": 1.0},
                "plugins": {"gait": {
                    "basic_motion_states": ["rl_basic", "walk"]
                }},
            },
            motion_mode,
            self.state,
        )
        completions = []
        plugin._acp_notify = lambda *args: completions.append(args)
        group = self.dev.MotionInterruptGroup()
        plugin.set_interrupt_group(group)
        group.register("gait", plugin.halt, plugin.motion_active)
        harness = AgentCoreBarrierContractHarness()
        harness.register(plugin.get_tool(), plugin.dispatch)

        selected = harness.call(
            "gait", "select", {"gait": "basic"}
        )
        self.assertTrue(motion_mode.requested.wait(timeout=1.0))
        self.assertEqual({selected["action_id"]}, harness.pending_actions())

        stopped = harness.call("gait", "stop", {})
        self.assertEqual(set(), harness.pending_actions())
        self.assertIn(stopped["state"], ("cancelling", "cancelled"))
        plugin._selection_thread.join(timeout=1.0)
        self.assertFalse(plugin.motion_active())
        self.assertEqual(selected["action_id"], completions[0][0])
        self.assertEqual("cancelled", completions[0][1])
        self.assertTrue(any(
            action == "switch"
            and args == {
                "target": "pd_stand",
                "force": True,
                "wait": False,
            }
            for action, args in motion_mode.calls
        ))
        self.assertEqual([], group.blocking_outputs())

    def test_gait_cancel_stays_fail_closed_until_origin_feedback(self):
        class LateTargetMotionMode:
            def __init__(self, state):
                self.state = state
                self.requested = threading.Event()

            def dispatch(self, _action, args):
                if args["target"] == "rl_basic":
                    self.requested.set()
                else:
                    # Simulate the original request taking effect after the
                    # restore request was already published.
                    self.state.current = "rl_basic"
                return {"state": "requested", "target": args["target"]}

        self.state.current = "pd_stand"
        motion_mode = LateTargetMotionMode(self.state)
        plugin = self.dev.GaitPlugin(
            {
                "control": {"mode_transition_timeout_sec": 1.0},
                "plugins": {"gait": {
                    "basic_motion_states": ["rl_basic", "walk"]
                }},
            },
            motion_mode,
            self.state,
        )
        plugin._restore_timeout_sec = 0.02
        completions = []
        plugin._acp_notify = lambda *args: completions.append(args)
        group = self.dev.MotionInterruptGroup()
        plugin.set_interrupt_group(group)
        group.register("gait", plugin.halt, plugin.motion_active)

        selected = plugin.dispatch("select", {"gait": "basic"})
        self.assertTrue(motion_mode.requested.wait(timeout=1.0))
        stopped = plugin.halt()
        self.assertEqual("cancelling", stopped["state"])
        plugin._selection_thread.join(timeout=1.0)

        self.assertEqual(selected["action_id"], completions[0][0])
        self.assertEqual("error", completions[0][1])
        self.assertTrue(plugin.motion_active())
        self.assertEqual(["gait"], group.blocking_outputs())
        self.assertIn("settling", plugin.dispatch("status", {})["state"])

        self.state.current = "pd_stand"
        self.assertFalse(plugin.motion_active())
        self.assertEqual([], group.blocking_outputs())

    def test_gait_timeout_restores_origin_before_releasing_gate(self):
        class TimeoutMotionMode:
            def __init__(self, state):
                self.state = state
                self.calls = []

            def dispatch(self, _action, args):
                self.calls.append(dict(args))
                if args["target"] == "pd_stand":
                    self.state.current = "pd_stand"
                return {"state": "requested", "target": args["target"]}

        self.state.current = "pd_stand"
        motion_mode = TimeoutMotionMode(self.state)
        plugin = self.dev.GaitPlugin(
            {
                "control": {"mode_transition_timeout_sec": 0.1},
                "plugins": {"gait": {
                    "basic_motion_states": ["rl_basic", "walk"]
                }},
            },
            motion_mode,
            self.state,
        )
        plugin._transition_timeout_sec = 0.02
        plugin._restore_timeout_sec = 0.05
        completions = []
        plugin._acp_notify = lambda *args: completions.append(args)

        selected = plugin.dispatch("select", {"gait": "basic"})
        plugin._selection_thread.join(timeout=1.0)

        self.assertTrue(any(
            call == {"target": "pd_stand", "force": True, "wait": False}
            for call in motion_mode.calls
        ))
        self.assertEqual(selected["action_id"], completions[0][0])
        self.assertEqual("error", completions[0][1])
        self.assertEqual("timeout_recovered", completions[0][2]["state"])
        self.assertFalse(plugin.motion_active())

    def test_select_rejects_unpublished_profile_before_publish(self):
        result = self.plugin.dispatch("select", {"gait": "terrain"})
        self.assertIn("error", result)
        self.assertEqual([], self.motion_mode.calls)

    def test_status_reflects_current_gait(self):
        self.state.current = "lower_body_balance"
        result = self.plugin.dispatch("status", {})
        self.assertEqual("active", result["state"])
        self.assertEqual("balanced", result["gait"])
        self.assertEqual("lower_body_balance", result["motion_state"])

    def test_unknown_action_returns_error(self):
        result = self.plugin.dispatch("foobar", {})
        self.assertIn("error", result)


class SpeakerAcpContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dev = load_device()

    def test_stop_bypasses_startup_acp_barrier_and_cancels_full_beep(self):
        class Stdin:
            def write(self, _data):
                pass

            def flush(self):
                pass

            def close(self):
                pass

        class Process:
            def __init__(self):
                self.stdin = Stdin()
                self.returncode = None

            def poll(self):
                return self.returncode

            def terminate(self):
                self.returncode = 0

            def wait(self, timeout=None):
                return self.returncode

            def kill(self):
                self.returncode = -9

        plugin = self.dev.SpeakerPlugin({}, "t800", FakeRos())
        plugin._check_pulse = lambda: None
        plugin._run_command = lambda _command: ""
        plugin._spawn_player = lambda: Process()
        startup_waiting = threading.Event()

        def startup_wait(cancel_event, _seconds):
            startup_waiting.set()
            return cancel_event.wait(timeout=1.0)

        plugin._startup_wait = startup_wait
        completions = []
        plugin._acp_notify = lambda *args: completions.append(args)
        group = self.dev.MotionInterruptGroup()
        group.register("speaker", plugin.halt, plugin.motion_active)
        plugin.set_interrupt_group(group)
        harness = AgentCoreBarrierContractHarness()
        harness.register(plugin.get_tool(), plugin.dispatch)

        started = harness.call(
            "speaker", "start", {"input_topic": "/perception/tts"}
        )
        self.assertTrue(startup_waiting.wait(timeout=1.0))
        self.assertEqual({started["action_id"]}, harness.pending_actions())
        startup_thread = plugin._startup_thread
        stop_started = time.monotonic()
        stopped = harness.call("speaker", "stop", {})

        self.assertLess(time.monotonic() - stop_started, 0.05)
        self.assertEqual(set(), harness.pending_actions())
        self.assertEqual("idle", stopped["state"])
        startup_thread.join(timeout=1.0)
        self.assertEqual(started["action_id"], completions[0][0])
        self.assertEqual("cancelled", completions[0][1])


# ── MotionRecorderPlugin tests ──────────────────────────────────────────────

class MotionRecorderPluginContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dev = load_device()

    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.state = FakeMotionState(
            current="lower_body_balance",
            available=["pd_stand"],
        )
        self.motion_mode = FakeMotionMode(self.state)
        self.plugin = self.dev.MotionRecorderPlugin(
            {"plugins": {"motion_recorder": {"recordings_dir": self.tmpdir.name}}},
            "t800", FakeRos(),
        )
        self.plugin.start()
        self.plugin.set_reset_controls(self.state, self.motion_mode)
        class ImmediateJointPlan:
            def __init__(self):
                self.request_id = 0
                self.calls = []

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                if action == "reset":
                    self.request_id += 1
                    return {
                        "state": "requested",
                        "request_id": self.request_id,
                    }
                if action == "cancel":
                    return {
                        "state": "requested",
                        "request_id": int(args.get("request_id", self.request_id)),
                    }
                if action == "status":
                    return {
                        "request_id": self.request_id,
                        "status": 1,
                    }
                return {"error": f"unexpected action: {action}"}

            def wait_for_request(self, request_id, _timeout, cancel_event):
                if cancel_event.is_set():
                    raise RuntimeError("reset cancelled")
                return {"request_id": request_id, "status": 1}

        self.joint_plan = ImmediateJointPlan()
        self.plugin.set_joint_plan(self.joint_plan)
        self.acp_calls = []
        self.plugin._acp_notify = lambda *args: self.acp_calls.append(args)

    def tearDown(self):
        self.plugin.stop()
        self.tmpdir.cleanup()

    def _record_start(self, args: dict) -> dict:
        result = self.plugin.dispatch("record_start", args)
        if result.get("state") == "preparing":
            self.plugin._prepare_thread.join(timeout=1.0)
            return {**result, **self.plugin.dispatch("status", {})}
        return result

    def _play(self, args=None) -> dict:
        result = self.plugin.dispatch("play", args or {})
        if result.get("state") == "preparing":
            self.plugin._prepare_thread.join(timeout=1.0)
            with self.plugin._lock:
                playing = self.plugin._playing
            return {**result, "state": "playing" if playing else result["state"]}
        return result

    def _list_entry(self, name: str) -> dict:
        listed = self.plugin.dispatch("list", {})
        return next(
            item for item in listed["recordings"] if item["name"] == name
        )

    def test_tool_schema_declares_all_actions(self):
        tool = self.plugin.get_tool()
        self.assertEqual("motion_recorder", tool["name"])
        actions = tool["inputSchema"]["properties"]["action"]["enum"]
        self.assertEqual(
            [
                "record_start", "record_stop", "play", "stop_playback",
                "list", "delete", "status",
            ],
            actions,
        )
        self.assertEqual(
            {"actions": ["record_start", "play"], "timeout": 3600},
            tool["inputSchema"]["x-completion"],
        )
        self.assertEqual(
            {
                "on_interrupt_motion": {"action": "stop_playback"},
                "on_interrupt_recording": {"action": "record_stop"},
            },
            tool["inputSchema"]["x-hooks"],
        )
        for removed in ("reset", "save", "load", "info"):
            with self.subTest(removed=removed):
                self.assertIn("unknown", self.plugin.dispatch(removed, {})["error"])

    def test_record_start_acp_completes_only_after_recording_is_saved(self):
        class JointPlan:
            def __init__(self):
                self.calls = []

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                if action == "reset":
                    return {"state": "requested", "request_id": 41}
                if action == "status":
                    return {"request_id": 41, "status": 1}
                return {"error": f"unexpected action: {action}"}

            def wait_for_request(self, request_id, timeout, cancel_event):
                self.calls.append(("wait_for_request", {
                    "request_id": request_id,
                    "timeout": timeout,
                    "cancelled": cancel_event.is_set(),
                }))
                return {"request_id": request_id, "status": 1}

        joint_plan = JointPlan()
        self.plugin.set_joint_plan(joint_plan)
        result = self.plugin.dispatch("record_start", {"label": "auto_reset"})

        self.assertEqual("preparing", result["state"])
        self.assertTrue(result["action_id"].startswith("t800_motion_record_start_"))
        self.plugin._prepare_thread.join(timeout=1.0)
        self.assertEqual("recording", self.plugin.dispatch("status", {})["state"])
        self.assertEqual("reset", joint_plan.calls[0][0])
        self.assertEqual("wait_for_request", joint_plan.calls[1][0])
        self.assertEqual([], self.acp_calls)

        self.plugin._on_joint_state(self._make_joint_state())
        saved = self.plugin.dispatch("record_stop", {})
        self.assertEqual("saved", saved["state"])
        self.assertEqual(result["action_id"], self.acp_calls[0][0])
        self.assertEqual("completed", self.acp_calls[0][1])
        self.assertEqual("saved", self.acp_calls[0][2]["state"])

    def test_record_start_auto_enters_lower_body_balance_before_reset(self):
        state = FakeMotionState(
            current="pd_stand", available=["lower_body_balance"]
        )
        motion_mode = FakeMotionMode(state)
        self.plugin.set_reset_controls(state, motion_mode)

        result = self.plugin.dispatch(
            "record_start", {"label": "auto_balance"}
        )
        self.plugin._prepare_thread.join(timeout=1.0)

        self.assertEqual("preparing", result["state"])
        self.assertEqual(
            ("switch", {
                "target": "lower_body_balance",
                "force": False,
                "wait": True,
            }),
            motion_mode.calls[0],
        )
        self.assertEqual("lower_body_balance", state.current)
        self.assertEqual("recording", self.plugin.dispatch("status", {})["state"])

    def test_concurrent_prepare_claim_allows_only_one_reset(self):
        reset_entered = threading.Event()
        release_reset = threading.Event()
        reset_calls = []

        def reset_before(operation, action_id, cancel_event):
            reset_calls.append((operation, action_id))
            reset_entered.set()
            release_reset.wait(timeout=1.0)
            return {"state": "completed"}

        self.plugin._reset_before_action = reset_before
        first_result = {}
        first_thread = threading.Thread(
            target=lambda: first_result.update(
                self.plugin.dispatch("record_start", {"label": "first"})
            )
        )
        first_thread.start()
        self.assertTrue(reset_entered.wait(timeout=1.0))
        # dispatch returns as soon as the prepare worker is launched.  The
        # worker can reach reset_entered before the caller thread stores that
        # return value, so synchronise with the caller before comparing ids.
        first_thread.join(timeout=1.0)
        self.assertFalse(first_thread.is_alive())

        second = self.plugin.dispatch("record_start", {"label": "second"})

        self.assertIn("another recorder action is preparing", second["error"])
        self.assertEqual(first_result["action_id"], second["action_id"])
        self.assertEqual(1, len(reset_calls))
        release_reset.set()
        self.plugin._prepare_thread.join(timeout=1.0)

    def test_play_auto_resets_before_first_override_under_same_acp_action(self):
        class JointPlan:
            def __init__(self):
                self.calls = []

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                if action == "reset":
                    return {"state": "requested", "request_id": 52}
                if action == "status":
                    return {"request_id": 52, "status": 1}
                return {"error": f"unexpected action: {action}"}

            def wait_for_request(self, request_id, timeout, cancel_event):
                self.calls.append(("wait_for_request", {"request_id": request_id}))
                return {"request_id": request_id, "status": 1}

        recording = {
            "metadata": {"label": "auto_play"},
            "frames": [
                {"timestamp": 0, "positions": safe_positions()},
                {"timestamp": 50, "positions": safe_positions()},
            ],
        }
        (Path(self.tmpdir.name) / "auto_play.json").write_text(
            json.dumps(recording), encoding="utf-8"
        )
        self.plugin._latest_joint_positions = safe_positions()
        joint_plan = JointPlan()
        self.plugin.set_joint_plan(joint_plan)

        result = self.plugin.dispatch("play", {"name": "auto_play"})

        self.assertEqual("preparing", result["state"])
        self.assertTrue(result["action_id"].startswith("t800_motion_play_"))
        self.plugin._prepare_thread.join(timeout=1.0)
        self.plugin._playback_thread.join(timeout=2.0)
        self.assertEqual("reset", joint_plan.calls[0][0])
        self.assertEqual("wait_for_request", joint_plan.calls[1][0])
        self.assertGreater(len(self.plugin._override_publisher.messages), 0)
        self.assertEqual(result["action_id"], self.acp_calls[0][0])
        self.assertEqual("completed", self.acp_calls[0][1])

    def test_status_returns_idle_initially(self):
        result = self.plugin.dispatch("status", {})
        self.assertEqual("idle", result["state"])

    def test_start_returns_actuator_ready_without_starting_recording(self):
        result = self.plugin.dispatch("start", {})
        self.assertEqual("ready", result["state"])
        self.assertEqual("idle", result["activity_state"])
        self.assertFalse(result["recording"])

    def test_halt_releases_activity_without_destroying_recorder(self):
        node = self.plugin._node

        self.plugin.halt()

        self.assertIs(node, self.plugin._node)
        started = self._record_start({"label": "after_safety"})
        self.assertEqual("recording", started["state"])

    def test_invalid_record_hz_is_rejected_at_startup(self):
        with self.assertRaises(ValueError):
            self.dev.MotionRecorderPlugin(
                {"plugins": {"motion_recorder": {
                    "recordings_dir": self.tmpdir.name,
                    "record_hz": 0,
                }}},
                "t800", FakeRos(),
            )

    def test_invalid_duration_does_not_enter_recording_state(self):
        result = self._record_start({"duration": -1})
        self.assertIn("error", result)
        self.assertFalse(self.plugin.dispatch("status", {})["recording"])

    def test_record_start_begins_recording(self):
        result = self._record_start({"label": "test_record"})
        self.assertEqual("recording", result["state"])
        self.plugin._on_joint_state(self._make_joint_state())
        time.sleep(0.05)
        result = self.plugin.dispatch("record_stop", {})
        self.assertEqual("saved", result["state"])
        self.assertGreater(result["frames"], 0)

    def test_record_stop_is_idempotent(self):
        result = self.plugin.dispatch("record_stop", {})
        self.assertEqual("idle", result["state"])
        self.assertFalse(result["recording"])

    def test_record_start_is_idempotent_instead_of_toggling_off(self):
        first = self._record_start({"label": "first"})
        second = self._record_start({"label": "second"})
        self.assertEqual("recording", first["state"])
        self.assertEqual("recording", second["state"])
        self.assertTrue(second["already_recording"])
        self.assertEqual("first", second["label"])
        self.assertTrue(self.plugin.dispatch("status", {})["recording"])

    def test_record_start_does_not_block_waiting_for_joint_data(self):
        started = time.monotonic()
        result = self._record_start({"label": "responsive"})
        elapsed = time.monotonic() - started
        self.assertEqual("recording", result["state"])
        self.assertLess(elapsed, 0.2)
        self.assertFalse(result["joint_data_available"])

    def test_recording_with_duration_auto_stops(self):
        result = self._record_start({"label": "timed", "duration": 0.1})
        self.assertEqual("recording", result["state"])
        self.assertEqual([], self.acp_calls)
        self.plugin._on_joint_state(self._make_joint_state())
        time.sleep(0.2)
        status = self.plugin.dispatch("status", {})
        self.assertFalse(status["recording"])
        self.assertEqual("duration", status["last_recording"]["stop_reason"])
        self.assertTrue(Path(status["last_recording"]["file"]).exists())
        self.assertEqual(result["action_id"], self.acp_calls[0][0])
        self.assertEqual("completed", self.acp_calls[0][1])

    def test_record_stop_hook_bypasses_acp_barrier_and_completes_parent(self):
        harness = AgentCoreBarrierContractHarness()
        harness.register(self.plugin.get_tool(), self.plugin.dispatch)

        started = harness.call(
            "motion_recorder",
            "record_start",
            {"label": "manual_hook"},
        )
        self.plugin._prepare_thread.join(timeout=1.0)
        self.assertEqual({started["action_id"]}, harness.pending_actions())
        self.plugin._on_joint_state(self._make_joint_state())

        stopped = harness.call("motion_recorder", "record_stop", {})
        self.assertEqual("saved", stopped["state"])
        self.assertEqual(set(), harness.pending_actions())
        self.assertEqual(started["action_id"], self.acp_calls[0][0])
        self.assertEqual("completed", self.acp_calls[0][1])

    def test_record_stop_during_prepare_release_window_stops_started_recording(self):
        recording_started = threading.Event()
        release_begin = threading.Event()
        original_begin = self.plugin._begin_recording

        def pause_after_begin(*args, **kwargs):
            result = original_begin(*args, **kwargs)
            recording_started.set()
            release_begin.wait(timeout=1.0)
            return result

        self.plugin._begin_recording = pause_after_begin
        started = self.plugin.dispatch(
            "record_start", {"label": "release_window"}
        )
        self.assertTrue(recording_started.wait(timeout=1.0))
        self.plugin._on_joint_state(self._make_joint_state())

        stopped = self.plugin.dispatch("record_stop", {})
        release_begin.set()
        self.plugin._prepare_thread.join(timeout=1.0)

        self.assertEqual("saved", stopped["state"])
        self.assertFalse(self.plugin.dispatch("status", {})["recording"])
        self.assertEqual(started["action_id"], self.acp_calls[0][0])
        self.assertEqual("completed", self.acp_calls[0][1])

    def test_record_stop_before_begin_prevents_late_recording_start(self):
        reset_entered = threading.Event()
        release_reset = threading.Event()

        def delayed_reset(*_args, **_kwargs):
            reset_entered.set()
            release_reset.wait(timeout=1.0)
            return {"state": "completed"}

        self.plugin._reset_before_action = delayed_reset
        started = self.plugin.dispatch(
            "record_start", {"label": "cancel_before_begin"}
        )
        self.assertTrue(reset_entered.wait(timeout=1.0))

        stopped = self.plugin.dispatch("record_stop", {})
        release_reset.set()
        self.plugin._prepare_thread.join(timeout=1.0)

        self.assertEqual("stopping", stopped["state"])
        self.assertFalse(self.plugin.dispatch("status", {})["recording"])
        self.assertEqual(started["action_id"], self.acp_calls[0][0])
        self.assertEqual("cancelled", self.acp_calls[0][1])

    def test_device_interrupt_stops_later_outputs_before_recording_persist_finishes(self):
        self._record_start({"label": "slow_persist"})
        self.plugin._on_joint_state(self._make_joint_state())
        persist_entered = threading.Event()
        release_persist = threading.Event()
        head_stopped = threading.Event()
        original_write_text = Path.write_text

        def blocking_write_text(path, *args, **kwargs):
            persist_entered.set()
            release_persist.wait(timeout=1.0)
            return original_write_text(path, *args, **kwargs)

        group = self.dev.MotionInterruptGroup()
        group.register(
            "motion_recorder",
            self.plugin.interrupt_motion,
            self.plugin.motion_active,
        )
        group.register(
            "head",
            lambda: head_stopped.set() or {"state": "stopped"},
            lambda: False,
        )
        interrupt_result = {}
        interrupt_thread = threading.Thread(
            target=lambda: interrupt_result.update(group.interrupt())
        )
        Path.write_text = blocking_write_text
        try:
            interrupt_thread.start()
            self.assertTrue(persist_entered.wait(timeout=1.0))
            later_output_stopped_promptly = head_stopped.wait(timeout=0.05)
            next_recording = self.plugin.dispatch(
                "record_start", {"label": "must_wait_for_save"}
            )
        finally:
            release_persist.set()
            interrupt_thread.join(timeout=1.0)
            Path.write_text = original_write_text

        self.assertTrue(later_output_stopped_promptly)
        self.assertIn("saving", next_recording["error"])
        self.assertFalse(self.plugin.dispatch("status", {})["recording"])
        self.assertEqual("interrupted", interrupt_result["state"])

    def test_old_auto_stop_cannot_stop_new_recording(self):
        self._record_start({"label": "timed", "duration": 0.05})
        time.sleep(0.01)
        self.plugin.dispatch("record_stop", {})
        self._record_start({"label": "second"})
        time.sleep(0.08)
        status = self.plugin.dispatch("status", {})
        self.assertTrue(status["recording"])
        self.assertEqual("second", status["record_label"])

    def test_record_hz_throttles_joint_callbacks(self):
        self._record_start({"label": "throttled"})
        for _ in range(10):
            self.plugin._on_joint_state(self._make_joint_state())
        result = self.plugin.dispatch("record_stop", {})
        self.assertEqual(1, result["frames"])

    def test_list_returns_saved_recordings(self):
        self._record_start({"label": "list_test"})
        self.plugin._on_joint_state(self._make_joint_state())
        self.plugin.dispatch("record_stop", {})
        result = self.plugin.dispatch("list", {})
        self.assertEqual("ready", result["state"])
        self.assertGreaterEqual(result["count"], 1)

    def test_load_and_play_reject_missing_frame_fields_as_invalid_recording(self):
        malformed = Path(self.tmpdir.name) / "malformed.json"
        malformed.write_text(json.dumps({
            "metadata": {"label": "malformed"},
            "frames": [
                {"positions": safe_positions()},
                {"positions": safe_positions(torso_yaw=0.1)},
            ],
        }), encoding="utf-8")

        played = self._play({"name": "malformed"})

        self.assertEqual("INVALID_ARGUMENT", played["code"])
        self.assertIn("timestamp", played["error"])
        self.assertEqual(0, self.plugin.dispatch("status", {})["buffer_frames"])

    def test_load_rejects_malformed_recording_matrix_without_mutating_buffer(self):
        def frame(timestamp, positions=None, **extra):
            return {
                "timestamp": timestamp,
                "positions": safe_positions() if positions is None else positions,
                **extra,
            }

        valid_path = Path(self.tmpdir.name) / "valid_buffer.json"
        valid_path.write_text(json.dumps({
            "metadata": {"label": "valid"},
            "frames": [frame(0), frame(50)],
        }), encoding="utf-8")
        self.plugin._frames = [frame(0), frame(50)]

        cases = [
            ("root_array", [], "root"),
            ("metadata_array", {"metadata": [], "frames": [frame(0)]}, "metadata"),
            ("frames_string", {"frames": "bad"}, "JSON array"),
            ("frames_empty", {"frames": []}, "at least"),
            ("frame_string", {"frames": ["bad"]}, "must be an object"),
            ("timestamp_bool", {"frames": [frame(True)]}, "timestamp must be a number"),
            ("timestamp_nan", {"frames": [frame(float("nan"))]}, "finite"),
            ("timestamp_overflow", {"frames": [frame(10**400)]}, "finite"),
            ("timestamp_negative", {"frames": [frame(-1)]}, "non-negative"),
            ("timestamp_duplicate", {"frames": [frame(0), frame(0)]}, "strictly increasing"),
            ("positions_missing", {"frames": [{"timestamp": 0}]}, "positions"),
            ("positions_short", {"frames": [frame(0, safe_positions()[:24])]}, "exactly 25"),
            ("positions_string", {"frames": [frame(0, safe_positions()[:24] + ["bad"])]}, "only numbers"),
            ("positions_inf", {"frames": [frame(0, safe_positions()[:24] + [float("inf")])]}, "finite"),
            ("positions_overflow", {"frames": [frame(0, safe_positions()[:24] + [10**400])]}, "finite"),
            ("velocities_short", {"frames": [frame(0, velocities=[0.0] * 24)]}, "velocities"),
            ("velocities_nan", {"frames": [frame(0, velocities=[0.0] * 24 + [float("nan")])]}, "finite"),
            (
                "frames_oversized",
                {"frames": [frame(index * 50) for index in range(self.plugin._MAX_FRAMES + 1)]},
                "maximum frame count",
            ),
        ]
        for name, payload, expected_error in cases:
            with self.subTest(name=name):
                (Path(self.tmpdir.name) / f"{name}.json").write_text(
                    json.dumps(payload), encoding="utf-8"
                )
                result = self._list_entry(name)
                self.assertEqual("INVALID_ARGUMENT", result["code"])
                self.assertIn(expected_error, result["error"])
                self.assertEqual(
                    2,
                    self.plugin.dispatch("status", {})["buffer_frames"],
                )

    def test_recording_resource_and_joint_limits_are_fail_closed(self):
        huge_path = Path(self.tmpdir.name) / "huge.json"
        with huge_path.open("wb") as stream:
            stream.truncate(32 * 1024 * 1024 + 1)
        huge = self._list_entry("huge")
        self.assertEqual("INVALID_ARGUMENT", huge["code"])
        self.assertIn("file size", huge["error"])

        long_path = Path(self.tmpdir.name) / "long_duration.json"
        long_path.write_text(json.dumps({
            "frames": [
                {"timestamp": 0, "positions": safe_positions()},
                {"timestamp": 300001, "positions": safe_positions()},
            ],
        }), encoding="utf-8")
        long_duration = self._list_entry("long_duration")
        self.assertEqual("INVALID_ARGUMENT", long_duration["code"])
        self.assertIn("duration", long_duration["error"])

        fast_start = safe_positions(torso_yaw=-1.0)
        fast_end = safe_positions(torso_yaw=1.0)
        fast_path = Path(self.tmpdir.name) / "fast_frames.json"
        fast_path.write_text(json.dumps({
            "frames": [
                {"timestamp": 0, "positions": fast_start},
                {"timestamp": 1, "positions": fast_end},
            ],
        }), encoding="utf-8")
        fast_frames = self._list_entry("fast_frames")
        self.assertEqual("INVALID_ARGUMENT", fast_frames["code"])
        self.assertIn("frame interval", fast_frames["error"])

        sample_heavy_path = Path(self.tmpdir.name) / "sample_heavy.json"
        sample_heavy_path.write_text(json.dumps({
            "frames": [
                {"timestamp": 0, "positions": safe_positions()},
                {"timestamp": 300000, "positions": safe_positions()},
            ],
        }), encoding="utf-8")
        self.plugin._on_joint_state(self._make_joint_state())
        sample_heavy = self.plugin.dispatch(
            "play", {"name": "sample_heavy", "speed_scale": 0.1}
        )
        self.assertEqual("preparing", sample_heavy["state"])
        self.plugin._prepare_thread.join(timeout=1.0)
        completion = next(
            call for call in self.acp_calls
            if call[0] == sample_heavy["action_id"]
        )
        self.assertEqual("error", completion[1])
        self.assertIn("sample count", completion[2]["error"])

        unsafe_positions = safe_positions()
        unsafe_positions[16] = -3.0
        unsafe_path = Path(self.tmpdir.name) / "unsafe_joint.json"
        unsafe_path.write_text(json.dumps({
            "frames": [
                {"timestamp": 0, "positions": unsafe_positions},
                {"timestamp": 50, "positions": unsafe_positions},
            ],
        }), encoding="utf-8")
        unsafe = self._list_entry("unsafe_joint")
        self.assertEqual("INVALID_ARGUMENT", unsafe["code"])
        self.assertIn("safe position limit", unsafe["error"])

        malformed_path = Path(self.tmpdir.name) / "list_malformed.json"
        malformed_path.write_text("[]", encoding="utf-8")
        listed = self.plugin.dispatch("list", {})
        malformed_entry = next(
            item for item in listed["recordings"]
            if item["name"] == "list_malformed"
        )
        self.assertEqual("INVALID_ARGUMENT", malformed_entry["code"])
        self.assertIn("root", malformed_entry["error"])

    def test_delete_removes_recording_file(self):
        self._record_start({"label": "del_test"})
        self.plugin._on_joint_state(self._make_joint_state())
        self.plugin.dispatch("record_stop", {})
        result = self.plugin.dispatch("delete", {"name": "del_test"})
        self.assertEqual("deleted", result["state"])
        list_result = self.plugin.dispatch("list", {})
        names = [r["name"] for r in list_result.get("recordings", [])]
        self.assertNotIn("del_test", names)

    def test_play_without_name_or_buffer_returns_error(self):
        result = self._play({})
        self.assertIn("error", result)

    def test_play_rejects_speed_scale_outside_schema_bounds(self):
        current = self._make_joint_state()
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": safe_positions(), "velocities": [0.0] * 25},
            {"timestamp": 50, "positions": safe_positions(torso_yaw=0.1), "velocities": [0.0] * 25},
        ]

        too_fast = self._play({"speed_scale": 10})
        non_finite = self._play({"speed_scale": float("nan")})
        boolean = self._play({"speed_scale": True})
        numeric_string = self._play({"speed_scale": "1"})

        self.assertEqual("SAFETY_LIMIT", too_fast["code"])
        self.assertEqual("INVALID_ARGUMENT", non_finite["code"])
        self.assertEqual("INVALID_ARGUMENT", boolean["code"])
        self.assertEqual("INVALID_ARGUMENT", numeric_string["code"])
        self.assertFalse(self.plugin.dispatch("status", {})["playing"])

    def test_play_without_buffer_and_unknown_name_returns_error(self):
        result = self._play({"name": "nonexistent"})
        self.assertIn("error", result)

    def test_play_rejects_single_frame_recording(self):
        self._record_start({"label": "play_test"})
        self.plugin._on_joint_state(self._make_joint_state())
        self.plugin.dispatch("record_stop", {})
        result = self._play({"name": "play_test"})
        self.assertIn("at least two", result["error"])

    def test_playback_state_after_play_with_joint_plan(self):
        class FakeJointPlan:
            def __init__(self):
                self.calls = []

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                if action == "reset":
                    return {"state": "requested", "request_id": 1}
                if action == "status":
                    return {"request_id": 1, "status": 1}
                return {"error": f"unexpected action: {action}"}

            def wait_for_request(self, request_id, _timeout, _cancel_event):
                return {"request_id": request_id, "status": 1}

        joint_plan = FakeJointPlan()
        self.plugin.set_joint_plan(joint_plan)
        current = self._make_joint_state()
        current.position = safe_positions(torso_yaw=0.0)
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {
                "timestamp": index * 50,
                "positions": safe_positions(torso_yaw=0.12 * index),
                "velocities": [0.0] * 25,
            }
            for index in range(5)
        ]

        result = self._play({"speed_scale": 1.0})
        self.assertEqual("playing", result["state"])
        deadline = time.monotonic() + 2.0
        while self.plugin.dispatch("status", {})["playing"] and time.monotonic() < deadline:
            time.sleep(0.01)

        self.assertEqual(
            ["reset"],
            [action for action, _args in joint_plan.calls if action != "status"],
            "playback may auto-reset once but must not plan every frame",
        )
        override_publishers = [
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        ]
        self.assertEqual(1, len(override_publishers))
        messages = override_publishers[0].messages
        self.assertGreater(len(messages), len(self.plugin._frames))
        self.assertEqual(0.0, messages[-1].weight)
        self.assertTrue(any(message.weight == 1.0 for message in messages[:-1]))
        self.assertTrue(self.plugin.dispatch("status", {})["needs_reset"])
        self.assertTrue(any(
            call[0] == result["action_id"] and call[1] == "completed"
            for call in self.acp_calls
        ))

    def test_playback_blends_from_current_pose_before_recorded_trajectory(self):
        current = self._make_joint_state()
        current.position = safe_positions(torso_yaw=-0.5)
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {
                "timestamp": index * 50,
                "positions": safe_positions(torso_yaw=0.5 + 0.05 * index),
                "velocities": [0.0] * 25,
            }
            for index in range(3)
        ]

        result = self._play({"speed_scale": 1.0})
        deadline = time.monotonic() + 2.0
        while self.plugin.dispatch("status", {})["playing"] and time.monotonic() < deadline:
            time.sleep(0.01)

        publisher = next(
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        )
        controlled = [message for message in publisher.messages if message.weight == 1.0]
        self.assertEqual(0.5, result["entry_blend_sec"])
        self.assertAlmostEqual(-0.5, controlled[0].position[0], places=3)
        max_step = max(
            abs(right.position[0] - left.position[0])
            for left, right in zip(controlled, controlled[1:])
        )
        self.assertLess(max_step, 0.05)
        blend_end = int(result["entry_blend_sec"] * result["playback_rate_hz"])
        velocity_join = abs(
            controlled[blend_end].velocity[0]
            - controlled[blend_end + 1].velocity[0]
        )
        self.assertLess(velocity_join, 0.25)

    def test_playback_smooths_velocity_across_recorded_frame_boundaries(self):
        current = self._make_joint_state()
        current.position = safe_positions(torso_yaw=0.0)
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {
                "timestamp": index * 100,
                "positions": safe_positions(torso_yaw=position),
                "velocities": [0.0] * 25,
            }
            for index, position in enumerate((0.0, 1.0, 0.0))
        ]

        self._play({"speed_scale": 1.0})
        deadline = time.monotonic() + 2.0
        while self.plugin.dispatch("status", {})["playing"] and time.monotonic() < deadline:
            time.sleep(0.01)

        publisher = next(
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        )
        controlled = [message for message in publisher.messages if message.weight == 1.0]
        trajectory = controlled[int(0.5 * 100.0) + 1:]
        max_velocity_step = max(
            abs(right.velocity[0] - left.velocity[0])
            for left, right in zip(trajectory, trajectory[1:])
        )
        self.assertLess(max_velocity_step, 5.0)
        self.assertTrue(all(0.0 <= message.position[0] <= 1.0 for message in trajectory))

    def test_playback_publish_failure_clears_playing_state(self):
        current = self._make_joint_state()
        current.position = safe_positions(torso_yaw=0.0)
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": safe_positions(), "velocities": [0.0] * 25},
            {"timestamp": 50, "positions": safe_positions(torso_yaw=0.1), "velocities": [0.0] * 25},
        ]
        publisher = next(
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        )

        def fail_publish(_message):
            raise RuntimeError("publisher unavailable")

        publisher.publish = fail_publish
        result = self._play({})
        self.assertIn("action_id", result)
        deadline = time.monotonic() + 1.0
        while self.plugin.dispatch("status", {})["playing"] and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertFalse(self.plugin.dispatch("status", {})["playing"])

    def test_playback_releases_override_if_motion_mode_changes(self):
        state = FakeMotionState(current="lower_body_balance", available=["pd_stand"])
        self.plugin.set_reset_controls(state, FakeMotionMode(state))
        current = self._make_joint_state()
        current.position = safe_positions(torso_yaw=0.0)
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": safe_positions(), "velocities": [0.0] * 25},
            {"timestamp": 1000, "positions": safe_positions(torso_yaw=0.5), "velocities": [0.0] * 25},
        ]

        self._play({})
        time.sleep(0.05)
        state.current = "pd_stand"
        deadline = time.monotonic() + 1.0
        while self.plugin.dispatch("status", {})["playing"] and time.monotonic() < deadline:
            time.sleep(0.01)

        status = self.plugin.dispatch("status", {})
        self.assertFalse(status["playing"])
        self.assertIn("lower_body_balance", status["playback_error"])
        publisher = next(
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        )
        self.assertEqual(0.0, publisher.messages[-1].weight)

    def test_stop_playback_releases_override_and_completes_acp_as_cancelled(self):
        current = self._make_joint_state()
        current.position = safe_positions(torso_yaw=0.0)
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": safe_positions(), "velocities": [0.0] * 25},
            {"timestamp": 2000, "positions": safe_positions(torso_yaw=0.5), "velocities": [0.0] * 25},
        ]

        started = self._play({})
        time.sleep(0.05)
        stopped = self.plugin.dispatch("stop_playback", {})
        self.assertIn(stopped["state"], ("stopped", "stopping"))
        deadline = time.monotonic() + 1.0
        while not any(
            call[0] == started["action_id"] and call[1] == "cancelled"
            for call in self.acp_calls
        ) and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(any(
            call[0] == started["action_id"] and call[1] == "cancelled"
            for call in self.acp_calls
        ))
        publisher = next(
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        )
        self.assertEqual(0.0, publisher.messages[-1].weight)

    def test_agent_core_interrupt_bypasses_barrier_and_requests_immediate_release(self):
        current = self._make_joint_state()
        current.position = safe_positions(torso_yaw=0.0)
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": safe_positions(), "velocities": [0.0] * 25},
            {"timestamp": 2000, "positions": safe_positions(torso_yaw=0.5), "velocities": [0.0] * 25},
        ]
        publisher = next(
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        )
        publish_started = threading.Event()
        release_publish = threading.Event()
        original_publish = publisher.publish
        blocked_once = False

        def block_first_override(message):
            nonlocal blocked_once
            if message.weight == 1.0 and not blocked_once:
                blocked_once = True
                publish_started.set()
                release_publish.wait(timeout=3.0)
            original_publish(message)

        publisher.publish = block_first_override
        interrupt_group = self.dev.MotionInterruptGroup()
        interrupt_group.register(
            "motion_recorder",
            self.plugin.interrupt_motion,
            self.plugin.motion_active,
        )
        self.plugin.set_interrupt_group(interrupt_group)
        harness = AgentCoreBarrierContractHarness()
        harness.register(self.plugin.get_tool(), self.plugin.dispatch)
        first = harness.call("motion_recorder", "play", {})
        self.assertTrue(publish_started.wait(timeout=1.0))
        stopped_result = {}
        stop_finished = threading.Event()

        def stop_through_agent_core():
            stopped_result.update(
                harness.call("motion_recorder", "stop_playback", {})
            )
            stop_finished.set()

        stop_thread = threading.Thread(target=stop_through_agent_core, daemon=True)
        stop_thread.start()

        try:
            returned_promptly = stop_finished.wait(timeout=0.02)
            pending_after_stop = harness.pending_actions()
            blocking_before_release = interrupt_group.blocking_outputs()
            released_promptly = bool(
                publisher.messages and publisher.messages[-1].weight == 0.0
            )
        finally:
            release_publish.set()
            stop_thread.join(timeout=2.0)

        self.assertTrue(returned_promptly)
        self.assertEqual(set(), pending_after_stop)
        self.assertEqual(["motion_recorder"], blocking_before_release)
        self.assertTrue(released_promptly)
        self.assertEqual("stopping", stopped_result["state"])
        deadline = time.monotonic() + 1.0
        while not any(
            call[0] == first["action_id"] and call[1] == "cancelled"
            for call in self.acp_calls
        ) and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(any(
            call[0] == first["action_id"] and call[1] == "cancelled"
            for call in self.acp_calls
        ))
        self.assertEqual(0.0, publisher.messages[-1].weight)
        self.assertEqual([], interrupt_group.blocking_outputs())

        restarted = self._play({})
        self.assertEqual("playing", restarted["state"])
        self.assertNotEqual(first["action_id"], restarted["action_id"])
        self.plugin.dispatch("stop_playback", {})

    def test_play_interrupt_cancels_integrated_reset_and_waits_for_idle(self):
        class ResetJointPlan:
            def __init__(self):
                self.calls = []
                self.status = {"request_id": 17, "status": 2, "progress": 0.2}
                self.entered_wait = threading.Event()

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                if action == "reset":
                    return {"state": "requested", "request_id": 17}
                if action == "status":
                    return dict(self.status)
                if action == "cancel":
                    return {"state": "requested", "request_id": 17}
                return {"error": f"unexpected action: {action}"}

            def wait_for_request(self, _request_id, _timeout, cancel_event):
                self.entered_wait.set()
                while not cancel_event.wait(0.01):
                    pass
                raise RuntimeError("reset cancelled")

        joint_plan = ResetJointPlan()
        self.plugin.set_joint_plan(joint_plan)
        current = self._make_joint_state()
        current.position = safe_positions()
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": safe_positions()},
            {"timestamp": 50, "positions": safe_positions()},
        ]
        interrupt_group = self.dev.MotionInterruptGroup()
        interrupt_group.register(
            "motion_recorder",
            self.plugin.interrupt_motion,
            self.plugin.motion_active,
        )
        self.plugin.set_interrupt_group(interrupt_group)
        harness = AgentCoreBarrierContractHarness()
        harness.register(self.plugin.get_tool(), self.plugin.dispatch)

        resetting = harness.call("motion_recorder", "play", {})
        self.assertTrue(joint_plan.entered_wait.wait(timeout=1.0))
        self.assertEqual({resetting["action_id"]}, harness.pending_actions())
        interrupted = harness.call("motion_recorder", "stop_playback", {})

        self.assertEqual(set(), harness.pending_actions())
        self.assertEqual("cancelling", interrupted["reset_result"]["state"])
        self.assertEqual(["motion_recorder"], interrupt_group.blocking_outputs())
        joint_plan.status = {"request_id": 17, "status": 1, "progress": 1.0}
        self.plugin._prepare_thread.join(timeout=1.0)
        deadline = time.monotonic() + 1.0
        while interrupt_group.blocking_outputs() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual([], interrupt_group.blocking_outputs())
        self.assertTrue(any(
            call[0] == resetting["action_id"] and call[1] == "cancelled"
            for call in self.acp_calls
        ))

    def test_reset_idle_wait_has_a_bounded_timeout(self):
        class StuckJointPlan:
            def dispatch(self, action, _args):
                if action == "status":
                    return {"request_id": 71, "status": 2, "progress": 0.5}
                return {"error": f"unexpected action: {action}"}

        started = time.monotonic()
        with self.assertRaises(TimeoutError):
            self.plugin._wait_reset_idle(
                71, StuckJointPlan(), timeout=0.02
            )
        self.assertLess(time.monotonic() - started, 0.2)

    def test_reset_settle_rejects_initial_zero_progress_idle(self):
        class InitialIdleJointPlan:
            def dispatch(self, action, _args):
                if action == "status":
                    return {
                        "request_id": 73,
                        "status": 1,
                        "progress": 0.0,
                    }
                return {"error": f"unexpected action: {action}"}

        with self.assertRaises(TimeoutError):
            self.plugin._wait_reset_idle(
                73, InitialIdleJointPlan(), timeout=0.02
            )

    def test_stuck_reset_posts_one_acp_error_and_recovers_on_late_idle(self):
        class StuckJointPlan:
            def __init__(self):
                self.status = {
                    "request_id": 72,
                    "status": 2,
                    "progress": 0.5,
                }

            def dispatch(self, action, args):
                if action == "reset":
                    return {"state": "requested", "request_id": 72}
                if action == "cancel":
                    return {
                        "state": "requested",
                        "request_id": args["request_id"],
                    }
                if action == "status":
                    return dict(self.status)
                return {"error": f"unexpected action: {action}"}

            def wait_for_request(self, *_args, **_kwargs):
                raise TimeoutError("reset feedback timed out")

        joint_plan = StuckJointPlan()
        self.plugin.set_joint_plan(joint_plan)
        self.plugin._reset_settle_timeout_sec = 0.02

        started = self.plugin.dispatch(
            "record_start", {"label": "stuck_reset"}
        )
        self.plugin._prepare_thread.join(timeout=0.3)

        self.assertFalse(self.plugin._prepare_thread.is_alive())
        self.assertEqual(started["action_id"], self.acp_calls[0][0])
        self.assertEqual("error", self.acp_calls[0][1])
        status = self.plugin.dispatch("status", {})
        self.assertFalse(status["preparing"])
        self.assertTrue(status["reset_pending"])
        self.assertEqual(
            "settling_after_error", status["last_reset"]["state"]
        )

        joint_plan.status = {
            "request_id": 72,
            "status": 1,
            "progress": 1.0,
        }
        recovered = self.plugin.dispatch("status", {})
        self.assertFalse(recovered["reset_pending"])
        self.assertEqual(1, len(self.acp_calls))

    def test_release_failure_keeps_motion_gate_settling_until_retry_succeeds(self):
        current = self._make_joint_state()
        current.position = safe_positions(torso_yaw=0.0)
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": safe_positions(), "velocities": [0.0] * 25},
            {"timestamp": 2000, "positions": safe_positions(torso_yaw=0.5), "velocities": [0.0] * 25},
        ]
        publisher = next(
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        )
        original_publish = publisher.publish
        fail_release = True

        def fail_zero_weight(message):
            if message.weight == 0.0 and fail_release:
                raise RuntimeError("release transport failed")
            original_publish(message)

        publisher.publish = fail_zero_weight
        interrupt_group = self.dev.MotionInterruptGroup()
        interrupt_group.register(
            "motion_recorder",
            self.plugin.interrupt_motion,
            self.plugin.motion_active,
        )
        self.plugin.set_interrupt_group(interrupt_group)
        started = self._play({})
        time.sleep(0.05)
        self.plugin.dispatch("stop_playback", {})
        deadline = time.monotonic() + 1.0
        while self.plugin.dispatch("status", {})["playing"] and time.monotonic() < deadline:
            time.sleep(0.01)

        self.assertEqual(["motion_recorder"], interrupt_group.blocking_outputs())
        self.assertTrue(self.plugin.dispatch("status", {})["override_release_failed"])
        self.assertTrue(any(
            call[0] == started["action_id"] and call[1] == "cancelled"
            for call in self.acp_calls
        ))

        fail_release = False
        retried = self.plugin.dispatch("stop_playback", {})
        self.assertEqual("idle", retried["state"])
        self.assertFalse(self.plugin.dispatch("status", {})["override_release_failed"])
        self.assertEqual([], interrupt_group.blocking_outputs())

    def test_natural_completion_release_failure_is_also_fail_closed(self):
        current = self._make_joint_state()
        current.position = safe_positions(torso_yaw=0.0)
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": safe_positions(), "velocities": [0.0] * 25},
            {"timestamp": 50, "positions": safe_positions(torso_yaw=0.1), "velocities": [0.0] * 25},
        ]
        publisher = next(
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        )
        original_publish = publisher.publish
        fail_release = True

        def fail_zero_weight(message):
            if message.weight == 0.0 and fail_release:
                raise RuntimeError("natural release failed")
            original_publish(message)

        publisher.publish = fail_zero_weight
        interrupt_group = self.dev.MotionInterruptGroup()
        interrupt_group.register(
            "motion_recorder",
            self.plugin.interrupt_motion,
            self.plugin.motion_active,
        )
        self.plugin.set_interrupt_group(interrupt_group)
        self._play({})
        deadline = time.monotonic() + 1.0
        while self.plugin.dispatch("status", {})["playing"] and time.monotonic() < deadline:
            time.sleep(0.01)

        self.assertTrue(self.plugin.dispatch("status", {})["override_release_failed"])
        self.assertEqual(["motion_recorder"], interrupt_group.blocking_outputs())

        fail_release = False
        self.plugin.dispatch("stop_playback", {})
        self.assertFalse(self.plugin.dispatch("status", {})["override_release_failed"])
        self.assertEqual([], interrupt_group.blocking_outputs())

    def test_device_interrupt_group_cancels_cross_plugin_pending_motion(self):
        class BlockingJointPlan:
            def __init__(self):
                self.calls = []
                self.entered_wait = threading.Event()
                self.release_wait = threading.Event()
                self.status = {"request_id": 31, "status": 2, "progress": 0.5}

            def current_motion(self):
                return "lower_body_balance", []

            def wait_until_idle(self, *_args, **_kwargs):
                return {}

            def wait_for_request(self, *_args, **_kwargs):
                self.entered_wait.set()
                self.release_wait.wait(timeout=1.0)
                return {}

            def acquire_head(self, _owner):
                return None

            def release_head(self, _owner):
                pass

            def _dispatch_owned(self, _owner, action, args):
                return self.dispatch(action, args)

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                if action in ("plan", "plan_named"):
                    return {"state": "requested", "request_id": 31}
                if action == "cancel":
                    return {"state": "requested", "request_id": 31}
                if action == "status":
                    return dict(self.status)
                return {"error": f"unexpected action: {action}"}

        plan = BlockingJointPlan()
        gesture = self.dev.GesturePlugin(plan)
        gesture_acp_calls = []
        gesture._acp_notify = lambda *args: gesture_acp_calls.append(args)
        interrupt_group = self.dev.MotionInterruptGroup()
        interrupt_group.register("gesture", gesture.halt, gesture.motion_active)
        interrupt_group.register(
            "motion_recorder",
            self.plugin.interrupt_motion,
            self.plugin.motion_active,
        )
        gesture.set_interrupt_group(interrupt_group)
        self.plugin.set_interrupt_group(interrupt_group)
        harness = AgentCoreBarrierContractHarness()
        harness.register(gesture.get_tool(), gesture.dispatch)
        harness.register(self.plugin.get_tool(), self.plugin.dispatch)

        gesture_started = harness.call("gesture", "sequence", {
            "steps": [{
                "joint_indices": [23],
                "target_positions": [0.1],
                "duration": 0.05,
            }],
            "reset_after": False,
            "force": True,
        })
        self.assertTrue(plan.entered_wait.wait(timeout=1.0))
        self.assertEqual(
            {gesture_started["action_id"]},
            harness.pending_actions(),
        )
        recorder_interrupt = harness.call(
            "motion_recorder", "stop_playback", {}
        )
        self.assertEqual(set(), harness.pending_actions())
        self.assertEqual("cancelled", gesture.dispatch("status", {})["state"])
        self.assertIn("gesture", recorder_interrupt["interrupted_outputs"])
        self.assertTrue(any(action == "cancel" for action, _ in plan.calls))
        plan.status = {"request_id": 31, "status": 1, "progress": 1.0}
        plan.release_wait.set()
        gesture._thread.join(timeout=1.0)
        deadline = time.monotonic() + 1.0
        while gesture.dispatch("status", {})["cancel_pending"] and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(any(
            call[0] == gesture_started["action_id"] and call[1] == "cancelled"
            for call in gesture_acp_calls
        ))
        self.assertEqual([], interrupt_group.blocking_outputs())

        current = self._make_joint_state()
        current.position = safe_positions(torso_yaw=0.0)
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": safe_positions(), "velocities": [0.0] * 25},
            {"timestamp": 2000, "positions": safe_positions(torso_yaw=0.5), "velocities": [0.0] * 25},
        ]
        playback_started = harness.call("motion_recorder", "play", {})
        time.sleep(0.05)
        gesture_interrupt = harness.call(
            "gesture", "stop_gesture", {"reset_after": True}
        )
        self.assertEqual(set(), harness.pending_actions())
        self.assertIn("motion_recorder", gesture_interrupt["interrupted_outputs"])
        self.assertTrue(gesture_interrupt["reset_after_ignored"])
        self.assertFalse(any(action == "reset" for action, _ in plan.calls))
        deadline = time.monotonic() + 1.0
        while not any(
            call[0] == playback_started["action_id"] and call[1] == "cancelled"
            for call in self.acp_calls
        ) and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(any(
            call[0] == playback_started["action_id"] and call[1] == "cancelled"
            for call in self.acp_calls
        ))
        publisher = next(
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        )
        self.assertEqual(0.0, publisher.messages[-1].weight)

    def test_recording_auto_stops_if_motion_mode_changes(self):
        state = FakeMotionState(current="lower_body_balance", available=["pd_stand"])
        self.plugin.set_reset_controls(state, FakeMotionMode(state))
        self._record_start({"label": "mode_guard"})
        self.plugin._on_joint_state(self._make_joint_state())
        state.current = "pd_stand"
        self.plugin._on_joint_state(self._make_joint_state())

        status = self.plugin.dispatch("status", {})
        self.assertFalse(status["recording"])
        self.assertEqual("motion_state_changed", status["last_recording"]["stop_reason"])

    def test_unknown_action_returns_error(self):
        result = self.plugin.dispatch("foobar", {})
        self.assertIn("error", result)

    def _make_joint_state(self):
        msg = Message()
        msg.position = safe_positions()
        msg.velocity = [float(i) * 0.01 for i in range(25)]
        return msg


if __name__ == "__main__":
    unittest.main()
