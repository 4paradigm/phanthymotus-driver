"""Contract tests for the GaitPlugin and MotionRecorderPlugin (from device.py)."""

import importlib.util
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


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
        self.current = current
        self.available = list(available or [])

    def current_motion(self):
        return self.current, list(self.available)


class FakeMotionMode:
    def __init__(self, state):
        self.state = state
        self.calls = []

    def dispatch(self, action, args):
        self.calls.append((action, dict(args)))
        target = args["target"]
        if self.state.available and target not in self.state.available and not args.get("force", False):
            return {"error": f"{target} is not available", "available": self.state.available}
        if args.get("wait", True):
            self.state.current = target
            return {"state": "completed", "current": target, "available": self.state.available}
        return {"state": "requested", "target": target, "previous": self.state.current}


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
            ["basic", "balanced", "terrain"],
            tool["inputSchema"]["properties"]["gait"]["enum"],
        )

    def test_start_is_actuator_lifecycle_ready(self):
        self.assertEqual("ready", self.plugin.dispatch("start", {})["state"])

    def test_list_reports_runtime_availability(self):
        result = self.plugin.dispatch("list", {})
        self.assertEqual("ready", result["state"])
        profiles = {item["name"]: item for item in result["profiles"]}
        self.assertEqual("rl_basic", profiles["basic"]["resolved_motion_state"])
        self.assertTrue(profiles["basic"]["available"])
        self.assertFalse(profiles["terrain"]["available"])

    def test_select_basic_prefers_runtime_available_rl_basic(self):
        result = self.plugin.dispatch("select", {"gait": "basic", "wait": False})
        self.assertEqual("requested", result["state"])
        self.assertEqual(
            ("switch", {"target": "rl_basic", "force": False, "wait": False}),
            self.motion_mode.calls[-1],
        )

    def test_select_basic_adapts_to_legacy_walk_state(self):
        self.state.available = ["walk"]
        result = self.plugin.dispatch("select", {"gait": "basic", "wait": False})
        self.assertEqual("requested", result["state"])
        self.assertEqual("walk", self.motion_mode.calls[-1][1]["target"])

    def test_select_rejects_unavailable_profile_before_publish(self):
        result = self.plugin.dispatch("select", {"gait": "terrain"})
        self.assertIn("error", result)
        self.assertEqual([], self.motion_mode.calls)

    def test_force_allows_unreported_profile(self):
        result = self.plugin.dispatch("select", {"gait": "terrain", "force": True, "wait": False})
        self.assertEqual("requested", result["state"])
        self.assertEqual("rl_terrain", self.motion_mode.calls[-1][1]["target"])

    def test_status_reflects_current_gait(self):
        self.state.current = "lower_body_balance"
        result = self.plugin.dispatch("status", {})
        self.assertEqual("active", result["state"])
        self.assertEqual("balanced", result["gait"])
        self.assertEqual("lower_body_balance", result["motion_state"])

    def test_unknown_action_returns_error(self):
        result = self.plugin.dispatch("foobar", {})
        self.assertIn("error", result)


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
        self.acp_calls = []
        self.plugin._acp_notify = lambda *args: self.acp_calls.append(args)

    def tearDown(self):
        self.plugin.stop()
        self.tmpdir.cleanup()

    def test_tool_schema_declares_all_actions(self):
        tool = self.plugin.get_tool()
        self.assertEqual("motion_recorder", tool["name"])
        actions = tool["inputSchema"]["properties"]["action"]["enum"]
        expected = {"record_start", "record_stop", "play", "stop_playback",
                    "reset", "save", "load", "list", "delete", "status", "info"}
        for action in expected:
            self.assertIn(action, actions)
        self.assertEqual(
            {"actions": ["play", "reset"], "timeout": 3600},
            tool["inputSchema"]["x-completion"],
        )

    def test_status_returns_idle_initially(self):
        result = self.plugin.dispatch("status", {})
        self.assertEqual("idle", result["state"])

    def test_start_returns_actuator_ready_without_starting_recording(self):
        result = self.plugin.dispatch("start", {})
        self.assertEqual("ready", result["state"])
        self.assertEqual("idle", result["activity_state"])
        self.assertFalse(result["recording"])

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
        result = self.plugin.dispatch("record_start", {"duration": -1})
        self.assertIn("error", result)
        self.assertFalse(self.plugin.dispatch("status", {})["recording"])

    def test_record_start_begins_recording(self):
        result = self.plugin.dispatch("record_start", {"label": "test_record"})
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

    def test_next_recording_waits_for_default_pose_reset_to_finish(self):
        class ResetJointPlan:
            def __init__(self):
                self.calls = []
                self.status = {"request_id": 7, "status": 2, "progress": 0.2}

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                if action == "reset":
                    return {"state": "requested", "request_id": 7, "request_type": 2}
                if action == "status":
                    return dict(self.status)
                return {"error": f"unexpected action: {action}"}

        state = FakeMotionState(
            current="lower_body_balance",
            available=["pd_stand"],
        )
        motion_mode = FakeMotionMode(state)
        joint_plan = ResetJointPlan()
        self.plugin.set_joint_plan(joint_plan)
        self.plugin.set_reset_controls(state, motion_mode)

        self.plugin.dispatch("record_start", {"label": "first_action"})
        self.plugin._on_joint_state(self._make_joint_state())
        stopped = self.plugin.dispatch("record_stop", {})
        self.assertEqual("saved", stopped["state"])

        blocked = self.plugin.dispatch("record_start", {"label": "too_early"})
        self.assertIn("reset required", blocked["error"])

        resetting = self.plugin.dispatch("reset", {})
        self.assertEqual("resetting", resetting["state"])
        self.assertIn(("reset", {}), joint_plan.calls)
        repeated = self.plugin.dispatch("reset", {})
        self.assertEqual(resetting["action_id"], repeated["action_id"])
        self.assertTrue(repeated["already_resetting"])
        self.assertEqual(1, sum(action == "reset" for action, _ in joint_plan.calls))
        pending = self.plugin.dispatch("status", {})
        self.assertTrue(pending["needs_reset"])
        self.assertTrue(pending["reset_pending"])

        joint_plan.status = {"request_id": 7, "status": 1, "progress": 1.0}
        ready = self.plugin.dispatch("status", {})
        self.assertFalse(ready["needs_reset"])
        self.assertFalse(ready["reset_pending"])
        self.assertTrue(any(
            call[0] == resetting["action_id"] and call[1] == "completed"
            for call in self.acp_calls
        ))
        started = self.plugin.dispatch("record_start", {"label": "second_action"})
        self.assertEqual("recording", started["state"])

    def test_reset_safely_enters_lower_body_balance_before_default_pose(self):
        class ResetJointPlan:
            def __init__(self):
                self.calls = []

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                if action == "reset":
                    return {"state": "requested", "request_id": 9, "request_type": 2}
                if action == "status":
                    return {"request_id": 9, "status": 2, "progress": 0.1}
                return {"error": f"unexpected action: {action}"}

        state = FakeMotionState(
            current="pd_stand",
            available=["lower_body_balance"],
        )
        motion_mode = FakeMotionMode(state)
        joint_plan = ResetJointPlan()
        self.plugin.set_joint_plan(joint_plan)
        self.plugin.set_reset_controls(state, motion_mode)

        blocked = self.plugin.dispatch("record_start", {"label": "unsafe_mode"})
        self.assertIn("lower_body_balance", blocked["error"])
        current = self._make_joint_state()
        current.position = [0.0] * 25
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": [0.0] * 25, "velocities": [0.0] * 25},
            {"timestamp": 50, "positions": [0.1] * 25, "velocities": [0.0] * 25},
        ]
        blocked_play = self.plugin.dispatch("play", {})
        self.assertIn("lower_body_balance", blocked_play["error"])

        result = self.plugin.dispatch("reset", {})
        self.assertEqual("resetting", result["state"])
        self.assertEqual(
            ("switch", {
                "target": "lower_body_balance",
                "force": False,
                "wait": True,
            }),
            motion_mode.calls[-1],
        )
        self.assertEqual("lower_body_balance", state.current)
        self.assertIn(("reset", {}), joint_plan.calls)

    def test_record_start_is_idempotent_instead_of_toggling_off(self):
        first = self.plugin.dispatch("record_start", {"label": "first"})
        second = self.plugin.dispatch("record_start", {"label": "second"})
        self.assertEqual("recording", first["state"])
        self.assertEqual("recording", second["state"])
        self.assertTrue(second["already_recording"])
        self.assertEqual("first", second["label"])
        self.assertTrue(self.plugin.dispatch("status", {})["recording"])

    def test_record_start_does_not_block_waiting_for_joint_data(self):
        started = time.monotonic()
        result = self.plugin.dispatch("record_start", {"label": "responsive"})
        elapsed = time.monotonic() - started
        self.assertEqual("recording", result["state"])
        self.assertLess(elapsed, 0.2)
        self.assertFalse(result["joint_data_available"])

    def test_recording_with_duration_auto_stops(self):
        result = self.plugin.dispatch("record_start", {"label": "timed", "duration": 0.1})
        self.assertEqual("recording", result["state"])
        self.plugin._on_joint_state(self._make_joint_state())
        time.sleep(0.2)
        status = self.plugin.dispatch("status", {})
        self.assertFalse(status["recording"])
        self.assertEqual("duration", status["last_recording"]["stop_reason"])
        self.assertTrue(Path(status["last_recording"]["file"]).exists())

    def test_old_auto_stop_cannot_stop_new_recording(self):
        self.plugin.dispatch("record_start", {"label": "timed", "duration": 0.05})
        time.sleep(0.01)
        self.plugin.dispatch("record_stop", {})
        self.plugin.dispatch("record_start", {"label": "second"})
        time.sleep(0.08)
        status = self.plugin.dispatch("status", {})
        self.assertTrue(status["recording"])
        self.assertEqual("second", status["record_label"])

    def test_record_hz_throttles_joint_callbacks(self):
        self.plugin.dispatch("record_start", {"label": "throttled"})
        for _ in range(10):
            self.plugin._on_joint_state(self._make_joint_state())
        result = self.plugin.dispatch("record_stop", {})
        self.assertEqual(1, result["frames"])

    def test_save_persists_buffer_to_disk(self):
        self.plugin.dispatch("record_start", {"label": "save_test"})
        self.plugin._on_joint_state(self._make_joint_state())
        self.plugin.dispatch("record_stop", {})
        result = self.plugin.dispatch("save", {"name": "my_recording", "label": "My Recording"})
        self.assertEqual("saved", result["state"])
        self.assertTrue(Path(result["file"]).exists())

    def test_save_without_frames_returns_error(self):
        result = self.plugin.dispatch("save", {"name": "empty"})
        self.assertIn("error", result)

    def test_list_returns_saved_recordings(self):
        self.plugin.dispatch("record_start", {"label": "list_test"})
        self.plugin._on_joint_state(self._make_joint_state())
        self.plugin.dispatch("record_stop", {})
        result = self.plugin.dispatch("list", {})
        self.assertEqual("ready", result["state"])
        self.assertGreaterEqual(result["count"], 1)

    def test_load_reads_recording_from_disk(self):
        self.plugin.dispatch("record_start", {"label": "load_test"})
        self.plugin._on_joint_state(self._make_joint_state())
        self.plugin.dispatch("record_stop", {})
        self.plugin.dispatch("save", {"name": "load_test_recording"})
        self.plugin.dispatch("record_start", {"label": "dummy"})
        self.plugin.dispatch("record_stop", {})
        result = self.plugin.dispatch("load", {"name": "load_test_recording"})
        self.assertEqual("loaded", result["state"])
        self.assertGreater(result["frames"], 0)

    def test_delete_removes_recording_file(self):
        self.plugin.dispatch("record_start", {"label": "del_test"})
        self.plugin._on_joint_state(self._make_joint_state())
        self.plugin.dispatch("record_stop", {})
        self.plugin.dispatch("save", {"name": "to_delete"})
        result = self.plugin.dispatch("delete", {"name": "to_delete"})
        self.assertEqual("deleted", result["state"])
        list_result = self.plugin.dispatch("list", {})
        names = [r["name"] for r in list_result.get("recordings", [])]
        self.assertNotIn("to_delete", names)

    def test_play_without_name_or_buffer_returns_error(self):
        result = self.plugin.dispatch("play", {})
        self.assertIn("error", result)

    def test_play_without_buffer_and_unknown_name_returns_error(self):
        result = self.plugin.dispatch("play", {"name": "nonexistent"})
        self.assertIn("error", result)

    def test_play_rejects_single_frame_recording(self):
        self.plugin.dispatch("record_start", {"label": "play_test"})
        self.plugin._on_joint_state(self._make_joint_state())
        self.plugin.dispatch("record_stop", {})
        self.plugin.dispatch("save", {"name": "play_test_rec"})
        result = self.plugin.dispatch("play", {"name": "play_test_rec"})
        self.assertIn("at least two", result["error"])

    def test_playback_state_after_play_with_joint_plan(self):
        class FakeJointPlan:
            def __init__(self):
                self.calls = []

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                return {"state": "requested", "request_id": 1}

        joint_plan = FakeJointPlan()
        self.plugin.set_joint_plan(joint_plan)
        current = self._make_joint_state()
        current.position = [0.0] * 25
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {
                "timestamp": index * 50,
                "positions": [0.0] * 12 + [0.12 * index] * 13,
                "velocities": [0.0] * 25,
            }
            for index in range(5)
        ]

        result = self.plugin.dispatch("play", {"speed_scale": 1.0})
        self.assertEqual("playing", result["state"])
        deadline = time.monotonic() + 2.0
        while self.plugin.dispatch("status", {})["playing"] and time.monotonic() < deadline:
            time.sleep(0.01)

        self.assertEqual([], joint_plan.calls, "playback must not restart joint_plan for every frame")
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
        current.position = [0.0] * 12 + [-0.5] * 13
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {
                "timestamp": index * 50,
                "positions": [0.0] * 12 + [0.5 + 0.05 * index] * 13,
                "velocities": [0.0] * 25,
            }
            for index in range(3)
        ]

        result = self.plugin.dispatch("play", {"speed_scale": 1.0})
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
        current.position = [0.0] * 25
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {
                "timestamp": index * 100,
                "positions": [0.0] * 12 + [position] * 13,
                "velocities": [0.0] * 25,
            }
            for index, position in enumerate((0.0, 1.0, 0.0))
        ]

        self.plugin.dispatch("play", {"speed_scale": 1.0})
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
        current.position = [0.0] * 25
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": [0.0] * 25, "velocities": [0.0] * 25},
            {"timestamp": 50, "positions": [0.1] * 25, "velocities": [0.0] * 25},
        ]
        publisher = next(
            publisher
            for publisher in self.plugin._node.publishers
            if publisher.topic == "/motion/joint_override_command"
        )

        def fail_publish(_message):
            raise RuntimeError("publisher unavailable")

        publisher.publish = fail_publish
        result = self.plugin.dispatch("play", {})
        self.assertEqual("playing", result["state"])
        deadline = time.monotonic() + 1.0
        while self.plugin.dispatch("status", {})["playing"] and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertFalse(self.plugin.dispatch("status", {})["playing"])

    def test_playback_releases_override_if_motion_mode_changes(self):
        state = FakeMotionState(current="lower_body_balance", available=["pd_stand"])
        self.plugin.set_reset_controls(state, FakeMotionMode(state))
        current = self._make_joint_state()
        current.position = [0.0] * 25
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": [0.0] * 25, "velocities": [0.0] * 25},
            {"timestamp": 1000, "positions": [0.5] * 25, "velocities": [0.0] * 25},
        ]

        self.plugin.dispatch("play", {})
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
        current.position = [0.0] * 25
        current.velocity = [0.0] * 25
        self.plugin._on_joint_state(current)
        self.plugin._frames = [
            {"timestamp": 0, "positions": [0.0] * 25, "velocities": [0.0] * 25},
            {"timestamp": 2000, "positions": [0.5] * 25, "velocities": [0.0] * 25},
        ]

        started = self.plugin.dispatch("play", {})
        time.sleep(0.05)
        stopped = self.plugin.dispatch("stop_playback", {})
        self.assertEqual("stopped", stopped["state"])
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

    def test_recording_auto_stops_if_motion_mode_changes(self):
        state = FakeMotionState(current="lower_body_balance", available=["pd_stand"])
        self.plugin.set_reset_controls(state, FakeMotionMode(state))
        self.plugin.dispatch("record_start", {"label": "mode_guard"})
        self.plugin._on_joint_state(self._make_joint_state())
        state.current = "pd_stand"
        self.plugin._on_joint_state(self._make_joint_state())

        status = self.plugin.dispatch("status", {})
        self.assertFalse(status["recording"])
        self.assertEqual("motion_state_changed", status["last_recording"]["stop_reason"])

    def test_unknown_action_returns_error(self):
        result = self.plugin.dispatch("foobar", {})
        self.assertIn("error", result)

    def test_info_returns_status(self):
        status_result = self.plugin.dispatch("status", {})
        info_result = self.plugin.dispatch("info", {})
        self.assertEqual(status_result["state"], info_result["state"])

    def _make_joint_state(self):
        msg = Message()
        msg.position = [float(i) * 0.1 for i in range(25)]
        msg.velocity = [float(i) * 0.01 for i in range(25)]
        return msg


if __name__ == "__main__":
    unittest.main()
