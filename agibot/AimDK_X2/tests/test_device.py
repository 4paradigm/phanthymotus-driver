"""Pure-Python unit tests for agibot/AimDK_X2/device.py — no ROS installation required.

Stubs rclpy / sensor_msgs / std_msgs / geometry_msgs / nav_msgs / aimdk_msgs with minimal
fakes before importing device.py, so build_plugins() can run and produce a real tool
inventory to assert against, following the "no test suite for phanthymotus-driver, hand-built
verification" note in CLAUDE.md.
"""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

DEVICE_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = DEVICE_DIR.parent.parent

for path in (str(REPO_ROOT), str(DEVICE_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)


class FakeMsg:
    """Generic auto-vivifying stand-in for any ROS message. Attribute access on a field
    that hasn't been set yet returns (and caches) a fresh FakeMsg, so chained field
    assignment like `request.header.stamp = ...` and `request.command.action.value = ...`
    works without needing a real message schema."""

    # Names jsonable() probes via hasattr() to detect numpy-likes/dataclasses; must NOT
    # auto-vivify these or hasattr() reports a false positive and jsonable() calls it.
    _AUTOVIV_BLOCKLIST = {"tolist"}

    def __getattr__(self, name):
        if name.startswith("__") or name in FakeMsg._AUTOVIV_BLOCKLIST:
            raise AttributeError(name)
        value = FakeMsg()
        object.__setattr__(self, name, value)
        return value


class FakeSrv:
    Request = FakeMsg
    Response = FakeMsg


class FakePublisher:
    def __init__(self, msg_type, topic, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeFuture:
    def __init__(self, result=None, exc=None):
        self._result = result
        self._exc = exc

    def done(self):
        return True

    def result(self):
        return self._result

    def exception(self):
        return self._exc


class FakeClient:
    def __init__(self, srv_type, name):
        self.srv_type = srv_type
        self.srv_name = name
        self.response = None
        self.last_request = None

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, request):
        self.last_request = request
        return FakeFuture(result=self.response if self.response is not None else FakeMsg())


class FakeClock:
    def now(self):
        return self

    def to_msg(self):
        return "stamp"


class FakeNode:
    def __init__(self, name, context=None):
        self.name = name
        self.context = context
        self.publishers = {}
        self.subscriptions = []
        self.clients = {}

    def create_publisher(self, msg_type, topic, qos):
        pub = FakePublisher(msg_type, topic, qos)
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        subscription = types.SimpleNamespace(msg_type=msg_type, topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        if subscription in self.subscriptions:
            self.subscriptions.remove(subscription)
            return True
        return False

    def destroy_publisher(self, publisher):
        if self.publishers.get(publisher.topic) is publisher:
            del self.publishers[publisher.topic]
            return True
        return False

    def create_client(self, srv_type, name):
        client = FakeClient(srv_type, name)
        self.clients[name] = client
        return client

    def get_clock(self):
        return FakeClock()

    def destroy_node(self):
        pass


class FakeQoSProfile:
    def __init__(self, depth=10, reliability=None, durability=None):
        self.depth = depth
        self.reliability = reliability
        self.durability = durability


class FakeQoSReliabilityPolicy:
    BEST_EFFORT = "BEST_EFFORT"


class FakeQoSDurabilityPolicy:
    TRANSIENT_LOCAL = "TRANSIENT_LOCAL"


class FakeExecutor:
    def add_node(self, node):
        pass


class FakeROS2:
    def __init__(self):
        self.ctx_robot = object()
        self.ctx_core = object()
        self.executor_robot = FakeExecutor()
        self.executor_core = FakeExecutor()


def _install_ros_stubs():
    """Register fake rclpy/message modules into sys.modules so device.py's deferred
    `from rclpy... import ...` / `from aimdk_msgs... import ...` calls resolve."""

    def module(name, **attrs):
        mod = types.ModuleType(name)
        for key, value in attrs.items():
            setattr(mod, key, value)
        sys.modules[name] = mod
        return mod

    rclpy = module("rclpy")
    module("rclpy.node", Node=FakeNode)
    module(
        "rclpy.qos",
        QoSProfile=FakeQoSProfile,
        QoSReliabilityPolicy=FakeQoSReliabilityPolicy,
        QoSDurabilityPolicy=FakeQoSDurabilityPolicy,
    )
    rclpy.node = sys.modules["rclpy.node"]
    rclpy.qos = sys.modules["rclpy.qos"]

    module("sensor_msgs")
    module("sensor_msgs.msg", CompressedImage=FakeMsg, Image=FakeMsg, Imu=FakeMsg, PointCloud2=FakeMsg)
    module("std_msgs")
    module("std_msgs.msg", String=FakeMsg)
    module("geometry_msgs")
    module("geometry_msgs.msg", Pose=FakeMsg)
    module("nav_msgs")
    module("nav_msgs.msg", Odometry=FakeMsg)

    module("aimdk_msgs")
    module(
        "aimdk_msgs.msg",
        CommonRequest=FakeMsg,
        HandCommand=FakeMsg,
        HandCommandArray=FakeMsg,
        HandStateArray=FakeMsg,
        JointCommand=FakeMsg,
        JointCommandArray=FakeMsg,
        McLocomotionVelocity=FakeMsg,
    )
    srv_names = [
        "ExecuteActionResource", "GetAllJointState", "GetCurrentInputSource", "GetHandType",
        "GetMcAction", "GetMicSourceRequest", "GetRobotResources", "GetStoredMapByName",
        "GetSystemState", "PlayEmoji", "PlayTts", "SetMcAction", "SetMcInputSource",
        "SetMcPresetMotion", "SetMicSourceRequest", "SetPmuLed",
    ]
    module("aimdk_msgs.srv", **{name: FakeSrv for name in srv_names})


_install_ros_stubs()

import yaml  # noqa: E402

import device  # noqa: E402
from common.vendor_runtime import DriverBundle  # noqa: E402


def load_driver_yaml_cards():
    with open(DEVICE_DIR / "driver.yaml", encoding="utf-8") as handle:
        manifest = yaml.safe_load(handle)
    return {card["name"]: card["type"] for card in manifest["cards"]}


def build_bundle_plugins(config=None):
    config = config if config is not None else {"end_effector": "hand", "plugins": {}}
    return device.build_plugins(config, "test_ns", FakeROS2())


def tool_definitions(plugins):
    definitions = []
    for plugin in plugins:
        definitions.extend(plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()])
    return definitions


def find_plugin(plugins, tool_name):
    for plugin in plugins:
        definitions = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
        if any(d["name"] == tool_name for d in definitions):
            return plugin
    raise KeyError(tool_name)


class ToolInventoryTests(unittest.TestCase):
    def test_tool_names_and_types_match_driver_yaml(self):
        plugins = build_bundle_plugins({"end_effector": "hand", "plugins": {"slam": {"enabled": True}}})
        definitions = tool_definitions(plugins)
        by_name = {d["name"]: d["type"] for d in definitions}
        expected = load_driver_yaml_cards()
        self.assertEqual(set(by_name), set(expected), "tool inventory must match driver.yaml cards exactly")
        for name, expected_type in expected.items():
            self.assertEqual(by_name[name], expected_type, f"tool '{name}' type mismatch")

    def test_no_duplicate_tool_names(self):
        plugins = build_bundle_plugins()
        names = [d["name"] for d in tool_definitions(plugins)]
        self.assertEqual(len(names), len(set(names)), "tool names must be unique")

    def test_slam_control_gated_by_config(self):
        plugins_off = build_bundle_plugins({"end_effector": "hand", "plugins": {}})
        names_off = {d["name"] for d in tool_definitions(plugins_off)}
        self.assertNotIn("slam_control", names_off)

        plugins_on = build_bundle_plugins({"end_effector": "hand", "plugins": {"slam": {"enabled": True}}})
        names_on = {d["name"] for d in tool_definitions(plugins_on)}
        self.assertIn("slam_control", names_on)

    def test_actuator_tools_are_typed_actuator(self):
        plugins = build_bundle_plugins()
        definitions = tool_definitions(plugins)
        expected_actuators = {
            "mc_mode", "locomotion", "preset_motion", "joint_command", "hand_command",
            "linkcraft", "pmu_led", "tts", "emoji", "mic_source",
        }
        by_name = {d["name"]: d["type"] for d in definitions}
        for name in expected_actuators:
            self.assertEqual(by_name[name], "actuator", f"'{name}' must be an actuator tool")

    def test_sensor_and_resource_tools_carry_expected_types(self):
        plugins = build_bundle_plugins()
        by_name = {d["name"]: d["type"] for d in tool_definitions(plugins)}
        self.assertEqual(by_name["model"], "resource")
        self.assertEqual(by_name["map_get"], "processor")
        for name in ("mc_state", "joint_state", "hand_state", "imu", "camera_rgb", "camera_depth", "lidar", "slam_pose"):
            self.assertEqual(by_name[name], "sensor")

    def test_mc_mode_and_preset_motion_action_enums_nonempty(self):
        plugins = build_bundle_plugins()
        by_name = {d["name"]: d for d in tool_definitions(plugins)}
        mc_mode_actions = by_name["mc_mode"]["inputSchema"]["properties"]["action"]["enum"]
        preset_actions = by_name["preset_motion"]["inputSchema"]["properties"]["action"]["enum"]
        self.assertEqual(set(mc_mode_actions), set(device.MC_ACTIONS))
        self.assertEqual(set(preset_actions), set(device.PRESET_MOTIONS))


class ModelPluginTests(unittest.TestCase):
    def test_urdf_served_for_each_vendored_variant(self):
        for variant in ("fist", "hand", "ultra"):
            plugins = build_bundle_plugins({"end_effector": variant, "plugins": {}})
            model_plugin = find_plugin(plugins, "model")
            result = model_plugin.dispatch("model", {})
            self.assertIn("urdf", result)
            self.assertIn("<robot", result["urdf"])

    def test_unknown_end_effector_variant_raises(self):
        plugins = build_bundle_plugins({"end_effector": "hand", "plugins": {}})
        model_plugin = find_plugin(plugins, "model")
        with self.assertRaises(ValueError):
            model_plugin.dispatch("model", {"variant": "nonexistent"})


class DispatchSmokeTests(unittest.TestCase):
    """Exercise a couple of simple service-backed dispatch() calls end-to-end against the
    fake ROS client, to catch request/response field mismatches (as opposed to only
    checking tool metadata)."""

    def test_mc_state_dispatch_returns_dict(self):
        plugins = build_bundle_plugins()
        nodes = plugins[0].nodes
        nodes.get_mc_action.response = FakeMsg()
        mc_state = find_plugin(plugins, "mc_state")
        result = mc_state.dispatch("mc_state", {})
        self.assertIsInstance(result, dict)

    def test_mc_mode_dispatch_sets_request_fields(self):
        plugins = build_bundle_plugins()
        nodes = plugins[0].nodes
        nodes.set_mc_action.response = FakeMsg()
        mc_mode = find_plugin(plugins, "mc_mode")
        action = next(iter(device.MC_ACTIONS))
        mc_mode.dispatch(action, {})
        sent = nodes.set_mc_action.last_request
        self.assertEqual(sent.command.action.value, device.MC_ACTIONS[action])

    def test_locomotion_registers_before_first_velocity_publish(self):
        plugins = build_bundle_plugins()
        locomotion = find_plugin(plugins, "locomotion")
        locomotion.nodes.set_mc_input_source.response = FakeMsg()
        locomotion.dispatch("set_velocity", {"forward": 0.5})
        self.assertTrue(locomotion._registered)
        self.assertEqual(len(locomotion.nodes.locomotion_pub.published), 1)
        self.assertEqual(locomotion.nodes.locomotion_pub.published[0].forward_velocity, 0.5)


class CameraMultiInstanceTests(unittest.TestCase):
    def setUp(self):
        self.plugins = build_bundle_plugins()
        self.camera = find_plugin(self.plugins, "camera_rgb")
        self.nodes = self.camera.nodes

    def test_rgb_tool_is_multi_instance_with_instance_source_config(self):
        tool = next(d for d in self.camera.get_tools() if d["name"] == "camera_rgb")
        self.assertTrue(tool["multiInstance"])
        self.assertIn("config", tool["inputSchema"]["properties"]["action"]["enum"])
        source = tool["configSchema"]["properties"]["camera_source"]
        self.assertEqual(source["scope"], "instance")
        self.assertEqual(source["default"], device.DEFAULT_CAMERA_RGB_SOURCE)
        self.assertEqual(
            {choice["const"] for choice in source["oneOf"]},
            set(device.CAMERA_RGB_SOURCES),
        )

    def test_info_returns_deterministic_instance_topic_before_start(self):
        result = self.camera.dispatch("info", {"_tool_name": "camera_rgb", "instance_id": "card-a-1"})
        self.assertEqual(result["state"], "idle")
        self.assertEqual(result["camera_source"], "interaction")
        self.assertEqual(
            result["topic_out"],
            [{"topic": "/test_ns/agibot_x2/camera_rgb/card_a_1_94b5805e", "format": "image/jpeg"}],
        )

    def test_missing_instance_id_is_rejected_for_every_rgb_action(self):
        for action in ("info", "start", "stop", "config"):
            with self.subTest(action=action), self.assertRaises(ValueError):
                self.camera.dispatch(action, {"_tool_name": "camera_rgb"})

    def test_lossy_ros_name_sanitizing_does_not_merge_instance_topics(self):
        dashed = self.camera.dispatch(
            "info", {"_tool_name": "camera_rgb", "instance_id": "card-a"},
        )["topic"]
        underscored = self.camera.dispatch(
            "info", {"_tool_name": "camera_rgb", "instance_id": "card_a"},
        )["topic"]
        self.assertNotEqual(dashed, underscored)

    def test_unicode_instance_id_starts_with_ascii_only_topic(self):
        result = self.camera.dispatch(
            "start", {"_tool_name": "camera_rgb", "instance_id": "camera-摄"},
        )
        self.assertEqual(result["state"], "running")
        self.assertTrue(result["topic"].isascii())
        self.assertNotIn("摄", result["topic"])

    def test_two_instances_can_stream_different_sources_concurrently(self):
        self.camera.dispatch("config", {
            "_tool_name": "camera_rgb", "instance_id": "front-card", "camera_source": "interaction",
        })
        self.camera.dispatch("config", {
            "_tool_name": "camera_rgb", "instance_id": "rear-card", "camera_source": "rear",
        })
        front = self.camera.dispatch("start", {"_tool_name": "camera_rgb", "instance_id": "front-card"})
        rear = self.camera.dispatch("start", {"_tool_name": "camera_rgb", "instance_id": "rear-card"})

        self.assertEqual(front["state"], "running")
        self.assertEqual(rear["state"], "running")
        self.assertNotEqual(front["topic"], rear["topic"])
        active_topics = {sub.topic for sub in self.nodes.robot.subscriptions}
        self.assertIn(device.CAMERA_RGB_SOURCES["interaction"], active_topics)
        self.assertIn(device.CAMERA_RGB_SOURCES["rear"], active_topics)

        rgb_subs = [
            sub for sub in self.nodes.robot.subscriptions
            if sub.topic in device.CAMERA_RGB_SOURCES.values()
        ]
        self.assertEqual(len(rgb_subs), 2)

    def test_bundle_start_consumes_nested_canvas_instance_config(self):
        bundle = DriverBundle(self.plugins)
        result = bundle.dispatch("camera_rgb", {
            "action": "start",
            "instance_id": "rear-from-canvas",
            "config": {"camera_source": "rear"},
        })

        self.assertEqual(result["state"], "running")
        self.assertEqual(result["camera_source"], "rear")
        self.assertEqual(result["robot_topic"], device.CAMERA_RGB_SOURCES["rear"])
        self.assertIn(
            device.CAMERA_RGB_SOURCES["rear"],
            {subscription.topic for subscription in self.nodes.robot.subscriptions},
        )

    def test_frame_is_forwarded_unchanged_and_counted(self):
        result = self.camera.dispatch("start", {"_tool_name": "camera_rgb", "instance_id": "card-frame"})
        source_topic = device.CAMERA_RGB_SOURCES["interaction"]
        subscription = next(sub for sub in self.nodes.robot.subscriptions if sub.topic == source_topic)
        frame = FakeMsg()
        subscription.callback(frame)

        publisher = self.nodes.core.publishers[result["topic"]]
        self.assertEqual(publisher.published, [frame])
        info = self.camera.dispatch("info", {"_tool_name": "camera_rgb", "instance_id": "card-frame"})
        self.assertEqual(info["frames_published"], 1)

    def test_running_instance_hot_switches_without_affecting_other_instance(self):
        self.camera.dispatch("start", {"_tool_name": "camera_rgb", "instance_id": "switch-me"})
        self.camera.dispatch("start", {"_tool_name": "camera_rgb", "instance_id": "keep-me"})
        keep_topic = self.camera.dispatch(
            "info", {"_tool_name": "camera_rgb", "instance_id": "keep-me"},
        )["topic"]

        switched = self.camera.dispatch("config", {
            "_tool_name": "camera_rgb", "instance_id": "switch-me", "camera_source": "stereo_left",
        })
        self.assertEqual(switched["state"], "running")
        self.assertEqual(switched["robot_topic"], device.CAMERA_RGB_SOURCES["stereo_left"])
        self.assertIn(keep_topic, self.nodes.core.publishers)
        rgb_subs = [
            sub.topic for sub in self.nodes.robot.subscriptions
            if sub.topic in device.CAMERA_RGB_SOURCES.values()
        ]
        self.assertEqual(rgb_subs.count(device.CAMERA_RGB_SOURCES["interaction"]), 1)
        self.assertEqual(rgb_subs.count(device.CAMERA_RGB_SOURCES["stereo_left"]), 1)

    def test_stop_releases_only_target_instance_and_is_idempotent(self):
        self.camera.dispatch("start", {"_tool_name": "camera_rgb", "instance_id": "a"})
        self.camera.dispatch("config", {
            "_tool_name": "camera_rgb", "instance_id": "b", "camera_source": "rear",
        })
        self.camera.dispatch("start", {"_tool_name": "camera_rgb", "instance_id": "b"})

        stopped = self.camera.dispatch("stop", {"_tool_name": "camera_rgb", "instance_id": "a"})
        stopped_again = self.camera.dispatch("stop", {"_tool_name": "camera_rgb", "instance_id": "a"})
        running = self.camera.dispatch("info", {"_tool_name": "camera_rgb", "instance_id": "b"})
        self.assertEqual(stopped["state"], "idle")
        self.assertEqual(stopped_again["state"], "idle")
        self.assertEqual(running["state"], "running")

    def test_invalid_source_is_rejected(self):
        with self.assertRaises(ValueError):
            self.camera.dispatch("config", {
                "_tool_name": "camera_rgb", "instance_id": "bad", "camera_source": "made_up",
            })

    def test_subscription_creation_failure_releases_provisional_instance(self):
        original = self.nodes.robot.create_subscription

        def fail_create_subscription(*args, **kwargs):
            raise RuntimeError("subscription failed")

        self.nodes.robot.create_subscription = fail_create_subscription
        try:
            with self.assertRaises(RuntimeError):
                self.camera.dispatch(
                    "start", {"_tool_name": "camera_rgb", "instance_id": "broken"},
                )
        finally:
            self.nodes.robot.create_subscription = original

        info = self.camera.dispatch(
            "info", {"_tool_name": "camera_rgb", "instance_id": "broken"},
        )
        self.assertEqual(info["state"], "idle")
        self.assertNotIn(info["topic"], self.nodes.core.publishers)

    def test_depth_card_has_explicit_lifecycle_actions(self):
        for action, expected_state in (
            ("start", "running"),
            ("info", "running"),
            ("stop", "idle"),
        ):
            result = self.camera.dispatch(action, {"_tool_name": "camera_depth"})
            self.assertEqual(result["state"], expected_state)

        with self.assertRaises(ValueError):
            self.camera.dispatch("unknown", {"_tool_name": "camera_depth"})


class StartStopLifecycleTests(unittest.TestCase):
    """README_dev.md's 'start/stop in dispatch (Required)' rule: the canvas UI calls every
    tool with {"action": "start"} the moment its card is placed, and {"action": "stop"} when
    removed. A plugin that doesn't short-circuit on these either crashes (KeyError on a
    required field the probe never supplies) or — worse for actuator tools — actually
    performs the real action (registers an MC input source, publishes a command, flips an
    LED) just from a card being dragged onto the canvas. This regression-tests that every
    plugin handles both without touching a service client or publisher."""

    def _assert_inert(self, plugin, definition, nodes):
        tool_name = definition["name"]
        publishers_before = {
            topic: len(pub.published) for topic, pub in nodes.robot.publishers.items()
        }
        for client in nodes.robot.clients.values():
            client.last_request = None

        instance_args = {"instance_id": "lifecycle-test"} if definition.get("multiInstance") else {}
        for action in ("start", "stop"):
            result = plugin.dispatch(action, {"_tool_name": tool_name, **instance_args})
            self.assertIsInstance(result, dict, f"{tool_name}.dispatch({action!r}) must return a dict")
            self.assertIn("state", result, f"{tool_name}.dispatch({action!r}) must report a state")

        for topic, pub in nodes.robot.publishers.items():
            self.assertEqual(
                len(pub.published), publishers_before[topic],
                f"{tool_name}'s start/stop must not publish to {topic}",
            )
        for name, client in nodes.robot.clients.items():
            self.assertIsNone(client.last_request, f"{tool_name}'s start/stop must not call service {name}")

    def test_every_tool_handles_start_stop_without_side_effects(self):
        plugins = build_bundle_plugins({"end_effector": "hand", "plugins": {"slam": {"enabled": True}}})
        nodes = plugins[0].nodes
        for plugin in plugins:
            for definition in (plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]):
                if definition["type"] == "resource":
                    continue  # resource tools always dispatch with action == tool name, never start/stop
                self._assert_inert(plugin, definition, nodes)


if __name__ == "__main__":
    unittest.main()
