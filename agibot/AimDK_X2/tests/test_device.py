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
import json
import math
from unittest import mock
from types import SimpleNamespace
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
        self.subscriptions.append((topic, callback))
        return object()

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


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.closed = False

    def settimeout(self, value):
        pass

    def connect(self, path):
        self.path = path

    def sendall(self, data):
        self.sent.append(data)

    def close(self):
        self.closed = True


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
    module("rclpy.serialization", serialize_message=lambda msg: b"serialized")

    module("sensor_msgs")
    module("sensor_msgs.msg", CameraInfo=FakeMsg, CompressedImage=FakeMsg, Image=FakeMsg, Imu=FakeMsg, PointCloud2=FakeMsg)
    module("std_msgs")
    module("std_msgs.msg", String=FakeMsg, UInt8MultiArray=FakeMsg)
    module("geometry_msgs")
    module("geometry_msgs.msg", Pose=FakeMsg)
    module("nav_msgs")
    module("nav_msgs.msg", Odometry=FakeMsg)

    module("aimdk_msgs")
    module(
        "aimdk_msgs.msg",
        CommonRequest=FakeMsg,
        McCommonState=FakeMsg,
        PmuState=FakeMsg,
        TouchState=FakeMsg,
        HandCommand=FakeMsg,
        HandCommandArray=FakeMsg,
        HandStateArray=FakeMsg,
        JointStateArray=FakeMsg,
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
import x2_bus_bridge  # noqa: E402
import x2_bridged_publisher  # noqa: E402
import x2_camera_frame  # noqa: E402


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
        plugins = build_bundle_plugins()
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
            "mc_mode", "locomotion", "preset_motion", "joint_command",
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
        for name in ("mc_state", "joints", "joint_state", "imu", "leg_odometry", "camera_rgb", "camera_rgb_frame", "head_touch", "pmu_state", "system_state", "linkcraft_catalog"):
            self.assertEqual(by_name[name], "sensor")

    def test_unavailable_hardware_cards_are_not_registered_by_default(self):
        names = {definition["name"] for definition in tool_definitions(build_bundle_plugins())}
        self.assertTrue({"hand_state", "hand_command", "camera_depth", "lidar", "slam_pose", "slam_control"}.isdisjoint(names))

    def test_optional_hardware_cards_register_only_when_enabled(self):
        plugins = build_bundle_plugins({"end_effector": "hand", "plugins": {
            "hand_state": {"enabled": True}, "hand_command": {"enabled": True},
            "camera_depth": {"enabled": True}, "lidar": {"enabled": True}, "slam": {"enabled": True},
        }})
        names = {definition["name"] for definition in tool_definitions(plugins)}
        self.assertTrue({"hand_state", "hand_command", "camera_depth", "lidar", "slam_pose", "slam_control"}.issubset(names))

    def test_confirmed_sensor_topics_are_wired(self):
        plugins = build_bundle_plugins()
        nodes = plugins[0].nodes
        expected = {
            "leg_odometry": "/aima/mc/leg_odometry",
            "head_touch": "/aima/hal/sensor/touch_head",
            "pmu_state": "/aima/hal/pmu/state",
        }
        for name, topic in expected.items():
            self.assertEqual(nodes.streams[name]["robot_topic"], topic)
            self.assertEqual(nodes.streams[name]["format"], "data/json")

    def test_camera_rgb_frame_replaces_public_camera_info_card(self):
        definitions = {item["name"]: item for item in tool_definitions(build_bundle_plugins())}
        self.assertNotIn("camera_info", definitions)
        frame = definitions["camera_rgb_frame"]
        self.assertEqual(frame["topic_out"][0]["format"], x2_camera_frame.ENVELOPE_FORMAT)
        self.assertEqual(frame["topic_out"][0]["schema"], x2_camera_frame.RGB_SCHEMA)

    def test_camera_rgb_frame_envelope_uses_ros_calibration_and_nominal_urdf_extrinsic(self):
        plugins = build_bundle_plugins()
        nodes = plugins[0].nodes
        nodes.camera_rgb_pub = FakePublisher(FakeMsg, "/test_ns/agibot_x2/camera_rgb", 5)
        nodes.camera_frame_pub = FakePublisher(FakeMsg, "/test_ns/agibot_x2/camera_rgb_frame", 5)
        callbacks = {topic: callback for topic, callback in nodes.robot.subscriptions}
        info = SimpleNamespace(
            header=SimpleNamespace(frame_id="rgb_head_center_link"), width=640, height=480,
            distortion_model="plumb_bob", d=[0.1, 0.2], k=[1.0] * 9, r=[1.0] * 9, p=[1.0] * 12,
        )
        callbacks["/aima/hal/sensor/rgb_head_front_center/camera_info"](info)
        image = SimpleNamespace(
            header=SimpleNamespace(frame_id="rgb_head_center_link", stamp=SimpleNamespace(sec=1, nanosec=2)),
            data=b"jpeg-data",
        )
        callbacks["/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed"](image)
        metadata, payload = x2_camera_frame.decode_envelope(bytes(nodes.camera_frame_pub.published[-1].data))
        self.assertEqual(payload, b"jpeg-data")
        self.assertEqual(metadata["schema"], x2_camera_frame.RGB_SCHEMA)
        extrinsic = metadata["calibration"]["base_to_camera"]
        self.assertEqual(extrinsic["target_frame"], "pelvis")
        self.assertEqual(extrinsic["status"], "nominal_zero_joint_pose")
        self.assertEqual(len(extrinsic["matrix_4x4_row_major"]), 16)

    def test_leg_odometry_is_distinct_from_slam_and_supports_lifecycle(self):
        plugins = build_bundle_plugins()
        nodes = plugins[0].nodes
        odometry = find_plugin(plugins, "leg_odometry")
        self.assertEqual(nodes.streams["leg_odometry"]["topic"], "/test_ns/agibot_x2/leg_odometry")
        self.assertEqual(odometry.dispatch("start", {}), {"state": "running"})
        self.assertEqual(odometry.dispatch("stop", {}), {"state": "idle"})
        self.assertNotIn("slam_odom", nodes.streams)

    def test_joints_skeleton_topic_and_payload_contract(self):
        plugins = build_bundle_plugins()
        nodes = plugins[0].nodes
        joints = find_plugin(plugins, "joints")
        self.assertEqual(nodes.streams["joints"]["format"], "sensor/skeleton")
        self.assertEqual(nodes.streams["joints"]["topic"], "/test_ns/state/joints")
        self.assertEqual(joints.dispatch("info", {})["data"]["joint_count"], 0)

    def test_joints_start_is_explicitly_supported(self):
        plugins = build_bundle_plugins()
        joints = find_plugin(plugins, "joints")
        self.assertEqual(joints.dispatch("start", {}), {"state": "running"})

    def test_joints_unknown_action_is_not_reported_as_running(self):
        joints = find_plugin(build_bundle_plugins(), "joints")
        self.assertIsNone(joints.dispatch("misspelled_action", {}))

    def test_joints_skeleton_uses_selected_urdf_variant(self):
        for variant, expected_count in (("hand", 27), ("fist", 27), ("ultra", 31)):
            plugins = build_bundle_plugins({"end_effector": variant, "plugins": {}})
            nodes = plugins[0].nodes
            self.assertEqual(sum(len(names) for names in nodes.skeleton_joints.values()), expected_count)

    def test_joint_state_callbacks_publish_skeleton_values(self):
        plugins = build_bundle_plugins()
        nodes = plugins[0].nodes
        joints_plugin = find_plugin(plugins, "joints")
        nodes.skeleton_pub = FakePublisher(FakeMsg, "/test_ns/state/joints", 5)
        callbacks = {topic: callback for topic, callback in nodes.robot.subscriptions}

        leg_names = nodes.skeleton_joints["leg"]
        leg_topic = "/aima/hal/joint/leg/state"
        leg_message = SimpleNamespace(joints=[
            SimpleNamespace(name=leg_names[0], position=0.1, velocity=0.2, effort=0.3, error_code=0),
            SimpleNamespace(name=leg_names[1], position=-0.4, velocity=0.5, effort=0.6, error_code=7),
        ])
        callbacks[leg_topic](leg_message)

        arm_names = nodes.skeleton_joints["arm"]
        arm_topic = "/aima/hal/joint/arm/state"
        callbacks[arm_topic](SimpleNamespace(joints=[
            SimpleNamespace(name=arm_names[0], position=1.0, velocity=1.1, effort=1.2, error_code=0),
        ]))

        payload = json.loads(nodes.skeleton_pub.published[-1].data)
        self.assertEqual(payload["format"], "sensor/skeleton")
        self.assertEqual(payload["joint_count"], 3)
        self.assertEqual(payload["joints"][0], {
            "idx": nodes.skeleton_joint_indices[leg_names[0]],
            "name": leg_names[0], "q": 0.1, "dq": 0.2, "tau": 0.3,
        })
        self.assertEqual(payload["joints"][1]["name"], leg_names[1])
        self.assertEqual(payload["joints"][1]["error_code"], 7)
        self.assertEqual(payload["joints"][2], {
            "idx": nodes.skeleton_joint_indices[arm_names[0]],
            "name": arm_names[0], "q": 1.0, "dq": 1.1, "tau": 1.2,
        })
        self.assertIsInstance(nodes.skeleton_pub.published[-1], FakeMsg)
        self.assertEqual(joints_plugin.dispatch("info", {})["data"], payload)

    def test_joint_state_callbacks_match_named_states_and_skip_unknowns(self):
        plugins = build_bundle_plugins()
        nodes = plugins[0].nodes
        nodes.skeleton_pub = FakePublisher(FakeMsg, "/test_ns/state/joints", 5)
        callbacks = {topic: callback for topic, callback in nodes.robot.subscriptions}
        leg_names = nodes.skeleton_joints["leg"]
        leg_topic = "/aima/hal/joint/leg/state"

        callbacks[leg_topic](SimpleNamespace(joints=[
            SimpleNamespace(name=leg_names[1], position=2.0, velocity=2.1, effort=2.2, error_code=0),
            SimpleNamespace(name="vendor_extra_joint", position=9.0, velocity=9.1, effort=9.2, error_code=0),
            SimpleNamespace(name=leg_names[0], position=1.0, velocity=1.1, effort=1.2, error_code=0),
        ]))

        payload = json.loads(nodes.skeleton_pub.published[-1].data)
        self.assertEqual(payload["joint_count"], 2)
        self.assertEqual([joint["name"] for joint in payload["joints"]], [leg_names[1], leg_names[0]])
        self.assertEqual(payload["joints"][0]["idx"], nodes.skeleton_joint_indices[leg_names[1]])
        self.assertEqual(payload["joints"][1]["idx"], nodes.skeleton_joint_indices[leg_names[0]])
        self.assertEqual(payload["diagnostics"]["unknown_joint_names"], ["vendor_extra_joint"])

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


class X2BridgeTests(unittest.TestCase):
    def test_bridged_publisher_does_not_retain_sensor_frames(self):
        publisher = x2_bridged_publisher.BridgedPublisher(FakeMsg, "/camera")
        fake_socket = FakeSocket()
        with mock.patch.object(x2_bridged_publisher.socket, "socket", return_value=fake_socket):
            for _ in range(32):
                publisher.publish(FakeMsg())
        self.assertEqual(publisher._count, 32)
        self.assertFalse(hasattr(publisher, "published"))

    def test_fastdds_bridge_profile_is_loopback_only(self):
        self.assertTrue(x2_bus_bridge.DEFAULT_FASTDDS_PROFILE.name.endswith("fastdds_bridge_local.xml"))
        text = x2_bus_bridge.DEFAULT_FASTDDS_PROFILE.read_text(encoding="utf-8")
        self.assertIn("<address>127.0.0.1</address>", text)
        self.assertIn("<useBuiltinTransports>false</useBuiltinTransports>", text)

    def test_select_sensor_tools_ignores_non_sensor_and_duplicate_topics(self):
        tools = [
            {"name": "camera_info", "type": "sensor", "topic_out": [
                {"topic": "/a", "format": "data/json"},
                {"topic": "/a", "format": "data/json"},
                {"topic": "/b", "format": "image/jpeg"},
            ]},
            {"name": "joint_command", "type": "actuator", "topic_out": [
                {"topic": "/cmd", "format": "data/json"},
            ]},
        ]
        self.assertEqual(x2_bus_bridge.select_sensor_tools(tools), {"camera_info": ["/a"]})

    def test_extract_data_payload_requires_data_field(self):
        good = {"result": {"content": [{"text": json.dumps({"data": {"ok": True}})}]}}
        self.assertEqual(x2_bus_bridge.extract_data_payload(good), {"ok": True})
        bad = {"result": {"content": [{"text": json.dumps({"value": 1})}]}}
        with self.assertRaises(ValueError):
            x2_bus_bridge.extract_data_payload(bad)

    def test_poll_failure_logging_is_bounded_and_reports_recovery(self):
        class FailingMcp:
            def sensor_info(self, name):
                raise RuntimeError("temporary failure")

        warnings = []
        infos = []
        bridge = x2_bus_bridge.SensorBusBridge(
            FailingMcp(),
            lambda topic, data: None,
            log_warning=warnings.append,
            log_info=infos.append,
        )
        bridge._sensors = {"imu": ["/imu"]}
        for _ in range(20):
            bridge.poll_once()
        self.assertEqual(len(warnings), 1)
        self.assertIn("1 consecutive poll", warnings[0])

        class RecoveredMcp:
            def sensor_info(self, name):
                return {"ok": True}

        bridge._mcp = RecoveredMcp()
        self.assertEqual(bridge.poll_once(), 1)
        self.assertEqual(len(infos), 1)
        self.assertIn("recovered after 20 failed polls", infos[0])

    def test_poll_failure_message_is_bounded(self):
        class FailingMcp:
            def sensor_info(self, name):
                raise RuntimeError("x" * 1000)

        warnings = []
        bridge = x2_bus_bridge.SensorBusBridge(
            FailingMcp(), lambda topic, data: None, log_warning=warnings.append
        )
        bridge._sensors = {"imu": ["/imu"]}
        bridge.poll_once()
        self.assertLessEqual(len(warnings[0]), 320)


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
        with mock.patch.object(device, "call_service", wraps=device.call_service) as call:
            mc_mode.dispatch(action, {})
        self.assertEqual(call.call_args.kwargs["timeout"], 20.0)
        sent = nodes.set_mc_action.last_request
        self.assertEqual(sent.command.action.value, device.MC_ACTIONS[action])
        self.assertEqual(sent.command.action_desc, action.upper())

    def test_mc_mode_exposes_only_live_firmware_actions_and_returns_acp_id(self):
        plugins = build_bundle_plugins()
        nodes = plugins[0].nodes
        nodes.set_mc_action.response = SimpleNamespace(response=SimpleNamespace(header=SimpleNamespace(code=0)))
        nodes._mc_mode_state = {
            "action_desc": "DAMPING_DEFAULT", "action_status": 100, "fsm_state": 4,
        }
        mc_mode = find_plugin(plugins, "mc_mode")
        definition = next(item for item in tool_definitions(plugins) if item["name"] == "mc_mode")
        actions = definition["inputSchema"]["properties"]["action"]["enum"]
        self.assertEqual(actions, ["damping_default"])
        self.assertNotIn("passive_default", actions)
        self.assertNotIn("stand_default", actions)
        self.assertNotIn("stand_up_default", actions)
        self.assertNotIn("zero_torque_default", actions)
        with mock.patch.object(device, "_acp_notify"):
            result = mc_mode.dispatch("damping_default", {})
        self.assertEqual(result["state"], "accepted")
        self.assertTrue(result["action_id"].startswith("x2_mc_mode_"))

    def test_mc_mode_confirmation_requires_live_non_transition_state(self):
        plugins = build_bundle_plugins()
        mc_mode = find_plugin(plugins, "mc_mode")
        mc_mode.nodes._mc_mode_state = {
            "action_desc": "STAND_DEFAULT", "action_status": 200, "fsm_state": 1,
        }
        with mock.patch.object(device, "_acp_notify") as notify:
            with mock.patch.object(device.time, "monotonic", side_effect=[0, 0, 31]):
                mc_mode._wait_for_mode_confirmation("id", "STAND_DEFAULT", "stand_default", 30)
        self.assertEqual(notify.call_args.args[1], "error")

    def test_mc_mode_falls_back_when_image_lacks_mc_common_state(self):
        plugins = build_bundle_plugins()
        mc_mode = find_plugin(plugins, "mc_mode")
        mc_mode.nodes.mc_state_available = False
        with mock.patch.object(device.threading, "Thread") as thread_cls:
            result = mc_mode.dispatch("damping_default", {})
        self.assertEqual(result["state"], "accepted")
        self.assertEqual(result["confirmation"], "service_accepted_state_unavailable")
        self.assertIs(thread_cls.call_args.kwargs["target"], device._acp_notify)
        self.assertEqual(thread_cls.call_args.kwargs["args"][1], "completed")

    def test_all_actuators_declare_physical_resources(self):
        plugins = build_bundle_plugins({"end_effector": "fist", "plugins": {"slam": {"enabled": True}}})
        for definition in tool_definitions(plugins):
            if definition["type"] == "actuator":
                self.assertIn("x-resource", definition["inputSchema"], definition["name"])

    def test_locomotion_registers_a_dedicated_source_before_publishing(self):
        plugins = build_bundle_plugins()
        locomotion = find_plugin(plugins, "locomotion")
        definition = next(item for item in tool_definitions(plugins) if item["name"] == "locomotion")
        actions = definition["inputSchema"]["properties"]["action"]["enum"]
        self.assertEqual(actions, ["move", "cancel"])
        locomotion.dispatch("move", {"forward": 0.5, "duration": -1})
        self.assertTrue(locomotion._registered)
        request = locomotion.nodes.set_mc_input_source.last_request
        self.assertEqual(request.action.value, 1001)
        self.assertEqual(request.input_source.name, "motus_x2")
        self.assertEqual(request.input_source.priority, 81)
        self.assertEqual(request.input_source.timeout, 1000)
        self.assertEqual(len(locomotion.nodes.locomotion_pub.published), 1)
        self.assertEqual(locomotion.nodes.locomotion_pub.published[0].forward_velocity, 0.5)
        self.assertEqual(locomotion.nodes.locomotion_pub.published[0].source, "motus_x2")

        locomotion.dispatch("move", {"forward": 0.0, "angular": 180.0, "duration": -1})
        self.assertAlmostEqual(
            locomotion.nodes.locomotion_pub.published[-1].angular_velocity,
            math.pi,
        )

    def test_locomotion_managed_source_rejection_prevents_publish(self):
        plugins = build_bundle_plugins()
        locomotion = find_plugin(plugins, "locomotion")
        response = FakeMsg()
        response.header.code = 1
        locomotion.nodes.set_mc_input_source.response = SimpleNamespace(response=response)
        with self.assertRaisesRegex(RuntimeError, "registration rejected"):
            locomotion.dispatch("move", {"duration": 1})
        self.assertEqual(locomotion.nodes.locomotion_pub.published, [])
        self.assertEqual(locomotion.nodes.set_mc_input_source.last_request.action.value, 1002)

    def test_locomotion_duration_is_bounded_and_schedules_a_stop(self):
        plugins = build_bundle_plugins()
        locomotion = find_plugin(plugins, "locomotion")
        locomotion.nodes.set_mc_input_source.response = FakeMsg()
        with mock.patch.object(device.threading, "Timer") as timer_cls:
            result = locomotion.dispatch("move", {"forward": 0.5, "duration": 1.5})
        self.assertEqual(result["state"], "accepted")
        self.assertEqual(result["duration"], 1.5)
        self.assertTrue(result["action_id"].startswith("x2_locomotion_"))
        self.assertEqual(locomotion.nodes.locomotion_pub.published[0].forward_velocity, 0.5)
        stop_call = next(call for call in timer_cls.call_args_list if call.args[0] == 1.5)
        heartbeat_call = next(call for call in timer_cls.call_args_list if call.args[0] == 0.02)
        heartbeat_call.args[1]()
        self.assertEqual(locomotion.nodes.locomotion_pub.published[-1].forward_velocity, 0.5)
        with mock.patch.object(device, "_acp_notify") as notify:
            stop_call.args[1]()
        zero = locomotion.nodes.locomotion_pub.published[-1]
        self.assertEqual((zero.forward_velocity, zero.lateral_velocity, zero.angular_velocity), (0.0, 0.0, 0.0))
        self.assertEqual(notify.call_args.args[1], "completed")
        self.assertFalse(locomotion._registered)
        self.assertEqual(locomotion.nodes.set_mc_input_source.last_request.action.value, 1003)
        definition = next(item for item in tool_definitions(plugins) if item["name"] == "locomotion")
        self.assertIn("duration", definition["inputSchema"]["properties"])
        with self.assertRaisesRegex(ValueError, "-1 or between 0.1 and 60"):
            locomotion.dispatch("move", {"forward": 0.5, "duration": 0})

    def test_locomotion_negative_one_is_continuous_and_cancel_stops(self):
        plugins = build_bundle_plugins()
        locomotion = find_plugin(plugins, "locomotion")
        locomotion.nodes.set_mc_input_source.response = FakeMsg()
        with mock.patch.object(device.threading, "Timer") as timer_cls:
            result = locomotion.dispatch("move", {"forward": 0.5, "duration": -1})
        self.assertEqual(result["state"], "accepted")
        self.assertEqual(result["duration"], -1)
        self.assertIsNone(locomotion._stop_timer)
        self.assertTrue(any(call.args[0] == 0.02 for call in timer_cls.call_args_list))
        action_id = result["action_id"]
        with mock.patch.object(device, "_acp_notify") as notify:
            result = locomotion.dispatch("cancel", {})
        self.assertEqual(result["state"], "cancelled")
        zero = locomotion.nodes.locomotion_pub.published[-1]
        self.assertEqual((zero.forward_velocity, zero.lateral_velocity, zero.angular_velocity), (0.0, 0.0, 0.0))
        self.assertFalse(locomotion._registered)
        self.assertEqual(locomotion.nodes.set_mc_input_source.last_request.action.value, 1003)
        self.assertEqual(notify.call_args.args[0], action_id)
        self.assertEqual(notify.call_args.args[1], "cancelled")

    def test_locomotion_requires_duration_and_defaults_forward_speed(self):
        plugins = build_bundle_plugins()
        locomotion = find_plugin(plugins, "locomotion")
        locomotion.nodes.set_mc_input_source.response = FakeMsg()
        with self.assertRaisesRegex(ValueError, "duration is required"):
            locomotion.dispatch("move", {})
        result = locomotion.dispatch("move", {"duration": -1})
        self.assertEqual(result["state"], "accepted")
        self.assertEqual(locomotion.nodes.locomotion_pub.published[-1].forward_velocity, 0.2)
        definition = next(item for item in tool_definitions(plugins) if item["name"] == "locomotion")
        self.assertEqual(definition["inputSchema"]["properties"]["forward"]["default"], 0.2)
        self.assertIn("allOf", definition["inputSchema"])

    def test_slam_relocalization_uses_vendor_map_id_command(self):
        plugins = build_bundle_plugins({"end_effector": "fist", "plugins": {"slam": {"enabled": True}}})
        slam = find_plugin(plugins, "slam_control")
        result = slam.dispatch("start_relocalization", {"map_id": 42})
        self.assertEqual(result["command"], "start_relocalization:42")
        self.assertEqual(slam.nodes.integrated_command_pub.published[-1].data, "start_relocalization:42")

    def test_hand_state_payload_marks_no_hand_hardware_unavailable(self):
        empty_sensors = SimpleNamespace(
            palm_touch_data=[0] * 36, back_of_hand_touch_data=[0] * 36,
            thumb_touch_data=[0] * 36, index_finger_touch_data=[0] * 36,
            middle_finger_touch_data=[0] * 36, ring_finger_touch_data=[0] * 36,
            little_finger_touch_data=[0] * 36,
        )
        payload = device.AimdkNodes._hand_state_payload(SimpleNamespace(
            left_hand_type=SimpleNamespace(value=0), right_hand_type=SimpleNamespace(value=0),
            left_hands=[], right_hands=[], left_touch_sensors=empty_sensors, right_touch_sensors=empty_sensors,
        ))
        self.assertFalse(payload["available"])
        self.assertEqual(payload["left"]["joint_count"], 0)
        self.assertEqual(payload["right"]["active_touch_channels"], 0)


class StartStopLifecycleTests(unittest.TestCase):
    """README_dev.md's 'start/stop in dispatch (Required)' rule: the canvas UI calls every
    tool with {"action": "start"} the moment its card is placed, and {"action": "stop"} when
    removed. A plugin that doesn't short-circuit on these either crashes (KeyError on a
    required field the probe never supplies) or — worse for actuator tools — actually
    performs the real action (registers an MC input source, publishes a command, flips an
    LED) just from a card being dragged onto the canvas. This regression-tests that every
    plugin handles both without touching a service client or publisher."""

    def _assert_inert(self, plugin, tool_name, nodes):
        publishers_before = {
            topic: len(pub.published) for topic, pub in nodes.robot.publishers.items()
        }
        for client in nodes.robot.clients.values():
            client.last_request = None

        for action in ("start", "stop"):
            result = plugin.dispatch(action, {"_tool_name": tool_name})
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
                self._assert_inert(plugin, definition["name"], nodes)


if __name__ == "__main__":
    unittest.main()
