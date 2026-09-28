"""Pure-Python unit tests for agibot/A3/device.py — no ROS installation required.

Stubs rclpy / sensor_msgs / std_msgs / ros2_plugin_proto with minimal fakes
before importing device.py, so build_plugins() can run and produce a real tool
inventory to assert against (same pattern as agibot/AimDK_X2/tests/test_device.py,
per the "no test suite for phanthymotus-driver, hand-built verification" note in
CLAUDE.md). RPC-backed plugins are exercised through an in-memory transport that
records every HTTP JSON RPC call, so payload shapes are asserted too.
"""

from __future__ import annotations

import json
import sys
import threading
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
    assignment like `msg.header.stamp = ...` works without a real message schema."""

    # Names jsonable() probes via hasattr() to detect numpy-likes/dataclasses; must NOT
    # auto-vivify these or hasattr() reports a false positive and jsonable() calls it.
    _AUTOVIV_BLOCKLIST = {"tolist"}

    def __getattr__(self, name):
        if name.startswith("__") or name in FakeMsg._AUTOVIV_BLOCKLIST:
            raise AttributeError(name)
        value = FakeMsg()
        object.__setattr__(self, name, value)
        return value


class FakePublisher:
    def __init__(self, msg_type, topic, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.published = []
        self.lock = threading.Lock()

    def publish(self, msg):
        with self.lock:
            self.published.append(msg)

    @property
    def topic_name(self):
        return self.topic


class FakeNode:
    def __init__(self, name, context=None):
        self.name = name
        self.context = context
        self.publishers = {}
        self.subscriptions = []

    def create_publisher(self, msg_type, topic, qos):
        pub = FakePublisher(msg_type, topic, qos)
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((topic, callback))
        return object()

    def get_clock(self):
        return FakeMsg()  # .now().to_msg() auto-vivifies

    def destroy_node(self):
        pass


class FakeQoSProfile:
    def __init__(self, depth=10, reliability=None, durability=None):
        self.depth = depth
        self.reliability = reliability
        self.durability = durability


class FakeQoSReliabilityPolicy:
    BEST_EFFORT = "BEST_EFFORT"


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
    `from rclpy... import ...` / `from ros2_plugin_proto... import ...` resolve."""

    def module(name, **attrs):
        mod = types.ModuleType(name)
        for key, value in attrs.items():
            setattr(mod, key, value)
        sys.modules[name] = mod
        return mod

    rclpy = module("rclpy")
    module("rclpy.node", Node=FakeNode)
    module("rclpy.qos", QoSProfile=FakeQoSProfile, QoSReliabilityPolicy=FakeQoSReliabilityPolicy)
    rclpy.node = sys.modules["rclpy.node"]
    rclpy.qos = sys.modules["rclpy.qos"]

    module("sensor_msgs")
    module("sensor_msgs.msg", Image=FakeMsg, Imu=FakeMsg, JointState=FakeMsg, PointCloud2=FakeMsg)
    module("std_msgs")
    module("std_msgs.msg", String=FakeMsg)

    # ros2_plugin_proto/msg/RosMsgWrapper — device.py publishes command wrappers
    # through it; a plain FakeMsg works (serialization_type / .data attrs).
    module("ros2_plugin_proto")
    module("ros2_plugin_proto.msg", RosMsgWrapper=FakeMsg)

    # The a3_aimdk protobuf wheel is NOT stubbed: on a dev machine it is absent, which
    # is exactly the code path unit tests exercise (JSON fallback / stream withholding).


_install_ros_stubs()

import yaml  # noqa: E402

import device  # noqa: E402


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

BASE_CONFIG = {
    "rpc": {"hdu": "10.42.10.10", "adu": "10.42.10.11", "mdu": "10.42.10.12", "timeout": 5.0},
    "ros": {"robot_domain_id": 232, "core_domain_id": 42},
    "plugins": {},
}

# config.yaml's full plugin set (matching the shipped config), advanced modules ON so
# the gated tools are covered too.
FULL_PLUGINS = {
    "mc_state": {"enabled": True}, "joints": {"enabled": True}, "imu": {"enabled": True},
    "camera": {"enabled": True, "streams": ["head_left_fisheye", "chest_front_d457_rgb"]},
    "lidar": {"enabled": True}, "bms": {"enabled": True}, "emergency": {"enabled": True},
    "wakeup": {"enabled": True}, "skill_status": {"enabled": True},
    "alerts": {"enabled": True, "poll_interval": 5.0}, "mc_mode": {"enabled": True},
    "locomotion": {"enabled": True}, "arm_command": {"enabled": True},
    "hand_command": {"enabled": True}, "neck_command": {"enabled": True},
    "waist_command": {"enabled": True}, "arm_compliance": {"enabled": True},
    "motion_play": {"enabled": True}, "tts": {"enabled": True}, "media_play": {"enabled": True},
    "audio_play": {"enabled": True}, "volume": {"enabled": True, "max_volume": 70},
    "mic_source": {"enabled": True}, "interaction": {"enabled": True},
    "resources": {"enabled": True}, "face_play": {"enabled": True},
    "skill_play": {"enabled": True},
    "mapping": {"enabled": True}, "navigation": {"enabled": True},
    "relocalization": {"enabled": True}, "auto_charging": {"enabled": True},
}


def load_driver_yaml_cards():
    with open(DEVICE_DIR / "driver.yaml", encoding="utf-8") as handle:
        manifest = yaml.safe_load(handle)
    return {card["name"]: card["type"] for card in manifest["cards"]}


class RecordingTransport:
    """In-memory stand-in for `requests.post`: records (url, body) pairs and answers
    every call with a configurable per-service response (default: ok header)."""

    def __init__(self, responses=None):
        self.calls = []
        self.responses = dict(responses or {})

    def __call__(self, url, payload, timeout):
        self.calls.append((url, payload))
        service = url.split("/rpc/aimdk.protocol.", 1)[-1]
        for prefix, response in self.responses.items():
            if service.startswith(prefix):
                return response
        return {"header": {"code": "0"}}

    def calls_to(self, service, method):
        marker = f"/rpc/aimdk.protocol.{service}/{method}"
        return [(url, body) for url, body in self.calls if marker in url]


def build_bundle_plugins(config=None, transport=None):
    config = config if config is not None else json.loads(json.dumps(BASE_CONFIG))
    config.setdefault("plugins", {}).setdefault("camera", {}).setdefault(
        "streams", ["head_left_fisheye", "chest_front_d457_rgb"])
    if transport is None:
        transport = RecordingTransport()
    # Inject the shared transport through config so A3Rpc hands it to every RpcClient.
    config["_transport_for_tests"] = transport
    original_init = device.A3Rpc.__init__

    def patched_init(self, cfg, transport=None):
        original_init(self, cfg, cfg.pop("_transport_for_tests", None) or transport)

    device.A3Rpc.__init__ = patched_init
    try:
        return device.build_plugins(config, "test_ns", FakeROS2()), transport
    finally:
        device.A3Rpc.__init__ = original_init


def tool_definitions(plugins):
    definitions = []
    for plugin in plugins.values():
        definitions.extend(plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()])
    return definitions


def find_plugin(plugins, tool_name):
    for plugin in plugins.values():
        definitions = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
        if any(d["name"] == tool_name for d in definitions):
            return plugin
    raise KeyError(tool_name)


# ---------------------------------------------------------------------------
# Inventory / schema tests
# ---------------------------------------------------------------------------

class ToolInventoryTests(unittest.TestCase):
    def test_tool_names_and_types_match_driver_yaml(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        by_name = {d["name"]: d["type"] for d in tool_definitions(plugins)}
        expected = load_driver_yaml_cards()
        self.assertEqual(set(by_name), set(expected), "tool inventory must match driver.yaml cards exactly")
        for name, expected_type in expected.items():
            self.assertEqual(by_name[name], expected_type, f"tool '{name}' type mismatch")

    def test_no_duplicate_tool_names(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        names = [d["name"] for d in tool_definitions(plugins)]
        self.assertEqual(len(names), len(set(names)), "tool names must be unique")

    def test_advanced_modules_gated_by_config(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = dict(FULL_PLUGINS, mapping={"enabled": False},
                                 navigation={"enabled": False},
                                 relocalization={"enabled": False},
                                 auto_charging={"enabled": False})
        plugins, _ = build_bundle_plugins(config)
        names = {d["name"] for d in tool_definitions(plugins)}
        for gated in ("mapping", "map_get", "navigation", "relocalization", "auto_charging"):
            self.assertNotIn(gated, names)

    def test_actuator_tools_are_typed_actuator(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        by_name = {d["name"]: d["type"] for d in tool_definitions(plugins)}
        expected_actuators = {
            "resource_list", "mc_mode", "locomotion", "arm_command", "hand_command",
            "neck_command", "waist_command", "arm_compliance", "motion_play", "tts",
            "media_play", "audio_play", "volume", "mic_source", "interaction",
            "face_play", "skill_play", "mapping", "navigation", "relocalization",
            "auto_charging",
        }
        for name in expected_actuators:
            self.assertEqual(by_name[name], "actuator", f"'{name}' must be an actuator tool")

    def test_sensor_and_resource_tools_carry_expected_types(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        by_name = {d["name"]: d["type"] for d in tool_definitions(plugins)}
        self.assertEqual(by_name["model"], "resource")
        self.assertEqual(by_name["map_get"], "processor")
        for name in ("mc_state", "arm_state", "hand_state", "neck_state", "imu",
                     "camera", "lidar", "bms", "emergency", "wakeup", "skill_status",
                     "alerts"):
            self.assertEqual(by_name[name], "sensor", f"'{name}' must be a sensor tool")

    def test_pb_streams_withheld_without_wheel(self):
        # aimdk (the a3_aimdk wheel) is not installed in the test environment, so the
        # pb-decoded stream mirrors must be absent from nodes.streams and their plugins
        # fall back to the no-stream tool description.
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        nodes = next(iter(plugins.values())).nodes
        for key in ("bms", "emergency", "wakeup", "skill_status"):
            self.assertNotIn(key, nodes.streams)
        for name in ("bms", "emergency", "wakeup", "skill_status"):
            plugin = find_plugin(plugins, name)
            self.assertFalse(plugin.has_stream, f"{name} must not claim a stream without the wheel")

    def test_camera_streams_follow_config(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = dict(FULL_PLUGINS, camera={
            "enabled": True, "streams": ["head_left_fisheye"]})
        plugins, _ = build_bundle_plugins(config)
        camera = find_plugin(plugins, "camera")
        self.assertEqual(list(camera.streams), ["camera_head_left_fisheye"])


# ---------------------------------------------------------------------------
# Lifecycle start/stop tests (canvas drags a card on/off — must stay inert)
# ---------------------------------------------------------------------------

class StartStopLifecycleTests(unittest.TestCase):
    def _assert_inert(self, plugin, tool_name, nodes):
        publishers_before = {
            topic: len(pub.published) for topic, pub in nodes.robot.publishers.items()
        }

        for action in ("start", "stop"):
            result = plugin.dispatch(action, {"_tool_name": tool_name})
            self.assertIsInstance(result, dict, f"{tool_name}.dispatch({action!r}) must return a dict")
            self.assertIn("state", result, f"{tool_name}.dispatch({action!r}) must report a state")

        for topic, pub in nodes.robot.publishers.items():
            self.assertEqual(
                len(pub.published), publishers_before[topic],
                f"{tool_name}'s start/stop must not publish to {topic}",
            )

    def test_every_tool_handles_start_stop_without_side_effects(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        # fresh transport per run so nothing was recorded outside the lifecycle calls
        plugins, transport = build_bundle_plugins(config)
        nodes = next(iter(plugins.values())).nodes
        for name, plugin in plugins.items():
            self._assert_inert(plugin, name, nodes)
        self.assertEqual(transport.calls, [], "start/stop must not issue any RPC")


# ---------------------------------------------------------------------------
# RPC payload / publisher dispatch tests
# ---------------------------------------------------------------------------

class RpcDispatchTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(json.dumps(BASE_CONFIG))
        self.config["plugins"] = FULL_PLUGINS
        self.plugins, self.transport = build_bundle_plugins(self.config)
        self.nodes = next(iter(self.plugins.values())).nodes

    # -- mc_mode / mc_state (MDU :56322) --

    def test_mc_mode_set_action_hits_mdu_56322(self):
        mc_mode = find_plugin(self.plugins, "mc_mode")
        mc_mode.dispatch("get_up", {})
        (url, body), = self.transport.calls_to("MotionControlActionService", "SetAction")
        self.assertIn("10.42.10.12:56322", url)
        self.assertEqual(body["command"]["action"], "MotionControlAction_GET_UP")

    def test_mc_state_available_lists_actions(self):
        response = {"commands": ["MotionControlAction_GET_UP", "MotionControlAction_DAMPING"]}
        self.transport.responses["MotionControlActionService"] = response
        mc_state = find_plugin(self.plugins, "mc_state")
        result = mc_state.dispatch("available", {})
        self.assertEqual(result["commands"], response["commands"])

    def test_mc_mode_unknown_action_raises(self):
        mc_mode = find_plugin(self.plugins, "mc_mode")
        with self.assertRaises(ValueError):
            mc_mode.dispatch("moonwalk", {})

    # -- locomotion / waist / face (RosMsgWrapper publishers, robot domain) --

    def test_locomotion_walk_publishes_wrapper(self):
        locomotion = find_plugin(self.plugins, "locomotion")
        result = locomotion.dispatch("walk", {"forward": 0.5, "angular": -0.25})
        pub = self.nodes.locomotion_pub
        (msg,) = pub.published
        self.assertEqual(msg.serialization_type, "pb")
        # without the protobuf wheel the payload is the JSON dict — verify round-trip
        payload = json.loads(bytes(msg.data))
        self.assertEqual(payload["forward_velocity"], 0.5)
        self.assertEqual(payload["angular_velocity"], -0.25)
        self.assertEqual(payload["mode"], "MotionControl_LocomotionMode_DEFAULT")
        self.assertEqual(result["state"], "published")

    def test_locomotion_rejects_out_of_range(self):
        locomotion = find_plugin(self.plugins, "locomotion")
        with self.assertRaises(ValueError):
            locomotion.dispatch("walk", {"forward": 1.5})

    def test_waist_command_publishes_wrapper(self):
        waist = find_plugin(self.plugins, "waist_command")
        waist.dispatch("send", {"pitch": 0.2, "height": -0.1})
        (msg,) = self.nodes.waist_pub.published
        payload = json.loads(bytes(msg.data))
        self.assertEqual(payload["waist_pitch"], 0.2)
        self.assertEqual(payload["waist_height"], -0.1)

    def test_waist_command_rejects_out_of_range(self):
        waist = find_plugin(self.plugins, "waist_command")
        with self.assertRaises(ValueError):
            waist.dispatch("send", {"yaw": 2.0})

    def test_face_play_and_cancel(self):
        face = find_plugin(self.plugins, "face_play")
        face.dispatch("play", {"e_path": "/agibot/data/resources/default/emoticon/smile"})
        (play_msg,) = self.nodes.face_play_pub.published
        play_payload = json.loads(bytes(play_msg.data))
        self.assertEqual(play_payload["e_path"], "/agibot/data/resources/default/emoticon/smile")
        self.assertFalse(play_payload["is_stop"])
        face.dispatch("cancel", {})
        cancel_msg = self.nodes.face_play_pub.published[-1]
        cancel_payload = json.loads(bytes(cancel_msg.data))
        self.assertTrue(cancel_payload["is_stop"])

    # -- arm / neck / hand JointState publishers --

    def test_arm_command_publishes_jointstate_with_zero_velocity(self):
        arm = find_plugin(self.plugins, "arm_command")
        arm.dispatch("send", {"left": [0.0] * 7, "duration_ms": 0})
        pub = self.nodes.arm_command_pub
        (msg,) = pub.published
        self.assertEqual(len(msg.name), 7)
        self.assertEqual(msg.name[0], "left_shoulder_pitch_joint")
        self.assertEqual(msg.velocity, [0.0] * 7)
        self.assertEqual(msg.effort, [0.0] * 7)

    def test_arm_command_rejects_wrong_count(self):
        arm = find_plugin(self.plugins, "arm_command")
        with self.assertRaises(ValueError):
            arm.dispatch("send", {"left": [0.0, 0.1]})

    def test_arm_command_rejects_out_of_limits(self):
        arm = find_plugin(self.plugins, "arm_command")
        with self.assertRaises(ValueError):
            arm.dispatch("send", {"left": [9.9] + [0.0] * 6})

    def test_neck_command_publishes_and_clamps(self):
        neck = find_plugin(self.plugins, "neck_command")
        neck.dispatch("send", {"yaw": 0.5, "duration_ms": 0})
        (msg,) = self.nodes.neck_command_pub.published
        self.assertEqual(msg.name, ["head_yaw_joint"])
        with self.assertRaises(ValueError):
            neck.dispatch("send", {"yaw": 1.2})

    def test_hand_command_publishes_with_hand_type(self):
        hand = find_plugin(self.plugins, "hand_command")
        hand.dispatch("send", {"right": [100, 200], "hand_type": "O10Hand"})
        (msg,) = self.nodes.hand_command_pub.published
        self.assertEqual(msg.name, ["right_hand_joint_0", "right_hand_joint_1"])
        self.assertEqual(msg.header.frame_id, "O10Hand")
        with self.assertRaises(ValueError):
            hand.dispatch("send", {"left": [3000]})

    def test_hand_command_rejects_unknown_type(self):
        hand = find_plugin(self.plugins, "hand_command")
        with self.assertRaises(ValueError):
            hand.dispatch("send", {"left": [0], "hand_type": "RobotHand"})

    # -- HDU RPCs: tts / volume / mic / interaction / resources --

    def test_tts_speak_posts_to_hdu_59301(self):
        tts = find_plugin(self.plugins, "tts")
        self.transport.responses["TTSService"] = {"header": {"code": "0"}, "trace_id": "t-1"}
        result = tts.dispatch("speak", {"text": "你好"})
        (url, body), = self.transport.calls_to("TTSService", "PlayTTS")
        self.assertIn("10.42.10.10:59301", url)
        self.assertEqual(body["text"], "你好")
        self.assertEqual(body["priority_level"], "INTERACTION_L6")
        self.assertTrue(body["is_interrupted"])
        self.assertEqual(result["trace_id"], "t-1")

    def test_tts_rejects_oversized_text(self):
        tts = find_plugin(self.plugins, "tts")
        with self.assertRaises(ValueError):
            tts.dispatch("speak", {"text": "字" * 400})  # 1200 bytes > 1024

    def test_volume_clamped_to_hard_max(self):
        volume = find_plugin(self.plugins, "volume")
        with self.assertRaises(ValueError):
            volume.dispatch("set", {"volume": 85})
        volume.dispatch("set", {"volume": 70})
        (url, body), = self.transport.calls_to("HalAudioService", "SetAudioVolume")
        self.assertIn("10.42.10.10:56666", url)
        self.assertEqual(body["audio_volume"], 70)
        self.assertEqual(body["type"], "SPEAKER_BUILT_IN")

    def test_mic_source_external(self):
        mic = find_plugin(self.plugins, "mic_source")
        mic.dispatch("external", {})
        (url, body), = self.transport.calls_to("HalAudioService", "SetMicSourceRequest")
        self.assertEqual(body["mic_source"], 1)

    def test_interaction_mode_set(self):
        interaction = find_plugin(self.plugins, "interaction")
        interaction.dispatch("mode_only_voice", {})
        (url, body), = self.transport.calls_to("AgentControlService", "SetAgentPropertiesRequest")
        self.assertEqual(body["contents"]["properties"]["2"], "only_voice")

    def test_resource_list_maps_type_names(self):
        resources = find_plugin(self.plugins, "resource_list")
        self.transport.responses["ResourceService"] = {"data": {"resources": ["握手"]}}
        result = resources.dispatch("motion", {})
        (url, body), = self.transport.calls_to("ResourceService", "GetResourceList")
        self.assertIn("10.42.10.10:51049", url)
        self.assertEqual(body["resource_type"], "RESOURCE_TYPE_MOTION")
        self.assertEqual(result["resources"], ["握手"])

    # -- MDU motion_play / arm_compliance / alerts --

    def test_motion_play_play_posts_path(self):
        motion = find_plugin(self.plugins, "motion_play")
        motion.dispatch("play", {"motion_id": "/agibot/data/resources/default/motion/woshou.mcap",
                                 "duration_ms": 5000})
        (url, body), = self.transport.calls_to("MotionCommandService", "SendMotionCommand")
        self.assertIn("10.42.10.12:56444", url)
        self.assertEqual(body["motion_id"], "/agibot/data/resources/default/motion/woshou.mcap")
        self.assertEqual(body["duration_ms"], 5000)
        self.assertTrue(body["cmd_end"])
        self.assertFalse(body["cmd_pause"])

    def test_motion_play_stop_play_uses_cmd_end(self):
        motion = find_plugin(self.plugins, "motion_play")
        motion.dispatch("stop_play", {})
        (_, body), = self.transport.calls_to("MotionCommandService", "SendMotionCommand")
        self.assertTrue(body["cmd_end"])
        self.assertEqual(body["motion_id"], "")

    def test_arm_compliance_dispatch(self):
        compliance = find_plugin(self.plugins, "arm_compliance")
        compliance.dispatch("enable", {})
        (url, _), = self.transport.calls_to("MotionControlMotionService", "EnableArmCompliance")
        self.assertIn("10.42.10.12:56322", url)

    def test_alerts_cooldown_limits_rpc_rate(self):
        alerts = find_plugin(self.plugins, "alerts")
        self.transport.responses["HDSService"] = {"data": {"alerts": [{"code": 1}]}}
        first = alerts.dispatch("query", {})
        second = alerts.dispatch("query", {})  # within the 5 s cooldown → cached
        self.assertEqual(self.transport.calls_to("HDSService", "GetAlertList").__len__(), 1)
        self.assertEqual(first["alerts"], second["alerts"])

    # -- ADU mapping / navigation / relocalization / auto_charging --

    def test_mapping_start_and_save(self):
        mapping = find_plugin(self.plugins, "mapping")
        mapping.dispatch("start_mapping", {})
        (url, body), = self.transport.calls_to("MappingService", "StartMapping")
        self.assertIn("10.42.10.11:50807", url)
        self.assertEqual(body["command"], "MappingCommand_START_MAPPING")
        mapping.dispatch("stop_save", {"map_name": "lobby"})
        (_, save_body), = self.transport.calls_to("MappingService", "StopMapping")
        self.assertEqual(save_body["command"], "MappingCommand_SAVING_MAP")
        self.assertEqual(save_body["map_name"], "lobby")

    def test_navigation_navi_to_pose_payload(self):
        navigation = find_plugin(self.plugins, "navigation")
        self.transport.responses["PncService"] = {"task_id": 7}
        navigation.dispatch("navi_to_pose", {"map_id": 3, "x": 1.5, "y": -2.0, "angle": 0.3})
        (url, body), = self.transport.calls_to("PncService", "PlanningNaviToPose2D")
        self.assertIn("10.42.10.11:53176", url)
        self.assertEqual(body["map_id"], 3)
        self.assertEqual(body["pose"]["position"], {"x": 1.5, "y": -2.0})
        self.assertEqual(body["pose"]["angle"], 0.3)
        self.assertEqual(navigation.last_task_id, 7)

    def test_navigation_control_reuses_last_task_id(self):
        navigation = find_plugin(self.plugins, "navigation")
        self.transport.responses["PncService"] = {"task_id": 9}
        navigation.dispatch("move_forward", {"map_id": 1, "distance": 0.5})
        navigation.dispatch("pause", {})
        (_, body), = self.transport.calls_to("PncService", "ActionPause")
        self.assertEqual(body["task_id"], 9)

    def test_relocalization_start_normal(self):
        relocalization = find_plugin(self.plugins, "relocalization")
        relocalization.dispatch("start_normal", {"map_dir": "/agibot/data/map/lobby"})
        (url, body), = self.transport.calls_to("SLAMRelocalizationService", "SLAMStartNormalRelocalization")
        self.assertIn("10.42.10.11:50583", url)
        self.assertEqual(body["related_map_dir"], "/agibot/data/map/lobby")

    def test_auto_charging_start(self):
        charging = find_plugin(self.plugins, "auto_charging")
        charging.dispatch("charge_start", {})
        (url, body), = self.transport.calls_to("SkillPilotService", "AutoCharging")
        self.assertEqual(body["command"], "AutoChargingCommand_START")
        self.assertEqual(body["trigger"], "AutoChargingTrigger_AGENT")

    def test_skill_play_start_returns_session(self):
        skill = find_plugin(self.plugins, "skill_play")
        self.transport.responses["SkillPilotService"] = {"data": {"session_id": "s-1"}}
        skill.dispatch("play", {"path": "/agibot/data/resources/default/skill/dance"})
        skill.dispatch("pause", {"session_id": "s-1"})
        pause_calls = self.transport.calls_to("SkillPilotService", "SkillPackage")
        self.assertEqual(pause_calls[-1][1]["command"], "Pause")
        self.assertEqual(pause_calls[-1][1]["session_id"], "s-1")

    def test_map_get_pixel_conversion(self):
        map_get = find_plugin(self.plugins, "map_get")
        self.transport.responses["MappingService"] = {
            "data": {"resolution": 0.05, "origin": {"x": 100, "y": 200}}}
        result = map_get.dispatch("query", {"map_id": 1, "x": 2.0, "y": 4.0})
        # pixel_x = 100 + 2.0*0.05 = 100.1 -> 100 ; pixel_y = 200 - 4.0*0.05 = 199.8 -> 199
        self.assertEqual(result["pixel"]["x"], 100)
        self.assertEqual(result["pixel"]["y"], 199)

    # -- model resource --

    def test_model_returns_urdf(self):
        model = find_plugin(self.plugins, "model")
        result = model.dispatch("model", {})
        self.assertIn("<robot", result["urdf"])
        self.assertIn("a3_ultra", result["urdf"])


class MirrorStreamTests(unittest.TestCase):
    """Feed messages into the robot-side subscriptions and verify the core-domain
    JSON mirror fires (the pattern the dashboard consumes)."""

    def setUp(self):
        self.config = json.loads(json.dumps(BASE_CONFIG))
        self.config["plugins"] = FULL_PLUGINS
        self.plugins, _ = build_bundle_plugins(self.config)
        self.nodes = next(iter(self.plugins.values())).nodes

    def _callback(self, topic):
        for robot_topic, callback in self.nodes.robot.subscriptions:
            if robot_topic == topic:
                return callback
        raise KeyError(topic)

    def test_arm_state_mirror_publishes_json(self):
        msg = FakeMsg()
        msg.name = ["left_shoulder_pitch_joint"]
        msg.position = [0.1]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self._callback("/motion/control/arm_joint_state")(msg)
        core_pub = self.nodes.core.publishers["/test_ns/agibot_a3/arm_state"]
        (out,) = core_pub.published
        payload = json.loads(out.data)
        self.assertEqual(payload["name"], ["left_shoulder_pitch_joint"])
        self.assertEqual(self.nodes.snapshot("arm_state")["position"], [0.1])

    def test_imu_snapshots_merge(self):
        for topic, key in (("/ros2/body_drive/pelvis_imu/data", "imu_pelvis"),
                           ("/ros2/body_drive/torso_imu/data", "imu_torso")):
            msg = FakeMsg()
            msg.linear_acceleration.x = 0.01
            self._callback(topic)(msg)
        imu = find_plugin(self.plugins, "imu")
        result = imu.dispatch("query", {})
        self.assertEqual(result["pelvis"]["linear_acceleration"]["x"], 0.01)
        self.assertEqual(result["torso"]["linear_acceleration"]["x"], 0.01)


if __name__ == "__main__":
    unittest.main()
