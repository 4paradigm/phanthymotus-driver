"""Pure-Python unit tests for agibot/A3/device.py — no ROS installation required.

Stubs rclpy / sensor_msgs / std_msgs / ros2_plugin_proto with minimal fakes
before importing device.py, so build_plugins() can run and produce a real tool
inventory to assert against (same pattern as agibot/AimDK_X2/tests/test_device.py,
per the "no test suite for phanthymotus-driver, hand-built verification" note in
CLAUDE.md). RPC-backed plugins are exercised through an in-memory transport that
records every HTTP JSON RPC call, so payload shapes are asserted too.
"""

from __future__ import annotations

import array
import json
import math
import struct
import sys
import tempfile
import threading
import time
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
    module("sensor_msgs.msg", Image=FakeMsg, CompressedImage=FakeMsg, Imu=FakeMsg,
           JointState=FakeMsg, PointCloud2=FakeMsg)
    module("std_msgs")
    # UInt8MultiArray carries the spatial_map sensor/mapping binary payload.
    module("std_msgs.msg", String=FakeMsg, UInt8MultiArray=FakeMsg)

    # ros2_plugin_proto/msg/RosMsgWrapper — device.py publishes command wrappers
    # through it; a plain FakeMsg works (serialization_type / .data attrs).
    module("ros2_plugin_proto")
    module("ros2_plugin_proto.msg", RosMsgWrapper=FakeMsg)

    # The a3_aimdk protobuf wheel is NOT stubbed: on a dev machine it is absent, which
    # is exactly the code path unit tests exercise (JSON fallback / stream withholding).


_install_ros_stubs()

import yaml  # noqa: E402

import device  # noqa: E402
import main  # noqa: E402
from common.vendor_runtime import DriverBundle  # noqa: E402


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

BASE_CONFIG = {
    "rpc": {"hdu": "10.42.10.10", "adu": "10.42.10.11", "mdu": "10.42.10.12", "timeout": 5.0},
    "ros": {"robot_domain_id": 232, "core_domain_id": 42},
    "plugins": {},
}

# config.yaml's full plugin set (matching the shipped config), advanced modules ON so
# the gated tools are covered too. Names aligned with tianyi2.0/q5_bundle/g1:
# lidar_cloud/battery/estop sensors, loco/arm_control/hand_control/head_control/
# waist_control actuators; wakeup/arm_compliance/resource_list cards dissolved.
FULL_PLUGINS = {
    "joints": {"enabled": True}, "imu": {"enabled": True},
    "joint_state": {"enabled": True},
    "camera": {"enabled": True,
               "streams": ["head_left_fisheye", "head_right_fisheye", "head_rear_fisheye",
                           "head_stereo_left_fisheye", "head_stereo_right_fisheye", "armpit_right_fisheye",
                           "chest_front_d457_rgb", "chest_front_d457_depth", "waist_front_d415_rgb",
                           "waist_front_d415_depth", "wrist_left_d405_rgb", "wrist_right_d405_rgb"]},
    "lidar_cloud": {"enabled": True}, "battery": {"enabled": True},
    "estop": {"enabled": True},
    "alerts": {"enabled": True, "poll_interval": 5.0}, "mc_mode": {"enabled": True},
    "loco": {"enabled": True}, "arm_control": {"enabled": True},
    "hand_control": {"enabled": True}, "head_control": {"enabled": True},
    "waist_control": {"enabled": True},
    "motion_play": {"enabled": True}, "tts": {"enabled": True},
    "audio": {"enabled": True, "max_volume": 70},
    "mic": {"enabled": True}, "ext_mic": {"enabled": True},
    "speaker": {"enabled": True},
    "interaction": {"enabled": True},
    "resources": {"enabled": True}, "face_play": {"enabled": True},
    "skill_play": {"enabled": True},
    "controlled_spatial": {"enabled": True}, "spatial_map": {"enabled": True},
    "auto_charging": {"enabled": True},
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
        self.handler = None  # optional test hook: handler(url, service) -> response

    def __call__(self, url, payload, timeout):
        self.calls.append((url, payload))
        service = url.split("/rpc/aimdk.protocol.", 1)[-1]
        if self.handler is not None:
            return self.handler(url, service)
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
        "streams", ["head_left_fisheye", "chest_front_d457_rgb",
                    "chest_front_d457_depth"])
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


def run_concurrently(fn_a, fn_b):
    """Run two dispatch calls in lockstep (barrier-released) on separate threads.

    ThreadingHTTPServer serves MCP calls concurrently, so two dispatches of the
    same long-running action can interleave arbitrarily. This releases both at
    the same instant and returns both results IN INPUT ORDER (fn_a, fn_b) —
    either may be the one that wins the lock. Re-raises the first dispatch
    error so a rejected RPC fails the test loudly.
    """
    barrier = threading.Barrier(2)
    results = [None, None]
    errors = []

    def _run(index, fn):
        barrier.wait(timeout=5)
        try:
            results[index] = fn()
        except Exception as exc:  # noqa: BLE001 — re-raised below
            errors.append(exc)

    threads = [threading.Thread(target=_run, args=(i, fn))
               for i, fn in enumerate((fn_a, fn_b))]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=5)
    if errors:
        raise errors[0]
    return results


# ---------------------------------------------------------------------------
# Inventory / schema tests
# ---------------------------------------------------------------------------

class ToolInventoryTests(unittest.TestCase):
    def test_driver_bundle_accepts_named_plugin_mapping(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = {"joints": {"enabled": True}}
        plugins, _ = build_bundle_plugins(config)

        bundle = DriverBundle(plugins)
        self.assertEqual(len(bundle.plugins), 1)
        self.assertIs(bundle.plugins[0], plugins["joints"])
        self.assertTrue(bundle.get_all_tools())

    def test_tool_names_and_types_match_driver_yaml(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        by_name = {d["name"]: d["type"] for d in tool_definitions(plugins)}
        expected = load_driver_yaml_cards()
        # Legacy test configuration enables the multiplexed compatibility card;
        # production config uses the three fixed camera cards instead.
        expected.pop("camera", None)
        if "camera" in by_name:
            by_name.pop("camera")
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
        config["plugins"] = dict(FULL_PLUGINS, controlled_spatial={"enabled": False},
                                 spatial_map={"enabled": False},
                                 auto_charging={"enabled": False})
        plugins, _ = build_bundle_plugins(config)
        names = {d["name"] for d in tool_definitions(plugins)}
        for gated in ("controlled_spatial", "spatial_map", "auto_charging"):
            self.assertNotIn(gated, names)

    def test_actuator_tools_are_typed_actuator(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        by_name = {d["name"]: d["type"] for d in tool_definitions(plugins)}
        expected_actuators = {
            "mc_mode", "loco", "arm_control", "hand_control", "head_control",
            "waist_control", "motion_play", "tts", "audio", "interaction",
            "face_play", "skill_play", "controlled_spatial", "auto_charging",
        }
        for name in expected_actuators:
            self.assertEqual(by_name[name], "actuator", f"'{name}' must be an actuator tool")

    def test_sensor_and_resource_tools_carry_expected_types(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        by_name = {d["name"]: d["type"] for d in tool_definitions(plugins)}
        self.assertEqual(by_name["model"], "resource")
        for name in ("joints", "imu", "camera", "lidar_cloud", "battery", "estop",
                     "alerts", "spatial_map"):
            self.assertEqual(by_name[name], "sensor", f"'{name}' must be a sensor tool")

    def test_pb_streams_withheld_without_wheel(self):
        # aimdk (the a3_aimdk wheel) is not installed in the test environment, so the
        # pb-decoded stream mirrors must be absent from nodes.streams; the stream cards
        # fall back to the no-stream tool description and skill_play drops its stream.
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        nodes = next(iter(plugins.values())).nodes
        for key in ("battery", "estop", "skill_status"):
            self.assertNotIn(key, nodes.streams)
        for name in ("battery", "estop"):
            plugin = find_plugin(plugins, name)
            self.assertFalse(plugin.has_stream, f"{name} must not claim a stream without the wheel")
        skill_play = find_plugin(plugins, "skill_play")
        self.assertFalse(skill_play.has_stream, "skill_play must not claim a stream without the wheel")

    def test_wrapper_command_cards_visible_without_ros2_plugin_proto(self):
        # Degraded startup path: the canvas still gets the complete card inventory;
        # dispatch fails clearly later when a wrapper command is actually requested.
        saved = {name: sys.modules.pop(name)
                 for name in ("ros2_plugin_proto", "ros2_plugin_proto.msg") if name in sys.modules}
        try:
            config = json.loads(json.dumps(BASE_CONFIG))
            config["plugins"] = FULL_PLUGINS
            plugins, _ = build_bundle_plugins(config)
            names = {d["name"] for d in tool_definitions(plugins)}
            for hidden in ("loco", "waist_control", "face_play"):
                self.assertNotIn(hidden, names,
                                 f"{hidden} must be withheld without ros2_plugin_proto")
            nodes = next(iter(plugins.values())).nodes
            self.assertFalse(nodes.wrapper_available)
            self.assertTrue(nodes.locomotion_pub is None)
            self.assertTrue(nodes.waist_pub is None)
            self.assertTrue(nodes.face_play_pub is None)
            # non-wrapper cards all still present
            for name in ("mc_mode", "arm_control", "hand_control", "head_control",
                         "motion_play", "tts", "skill_play"):
                self.assertIn(name, names)
        finally:
            sys.modules.update(saved)

    def test_camera_streams_follow_config(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = dict(FULL_PLUGINS, camera={
            "enabled": True, "streams": ["head_left_fisheye", "chest_front_d457_depth"]})
        plugins, _ = build_bundle_plugins(config)
        camera = find_plugin(plugins, "camera")
        self.assertEqual(list(camera.streams), ["camera_head_left_fisheye",
                                                "camera_chest_front_d457_depth"])

    def test_camera_rgb_stream_reencodes_to_jpeg_when_cv2_available(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        nodes = next(iter(plugins.values())).nodes

        try:
            import numpy as np  # noqa: F401
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("numpy/cv2 not installed on this host")

        import numpy as np
        subs = {topic: cb for topic, cb in nodes.robot.subscriptions}
        rgb_cb = subs["/hal/head_left_fisheye_camera/rgb"]
        rgb_pub = nodes.core.publishers[f"/{nodes.namespace}/camera_head_left_fisheye"]

        import sensor_msgs.msg as sensor_msgs_real  # noqa: F401 — must be the stub

        h, w = 4, 6
        frame = bytes(bytearray(np.arange(h * w * 3, dtype=np.uint8)))
        msg = FakeMsg()
        msg.height, msg.width, msg.encoding, msg.data = h, w, "rgb8", frame
        rgb_cb(msg)
        self.assertEqual(len(rgb_pub.published), 1, "RGB frame must be re-encoded and published")
        out = rgb_pub.published[0]
        self.assertEqual(out.format, "jpeg")
        self.assertIsInstance(out.data, bytes)
        self.assertGreater(len(out.data), 0)

    def test_camera_bgr8_stream_encodes_to_jpeg_without_conversion(self):
        # Regression for the COLOR_BGR2BGR bug: "COLOR_BGR2BGR" is not an OpenCV
        # constant, so getattr raised and every bgr8 frame was silently dropped.
        # BGR is what cv2.imencode expects natively — no conversion needed.
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        nodes = next(iter(plugins.values())).nodes

        try:
            import numpy as np  # noqa: F401
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("numpy/cv2 not installed on this host")

        subs = {topic: cb for topic, cb in nodes.robot.subscriptions}
        rgb_cb = subs["/hal/head_left_fisheye_camera/rgb"]
        rgb_pub = nodes.core.publishers[f"/{nodes.namespace}/camera_head_left_fisheye"]

        h, w = 4, 6
        frame = bytes(bytearray(np.arange(h * w * 3, dtype=np.uint8)))
        msg = FakeMsg()
        msg.height, msg.width, msg.encoding, msg.data = h, w, "bgr8", frame
        rgb_cb(msg)
        self.assertEqual(len(rgb_pub.published), 1, "bgr8 frame must be encoded, not dropped")
        out = rgb_pub.published[0]
        self.assertEqual(out.format, "jpeg")
        self.assertGreater(len(out.data), 0)

    def test_camera_depth_stream_zlib_compresses_uint16(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        nodes = next(iter(plugins.values())).nodes

        try:
            import numpy as np  # noqa: F401
        except ImportError:
            self.skipTest("numpy not installed on this host")

        import zlib
        subs = {topic: cb for topic, cb in nodes.robot.subscriptions}
        depth_cb = subs["/hal/chest_front_d457_camera/depth"]
        depth_pub = nodes.core.publishers[f"/{nodes.namespace}/camera_chest_front_d457_depth"]

        import numpy as np
        h, w = 4, 6
        depth = np.arange(h * w, dtype=np.uint16)
        msg = FakeMsg()
        msg.height, msg.width, msg.encoding, msg.data = h, w, "16UC1", depth.tobytes()
        depth_cb(msg)
        deadline = time.time() + 1.0
        while not depth_pub.published and time.time() < deadline:
            time.sleep(0.01)
        self.assertEqual(len(depth_pub.published), 1, "depth frame must be compressed and published")
        out = depth_pub.published[0]
        self.assertEqual(out.format, "16UC1; compressedDepth zlib")
        self.assertEqual(zlib.decompress(out.data), depth.tobytes())
        self.assertLess(len(out.data), len(depth.tobytes()))

    def test_camera_encoder_skips_unknown_encoding(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        nodes = next(iter(plugins.values())).nodes
        subs = {topic: cb for topic, cb in nodes.robot.subscriptions}
        rgb_pub = nodes.core.publishers[f"/{nodes.namespace}/camera_head_left_fisheye"]

        try:
            import numpy as np  # noqa: F401
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("numpy/cv2 not installed on this host")

        msg = FakeMsg()
        msg.height, msg.width, msg.encoding, msg.data = 4, 6, "yuyv", bytes(4 * 6 * 3)
        subs["/hal/head_left_fisheye_camera/rgb"](msg)
        self.assertEqual(rgb_pub.published, [], "frames with unsupported encoding must be dropped")

    def test_joints_card_multiplexes_three_groups(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        joints = find_plugin(plugins, "joints")
        for group, robot_topic in (("arm", "/motion/control/arm_joint_state"),
                                   ("hand", "/motion/control/hand_joint_state"),
                                   ("neck", "/motion/control/neck_joint_state")):
            result = joints.dispatch("query", {"group": group})
            self.assertEqual(result["robot_topic"], robot_topic)
        with self.assertRaises(ValueError):
            joints.dispatch("query", {"group": "tail"})

    def test_joints_info_keeps_aggregated_skeleton_topic(self):
        # info().topic_out is authoritative after placement and must remain the
        # aggregated skeleton stream; raw group streams stay metadata only.
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        joints = find_plugin(plugins, "joints")
        for group, key in (("arm", "arm_state"), ("hand", "hand_state"), ("neck", "neck_state")):
            result = joints.dispatch("info", {"group": group})
            self.assertEqual(result["topic_out"],
                             [{"topic": "/test_ns/state/joints", "format": "sensor/skeleton"}])
        # default (no group arg) → arm
        result = joints.dispatch("info", {})
        self.assertEqual(result["topic_out"],
                         [{"topic": "/test_ns/state/joints", "format": "sensor/skeleton"}])

    def test_camera_dispatch_accepts_every_advertised_stream(self):
        # Regression for the double-prefix bug: the schema enum advertises unprefixed
        # names (config.yaml plugins.camera.streams naming) while internal stream
        # keys carry the camera_ prefix — every advertised value plus the default
        # must resolve, or the card was unusable as shipped.
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        camera = find_plugin(plugins, "camera")
        definition = camera.get_tool()
        enum = definition["inputSchema"]["properties"]["stream"]["enum"]
        self.assertEqual(enum, FULL_PLUGINS["camera"]["streams"])
        for name in enum:
            result = camera.dispatch("query", {"stream": name})
            self.assertEqual(result["robot_topic"], camera.streams[f"camera_{name}"]["robot_topic"])
        # default (no stream arg) resolves to the first configured stream
        default = camera.dispatch("query", {})
        first = camera.streams["camera_head_left_fisheye"]
        self.assertEqual(default["robot_topic"], first["robot_topic"])
        with self.assertRaises(ValueError):
            camera.dispatch("query", {"stream": "nope"})

    def test_camera_info_returns_selected_topic_out(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        camera = find_plugin(plugins, "camera")
        for name in ("chest_front_d457_depth", "head_left_fisheye"):
            result = camera.dispatch("info", {"stream": name})
            stream = camera.streams[f"camera_{name}"]
            self.assertEqual(result["topic_out"],
                             [{"topic": stream["topic"], "format": stream["format"]}])
        result = camera.dispatch("info", {})
        stream = camera.streams["camera_head_left_fisheye"]
        self.assertEqual(result["topic_out"][0]["topic"], stream["topic"])


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

        # spatial_map's card lifecycle deliberately drives its polling thread
        # (tianyi pattern) — the thread is stopped now, so verify it can be
        # restarted without any RPC leaking out (first poll happens after
        # publish_interval, not synchronously).
        spatial_map = find_plugin(plugins, "spatial_map")
        result = spatial_map.dispatch("start", {"_tool_name": "spatial_map"})
        self.assertEqual(result["state"], "running")
        self.assertEqual(transport.calls, [], "card start must not issue any RPC")
        spatial_map.stop()


# ---------------------------------------------------------------------------
# RPC payload / publisher dispatch tests
# ---------------------------------------------------------------------------

class RpcDispatchTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(json.dumps(BASE_CONFIG))
        self.config["plugins"] = FULL_PLUGINS
        self.plugins, self.transport = build_bundle_plugins(self.config)
        self.nodes = next(iter(self.plugins.values())).nodes

    # -- mc_mode (MDU :56322, absorbs former mc_state queries + fixed FSM) --

    def test_mc_mode_set_action_hits_mdu_56322(self):
        mc_mode = find_plugin(self.plugins, "mc_mode")
        mc_mode.dispatch("get_up", {})
        (url, body), = self.transport.calls_to("MotionControlActionService", "SetAction")
        self.assertIn("10.42.10.12:56322", url)
        self.assertEqual(body["command"]["action"], "MotionControlAction_GET_UP")

    def test_mc_mode_available_lists_actions(self):
        response = {"commands": ["MotionControlAction_GET_UP", "MotionControlAction_DAMPING"]}
        self.transport.responses["MotionControlActionService"] = response
        mc_mode = find_plugin(self.plugins, "mc_mode")
        result = mc_mode.dispatch("available", {})
        self.assertEqual(result["commands"], response["commands"])

    def test_mc_mode_unknown_action_raises(self):
        mc_mode = find_plugin(self.plugins, "mc_mode")
        with self.assertRaises(ValueError):
            mc_mode.dispatch("moonwalk", {})

    def test_mc_mode_fsm_rejects_from_resting_state(self):
        # GetAction → DAMPING: resting state only allows get_up; lie_down must be
        # rejected with a suggestion and no SetAction RPC may fire.
        self.transport.responses["MotionControlActionService/GetAction"] = {
            "action": "MOTIONCONTROLACTION_DAMPING"}
        mc_mode = find_plugin(self.plugins, "mc_mode")
        result = mc_mode.dispatch("lie_down", {})
        self.assertEqual(result["state"], "rejected")
        self.assertEqual(result["current"], "DAMPING")
        self.assertEqual(result["requested"], "lie_down")
        self.assertIn("get_up", result["suggestion"])
        self.assertEqual(result["allowed"], ["get_up"])
        self.assertEqual(self.transport.calls_to("MotionControlActionService", "SetAction"), [])
        # FSM rejection happens before the runtime cross-check
        self.assertEqual(self.transport.calls_to("MotionControlActionService", "GetAvailableActions"), [])

    def test_mc_mode_fsm_allows_from_motion_state(self):
        self.transport.responses["MotionControlActionService/GetAction"] = {
            "action": "MOTIONCONTROLACTION_MOTION"}
        mc_mode = find_plugin(self.plugins, "mc_mode")
        result = mc_mode.dispatch("lie_down", {})
        self.assertEqual(result["requested"], "LIE_DOWN")
        (url, body), = self.transport.calls_to("MotionControlActionService", "SetAction")
        self.assertEqual(body["command"]["action"], "MotionControlAction_LIE_DOWN")

    def test_mc_mode_runtime_cross_check_rejects(self):
        # FSM allows it (MOTION), but the live GetAvailableActions list doesn't.
        self.transport.responses["MotionControlActionService/GetAction"] = {
            "data": {"command": {"action": "MOTIONCONTROLACTION_MOTION"}}}
        self.transport.responses["MotionControlActionService/GetAvailableActions"] = {
            "commands": ["MotionControlAction_GET_UP"]}
        mc_mode = find_plugin(self.plugins, "mc_mode")
        result = mc_mode.dispatch("passive", {})
        self.assertEqual(result["state"], "rejected")
        self.assertIn("运行时可用动作列表", result["suggestion"])
        self.assertEqual(result["available"], ["MotionControlAction_GET_UP"])
        self.assertEqual(self.transport.calls_to("MotionControlActionService", "SetAction"), [])

    def test_mc_mode_unparseable_state_is_permissive(self):
        # GetAction returning nothing parseable ('') must not block the transition;
        # the runtime cross-check (empty commands → skipped) lets SetAction through.
        self.transport.responses["MotionControlActionService/GetAction"] = {"header": {"code": "0"}}
        mc_mode = find_plugin(self.plugins, "mc_mode")
        result = mc_mode.dispatch("damping", {})
        self.assertEqual(result["requested"], "DAMPING")
        self.assertEqual(len(self.transport.calls_to("MotionControlActionService", "SetAction")), 1)

    # -- MOTION-state gate on loco walk / arm send (dev guide §7.3 prerequisite) --

    def test_loco_walk_rejected_outside_motion_state(self):
        # DAMPING is confidently non-MOTION → walk must be rejected with a get_up
        # hint and no velocity frame may be published.
        self.transport.responses["MotionControlActionService/GetAction"] = {
            "action": "MOTIONCONTROLACTION_DAMPING"}
        loco = find_plugin(self.plugins, "loco")
        result = loco.dispatch("walk", {"forward": 0.5})
        self.assertEqual(result["state"], "rejected")
        self.assertEqual(result["current"], "DAMPING")
        self.assertIn("get_up", result["suggestion"])
        self.assertEqual(self.nodes.locomotion_pub.published, [])

    def test_loco_walk_unparseable_state_is_permissive(self):
        # State unknown → the gate must not block; walking starts as usual.
        self.transport.responses["MotionControlActionService/GetAction"] = {"header": {"code": "0"}}
        loco = find_plugin(self.plugins, "loco")
        result = loco.dispatch("walk", {"forward": 0.5})
        self.assertEqual(result["state"], "walking")
        loco.dispatch("stop", {})

    def test_arm_send_rejected_outside_motion_state(self):
        self.transport.responses["MotionControlActionService/GetAction"] = {
            "data": {"state": "MOTIONCONTROLACTION_LIE_DOWN"}}
        arm = find_plugin(self.plugins, "arm_control")
        result = arm.dispatch("send", {"left": [0.0] * 7, "duration_ms": 0})
        self.assertEqual(result["state"], "rejected")
        self.assertEqual(result["current"], "LIE_DOWN")
        self.assertIn("get_up", result["suggestion"])
        self.assertEqual(self.nodes.arm_command_pub.published, [])
        # compliance RPCs are state-independent and must NOT be gated
        self.transport.responses["MotionControlActionService/GetAction"] = {
            "action": "MOTIONCONTROLACTION_DAMPING"}
        result = arm.dispatch("compliance_check", {})
        self.assertNotEqual(result.get("state"), "rejected")

    def test_arm_send_unparseable_state_is_permissive(self):
        self.transport.responses["MotionControlActionService/GetAction"] = {"header": {"code": "0"}}
        arm = find_plugin(self.plugins, "arm_control")
        result = arm.dispatch("send", {"left": [0.0] * 7, "duration_ms": 0})
        self.assertEqual(result["state"], "published")

    # -- loco / waist / face (RosMsgWrapper publishers, robot domain) --

    def _loco_payloads(self):
        return [json.loads(bytes(msg.data)) for msg in self.nodes.locomotion_pub.published]

    def test_loco_walk_publishes_wrapper(self):
        loco = find_plugin(self.plugins, "loco")
        result = loco.dispatch("walk", {"forward": 0.5, "angular": -28.65})
        pub = self.nodes.locomotion_pub
        (msg,) = pub.published
        self.assertEqual(msg.serialization_type, "pb")
        # without the protobuf wheel the payload is the JSON dict — verify round-trip
        payload = json.loads(bytes(msg.data))
        self.assertEqual(payload["forward_velocity"], 0.5)
        self.assertEqual(payload["mode"], "MotionControl_LocomotionMode_DEFAULT")
        # angular is declared in deg/s; the wire payload carries the normalized ratio.
        self.assertAlmostEqual(payload["angular_velocity"], math.radians(-28.65))
        self.assertEqual(result["state"], "walking")
        self.assertTrue(result["action_id"].startswith("loco_walk_"))
        loco.dispatch("stop", {})

    def test_loco_rejects_out_of_range(self):
        loco = find_plugin(self.plugins, "loco")
        with self.assertRaises(ValueError):
            loco.dispatch("walk", {"forward": 1.5})
        with self.assertRaises(ValueError):
            loco.dispatch("walk", {"angular": 90.0})  # > 57.3 deg/s limit
        with self.assertRaises(ValueError):
            loco.dispatch("walk", {"duration": 0.0})
        with self.assertRaises(ValueError):
            loco.dispatch("walk", {"duration": 61.0})
        with self.assertRaises(ValueError):
            loco.dispatch("walk", {"forward": 0.0, "lateral": 0.0, "angular": 0.0})

    def test_loco_schema_declares_x_completion(self):
        schema = find_plugin(self.plugins, "loco").get_tool()["inputSchema"]
        self.assertEqual(schema["x-completion"], {"actions": ["walk"], "timeout": 60})
        self.assertIn("stop_move", schema["properties"]["action"]["enum"])

    def test_loco_walk_reports_acp_completion_after_duration(self):
        loco = find_plugin(self.plugins, "loco")
        loco._PUBLISH_INTERVAL_S = 0.02
        loco._STOP_FRAMES = 2
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda *a: captured.append(a)
        try:
            result = loco.dispatch("walk", {"forward": 0.3, "duration": 0.1})
            deadline = time.time() + 5
            while not captured and time.time() < deadline:
                time.sleep(0.02)
            (action_id, status, payload, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "completed")
            self.assertEqual(payload["duration"], 0.1)
            self.assertTrue(payload["stopped_automatically"])
            self.assertEqual(tool_name, "loco")
            # stream kept republishing during the window, then sent zero-velocity frames
            payloads = self._loco_payloads()
            self.assertGreater(len(payloads), 2)
            self.assertTrue(all(p["forward_velocity"] == 0.3 for p in payloads[:-2]))
            self.assertTrue(all(p["forward_velocity"] == 0.0 for p in payloads[-2:]))
        finally:
            device._acp_notify = original_notify

    def test_loco_stop_settles_pending_waiter(self):
        loco = find_plugin(self.plugins, "loco")
        loco._PUBLISH_INTERVAL_S = 0.2
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda *a: captured.append(a)
        try:
            loco.dispatch("walk", {"forward": 0.5, "duration": 30.0})
            stop_result = loco.dispatch("stop", {})
            self.assertEqual(stop_result["state"], "idle")
            self.assertTrue(stop_result["was_walking"])
            time.sleep(0.5)
            # the superseded worker must NOT report completed; only the cancel notify fired
            self.assertEqual([c[1] for c in captured], ["cancelled"])
            self.assertIn("reason", captured[0][2])
            payloads = self._loco_payloads()
            self.assertEqual(payloads[-1]["forward_velocity"], 0.0)
        finally:
            device._acp_notify = original_notify

    def test_loco_new_command_settles_previous(self):
        loco = find_plugin(self.plugins, "loco")
        loco._PUBLISH_INTERVAL_S = 0.2
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda *a: captured.append(a)
        try:
            first = loco.dispatch("walk", {"forward": 0.5, "duration": 30.0})
            second = loco.dispatch("walk", {"forward": -0.5, "duration": 30.0})
            time.sleep(0.5)
            self.assertEqual([c[1] for c in captured], ["cancelled"])
            self.assertEqual(captured[0][0], first["action_id"])
            self.assertNotEqual(first["action_id"], second["action_id"])
            loco.dispatch("stop", {})
        finally:
            device._acp_notify = original_notify

    def test_loco_simultaneous_walks_settle_exactly_one(self):
        # 10th PR review: the settle→arm transition must be ONE lock-held
        # section. Two truly simultaneous walks (barrier-released) must never
        # both observe "no active action": the loser is cancelled immediately,
        # exactly one walker stays armed, and every action gets exactly one
        # terminal post.
        loco = find_plugin(self.plugins, "loco")
        loco._PUBLISH_INTERVAL_S = 0.2
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            first, second = run_concurrently(
                lambda: loco.dispatch("walk", {"forward": 0.5, "duration": 30.0}),
                lambda: loco.dispatch("walk", {"forward": -0.5, "duration": 30.0}))
            ids = {first["action_id"], second["action_id"]}
            self.assertEqual(len(ids), 2)  # distinct actions — no id overwrite
            loser = captured[0][0]
            self.assertIn(loser, ids)
            self.assertEqual(captured[0][1], "cancelled")
            # the survivor is the one still armed, and it is the only walker
            survivor = second["action_id"] if loser == first["action_id"] else first["action_id"]
            self.assertEqual(loco._move_action_id, survivor)
            loco.dispatch("stop", {})
            time.sleep(0.5)
            # each action reported exactly once, and both got a terminal —
            # never a double post for one action, never a silent orphan.
            self.assertEqual(sorted(c[1] for c in captured), ["cancelled", "cancelled"])
            for action_id in ids:
                self.assertEqual(len([c for c in captured if c[0] == action_id]), 1)
        finally:
            device._acp_notify = original_notify

    def test_waist_control_publishes_wrapper(self):
        waist = find_plugin(self.plugins, "waist_control")
        waist.dispatch("send", {"pitch": 0.2, "height": -0.1})
        (msg,) = self.nodes.waist_pub.published
        payload = json.loads(bytes(msg.data))
        self.assertEqual(payload["waist_pitch"], 0.2)
        self.assertEqual(payload["waist_height"], -0.1)

    def test_waist_control_rejects_out_of_range(self):
        waist = find_plugin(self.plugins, "waist_control")
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

    # -- arm / head / hand JointState publishers --

    def test_arm_control_publishes_jointstate_with_zero_velocity(self):
        arm = find_plugin(self.plugins, "arm_control")
        arm.dispatch("send", {"left": [0.0] * 7, "duration_ms": 0})
        pub = self.nodes.arm_command_pub
        (msg,) = pub.published
        self.assertEqual(len(msg.name), 7)
        self.assertEqual(msg.name[0], "left_shoulder_pitch_joint")
        self.assertEqual(msg.velocity, [0.0] * 7)
        self.assertEqual(msg.effort, [0.0] * 7)

    def test_arm_control_rejects_wrong_count(self):
        arm = find_plugin(self.plugins, "arm_control")
        with self.assertRaises(ValueError):
            arm.dispatch("send", {"left": [0.0, 0.1]})

    def test_arm_control_rejects_out_of_limits(self):
        arm = find_plugin(self.plugins, "arm_control")
        with self.assertRaises(ValueError):
            arm.dispatch("send", {"left": [9.9] + [0.0] * 6})

    def test_head_control_publishes_and_clamps(self):
        head = find_plugin(self.plugins, "head_control")
        head.dispatch("send", {"yaw": 0.5, "duration_ms": 0})
        (msg,) = self.nodes.neck_command_pub.published
        self.assertEqual(msg.name, ["head_yaw_joint"])
        with self.assertRaises(ValueError):
            head.dispatch("send", {"yaw": 1.2})

    def test_hand_control_publishes_with_hand_type(self):
        hand = find_plugin(self.plugins, "hand_control")
        hand.dispatch("send", {"right": [100, 200], "hand_type": "O10Hand"})
        (msg,) = self.nodes.hand_command_pub.published
        self.assertEqual(msg.name, ["right_hand_joint_0", "right_hand_joint_1"])
        self.assertEqual(msg.header.frame_id, "O10Hand")
        with self.assertRaises(ValueError):
            hand.dispatch("send", {"left": [3000]})

    def test_hand_control_rejects_unknown_type(self):
        hand = find_plugin(self.plugins, "hand_control")
        with self.assertRaises(ValueError):
            hand.dispatch("send", {"left": [0], "hand_type": "RobotHand"})

    def test_readable_hand_and_head_actions(self):
        hand = find_plugin(self.plugins, "hand_control").get_tool()
        head = find_plugin(self.plugins, "head_control").get_tool()
        hand_actions = hand["inputSchema"]["properties"]["action"]["enum"]
        head_actions = head["inputSchema"]["properties"]["action"]["enum"]
        self.assertTrue({"open", "close", "set_position"}.issubset(hand_actions))
        self.assertNotIn("open_left", hand_actions)
        self.assertEqual(hand["inputSchema"]["properties"]["side"]["enum"],
                         ["left", "right", "both"])
        self.assertTrue({"look_left", "look_right", "look_up", "look_down", "center",
                         "set_position"}.issubset(head_actions))

    def test_hand_open_selects_side_in_one_action(self):
        hand = find_plugin(self.plugins, "hand_control")
        hand.dispatch("open", {"side": "left"})
        (msg,) = self.nodes.hand_command_pub.published
        self.assertEqual(msg.name, [f"left_hand_joint_{index}" for index in range(6)])
        self.assertEqual(msg.position, [0] * 6)

    # -- HDU RPCs: tts (absorbs media_play) / audio (absorbs audio_play+volume) /
    #    interaction (absorbs mic_source) / resources --

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

    def test_tts_schema_declares_x_completion(self):
        schema = find_plugin(self.plugins, "tts").get_tool()["inputSchema"]
        self.assertEqual(schema["x-completion"],
                         {"actions": ["speak", "play_media"], "timeout": 180})

    def test_tts_speak_reports_acp_completion(self):
        # speak is a long-running action: dispatch returns immediately with an
        # action_id and a daemon worker polls GetAudioStatus until state 2
        # (播报完成), then POSTs the ACP completion.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            tts = find_plugin(self.plugins, "tts")
            tts._POLL_INTERVAL_S = 0.02
            self.transport.responses["TTSService/PlayTTS"] = {"header": {"code": "0"},
                                                              "trace_id": "t-7"}
            self.transport.responses["TTSService/GetAudioStatus"] = {"state": 2}
            result = tts.dispatch("speak", {"text": "你好"})
            self.assertEqual(result["state"], "playing")
            self.assertTrue(result["action_id"].startswith("tts_speak_"))
            self.assertEqual(result["trace_id"], "t-7")
            deadline = time.time() + 5
            while not captured and time.time() < deadline:
                time.sleep(0.02)
            (action_id, status, payload, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "completed")
            self.assertEqual(payload["trace_id"], "t-7")
            self.assertEqual(payload["final_state"], 2)
            self.assertEqual(tool_name, "tts")
        finally:
            device._acp_notify = original_notify

    def test_tts_play_media_error_state_reports_error(self):
        # state 3 (异常) must surface as an ACP error, not a completion.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            tts = find_plugin(self.plugins, "tts")
            tts._POLL_INTERVAL_S = 0.02
            self.transport.responses["TTSService/PlayMediaFile"] = {"trace_id": "t-8"}
            self.transport.responses["TTSService/GetAudioStatus"] = {"data": {"state": 3}}
            result = tts.dispatch("play_media", {"file_name": "intro.mp4"})
            self.assertEqual(result["state"], "playing")
            deadline = time.time() + 5
            while not captured and time.time() < deadline:
                time.sleep(0.02)
            (action_id, status, payload, tool_name), = captured
            self.assertEqual(status, "error")
            self.assertEqual(payload["final_state"], 3)
            self.assertEqual(tool_name, "tts")
        finally:
            device._acp_notify = original_notify

    def test_tts_stop_settles_pending_waiter(self):
        # stop_trace_id must invalidate the polling worker AND post an immediate
        # cancelled — a silent id clear leaves the ACP barrier pending to timeout.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            tts = find_plugin(self.plugins, "tts")
            tts._POLL_INTERVAL_S = 0.2  # long enough to still be polling
            self.transport.responses["TTSService/GetAudioStatus"] = {"state": 1}
            result = tts.dispatch("speak", {"text": "你好"})
            tts.dispatch("stop_trace_id", {"trace_id": ""})
            time.sleep(0.5)
            (action_id, status, _, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "cancelled")
            self.assertEqual(tool_name, "tts")
        finally:
            device._acp_notify = original_notify

    def test_tts_stop_unrelated_trace_does_not_cancel_current_waiter(self):
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda *args: captured.append(args)
        try:
            tts = find_plugin(self.plugins, "tts")
            tts._POLL_INTERVAL_S = 0.2
            self.transport.responses["TTSService/PlayTTS"] = {"trace_id": "t-current"}
            self.transport.responses["TTSService/GetAudioStatus"] = {"state": 1}
            result = tts.dispatch("speak", {"text": "当前播报"})
            tts.dispatch("stop_trace_id", {"trace_id": "t-old"})
            self.assertEqual(tts._play_action_id, result["action_id"])
            self.assertEqual(captured, [])
            tts.dispatch("stop", {})
        finally:
            device._acp_notify = original_notify

    def test_tts_framework_stop_interrupts_current_playback(self):
        # 6th PR review: the framework "stop" action must physically silence the
        # audio (StopTTSTraceId with the active trace_id) and settle the ACP
        # waiter — not just acknowledge "idle" while the voice keeps playing.
        # 7th PR review: settling also means an immediate cancelled post.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            tts = find_plugin(self.plugins, "tts")
            tts._POLL_INTERVAL_S = 0.2  # long enough to still be polling
            self.transport.responses["TTSService/PlayTTS"] = {"trace_id": "t-active"}
            self.transport.responses["TTSService/GetAudioStatus"] = {"state": 1}
            result = tts.dispatch("speak", {"text": "长文本播报"})
            tts.dispatch("stop", {})
            (url, body), = self.transport.calls_to("TTSService", "StopTTSTraceId")
            self.assertEqual(body["trace_id"], "t-active")
            time.sleep(0.5)
            (action_id, status, _, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "cancelled")
            self.assertEqual(tool_name, "tts")
        finally:
            device._acp_notify = original_notify

    def test_tts_not_yet_played_state_keeps_polling(self):
        # GetAudioStatus returning 0 (未播) right after PlayTTS is accepted is a
        # normal race, not a playback error — the worker must keep polling until
        # state flips to playing/finished (the timeout is the backstop).
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            tts = find_plugin(self.plugins, "tts")
            tts._POLL_INTERVAL_S = 0.02
            self.transport.responses["TTSService/PlayTTS"] = {"trace_id": "t-slow"}
            # every poll answers 未播 (state 0) — the worker must not error out
            self.transport.responses["TTSService/GetAudioStatus"] = {"state": 0}
            tts.dispatch("speak", {"text": "刚开始还没播"})
            time.sleep(0.4)
            self.assertEqual(captured, [], "state 0 (未播) must be treated as pending")
            # flip to finished → the same worker reports completion
            self.transport.responses["TTSService/GetAudioStatus"] = {"state": 2}
            deadline = time.time() + 5
            while not captured and time.time() < deadline:
                time.sleep(0.02)
            (action_id, status, _, tool_name), = captured
            self.assertEqual(status, "completed")
            self.assertEqual(tool_name, "tts")
        finally:
            device._acp_notify = original_notify

    def test_tts_second_speak_settles_previous_and_stops_old_trace(self):
        # Concurrent MCP calls make a second speak reachable while the first is
        # still playing: the first action gets an immediate cancelled, the OLD
        # trace_id is stopped (not the new one), and only the second action
        # stays armed.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            tts = find_plugin(self.plugins, "tts")
            tts._POLL_INTERVAL_S = 0.2
            self.transport.responses["TTSService/PlayTTS"] = {"trace_id": "t-old"}
            self.transport.responses["TTSService/GetAudioStatus"] = {"state": 1}
            first = tts.dispatch("speak", {"text": "第一句"})
            # second speak gets a different trace_id from the service
            self.transport.responses["TTSService/PlayTTS"] = {"trace_id": "t-new"}
            second = tts.dispatch("speak", {"text": "第二句"})
            self.assertNotEqual(first["action_id"], second["action_id"])
            # the OLD playback is physically interrupted with the OLD trace_id
            (url, body), = self.transport.calls_to("TTSService", "StopTTSTraceId")
            self.assertEqual(body["trace_id"], "t-old")
            time.sleep(0.5)
            self.assertEqual([(c[0], c[1], c[3]) for c in captured],
                             [(first["action_id"], "cancelled", "tts")])
            self.assertEqual(captured[0][2]["reason"], "replaced_by_new_playback")
            self.assertEqual(tts._play_action_id, second["action_id"])
        finally:
            device._acp_notify = original_notify

    def test_tts_simultaneous_speaks_settle_exactly_one(self):
        # 10th PR review: two barrier-released speaks must serialise through
        # the lock-held settle→stop-old-trace→arm transition — distinct ids,
        # the loser cancelled exactly once, and the OLD trace interrupted.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            tts = find_plugin(self.plugins, "tts")
            tts._POLL_INTERVAL_S = 5.0  # keep the survivor polling
            self.transport.responses["TTSService/GetAudioStatus"] = {"state": 1}
            traces = iter(["t-a", "t-b"])
            self.transport.handler = lambda url, service: (
                {"trace_id": next(traces)}
                if service.startswith("TTSService/PlayTTS")
                else {"header": {"code": "0"}})
            first, second = run_concurrently(
                lambda: tts.dispatch("speak", {"text": "第一句"}),
                lambda: tts.dispatch("speak", {"text": "第二句"}))
            self.assertEqual(len({first["action_id"], second["action_id"]}), 2)
            loser = captured[0][0]
            self.assertEqual(captured[0][1], "cancelled")
            self.assertEqual(captured[0][2]["reason"], "replaced_by_new_playback")
            loser_result = second if loser == second["action_id"] else first
            survivor_result = second if loser == first["action_id"] else first
            self.assertEqual(tts._play_action_id, survivor_result["action_id"])
            # the interrupted trace is the loser's, never the survivor's
            (url, body), = self.transport.calls_to("TTSService", "StopTTSTraceId")
            self.assertEqual(body["trace_id"], loser_result["trace_id"])
            tts.dispatch("stop", {})
            time.sleep(0.3)
            for action_id in (first["action_id"], second["action_id"]):
                self.assertEqual(len([c for c in captured if c[0] == action_id]), 1)
        finally:
            device._acp_notify = original_notify


    def test_tts_play_media_posts_play_media_file(self):
        tts = find_plugin(self.plugins, "tts")
        tts.dispatch("play_media", {"file_name": "welcome.mp3"})
        (url, body), = self.transport.calls_to("TTSService", "PlayMediaFile")
        self.assertIn("10.42.10.10:59301", url)
        self.assertEqual(body["file_name"], "welcome.mp3")

    def test_audio_play_posts_play_file(self):
        audio = find_plugin(self.plugins, "audio")
        audio.dispatch("play", {"file_name": "ding.wav"})
        (url, body), = self.transport.calls_to("HalAudioService", "PlayFile")
        self.assertIn("10.42.10.10:56666", url)
        self.assertEqual(body["file_name"], "ding.wav")

    def test_audio_volume_clamped_to_hard_max(self):
        audio = find_plugin(self.plugins, "audio")
        with self.assertRaises(ValueError):
            audio.dispatch("set_volume", {"volume": 85})
        audio.dispatch("set_volume", {"volume": 70})
        (url, body), = self.transport.calls_to("HalAudioService", "SetAudioVolume")
        self.assertIn("10.42.10.10:56666", url)
        self.assertEqual(body["audio_volume"], 70)
        self.assertEqual(body["type"], "SPEAKER_BUILT_IN")

    def test_interaction_mic_external(self):
        interaction = find_plugin(self.plugins, "interaction")
        interaction.dispatch("mic_external", {})
        (url, body), = self.transport.calls_to("HalAudioService", "SetMicSourceRequest")
        self.assertEqual(body["mic_source"], 1)

    def test_interaction_mode_set(self):
        interaction = find_plugin(self.plugins, "interaction")
        interaction.dispatch("mode_only_voice", {})
        (url, body), = self.transport.calls_to("AgentControlService", "SetAgentPropertiesRequest")
        self.assertEqual(body["contents"]["properties"]["2"], "only_voice")

    def test_resource_list_actions_per_play_card(self):
        # resource_list card dissolved: each play card exposes its own GetResourceList
        # query as a `list` action (ResourceService @ HDU :51049).
        self.transport.responses["ResourceService"] = {"data": {"resources": ["握手"]}}
        cases = [
            (find_plugin(self.plugins, "motion_play"), "list", "RESOURCE_TYPE_MOTION"),
            (find_plugin(self.plugins, "face_play"), "list", "RESOURCE_TYPE_EMOTICON"),
            (find_plugin(self.plugins, "audio"), "list", "RESOURCE_TYPE_AUDIO"),
            (find_plugin(self.plugins, "skill_play"), "list", "RESOURCE_TYPE_SKILL"),
            (find_plugin(self.plugins, "skill_play"), "list_offring_work", "RESOURCE_TYPE_OFFRING_WORK"),
        ]
        for plugin, action, expected_type in cases:
            result = plugin.dispatch(action, {})
            self.assertEqual(result["resources"], ["握手"])
            (url, body), = self.transport.calls_to("ResourceService", "GetResourceList")
            self.assertIn("10.42.10.10:51049", url)
            self.assertEqual(body["resource_type"], expected_type)
            self.transport.calls.clear()

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
        # 1ms of playback — the completion worker settles immediately instead of
        # outliving the test: a 5 s survivor polls on into later tts tests and
        # its ACP post lands inside their monkeypatched _acp_notify windows.
        motion.dispatch("stop_play", {})

    def test_motion_play_stop_play_uses_cmd_end(self):
        motion = find_plugin(self.plugins, "motion_play")
        motion.dispatch("stop_play", {})
        (_, body), = self.transport.calls_to("MotionCommandService", "SendMotionCommand")
        self.assertTrue(body["cmd_end"])
        self.assertEqual(body["motion_id"], "")

    def test_motion_play_play_reports_acp_completion(self):
        # A3 exposes no motion-status topic/RPC, so play is async: dispatch returns
        # immediately with an action_id and a daemon worker POSTs the ACP completion
        # once duration_ms elapses. Stub _acp_notify and wait for the callback.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            motion = find_plugin(self.plugins, "motion_play")
            result = motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                              "duration_ms": 50})
            self.assertEqual(result["state"], "playing")
            self.assertTrue(result["action_id"].startswith("motion_play_"))
            deadline = time.time() + 5
            while not captured and time.time() < deadline:
                time.sleep(0.02)
            (action_id, status, payload, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "completed")
            self.assertEqual(payload["motion_id"], "/agibot/motions/wave.mcap")
            self.assertEqual(payload["duration_ms"], 50)
            self.assertEqual(tool_name, "motion_play")
        finally:
            device._acp_notify = original_notify

    def test_actuator_schemas_declare_x_resource(self):
        # README_dev x-resource contract: an acting tool must declare the physical
        # channels it drives on EVERY tool — partial declaration is the unsafe trap.
        # mc_mode is the one deliberate exception (g1 switch_mode precedent: a whole-
        # body posture transition must stay exclusive against everything).
        expected = {
            "loco": "base",
            "arm_control": ["arm_l", "arm_r"],
            "hand_control": ["hand_l", "hand_r"],
            "head_control": "head",
            "waist_control": "waist",
            "motion_play": ["base", "arm_l", "arm_r", "hand_l", "hand_r", "waist", "head"],
            "tts": "mouth",
            "audio": "mouth",
            "interaction": "mouth",
            "face_play": "head",
            "skill_play": ["base", "arm_l", "arm_r", "hand_l", "hand_r", "waist", "head"],
            "controlled_spatial": "base",
            "auto_charging": "base",
        }
        for name, channels in expected.items():
            schema = find_plugin(self.plugins, name).get_tool()["inputSchema"]
            self.assertEqual(schema.get("x-resource"), channels, f"{name} x-resource")
        mc_mode_schema = find_plugin(self.plugins, "mc_mode").get_tool()["inputSchema"]
        self.assertNotIn("x-resource", mc_mode_schema)

    def test_motion_play_schema_declares_x_completion(self):
        schema = find_plugin(self.plugins, "motion_play").get_tool()["inputSchema"]
        self.assertEqual(schema["x-completion"], {"actions": ["play"], "timeout": 600})

    def test_motion_play_rejects_out_of_bounds_duration(self):
        # duration_ms bounds must match the 600 s x-completion timeout: a zero
        # or negative value would report completion instantly while the RPC
        # still runs (zero is rejected: with no motion-status feedback the A3
        # cannot observe an open-ended playback); >600000 would outlive the
        # Agent Core waiter.
        motion = find_plugin(self.plugins, "motion_play")
        for bad in (0, -1, device.MOTION_PLAY_MAX_DURATION_MS + 1):
            with self.assertRaises(ValueError):
                motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                         "duration_ms": bad})
        # Nothing was sent to the robot for the rejected calls.
        self.assertEqual(self.transport.calls_to("MotionCommandService", "SendMotionCommand"), [])

    def test_motion_play_second_play_settles_previous(self):
        # 9th PR review: ThreadingHTTPServer serves MCP calls concurrently, so a
        # second play can arrive while the first is still running. The first
        # action must be settled — immediate cancelled post + cmd_end for the
        # robot — never orphaned on a silent id mismatch.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            motion = find_plugin(self.plugins, "motion_play")
            motion._PLAY_WORKER_TICK_S = 0.02
            first = motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                             "duration_ms": 5000})
            second = motion.dispatch("play", {"motion_id": "/agibot/motions/dance.mcap",
                                              "duration_ms": 5000})
            self.assertNotEqual(first["action_id"], second["action_id"])
            time.sleep(0.5)
            # exactly one post so far: the first action cancelled immediately
            self.assertEqual([(c[0], c[1], c[3]) for c in captured],
                             [(first["action_id"], "cancelled", "motion_play")])
            self.assertEqual(captured[0][2]["reason"], "replaced_by_new_play")
            # the replacement ended the first motion physically before arming
            calls = self.transport.calls_to("MotionCommandService", "SendMotionCommand")
            end_calls = [c for c in calls if c[1]["cmd_end"] and not c[1]["motion_id"]]
            self.assertEqual(len(end_calls), 1)  # cmd_end for the first motion only
            self.assertEqual([c[1]["motion_id"] for c in calls],
                             ["/agibot/motions/wave.mcap", "",
                              "/agibot/motions/dance.mcap"])
            self.assertTrue(calls[1][1]["cmd_end"])
            # the second worker is the only one left armed and must not be able
            # to report the first action's completion
            self.assertEqual(motion._play_action_id, second["action_id"])
        finally:
            device._acp_notify = original_notify

    def test_motion_play_pause_suspends_completion_countdown(self):
        # cmd_pause freezes the robot-side player, so wall-clock time no longer
        # maps to playback progress: the completion countdown must freeze too,
        # otherwise Agent Core is told "completed" while the motion is paused,
        # and after resume the robot keeps moving with no waiter armed.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            motion = find_plugin(self.plugins, "motion_play")
            motion._PLAY_WORKER_TICK_S = 0.02
            result = motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                              "duration_ms": 100})
            self.assertEqual(result["state"], "playing")
            motion.dispatch("pause", {})
            # the original 100 ms deadline passes while paused
            time.sleep(0.4)
            self.assertEqual(captured, [])  # countdown frozen, no completed post
            motion.dispatch("resume", {})
            deadline = time.time() + 5
            while not captured and time.time() < deadline:
                time.sleep(0.02)
            (action_id, status, payload, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "completed")
            self.assertEqual(tool_name, "motion_play")
            # pause/resume reached the robot-side player
            pause_calls = self.transport.calls_to("MotionCommandService", "SendMotionCommand")
            self.assertEqual([c[1]["cmd_pause"] for c in pause_calls[1:]], [True, False])
        finally:
            device._acp_notify = original_notify

    def test_motion_play_pause_while_idle_then_play_completes(self):
        # 11th PR review: pause with no active play must not latch the pause flag —
        # a stale _paused with no armed worker would freeze the NEXT play's
        # countdown forever and hang Agent Core's barrier to the 600 s timeout.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            motion = find_plugin(self.plugins, "motion_play")
            motion._PLAY_WORKER_TICK_S = 0.02
            # pause while idle → no-op, no cmd_pause RPC leaves the driver
            self.assertEqual(motion.dispatch("pause", {}), {"state": "idle"})
            self.assertFalse(motion._paused.is_set())
            result = motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                              "duration_ms": 100})
            self.assertEqual(result["state"], "playing")
            deadline = time.time() + 5
            while not captured and time.time() < deadline:
                time.sleep(0.02)
            (action_id, status, payload, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "completed")
            self.assertEqual(tool_name, "motion_play")
        finally:
            device._acp_notify = original_notify

    def test_motion_play_pause_then_stop_clears_flag_for_next_play(self):
        # 11th PR review companion: pause an ACTIVE play, stop it, then a fresh
        # play must complete even though nobody ever sent resume — the settle
        # path clears the flag that belonged to the cancelled action.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            motion = find_plugin(self.plugins, "motion_play")
            motion._PLAY_WORKER_TICK_S = 0.02
            first = motion.dispatch("play", {"motion_id": "/agibot/motions/a.mcap",
                                             "duration_ms": 5000})
            motion.dispatch("pause", {})
            self.assertTrue(motion._paused.is_set())
            motion.dispatch("stop_play", {})
            self.assertEqual([(c[0], c[1]) for c in captured],
                             [(first["action_id"], "cancelled")])
            second = motion.dispatch("play", {"motion_id": "/agibot/motions/b.mcap",
                                              "duration_ms": 100})
            deadline = time.time() + 5
            while len(captured) < 2 and time.time() < deadline:
                time.sleep(0.02)
            self.assertEqual(captured[-1][:2], (second["action_id"], "completed"))
        finally:
            device._acp_notify = original_notify

    def test_motion_play_paused_worker_does_not_busy_spin(self):
        # 12th PR review: Event.wait(timeout) returns IMMEDIATELY while set, so the
        # old paused loop (`self._paused.wait(tick); if paused: continue`) ran a
        # tight no-sleep spin for up to 600 s — pegging a CPU core for the whole
        # pause. While paused the worker must sleep at the tick instead: during a
        # paused window the worker calls time.sleep (the tick) and never calls
        # Event.wait. NB: device.py does `import time`, so patching time.sleep
        # patches the GLOBAL time module — the wrapper delegates to the real
        # sleep so the test's own polling and other workers keep working.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        real_sleep = time.sleep
        sleep_calls, wait_calls = [], []
        motion = find_plugin(self.plugins, "motion_play")
        motion._PLAY_WORKER_TICK_S = 0.05
        paused_event = motion._paused
        real_wait = paused_event.wait
        try:
            def counting_wait(timeout=None):
                if paused_event.is_set():
                    wait_calls.append(timeout)
                return real_wait(timeout)

            def counting_sleep(seconds):
                if paused_event.is_set():
                    sleep_calls.append(seconds)
                return real_sleep(seconds)

            paused_event.wait = counting_wait
            time.sleep = counting_sleep
            motion.dispatch("play", {"motion_id": "/agibot/motions/long.mcap",
                                     "duration_ms": 5000})
            motion.dispatch("pause", {})
            self.assertTrue(paused_event.is_set())
            # hold the pause ~0.4 s: the buggy worker would spin thousands of
            # instant wait() returns, the fixed one sleeps ~8 single ticks.
            real_sleep(0.4)
            self.assertGreaterEqual(len(sleep_calls), 4)
            self.assertEqual(wait_calls, [])  # Event.wait is never the paused wait
            # the worker's own paused sleeps are exactly one tick each; other
            # plugins' poll workers may also be sleeping during this window
            # (e.g. 1.0 s alert polls), so exclude foreign intervals instead of
            # pinning an exact multiset.
            own = [s for s in sleep_calls
                   if abs(s - motion._PLAY_WORKER_TICK_S) < 1e-9]
            self.assertGreaterEqual(own, [motion._PLAY_WORKER_TICK_S] * 4)
            self.assertLess(max(own), motion._PLAY_WORKER_TICK_S + 1e-9)
        finally:
            paused_event.wait = real_wait
            time.sleep = real_sleep
            motion.dispatch("stop_play", {})
            device._acp_notify = original_notify
        self.assertEqual([(c[0], c[1]) for c in captured],
                         [(captured[0][0], "cancelled")])
        self.assertEqual(captured[0][0], captured[0][0])

    def test_motion_play_completion_vs_stop_race_posts_exactly_one_terminal(self):
        # 10th PR review: natural completion and a concurrent stop race for the
        # same action — whoever wins the lock claims the terminal, the loser
        # stays silent. Exactly one post per action_id, status either completed
        # or cancelled but never both, never a double post.
        motion = find_plugin(self.plugins, "motion_play")
        motion._PLAY_WORKER_TICK_S = 0.02
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            result = motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                              "duration_ms": 60})
            # fire stop_play the instant the 60 ms countdown is about to expire
            stopper = threading.Timer(0.05, lambda: motion.dispatch("stop_play", {}))
            stopper.start()
            stopper.join()
            deadline = time.time() + 5
            while not captured and time.time() < deadline:
                time.sleep(0.02)
            time.sleep(0.2)  # let any double-posting bug surface
            posts = [c for c in captured if c[0] == result["action_id"]]
            self.assertEqual(len(posts), 1)
            self.assertIn(posts[0][1], ("completed", "cancelled"))
            self.assertEqual(motion._play_action_id, None)
        finally:
            device._acp_notify = original_notify

    def test_motion_play_simultaneous_plays_never_overlap(self):
        # 10th PR review: two barrier-released plays must serialise through the
        # lock-held settle→arm transition — distinct action ids (no overwrite),
        # the loser cancelled exactly once, the survivor armed, and the robot
        # sees cmd_end for the first motion BEFORE the second starts.
        motion = find_plugin(self.plugins, "motion_play")
        motion._PLAY_WORKER_TICK_S = 0.02
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            first, second = run_concurrently(
                lambda: motion.dispatch("play", {"motion_id": "/agibot/motions/a.mcap",
                                                 "duration_ms": 5000}),
                lambda: motion.dispatch("play", {"motion_id": "/agibot/motions/b.mcap",
                                                 "duration_ms": 5000}))
            self.assertEqual(len({first["action_id"], second["action_id"]}), 2)
            loser = captured[0][0]
            self.assertIn(loser, (first["action_id"], second["action_id"]))
            survivor = second["action_id"] if loser == first["action_id"] else first["action_id"]
            self.assertEqual(captured[0][1], "cancelled")
            self.assertEqual(captured[0][2]["reason"], "replaced_by_new_play")
            self.assertEqual(motion._play_action_id, survivor)
            # the lock serialises the two dispatches into exactly one shape:
            # [play(lock-winner), cmd_end(settle), play(survivor)] — the robot
            # never receives two overlapping plays without an end between.
            motions = [c[1]["motion_id"] for c in
                       self.transport.calls_to("MotionCommandService", "SendMotionCommand")]
            self.assertEqual(len(motions), 3)
            self.assertEqual(motions.count(""), 1)  # exactly one cmd_end separator
            self.assertEqual(motions[1], "")        # it sits between the two plays
            loser_motion = motions[0]
            expected_loser_motion = ("/agibot/motions/a.mcap"
                                     if loser == first["action_id"]
                                     else "/agibot/motions/b.mcap")
            self.assertEqual(loser_motion, expected_loser_motion)
            self.assertEqual(motions[2], ("/agibot/motions/b.mcap"
                                          if loser == first["action_id"]
                                          else "/agibot/motions/a.mcap"))
            motion.dispatch("stop_play", {})
            time.sleep(0.3)
            for action_id in (first["action_id"], second["action_id"]):
                self.assertEqual(len([c for c in captured if c[0] == action_id]), 1)
        finally:
            device._acp_notify = original_notify

    def test_motion_play_pause_stop_then_new_play_completes_without_resume(self):
        # 10th PR review (pause-state leak): pause belongs to the settled
        # action only. pause → stop_play → a fresh play with NO resume must
        # complete — before the fix the stale _paused flag froze the second
        # worker at _paused.wait() forever.
        motion = find_plugin(self.plugins, "motion_play")
        motion._PLAY_WORKER_TICK_S = 0.02
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            first = motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                             "duration_ms": 5000})
            motion.dispatch("pause", {})
            motion.dispatch("stop_play", {})
            self.assertEqual([(c[0], c[1]) for c in captured],
                             [(first["action_id"], "cancelled")])
            self.assertEqual(captured[0][2]["reason"], "cancelled_by_request")
            second = motion.dispatch("play", {"motion_id": "/agibot/motions/dance.mcap",
                                              "duration_ms": 100})
            # no resume() — the fresh play must run to completion on its own
            deadline = time.time() + 5
            while len(captured) < 2 and time.time() < deadline:
                time.sleep(0.02)
            self.assertEqual([(c[0], c[1]) for c in captured],
                             [(first["action_id"], "cancelled"),
                              (second["action_id"], "completed")])
            self.assertEqual(motion._play_action_id, None)
        finally:
            device._acp_notify = original_notify

    def test_motion_play_play_rejected_outside_motion_state(self):
        # 7th PR review: motion_player is a MOTION-mode application (dev guide
        # §7.1) — a full-body motion in DAMPING would command unpowered joints.
        # The gate matches loco/arm: reject before SendMotionCommand fires.
        self.transport.responses["MotionControlActionService/GetAction"] = {
            "action": "MOTIONCONTROLACTION_DAMPING"}
        motion = find_plugin(self.plugins, "motion_play")
        result = motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                          "duration_ms": 5000})
        self.assertEqual(result["state"], "rejected")
        self.assertEqual(result["current"], "DAMPING")
        self.assertIn("get_up", result["suggestion"])
        # rejected before any RPC reached the robot
        self.assertEqual(self.transport.calls_to("MotionCommandService", "SendMotionCommand"), [])

    def test_motion_play_play_nonzero_header_code_raises_without_arming(self):
        # 8th PR review: a rejected SendMotionCommand (nonzero vendor header.code)
        # must raise instead of returning state=playing — otherwise the ACP
        # worker later reports completed for a motion the robot never started.
        self.transport.responses["MotionCommandService/SendMotionCommand"] = {
            "header": {"code": "3001", "msg": "motion not found"}}
        motion = find_plugin(self.plugins, "motion_play")
        with self.assertRaises(device.RpcError) as ctx:
            motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                     "duration_ms": 5000})
        self.assertIn("3001", str(ctx.exception))
        # no completion worker armed → framework stop reports idle, nothing posted
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            self.assertEqual(motion.dispatch("stop", {}), {"state": "idle"})
            time.sleep(0.3)
            self.assertEqual(captured, [])
        finally:
            device._acp_notify = original_notify

    def test_motion_play_play_unparseable_state_is_permissive(self):
        # Same permissive-on-unparseable policy as the loco/arm gates: an
        # unrecognised GetAction body must not lock the operator out.
        self.transport.responses["MotionControlActionService/GetAction"] = {"header": {"code": "0"}}
        motion = find_plugin(self.plugins, "motion_play")
        result = motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                          "duration_ms": 100})
        self.assertEqual(result["state"], "playing")
        motion.dispatch("stop_play", {})

    def test_motion_play_stop_play_settles_pending_waiter(self):
        # stop_play (and reset) must invalidate the duration-based completion
        # worker AND post an immediate cancelled to Agent Core — merely clearing
        # the id would leave the ACP barrier pending until its 600 s timeout.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            motion = find_plugin(self.plugins, "motion_play")
            motion._PLAY_WORKER_TICK_S = 0.02
            result = motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                              "duration_ms": 5000})
            motion.dispatch("stop_play", {})
            time.sleep(0.5)
            (action_id, status, _, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "cancelled")
            self.assertEqual(tool_name, "motion_play")
            calls = self.transport.calls_to("MotionCommandService", "SendMotionCommand")
            self.assertEqual(calls[-1][1]["cmd_end"], True)
        finally:
            device._acp_notify = original_notify

    def test_motion_play_framework_stop_sends_cmd_end_and_settles(self):
        # 6th PR review: the framework "stop" action must physically halt the
        # motion (cmd_end) and settle the ACP waiter — not just return idle
        # while the robot keeps moving and the worker POSTs a phantom completed.
        # 7th PR review: settling also means an immediate cancelled post.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            motion = find_plugin(self.plugins, "motion_play")
            motion._PLAY_WORKER_TICK_S = 0.02
            result = motion.dispatch("play", {"motion_id": "/agibot/motions/wave.mcap",
                                              "duration_ms": 5000})
            motion.dispatch("stop", {})
            calls = self.transport.calls_to("MotionCommandService", "SendMotionCommand")
            self.assertEqual(calls[-1][1]["cmd_end"], True)
            time.sleep(0.5)
            (action_id, status, _, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "cancelled")
            self.assertEqual(tool_name, "motion_play")
        finally:
            device._acp_notify = original_notify

    def test_arm_control_compliance_dispatch(self):
        arm = find_plugin(self.plugins, "arm_control")
        arm.dispatch("compliance_enable", {})
        (url, _), = self.transport.calls_to("MotionControlMotionService", "EnableArmCompliance")
        self.assertIn("10.42.10.12:56322", url)

    def test_alerts_cooldown_limits_rpc_rate(self):
        alerts = find_plugin(self.plugins, "alerts")
        self.transport.responses["HDSService"] = {"data": {"alerts": [{"code": 1}]}}
        first = alerts.dispatch("query", {})
        second = alerts.dispatch("query", {})  # within the 5 s cooldown → cached
        self.assertEqual(self.transport.calls_to("HDSService", "GetAlertList").__len__(), 1)
        self.assertEqual(first["alerts"], second["alerts"])

    # -- ADU controlled_spatial (mapping + navigation + relocalization + map query) --

    def test_controlled_spatial_mapping_start_and_save(self):
        spatial = find_plugin(self.plugins, "controlled_spatial")
        spatial.dispatch("start_mapping", {})
        (url, body), = self.transport.calls_to("MappingService", "StartMapping")
        self.assertIn("10.42.10.11:50807", url)
        self.assertEqual(body["command"], "MappingCommand_START_MAPPING")
        spatial.dispatch("stop_save", {"map_name": "lobby"})
        (_, save_body), = self.transport.calls_to("MappingService", "StopMapping")
        self.assertEqual(save_body["command"], "MappingCommand_SAVING_MAP")
        self.assertEqual(save_body["map_name"], "lobby")

    def test_controlled_spatial_navi_to_pose_payload(self):
        spatial = find_plugin(self.plugins, "controlled_spatial")
        self.transport.responses["PncService"] = {"task_id": 7}
        spatial.dispatch("navi_to_pose", {"map_id": 3, "x": 1.5, "y": -2.0, "angle": 0.3})
        (url, body), = self.transport.calls_to("PncService", "PlanningNaviToPose2D")
        self.assertIn("10.42.10.11:53176", url)
        self.assertEqual(body["map_id"], 3)
        self.assertEqual(body["pose"]["position"], {"x": 1.5, "y": -2.0})
        self.assertEqual(body["pose"]["angle"], 0.3)
        self.assertEqual(spatial.last_task_id, 7)

    def test_controlled_spatial_control_reuses_last_task_id(self):
        spatial = find_plugin(self.plugins, "controlled_spatial")
        self.transport.responses["PncService"] = {"task_id": 9}
        spatial.dispatch("move_forward", {"map_id": 1, "distance": 0.5})
        spatial.dispatch("pause", {})
        (_, body), = self.transport.calls_to("PncService", "ActionPause")
        self.assertEqual(body["task_id"], 9)

    def test_controlled_spatial_schema_declares_x_completion(self):
        schema = find_plugin(self.plugins, "controlled_spatial").get_tool()["inputSchema"]
        self.assertEqual(schema["x-completion"], {
            "actions": ["navi_to_goal", "navi_to_pose", "linear_to_goal",
                        "linear_to_pose", "move_forward", "spin_turn"],
            "timeout": 180})

    def test_controlled_spatial_navi_reports_acp_completion(self):
        # Navigation is a long-running action: dispatch returns immediately with
        # an action_id and a daemon worker polls PncService/ActionGetState until
        # a terminal state, then POSTs the ACP completion.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            spatial = find_plugin(self.plugins, "controlled_spatial")
            spatial._NAV_POLL_INTERVAL_S = 0.02
            # One PncService response serves both the dispatch and the polls
            # (RecordingTransport matches by service prefix).
            self.transport.responses["PncService"] = {"task_id": 7, "data": {"state": "finished"}}
            result = spatial.dispatch("navi_to_pose", {"map_id": 3, "x": 1.0, "y": 1.0, "angle": 0.0})
            self.assertEqual(result["state"], "navigating")
            self.assertTrue(result["action_id"].startswith("a3_nav_"))
            self.assertEqual(result["task_id"], 7)
            deadline = time.time() + 5
            while not captured and time.time() < deadline:
                time.sleep(0.02)
            (action_id, status, payload, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "completed")
            self.assertEqual(payload["task_id"], 7)
            self.assertEqual(payload["final_state"], "finished")
            self.assertEqual(tool_name, "controlled_spatial")
        finally:
            device._acp_notify = original_notify

    def test_controlled_spatial_cancel_settles_pending_waiter(self):
        # cancel must invalidate the polling worker AND post an immediate
        # cancelled — a silent id clear leaves the ACP barrier pending to timeout.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            spatial = find_plugin(self.plugins, "controlled_spatial")
            spatial._NAV_POLL_INTERVAL_S = 0.2  # long enough to still be polling
            self.transport.responses["PncService"] = {"task_id": 5, "data": {"state": "running"}}
            result = spatial.dispatch("move_forward", {"map_id": 1, "distance": 0.5})
            spatial.dispatch("cancel", {})
            time.sleep(0.5)
            (action_id, status, _, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "cancelled")
            self.assertEqual(tool_name, "controlled_spatial")
        finally:
            device._acp_notify = original_notify

    def test_controlled_spatial_framework_stop_cancels_active_task(self):
        # 6th PR review: the framework "stop" action must physically cancel the
        # running nav task (ActionCancel with the last task_id) and settle the
        # ACP waiter — not just return idle while the robot keeps driving.
        # 7th PR review: settling also means an immediate cancelled post.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            spatial = find_plugin(self.plugins, "controlled_spatial")
            spatial._NAV_POLL_INTERVAL_S = 0.2  # long enough to still be polling
            self.transport.responses["PncService"] = {"task_id": 11, "data": {"state": "running"}}
            result = spatial.dispatch("navi_to_pose", {"map_id": 3, "x": 1.0, "y": 1.0, "angle": 0.0})
            spatial.dispatch("stop", {})
            (url, body), = self.transport.calls_to("PncService", "ActionCancel")
            self.assertEqual(body["task_id"], 11)
            time.sleep(0.5)
            (action_id, status, _, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "cancelled")
            self.assertEqual(tool_name, "controlled_spatial")
        finally:
            device._acp_notify = original_notify

    def test_controlled_spatial_second_nav_settles_previous_and_cancels_old_task(self):
        # Same concurrency class as motion_play (ThreadingHTTPServer): a second
        # nav must settle the first waiter (immediate cancelled) and physically
        # ActionCancel the OLD task BEFORE the new navi RPC fires — otherwise
        # the robot executes two nav tasks at once and the first barrier orphans.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            spatial = find_plugin(self.plugins, "controlled_spatial")
            spatial._NAV_POLL_INTERVAL_S = 5.0  # keep both workers polling
            responses = iter([{"task_id": 11, "data": {"state": "running"}},
                              {"task_id": 12, "data": {"state": "running"}}])
            self.transport.handler = lambda url, service: (
                next(responses) if service.startswith("PncService/MoveForward")
                else {"header": {"code": "0"}})
            first = spatial.dispatch("move_forward", {"map_id": 1, "distance": 0.5})
            second = spatial.dispatch("move_forward", {"map_id": 1, "distance": 1.0})
            self.assertNotEqual(first["action_id"], second["action_id"])
            time.sleep(0.3)
            # exactly one post so far: the first action cancelled immediately
            self.assertEqual([(c[0], c[1], c[3]) for c in captured],
                             [(first["action_id"], "cancelled", "controlled_spatial")])
            self.assertEqual(captured[0][2]["reason"], "replaced_by_new_navigation")
            # the replacement cancelled the OLD task before the new nav fired
            (url, body), = self.transport.calls_to("PncService", "ActionCancel")
            self.assertEqual(body["task_id"], 11)
            # the two MoveForward RPCs fired in order
            nav_calls = self.transport.calls_to("PncService", "MoveForward")
            self.assertEqual(len(nav_calls), 2)
            self.assertEqual([c[1]["distance"] for c in nav_calls], [0.5, 1.0])
            # the second worker is the only one left armed
            self.assertEqual(spatial._nav_action_id, second["action_id"])
            self.assertEqual(spatial.last_task_id, 12)
        finally:
            device._acp_notify = original_notify

    def test_controlled_spatial_simultaneous_navs_never_overlap(self):
        # 10th PR review: two barrier-released nav dispatches must serialise
        # through the lock-held settle→ActionCancel→new-nav→arm transition:
        # distinct ids, the loser cancelled once, and the OLD task cancelled
        # before the new nav RPC fires.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            spatial = find_plugin(self.plugins, "controlled_spatial")
            spatial._NAV_POLL_INTERVAL_S = 5.0  # keep the survivor polling
            task_ids = iter([11, 12])
            self.transport.handler = lambda url, service: (
                {"task_id": next(task_ids), "data": {"state": "running"}}
                if service.startswith("PncService/MoveForward")
                else {"header": {"code": "0"}})
            first, second = run_concurrently(
                lambda: spatial.dispatch("move_forward", {"map_id": 1, "distance": 0.5}),
                lambda: spatial.dispatch("move_forward", {"map_id": 1, "distance": 1.0}))
            self.assertEqual(len({first["action_id"], second["action_id"]}), 2)
            loser = captured[0][0]
            self.assertEqual(captured[0][1], "cancelled")
            self.assertEqual(captured[0][2]["reason"], "replaced_by_new_navigation")
            loser_result = second if loser == second["action_id"] else first
            survivor_result = second if loser == first["action_id"] else first
            self.assertEqual(spatial._nav_action_id, survivor_result["action_id"])
            # the cancelled task is the loser's, and it was cancelled BEFORE
            # the second MoveForward fired (no overlapping nav tasks).
            cancel_calls = self.transport.calls_to("PncService", "ActionCancel")
            self.assertEqual([c[1]["task_id"] for c in cancel_calls], [loser_result["task_id"]])
            nav_calls = self.transport.calls_to("PncService", "MoveForward")
            self.assertEqual(len(nav_calls), 2)
            cancel_index = self.transport.calls.index(cancel_calls[0])
            self.assertLess(cancel_index,
                            self.transport.calls.index(nav_calls[1]))
            spatial.dispatch("stop", {})
            time.sleep(0.3)
            for action_id in (first["action_id"], second["action_id"]):
                self.assertEqual(len([c for c in captured if c[0] == action_id]), 1)
        finally:
            device._acp_notify = original_notify

    def test_controlled_spatial_relocalization_start(self):
        spatial = find_plugin(self.plugins, "controlled_spatial")
        spatial.dispatch("start_relocalization", {"map_dir": "/agibot/data/map/lobby"})
        (url, body), = self.transport.calls_to("SLAMRelocalizationService", "SLAMStartNormalRelocalization")
        self.assertIn("10.42.10.11:50583", url)
        self.assertEqual(body["related_map_dir"], "/agibot/data/map/lobby")

    def test_controlled_spatial_get_map_pixel_conversion(self):
        spatial = find_plugin(self.plugins, "controlled_spatial")
        self.transport.responses["MappingService"] = {
            "data": {"resolution": 0.05, "origin": {"x": 100, "y": 200}}}
        result = spatial.dispatch("get_map", {"map_id": 1, "x": 101.0, "y": 199.0})
        # 8th PR review: world→pixel is the INVERSE of the map transform —
        # subtract the metric origin, divide by resolution (row 0 = highest y):
        # pixel_x = (101 - 100)/0.05 = 20 ; pixel_y = (200 - 199)/0.05 = 20
        self.assertEqual(result["pixel"]["x"], 20)
        self.assertEqual(result["pixel"]["y"], 20)

    def test_controlled_spatial_get_map_pixel_inverse_of_map_transform(self):
        # Round-trip: publishing maps grid cell (row, col) to world
        # (origin + (idx + 0.5)*res, origin - (idx + 0.5)*res); querying a
        # point inside that cell must land back on it. Query slightly inward
        # from the centre so the round() is unambiguous.
        spatial = find_plugin(self.plugins, "controlled_spatial")
        mapping = find_plugin(self.plugins, "spatial_map")
        response = {"data": {
            "resolution": 0.05, "origin": {"x": 100, "y": 200},
            "occupancy_grid": [[127, 0], [0, 127]]}}
        mapping.publish_map(response)
        pub = self.nodes.core.publishers["/test_ns/spatial_map"]
        (msg,) = pub.published
        buf = bytes(msg.data)
        header_size = struct.calcsize("<fffBI")
        # only floor point: row 0, col 0 → world (100.025, 199.975)
        wx, wy, _ = struct.unpack_from("<fff", buf, header_size)
        self.assertAlmostEqual(wx, 100 + 0.5 * 0.05, places=4)
        self.assertAlmostEqual(wy, 200 - 0.5 * 0.05, places=4)
        self.transport.responses["MappingService"] = response
        result = spatial.dispatch("get_map", {"map_id": 0, "x": wx - 0.02, "y": wy + 0.02})
        # continuous pixel (0.1, 0.1) → cell (0, 0)
        self.assertEqual(result["pixel"]["x"], 0)
        self.assertEqual(result["pixel"]["y"], 0)

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

    def test_skill_play_state_without_stream_reports_unknown(self):
        skill = find_plugin(self.plugins, "skill_play")
        result = skill.dispatch("state", {})
        self.assertEqual(result, {"state": "unknown"})

    def test_skill_play_schema_declares_x_completion(self):
        schema = find_plugin(self.plugins, "skill_play").get_tool()["inputSchema"]
        self.assertEqual(schema["x-completion"], {"actions": ["play"], "timeout": 600})

    def test_skill_play_play_reports_acp_completion(self):
        # play is a long-running action: dispatch returns immediately with an
        # action_id and a daemon worker polls the skill_status stream snapshot
        # until a terminal state, then POSTs the ACP completion.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            skill = find_plugin(self.plugins, "skill_play")
            skill._POLL_INTERVAL_S = 0.02
            skill.nodes.values["skill_status"] = {"state": "finished"}
            self.transport.responses["SkillPilotService"] = {"data": {"session_id": "s-9"}}
            result = skill.dispatch("play", {"path": "/agibot/skills/dance"})
            self.assertEqual(result["state"], "playing")
            self.assertTrue(result["action_id"].startswith("skill_play_"))
            self.assertEqual(result["session_id"], "s-9")
            deadline = time.time() + 5
            while not captured and time.time() < deadline:
                time.sleep(0.02)
            (action_id, status, payload, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "completed")
            self.assertEqual(payload["session_id"], "s-9")
            self.assertEqual(payload["final_state"], "finished")
            self.assertEqual(tool_name, "skill_play")
        finally:
            device._acp_notify = original_notify

    def test_skill_play_stop_play_settles_pending_waiter(self):
        # stop_play must invalidate the polling worker AND post an immediate
        # cancelled — a silent id clear leaves the ACP barrier pending to timeout.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            skill = find_plugin(self.plugins, "skill_play")
            skill._POLL_INTERVAL_S = 0.2  # long enough to still be polling
            skill.nodes.values["skill_status"] = {"state": "running"}
            self.transport.responses["SkillPilotService"] = {"data": {"session_id": "s-2"}}
            result = skill.dispatch("play", {"path": "/agibot/skills/dance"})
            skill.dispatch("stop_play", {"session_id": "s-2"})
            time.sleep(0.5)
            (action_id, status, _, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "cancelled")
            self.assertEqual(tool_name, "skill_play")
        finally:
            device._acp_notify = original_notify

    def test_skill_play_framework_stop_stops_last_session(self):
        # 6th PR review: the framework "stop" action must physically stop the
        # running skill (SkillPilot Stop with the remembered session_id) and
        # settle the ACP waiter — stop args carry no session_id.
        # 7th PR review: settling also means an immediate cancelled post.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            skill = find_plugin(self.plugins, "skill_play")
            skill._POLL_INTERVAL_S = 0.2  # long enough to still be polling
            skill.nodes.values["skill_status"] = {"state": "running"}
            self.transport.responses["SkillPilotService"] = {"data": {"session_id": "s-7"}}
            result = skill.dispatch("play", {"path": "/agibot/skills/dance"})
            skill.dispatch("stop", {})
            calls = self.transport.calls_to("SkillPilotService", "SkillPackage")
            self.assertEqual(calls[-1][1]["command"], "Stop")
            self.assertEqual(calls[-1][1]["session_id"], "s-7")
            time.sleep(0.5)
            (action_id, status, _, tool_name), = captured
            self.assertEqual(action_id, result["action_id"])
            self.assertEqual(status, "cancelled")
            self.assertEqual(tool_name, "skill_play")
        finally:
            device._acp_notify = original_notify

    def test_skill_play_second_play_settles_previous_and_stops_old_session(self):
        # Same concurrency class as motion_play (ThreadingHTTPServer): a second
        # play must settle the first waiter (immediate cancelled) and physically
        # Stop the OLD session_id BEFORE the new Start arms — otherwise the
        # robot runs two skills at once and the first barrier orphans.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            skill = find_plugin(self.plugins, "skill_play")
            skill._POLL_INTERVAL_S = 5.0  # keep both workers polling
            skill.nodes.values["skill_status"] = {"state": "running"}
            responses = iter([{"data": {"session_id": "s-1"}},
                              {"data": {"session_id": "s-2"}}])
            def skill_handler(url, service):
                if not service.startswith("SkillPilotService/SkillPackage"):
                    return {"header": {"code": "0"}}
                body = next(c for u, c in reversed(self.transport.calls)
                             if "SkillPilotService/SkillPackage" in u)
                # Start calls consume the queued session_ids; Stop reuses the old one.
                if body.get("command") == "Start":
                    return next(responses)
                return {"data": {"session_id": body.get("session_id", "")}}
            self.transport.handler = skill_handler
            first = skill.dispatch("play", {"path": "/agibot/skills/a"})
            second = skill.dispatch("play", {"path": "/agibot/skills/b"})
            self.assertEqual(first["session_id"], "s-1")
            self.assertEqual(second["session_id"], "s-2")
            self.assertNotEqual(first["action_id"], second["action_id"])
            time.sleep(0.3)
            # exactly one post so far: the first action cancelled immediately
            self.assertEqual([(c[0], c[1], c[3]) for c in captured],
                             [(first["action_id"], "cancelled", "skill_play")])
            self.assertEqual(captured[0][2]["reason"], "replaced_by_new_play")
            # SkillPackage order: Start(path=a) … Stop(s-1) then Start(path=b) —
            # the replacement stopped the OLD session, never the new one.
            calls = self.transport.calls_to("SkillPilotService", "SkillPackage")
            self.assertEqual([(c[1]["command"], c[1].get("session_id"),
                               c[1].get("path")) for c in calls],
                             [("Start", "", "/agibot/skills/a"),
                              ("Stop", "s-1", ""),
                              ("Start", "", "/agibot/skills/b")])
            # the second worker is the only one left armed
            self.assertEqual(skill._play_action_id, second["action_id"])
            self.assertEqual(skill._last_session_id, "s-2")
        finally:
            device._acp_notify = original_notify

    def test_skill_play_simultaneous_plays_never_overlap(self):
        # 10th PR review: two barrier-released plays must serialise through the
        # lock-held settle→Stop-old-session→Start→arm transition: distinct ids,
        # the loser cancelled once, and the OLD session stopped before the new
        # Start fires.
        captured = []
        original_notify = device._acp_notify
        device._acp_notify = lambda action_id, status, result, tool="": captured.append(
            (action_id, status, result, tool))
        try:
            skill = find_plugin(self.plugins, "skill_play")
            skill._POLL_INTERVAL_S = 5.0  # keep the survivor polling
            skill.nodes.values["skill_status"] = {"state": "running"}
            sessions = iter(["s-a", "s-b"])
            def skill_handler(url, service):
                if not service.startswith("SkillPilotService/SkillPackage"):
                    return {"header": {"code": "0"}}
                body = next(c for u, c in reversed(self.transport.calls)
                             if "SkillPilotService/SkillPackage" in u)
                if body.get("command") == "Start":
                    return {"data": {"session_id": next(sessions)}}
                return {"data": {"session_id": body.get("session_id", "")}}
            self.transport.handler = skill_handler
            first, second = run_concurrently(
                lambda: skill.dispatch("play", {"path": "/agibot/skills/a"}),
                lambda: skill.dispatch("play", {"path": "/agibot/skills/b"}))
            self.assertEqual(len({first["action_id"], second["action_id"]}), 2)
            loser = captured[0][0]
            self.assertEqual(captured[0][1], "cancelled")
            self.assertEqual(captured[0][2]["reason"], "replaced_by_new_play")
            loser_result = second if loser == second["action_id"] else first
            survivor_result = second if loser == first["action_id"] else first
            self.assertEqual(skill._play_action_id, survivor_result["action_id"])
            # the stopped session is the loser's, stopped BEFORE the new Start
            calls = self.transport.calls_to("SkillPilotService", "SkillPackage")
            stop_calls = [c for c in calls if c[1]["command"] == "Stop"]
            self.assertEqual([c[1]["session_id"] for c in stop_calls], [loser_result["session_id"]])
            self.assertLess(calls.index(stop_calls[0]), calls.index(calls[-1]))
            skill.dispatch("stop", {})
            time.sleep(0.3)
            for action_id in (first["action_id"], second["action_id"]):
                self.assertEqual(len([c for c in captured if c[0] == action_id]), 1)
        finally:
            device._acp_notify = original_notify

    # -- model resource --

    def test_model_returns_urdf(self):
        model = find_plugin(self.plugins, "model")
        result = model.dispatch("model", {})
        self.assertIn("<robot", result["urdf"])
        self.assertTrue("a3_ultra" in result["urdf"].lower())


class RobotSubnetIpTests(unittest.TestCase):
    """_robot_subnet_ip must only trust a genuine 10.42.10.x source address.

    On a host with no specific route to the robot subnet, the UDP-connect trick
    returns the *default-route* address (e.g. an office LAN 192.168.x.x) —
    whitelisting that would silently expose domain 42 on the office LAN while
    domain 232 still cannot reach the A3 units. Patch socket to force each path.
    """

    def _patch_socket(self, getsockname_ip):
        class FakeSocket:
            def __init__(self, family, kind):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

            def connect(self, address):
                pass

            def getsockname(self):
                return (getsockname_ip, 12345)

        original_socket = main.socket.socket
        main.socket.socket = FakeSocket
        self.addCleanup(setattr, main.socket, "socket", original_socket)

    def test_returns_address_on_robot_subnet(self):
        self._patch_socket("10.42.10.77")
        self.assertEqual(main._robot_subnet_ip(), "10.42.10.77")

    def test_rejects_default_route_office_address(self):
        self._patch_socket("192.168.1.42")
        self.assertEqual(main._robot_subnet_ip(), "")

    def test_rejects_adjacent_subnet(self):
        # 10.42.11.x is NOT the A3 body subnet — must not be whitelisted.
        self._patch_socket("10.42.11.5")
        self.assertEqual(main._robot_subnet_ip(), "")

    def test_socket_error_returns_empty(self):
        class FailingSocket:
            def __init__(self, family, kind):
                raise OSError("no route to host")

        original_socket = main.socket.socket
        main.socket.socket = FailingSocket
        self.addCleanup(setattr, main.socket, "socket", original_socket)
        self.assertEqual(main._robot_subnet_ip(), "")

    def test_select_profile_writes_profile_and_sets_env(self):
        # Successful path: on a robot-subnet host the profile must actually be
        # written (to a directory the Dockerfile creates) and the env var set.
        # Regression: the profile used to live in /work/agibot-a3/, which no
        # build step creates — the write failed on the robot, the OSError
        # branch cleared FASTRTPS_DEFAULT_PROFILES_FILE, and both domains
        # fell back to every interface.
        class FakeSocket:
            def __init__(self, family, kind):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

            def connect(self, address):
                pass

            def getsockname(self):
                return ("10.42.10.77", 12345)

        original_socket = main.socket.socket
        main.socket.socket = FakeSocket
        self.addCleanup(setattr, main.socket, "socket", original_socket)
        # Redirect the profile path into a temp dir (parent directory exists,
        # so the write succeeds like it does in the container).
        with tempfile.TemporaryDirectory() as tmp:
            fake_path = str(Path(tmp) / "dds-profile.xml")
            original_profile_path = main.PROFILE_PATH
            main.PROFILE_PATH = fake_path
            self.addCleanup(setattr, main, "PROFILE_PATH", original_profile_path)
            original_env = main.os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
            self.addCleanup(main.os.environ.pop, "FASTRTPS_DEFAULT_PROFILES_FILE", None)
            if original_env is not None:
                self.addCleanup(main.os.environ.__setitem__,
                                "FASTRTPS_DEFAULT_PROFILES_FILE", original_env)
            main._select_profile()
            self.assertEqual(main.os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"),
                             fake_path)
            text = Path(fake_path).read_text(encoding="utf-8")
            self.assertIn("<address>10.42.10.77</address>", text)
            self.assertIn("<address>127.0.0.1</address>", text)

    def test_profile_path_lives_in_dockerfile_created_directory(self):
        # /work/agibot/A3/ is COPYied by the Dockerfile; /work/agibot-a3/ never
        # exists, so writing there would fail on the robot.
        self.assertTrue(main.PROFILE_PATH.startswith("/work/agibot/A3/"))


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
        core_pub = self.nodes.core.publishers["/test_ns/arm_state"]
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

    def test_imu_info_lists_both_topic_out_streams(self):
        # 6th PR review: Agent Core treats info() as authoritative for topic
        # inference — the imu card must advertise both mirrored streams.
        imu = find_plugin(self.plugins, "imu")
        result = imu.dispatch("info", {})
        topics = {(entry["topic"], entry["format"]) for entry in result["topic_out"]}
        self.assertEqual(topics, {
            ("/test_ns/state/imu", "data/json"),
        })
        self.assertEqual(set(result["streams"]), {"imu_pelvis", "imu_torso"})

    def test_lidar_info_lists_topic_out(self):
        lidar = find_plugin(self.plugins, "lidar_cloud")
        result = lidar.dispatch("info", {})
        stream = self.nodes.streams["lidar_cloud"]
        self.assertEqual(result["topic_out"],
                         [{"topic": stream["topic"], "format": stream["format"]}])

    def test_battery_and_estop_info_report_unavailable_without_wheel(self):
        # 6th PR review: in degraded mode (no a3_aimdk wheel) the battery/estop
        # cards must say "unavailable" instead of a perpetually-running
        # placeholder that Agent Core would treat as a live stream.
        for name in ("battery", "estop"):
            plugin = find_plugin(self.plugins, name)
            for action in ("query", "info"):
                result = plugin.dispatch(action, {})
                self.assertEqual(result["state"], "unavailable",
                                 f"{name}.{action} must be explicit in degraded mode")
                self.assertIn("a3_aimdk", result["reason"])

    def test_mic_cards_remain_visible_without_audio_msgs(self):
        config = json.loads(json.dumps(BASE_CONFIG))
        config["plugins"] = FULL_PLUGINS
        plugins, _ = build_bundle_plugins(config)
        for name in ("mic", "ext_mic"):
            plugin = find_plugin(plugins, name)
            definition = plugin.get_tool()
            self.assertEqual(definition["type"], "sensor")
            self.assertEqual(definition["topic_out"], [{
                "topic": f"/test_ns/{name}/audio", "format": "audio/pcm-16k",
            }])
            result = plugin.dispatch("info", {})
            self.assertEqual(result["state"], "unavailable")


class SpatialMapTests(unittest.TestCase):
    """spatial_map sensor card: occupancy grid → canonical sensor/mapping binary."""

    def setUp(self):
        self.config = json.loads(json.dumps(BASE_CONFIG))
        self.config["plugins"] = FULL_PLUGINS
        self.plugins, self.transport = build_bundle_plugins(self.config)
        self.nodes = next(iter(self.plugins.values())).nodes
        self.plugin = find_plugin(self.plugins, "spatial_map")

    def test_tool_declares_mapping_topic(self):
        definition = self.plugin.get_tool()
        self.assertEqual(definition["name"], "spatial_map")
        self.assertEqual(definition["type"], "sensor")
        self.assertEqual(definition["topic_out"],
                         [{"topic": "/test_ns/spatial_map", "format": "sensor/mapping"}])

    def test_info_lists_topic_out(self):
        # 7th PR review: Agent Core derives subscribable topics from the info
        # response too — it must mirror the static topic_out declaration.
        result = self.plugin.dispatch("info", {})
        self.assertEqual(result["state"], "idle")
        self.assertEqual(result["topic_out"],
                         [{"topic": "/test_ns/spatial_map", "format": "sensor/mapping"}])
        self.assertEqual(result["topic"], "/test_ns/spatial_map")

    def test_publish_map_binary_format(self):
        response = {"data": {
            "resolution": 0.05, "origin": {"x": 100, "y": 200},
            "occupancy_grid": [[127, 0, 1], [0, 127, 0]],
        }}
        published = self.plugin.publish_map(response)
        self.assertTrue(published)
        pub = self.nodes.core.publishers["/test_ns/spatial_map"]
        (msg,) = pub.published
        buf = bytes(msg.data)
        self.assertIsInstance(msg.data, array.array)

        # header: robot x/y/yaw, kind=7 (mapping), point count
        x, y, yaw, kind, point_count = struct.unpack_from("<fffBI", buf, 0)
        self.assertEqual((x, y, yaw), (0.0, 0.0, 0.0))
        self.assertEqual(kind, 7)
        # floor cells (127) at (0,0)/(1,1) + feature cell (1) at (0,2)
        self.assertEqual(point_count, 3)

        # first point: floor layer at row 0, col 0 → origin + 0.5*res
        # (header is 17 bytes: 3 floats + 1 byte kind + uint32 count — no packing)
        header_size = struct.calcsize("<fffBI")
        px, py, pz = struct.unpack_from("<fff", buf, header_size)
        self.assertAlmostEqual(px, 100 + 0.5 * 0.05, places=4)
        # grid row 0 is the highest y: world_y = origin_y - 0.5*res
        self.assertAlmostEqual(py, 200 - 0.5 * 0.05, places=4)
        self.assertAlmostEqual(pz, -0.03, places=5)

        # trailing meta JSON (version 3, no robot pose available)
        meta_offset = header_size + point_count * 12
        (meta_len,) = struct.unpack_from("<I", buf, meta_offset)
        meta = json.loads(buf[meta_offset + 4:meta_offset + 4 + meta_len])
        self.assertEqual(meta["version"], 3)
        self.assertFalse(meta["robot"]["pose_available"])
        self.assertEqual(meta["floor_points"], 2)
        self.assertEqual(meta["feature_points"], 1)
        self.assertEqual(meta["grid_points"], 3)
        self.assertAlmostEqual(meta["resolution"], 0.05, places=6)

    def test_refresh_polls_rpc_and_publishes(self):
        self.transport.responses["MappingService"] = {"data": {
            "resolution": 0.05, "origin": [0, 0], "occupancy_grid": [[0, 100]]}}
        result = self.plugin.dispatch("refresh", {"map_id": 2})
        self.assertTrue(result["published"])
        (url, body), = self.transport.calls_to("MappingService", "Get2DWholeMap")
        self.assertIn("10.42.10.11:50807", url)
        self.assertEqual(body["map_id"], 2)
        pub = self.nodes.core.publishers["/test_ns/spatial_map"]
        self.assertEqual(len(pub.published), 1)

    def test_empty_grid_publishes_empty_frame(self):
        published = self.plugin.publish_map({"data": {"resolution": 0.05}})
        self.assertTrue(published)
        pub = self.nodes.core.publishers["/test_ns/spatial_map"]
        (msg,) = pub.published
        buf = bytes(msg.data)
        *_, point_count = struct.unpack_from("<fffBI", buf, 0)
        self.assertEqual(point_count, 0)


if __name__ == "__main__":
    unittest.main()
