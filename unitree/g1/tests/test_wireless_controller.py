import importlib.util
import json
import struct
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def json_float(value):
    return struct.pack("f", value)


class Message:
    pass


class FakeString(Message):
    def __init__(self):
        self.data = ""


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def warn(self, message):
        self.warnings.append(message)


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.publishers = []
        self.timers = []
        self.logger = FakeLogger()

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher(topic)
        self.publishers.append(publisher)
        return publisher

    def create_timer(self, interval_sec, callback):
        timer = types.SimpleNamespace(interval_sec=interval_sec, callback=callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self.logger


class FakeExecutor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class FakeChannelSubscriber:
    instances = []

    def __init__(self, topic, message_type):
        self.topic = topic
        self.message_type = message_type
        self.callback = None
        self.queue_len = None
        FakeChannelSubscriber.instances.append(self)

    def Init(self, callback, queue_len):
        self.callback = callback
        self.queue_len = queue_len


class LowState:
    pass


def install_stubs():
    FakeChannelSubscriber.instances = []

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos

    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.Header = type("Header", (Message,), {})
    std_msgs.String = FakeString
    sys.modules["std_msgs.msg"] = std_msgs

    audio_msgs = types.ModuleType("audio_msgs.msg")
    audio_msgs.AudioChunk = type("AudioChunk", (Message,), {})
    sys.modules["audio_msgs.msg"] = audio_msgs

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = FakeChannelSubscriber
    sys.modules["unitree_sdk2py"] = types.ModuleType("unitree_sdk2py")
    sys.modules["unitree_sdk2py.core"] = types.ModuleType("unitree_sdk2py.core")
    sys.modules["unitree_sdk2py.core.channel"] = channel

    g1_audio = types.ModuleType("unitree_sdk2py.g1.audio.g1_audio_client")
    g1_audio.AudioClient = object
    sys.modules["unitree_sdk2py.g1"] = types.ModuleType("unitree_sdk2py.g1")
    sys.modules["unitree_sdk2py.g1.audio"] = types.ModuleType("unitree_sdk2py.g1.audio")
    sys.modules["unitree_sdk2py.g1.audio.g1_audio_client"] = g1_audio

    dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    dds.LowState_ = LowState
    sys.modules["unitree_sdk2py.idl"] = types.ModuleType("unitree_sdk2py.idl")
    sys.modules["unitree_sdk2py.idl.unitree_hg"] = types.ModuleType("unitree_sdk2py.idl.unitree_hg")
    sys.modules["unitree_sdk2py.idl.unitree_hg.msg"] = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg")
    sys.modules["unitree_sdk2py.idl.unitree_hg.msg.dds_"] = dds

    pointcloud_utils = types.ModuleType("pointcloud_utils")
    pointcloud_utils.gravity_align_inplace = lambda *args, **kwargs: None
    sys.modules["pointcloud_utils"] = pointcloud_utils

    numpy = types.ModuleType("numpy")
    numpy.ndarray = object
    sys.modules.setdefault("numpy", numpy)


def load_device():
    install_stubs()
    spec = importlib.util.spec_from_file_location("g1_device_wireless_test", ROOT / "device.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class WirelessControllerPluginTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = load_device()

    def setUp(self):
        FakeChannelSubscriber.instances = []
        self.executor = FakeExecutor()
        self.plugin = self.device.WirelessControllerPlugin({}, "test_g1", self.executor)

    def test_tool_contract(self):
        tool = self.plugin.get_tool()
        self.assertEqual("wireless_controller", tool["name"])
        self.assertEqual("sensor", tool["type"])
        self.assertEqual(
            [{"topic": "/test_g1/state/wireless_controller", "format": "data/json"}],
            tool["topic_out"],
        )
        self.assertEqual(["read", "info"], tool["inputSchema"]["properties"]["action"]["enum"])
        self.assertIn("read-only", tool["description"])

    def test_subscribes_to_unitree_lowstate_dds(self):
        self.assertEqual(1, len(FakeChannelSubscriber.instances))
        sub = FakeChannelSubscriber.instances[0]
        self.assertEqual("rt/lowstate", sub.topic)
        self.assertIs(sub.message_type, LowState)
        self.assertEqual(10, sub.queue_len)

    def test_publishes_json_and_returns_current_input_state(self):
        remote = [0] * 40
        remote[2] = 1
        remote[3] = 2
        remote[4:8] = list(json_float(0.1))
        remote[8:12] = list(json_float(0.3))
        remote[12:16] = list(json_float(-0.4))
        remote[20:24] = list(json_float(-0.2))
        msg = types.SimpleNamespace(wireless_remote=remote)
        FakeChannelSubscriber.instances[0].callback(msg)

        node = self.executor.nodes[0]
        self.assertEqual([], node.publishers[0].messages)
        self.plugin._node._publish_state()
        published = json.loads(node.publishers[0].messages[-1].data)
        self.assertEqual("running", published["state"])
        self.assertTrue(published["connected"])
        self.assertEqual("fresh", published["reason"])
        self.assertEqual(1, published["sample_count"])
        self.assertEqual("rt/lowstate", published["source_topic"])
        self.assertEqual(0, published["last_update_ago_ms"])
        self.assertAlmostEqual(0.1, published["left_stick"]["x"])
        self.assertAlmostEqual(-0.2, published["left_stick"]["y"])
        self.assertTrue(published["left_stick"]["active"])
        self.assertEqual("y", published["left_stick"]["primary_axis"])
        self.assertEqual("negative", published["left_stick"]["direction"])
        self.assertAlmostEqual(0.3, published["right_stick"]["x"])
        self.assertAlmostEqual(-0.4, published["right_stick"]["y"])
        self.assertTrue(published["right_stick"]["active"])
        self.assertEqual(513, published["buttons"]["raw"])
        self.assertTrue(published["buttons"]["active"])

        state = self.plugin.dispatch("wireless_controller", {})
        self.assertEqual("running", state["state"])
        self.assertTrue(state["connected"])
        self.assertAlmostEqual(0.1, state["left_stick"]["x"])
        self.assertAlmostEqual(-0.2, state["left_stick"]["y"])
        self.assertAlmostEqual(0.3, state["right_stick"]["x"])
        self.assertAlmostEqual(-0.4, state["right_stick"]["y"])
        self.assertEqual(513, state["buttons"]["raw"])
        self.assertEqual(1, state["sample_count"])
        self.assertGreaterEqual(state["last_update_ago_ms"], 0)

        read_state = self.plugin.dispatch("read", {})
        self.assertEqual(state["buttons"], read_state["buttons"])

    def test_no_data_state_is_explicit(self):
        state = self.plugin.dispatch("read", {})
        self.assertEqual("no_data", state["state"])
        self.assertFalse(state["connected"])
        self.assertEqual("no_data", state["reason"])
        self.assertEqual(0, state["sample_count"])
        self.assertEqual(-1, state["last_update_ago_ms"])
        self.assertEqual(
            [{"topic": "/test_g1/state/wireless_controller", "format": "data/json"}],
            state["topic_out"],
        )

    def test_info_reports_source_and_freshness_without_axes(self):
        msg = types.SimpleNamespace(wireless_remote=[0] * 40)
        FakeChannelSubscriber.instances[0].callback(msg)

        info = self.plugin.dispatch("info", {})
        self.assertEqual("running", info["state"])
        self.assertEqual("rt/lowstate", info["source_topic"])
        self.assertEqual(
            [{"topic": "/test_g1/state/wireless_controller", "format": "data/json"}],
            info["topic_out"],
        )
        self.assertEqual(1.0, info["fresh_timeout_sec"])
        self.assertEqual(0.1, info["deadzone"])
        self.assertTrue(info["connected"])
        self.assertEqual(1, info["sample_count"])
        self.assertNotIn("left_stick", info)
        self.assertNotIn("buttons", info)

    def test_stale_state_is_published_by_timer(self):
        node = self.executor.nodes[0]
        stale_now = 102.0
        msg = types.SimpleNamespace(wireless_remote=[0] * 40)
        self.plugin._node._on_wireless_controller(msg)
        with self.plugin._node._lock:
            self.plugin._node._last_state["_updated_at"] = 100.0

        state = self.plugin._node.current_state(now=stale_now)
        self.assertEqual("stale", state["state"])
        self.assertFalse(state["connected"])
        self.assertEqual("stale", state["reason"])
        self.assertEqual(1, state["sample_count"])
        self.assertGreaterEqual(state["last_update_ago_ms"], 1000)

        self.plugin._node._publish_state(now=stale_now)
        published = json.loads(node.publishers[0].messages[-1].data)
        self.assertEqual("stale", published["state"])
        self.assertFalse(published["connected"])

    def test_config_and_main_register_plugin(self):
        config_text = (ROOT / "config.yaml").read_text()
        main_text = (ROOT / "main.py").read_text()
        self.assertIn("wireless_controller:", config_text)
        self.assertIn("WirelessControllerPlugin", main_text)
        self.assertIn('plugins_cfg.get("wireless_controller"', main_text)


if __name__ == "__main__":
    unittest.main()
