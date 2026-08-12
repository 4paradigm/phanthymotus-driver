import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class FakeString:
    def __init__(self):
        self.data = ""


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self, name):
        self.publishers = []

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher(topic)
        self.publishers.append(publisher)
        return publisher

    def create_timer(self, interval_sec, callback):
        return types.SimpleNamespace(interval_sec=interval_sec, callback=callback)

    def get_logger(self):
        return types.SimpleNamespace(info=lambda message: None, warn=lambda message: None)


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
        self.__class__.instances.append(self)

    def Init(self, callback, queue_len):
        self.callback = callback
        self.queue_len = queue_len


class SportModeState:
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
    sys.modules.update({"rclpy": rclpy, "rclpy.node": rclpy_node, "rclpy.qos": rclpy_qos})
    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.Header = type("Header", (), {})
    std_msgs.String = FakeString
    audio_msgs = types.ModuleType("audio_msgs.msg")
    audio_msgs.AudioChunk = type("AudioChunk", (), {})
    sys.modules.update({"std_msgs.msg": std_msgs, "audio_msgs.msg": audio_msgs})

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = FakeChannelSubscriber
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_go.msg.dds_")
    dds.SportModeState_ = SportModeState
    g1_audio = types.ModuleType("unitree_sdk2py.g1.audio.g1_audio_client")
    g1_audio.AudioClient = object
    sys.modules.update({
        "unitree_sdk2py": types.ModuleType("unitree_sdk2py"),
        "unitree_sdk2py.core": types.ModuleType("unitree_sdk2py.core"),
        "unitree_sdk2py.core.channel": channel,
        "unitree_sdk2py.idl": types.ModuleType("unitree_sdk2py.idl"),
        "unitree_sdk2py.idl.unitree_go": types.ModuleType("unitree_sdk2py.idl.unitree_go"),
        "unitree_sdk2py.idl.unitree_go.msg": types.ModuleType("unitree_sdk2py.idl.unitree_go.msg"),
        "unitree_sdk2py.idl.unitree_go.msg.dds_": dds,
        "unitree_sdk2py.g1": types.ModuleType("unitree_sdk2py.g1"),
        "unitree_sdk2py.g1.audio": types.ModuleType("unitree_sdk2py.g1.audio"),
        "unitree_sdk2py.g1.audio.g1_audio_client": g1_audio,
    })
    pointcloud_utils = types.ModuleType("pointcloud_utils")
    pointcloud_utils.gravity_align_inplace = lambda *args, **kwargs: None
    numpy = types.ModuleType("numpy")
    numpy.ndarray = object
    sys.modules.update({"pointcloud_utils": pointcloud_utils, "numpy": numpy})


def load_device():
    install_stubs()
    spec = importlib.util.spec_from_file_location("g1_foot_contact_slip_test", ROOT / "device.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FootContactSlipPluginTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = load_device()

    def setUp(self):
        FakeChannelSubscriber.instances = []
        self.executor = FakeExecutor()
        self.plugin = self.device.FootContactSlipPlugin({}, "test_g1", self.executor)
        self.node = self.plugin._node

    def _sample(self, forces, now):
        self.node._on_sport_state(types.SimpleNamespace(foot_force=forces), now=now)

    def test_tool_and_subscription_contract(self):
        tool = self.plugin.get_tool()
        self.assertEqual("foot_contact_slip", tool["name"])
        self.assertEqual("sensor", tool["type"])
        self.assertEqual({}, tool["inputSchema"]["properties"])
        self.assertIn("never commands motion", tool["description"])
        self.assertEqual(
            [{"topic": "/test_g1/state/foot_contact_slip", "format": "data/json"}],
            tool["topic_out"],
        )
        self.assertEqual("rt/odommodestate", FakeChannelSubscriber.instances[0].topic)
        self.assertIs(SportModeState, FakeChannelSubscriber.instances[0].message_type)

    def test_zero_force_is_explicitly_not_observable(self):
        self._sample([0, 0, 0, 0], 10.0)
        state = self.node.current_state(now=10.0)
        self.assertEqual("unsupported", state["state"])
        self.assertFalse(state["contact_observable"])
        self.assertFalse(state["force_signal_observed"])
        self.assertEqual("no_force_signal_observed", state["reason"])
        self.assertIsNone(state["left_contact"])
        self.assertIsNone(state["slip_score"])
        self.assertNotIn("source_topic", state)
        self.assertNotIn("topic_out", state)

    def test_aggregates_contacts_and_load_ratio(self):
        self._sample([35, 25, 20, 20], 20.0)
        state = self.node.current_state(now=20.0)
        self.assertEqual("running", state["state"])
        self.assertTrue(state["left_contact"])
        self.assertTrue(state["right_contact"])
        self.assertEqual(0.6, state["left_load_ratio"])
        self.assertEqual(0.4, state["right_load_ratio"])
        self.assertEqual(0.0, state["slip_score"])
        self.assertEqual("none", state["slipping_foot"])
        self.assertNotIn("source_topic", state)
        self.assertNotIn("topic_out", state)

    def test_reports_conservative_left_force_instability(self):
        self._sample([80, 80, 40, 40], 30.0)
        self._sample([12, 12, 40, 40], 30.1)
        state = self.node.current_state(now=30.1)
        self.assertGreaterEqual(state["slip_score"], 0.65)
        self.assertEqual("left", state["slipping_foot"])
        self.assertEqual("force_distribution_unstable", state["reason"])

    def test_stale_data_never_claims_contact_or_slip(self):
        self._sample([30, 30, 30, 30], 40.0)
        state = self.node.current_state(now=41.0)
        self.assertEqual("stale", state["state"])
        self.assertFalse(state["connected"])
        self.assertIsNone(state["right_contact"])
        self.assertIsNone(state["slip_score"])

    def test_dispatch_has_no_read_action(self):
        self._sample([30, 30, 30, 30], 50.0)
        state = self.plugin.dispatch("foot_contact_slip", {})
        self.assertEqual("running", state["state"])
        self.assertIsNone(self.plugin.dispatch("read", {}))
        self.assertEqual("running", self.plugin.dispatch("info", {})["state"])

    def test_publishes_and_is_registered(self):
        self._sample([30, 30, 30, 30], 60.0)
        self.node._publish_state()
        published = json.loads(self.executor.nodes[0].publishers[0].messages[-1].data)
        self.assertEqual("running", published["state"])
        self.assertTrue(published["contact_observable"])
        self.assertNotIn("source_topic", published)
        self.assertNotIn("topic_out", published)
        self.assertIn("foot_contact_slip:", (ROOT / "config.yaml").read_text())
        self.assertIn("FootContactSlipPlugin", (ROOT / "main.py").read_text())


if __name__ == "__main__":
    unittest.main()
