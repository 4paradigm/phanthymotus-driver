import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
G1_DIR = REPO_ROOT / "unitree" / "g1"


class _FakeLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


class _FakeClock:
    def __init__(self, node):
        self._node = node

    def now(self):
        return types.SimpleNamespace(nanoseconds=self._node._fake_clock_ns)


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeNode:
    def __init__(self, *_args, **_kwargs):
        self._fake_clock_ns = 2_000_000_000
        self._created_subscriptions = []
        self._created_timers = []
        self._created_publishers = []

    def create_publisher(self, msg_type, topic, qos):
        pub = _FakePublisher()
        self._created_publishers.append((msg_type, topic, qos, pub))
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = types.SimpleNamespace(msg_type=msg_type, topic=topic, callback=callback, qos=qos)
        self._created_subscriptions.append(sub)
        return sub

    def create_timer(self, period, callback):
        timer = types.SimpleNamespace(period=period, callback=callback)
        self._created_timers.append(timer)
        return timer

    def get_clock(self):
        return _FakeClock(self)

    def get_logger(self):
        return _FakeLogger()


class _FakeQoSProfile:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class _String:
    def __init__(self, data=""):
        self.data = data


class _Header:
    pass


class _UInt8MultiArray:
    def __init__(self):
        self.data = []


class _CompressedImage:
    def __init__(self, sec=0, nanosec=0):
        self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec, nanosec=nanosec))


class _Image(_CompressedImage):
    pass


def _install_import_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_node.Node = _FakeNode
    rclpy_qos.QoSProfile = _FakeQoSProfile
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT="best_effort")
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE="volatile")

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Header = _Header
    std_msgs_msg.String = _String
    std_msgs_msg.UInt8MultiArray = _UInt8MultiArray

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CompressedImage = _CompressedImage
    sensor_msgs_msg.Image = _Image

    audio_msgs = types.ModuleType("audio_msgs")
    audio_msgs_msg = types.ModuleType("audio_msgs.msg")
    audio_msgs_msg.AudioChunk = type("AudioChunk", (), {})

    pointcloud_utils = types.ModuleType("pointcloud_utils")
    pointcloud_utils.gravity_align_inplace = lambda *_args, **_kwargs: None

    numpy = types.ModuleType("numpy")

    audio_client_mod = types.ModuleType("unitree_sdk2py.g1.audio.g1_audio_client")
    audio_client_mod.AudioClient = type("AudioClient", (), {})

    modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "audio_msgs": audio_msgs,
        "audio_msgs.msg": audio_msgs_msg,
        "pointcloud_utils": pointcloud_utils,
        "numpy": numpy,
        "unitree_sdk2py": types.ModuleType("unitree_sdk2py"),
        "unitree_sdk2py.g1": types.ModuleType("unitree_sdk2py.g1"),
        "unitree_sdk2py.g1.audio": types.ModuleType("unitree_sdk2py.g1.audio"),
        "unitree_sdk2py.g1.audio.g1_audio_client": audio_client_mod,
    }
    sys.modules.update(modules)


def _load_device_module():
    _install_import_stubs()
    spec = importlib.util.spec_from_file_location("g1_device_under_test", G1_DIR / "device.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _FakeExecutor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class NetworkDdsHealthTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = _load_device_module()

    def test_tool_schema_is_sensor_only(self):
        executor = _FakeExecutor()
        plugin = self.device.NetworkDdsHealthPlugin({"publish_hz": 2.0, "mcp_port": 15701}, "g1", executor)

        tool = plugin.get_tools()[0]
        self.assertEqual(tool["name"], "network_dds_health")
        self.assertEqual(tool["type"], "sensor")
        self.assertEqual(
            tool["inputSchema"]["properties"]["action"]["enum"],
            ["current_frame"],
        )
        self.assertIn("current_frame", tool["inputSchema"]["x-action-params"])
        self.assertEqual(tool["topic_out"], [{"topic": "/g1/state/network_dds_health", "format": "data/json"}])
        self.assertEqual(tool["configSchema"]["properties"]["publish_hz"]["default"], 2.0)
        self.assertNotIn("read", json.dumps(tool).lower())
        self.assertNotIn("snapshot", json.dumps(tool).lower())
        self.assertEqual(len(executor.nodes), 1)

    def test_publish_rate_config_controls_timer_period(self):
        node = self.device._NetworkDdsHealthNode("/g1/state/network_dds_health", "g1", 15701, publish_hz=5.0)

        self.assertAlmostEqual(node._created_timers[0].period, 0.2)

    def test_current_frame_action_returns_latest_state_without_topic_metadata(self):
        executor = _FakeExecutor()
        plugin = self.device.NetworkDdsHealthPlugin({"publish_hz": 2.0, "mcp_port": 15701}, "g1", executor)
        executor.nodes[0]._on_topic("battery", _String("{}"))

        frame = plugin.dispatch("current_frame", {})

        self.assertIn("checks", frame)
        self.assertIn("metrics", frame)
        self.assertNotIn("source_topic", frame)
        self.assertNotIn("topic_out", frame)
        self.assertNotIn("snapshot", json.dumps(frame).lower())

    def test_subscribes_to_existing_driver_topics(self):
        node = self.device._NetworkDdsHealthNode("/g1/state/network_dds_health", "g1", 15701)

        topics = {sub.topic for sub in node._created_subscriptions}
        self.assertIn("/g1/loco/state", topics)
        self.assertIn("/g1/state/imu", topics)
        self.assertIn("/g1/state/battery", topics)
        self.assertIn("/g1/lidar/cloud", topics)

    def test_freshness_and_optional_motion_topic(self):
        node = self.device._NetworkDdsHealthNode("/g1/state/network_dds_health", "g1", 15701)
        node._fake_clock_ns = 2_050_000_000

        node._on_topic("loco_state", _String("{}"))
        node._on_topic("lidar_cloud", _UInt8MultiArray())

        state = node.current_state()
        self.assertTrue(state["checks"]["mcp"])
        self.assertTrue(next(item for item in state["dds"] if item["name"] == "loco_state")["fresh"])
        self.assertTrue(state["lidar"][0]["fresh"])
        self.assertFalse(state["optional"][0]["required"])
        self.assertNotIn("camera_ok", state)
        self.assertNotIn("source_topic", state)
        self.assertNotIn("topic_out", state)

    def test_stale_and_estimated_missed_samples(self):
        stat = self.device._TopicHealth("imu", "/g1/state/imu", "dds", expected_hz=20.0, stale_after_s=0.5, required=True)
        stat.record(_String("{}"), now=10.0, clock_ns=None)
        stat.record(_String("{}"), now=10.5, clock_ns=None)

        payload = stat.payload(now=11.2)
        self.assertEqual(payload["status"], "stale")
        self.assertGreaterEqual(payload["missed"], 9)

    def test_optional_motion_topic_does_not_fail_dds_group(self):
        node = self.device._NetworkDdsHealthNode("/g1/state/network_dds_health", "g1", 15701)
        for name in ("loco_state", "imu", "battery", "joints", "mainboard"):
            node._on_topic(name, _String("{}"))

        state = node.current_state()
        self.assertTrue(state["checks"]["dds"])
        self.assertEqual(state["optional"][0]["status"], "no_data")
        self.assertFalse(state["optional"][0]["required"])

    def test_publish_outputs_json_status_without_internal_fields(self):
        node = self.device._NetworkDdsHealthNode("/g1/state/network_dds_health", "g1", 15701)
        node._on_topic("battery", _String("{}"))
        node._publish()

        published = json.loads(node._pub.messages[-1].data)
        self.assertIn("checks", published)
        self.assertIn("metrics", published)
        self.assertIn("dds", published)
        self.assertIn("lidar", published)
        self.assertNotIn("camera_ok", published)
        self.assertNotIn("source_topic", published)
        self.assertNotIn("topic_out", published)


class NetworkDdsHealthWiringTests(unittest.TestCase):
    def test_config_enables_plugin(self):
        config_text = (G1_DIR / "config.yaml").read_text()
        self.assertIn("network_dds_health:", config_text)
        self.assertIn("publish_hz: 2.0", config_text)

    def test_bundle_loads_plugin_after_state_before_camera(self):
        main_text = (G1_DIR / "main.py").read_text()
        state_idx = main_text.index("StatePlugin loaded")
        health_idx = main_text.index("NetworkDdsHealthPlugin loaded")
        camera_idx = main_text.index("RealSensePlugin loaded")
        self.assertLess(state_idx, health_idx)
        self.assertLess(health_idx, camera_idx)


if __name__ == "__main__":
    unittest.main()
