from __future__ import annotations

import importlib.util
import json
import sys
import time
import types
import unittest
from pathlib import Path


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self, *args, **kwargs):
        self.publishers = []

    def create_publisher(self, *args):
        publisher = FakePublisher()
        self.publishers.append(publisher)
        return publisher

    def create_timer(self, *args):
        return object()

    def destroy_node(self):
        pass


def load_device():
    numpy = types.ModuleType("numpy")
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.node.Node = FakeNode
    rclpy.qos = types.ModuleType("rclpy.qos")
    rclpy.qos.QoSProfile = lambda **kwargs: kwargs
    rclpy.qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    rclpy.qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.msg.JointState = object
    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = types.ModuleType("std_msgs.msg")

    class String:
        data = ""

    std_msgs.msg.String = String
    old_modules = {name: sys.modules.get(name) for name in (
        "numpy", "rclpy", "rclpy.node", "rclpy.qos", "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg",
    )}
    sys.modules.update({
        "numpy": numpy,
        "rclpy": rclpy, "rclpy.node": rclpy.node, "rclpy.qos": rclpy.qos,
        "sensor_msgs": sensor_msgs, "sensor_msgs.msg": sensor_msgs.msg,
        "std_msgs": std_msgs, "std_msgs.msg": std_msgs.msg,
    })
    try:
        path = Path(__file__).parents[1] / "device.py"
        module = types.ModuleType("adam_device_test")
        module.__file__ = str(path)
        source = "from __future__ import annotations\n" + path.read_text(encoding="utf-8")
        exec(compile(source, str(path), "exec"), module.__dict__)
        return module
    finally:
        for name, old_module in old_modules.items():
            if old_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old_module


class FakeExecutor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, node):
        self.nodes.remove(node)


class FakeCache:
    def __init__(self, snapshot=None, reader_available=True):
        self.snapshot_value = snapshot
        self.reader_available = reader_available
        self.starts = 0
        self.stops = 0

    def snapshot(self, timeout_sec):
        return self.snapshot_value

    def status(self, timeout_sec):
        return {"reader_available": self.reader_available, "fresh": False, "last_sample_age_ms": None}

    def start(self):
        self.starts += 1
        return self.reader_available

    def stop(self):
        self.stops += 1


class HandStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = load_device()

    def test_payload_normalizes_short_and_malformed_positions(self):
        payload = self.device._hand_state_payload([100, "bad", 2000], 10, fresh=True)
        self.assertEqual(payload["position"], [100, 0, 1000] + [0] * 9)
        self.assertEqual(payload["left"]["channels"]["pinky"], 100)
        self.assertEqual(payload["right"]["position"], [0] * 6)

    def test_sensor_payload_has_tianyi_style_hands(self):
        snapshot = self.device._hand_state_payload(range(0, 1200, 100), 10, fresh=True)
        payload = self.device._hand_state_sensor_payload(snapshot)
        self.assertEqual(len(payload["hands"]["left"]["fingers"]), 6)
        self.assertEqual(payload["hands"]["right"]["fingers"][5]["name"], "thumb_rotate")
        self.assertEqual(payload["hands"]["left"]["fingers"][0]["position_label"], "fully_closed")

    def test_cache_reports_missing_and_stale_samples(self):
        cache = self.device.HandStateCache()
        self.assertIsNone(cache.snapshot(1.0))
        self.assertFalse(cache.status(1.0)["reader_available"])
        cache._latest_position = [1] * 12
        cache._received_at_ms = int(time.time() * 1000) - 2000
        cache._received_monotonic = time.monotonic() - 2
        self.assertFalse(cache.snapshot(0.1)["fresh"])

    def test_publisher_is_live_without_start_action(self):
        executor = FakeExecutor()
        snapshot = self.device._hand_state_payload([250] * 12, int(time.time() * 1000), fresh=True)
        plugin = self.device.HandStatePlugin({}, "adam", executor, state_cache=FakeCache(snapshot))
        plugin._node._publish()
        self.assertEqual(len(plugin._node._pub.messages), 1)
        self.assertEqual(plugin._node.metrics()["publish_count"], 1)

    def test_sensor_is_read_only_and_never_uses_command_writer(self):
        executor = FakeExecutor()
        snapshot = self.device._hand_state_payload([500] * 12, int(time.time() * 1000), fresh=True)
        cache = FakeCache(snapshot)
        plugin = self.device.HandStatePlugin({"publish_rate_hz": 30}, "adam", executor, state_cache=cache)
        tool = plugin.get_tool()
        self.assertEqual(tool["type"], "sensor")
        self.assertTrue(tool["readOnly"])
        self.assertFalse(tool["multiInstance"])
        self.assertEqual(tool["topic_out"][0], {"topic": "/adam/state/hand_state", "format": "data/json"})
        plugin.start()
        plugin._node._publish()
        published = json.loads(plugin._node._pub.messages[0].data)
        self.assertEqual(published["hands"]["left"]["fingers"][0]["position"], 500)
        self.assertEqual(cache.starts, 1)
        self.assertFalse(hasattr(plugin, "_hand_pub"))

    def test_sensor_reports_waiting_when_reader_has_no_sample(self):
        plugin = self.device.HandStatePlugin({}, "adam", FakeExecutor(), state_cache=FakeCache())
        self.assertEqual(plugin.dispatch("read", {})["state"], "waiting")
        unavailable = self.device.HandStatePlugin({}, "adam", FakeExecutor(), state_cache=FakeCache(reader_available=False))
        self.assertEqual(unavailable.dispatch("read", {})["state"], "unavailable")


if __name__ == "__main__":
    unittest.main()
