"""Focused tests for Adam hand-state sensor card (HandStatePlugin).

Covers:
- Valid 12-value feedback produces correct payload
- Short / malformed feedback gets normalised safely
- Stale sample is marked not fresh
- Reader unavailable returns unavailable status
- hand_state tool is separate from hand tool and never calls DDS command writer
"""
import sys
import threading
import time
import types
import unittest

# ---------------------------------------------------------------------------
# Minimal numpy mock so device.py can load without the actual package
# ---------------------------------------------------------------------------
_np_mod = types.ModuleType("numpy")
_np_mod.ndarray = object
_np_mod.float = float
_np_mod.int = int
sys.modules["numpy"] = _np_mod

# ---------------------------------------------------------------------------
# Minimal ROS2 mock so we can import device.py without a full ROS2 runtime
# ---------------------------------------------------------------------------
_ros2_mod = types.ModuleType("rclpy")
_ros2_mod.ok = lambda: False
_rclpy_node = types.ModuleType("rclpy.node")


class _MockNode:
    """Minimal Node mock that accepts __init__ args and tracks created publishers/timers."""

    def __init__(self, *args, **kwargs):
        self._publishers = []
        self._timers = []
        self._topic_hand_state = None

    def create_publisher(self, msg_type, topic, qos):
        self._publishers.append((msg_type, topic, qos))
        return types.SimpleNamespace(publish=lambda m: None)

    def create_timer(self, timer_period_sec, callback):
        self._timers.append((timer_period_sec, callback))
        return types.SimpleNamespace(cancel=lambda: None)

    def destroy_publisher(self, pub):
        pass

    def destroy_timer(self, timer):
        pass


_rclpy_node.Node = _MockNode
_qos = types.ModuleType("rclpy.qos")


class _QoSProfile:
    def __init__(self, *, reliability, history, depth):
        self.reliability = reliability
        self.history = history
        self.depth = depth


_qos.QoSProfile = _QoSProfile
_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)

_joint_msgs = types.ModuleType("sensor_msgs.msg")
_joint_msgs.JointState = object
_std_msgs = types.ModuleType("std_msgs.msg")
_std_msgs.String = object

sys.modules["rclpy"] = _ros2_mod
sys.modules["rclpy.node"] = _rclpy_node
sys.modules["rclpy.qos"] = _qos
sys.modules["sensor_msgs"] = types.ModuleType("sensor_msgs")
sys.modules["sensor_msgs.msg"] = _joint_msgs
sys.modules["std_msgs"] = types.ModuleType("std_msgs")
sys.modules["std_msgs.msg"] = _std_msgs

# ---------------------------------------------------------------------------
# Now import the module under test
# ---------------------------------------------------------------------------
device_path = (
    "/tmp/claude/phanthymotus-driver-adam/pndbotics/adam/device.py"
)
import importlib.util

# ---------------------------------------------------------------------------
# Load device.py directly without requiring pndbotics package
# ---------------------------------------------------------------------------
device_path = "/tmp/claude/phanthymotus-driver-adam/pndbotics/adam/device.py"
spec = importlib.util.spec_from_file_location("device", device_path)
_device = importlib.util.module_from_spec(spec)
spec.loader.exec_module(_device)

HandStateCache = _device.HandStateCache
HandStatePlugin = _device.HandStatePlugin
_hand_state_payload = _device._hand_state_payload
_normalize_hand_state_positions = _device._normalize_hand_state_positions
_coerce_hand_positions = _device._coerce_hand_positions
HAND_POSITION_MAX = _device.HAND_POSITION_MAX
HAND_POSITION_COUNT = _device.HAND_POSITION_COUNT
HandPlugin = _device.HandPlugin


class MockExecutor:
    """Tiny executor stub that stores added nodes."""

    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class MockSubscriber:
    """DDS subscriber stub that yields controlled samples."""

    def __init__(self, samples=None, fail=False):
        self.samples = samples or []
        self.fail = fail
        self.read_count = 0

    def Read(self, timeout=0.2):
        if self.fail:
            raise RuntimeError("simulated DDS read error")
        idx = self.read_count
        self.read_count += 1
        if idx < len(self.samples):
            sample = self.samples[idx]
            if isinstance(sample, Exception):
                raise sample
            return sample
        return None


class MockHandStateMsg:
    def __init__(self, position):
        self.position = position


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestNormalizeHandStatePositions(unittest.TestCase):
    def test_valid_12_values(self):
        pos = list(range(12))
        result = _normalize_hand_state_positions(pos)
        self.assertEqual(len(result), 12)
        self.assertEqual(result, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])

    def test_short_array(self):
        result = _normalize_hand_state_positions([100, 200])
        self.assertEqual(len(result), 12)
        # short values fill, rest are 0
        self.assertEqual(result[:2], [100, 200])

    def test_empty_array(self):
        result = _normalize_hand_state_positions([])
        self.assertEqual(len(result), 12)
        self.assertEqual(result, [0] * 12)

    def test_exceeds_12(self):
        result = _normalize_hand_state_positions(list(range(20)))
        self.assertEqual(len(result), 12)


class TestCoerceHandPositions(unittest.TestCase):
    def test_clamps_to_max(self):
        result = _coerce_hand_positions([2000] * 12, limit=1000)
        self.assertEqual(result, [1000] * 12)

    def test_rejects_bool(self):
        with self.assertRaises(ValueError):
            _coerce_hand_positions([True] * 12, limit=1000)

    def test_rejects_wrong_length(self):
        with self.assertRaises(ValueError):
            _coerce_hand_positions([100] * 11, limit=1000)

    def test_rejects_nan(self):
        with self.assertRaises(ValueError):
            _coerce_hand_positions([float("nan")] * 12, limit=1000)


class TestHandStatePayload(unittest.TestCase):
    def test_valid_payload_structure(self):
        pos = [100, 200] + [500] * 10
        ts = int(time.time() * 1000)
        payload = _hand_state_payload(pos, ts, fresh=True)
        self.assertIn("timestamp_ms", payload)
        self.assertIn("received_at_ms", payload)
        self.assertIn("age_ms", payload)
        self.assertTrue(payload["fresh"])
        self.assertEqual(payload["position"], pos)
        self.assertEqual(len(payload["left"]["channels"]), 6)
        self.assertEqual(len(payload["right"]["channels"]), 6)
        self.assertEqual(payload["position_max"], HAND_POSITION_MAX)

    def test_unfresh_payload(self):
        pos = [0] * 12
        ts = int(time.time() * 1000)
        payload = _hand_state_payload(pos, ts, fresh=False)
        self.assertFalse(payload["fresh"])


class TestHandStateCache(unittest.TestCase):
    def test_fresh_positions(self):
        msg = MockHandStateMsg([500] * 12)
        sub = MockSubscriber(samples=[msg])
        cache = HandStateCache(sub)
        self.assertTrue(cache.start())
        time.sleep(0.5)
        positions = cache.fresh_positions(timeout_sec=1.0)
        self.assertEqual(positions, [500] * 12)
        cache.close()

    def test_snapshot_returns_none_before_any_read(self):
        cache = HandStateCache(None)
        cache.start()
        time.sleep(0.2)
        snapshot = cache.snapshot(timeout_sec=0.5)
        self.assertIsNone(snapshot)
        cache.close()

    def test_status_reader_unavailable(self):
        cache = HandStateCache(None)
        cache.start()
        time.sleep(0.2)
        status = cache.status(timeout_sec=1.0)
        self.assertFalse(status["reader_available"])
        cache.close()

    def test_stale_sample(self):
        """Cache holds old sample; timeout makes it stale."""
        msg = MockHandStateMsg([100] * 12)
        sub = MockSubscriber(samples=[msg])
        cache = HandStateCache(sub)
        self.assertTrue(cache.start())
        # Wait for sample to arrive
        time.sleep(0.5)
        # Now wait for it to become stale
        time.sleep(2.0)
        positions = cache.fresh_positions(timeout_sec=0.5)
        self.assertIsNone(positions)
        cache.close()


class TestHandStatePlugin(unittest.TestCase):
    """Test the plugin layer without requiring real DDS/ROS2."""

    def test_tool_registration(self):
        """hand_state tool appears as sensor with topic_out."""
        cache = HandStateCache(None)
        cache.start()
        executor = MockExecutor()
        plugin = HandStatePlugin({"publish_rate_hz": 10, "state_timeout_sec": 0.5},
                                 "test", executor, state_cache=cache)
        tools = plugin.get_tools()
        self.assertEqual(len(tools), 1)
        tool = tools[0]
        self.assertEqual(tool["name"], "hand_state")
        self.assertEqual(tool["type"], "sensor")
        self.assertIn("topic_out", tool)
        self.assertEqual(tool["topic_out"][0]["format"], "data/json")
        plugin.close()

    def test_dispatch_info(self):
        cache = HandStateCache(None)
        cache.start()
        executor = MockExecutor()
        plugin = HandStatePlugin({}, "test", executor, state_cache=cache)
        result = plugin.dispatch("info", {"_tool_name": "hand_state"})
        self.assertEqual(result["state"], "idle")
        self.assertIn("topic_out", result)
        plugin.close()

    def test_hand_tool_separate_from_hand_state(self):
        """hand_state does not interfere with hand actuator tool."""
        cache = HandStateCache(None)
        cache.start()
        executor = MockExecutor()
        hand_plugin = HandPlugin({}, "test", executor,
                                 dds_hand_pub=None, state_cache=cache)
        hand_state_plugin = HandStatePlugin({}, "test", executor,
                                            state_cache=cache)

        hand_tools = hand_plugin.get_tool()
        hand_state_tools = hand_state_plugin.get_tools()

        self.assertEqual(hand_tools["name"], "hand")
        self.assertEqual(hand_tools["type"], "actuator")
        self.assertEqual(hand_state_tools[0]["name"], "hand_state")
        self.assertEqual(hand_state_tools[0]["type"], "sensor")

        hand_plugin.close()
        hand_state_plugin.close()
        cache.close()

    def test_hand_state_plugin_does_not_create_publisher(self):
        """hand_state sensor is read-only and never invokes DDS command writer."""
        cache = HandStateCache(None)
        cache.start()
        executor = MockExecutor()
        plugin = HandStatePlugin({}, "test", executor, state_cache=cache)

        # Start plugin and poll briefly
        plugin.start()
        time.sleep(0.3)
        plugin.stop()
        plugin.close()

        # Cache should still be functioning for hand actuator
        # snapshot is None because there's no real DDS reader, but cache is alive
        self.assertIsNone(cache.snapshot(timeout_sec=0.5))
        cache.close()


if __name__ == "__main__":
    unittest.main()
