from __future__ import annotations

import importlib.util
import sys
import time
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "pndbotics/adam"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))


def load_device():
    if "numpy" not in sys.modules:
        numpy = types.ModuleType("numpy")
        numpy.float64 = float
        sys.modules["numpy"] = numpy
    spec = importlib.util.spec_from_file_location("adam_device_contract", DRIVER / "device.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


adam = load_device()


class FakeSubscriber:
    def __init__(self):
        self.closed = False

    def Close(self):
        self.closed = True


class AdamDriverContractTests(unittest.TestCase):
    def test_health_payload_reports_transport_and_control_topology(self):
        state = type("State", (), {
            "mode_pr": 0,
            "motor_state": [object()] * 31,
            "tick": 123,
        })()
        payload = adam._state_health_payload(state, time.monotonic())
        self.assertEqual("online", payload["status"])
        self.assertEqual("PR 串联关节控制", payload["control_topology"])
        self.assertEqual(31, payload["body_joint_count"])
        self.assertEqual(123, payload["tick"])

    def test_hand_cache_status_distinguishes_waiting_and_stale(self):
        cache = adam.HandStateCache(FakeSubscriber())
        waiting = adam._hand_status_payload(cache, 1.0)
        self.assertEqual("waiting", waiting["state"])
        self.assertFalse(waiting["fresh"])

        with cache._lock:
            cache._latest_position = [100] * 12
            cache._received_at_ms = int(time.time() * 1000) - 2000
            cache._received_monotonic = time.monotonic() - 2
        stale = cache.snapshot(timeout_sec=1.0)
        self.assertIsNotNone(stale)
        self.assertFalse(stale["fresh"])
        self.assertEqual(12, len(stale["position"]))
        cache.close()

    def test_hand_state_payload_exposes_left_and_right_channels(self):
        payload = adam._hand_state_payload(
            list(range(12)), int(time.time() * 1000), fresh=True)
        self.assertEqual([0, 1, 2, 3, 4, 5], payload["left"]["position"])
        self.assertEqual([6, 7, 8, 9, 10, 11], payload["right"]["position"])
        self.assertEqual(1000, payload["position_max"])
        self.assertEqual(6, payload["left"]["motor_channel_count"])
        self.assertEqual(5, payload["right"]["finger_count"])

    def test_state_plugin_tools_include_health_and_hands(self):
        node = type("Node", (), {
            "_topic_skeleton": "/adam/state/joints",
            "_topic_imu": "/adam/state/imu",
            "_topic_battery": "/adam/state/battery",
            "_topic_health": "/adam/state/health",
            "_topic_hand": "/adam/state/hands",
        })()
        plugin = object.__new__(adam.StatePlugin)
        plugin._node = node
        tools = {tool["name"]: tool for tool in plugin.get_tools()}
        self.assertIn("health", tools)
        self.assertIn("hands", tools)
        self.assertEqual(
            "/adam/state/health", tools["health"]["topic_out"][0]["topic"])
        self.assertEqual(
            "/adam/state/hands", tools["hands"]["topic_out"][0]["topic"])


if __name__ == "__main__":
    unittest.main()
