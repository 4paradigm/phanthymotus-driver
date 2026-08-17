import importlib.util
import sys
import threading
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load_voice_gesture():
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.String = type("String", (), {})
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos
    sys.modules["std_msgs.msg"] = std_msgs

    spec = importlib.util.spec_from_file_location(
        "t800_voice_gesture_contract", ROOT / "voice_gesture.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeUnavailableClient:
    def __init__(self):
        self.wait_timeout = None

    def wait_for_service(self, timeout_sec):
        self.wait_timeout = timeout_sec
        return False


class VoiceGestureMotorEnableTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_voice_gesture()

    def make_plugin(self, *, required=False):
        plugin = self.module.VoiceGesturePlugin.__new__(self.module.VoiceGesturePlugin)
        plugin._auto_enable_motors = True
        plugin._motor_enable_required = required
        plugin._motor_enable_discovery_timeout = 0.25
        plugin._motor_enable_timeout = 1.0
        plugin._motor_enable_service = "/hardware/enable_motor"
        plugin._motor_enable_client = FakeUnavailableClient()
        plugin._motor_enable_type = types.SimpleNamespace(Request=type("Request", (), {}))
        plugin._lock = threading.RLock()
        plugin._status = {"last_motor_enable_error": ""}
        plugin.events = []
        plugin._publish_event = lambda event, **fields: plugin.events.append((event, fields))
        return plugin

    def test_missing_optional_service_continues_to_planner(self):
        plugin = self.make_plugin(required=False)

        self.assertTrue(plugin._ensure_motors_enabled("shake_hand", "握手"))
        self.assertEqual(0.25, plugin._motor_enable_client.wait_timeout)
        self.assertEqual("motor_enable_degraded", plugin.events[-1][0])
        self.assertTrue(plugin.events[-1][1]["continuing"])
        self.assertIn("unavailable", plugin._status["last_motor_enable_error"])

    def test_missing_required_service_remains_fail_closed(self):
        plugin = self.make_plugin(required=True)

        self.assertFalse(plugin._ensure_motors_enabled("shake_hand", "握手"))
        self.assertEqual("motor_enable_error", plugin.events[-1][0])
        self.assertFalse(plugin.events[-1][1]["continuing"])

    def test_asr_starts_motion_after_optional_service_degrades(self):
        plugin = self.make_plugin(required=False)
        plugin._require_wake_word = True
        plugin._enabled = True
        plugin._motions = {
            "shake_hand": {"phrases": ["握手"], "motion_plan": []},
        }
        plugin._led_wake_mode = "blink_green"
        plugin._flash_led = lambda *_args: None
        started = []
        plugin._start_motion = lambda motion_id, text: started.append((motion_id, text))

        plugin._on_asr(types.SimpleNamespace(
            data='{"text":"小范小范握手。","kws_triggered":true}'
        ))

        self.assertEqual([("shake_hand", "小范小范握手。")], started)
        self.assertIn("motor_enable_degraded", [event for event, _fields in plugin.events])

    def test_disabled_automatic_enable_skips_service(self):
        plugin = self.make_plugin(required=True)
        plugin._auto_enable_motors = False

        self.assertTrue(plugin._ensure_motors_enabled("shake_hand", "握手"))
        self.assertEqual("motor_enable_skipped", plugin.events[-1][0])
        self.assertEqual("", plugin._status["last_motor_enable_error"])


class VoiceGestureDeploymentConfigTests(unittest.TestCase):
    def test_real_device_config_is_complete_and_example_is_retained(self):
        deployment = (ROOT / "config.voice-gesture.yaml").read_text(encoding="utf-8")
        example = (ROOT / "voice_gesture.example.yaml").read_text(encoding="utf-8")
        base = (ROOT / "config.yaml").read_text(encoding="utf-8")

        self.assertIn('ros_namespace: "ubuntu"', deployment)
        self.assertIn('asr_topic: "/ubuntu/mic/audio/asr"', deployment)
        self.assertIn('events_topic: "/ubuntu/voice_gesture/events"', deployment)
        self.assertIn("motor_enable_required: false", deployment)
        self.assertIn("voice_gesture:", example)
        self.assertNotIn("voice_gesture:", base)

    def test_extension_image_contains_real_device_config(self):
        dockerfile = (ROOT / "Dockerfile.voice-gesture").read_text(encoding="utf-8")
        self.assertIn("config.voice-gesture.yaml /work/", dockerfile)


if __name__ == "__main__":
    unittest.main()
