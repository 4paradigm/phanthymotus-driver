import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class _Node:
    def __init__(self, *_args, **_kwargs):
        pass

    def create_publisher(self, *_args, **_kwargs):
        self.publisher = _Publisher()
        return self.publisher

    def create_subscription(self, *_args, **_kwargs):
        return object()


class _String:
    def __init__(self):
        self.data = ""


def _load_device():
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.node.Node = _Node
    rclpy.qos = types.ModuleType("rclpy.qos")
    rclpy.qos.QoSProfile = lambda **kwargs: kwargs
    rclpy.qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    rclpy.qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy.qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1, TRANSIENT_LOCAL=2)
    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = types.ModuleType("std_msgs.msg")
    std_msgs.msg.String = _String
    std_msgs.msg.Bool = type("Bool", (), {})
    sys.modules.update({
        "rclpy": rclpy,
        "rclpy.node": rclpy.node,
        "rclpy.qos": rclpy.qos,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs.msg,
    })
    spec = importlib.util.spec_from_file_location(
        "tianyi2_device", Path(__file__).with_name("device.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Ros2:
    def __init__(self):
        self.ctx_core = object()
        self.ctx_tianyi = object()
        self.executor_core = types.SimpleNamespace(add_node=lambda _node: None)
        self.executor_tianyi = types.SimpleNamespace(add_node=lambda _node: None)


class _VoiceMessage:
    def __init__(self, payload):
        self.data = json.dumps(payload)


class _Vector:
    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = x, y, z


class _WrenchMessage:
    def __init__(self, force, torque=None):
        self.wrench = types.SimpleNamespace(
            force=_Vector(*force), torque=_Vector(*(torque or (0, 0, 0))))


class SensorPluginTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = _load_device()

    def test_voice_contract_and_deduplication(self):
        plugin = self.device.AsrPlugin({}, "robot", _Ros2())
        plugin.start()
        payload = {
            "asr": {"event": 4, "keyword": "嗨天轶", "iat_id": "id-1", "iat_text": "你好"},
            "timestamp_ms": 123,
        }
        plugin._on_state_voice(_VoiceMessage(payload))
        plugin._on_state_voice(_VoiceMessage(payload))
        self.assertEqual(len(plugin._pub.messages), 1)
        output = json.loads(plugin._pub.messages[0])
        self.assertEqual(output["iat_id"], "id-1")
        self.assertEqual(output["event_name"], "EVENT_WAKEUP")
        self.assertEqual(plugin.dispatch("read", {})["iat_text"], "你好")

        state = {"asr": {"event": 3, "iat_text": ""}, "timestamp_ms": 124}
        plugin._on_state_voice(_VoiceMessage(state))
        self.assertEqual(len(plugin._pub.messages), 1)

    def test_voice_payload_event_names_take_precedence(self):
        plugin = self.device.AsrPlugin({}, "robot", _Ros2())
        plugin.start()
        plugin._on_state_voice(_VoiceMessage({
            "asr": {"event": 4, "event_name": "wakeup", "event_label": "唤醒", "iat_text": ""},
            "timestamp_ms": 125,
        }))
        output = json.loads(plugin._pub.messages[0])
        self.assertEqual(output["event_name"], "wakeup")
        self.assertEqual(output["event_label"], "唤醒")

    def test_force_baseline_contact_release_and_direction(self):
        plugin = self.device.ForceTorqueStatePlugin({}, "robot", _Ros2())
        plugin.start()
        neutral = _WrenchMessage((0, 0, 0))
        for _ in range(plugin._BASELINE_FRAMES):
            plugin._on_force("left", neutral)
        self.assertTrue(plugin._baseline_ready["left"])
        plugin._last_event_time["left"] = 0
        plugin._on_force("left", _WrenchMessage((4, 0, 0)))
        self.assertEqual(len(plugin._pub.messages), 1)
        contact = json.loads(plugin._pub.messages[0])
        self.assertEqual(contact["event"], "contact_left")
        self.assertEqual(contact["direction"], "push_forward")

        plugin._last_event_time["left"] = 0
        plugin._on_force("left", neutral)
        release = json.loads(plugin._pub.messages[-1])
        self.assertEqual(release["event"], "release_left")
        self.assertEqual(release["direction"], "release")

    def test_force_direction_mapping(self):
        plugin = object.__new__(self.device.ForceTorqueStatePlugin)
        expected = {
            (1, 0, 0): "push_forward", (-1, 0, 0): "push_back",
            (0, 1, 0): "push_left", (0, -1, 0): "push_right",
            (0, 0, 1): "push_up", (0, 0, -1): "push_down",
        }
        for vector, direction in expected.items():
            self.assertEqual(plugin._direction(vector), direction)


if __name__ == "__main__":
    unittest.main()
