import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_TEST = ROOT / "tests" / "test_device_contract.py"


def load_voice_control():
    contract_spec = importlib.util.spec_from_file_location("t800_contract_stubs", CONTRACT_TEST)
    contract = importlib.util.module_from_spec(contract_spec)
    contract_spec.loader.exec_module(contract)
    contract.install_ros_stubs()
    spec = importlib.util.spec_from_file_location("t800_voice_control", ROOT / "voice_control.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, contract


class FakeGesture:
    def __init__(self):
        self.calls = []

    def dispatch(self, action, arguments):
        self.calls.append((action, arguments))
        return {"state": "started", "action": action}


class VoiceControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module, cls.contract = load_voice_control()

    def make_plugin(self):
        self.gesture = FakeGesture()
        config = {
            "plugins": {
                "voice_control": {
                    "require_wake_word": True,
                    "cooldown_sec": 0,
                    "actions": {
                        "wave": {
                            "description": "挥手",
                            "phrases": ["你好", "您好"],
                            "target": {
                                "tool": "gesture",
                                "action": "play",
                                "arguments": {"name": "wave_hands"},
                            },
                        },
                        "shake_hand": {
                            "description": "握手",
                            "phrases": ["握手"],
                            "target": {
                                "tool": "gesture",
                                "action": "play",
                                "arguments": {"name": "shake_hand"},
                            },
                        },
                    },
                }
            }
        }
        plugin = self.module.VoiceControlPlugin(
            config, "robot", self.contract.FakeRos(), {"gesture": self.gesture}
        )
        plugin.start()
        return plugin

    @staticmethod
    def message(payload):
        return types.SimpleNamespace(data=json.dumps(payload, ensure_ascii=False))

    def test_rule_routes_final_wake_word_result_to_whitelisted_gesture(self):
        plugin = self.make_plugin()
        plugin._on_asr(self.message({"text": "你好", "kws_triggered": True}))
        self.assertEqual([("play", {"name": "wave_hands"})], self.gesture.calls)
        event = json.loads(plugin._events_pub.messages[-1].data)
        self.assertEqual("action_dispatched", event["event"])
        self.assertEqual("wave", event["action_id"])

    def test_rule_requires_wake_word(self):
        plugin = self.make_plugin()
        plugin._on_asr(self.message({"text": "握手", "kws_triggered": False}))
        self.assertEqual([], self.gesture.calls)
        event = json.loads(plugin._events_pub.messages[-1].data)
        self.assertEqual("wake_word_required", event["reason"])

if __name__ == "__main__":
    unittest.main()
