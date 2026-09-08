import importlib.util
import sys
import threading
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class Message:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")


class FakeNode:
    def __init__(self, name):
        self._name = name

    def create_publisher(self, *a, **kw):
        return types.SimpleNamespace(publish=lambda msg: None)

    def create_subscription(self, *a, **kw):
        return object()

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None,
                                     error=lambda *a: None, debug=lambda *a: None)


def install_stubs():
    rclpy = types.ModuleType("rclpy")
    sys.modules.setdefault("rclpy", rclpy)
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos

    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.Header = type("Header", (Message,), {})
    std_msgs.String = type("String", (Message,),
                           {"__init__": lambda self: setattr(self, "data", "")})
    std_msgs.UInt8MultiArray = type("UInt8MultiArray", (Message,), {})
    sys.modules["std_msgs.msg"] = std_msgs

    audio_msgs = types.ModuleType("audio_msgs.msg")
    audio_msgs.AudioChunk = type("AudioChunk", (Message,), {})
    sys.modules["audio_msgs.msg"] = audio_msgs

    for name in ("unitree_sdk2py", "unitree_sdk2py.g1", "unitree_sdk2py.g1.audio"):
        sys.modules.setdefault(name, types.ModuleType(name))
    audio_mod = types.ModuleType("unitree_sdk2py.g1.audio.g1_audio_client")
    audio_mod.AudioClient = type("AudioClient", (), {})
    sys.modules["unitree_sdk2py.g1.audio.g1_audio_client"] = audio_mod

    pcu = types.ModuleType("pointcloud_utils")
    pcu.gravity_align_inplace = lambda *a, **kw: None
    sys.modules["pointcloud_utils"] = pcu

    sms = types.ModuleType("sport_mode_state")
    sms.FSM_MODES = {}
    sms.LOCO_STATES = set()
    sms.LIMP_STATES = set()
    sms.BALANCED_SQUAT = 706
    sms.LIE_TO_STAND = 702
    sms.fsm_name = lambda fsm_id: str(fsm_id)
    sms.fsm_describe = lambda fsm_id: str(fsm_id)
    sys.modules["sport_mode_state"] = sms

    np = types.ModuleType("numpy")
    np.float32 = float
    np.zeros = lambda shape, dtype=None: [[0.0, 0.0, 0.0] for _ in range(shape[0])]
    sys.modules["numpy"] = np


def load_device():
    install_stubs()
    spec = importlib.util.spec_from_file_location("g1_device_greet_test", ROOT / "device.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


G1 = load_device()


class FakeAudio:
    def __init__(self, led_ret=0, tts_ret=0):
        self.led_ret = led_ret
        self.tts_ret = tts_ret
        self.calls = []

    def LedControl(self, r, g, b):
        self.calls.append(("led", r, g, b))
        return self.led_ret

    def TtsMaker(self, text, voice):
        self.calls.append(("tts", text, voice))
        return self.tts_ret


class FakeArm:
    def __init__(self, ret=0):
        self.ret = ret
        self.calls = []

    def ExecuteAction(self, action_id):
        self.calls.append(("wave", action_id))
        return self.ret


class GreetPluginTest(unittest.TestCase):
    def setUp(self):
        self.notifications = []
        self.real_notify = G1._loco_acp_notify
        G1._loco_acp_notify = lambda action_id, status, result, tool="loco": self.notifications.append(
            (action_id, status, result, tool)
        )
        self.addCleanup(lambda: setattr(G1, "_loco_acp_notify", self.real_notify))
        self.real_duration = G1._onboard_tts_duration_s
        G1._onboard_tts_duration_s = lambda text: 0
        self.addCleanup(lambda: setattr(G1, "_onboard_tts_duration_s", self.real_duration))

    def plugin(self, audio=None, arm=None):
        return G1.GreetPlugin({}, "test", None, arm or FakeArm(), audio or FakeAudio(), threading.Lock(), threading.Lock())

    def test_requires_confirmation_for_physical_actions(self):
        p = self.plugin()
        self.assertEqual(p.dispatch("greet", {}), {"error": "greet requires confirm=true", "code": "PRECONDITION_FAILED"})
        self.assertEqual(p.dispatch("wave", {}), {"error": "wave requires confirm=true", "code": "PRECONDITION_FAILED"})

    def test_rejects_invalid_rgb_before_hardware_call(self):
        audio = FakeAudio()
        p = self.plugin(audio=audio)
        for args in ({"r": -1}, {"g": 256}, {"b": True}, {"r": "255"}):
            result = p.dispatch("led", args)
            self.assertEqual(result["code"], "INVALID_ARGUMENT")
        self.assertEqual(audio.calls, [])

    def test_greet_ids_are_unique(self):
        p = self.plugin()
        p._wait_or_cancelled = lambda duration: False
        first = p.dispatch("greet", {"confirm": True})
        second = p.dispatch("greet", {"confirm": True})
        self.assertNotEqual(first["action_id"], second["action_id"])
        while len(self.notifications) < 2:
            threading.Event().wait(0.01)

    def test_failure_paths_report_acp_error(self):
        cases = [
            (FakeAudio(led_ret=7), FakeArm(), "LedControl failed: code=7"),
            (FakeAudio(tts_ret=8), FakeArm(), "TtsMaker failed: code=8"),
            (FakeAudio(), FakeArm(ret=9), "high wave failed: code=9"),
        ]
        for audio, arm, message in cases:
            self.notifications.clear()
            p = self.plugin(audio=audio, arm=arm)
            p._wait_or_cancelled = lambda duration: False
            p._run_greet("action", "hello")
            self.assertEqual(self.notifications[-1][1], "error")
            self.assertIn(message, self.notifications[-1][2]["error"])

    def test_stop_cancels_before_later_hardware_commands(self):
        audio = FakeAudio()
        arm = FakeArm()
        p = self.plugin(audio=audio, arm=arm)
        original_tts = G1._onboard_tts

        def stop_after_tts(*args):
            ret = original_tts(*args)
            p.stop()
            return ret

        G1._onboard_tts = stop_after_tts
        self.addCleanup(lambda: setattr(G1, "_onboard_tts", original_tts))
        p._run_greet("action", "hello")
        self.assertEqual(audio.calls, [("led", 0, 255, 0), ("tts", "hello", 0)])
        self.assertEqual(arm.calls, [])
        self.assertEqual(self.notifications[-1][1], "cancelled")

    def test_stop_during_wave_wait_reports_cancelled_not_completed(self):
        p = self.plugin()
        p._wait_or_cancelled = lambda duration: True
        p._run_greet("action", "hello")
        self.assertEqual(self.notifications[-1][1], "cancelled")


if __name__ == "__main__":
    unittest.main()
