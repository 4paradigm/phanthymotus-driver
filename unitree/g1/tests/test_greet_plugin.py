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

    def test_invalid_led_rgb_config_falls_back_to_default(self):
        for bad in ([255], [0, 255], "green", [0, 255, 0, 1], [0, -1, 0], [0, 256, 0], [True, 0, 0]):
            p = G1.GreetPlugin({"led_rgb": bad}, "test", None, FakeArm(), FakeAudio(),
                               threading.Lock(), threading.Lock())
            self.assertEqual(p._led_rgb, (0, 255, 0), f"bad led_rgb={bad!r}")

    def test_valid_led_rgb_config_is_parsed(self):
        p = G1.GreetPlugin({"led_rgb": [12, 34, 56]}, "test", None, FakeArm(), FakeAudio(),
                           threading.Lock(), threading.Lock())
        self.assertEqual(p._led_rgb, (12, 34, 56))

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

    def test_greet_wave_and_speak_ids_are_unique(self):
        p = self.plugin()
        p._wait_or_cancelled = lambda duration: False
        ids = {}
        for name, action, args in (("greet", "greet", {"confirm": True}),
                                   ("wave", "wave", {"confirm": True}),
                                   ("speak", "speak", {"text": "hello"})):
            result = p.dispatch(action, args)
            ids[name] = result["action_id"]
            while len(self.notifications) < len(ids):
                threading.Event().wait(0.01)
        self.assertNotEqual(ids["greet"], ids["wave"])
        self.assertNotEqual(ids["greet"], ids["speak"])
        self.assertNotEqual(ids["wave"], ids["speak"])
        self.assertTrue(ids["greet"].startswith("g1_greet_"))
        self.assertTrue(ids["wave"].startswith("g1_wave_"))
        self.assertTrue(ids["speak"].startswith("g1_speak_"))

    def test_concurrent_dispatch_rejected_with_resource_busy(self):
        p = self.plugin()
        release = threading.Event()

        def hold_then_cancel(duration):
            release.wait(5.0)
            return True

        p._wait_or_cancelled = hold_then_cancel
        first = p.dispatch("greet", {"confirm": True})
        self.assertEqual(first["status"], "executing")
        # The slot is reserved synchronously in dispatch(), so a second physical
        # request is rejected immediately rather than queueing behind the first
        # (which would blow past its declared 60s x-completion timeout).
        for action, args in (("greet", {"confirm": True}), ("wave", {"confirm": True}), ("speak", {"text": "hi"})):
            result = p.dispatch(action, args)
            self.assertEqual(result["code"], "RESOURCE_BUSY", f"action={action}")
            self.assertNotIn("action_id", result)
        # led stays available (independent of the greet worker slot)
        self.assertEqual(p.dispatch("led", {"r": 0, "g": 1, "b": 2})["ret"], 0)
        release.set()  # let the first worker finish (cancelled) and release the slot

    def test_speak_reports_acp_completion_after_tts(self):
        audio = FakeAudio()
        p = self.plugin(audio=audio)
        result = p.dispatch("speak", {"text": "hello"})
        while not self.notifications:
            threading.Event().wait(0.01)
        self.assertEqual(result["status"], "executing")
        self.assertEqual(audio.calls, [("tts", "hello", 0)])
        self.assertEqual(self.notifications[-1][0], result["action_id"])
        self.assertEqual(self.notifications[-1][1], "completed")
        self.assertEqual(self.notifications[-1][2], {"ret": 0, "text": "hello"})
        self.assertEqual(self.notifications[-1][3], "greet")
    def test_wave_reports_acp_completion_after_duration(self):
        arm = FakeArm()
        p = self.plugin(arm=arm)
        waits = []
        p._wait_or_cancelled = lambda duration: waits.append(duration) or False
        result = p.dispatch("wave", {"confirm": True})
        while not self.notifications:
            threading.Event().wait(0.01)
        self.assertEqual(result["status"], "executing")
        self.assertEqual(arm.calls, [("wave", p._HIGH_WAVE_ACTION_ID)])
        self.assertEqual(waits, [p._HIGH_WAVE_DURATION_S])
        self.assertEqual(self.notifications[-1][0], result["action_id"])
        self.assertEqual(self.notifications[-1][1], "completed")
        self.assertEqual(self.notifications[-1][3], "greet")

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
