import importlib.util
import subprocess
import sys
import types
from pathlib import Path



def _load_device(monkeypatch):
    class Node:
        def __init__(self, name):
            self.subscriptions = []

        def create_subscription(self, message_type, topic, callback, qos):
            self.subscriptions.append((topic, callback))

    class String:
        def __init__(self, data=""):
            self.data = data

    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: object()
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    modules = {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": types.ModuleType("rclpy.node"),
        "rclpy.qos": qos,
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
        "audio_msgs": types.ModuleType("audio_msgs"),
        "audio_msgs.msg": types.ModuleType("audio_msgs.msg"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
    }
    modules["rclpy.node"].Node = Node
    modules["std_msgs.msg"].String = String
    class AudioChunk:
        def __init__(self):
            self.header = types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=0, nanosec=0))

    modules["audio_msgs.msg"].AudioChunk = AudioChunk
    modules["sensor_msgs.msg"].CompressedImage = type("CompressedImage", (), {})
    modules["sensor_msgs.msg"].Image = type("Image", (), {})
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "device.py"
    spec = importlib.util.spec_from_file_location("bumi_mic_card_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, String


def test_mic_stop_reaps_subprocess_before_restart(monkeypatch):
    module, _ = _load_device(monkeypatch)
    executor = types.SimpleNamespace(add_node=lambda node: None)
    media = types.SimpleNamespace(get_wakeup_words=lambda: "小范小范")
    card = module.MicPlugin({}, "robot", executor, media)
    events = []

    class Process:
        stdout = ()

        def poll(self):
            return None if "reaped" not in events else 0

        def terminate(self):
            events.append("terminate")

        def wait(self, timeout=None):
            events.append("wait")
            if "kill" not in events:
                raise subprocess.TimeoutExpired("mic", timeout)
            events.append("reaped")

        def kill(self):
            events.append("kill")

    card._proc = Process()
    monkeypatch.setattr(module.subprocess, "Popen", lambda *args, **kwargs: (
        events.append("spawn") or types.SimpleNamespace(stdout=(), poll=lambda: None)))

    assert card.dispatch("stop", {}) == {"state": "idle"}
    assert events == ["terminate", "wait", "kill", "wait", "reaped"]
    card.dispatch("start", {})
    assert events[-1] == "spawn"

def test_mic_stop_does_not_claim_idle_if_killed_process_is_still_running(monkeypatch):
    module, _ = _load_device(monkeypatch)
    card = module.MicPlugin({}, "robot", types.SimpleNamespace(add_node=lambda node: None),
                            object())

    class Process:
        def poll(self):
            return None

        def terminate(self):
            pass

        def kill(self):
            pass

        def wait(self, timeout=None):
            raise subprocess.TimeoutExpired("mic", timeout)

    process = Process()
    card._proc = process
    assert card.dispatch("stop", {}) == {
        "state": "error", "message": "mic subprocess did not exit after kill"}
    assert card._proc is process

def test_audio_chunk_timestamp_uses_first_buffered_frame(monkeypatch):
    module, String = _load_device(monkeypatch)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    published = {}

    class Node:
        def __init__(self, name):
            pass

        def create_publisher(self, message_type, topic, qos):
            published[topic] = []
            return types.SimpleNamespace(publish=published[topic].append)

    class Done(BaseException):
        pass

    times = iter((1_700_000_000_000_000, 1_700_000_000_020_000))

    def capture():
        try:
            timestamp_us = next(times)
        except StopIteration:
            raise Done()
        return types.SimpleNamespace(timestamp_us=timestamp_us,
                                     channels=8, sample_rate=16000,
                                     audio_data=[10] * (320 * 8))

    monkeypatch.setattr(sys.modules["rclpy"], "init", lambda: None, raising=False)
    monkeypatch.setattr(sys.modules["rclpy.node"], "Node", Node)
    monkeypatch.setitem(sys.modules, "common", types.SimpleNamespace(
        logsafe=types.SimpleNamespace(install=lambda **kwargs: None)))
    monkeypatch.setitem(sys.modules, "mediacontrol_py", types.SimpleNamespace(
        MediaController=types.SimpleNamespace(instance=lambda: types.SimpleNamespace(
            init=lambda: None, get_audio_capture_data=capture))))
    monkeypatch.setattr(sys.modules["std_msgs.msg"], "String", String)
    monkeypatch.setattr(module.time, "sleep", lambda seconds: None)

    try:
        module._mic_subprocess("robot")
    except Done:
        pass

    audio = published["/robot/mic/audio"]
    assert len(audio) == 1
    stamp = audio[0].header.stamp
    assert stamp.sec * 1_000_000 + stamp.nanosec // 1000 == 1_700_000_000_000_000
