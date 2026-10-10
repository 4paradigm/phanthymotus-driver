import importlib.util
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


def test_mic_publishes_mono_audio_from_sdk_frames(monkeypatch):
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
    assert audio[0].format == "pcm_16k_16bit_mono"
    assert audio[0].data == (500).to_bytes(2, "little", signed=True) * 640
