import importlib.util
import json
import sys
import threading
import types
from concurrent.futures import ThreadPoolExecutor
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
    modules["audio_msgs.msg"].AudioChunk = type("AudioChunk", (), {})
    modules["sensor_msgs.msg"].CompressedImage = type("CompressedImage", (), {})
    modules["sensor_msgs.msg"].Image = type("Image", (), {})
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "device.py"
    spec = importlib.util.spec_from_file_location("bumi_mic_card_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, String


def test_mic_card_reports_direction_only_while_fresh_and_clears_on_stop(
    monkeypatch, tmp_path
):
    module, String = _load_device(monkeypatch)
    monkeypatch.setattr(module, "_MIC_DIRECTION_CALIBRATION", tmp_path / "calibration.json")

    class Process:
        stdout = ()
        running = True

        def poll(self):
            return None if self.running else 0

        def terminate(self):
            self.running = False

    processes = []

    def popen(*args, **kwargs):
        process = Process()
        processes.append(process)
        return process

    monkeypatch.setattr(module.subprocess, "Popen", popen)
    executor = types.SimpleNamespace(add_node=lambda node: None)
    added_words = []
    media = types.SimpleNamespace(
        get_wakeup_words=lambda: "",
        add_wakeup_words=lambda words: added_words.append(words) or True,
    )
    card = module.MicPlugin({}, "robot", executor, media)

    assert card.dispatch("start", {})["state"] == "running"
    assert card.dispatch("start", {})["state"] == "running"
    assert added_words == [module._MIC_WAKEUP_WORD]
    assert card.dispatch("add_wakeup_word", {})["state"] == "configured"
    assert len(added_words) == 1
    assert len(processes) == 1
    assert len(card.get_tool()["topic_out"]) == 2

    card._on_direction(String(json.dumps({"state": "fresh", "angle": 90})))
    info = card.dispatch("info", {})
    assert info["sound_direction"]["angle"] == 90
    assert info["topic_out"] == card.get_tool()["topic_out"]
    card._last_direction_time -= 11
    assert card.dispatch("info", {})["sound_direction"] == {"state": "stale"}

    assert card.dispatch("stop", {}) == {"state": "idle"}
    assert card.dispatch("start", {})["state"] == "running"
    assert len(added_words) == 1
    assert card.dispatch("stop", {}) == {"state": "idle"}
    assert card.dispatch("info", {})["sound_direction"] == {"state": "no_event"}
    card._on_direction(String(json.dumps({"state": "fresh", "angle": 180})))
    assert card.dispatch("info", {})["sound_direction"] == {"state": "no_event"}


def test_concurrent_calibrations_keep_both_directions(monkeypatch, tmp_path):
    module, _ = _load_device(monkeypatch)
    path = tmp_path / "calibration.json"
    monkeypatch.setattr(module, "_MIC_DIRECTION_CALIBRATION", path)
    monkeypatch.setitem(sys.modules, "sound_direction", types.SimpleNamespace(
        estimate_signature=lambda audio, channels, rate: (1.0, 2.0, 3.0)))
    clock = threading.local()

    def monotonic():
        clock.tick = getattr(clock, "tick", -1) + 1
        return clock.tick

    monkeypatch.setattr(module, "time", types.SimpleNamespace(
        monotonic=monotonic, sleep=lambda seconds: None))
    media = types.SimpleNamespace(get_audio_capture_data=lambda: types.SimpleNamespace(
        channels=8, sample_rate=16000, audio_data=[1] * 8))
    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.MicPlugin({}, "robot", executor, media)
    original_read = Path.read_text
    both_read_old_file = threading.Barrier(2)

    def read_text(file, *args, **kwargs):
        if file == path and not file.exists():
            try:
                both_read_old_file.wait(timeout=0.2)
            except threading.BrokenBarrierError:
                pass
        return original_read(file, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(lambda direction: card.dispatch(
            f"calibrate_{direction}", {}), ("front", "right")))

    assert all(result["state"] == "calibrated" for result in results)
    assert set(json.loads(path.read_text())) == {"front", "right"}
