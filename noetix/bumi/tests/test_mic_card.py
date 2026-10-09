import importlib.util
import json
import subprocess
import sys
import threading
import types
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np


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

        def wait(self, timeout=None):
            return 0

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
        estimate_signature=lambda audio, channels, rate: (1.0, 2.0, 3.0),
        is_voiced_audio=lambda audio, channels, rate: True))
    clock = threading.local()

    def monotonic():
        clock.tick = getattr(clock, "tick", -1) + 1
        return clock.tick

    monkeypatch.setattr(module, "time", types.SimpleNamespace(
        monotonic=monotonic, sleep=lambda seconds: None))
    media = types.SimpleNamespace(get_audio_capture_data=lambda: types.SimpleNamespace(
        channels=8, sample_rate=16000, audio_data=[1] * (16000 * 8)))
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


def test_mic_ignores_malformed_direction_messages(monkeypatch):
    module, String = _load_device(monkeypatch)
    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.MicPlugin({}, "robot", executor, object())
    card._proc = types.SimpleNamespace(poll=lambda: None)

    for payload in ("{", "[]", "42", '{"state":"fresh","angle":"right"}'):
        card._on_direction(String(payload))
        assert card.dispatch("info", {})["sound_direction"] == {"state": "no_event"}

    card._on_direction(String('{"state":"fresh","angle":90}'))
    assert card.dispatch("info", {})["sound_direction"]["angle"] == 90


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


def test_calibration_rejects_repeated_capture_frames(monkeypatch, tmp_path):
    module, _ = _load_device(monkeypatch)
    path = tmp_path / "calibration.json"
    monkeypatch.setattr(module, "_MIC_DIRECTION_CALIBRATION", path)
    monkeypatch.setitem(sys.modules, "sound_direction", types.SimpleNamespace(
        estimate_signature=lambda audio, channels, rate: (1.0, 2.0, 3.0),
        is_voiced_audio=lambda audio, channels, rate: True))
    ticks = [0.0]
    sleeps = []

    def monotonic():
        ticks[0] += 0.01
        return ticks[0]

    monkeypatch.setattr(module, "time", types.SimpleNamespace(
        monotonic=monotonic, sleep=lambda seconds: sleeps.append(seconds)))
    events = []

    class Process:
        stdout = ()
        running = True

        def poll(self):
            return None if self.running else 0

        def terminate(self):
            events.append("terminate")
            self.running = False

        def wait(self, timeout=None):
            events.append("wait")
            return 0

    old_process = Process()
    frame = types.SimpleNamespace(channels=8, sample_rate=16000,
                                  audio_data=[100] * 1280)

    def capture():
        events.append("capture")
        return frame

    media = types.SimpleNamespace(get_audio_capture_data=capture,
                                  get_wakeup_words=lambda: "小范小范")
    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.MicPlugin({}, "robot", executor, media)
    card._proc = old_process
    monkeypatch.setattr(module.subprocess, "Popen", lambda *args, **kwargs: (
        events.append("spawn") or Process()))

    assert card.dispatch("calibrate_front", {})["state"] == "no_voice"
    assert events.index("wait") < events.index("capture")
    assert events[-1] == "spawn"
    assert sleeps
    assert not path.exists()


def test_calibration_does_not_save_coherent_noise(monkeypatch, tmp_path):
    module, _ = _load_device(monkeypatch)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    path = tmp_path / "calibration.json"
    monkeypatch.setattr(module, "_MIC_DIRECTION_CALIBRATION", path)
    rng = np.random.default_rng(37)
    source = rng.normal(0, 800, 32000).astype(np.int16)
    channels = [np.roll(source, shift) for shift in (0, 2, -3, 1)]
    channels.extend([np.zeros_like(source) for _ in range(4)])
    frames = iter(np.stack(channels, axis=1).reshape(-1, 1280))
    ticks = [0.0]

    def monotonic():
        ticks[0] += 0.01
        return ticks[0]

    monkeypatch.setattr(module, "time", types.SimpleNamespace(
        monotonic=monotonic, sleep=lambda seconds: None))

    def capture():
        frame = next(frames, [])
        return types.SimpleNamespace(channels=8, sample_rate=16000,
                                     audio_data=frame.tolist() if len(frame) else [])

    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.MicPlugin({}, "robot", executor, types.SimpleNamespace(
        get_audio_capture_data=capture))

    assert card.dispatch("calibrate_front", {})["state"] == "no_voice"
    assert not path.exists()


def test_wake_status_sampling_catches_short_separate_updates(monkeypatch):
    module, _ = _load_device(monkeypatch)

    def status(reason, message_id):
        return types.SimpleNamespace(
            reason=types.SimpleNamespace(name=reason),
            header=types.SimpleNamespace(message_id=message_id,
                                         timestamp_us=message_id * 1000))

    statuses = iter((status("CMD_SLEEPED", 1), status("AUDIO_WAKEUPED", 2),
                     status("CMD_SLEEPED", 3), status("AUDIO_WAKEUPED", 4),
                     status("AUDIO_WAKEUPED", 4)))
    media = types.SimpleNamespace(get_system_status=lambda: next(statuses))
    next_poll = 0.0
    last_key = None
    detected = []
    for now in (0.0, 0.01, 0.02, 0.03, 0.04):
        next_poll, last_key, event = module._poll_mic_wake_status(
            media, now, next_poll, last_key)
        if event is not None:
            detected.append(event)

    assert detected == [(2, 2000), (4, 4000)]
