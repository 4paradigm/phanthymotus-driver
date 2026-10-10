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
    media = object()
    card = module.MicPlugin({}, "robot", executor, media)
    assert "check_direction" in card.get_tool()["inputSchema"]["properties"]["action"]["enum"]
    assert "check_direction" in card.get_tool()["inputSchema"]["x-action-params"]

    assert card.dispatch("start", {})["state"] == "running"
    assert card.dispatch("check_direction", {})["sound_direction"] == {"state": "no_event"}
    assert card.dispatch("start", {})["state"] == "running"
    assert "add_wakeup_word" not in card.get_tool()["inputSchema"]["properties"]["action"]["enum"]
    assert len(processes) == 1
    assert len(card.get_tool()["topic_out"]) == 2

    card._on_direction(String(json.dumps({"state": "fresh", "angle": 90})))
    info = card.dispatch("info", {})
    assert info["sound_direction"]["angle"] == 90
    assert info["topic_out"] == card.get_tool()["topic_out"]
    assert card.dispatch("check_direction", {})["sound_direction"]["angle"] == 90
    card._on_direction(String(json.dumps({"state": "no_event"})))
    assert card.dispatch("check_direction", {})["sound_direction"]["angle"] == 90
    card._last_direction_time -= 11
    assert card.dispatch("info", {})["sound_direction"] == {"state": "stale"}
    assert card.dispatch("check_direction", {})["sound_direction"] == {"state": "stale"}

    assert card.dispatch("stop", {}) == {"state": "idle"}
    assert card.dispatch("start", {})["state"] == "running"
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


def test_direction_contract_from_estimator_to_subscription(monkeypatch):
    """发布端 _mic_activity_payload → JSON → _on_direction 的契约。"""
    module, String = _load_device(monkeypatch)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    sys.modules.pop("sound_direction", None)

    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.MicPlugin({}, "robot", executor, object())
    card._proc = types.SimpleNamespace(poll=lambda: None)

    calibration = {"front": [2.0, -1.0, 0.0], "right": [0.0, 1.0, -2.0]}
    rng = np.random.default_rng(73)
    source = rng.normal(0, 2000, 4096).astype(np.int16)
    channels = [np.roll(source, shift) for shift in (0, 2, -3, 1)]
    channels.extend([np.zeros_like(source) for _ in range(4)])
    raw = module._mic_activity_payload(
        np.stack(channels, axis=1).reshape(-1), calibration, 1)
    payload = json.loads(raw)
    assert payload["state"] == "fresh"
    assert type(payload["angle"]) is int
    card._on_direction(String(raw))
    assert card.dispatch("info", {})["sound_direction"]["angle"] == payload["angle"]

    # 发布端若把角度序列化成 float，订阅端必须拒收整条消息。
    with card._direction_lock:
        card._last_direction = None
        card._last_direction_time = 0.0
    card._on_direction(String(
        json.dumps({**payload, "angle": float(payload["angle"]) + 0.5})))
    assert card.dispatch("info", {})["sound_direction"] == {"state": "no_event"}


def test_mic_capture_time_falls_back_when_sdk_timestamp_is_missing(monkeypatch):
    module, _ = _load_device(monkeypatch)
    assert module._mic_capture_timestamp_us(types.SimpleNamespace(timestamp_us=123), 456) == 123
    assert module._mic_capture_timestamp_us(types.SimpleNamespace(timestamp_us=0), 456) == 456
    assert module._mic_capture_timestamp_us(types.SimpleNamespace(), 456) == 456


def test_sound_activity_reports_coherent_source_without_wake_status(monkeypatch):
    module, String = _load_device(monkeypatch)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    sys.modules.pop("sound_direction", None)
    rng = np.random.default_rng(71)
    source = rng.normal(0, 2000, 4096).astype(np.int16)
    channels = [np.roll(source, shift) for shift in (0, 2, -3, 1)]
    channels.extend([np.zeros_like(source) for _ in range(4)])
    audio = np.stack(channels, axis=1).reshape(-1)
    calibration = {"front": [2, -3, 1], "right": [0, 1, -2]}

    payload = json.loads(module._mic_activity_payload(audio, calibration, 1234))
    assert payload["trigger"] == "sound_activity"
    assert payload["state"] == "fresh"
    assert payload["angle"] == 0
    right_channels = [np.roll(source, shift) for shift in (0, 0, 1, -2)]
    right_channels.extend([np.zeros_like(source) for _ in range(4)])
    right_audio = np.stack(right_channels, axis=1).reshape(-1)
    assert json.loads(module._mic_activity_payload(
        right_audio, calibration, 1235))["angle"] == 90
    assert module._mic_activity_payload(np.zeros_like(audio), calibration, 1234) is None
    unrelated = rng.normal(0, 2000, (4096, 8)).astype(np.int16).reshape(-1)
    assert module._mic_activity_payload(unrelated, calibration, 1234) is None
    assert module._mic_activity_payload(audio, {}, 1234) is None

    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.MicPlugin({}, "robot", executor, object())
    card._proc = types.SimpleNamespace(poll=lambda: None)
    card._on_direction(String(json.dumps(payload)))
    assert card.dispatch("check_direction", {})["sound_direction"]["angle"] == 0


def test_mic_capture_ignores_vendor_wake_and_paused_fan_noise(monkeypatch, tmp_path, capsys):
    module, String = _load_device(monkeypatch)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    sys.modules.pop("sound_direction", None)
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps({"front": [2, -3, 1], "right": [0, 1, -2]}))
    monkeypatch.setattr(module, "_MIC_DIRECTION_CALIBRATION", path)
    published = {}

    class Node:
        def __init__(self, name):
            pass

        def create_publisher(self, message_type, topic, qos):
            published[topic] = []
            return types.SimpleNamespace(publish=published[topic].append)

    monkeypatch.setattr(sys.modules["rclpy"], "init", lambda: None, raising=False)
    monkeypatch.setattr(sys.modules["rclpy.node"], "Node", Node)
    monkeypatch.setitem(sys.modules, "common", types.SimpleNamespace(
        logsafe=types.SimpleNamespace(install=lambda **kwargs: None)))

    class Done(BaseException):
        pass

    rng = np.random.default_rng(73)
    source = rng.normal(0, 2000, 62 * 640).astype(np.int16)
    channels = [np.roll(source, shift) for shift in (0, 2, -3, 1)]
    channels.extend([np.zeros_like(source) for _ in range(4)])
    audio = np.stack(channels, axis=1)
    # 前 16 帧是同方向的低音量背景声，随后才出现明显发声。
    audio[:10240] //= 50
    # 说话结束后，后方风扇略高于初始背景，但不应刷新说话者的角度。
    rear_channels = [np.roll(source, shift) for shift in (0, -2, 3, -1)]
    rear_channels.extend([np.zeros_like(source) for _ in range(4)])
    rear_audio = np.stack(rear_channels, axis=1)
    audio[32 * 640:] = rear_audio[32 * 640:] // 30
    frames = iter(enumerate(audio.reshape(62, 640, 8)))
    clock = [0.0]
    capture_times = iter(1_700_000_000_000_000 + n * 40_000 for n in range(62))
    quiet_direction_counts = []

    def capture():
        try:
            frame_index, frame = next(frames)
        except StopIteration:
            raise Done()
        if frame_index == 16:
            quiet_direction_counts.append(len(published["/robot/mic/sound_direction"]))
        clock[0] += 0.04  # 每帧 40 毫秒，模拟持续输入。
        data = frame.reshape(-1).tolist()
        if frame_index == 10:
            data.append(1)  # SDK 偶发的不完整 8 通道帧不得中断音频。
        return types.SimpleNamespace(timestamp_us=next(capture_times),
                                     channels=8, sample_rate=16000,
                                     audio_data=data)

    status_calls = [0]

    def wake_status():
        status_calls[0] += 1
        return types.SimpleNamespace(
            reason=types.SimpleNamespace(name="AUDIO_WAKEUPED"),
            header=types.SimpleNamespace(message_id=status_calls[0],
                                         timestamp_us=status_calls[0] * 1000))

    media = types.SimpleNamespace(
        init=lambda: True,
        get_system_status=wake_status,
        get_audio_capture_data=capture)
    monkeypatch.setitem(sys.modules, "mediacontrol_py", types.SimpleNamespace(
        MediaController=types.SimpleNamespace(instance=lambda: media)))
    monkeypatch.setattr(sys.modules["std_msgs.msg"], "String", String)
    monkeypatch.setattr(module.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])

    try:
        module._mic_subprocess("robot")
    except Done:
        pass

    messages = published["/robot/mic/sound_direction"]
    fresh_messages = [message for message in messages
                      if json.loads(message.data)["state"] == "fresh"]
    assert quiet_direction_counts == [0]
    assert len(fresh_messages) >= 3  # 100 毫秒检查一次，发声后应多次发布。
    assert len(fresh_messages) <= 7  # 16 帧 × 40 ms；删除限流后会超出该上界。
    assert json.loads(fresh_messages[0].data)["angle"] == 0
    speech_end_us = 1_700_000_000_000_000 + 32 * 40_000
    assert all(json.loads(message.data)["audio_window_start_us"] < speech_end_us
               for message in fresh_messages)
    assert json.loads(fresh_messages[0].data)["trigger"] == "sound_activity"
    assert all(json.loads(message.data).get("trigger") != "vendor_audio_wakeup"
               for message in messages)
    assert status_calls[0] == 0
    assert (json.loads(fresh_messages[0].data)["audio_window_end_us"]
            <= 1_700_000_000_000_000 + 16 * 40_000 + 300_000)
    assert [json.loads(message.data)["state"] for message in messages].count("no_event") == 1
    assert json.loads(messages[-1].data)["state"] == "no_event"
    assert "[mic_subprocess] error:" not in capsys.readouterr().out
    audio_stamps = [msg.header.stamp.sec * 1_000_000
                    + msg.header.stamp.nanosec // 1000
                    for msg in published["/robot/mic/audio"]]
    for message in fresh_messages:
        direction = json.loads(message.data)
        assert direction["audio_window_start_us"] <= direction["audio_window_end_us"]
        assert direction["audio_window_end_us"] - direction["audio_window_start_us"] <= 1_000_000
        assert any(direction["audio_window_start_us"] <= stamp <=
                   direction["audio_window_end_us"] for stamp in audio_stamps)


def test_calibration_rejects_nonfinite_values(monkeypatch, tmp_path):
    module, _ = _load_device(monkeypatch)
    path = tmp_path / "calibration.json"
    path.write_text('{"front":[NaN,1,2],"right":[1,Infinity,2]}')
    monkeypatch.setattr(module, "_MIC_DIRECTION_CALIBRATION", path)
    assert module._load_calibration() == {}


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


def test_mic_direction_rejects_unexpected_shapes(monkeypatch):
    module, String = _load_device(monkeypatch)
    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.MicPlugin({}, "robot", executor, object())
    card._proc = types.SimpleNamespace(poll=lambda: None)

    for payload in (
        '{"state":"unknown","angle":90}',
        '{"state":"fresh","angle":360}',
        '{"state":"fresh","angle":-1}',
        '{"state":"fresh"}',
        '{"state":"ambiguous","angle":true}',
        '{"state":"ambiguous","angle":90,"extra":{"nested":1}}',
    ):
        card._on_direction(String(payload))
        assert card.dispatch("info", {})["sound_direction"] == {"state": "no_event"}

    card._on_direction(String('{"state":"ambiguous","angle":90}'))
    assert card.dispatch("info", {})["sound_direction"]["angle"] == 90


def test_info_survives_corrupt_calibration_files(monkeypatch, tmp_path):
    module, _ = _load_device(monkeypatch)
    path = tmp_path / "calibration.json"
    monkeypatch.setattr(module, "_MIC_DIRECTION_CALIBRATION", path)
    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.MicPlugin({}, "robot", executor, object())

    for content in ("not json", "5", "null", '"str"',
                    '{"front":"x"}', '{"front":[1,"2",3]}',
                    '{"front":[1,2]}', '{"front":[true,1,2]}',
                    "[" * 3000 + "]" * 3000):
        path.write_text(content)
        info = card.dispatch("info", {})
        assert info["calibrated_directions"] == []
        assert info["sound_direction"] == {"state": "no_event"}

    path.write_text(json.dumps({"front": [2, -1, 0], "right": [0, 1, -2.5]}))
    assert card.dispatch("info", {})["calibrated_directions"] == ["front", "right"]

    # 非 front/right 键即使形状像签名也不得进入标定视图。
    path.write_text(json.dumps({"front": [2, -1, 0], "back": [1, 2, 3]}))
    assert set(module._load_calibration()) == {"front"}


def test_calibration_result_survives_restart_failure(monkeypatch, tmp_path):
    module, _ = _load_device(monkeypatch)
    path = tmp_path / "calibration.json"
    monkeypatch.setattr(module, "_MIC_DIRECTION_CALIBRATION", path)
    monkeypatch.setitem(sys.modules, "sound_direction", types.SimpleNamespace(
        estimate_signature=lambda audio, channels, rate: (1.0, 2.0, 3.0),
        is_voiced_audio=lambda audio, channels, rate: True))
    ticks = [0.0]

    def monotonic():
        ticks[0] += 0.01
        return ticks[0]

    monkeypatch.setattr(module, "time", types.SimpleNamespace(
        monotonic=monotonic, sleep=lambda seconds: None))
    media = types.SimpleNamespace(
        get_audio_capture_data=lambda: types.SimpleNamespace(
            channels=8, sample_rate=16000, audio_data=[100] * (16000 * 8)),
        get_wakeup_words=lambda: "小范小范")
    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.MicPlugin({}, "robot", executor, media)

    class Process:
        stdout = ()

        def poll(self):
            return None

        def terminate(self):
            pass

        def wait(self, timeout=None):
            return 0

    card._proc = Process()

    def fail_start():
        raise RuntimeError("spawn failed")

    monkeypatch.setattr(card, "start", fail_start)
    # 标定文件里预先存在的手工顶层键不得在写回时丢失。
    path.write_text(json.dumps({"version": 2}))

    result = card.dispatch("calibrate_front", {})
    assert result["state"] == "calibrated"
    assert path.exists()
    saved = json.loads(path.read_text())
    assert saved["version"] == 2
    assert isinstance(saved["front"], list)
    assert card._proc is None
