"""Contract tests for the independent Bumi sound direction card."""

import importlib.util
import json
import sys
import threading
import types
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import yaml

from test_mic_card import _load_device


def _load_card(monkeypatch):
    _load_device(monkeypatch)  # Install hardware-free ROS message stubs.
    path = Path(__file__).resolve().parents[1] / "direction_card.py"
    monkeypatch.syspath_prepend(str(path.parent))
    spec = importlib.util.spec_from_file_location("bumi_direction_card_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_direction_card_reuses_existing_calibration_and_reports_its_own_topic(
    monkeypatch, tmp_path
):
    module = _load_card(monkeypatch)
    calibration = tmp_path / "sound_direction_calibration.json"
    calibration.write_text(json.dumps({"front": [1.0, 0.0, 0.0],
                                       "right": [0.0, 1.0, 0.0]}))
    monkeypatch.setattr(module, "CALIBRATION_PATH", calibration)
    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.SoundDirectionPlugin({}, "robot", executor)

    assert card.get_tool()["name"] == "sound_direction"
    assert card.get_tool()["type"] == "sensor"
    assert card.get_tool()["inputSchema"] == {"type": "object", "properties": {}}
    assert card.get_tool()["topic_out"] == [
        {"topic": "/robot/sound_direction", "format": "data/json"}]
    control = card.get_tools()[1]
    assert control["name"] == "sound_direction_control"
    assert control["type"] == "actuator"
    assert "topic_out" not in control
    assert control["inputSchema"]["x-action-params"]["set_parameters"]["params"] == [
        "onset_level", "onset_ratio", "burst_level", "burst_ratio",
        "update_interval_ms"]
    info = card.dispatch("info", {})
    assert info["calibrated_directions"] == ["front", "right"]
    assert info["sound_direction"] == {"state": "no_event"}


def test_direction_card_does_not_require_mic_process(monkeypatch):
    module = _load_card(monkeypatch)
    executor = types.SimpleNamespace(add_node=lambda node: None)
    card = module.SoundDirectionPlugin({}, "robot", executor)
    started = []

    class Process:
        stdout = (b"__BUMI_DIRECTION_READY__\n",)

        def poll(self):
            return None

        def terminate(self):
            started.append("stopped")

        def wait(self, timeout=None):
            return 0

    monkeypatch.setattr(module.subprocess, "Popen", lambda *a, **kw: started.append("started") or Process())
    assert card.dispatch("start", {})["state"] == "running"
    assert started == ["started"]
    assert card.dispatch("stop", {}) == {"state": "idle"}
    assert started == ["started", "stopped"]


def test_direction_sensor_start_reports_worker_initialization_failure(monkeypatch):
    module = _load_card(monkeypatch)
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))

    class ExitedProcess:
        stdout = ()

        def poll(self):
            return 1

        def terminate(self):
            pass

        def wait(self, timeout=None):
            return 1

    monkeypatch.setattr(module.subprocess, "Popen", lambda *args, **kwargs: ExitedProcess())
    result = card.dispatch("start", {"_tool_name": "sound_direction"})
    assert result["state"] == "error"
    assert card._proc is None


def test_direction_auto_start_reports_worker_initialization_failure(monkeypatch):
    module = _load_card(monkeypatch)
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))

    class ExitedProcess:
        stdout = ()

        def poll(self):
            return 1

        def terminate(self):
            pass

        def wait(self, timeout=None):
            return 1

    monkeypatch.setattr(module.subprocess, "Popen", lambda *args, **kwargs: ExitedProcess())
    assert card.start() is False
    assert card._proc is None


def test_mic_card_only_advertises_audio_after_migration(monkeypatch):
    module, _ = _load_device(monkeypatch)
    card = module.MicPlugin({}, "robot", types.SimpleNamespace(add_node=lambda node: None), object())
    assert card.get_tool()["topic_out"] == [
        {"topic": "/robot/mic/audio", "format": "audio/pcm-16k"}]
    assert card.get_tool()["inputSchema"] == {"type": "object", "properties": {}}


def test_bundle_exposes_audio_and_direction_as_separate_cards(monkeypatch, tmp_path):
    device, _ = _load_device(monkeypatch)
    direction = _load_card(monkeypatch)
    monkeypatch.setitem(sys.modules, "device", device)
    monkeypatch.setitem(sys.modules, "direction_card", direction)
    monkeypatch.setitem(sys.modules, "rclpy.executors", types.ModuleType("rclpy.executors"))
    path = Path(__file__).resolve().parents[1] / "main.py"
    spec = importlib.util.spec_from_file_location("bumi_direction_bundle_test", path)
    bundle_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bundle_module)
    cfg = {"plugins": {"mic": {"enabled": True},
                       "sound_direction": {"enabled": True}}}
    bundle = bundle_module.BumiDeviceBundle(
        cfg, "robot", types.SimpleNamespace(add_node=lambda node: None), None, object())
    assert {tool["name"] for tool in bundle.get_all_tools()} == {
        "mic", "sound_direction", "sound_direction_control"}
    started = []
    for plugin in bundle._plugins:
        plugin.start = lambda name=plugin.get_tool()["name"]: started.append(name)
    bundle.start_all()
    assert started == ["mic", "sound_direction"]
    manifest = yaml.safe_load((path.parent / "driver.yaml").read_text(encoding="utf-8"))
    categories = {card["name"]: card["type"] for card in manifest["cards"]}
    assert categories["sound_direction"] == "sensor"
    assert categories["sound_direction_control"] == "actuator"
    control_info = bundle.dispatch("sound_direction_control", {"action": "check_direction"})
    assert control_info["sound_direction"] == {"state": "no_event"}
    assert "topic_out" not in control_info
    assert bundle.dispatch("sound_direction", {"action": "info"})["topic_out"] == [
        {"topic": "/robot/sound_direction", "format": "data/json"}]
    assert bundle.dispatch("sound_direction", {"action": "set_parameters",
                                               "onset_level": 20}) is None
    monkeypatch.setattr(direction, "SETTINGS_PATH", tmp_path / "settings.json")
    assert bundle.dispatch("sound_direction_control", {"action": "set_parameters",
                                                       "onset_level": 20})["state"] == "configured"
    assert bundle.dispatch("sound_direction_control", {"action": "info"})[
        "parameters"]["onset_level"] == 20


def test_control_lifecycle_does_not_stop_direction_sensor(monkeypatch):
    module = _load_card(monkeypatch)
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))
    stopped = []

    class Process:
        def poll(self):
            return None

        def terminate(self):
            stopped.append(True)

        def wait(self, timeout=None):
            return 0

    card._proc = Process()
    assert card.dispatch("start", {"_tool_name": "sound_direction_control"}) == {
        "state": "ready"}
    assert card.dispatch("stop", {"_tool_name": "sound_direction_control"}) == {
        "state": "idle"}
    assert stopped == []
    assert card.dispatch("stop", {"_tool_name": "sound_direction"}) == {
        "state": "idle"}
    assert stopped == [True]


def test_direction_card_validates_observations(monkeypatch):
    module = _load_card(monkeypatch)
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))
    card._proc = types.SimpleNamespace(poll=lambda: None)
    for payload in ("{", "[]", "42", '{"state":"fresh","angle":360}',
                    '{"state":"fresh","angle":"right"}', '{"state":"fresh","angle":90,"x":{}}'):
        card._on_direction(module.String(payload))
        assert card.dispatch("info", {})["sound_direction"] == {"state": "no_event"}
    card._on_direction(module.String('{"state":"fresh","angle":90}'))
    assert card.dispatch("check_direction", {})["sound_direction"]["angle"] == 90
    card._last_direction_time -= 11
    assert card.dispatch("info", {})["sound_direction"] == {"state": "stale"}


def test_exited_direction_worker_cannot_report_or_receive_fresh_angle(monkeypatch):
    module = _load_card(monkeypatch)
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))
    exit_code = [None]
    card._proc = types.SimpleNamespace(poll=lambda: exit_code[0])
    card._on_direction(module.String('{"state":"fresh","angle":90}'))
    assert card.dispatch("check_direction", {})["sound_direction"]["state"] == "fresh"

    exit_code[0] = 1
    card._on_direction(module.String('{"state":"fresh","angle":180}'))
    for action in ("info", "check_direction"):
        result = card.dispatch(action, {"_tool_name": "sound_direction_control"})
        assert result["state"] == "idle"
        assert result["sound_direction"] == {"state": "no_event"}


def test_direction_card_preserves_front_right_calibration(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    path = tmp_path / "sound_direction_calibration.json"
    path.write_text(json.dumps({"version": 2, "front": [1.0, 0.0, 0.0]}))
    monkeypatch.setattr(module, "CALIBRATION_PATH", path)
    ticks = [0.0]

    def monotonic():
        ticks[0] += 0.01
        return ticks[0]

    monkeypatch.setattr(module.time, "monotonic", monotonic)
    monkeypatch.setattr(module.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(module, "is_voiced_audio", lambda *args: True)
    monkeypatch.setattr(module, "estimate_signature", lambda *args: [0.0, 1.0, 0.0])
    count = [0]

    def capture():
        count[0] += 1
        return types.SimpleNamespace(
            channels=8, sample_rate=16000,
            audio_data=[count[0]] * (8 * 640))

    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None),
        types.SimpleNamespace(get_audio_capture_data=capture))
    result = card.dispatch("calibrate_right", {})
    assert result == {"state": "calibrated", "direction": "right", "remaining": []}
    assert json.loads(path.read_text()) == {
        "version": 2, "front": [1.0, 0.0, 0.0], "right": [0.0, 1.0, 0.0]}


def test_calibration_reports_restart_failure_but_preserves_saved_signature(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    path = tmp_path / "calibration.json"
    monkeypatch.setattr(module, "CALIBRATION_PATH", path)
    ticks = [0.0]

    def monotonic():
        ticks[0] += 0.01
        return ticks[0]

    monkeypatch.setattr(module.time, "monotonic", monotonic)
    monkeypatch.setattr(module.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(module, "is_voiced_audio", lambda *args: True)
    monkeypatch.setattr(module, "estimate_signature", lambda *args: [1.0, 2.0, 3.0])
    count = [0]

    def capture():
        count[0] += 1
        return types.SimpleNamespace(
            channels=8, sample_rate=16000,
            audio_data=[count[0]] * (8 * 16000))

    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None),
        types.SimpleNamespace(get_audio_capture_data=capture))

    class RunningProcess:
        def poll(self):
            return None

        def terminate(self):
            pass

        def wait(self, timeout=None):
            return 0

    card._proc = RunningProcess()
    monkeypatch.setattr(module.subprocess, "Popen", lambda *args, **kwargs: (
        _ for _ in ()).throw(OSError("spawn failed")))
    result = card.dispatch("calibrate_front", {})
    assert result["state"] == "error"
    assert result["calibration_saved"] is True
    assert result["direction"] == "front"
    assert json.loads(path.read_text()) == {"front": [1.0, 2.0, 3.0]}

    class ExitedProcess(RunningProcess):
        stdout = ()

        def poll(self):
            return 1

    card._proc = RunningProcess()
    monkeypatch.setattr(module.subprocess, "Popen", lambda *args, **kwargs: ExitedProcess())
    result = card.dispatch("calibrate_front", {})
    assert result["state"] == "error"
    assert result["calibration_saved"] is True
    assert card._proc is None


def test_concurrent_direction_calibrations_keep_both_signatures(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    path = tmp_path / "calibration.json"
    monkeypatch.setattr(module, "CALIBRATION_PATH", path)
    monkeypatch.setattr(module, "is_voiced_audio", lambda *args: True)
    monkeypatch.setattr(module, "estimate_signature", lambda *args: [1.0, 2.0, 3.0])
    clock = threading.local()

    def monotonic():
        clock.tick = getattr(clock, "tick", -1) + 1
        return clock.tick

    monkeypatch.setattr(module.time, "monotonic", monotonic)
    monkeypatch.setattr(module.time, "sleep", lambda seconds: None)
    count = threading.local()

    def capture():
        count.value = getattr(count, "value", 0) + 1
        return types.SimpleNamespace(
            channels=8, sample_rate=16000,
            audio_data=[count.value] * (16000 * 8))

    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None),
        types.SimpleNamespace(get_audio_capture_data=capture))
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(lambda direction: card.dispatch(
            f"calibrate_{direction}", {}), ("front", "right")))
    assert all(result["state"] == "calibrated" for result in results)
    assert set(json.loads(path.read_text())) == {"front", "right"}


def test_direction_calibration_rejects_repeated_frames(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    path = tmp_path / "calibration.json"
    monkeypatch.setattr(module, "CALIBRATION_PATH", path)
    ticks = [0.0]

    def monotonic():
        ticks[0] += 0.01
        return ticks[0]

    monkeypatch.setattr(module.time, "monotonic", monotonic)
    monkeypatch.setattr(module.time, "sleep", lambda seconds: None)
    frame = types.SimpleNamespace(channels=8, sample_rate=16000,
                                  audio_data=[100] * (8 * 640))
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None),
        types.SimpleNamespace(get_audio_capture_data=lambda: frame))
    assert card.dispatch("calibrate_front", {})["state"] == "no_voice"
    assert not path.exists()


def test_direction_info_ignores_corrupt_or_nonfinite_calibration(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    path = tmp_path / "calibration.json"
    monkeypatch.setattr(module, "CALIBRATION_PATH", path)
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))
    for content in ("not json", "null", '{"front":[NaN,1,2]}',
                    '{"front":[1,2],"right":[1,2]}'):
        path.write_text(content)
        assert card.dispatch("info", {})["calibrated_directions"] == []


def test_direction_thresholds_can_be_saved_reset_and_reused(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    settings_path = tmp_path / "direction_settings.json"
    monkeypatch.setattr(module, "SETTINGS_PATH", settings_path)
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))
    defaults = card.dispatch("info", {})["parameters"]
    assert defaults == {
        "onset_level": 10.0, "onset_ratio": 1.8,
        "burst_level": 15.0, "burst_ratio": 3.0,
        "update_interval_ms": 100}
    result = card.dispatch("set_parameters", {"onset_ratio": 2.4,
                                               "burst_ratio": 4.0,
                                               "onset_level": None})
    assert result["state"] == "configured"
    assert result["parameters"]["onset_ratio"] == 2.4
    assert json.loads(settings_path.read_text()) == result["parameters"]
    new_card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))
    assert new_card.dispatch("info", {})["parameters"]["burst_ratio"] == 4.0
    assert new_card.dispatch("reset_parameters", {})["parameters"] == defaults


def test_direction_thresholds_reject_invalid_values_without_overwriting(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    settings_path = tmp_path / "direction_settings.json"
    monkeypatch.setattr(module, "SETTINGS_PATH", settings_path)
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))
    for args in ({"onset_ratio": float("nan")}, {"burst_ratio": 0.5},
                 {"update_interval_ms": 0}, {"unknown": 12}):
        assert card.dispatch("set_parameters", args)["state"] == "invalid_parameters"
    assert not settings_path.exists()


def test_reconfiguration_reports_restart_failure_with_saved_settings(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    settings_path = tmp_path / "direction_settings.json"
    monkeypatch.setattr(module, "SETTINGS_PATH", settings_path)
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))

    class RunningProcess:
        def poll(self):
            return None

        def terminate(self):
            pass

        def wait(self, timeout=None):
            return 0

    card._proc = RunningProcess()
    monkeypatch.setattr(module.subprocess, "Popen", lambda *a, **kw: (
        _ for _ in ()).throw(OSError("spawn failed")))
    result = card.dispatch("set_parameters", {"onset_ratio": 2.4})
    assert result["state"] == "error"
    assert result["parameters"]["onset_ratio"] == 2.4
    assert json.loads(settings_path.read_text())["onset_ratio"] == 2.4


def test_control_settings_restart_reader_with_new_values(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, "SETTINGS_PATH", tmp_path / "settings.json")
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))
    events = []

    class Process:
        stdout = (b"__BUMI_DIRECTION_READY__\n",)

        def poll(self):
            return None

        def terminate(self):
            events.append("stop")

        def wait(self, timeout=None):
            events.append("reaped")
            return 0

    card._proc = Process()
    def spawn(*args, **kwargs):
        events.append(("spawn", module.load_parameters()))
        return Process()

    monkeypatch.setattr(module.subprocess, "Popen", spawn)
    result = card.dispatch("set_parameters", {
        "_tool_name": "sound_direction_control",
        "onset_ratio": 2.6, "update_interval_ms": 250})
    assert result["state"] == "configured"
    assert events == ["stop", "reaped", ("spawn", result["parameters"])]
    assert result["parameters"]["onset_ratio"] == 2.6
    assert result["parameters"]["update_interval_ms"] == 250


def test_control_reports_immediately_exited_reader(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, "SETTINGS_PATH", tmp_path / "settings.json")
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))

    class Process:
        stdout = ()

        def __init__(self, exit_code):
            self.exit_code = exit_code

        def poll(self):
            return self.exit_code

        def terminate(self):
            pass

        def wait(self, timeout=None):
            return 0

    card._proc = Process(None)
    monkeypatch.setattr(module.subprocess, "Popen", lambda *args, **kwargs: Process(1))
    result = card.dispatch("set_parameters", {
        "_tool_name": "sound_direction_control", "onset_ratio": 2.6})
    assert result["state"] == "error"
    assert "restart" in result["message"]


def test_control_rejects_reader_that_exits_during_initialization(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, "SETTINGS_PATH", tmp_path / "settings.json")
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))

    class Process:
        stdout = ()

        def __init__(self):
            self.polls = 0

        def poll(self):
            self.polls += 1
            return None if self.polls <= 2 else 1

        def terminate(self):
            pass

        def wait(self, timeout=None):
            return 1

    card._proc = Process()
    monkeypatch.setattr(module.subprocess, "Popen", lambda *args, **kwargs: Process())
    result = card.dispatch("set_parameters", {"onset_ratio": 2.6})
    assert result["state"] == "error"
    assert "restart" in result["message"]


def test_direction_worker_publishes_speech_then_clears_without_fan_angle(
    monkeypatch, tmp_path
):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, "SETTINGS_PATH", tmp_path / "settings.json")
    card = module.SoundDirectionPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda node: None))
    settings = {"onset_level": 11, "onset_ratio": 1.7,
                "burst_level": 14, "burst_ratio": 2.9,
                "update_interval_ms": 250}
    assert card.dispatch("set_parameters", {
        "_tool_name": "sound_direction_control", **settings})["state"] == "configured"
    gate_options = []
    original_gate = module.SoundActivityGate

    class TrackedGate(original_gate):
        def __init__(self, **kwargs):
            gate_options.append(kwargs)
            super().__init__(**kwargs)

    monkeypatch.setattr(module, "SoundActivityGate", TrackedGate)
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps({"front": [2, -3, 1], "right": [0, 1, -2]}))
    monkeypatch.setattr(module, "CALIBRATION_PATH", path)
    published = []

    class Node:
        def __init__(self, name):
            pass

        def create_publisher(self, message_type, topic, qos):
            assert topic == "/robot/sound_direction"
            return types.SimpleNamespace(publish=published.append)

    monkeypatch.setattr(module, "Node", Node)
    monkeypatch.setattr(module.rclpy, "init", lambda: None, raising=False)
    monkeypatch.setitem(sys.modules, "common", types.SimpleNamespace(
        logsafe=types.SimpleNamespace(install=lambda **kwargs: None)))
    class Done(BaseException):
        pass

    rng = np.random.default_rng(73)
    source = rng.normal(0, 2000, 62 * 640).astype(np.int16)
    front = [np.roll(source, shift) for shift in (0, 2, -3, 1)]
    front.extend([np.zeros_like(source) for _ in range(4)])
    audio = np.stack(front, axis=1)
    audio[:10240] //= 50
    rear = [np.roll(source, shift) for shift in (0, -2, 3, -1)]
    rear.extend([np.zeros_like(source) for _ in range(4)])
    rear_audio = np.stack(rear, axis=1)
    audio[32 * 640:] = rear_audio[32 * 640:] // 30
    frames = iter(enumerate(audio.reshape(62, 640, 8)))
    clock = [0.0]

    def capture():
        try:
            index, frame = next(frames)
        except StopIteration:
            raise Done()
        clock[0] += 0.04
        return types.SimpleNamespace(
            timestamp_us=1_700_000_000_000_000 + index * 40_000,
            channels=8, sample_rate=16000, audio_data=frame.reshape(-1).tolist())

    media = types.SimpleNamespace(init=lambda: True, get_audio_capture_data=capture)
    monkeypatch.setitem(sys.modules, "mediacontrol_py", types.SimpleNamespace(
        MediaController=types.SimpleNamespace(instance=lambda: media)))
    monkeypatch.setattr(module.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(module.time, "time", lambda: clock[0])
    try:
        module._direction_subprocess("robot")
    except Done:
        pass

    messages = [json.loads(message.data) for message in published]
    fresh = [message for message in messages if message["state"] == "fresh"]
    assert gate_options == [{key: settings[key] for key in (
        "onset_level", "onset_ratio", "burst_level", "burst_ratio")}]
    assert 2 <= len(fresh) <= 5
    assert all(next_message["timestamp_ms"] - message["timestamp_ms"] >= 250
               for message, next_message in zip(fresh, fresh[1:]))
    assert all(message["angle"] == 0 for message in fresh)
    assert messages[-1] == {"state": "no_event"}
    assert all(message["audio_window_start_us"] <= message["audio_window_end_us"]
               for message in fresh)
