"""Bumi recording contracts, failure cleanup and real stopwatch-clock muxing."""
from __future__ import annotations
from array import array
import io
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import types
import wave

import pytest

from test_mic_card import _load_device

_SPEC = importlib.util.spec_from_file_location(
    "bumi_av_recording", Path(__file__).resolve().parents[1] / "bumi_av_recording.py")
av = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = av
_SPEC.loader.exec_module(av)


def pcm(samples, value=1000):
    return int(value).to_bytes(2, "little", signed=True) * samples


def test_recorder_is_bumi_local_and_shipped_with_existing_device_copy():
    root = Path(__file__).resolve().parents[1]
    assert Path(av.__file__).resolve().parent == root
    dockerfile = (root / "Dockerfile").read_text(encoding="utf-8")
    copies = [line.split() for line in dockerfile.splitlines() if line.startswith("COPY device.py ")]
    assert len(copies) == 1
    assert "bumi_av_recording.py" in copies[0][1:-1]
    assert "foot_pose.py" in copies[0][1:-1]
    assert "foot_pose_state.py" in copies[0][1:-1]
    assert copies[0][-1] == "/work/"


def run_capture(monkeypatch, directory, events, duration=1, include_audio=True, collector=None):
    class Clock:
        now = 0.0

        def monotonic(self):
            return self.now

    clock = Clock()
    events = sorted(events, key=lambda item: item[1])

    class Buffer:
        def get(self, timeout):
            if events and events[0][1] <= clock.now + timeout:
                source, received, kind, data = events.pop(0)
                clock.now = max(clock.now, received)
                return kind, 10_000_000_000 + round(source * 1e9), data, received, received - source
            clock.now += timeout
            return None

    with monkeypatch.context() as context:
        context.setattr(av.time, "monotonic", clock.monotonic)
        return (collector or av.collect)(Buffer(), directory, duration, threading.Event(), include_audio)


def audio_events(duration=1, value=1000):
    return [(i * .04, i * .04, "audio", pcm(640, value))
            for i in range(1, round(duration / .04) + 1)]


def camera_events(times=(0, .15, .42, .76, .95)):
    return [(t, t + .01, "video", b"jpeg") for t in times]


def test_source_clock_keeps_preroll_audio_and_fills_exact_duration(monkeypatch, tmp_path):
    frames, info = run_capture(monkeypatch, tmp_path, camera_events() + audio_events())
    assert [t for t, _ in frames] == [0, .15, .42, .76, .95]
    assert info["audio_samples"] == 16000
    assert info["audio_gap_s"] == 0
    with wave.open(str(tmp_path / "audio.wav")) as stream:
        assert stream.getnframes() == 16000
        assert stream.readframes(16000) == pcm(16000)


@pytest.mark.parametrize("audio,code", [([], "AUDIO_UNAVAILABLE"),
    (audio_events(value=0), "AUDIO_NO_SIGNAL"),
    (audio_events()[:4], "AUDIO_INTERRUPTED")])
def test_missing_zero_or_interrupted_audio_is_not_silent_success(monkeypatch, tmp_path, audio, code):
    with pytest.raises(av.RecordingError) as error:
        run_capture(monkeypatch, tmp_path, camera_events() + audio)
    assert error.value.code == code


def test_camera_loss_and_backwards_stamps_fail(monkeypatch, tmp_path):
    with pytest.raises(av.RecordingError) as error:
        run_capture(monkeypatch, tmp_path, camera_events((0,)) + audio_events(2), duration=2)
    assert error.value.code == "VIDEO_INTERRUPTED"
    with pytest.raises(av.RecordingError) as error:
        run_capture(monkeypatch, tmp_path, [(0, 0, "video", b"jpeg"),
                    (.2, .2, "video", b"jpeg"), (.1, .3, "video", b"jpeg")], include_audio=False)
    assert error.value.code == "INVALID_MEDIA_TIMESTAMP"


def test_silent_recording_does_not_wait_for_microphone(monkeypatch, tmp_path):
    frames, info = run_capture(monkeypatch, tmp_path, camera_events(), include_audio=False)
    assert len(frames) == 5
    assert "audio_samples" not in info


def test_input_buffer_rejects_invalid_clock_and_overflow(monkeypatch):
    monkeypatch.setattr(av.time, "time_ns", lambda: 10_000_000_000)
    buffer = av.InputBuffer()
    buffer.put("audio", 0, pcm(640))
    with pytest.raises(av.RecordingError, match="fresh host"):
        buffer.get(0)
    buffer = av.InputBuffer()
    for _ in range(129):
        buffer.put("video", 10_000_000_000, b"jpeg")
    with pytest.raises(av.RecordingError) as error:
        buffer.get(0)
    assert error.value.code == "RECORD_BUFFER_OVERFLOW"


def test_timed_frames_repeat_missing_samples_instead_of_accelerating(tmp_path):
    for index, data in enumerate([b"zero", b"half", b"late"]):
        (tmp_path / ("%06d.jpg" % index)).write_bytes(data)
    pattern, info = av.prepare_frames(tmp_path, [(0, "000000.jpg"), (.5, "000001.jpg"),
                                               (.91, "000002.jpg")], 1, 10, threading.Event())
    assert info["encoded_frames"] == 10
    assert info["repeated_frames"] == 8  # .91 is after the last output tick.
    assert [Path(str(pattern) % i).read_bytes() for i in range(10)] == [b"zero"] * 5 + [b"half"] * 5


def test_audio_timeline_trims_overlap_and_rejects_long_gap():
    with wave.open(io.BytesIO(), "wb") as wav:
        timeline = av.AudioTimeline(wav, 0, 1)
        timeline.add(100_000_000, pcm(1600))
        timeline.add(180_000_000, pcm(1600))
        assert timeline.trimmed == 320
        with pytest.raises(av.RecordingError, match="250 ms"):
            timeline.add(800_000_000, pcm(1600))


def make_plugin(monkeypatch, tmp_path, mic_enabled=True):
    module, _ = _load_device(monkeypatch)
    executor = types.SimpleNamespace(add_node=lambda node: None, remove_node=lambda node: None)
    camera = types.SimpleNamespace(is_running=lambda: True, _color_topic="/robot/camera/color")
    plugin = module.VisionCapturePlugin({"output_dir": str(tmp_path)}, camera,
                                      namespace="robot", executor=executor, mic_enabled=mic_enabled)
    monkeypatch.setattr(module.shutil, "which", lambda name: "ffmpeg")
    return plugin, module


def test_schema_and_queued_audio_option_are_real(monkeypatch, tmp_path):
    plugin, module = make_plugin(monkeypatch, tmp_path)
    from unittest.mock import Mock
    monkeypatch.setattr(module.threading, "Thread", Mock())
    schema = plugin.get_tool()["inputSchema"]
    assert schema["properties"]["include_audio"]["default"] is False
    assert "include_audio" in schema["x-action-params"]["record_video"]["params"]
    assert schema["x-completion"]["timeout"] >= 75
    result = plugin.dispatch("record_video", {"duration_s": 10, "include_audio": True})
    assert result["state"] == "queued" and result["include_audio"] is True
    assert plugin._active_recording["include_audio"] is True
    assert plugin._video_topic == "/robot/camera/color"
    assert plugin._audio_topic == "/robot/mic/audio_recording"
    assert plugin.dispatch("record_video", {})["code"] == "RECORD_IN_PROGRESS"


def test_actuator_start_is_marker_but_business_action_checks_camera(monkeypatch, tmp_path):
    plugin, _ = make_plugin(monkeypatch, tmp_path)
    plugin._worker = None
    assert plugin.start() == {"state": "ready"}
    assert plugin.dispatch("start", {}) == {"state": "ready"}
    assert plugin.dispatch("capture_photo", {})["ok"] is False
    assert plugin.dispatch("record_video", {})["ok"] is False


def test_canvas_stop_returns_idle_without_joining_and_keeps_slot_until_cleanup(monkeypatch, tmp_path):
    plugin, module = make_plugin(monkeypatch, tmp_path)
    from unittest.mock import Mock
    worker, process = Mock(), Mock()
    active = {"action_id": "pending", "thread": worker, "process": process,
              "state": "recording", "cancel_event": threading.Event()}
    plugin._active_recording = active
    result = plugin.dispatch("stop", {})
    assert result["state"] == "idle" and result["cancellation_pending"] is True
    assert result["recording_state"] == "stopping"
    assert active["cancel_event"].is_set()
    worker.join.assert_not_called()
    process.terminate.assert_not_called()
    assert plugin._active_recording is active
    assert plugin.dispatch("record_video", {})["code"] == "RECORD_IN_PROGRESS"
    # Starting a canvas again must never undo the cancellation request.
    assert plugin.dispatch("start", {})["state"] == "ready"
    assert active["cancel_event"].is_set()
    notify = Mock()
    monkeypatch.setattr(module, "_acp_notify", notify)
    plugin._finish_recording(active, "cancelled", plugin._cancelled_result())
    assert plugin._active_recording is None
    assert plugin._last_recording["status"] == "cancelled"
    notify.assert_called_once()


def test_bundle_shutdown_still_waits_for_recording_cleanup(monkeypatch, tmp_path):
    plugin, _ = make_plugin(monkeypatch, tmp_path)
    from unittest.mock import Mock
    worker, process = Mock(), Mock()
    process.stdin = None
    process.poll.return_value = 0
    worker.is_alive.return_value = False
    plugin._active_recording = {"action_id": "shutdown", "thread": worker,
                                "process": process, "cancel_event": threading.Event()}
    assert plugin.stop()["state"] == "idle"
    worker.join.assert_called_once_with(timeout=6)


def test_bundle_passes_executor_namespace_and_mic_availability(monkeypatch, tmp_path):
    import ast
    import sys
    module, _ = _load_device(monkeypatch)
    monkeypatch.setitem(sys.modules, "device", module)
    main_path = Path(__file__).resolve().parents[1] / "main.py"
    tree = ast.parse(main_path.read_text(encoding="utf-8"))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "BumiDeviceBundle")
    namespace = {"os": os}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(main_path), "exec"), namespace)
    executor = object()
    bundle = namespace["BumiDeviceBundle"]({"plugins": {"mic": {"enabled": True},
        "vision_capture": {"enabled": True, "output_dir": str(tmp_path)}}}, "custom", executor, None, object())
    recorder = next(plugin for plugin in bundle._plugins if isinstance(plugin, module.VisionCapturePlugin))
    assert recorder._executor is executor
    assert recorder._audio_topic == "/custom/mic/audio_recording"
    assert recorder._mic_enabled is True
    schema = next(tool for tool in bundle.get_all_tools() if tool["name"] == "vision_capture")
    assert "include_audio" in schema["inputSchema"]["properties"]


@pytest.mark.parametrize("value", [1, "true", None, [], {}])
def test_audio_option_type_is_validated(monkeypatch, tmp_path, value):
    plugin, _ = make_plugin(monkeypatch, tmp_path)
    assert plugin.dispatch("record_video", {"include_audio": value})["code"] == "INVALID_AUDIO_OPTION"


def test_mic_disabled_and_encoder_missing_fail_before_start(monkeypatch, tmp_path):
    plugin, module = make_plugin(monkeypatch, tmp_path, mic_enabled=False)
    assert plugin.dispatch("record_video", {"include_audio": True})["code"] == "AUDIO_UNAVAILABLE"
    monkeypatch.setattr(module.shutil, "which", lambda name: None)
    assert plugin.dispatch("record_video", {})["code"] == "ENCODER_UNAVAILABLE"


def test_configured_maximum_is_reflected_in_schema_and_default(monkeypatch, tmp_path):
    plugin, module = make_plugin(monkeypatch, tmp_path)
    from unittest.mock import Mock
    plugin._max_duration_s = 3
    monkeypatch.setattr(module.threading, "Thread", Mock())
    prop = plugin.get_tool()["inputSchema"]["properties"]["duration_s"]
    assert prop["maximum"] == prop["default"] == 3
    assert plugin.dispatch("record_video", {})["requested_duration_s"] == 3


def test_thread_start_failure_releases_recording_slot(monkeypatch, tmp_path):
    plugin, module = make_plugin(monkeypatch, tmp_path)
    from unittest.mock import Mock
    thread = Mock()
    thread.start.side_effect = RuntimeError("cannot start")
    monkeypatch.setattr(module.threading, "Thread", Mock(return_value=thread))
    assert plugin.dispatch("record_video", {})["code"] == "RECORD_START_FAILED"
    assert plugin._active_recording is None


def test_recording_error_releases_subscriber_and_reports_terminal_once(monkeypatch, tmp_path):
    plugin, module = make_plugin(monkeypatch, tmp_path)
    from unittest.mock import Mock
    node = Mock()
    plugin._executor = Mock()
    monkeypatch.setattr(av, "make_input_node", Mock(return_value=node))
    monkeypatch.setattr(av, "collect", Mock(side_effect=av.RecordingError("AUDIO_UNAVAILABLE", "no microphone")))
    notify = Mock()
    monkeypatch.setattr(module, "_acp_notify", notify)
    active = {"action_id": "test", "duration_s": 1, "include_audio": True,
              "cancel_event": threading.Event(), "process": None, "path": None}
    plugin._active_recording = active
    plugin._record_video_async(active, 1)
    plugin._finish_recording(active, "error", {})
    plugin._executor.remove_node.assert_called_once_with(node)
    node.destroy_node.assert_called_once()
    assert plugin._active_recording is None
    assert plugin._last_recording["code"] == "AUDIO_UNAVAILABLE"
    notify.assert_called_once()
    assert list((tmp_path / "videos").iterdir()) == []


@pytest.mark.parametrize("mode", ["error", "cancel", "timeout"])
def test_encoding_failure_cleans_owned_file_and_process(monkeypatch, tmp_path, mode):
    plugin, _ = make_plugin(monkeypatch, tmp_path)
    from unittest.mock import Mock
    node, process = Mock(), Mock()
    process.stdin = None
    process.poll.return_value = 1 if mode == "error" else None
    process.returncode = 1
    active = {"duration_s": 1, "include_audio": True, "cancel_event": threading.Event()}
    monkeypatch.setattr(av, "make_input_node", lambda *args: node)
    monkeypatch.setattr(av, "collect", lambda *args: ([(0, "source.jpg")], {}))
    monkeypatch.setattr(av, "prepare_frames", lambda *args: (tmp_path / "%06d.jpg", {}))

    def launch(command, **kwargs):
        Path(command[-1]).write_bytes(b"partial")
        if mode == "cancel":
            active["cancel_event"].set()
        return process

    monkeypatch.setattr(av.subprocess, "Popen", launch)
    monkeypatch.setattr(av, "ENCODE_TIMEOUT_S", 0 if mode == "timeout" else 30)
    result = av.record(plugin, active)
    assert result["code"] == {"error": "AV_ENCODE_FAILED", "cancel": "RECORD_CANCELLED",
                              "timeout": "ENCODE_TIMEOUT"}[mode]
    assert active["process"] is None
    assert list((tmp_path / "videos").iterdir()) == []
    if mode != "error":
        process.terminate.assert_called_once()


def test_stop_after_encode_before_notify_cannot_leave_success_file(monkeypatch, tmp_path):
    plugin, module = make_plugin(monkeypatch, tmp_path)
    path = tmp_path / "owned.mp4"
    path.write_bytes(b"finished")
    cancel = threading.Event()
    cancel.set()
    active = {"action_id": "race", "cancel_event": cancel, "path": str(path)}
    plugin._active_recording = active
    events = []
    monkeypatch.setattr(module, "_acp_notify", lambda *args: events.append(args))
    plugin._finish_recording(active, "completed", {"ok": True, "file_path": str(path)})
    assert not path.exists()
    assert events[0][1] == "cancelled"
    assert plugin._last_recording["code"] == "RECORD_CANCELLED"


def test_photo_still_uses_existing_camera_cache(monkeypatch, tmp_path):
    plugin, _ = make_plugin(monkeypatch, tmp_path)
    monkeypatch.setattr(plugin, "_frame", lambda: ({"data": b"photo jpeg", "timestamp_ms": av.time.time() * 1000}, 1))
    result = plugin.dispatch("capture_photo", {})
    assert result["ok"] and result["media_type"] == "photo"
    assert Path(result["file_path"]).read_bytes() == b"photo jpeg"


@pytest.mark.skipif(not shutil.which("ffmpeg"), reason="real FFmpeg required")
@pytest.mark.parametrize("include_audio", [False, True])
def test_real_ten_second_stopwatch_clock_with_camera_drops(monkeypatch, tmp_path, include_audio):
    from PIL import Image, ImageDraw
    duration, fps, flash_time = 10, 15, 3.0
    events = []
    # Deliberate loss and uneven frame cadence reproduce the original speed-up.
    for index in range(duration * fps):
        if index % 5 == 2:
            continue
        t = index / fps
        color = "white" if 3 <= t < 3.1 else "black"
        image = Image.new("RGB", (160, 96), color)
        ImageDraw.Draw(image).text((12, 40), "%05.2f sec / %d" % (t, int(t)), fill="red")
        output = io.BytesIO()
        image.save(output, format="JPEG")
        events.append((t, t + .01, "video", output.getvalue()))
    # An audible tone and impulse precisely at the visible flash.
    values = array("h", [int(1000 * math.sin(i * .15)) for i in range(duration * av.RATE)])
    values[round(flash_time * av.RATE):round(flash_time * av.RATE) + 16] = array("h", [30000] * 16)
    for index in range(duration * 25):
        end = (index + 1) * .04
        data = values[index * 640:(index + 1) * 640].tobytes()
        events.append((end, end, "audio", data))
    # Exercise the actual Bumi worker, private recorder and terminal notification.
    plugin, module = make_plugin(monkeypatch, tmp_path)
    from unittest.mock import Mock
    original_collect = av.collect
    monkeypatch.setattr(av, "collect", lambda buffer, directory, seconds, cancel, audio:
                        run_capture(monkeypatch, directory, events, seconds, audio, collector=original_collect))
    node = Mock()
    monkeypatch.setattr(av, "make_input_node", Mock(return_value=node))
    notify = Mock()
    monkeypatch.setattr(module, "_acp_notify", notify)
    active = {"action_id": "real-ffmpeg-test", "duration_s": duration, "include_audio": include_audio,
              "cancel_event": threading.Event(), "process": None, "path": None}
    plugin._active_recording = active
    plugin._record_video_async(active, duration)
    info = video_info = plugin._last_recording
    assert info["ok"] and info["status"] == "completed", info
    assert info["include_audio"] is include_audio
    assert plugin._active_recording is None
    notify.assert_called_once()
    node.destroy_node.assert_called_once()
    assert len(list((tmp_path / "videos").iterdir())) == 1  # no spool left behind
    output = Path(info["file_path"])
    decoded = subprocess.run(["ffmpeg", "-nostdin", "-v", "info", "-i", str(output),
                              "-map", "0:v:0", "-pix_fmt", "rgb24", "-f", "rawvideo", "-"],
                             check=True, capture_output=True)
    assert b"Duration: 00:00:10.00" in decoded.stderr
    size = 160 * 96 * 3
    assert len(decoded.stdout) // size == duration * fps
    flash_index = next(i for i in range(duration * fps)
                       if decoded.stdout[i * size] > 200)
    assert abs(flash_index / fps - flash_time) <= 1 / fps
    assert video_info["repeated_frames"] == 30
    assert info["frames"] < video_info["encoded_frames"]
    if include_audio:
        raw = subprocess.run(["ffmpeg", "-nostdin", "-v", "error", "-i", str(output),
                              "-map", "0:a:0", "-f", "s16le", "-"], check=True, capture_output=True).stdout
        samples = array("h", raw)
        peak = max(range(len(samples)), key=lambda i: abs(samples[i])) / av.RATE
        assert abs(peak - flash_time) < .02
        assert abs(peak - flash_index / fps) < .1
        assert info["audio_samples"] == duration * av.RATE
        evidence = os.environ.get("BUMI_AV_EVIDENCE_DIR")
        if evidence:
            directory = Path(evidence)
            directory.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(output, directory / "synthetic_stopwatch_10s.mp4")
            (directory / "timing.json").write_text(json.dumps({**info, **video_info,
                "file_path": str(directory / "synthetic_stopwatch_10s.mp4"),
                "duration_s": duration, "visible_flash_s": flash_index / fps,
                "audio_impulse_s": peak, "av_error_s": abs(peak - flash_index / fps),
                "source": "synthetic fixture, NOT Bumi hardware"}, indent=2), encoding="utf-8")
    else:
        assert b"Audio: " not in decoded.stderr
