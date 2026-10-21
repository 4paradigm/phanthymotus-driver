from __future__ import annotations

import importlib.util
import ctypes
import io
import json
from pathlib import Path
import sys
import threading
import types

import pytest

from playback_telemetry import (
    PlaybackTracker,
    decode_packet,
    encode_packet,
    parse_audio_format,
)
from q5_media_bridge import BridgeWorker, speaker_chunk_from_message


class _Message:
    def __init__(self, audio_format, data=b"\x00\x01"):
        self.format = audio_format
        self.data = data


def _load_legacy_device(monkeypatch):
    modules = {
        "legacy_direct_control": types.SimpleNamespace(ArmControlPlugin=object),
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.action": types.SimpleNamespace(ActionClient=object),
        "rclpy.node": types.SimpleNamespace(Node=object),
        "rclpy.qos": types.SimpleNamespace(
            DurabilityPolicy=types.SimpleNamespace(VOLATILE=0),
            HistoryPolicy=types.SimpleNamespace(KEEP_LAST=0),
            ReliabilityPolicy=types.SimpleNamespace(RELIABLE=0, BEST_EFFORT=1),
            QoSProfile=lambda **kwargs: kwargs,
        ),
        "std_srvs": types.ModuleType("std_srvs"),
        "std_srvs.srv": types.SimpleNamespace(Trigger=object),
        "xbot_common_interfaces": types.ModuleType("xbot_common_interfaces"),
        "xbot_common_interfaces.action": types.SimpleNamespace(AudioPlay=object),
        "xbot_common_interfaces.srv": types.SimpleNamespace(SetVolume=object),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).with_name("legacy_device.py")
    spec = importlib.util.spec_from_file_location("legacy_device_telemetry_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bridge_preserves_legacy_pcm_and_extracts_valid_tracking_metadata():
    legacy = speaker_chunk_from_message(_Message("audio/pcm-16k", b"legacy"))
    assert legacy["pcm"] == b"legacy"
    assert legacy["metadata"].tracked is False

    item = speaker_chunk_from_message(_Message(
        "audio/pcm-16k;speech_id=A;sequence=2;eof=1", b"tracked"))
    assert item["pcm"] == b"tracked"
    assert item["metadata"].speech_id == "A"
    assert item["metadata"].sequence == 2
    assert item["metadata"].eof is True


def test_bridge_rejects_malformed_or_unsupported_audio_metadata():
    with pytest.raises(ValueError, match="sequence"):
        speaker_chunk_from_message(_Message(
            "audio/pcm-16k;speech_id=A;eof=1"))
    with pytest.raises(ValueError, match="unsupported audio format"):
        speaker_chunk_from_message(_Message("audio/wav"))


def test_bridge_accepts_standardized_playback_event_for_domain42_publish():
    bridge = BridgeWorker("q5")
    bridge.publish_speaker_event({"speech_id": "A", "event": "device_playback_completed"})
    command = bridge._cmd_q.get(timeout=1)
    assert command == {
        "kind": "speaker_event",
        "event": {"speech_id": "A", "event": "device_playback_completed"},
    }


class _Stdin:
    def __init__(self):
        self.data = bytearray()

    def write(self, data):
        self.data.extend(data)

    def flush(self):
        pass


def _plugin(module):
    plugin = module.SpeakerPlugin.__new__(module.SpeakerPlugin)
    plugin._process = types.SimpleNamespace(stdin=_Stdin())
    plugin._telemetry = PlaybackTracker()
    plugin._telemetry_enabled = True
    plugin._output_rate = 44100
    plugin._output_channels = 2
    plugin._rate = 16000
    plugin._channels = 1
    plugin._running = True
    plugin._backend_ready = threading.Event()
    plugin._backend_ready.set()
    plugin._event_topic = "/q5/speaker/playback_events"
    plugin._published_events = []
    plugin._client = types.SimpleNamespace(
        publish_speaker_event=plugin._published_events.append)
    return plugin


class _FakeAlsaFunction:
    def __init__(self, callback):
        self.callback = callback
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.callback(*args)


class _FakeAlsa:
    def __init__(self, write_results=(), delay_results=(), drain_result=0,
                 state_error=None, state_results=()):
        self.write_results = iter(write_results)
        self.delay_results = iter(delay_results)
        self.write_calls = 0
        self.state_error = state_error
        self.state_results = iter(state_results)
        self.snd_pcm_open = _FakeAlsaFunction(lambda *_args: 0)
        self.snd_pcm_set_params = _FakeAlsaFunction(lambda *_args: 0)
        self.snd_pcm_prepare = _FakeAlsaFunction(lambda *_args: 0)
        self.snd_pcm_recover = _FakeAlsaFunction(lambda *_args: -1)
        self.snd_pcm_state = _FakeAlsaFunction(lambda *_args: self._state())
        self.snd_pcm_close = _FakeAlsaFunction(lambda *_args: 0)
        self.snd_pcm_writei = _FakeAlsaFunction(self._write)
        self.snd_pcm_delay = _FakeAlsaFunction(self._delay)
        self.snd_pcm_drain = _FakeAlsaFunction(lambda *_args: drain_result)

    def _state(self):
        if self.state_error is not None:
            raise self.state_error
        try:
            return next(self.state_results)
        except StopIteration:
            return 3

    def _write(self, _pcm, _buffer, frames):
        self.write_calls += 1
        try:
            result = next(self.write_results)
        except StopIteration:
            return int(frames)
        return int(frames) if result == "all" else result

    def _delay(self, _pcm, delay_pointer):
        try:
            result = next(self.delay_results)
        except StopIteration:
            result = 0
        if result < 0:
            return result
        delay_pointer._obj.value = result
        return 0


def _packet(speech_id, sequence, eof):
    metadata = parse_audio_format(
        f"audio/pcm-16k;speech_id={speech_id};sequence={sequence};eof={int(eof)}")
    return encode_packet(metadata, b"\x00\x00" * 160)


def _run_remote_player(monkeypatch, command, fake_alsa, packets):
    fake_stdin = types.SimpleNamespace(buffer=io.BytesIO(b"".join(packets)))
    fake_stdout = io.StringIO()
    monkeypatch.setattr(ctypes, "CDLL", lambda _name: fake_alsa)
    monkeypatch.setattr(sys, "stdin", fake_stdin)
    monkeypatch.setattr(sys, "stdout", fake_stdout)
    exec(command, {"__name__": "__main__"})
    return [json.loads(line) for line in fake_stdout.getvalue().splitlines()]


def test_speaker_frames_structured_chunk_and_registers_receipt(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    plugin = _plugin(module)
    item = speaker_chunk_from_message(_Message(
        "audio/pcm-16k;speech_id=A;sequence=0;eof=1", b"\x00\x00" * 160))

    assert plugin._write_speaker_chunk(item)
    metadata, pcm = decode_packet(io.BytesIO(bytes(plugin._process.stdin.data)))
    assert metadata.speech_id == "A"
    assert metadata.eof is True
    assert pcm == b"\x00\x00" * 160
    assert plugin._telemetry.get_status("A")["state"] == "SPEAKER_RECEIVED"


def test_speaker_waits_for_backend_ready_before_registering_first_chunk(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    plugin = _plugin(module)
    plugin._backend_ready.clear()
    item = speaker_chunk_from_message(_Message(
        "audio/pcm-16k;speech_id=A;sequence=0;eof=1", b"\x00\x00" * 160))
    result = []
    thread = threading.Thread(target=lambda: result.append(
        plugin._write_speaker_chunk(item)))
    thread.start()

    assert plugin._telemetry.get_status("A")["state"] == "UNKNOWN"
    plugin._handle_remote_event({"event": "backend_ready", "generation": 10})
    thread.join(timeout=1)

    assert result == [True]
    assert plugin._telemetry.get_status("A")["state"] == "SPEAKER_RECEIVED"


def test_remote_events_drive_status_and_public_query_actions(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    plugin = _plugin(module)
    item = speaker_chunk_from_message(_Message(
        "audio/pcm-16k;speech_id=A;sequence=0;eof=1", b"\x00\x00" * 160))
    plugin._write_speaker_chunk(item)
    plugin._handle_remote_event({
        "event": "pcm_submitted", "speech_id": "A", "submitted_frames": 441,
        "total_output_frames": 441, "generation": 9, "timestamp_ns": 100,
        "device_sample_rate_hz": 44100,
    })
    plugin._handle_remote_event({
        "event": "device_progress", "speech_id": "A", "consumed_frames": 441,
        "generation": 9, "timestamp_ns": 200, "evidence": "snd_pcm_delay",
    })

    status = plugin.dispatch("get_playback_status", {"speech_id": "A"})
    waited = plugin.dispatch(
        "wait_for_completion", {"speech_id": "A", "timeout_seconds": 0})
    events = plugin.dispatch(
        "get_recent_events", {"speech_id": "A", "limit": 10})
    capabilities = plugin.dispatch("get_capabilities", {})
    assert status["state"] == "COMPLETED"
    assert waited["result"] == "completed"
    assert events["events"][-1]["event"] == "device_playback_completed"
    assert capabilities["acoustic_completion"] is False
    assert capabilities["event_topic"] == "/q5/speaker/playback_events"
    assert plugin._published_events[-1]["event"] == "device_playback_completed"
    published = len(plugin._published_events)
    plugin._handle_remote_event({
        "event": "device_progress", "speech_id": "A", "consumed_frames": 441,
        "generation": 9, "timestamp_ns": 300, "evidence": "snd_pcm_delay",
    })
    assert len(plugin._published_events) == published


def test_remote_failure_never_reports_completed(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    plugin = _plugin(module)
    item = speaker_chunk_from_message(_Message(
        "audio/pcm-16k;speech_id=A;sequence=0;eof=1", b"\x00\x00" * 160))
    plugin._write_speaker_chunk(item)
    plugin._handle_remote_event({
        "event": "failed", "speech_id": "A", "code": "ALSA_WRITE_FAILED",
        "message": "device disconnected", "generation": 1, "timestamp_ns": 10,
    })
    assert plugin.dispatch("get_playback_status", {"speech_id": "A"})["state"] == "FAILED"


def test_new_backend_generation_is_accepted_and_stale_failure_is_ignored(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    plugin = _plugin(module)
    plugin._handle_remote_event({"event": "backend_ready", "generation": 10})
    first = speaker_chunk_from_message(_Message(
        "audio/pcm-16k;speech_id=A;sequence=0;eof=1", b"\x00\x00" * 160))
    assert plugin._write_speaker_chunk(first)
    plugin._handle_remote_event({
        "event": "pcm_submitted", "speech_id": "A", "submitted_frames": 441,
        "total_output_frames": 441, "generation": 10, "timestamp_ns": 10,
    })
    plugin._telemetry.fail_active("PLAYBACK_STOPPED", "session stopped", "INTERRUPTED")

    plugin._handle_remote_event({"event": "backend_ready", "generation": 11})
    second = speaker_chunk_from_message(_Message(
        "audio/pcm-16k;speech_id=B;sequence=0;eof=1", b"\x00\x00" * 160))
    assert plugin._write_speaker_chunk(second)
    plugin._handle_remote_event({
        "event": "pcm_submitted", "speech_id": "B", "submitted_frames": 441,
        "total_output_frames": 441, "generation": 11, "timestamp_ns": 20,
    })
    plugin._handle_remote_event({
        "event": "failed", "speech_id": "B", "code": "OLD_FAILURE",
        "message": "late old session event", "generation": 10, "timestamp_ns": 30,
    })

    assert plugin._telemetry.get_status("B")["state"] == "PCM_SUBMITTED"


def test_remote_event_eof_fails_active_task_instead_of_hanging(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    plugin = _plugin(module)
    plugin._process = types.SimpleNamespace(stdin=_Stdin(), stdout=io.BytesIO(b""))
    item = speaker_chunk_from_message(_Message(
        "audio/pcm-16k;speech_id=A;sequence=0;eof=1", b"\x00\x00" * 160))
    plugin._write_speaker_chunk(item)

    plugin._read_remote_events(plugin._process)

    assert plugin._telemetry.get_status("A")["error_code"] == "REMOTE_TELEMETRY_ENDED"


def test_remote_progress_without_trusted_evidence_never_completes(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    plugin = _plugin(module)
    item = speaker_chunk_from_message(_Message(
        "audio/pcm-16k;speech_id=A;sequence=0;eof=1", b"\x00\x00" * 160))
    plugin._write_speaker_chunk(item)
    plugin._handle_remote_event({
        "event": "pcm_submitted", "speech_id": "A", "submitted_frames": 441,
        "total_output_frames": 441, "generation": 9, "timestamp_ns": 100,
        "device_sample_rate_hz": 44100,
    })
    plugin._handle_remote_event({
        "event": "device_progress", "speech_id": "A", "consumed_frames": 441,
        "generation": 9, "timestamp_ns": 200,
    })

    assert plugin.dispatch("get_playback_status", {"speech_id": "A"})["state"] != "COMPLETED"


def test_generated_remote_player_is_valid_python_and_uses_delay_observation(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    command = module._q5_alsa_speaker_command("hw:2,0", 44100, 2)
    compile(command, "<q5-remote-player>", "exec")
    assert "snd_pcm_delay" in command
    assert "snd_pcm_state" in command
    assert "snd_pcm_writei" in command
    assert "device_progress" in command


def test_generated_player_underrun_discards_same_speech_and_fails_buffered_tasks(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    fake = _FakeAlsa(write_results=["all", -32, "all"], delay_results=[-5])
    command = module._q5_alsa_speaker_command(
        "hw:2,0", 44100, 2, max_write_stalls=3, max_progress_failures=3)

    events = _run_remote_player(monkeypatch, command, fake, [
        _packet("A", 0, False),
        _packet("B", 0, True),
        _packet("B", 1, True),
        _packet("C", 0, True),
        _packet("B", 0, True),
    ])

    failures = {(event.get("speech_id"), event.get("event"), event.get("code"))
                for event in events}
    assert ("A", "underrun", "ALSA_UNDERRUN") in failures
    assert ("B", "underrun", "ALSA_UNDERRUN") in failures
    assert not any(event.get("speech_id") in {"A", "B"}
                   and event.get("event") == "device_progress" for event in events)
    assert any(event.get("speech_id") == "C"
               and event.get("evidence") in {"snd_pcm_delay", "snd_pcm_drain"}
               for event in events)
    assert fake.write_calls == 3


def test_generated_player_capacity_rejects_new_task_and_keeps_active_records(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    fake = _FakeAlsa()
    command = module._q5_alsa_speaker_command(
        "hw:2,0", 44100, 2, max_records=2)

    events = _run_remote_player(monkeypatch, command, fake, [
        _packet("A", 0, False),
        _packet("B", 0, False),
        _packet("C", 0, True),
    ])

    assert any(event.get("speech_id") == "C"
               and event.get("code") == "PLAYBACK_CAPACITY" for event in events)
    assert not any(event.get("speech_id") in {"A", "B"}
                   and event.get("event") in {"failed", "underrun"} for event in events)


def test_generated_player_tombstones_first_nonzero_sequence(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    fake = _FakeAlsa()
    command = module._q5_alsa_speaker_command("hw:2,0", 44100, 2)

    events = _run_remote_player(monkeypatch, command, fake, [
        _packet("A", 1, False),
        _packet("A", 0, True),
    ])

    assert any(event.get("speech_id") == "A"
               and event.get("code") == "CHUNK_SEQUENCE_ERROR" for event in events)
    assert fake.write_calls == 0


def test_generated_player_bounds_write_and_delay_failures(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    fake = _FakeAlsa(
        write_results=[-11, 0, -11, 441, 441],
        delay_results=[-5, -5],
        drain_result=-5,
    )
    command = module._q5_alsa_speaker_command(
        "hw:2,0", 44100, 2, max_write_stalls=3, max_progress_failures=2)

    events = _run_remote_player(monkeypatch, command, fake, [
        _packet("stalled", 0, True),
        _packet("delay", 0, False),
        _packet("delay", 1, True),
    ])

    assert any(event.get("speech_id") == "stalled"
               and event.get("code") == "ALSA_WRITE_STALLED" for event in events)
    assert any(event.get("speech_id") == "delay"
               and event.get("code") == "ALSA_DELAY_FAILED" for event in events)
    assert not any(event.get("event") == "device_progress" for event in events)


def test_generated_player_bounds_successful_delay_without_progress(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    fake = _FakeAlsa(delay_results=[100000])
    command = module._q5_alsa_speaker_command(
        "hw:2,0", 44100, 2, progress_timeout_seconds=0)

    events = _run_remote_player(
        monkeypatch, command, fake, [_packet("A", 0, True)])

    assert any(event.get("speech_id") == "A"
               and event.get("code") == "ALSA_PROGRESS_TIMEOUT" for event in events)
    assert not any(event.get("speech_id") == "A"
                   and event.get("event") == "device_progress" for event in events)


def test_generated_player_xrun_fails_buffered_tasks_without_waiting(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    fake = _FakeAlsa(state_results=[4])
    command = module._q5_alsa_speaker_command("hw:2,0", 44100, 2)

    events = _run_remote_player(
        monkeypatch, command, fake, [_packet("A", 0, False)])

    assert any(event.get("speech_id") == "A"
               and event.get("code") == "ALSA_UNDERRUN" for event in events)
    assert not any(event.get("speech_id") == "A"
                   and event.get("event") == "device_progress" for event in events)


def test_generated_player_drain_failure_never_emits_completion_evidence(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    fake = _FakeAlsa(delay_results=[1000], drain_result=-5)
    command = module._q5_alsa_speaker_command("hw:2,0", 44100, 2)

    events = _run_remote_player(
        monkeypatch, command, fake, [_packet("A", 0, True)])

    assert any(event.get("speech_id") == "A"
               and event.get("code") == "ALSA_DRAIN_FAILED" for event in events)
    assert not any(event.get("speech_id") == "A"
                   and event.get("evidence") == "snd_pcm_drain" for event in events)


def test_generated_player_abnormal_exit_fails_before_successful_drain(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    fake = _FakeAlsa(state_error=RuntimeError("state query crashed"))
    command = module._q5_alsa_speaker_command("hw:2,0", 44100, 2)

    events = _run_remote_player(
        monkeypatch, command, fake, [_packet("A", 0, True)])

    assert any(event.get("speech_id") == "A"
               and event.get("code") == "REMOTE_PLAYER_FAILED" for event in events)
    assert not any(event.get("speech_id") == "A"
                   and event.get("event") == "device_progress" for event in events)


def test_speaker_tool_declares_telemetry_actions(monkeypatch):
    module = _load_legacy_device(monkeypatch)
    plugin = module.SpeakerPlugin.__new__(module.SpeakerPlugin)
    plugin._volume = 100
    plugin._event_topic = "/q5/speaker/playback_events"
    actions = plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
    assert {"get_playback_status", "get_recent_events", "wait_for_completion",
            "get_capabilities"}.issubset(actions)
