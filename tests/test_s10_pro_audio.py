from __future__ import annotations

import importlib.util
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "deep_robotics" / "s10_pro"


def _load_audio(monkeypatch):
    monkeypatch.syspath_prepend(str(DRIVER))
    spec = importlib.util.spec_from_file_location("s10_pro_audio_for_test", DRIVER / "audio.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_audio_cards_advertise_asr_and_tts_contracts(monkeypatch):
    audio = _load_audio(monkeypatch)
    mic = audio.S10MicPlugin({"audio": {}}, "s10_pro", nodes=None)
    speaker = audio.S10SpeakerPlugin({"audio": {}, "ros": {"core_domain_id": None}})

    mic_tool = mic.get_tool()
    speaker_tool = speaker.get_tool()
    assert mic_tool["name"] == "mic"
    assert mic_tool["topic_out"] == [{
        "topic": "/s10_pro/mic/audio",
        "format": "audio/pcm-16k",
        "message_type": "audio_msgs/msg/AudioChunk",
    }]
    assert speaker_tool["topic_in"] == [{
        "format": "audio/pcm-16k",
        "message_type": "audio_msgs/msg/AudioChunk",
    }]
    assert speaker_tool["inputSchema"]["x-resource"] == "mouth"
    assert mic.dispatch("start", {})["state"] == "error"
    assert speaker.dispatch("start", {})["error"] == "input_topic is required"
    assert mic.dispatch("unknown", {}) is None
    assert speaker.dispatch("unknown", {}) is None


def test_audio_device_names_and_pcm_commands(monkeypatch):
    audio = _load_audio(monkeypatch)
    assert audio._pcm_card("default") == "default"
    assert audio._pcm_card("1,0") == "plughw:1,0"
    assert audio._pcm_card("hw:1,0") == "hw:1,0"
    assert audio._capture_command("default") == [
        "arecord", "-q", "-D", "default", "-t", "raw", "-f", "S16_LE",
        "-r", "16000", "-c", "2",
    ]
    interleaved = [sample for pair in zip([100] * 512, [-20] * 512) for sample in pair]
    assert audio._right_channel_pcm(audio.struct.pack("<1024h", *interleaved)) == audio.struct.pack(
        "<512h", *([-20] * 512))
    capture_setup = audio._capture_launch_command("default")
    assert capture_setup[:2] == ["sh", "-lc"]
    assert "Left Line Mux' MicL" in capture_setup[2]
    assert "Right Line Mux' MicR" in capture_setup[2]
    assert "Left PGA Mux' DifferentialL" in capture_setup[2]
    assert "Left Channel Capture Volume' 4" in capture_setup[2]
    assert "Right Channel Capture Volume' 4" in capture_setup[2]
    assert "case \"$gate\" in" in capture_setup[2]
    assert "off) : ;;" in capture_setup[2]
    assert "ALC Capture NG Switch' off" in capture_setup[2]
    assert "arecord -q" in capture_setup[2]
    assert audio._playback_command("default")[0:4] == ["aplay", "-q", "-D", "default"]


def test_mic_capture_uses_bounded_latest_only_queue_and_reports_drops(monkeypatch):
    audio = _load_audio(monkeypatch)
    queue_args = {}
    dropped = {"value": 7}

    class Bridge:
        def create_publisher(self, lane, topic, message_type, **kwargs):
            queue_args.update(lane=lane, topic=topic, message_type=message_type, **kwargs)
            return SimpleNamespace(publish=lambda _message: None)

        def stats(self, lane):
            assert lane == "mic_audio"
            return {"dropped": dropped["value"]}

    class Process:
        def poll(self):
            return None

    class WorkerThread:
        def __init__(self, *_args, **_kwargs):
            pass

        def start(self):
            pass

    message_module = SimpleNamespace(AudioChunk=object)
    monkeypatch.setitem(sys.modules, "audio_msgs", SimpleNamespace(msg=message_module))
    monkeypatch.setitem(sys.modules, "audio_msgs.msg", message_module)
    monkeypatch.setattr(audio.shutil, "which", lambda _name: "/usr/bin/arecord")
    monkeypatch.setattr(audio.subprocess, "Popen", lambda *_args, **_kwargs: Process())
    monkeypatch.setattr(audio.threading, "Thread", WorkerThread)
    monkeypatch.setattr(audio, "_audio_launch", lambda *_args: (["arecord"], None))

    plugin = audio.S10MicPlugin({"audio": {}}, "s10_pro", nodes=SimpleNamespace(bridge=Bridge()))
    plugin.mode = "local"
    assert plugin._start_capture()["state"] == "running"
    assert queue_args["queue_depth"] == 4
    assert queue_args["latest_only"] is True

    dropped["value"] = 10
    status = plugin.dispatch("info", {})
    assert status["frames_dropped"] == 3
    assert status["capture_timestamp_trusted"] is False
    assert status["pcm_peak"] == 0
    assert status["pcm_rms"] == 0.0
    assert status["pcm_window_ms"] == 0
    assert status["pcm_peak_window"] == 0


def test_mic_capture_stamps_frames_from_pipe_arrival(monkeypatch):
    audio = _load_audio(monkeypatch)

    class AudioChunk:
        def __init__(self):
            self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))

    message_module = SimpleNamespace(AudioChunk=AudioChunk)
    monkeypatch.setitem(sys.modules, "audio_msgs", SimpleNamespace(msg=message_module))
    monkeypatch.setitem(sys.modules, "audio_msgs.msg", message_module)

    class Stdout:
        def __init__(self):
            left = [100] * 512
            right = [1] * 512
            interleaved = [sample for pair in zip(left, right) for sample in pair]
            self.chunks = [audio.struct.pack("<1024h", *interleaved), b""]

        def read(self, _size):
            return self.chunks.pop(0)

    class Stderr:
        def read(self, _size):
            return b""

    class Process:
        def __init__(self):
            self.stdout = Stdout()
            self.stderr = Stderr()
            self.returncode = 0

        def poll(self):
            return 0

    published = []
    plugin = audio.S10MicPlugin({"audio": {}}, "s10_pro", nodes=None)
    plugin._process = Process()
    plugin._publisher = SimpleNamespace(publish=published.append)
    plugin._capture_loop()

    assert len(published) == 1
    assert published[0].format == audio.AUDIO_FORMAT
    assert len(published[0].data) == 1024
    assert plugin._pcm_peak == 1
    assert plugin._pcm_rms == 1.0
    assert plugin._pcm_nonzero_ratio == 1.0
    info = plugin.dispatch("info", {})
    assert info["pcm_peak_window"] == 1
    assert info["pcm_rms_window"] == 1.0
    assert info["pcm_nonzero_ratio_window"] == 1.0
    assert info["pcm_window_ms"] == 32
    stamp = published[0].header.stamp
    assert stamp.sec > 0
    assert 0 <= stamp.nanosec < 1_000_000_000


def test_build_plugins_registers_audio_cards_for_pro_only(monkeypatch):
    monkeypatch.syspath_prepend(str(DRIVER))
    spec = importlib.util.spec_from_file_location("s10_pro_device_audio_test", DRIVER / "device.py")
    device = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(device)

    class Native:
        def __init__(self, _config):
            pass

    monkeypatch.setattr(device, "BasicServerClient", Native)
    pro = device.build_plugins({"model_variant": "pro"})
    standard = device.build_plugins({"model_variant": "standard"})
    assert {type(plugin).__name__ for plugin in pro} >= {"S10MicPlugin", "S10SpeakerPlugin"}
    expected_prefixes = {
        "S10LocoServoPlugin": "loco_servo",
        "S10ControlledSpatialPlugin": "controlled_spatial",
        "S10MicPlugin": "mic",
        "S10SpeakerPlugin": "speaker",
    }
    for plugin in pro:
        expected = expected_prefixes.get(type(plugin).__name__)
        if expected:
            assert plugin.PREFIX == expected
            assert plugin.get_tool()["name"] == expected
    assert not {type(plugin).__name__ for plugin in standard} & {"S10MicPlugin", "S10SpeakerPlugin"}


def test_speaker_discards_invalid_and_tts_eof_frames(monkeypatch):
    audio = _load_audio(monkeypatch)
    plugin = audio.S10SpeakerPlugin({"audio": {}})
    plugin._stop = threading.Event()
    plugin._process = SimpleNamespace(poll=lambda: None, stdin=SimpleNamespace(writes=[]))
    plugin._process.stdin.write = lambda payload: plugin._process.stdin.writes.append(payload)

    class Subscriber:
        def __init__(self):
            self.items = [
                (audio.AUDIO_FORMAT, b"\x00\x01" * 32),
                ("pcm_16k_16bit_mono", b"\x02\x03" * 32),
                ("audio/pcm-48k", b"\x00\x01" * 32),
                (audio.AUDIO_FORMAT, audio.AUDIO_EOF_MAGIC),
                (audio.AUDIO_FORMAT, b"\x00"),
                None,
            ]

        def get(self, timeout):
            item = self.items.pop(0)
            if item is None:
                plugin._stop.set()
            return item

        def is_alive(self):
            return True

    plugin._subscriber = Subscriber()
    plugin._playback_loop()
    assert plugin._process.stdin.writes == [b"\x00\x01" * 32, b"\x02\x03" * 32]
    assert plugin._frames == 2
    assert plugin._dropped == 3


def test_speaker_cleans_up_player_when_audio_subscriber_dies(monkeypatch):
    audio = _load_audio(monkeypatch)
    plugin = audio.S10SpeakerPlugin({"audio": {}})
    plugin._stop = threading.Event()

    class Stream:
        closed = False

        def close(self):
            self.closed = True

    class Process:
        returncode = None

        def __init__(self):
            self.stdin = Stream()
            self.stderr = Stream()
            self.terminated = False

        def poll(self):
            return self.returncode

        def terminate(self):
            self.terminated = True
            self.returncode = -15

        def wait(self, timeout):
            return self.returncode

    class Subscriber:
        stopped = False

        def is_alive(self):
            return False

        def stop(self):
            self.stopped = True

    process, subscriber = Process(), Subscriber()
    plugin._process, plugin._subscriber = process, subscriber
    plugin._topic = "/remote_control/mic"

    plugin._playback_loop()

    status = plugin.dispatch("info", {})
    assert status["state"] == "idle"
    assert status["input_topic"] is None
    assert status["last_error"] == "Core-domain audio subscriber exited"
    assert process.terminated
    assert process.stdin.closed and process.stderr.closed
    assert subscriber.stopped
