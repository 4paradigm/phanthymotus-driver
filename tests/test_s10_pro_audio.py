from __future__ import annotations

import importlib.util
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
    assert mic.dispatch("start", {})["state"] == "error"
    assert speaker.dispatch("start", {})["error"] == "input_topic is required"


def test_audio_device_names_and_pcm_commands(monkeypatch):
    audio = _load_audio(monkeypatch)
    assert audio._pcm_card("default") == "default"
    assert audio._pcm_card("1,0") == "plughw:1,0"
    assert audio._pcm_card("hw:1,0") == "hw:1,0"
    assert audio._capture_command("default") == [
        "arecord", "-q", "-D", "default", "-t", "raw", "-f", "S16_LE",
        "-r", "16000", "-c", "1",
    ]
    assert audio._playback_command("default")[0:4] == ["aplay", "-q", "-D", "default"]


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
