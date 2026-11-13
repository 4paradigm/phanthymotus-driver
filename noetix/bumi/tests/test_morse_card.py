"""Contract and decoder tests for the Bumi ``morse`` card.

The card is imported straight from ``Morse.py``: it deliberately keeps ROS and
the vendor SDK out of module scope, so these tests need neither rclpy nor the
MediaController binary.
"""

import importlib.util
import sys
import time
import types
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / "Morse.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("bumi_morse_card", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def module():
    return _load_module()


class _FakeMedia:
    def __init__(self):
        self.streams = []
        self.enabled = None
        self.paused = 0
        self.set_calls = 0

    def set_external_custom_audio_data_to_playback_enable(self, enable):
        self.enabled = enable
        self.set_calls += 1

    def resume_audio_playback(self):
        pass

    def pause_audio_playback(self):
        self.paused += 1

    def publish_external_audio_playback_stream(self, stream):
        self.streams.append(stream)


def _install_fake_media(monkeypatch):
    class AudioStream:
        def __init__(self):
            self.channels = 0
            self.sample_rate = 0
            self.format = 0
            self.duration_ms = 0
            self.timestamp_us = 0
            self.audio_data = []

    monkeypatch.setitem(sys.modules, "mediacontrol_py",
                        types.SimpleNamespace(AudioStream=AudioStream))


def _wait_idle(card, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        info = card.dispatch("info", {})
        if info["state"] != "speaking":
            return info
        time.sleep(0.01)
    raise AssertionError("speech worker did not finish")


# ── Decoder ──────────────────────────────────────────────────────────────────

def test_every_character_round_trips(module):
    for character, code in module._ALPHABET.items():
        decoded = module.decode_morse(code)
        assert decoded["text"] == character
        assert decoded["letters"] == 1
        assert decoded["unknown"] == []


def test_decodes_hello_world(module):
    decoded = module.decode_morse(
        ".... . .-.. .-.. --- / .-- --- .-. .-.. -..")
    assert decoded["text"] == "HELLO WORLD"
    assert decoded["letters"] == 10
    assert decoded["normalized"].endswith("/ .-- --- .-. .-.. -..")


@pytest.mark.parametrize("text", [
    ".... .. / -... ..- -- ..",
    ".... .. | -... ..- -- ..",
    ".... ..  -... ..- -- ..",
    ".... ..\n\n-... ..- -- ..",
])
def test_word_separators(module, text):
    assert module.decode_morse(text)["text"] == "HI BUMI"


def test_full_width_characters(module):
    decoded = module.decode_morse("．．．．　．．")
    assert decoded["text"] == "HI"


def test_unknown_code_is_reported_and_marked(module):
    decoded = module.decode_morse(".... . .-.. .-.. --- .--..-")
    assert decoded["text"] == "HELLO?"
    assert decoded["letters"] == 5
    assert len(decoded["unknown"]) == 1
    entry = decoded["unknown"][0]
    assert entry["word"] == 1 and entry["letter"] == 6
    assert entry["code"] == ".--..-" and entry["reason"] == "unknown_code"


def test_invalid_symbol_is_reported(module):
    decoded = module.decode_morse(".... ..x")
    assert decoded["text"] == "H?"
    assert decoded["unknown"][0]["reason"] == "invalid_symbol"


def test_speakable_text_is_flite_safe(module):
    # flite gets its text inside an ffmpeg filtergraph: punctuation must not
    # reach it as syntax, and lowercase keeps all-caps words from being spelled.
    assert module.speakable_text("HELLO WORLD") == "hello world"
    assert module.speakable_text("A, B?") == "a comma b question mark"
    assert module.speakable_text("-") == "hyphen"
    assert module.speakable_text("A-B") == "a hyphen b"
    assert module.speakable_text("<>") == ""


@pytest.mark.parametrize("value", [None, 5, [], {}])
def test_non_string_input_is_rejected(module, value):
    with pytest.raises(module.MorseInputError):
        module.decode_morse(value)


@pytest.mark.parametrize("value", ["", "   ", "/", "| |"])
def test_empty_input_is_rejected(module, value):
    with pytest.raises(module.MorseInputError):
        module.decode_morse(value)


def test_overlong_input_is_rejected(module):
    with pytest.raises(module.MorseInputError):
        module.decode_morse("." * (module.MAX_MORSE_CHARS + 1))


def test_overlong_translation_is_rejected(module):
    with pytest.raises(module.MorseInputError):
        module.decode_morse(" ".join(["...."] * (module.MAX_TEXT_CHARS + 1)))


# ── Card contract ────────────────────────────────────────────────────────────

def test_tool_contract(module):
    card = module.MorsePlugin({}, "robot", None, _FakeMedia())
    tool = card.get_tool()
    assert tool["name"] == "morse"
    assert tool["type"] == "actuator"
    assert tool["multiInstance"] is False
    assert tool["topic_out"] == []
    schema = tool["inputSchema"]
    assert schema["required"] == ["action"]
    assert schema["properties"]["action"]["enum"] == [
        "speak", "decode", "stop", "info", "start"]
    assert set(schema["x-action-params"]) == set(
        schema["properties"]["action"]["enum"])
    assert schema["x-action-params"]["speak"]["params"] == ["morse"]
    assert schema["properties"]["morse"]["maxLength"] == module.MAX_MORSE_CHARS
    assert card.get_tools() == [tool]


def test_config_validates_voice_and_speed(module):
    card = module.MorsePlugin({"voice": "kal", "speed": 99},
                              "robot", None, _FakeMedia())
    assert card._voice == "kal"
    assert card._speed == 2.0
    card = module.MorsePlugin({"voice": "--*", "speed": "fast"},
                              "robot", None, _FakeMedia())
    assert card._voice == "slt"
    assert card._speed == 1.0


def test_decode_action_needs_no_tts(module):
    card = module.MorsePlugin({}, "robot", None, None)
    result = card.dispatch("decode", {"morse": ".... .."})
    assert result["state"] == "decoded"
    assert result["text"] == "HI"


def test_decode_action_reports_bad_input(module):
    card = module.MorsePlugin({}, "robot", None, None)
    result = card.dispatch("decode", {"morse": ""})
    assert result["state"] == "error"
    assert result["code"] == "INVALID_MORSE"


def test_speak_refuses_unknown_codes(module):
    card = module.MorsePlugin({}, "robot", None, _FakeMedia())
    result = card.dispatch("speak", {"morse": ".-.-.-.-.-"})
    assert result["state"] == "invalid_morse"
    assert "未播放" in result["message"]


def test_speak_without_tts_is_unavailable(module):
    card = module.MorsePlugin({}, "robot", None, _FakeMedia())
    card._ffmpeg = None
    result = card.dispatch("speak", {"morse": ".... .."})
    assert result["state"] == "unavailable"
    assert "flite" in result["message"]


def test_speak_streams_decoded_text(module, monkeypatch):
    _install_fake_media(monkeypatch)
    media = _FakeMedia()
    card = module.MorsePlugin({}, "robot", None, media)
    monkeypatch.setattr(card, "_tts_ready", lambda: True)
    monkeypatch.setattr(card, "_synthesize",
                        lambda text: b"\x00\x00" * 6400)  # 0.2 s stereo

    result = card.dispatch("speak", {"morse": ".... . .-.. .-.. ---"})
    assert result["state"] == "speaking"
    assert result["text"] == "HELLO"

    info = _wait_idle(card)
    assert info["state"] == "idle"
    assert info["last"]["state"] == "completed"
    assert info["last"]["text"] == "HELLO"
    assert media.enabled is True
    assert media.set_calls == 1
    assert len(media.streams) == 2
    assert all(stream.channels == 2 for stream in media.streams)
    assert all(stream.sample_rate == 16000 for stream in media.streams)
    assert all(stream.format == 2 for stream in media.streams)
    assert all(stream.duration_ms == 100 for stream in media.streams)
    assert all(len(stream.audio_data) == 3200 for stream in media.streams)


def test_each_utterance_rewrites_the_playback_route(module, monkeypatch):
    # The speaker card disables external playback when it stops, so a cached
    # "route already open" flag would leave this card silent afterwards.
    _install_fake_media(monkeypatch)
    media = _FakeMedia()
    card = module.MorsePlugin({}, "robot", None, media)
    monkeypatch.setattr(card, "_tts_ready", lambda: True)
    monkeypatch.setattr(card, "_synthesize",
                        lambda text: b"\x00\x00" * 6400)

    card.dispatch("speak", {"morse": ".... .."})
    _wait_idle(card)
    card.dispatch("speak", {"morse": ".... .."})
    _wait_idle(card)
    assert media.set_calls == 2


def test_stop_cancels_playback(module, monkeypatch):
    _install_fake_media(monkeypatch)
    media = _FakeMedia()
    card = module.MorsePlugin({}, "robot", None, media)
    monkeypatch.setattr(card, "_tts_ready", lambda: True)
    monkeypatch.setattr(card, "_synthesize",
                        lambda text: b"\x00\x00" * (6400 * 20))  # 2 s

    card.dispatch("speak", {"morse": ".... .."})
    time.sleep(0.05)
    result = card.dispatch("stop", {})
    assert result == {"state": "idle", "stopped": True}
    info = card.dispatch("info", {})
    assert info["state"] == "idle"
    assert info["last"]["state"] == "stopped"
    assert media.paused == 1
    assert len(media.streams) < 20


def test_stop_without_speech_is_a_no_op(module):
    media = _FakeMedia()
    card = module.MorsePlugin({}, "robot", None, media)
    assert card.dispatch("stop", {}) == {"state": "idle", "stopped": False}
    assert media.paused == 0


def test_info_reports_missing_tts(module):
    card = module.MorsePlugin({}, "robot", None, None)
    card._ffmpeg = None
    info = card.dispatch("info", {})
    assert info["state"] == "unavailable"
    assert info["tts"]["available"] is False
    assert info["message"]
