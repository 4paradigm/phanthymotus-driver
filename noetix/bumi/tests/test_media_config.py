"""The media-config card's rules, exercised off a robot.

`media_config_spec` imports no `rclpy` precisely so these can run on a laptop.
What is worth pinning down is the judgment that would be silent if it broke:
the wake word is read-only (the SDK has no setter), a timeout is in milliseconds
not seconds, and a boolean flag is a real boolean rather than a truthy string.
"""
import sys
import types
from pathlib import Path

import pytest

from test_mic_card import _load_device

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import media_config_spec as spec  # noqa: E402


# ── the vocabulary: what is writable and what a caller may send ─────────────

def test_the_wake_word_is_read_only_because_the_sdk_locks_it():
    # The pybind module exposes `get_wakeup_words` and no `set_wakeup_words`;
    # advertising a set action here would be a no-op that errors on hardware.
    assert spec.is_writable("wakeup_words") is False
    assert spec.SETTINGS["wakeup_words"]["setter"] is None
    assert "wakeup_words" not in spec.WRITABLE_KEYS


def test_every_writable_setting_has_a_real_getter_and_setter_pair():
    for key in spec.WRITABLE_KEYS:
        assert spec.SETTINGS[key]["getter"].startswith("get_")
        assert spec.SETTINGS[key]["setter"].startswith("set_")


def test_writable_settings_cover_timeout_cue_and_response_words():
    assert set(spec.WRITABLE_KEYS) == {
        "timeout", "audio_cue", "wakeup_response", "sleep_response",
    }


# ── validation: the part where a wrong unit or type fails silently ──────────

def test_a_timeout_is_validated_in_milliseconds_not_seconds():
    # A caller that sends `5` meaning "5 s" would otherwise be accepted and the
    # robot would sleep after 5 ms. The guard returns the int the SDK wants.
    assert spec.validate_timeout_ms(120000) == 120000
    assert spec.validate_timeout_ms("90000") == 90000


def test_a_timeout_outside_the_range_is_rejected():
    with pytest.raises(ValueError):
        spec.validate_timeout_ms(0)
    with pytest.raises(ValueError):
        spec.validate_timeout_ms(3_600_001)


def test_a_boolean_timeout_is_not_a_duration():
    with pytest.raises(ValueError):
        spec.validate_timeout_ms(True)


def test_a_non_finite_timeout_is_rejected():
    with pytest.raises(ValueError):
        spec.validate_timeout_ms(float("nan"))
    with pytest.raises(ValueError):
        spec.validate_timeout_ms(float("inf"))


def test_response_words_must_be_a_non_empty_string():
    assert spec.validate_response_words(" 你好 ") == "你好"
    with pytest.raises(ValueError):
        spec.validate_response_words("")
    with pytest.raises(ValueError):
        spec.validate_response_words("   ")
    with pytest.raises(ValueError):
        spec.validate_response_words(123)


def test_response_words_have_a_sanity_cap():
    with pytest.raises(ValueError):
        spec.validate_response_words("x" * (spec.RESPONSE_WORDS_MAX_CHARS + 1))


def test_the_audio_cue_is_a_real_boolean_not_a_truthy_string():
    assert spec.validate_audio_cue(True) is True
    assert spec.validate_audio_cue(False) is False
    with pytest.raises(ValueError):
        spec.validate_audio_cue("true")
    with pytest.raises(ValueError):
        spec.validate_audio_cue(1)


def test_validate_dispatches_per_setting_and_rejects_unknown_keys():
    assert spec.validate("timeout", 5000) == 5000
    assert spec.validate("audio_cue", True) is True
    assert spec.validate("wakeup_response", "在") == "在"
    with pytest.raises(ValueError):
        spec.validate("wakeup_words", "任意")


# ── the interface ───────────────────────────────────────────────────────────

def test_the_interface_names_the_timeout_unit_and_range():
    interface = spec.interface()

    assert interface["card"] == "media_config"
    assert interface["timeout_unit"] == "milliseconds"
    assert interface["timeout_range_ms"] == [1, 3_600_000]
    keys = {setting["key"] for setting in interface["settings"]}
    assert "wakeup_words" in keys
    by_key = {setting["key"]: setting["writable"]
              for setting in interface["settings"]}
    assert by_key["wakeup_words"] is False
    assert by_key["timeout"] is True


def test_every_setting_has_a_chinese_label():
    for key in spec.SETTING_KEYS:
        assert spec.label(key) != key, key


# ── the plugin: what only breaks against a live SDK ─────────────────────────

class _Media:
    """A MediaController stand-in with the exact get/set surface this card uses."""

    def __init__(self, timeout_ms=120000, audio_cue=True,
                 wakeup_response="我在", sleep_response="好的",
                 wakeup_words="小悟"):
        self.timeout_ms = timeout_ms
        self.audio_cue = audio_cue
        self.wakeup_response = wakeup_response
        self.sleep_response = sleep_response
        self.wakeup_words = wakeup_words
        self.set_calls = []

    def get_timeout(self):
        return self.timeout_ms

    def set_timeout(self, value):
        self.set_calls.append(("timeout", value))
        self.timeout_ms = value

    def get_audio_cue_enable(self):
        return self.audio_cue

    def set_audio_cue_enable(self, value):
        self.set_calls.append(("audio_cue", value))
        self.audio_cue = value

    def get_wakeup_response_words(self):
        return self.wakeup_response

    def set_wakeup_response_words(self, value):
        self.set_calls.append(("wakeup_response", value))
        self.wakeup_response = value

    def get_sleep_response_words(self):
        return self.sleep_response

    def set_sleep_response_words(self, value):
        self.set_calls.append(("sleep_response", value))
        self.sleep_response = value

    def get_wakeup_words(self):
        return self.wakeup_words


def _plugin(monkeypatch, media):
    module, _ = _load_device(monkeypatch)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    plugin = module.MediaConfigPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda _: None), media)
    return module, plugin


def test_the_card_is_an_actuator_with_no_topic_out(monkeypatch):
    module, plugin = _plugin(monkeypatch, _Media())
    tool = plugin.get_tool()

    assert tool["name"] == "media_config"
    assert tool["type"] == "actuator"
    assert tool["multiInstance"] is False
    assert tool["topic_out"] == []
    # The wake word is read-only, so no set_wakeup_words action exists.
    enum = tool["inputSchema"]["properties"]["action"]["enum"]
    assert "set_timeout" in enum
    assert "set_wakeup_words" not in enum
    assert "set_audio_cue" in enum


def test_get_reads_every_setting_including_the_read_only_wake_word(monkeypatch):
    _, plugin = _plugin(monkeypatch, _Media())

    result = plugin.dispatch("get", {})

    assert result["controller_present"] is True
    assert result["values"]["timeout"] == 120000
    assert result["values"]["audio_cue"] is True
    assert result["values"]["wakeup_words"] == "小悟"


def test_set_writes_and_reads_back_the_new_value(monkeypatch):
    _, plugin = _plugin(monkeypatch, _Media())
    monkeypatch.setattr(plugin, "_wait_for_config_slot", lambda: None)

    result = plugin.dispatch("set_timeout", {"timeout": 60000})

    assert result["state"] == "set"
    assert result["readback"] == 60000


def test_a_set_of_an_out_of_range_timeout_is_rejected_before_writing(monkeypatch):
    _, plugin = _plugin(monkeypatch, _Media())
    monkeypatch.setattr(plugin, "_wait_for_config_slot", lambda: None)

    result = plugin.dispatch("set_timeout", {"timeout": 0})

    assert result["state"] == "error"
    assert "毫秒" in result["error"]


def test_a_set_of_a_boolean_cue_accepts_only_real_booleans(monkeypatch):
    _, plugin = _plugin(monkeypatch, _Media())
    monkeypatch.setattr(plugin, "_wait_for_config_slot", lambda: None)

    ok = plugin.dispatch("set_audio_cue", {"audio_cue": False})
    bad = plugin.dispatch("set_audio_cue", {"audio_cue": "false"})

    assert ok["state"] == "set"
    assert bad["state"] == "error"


def test_setting_the_read_only_wake_word_is_refused(monkeypatch):
    _, plugin = _plugin(monkeypatch, _Media())

    result = plugin.dispatch("set_wakeup_words", {"wakeup_words": "x"})

    assert result["state"] == "error"
    assert "只读" in result["error"]


def test_a_missing_argument_is_a_clear_error_not_a_key_error(monkeypatch):
    _, plugin = _plugin(monkeypatch, _Media())

    result = plugin.dispatch("set_timeout", {})

    assert result["state"] == "error"
    assert "timeout" in result["error"]


def test_without_a_controller_sets_are_refused_and_info_stays_honest(monkeypatch):
    _, plugin = _plugin(monkeypatch, None)

    assert plugin.dispatch("set_timeout", {"timeout": 5})["state"] == "error"
    info = plugin.dispatch("info", {})
    assert info["controller_present"] is False
    assert info["values"] is None
    assert plugin.dispatch("get", {}) == {"controller_present": False,
                                          "values": None}


def test_info_reads_back_all_values_without_waiting_for_a_topic(monkeypatch):
    _, plugin = _plugin(monkeypatch, _Media(timeout_ms=90000))

    info = plugin.dispatch("info", {})

    assert info["controller_present"] is True
    assert info["values"]["timeout"] == 90000
    assert info["values"]["wakeup_words"] == "小悟"
    assert info["interface"]["timeout_unit"] == "milliseconds"


def test_start_and_stop_answer_the_actuator_shape(monkeypatch):
    _, plugin = _plugin(monkeypatch, _Media())

    assert plugin.dispatch("start", {}) == {"state": "ready"}
    assert plugin.dispatch("stop", {}) == {"state": "idle"}


def test_an_unknown_action_is_none(monkeypatch):
    _, plugin = _plugin(monkeypatch, _Media())

    assert plugin.dispatch("nonsense", {}) is None


def test_the_spec_module_reaches_the_image():
    """The bundle copies modules into the image one by one; a spec on disk but
    not in the image passes every test and then fails to import on the robot."""
    dockerfile = (Path(__file__).resolve().parents[1] / "Dockerfile").read_text(
        encoding="utf-8")

    assert "COPY media_config_spec.py /work/media_config_spec.py" in dockerfile
