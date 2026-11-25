"""Pure-spec tests: decoding, beat tables, spec-owned choreography, honesty."""

from __future__ import annotations

import math
import struct
import sys
import wave
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from beat_performance_spec import (
    ACTION_COMMANDS,
    FRAME_SAMPLES,
    SAMPLE_RATE,
    BeatSpecError,
    DecodedAudio,
    build_choreography,
    decode_audio,
    detect_beats,
    iter_audio_frames,
    normalise_config,
    protection_result,
    status_payload,
    validate_audio_path,
)


def _pcm_at_beats(beat_times: list[float], duration_s: float,
                  *, amplitude: int = 14000, hit_s: float = 0.04) -> bytes:
    """A click track: short loud bursts at the given times, silence between."""
    samples = [0] * int(duration_s * SAMPLE_RATE)
    for beat in beat_times:
        start = int(beat * SAMPLE_RATE)
        for index in range(start, min(start + int(hit_s * SAMPLE_RATE), len(samples))):
            samples[index] = amplitude if (index - start) % 2 == 0 else -amplitude
    return struct.pack(f"<{len(samples)}h", *samples)


def _write_wav(path, pcm: bytes, *, channels: int = 1, rate: int = SAMPLE_RATE,
               width: int = 2) -> None:
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(channels)
        wav.setsampwidth(width)
        wav.setframerate(rate)
        wav.writeframes(pcm)


# ── file intake / decoding ───────────────────────────────────────────────────

def test_upload_paths_are_confined_to_the_driver_upload_dir(tmp_path):
    allowed = (str(tmp_path / "uploads"),)
    good = validate_audio_path(str(tmp_path / "uploads" / "2026-11-25" / "song.wav"), allowed)
    assert good == str(tmp_path / "uploads" / "2026-11-25" / "song.wav")
    for bad in ("/etc/passwd", str(tmp_path / "uploads" / ".." / "secret.pcm"),
                str(tmp_path / "downloads" / "song.mp3"), "song.wav", ""):
        with pytest.raises(BeatSpecError):
            validate_audio_path(bad, allowed)


def test_raw_pcm_is_decoded_as_16k_s16le_mono(tmp_path):
    pcm = _pcm_at_beats([0.5, 1.0], 2.0)
    path = tmp_path / "click.pcm"
    path.write_bytes(pcm)
    decoded = decode_audio(str(path), allowed_dirs=(str(tmp_path),))

    assert decoded.source == "pcm"
    assert decoded.sample_rate == SAMPLE_RATE
    assert decoded.pcm == pcm
    assert decoded.duration_s == pytest.approx(2.0)


def test_stereo_wav_is_downmixed_without_changing_the_sample_rate(tmp_path):
    mono = _pcm_at_beats([0.5], 1.0)
    samples = struct.unpack(f"<{len(mono) // 2}h", mono)
    stereo = struct.pack(f"<{len(samples) * 2}h",
                         *[value for s in samples for _ in range(2) for value in (s,)])
    path = tmp_path / "stereo.wav"
    _write_wav(path, stereo, channels=2)

    decoded = decode_audio(str(path), allowed_dirs=(str(tmp_path),))
    assert decoded.source == "wav"
    assert decoded.pcm == mono


def test_a_mislabelled_sample_rate_is_rejected_not_silently_resampled(tmp_path):
    path = tmp_path / "wrong_rate.wav"
    _write_wav(path, _pcm_at_beats([0.5], 1.0), rate=44100)
    with pytest.raises(BeatSpecError, match="44100"):
        decode_audio(str(path), allowed_dirs=(str(tmp_path),))


def test_an_oversized_duration_is_rejected_before_any_audio_is_published(tmp_path):
    path = tmp_path / "long.pcm"
    path.write_bytes(b"\x00\x00" * (SAMPLE_RATE * 5))
    with pytest.raises(BeatSpecError, match="longer than"):
        decode_audio(str(path), max_duration_s=1.0, allowed_dirs=(str(tmp_path),))


# ── beat detection ───────────────────────────────────────────────────────────

def test_click_track_beats_land_on_their_transients():
    expected = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5]
    audio = DecodedAudio(_pcm_at_beats(expected, 3.0), SAMPLE_RATE, 3.0, "pcm")
    beats, source = detect_beats(audio)

    assert source == "detected"
    assert len(beats) == len(expected)
    assert all(abs(beat.time_s - want) <= 0.02
               for beat, want in zip(beats, expected))


def test_silence_uses_a_declared_fallback_metronome_instead_of_pretending_detection():
    audio = DecodedAudio(b"\x00\x00" * (SAMPLE_RATE * 2), SAMPLE_RATE, 2.0, "pcm")
    beats, source = detect_beats(audio, fallback_bpm=120.0)

    assert source == "fallback_bpm"
    assert len(beats) == 4
    assert [beat.time_s for beat in beats] == [0.0, 0.5, 1.0, 1.5]


def test_adjacent_transients_are_one_beat_not_a_burst_of_commands():
    # Two clicks 40 ms apart must not create two action edges.
    audio = DecodedAudio(_pcm_at_beats([0.5, 0.54, 1.5, 2.5, 3.5], 4.0),
                         SAMPLE_RATE, 4.0, "pcm")
    beats, source = detect_beats(audio)
    assert source == "detected"
    assert all(b.time_s - a.time_s >= 0.25 for a, b in zip(beats, beats[1:]))


# ── choreography ─────────────────────────────────────────────────────────────

def test_action_order_and_beat_relationship_come_from_the_spec():
    config = normalise_config({
        "action_cycle": "cheer,wave",
        "action_every_beats": 2,
        "turn_every_beats": 4,
        "action_cooldown_s": 0.5,
        "turn_cooldown_s": 0.5,
        "turn_vyaw": 0.5,
    })
    beats = [type("Beat", (), {"time_s": i * 0.5, "strength": 100.0})()
             for i in range(8)]
    events = build_choreography(beats, config)
    actions = [(e.time_s, e.action, e.command) for e in events if e.kind == "semantic_action"]
    turns = [(e.time_s, e.vyaw, e.duration_ms) for e in events if e.kind == "turn_pulse"]

    assert actions == [
        (0.5, "cheer", "CHEER"), (1.5, "wave", "SWING"),
        (2.5, "cheer", "CHEER"), (3.5, "wave", "SWING"),
    ]
    assert [turn[0] for turn in turns] == [1.5, 3.5]
    assert all(turn[1] == 0.5 and turn[2] == 60 for turn in turns)


def test_cooldown_suppresses_actions_on_back_to_back_beats():
    config = normalise_config({"action_every_beats": 1, "action_cooldown_s": 0.5})
    beats = [type("Beat", (), {"time_s": i * 0.2, "strength": 100.0})()
             for i in range(10)]
    actions = [event for event in build_choreography(beats, config)
               if event.kind == "semantic_action"]
    assert [event.time_s for event in actions] == [
        pytest.approx(0.0), pytest.approx(0.6),
        pytest.approx(1.2), pytest.approx(1.8),
    ]


def test_zero_yaw_disables_turn_pulses():
    config = normalise_config({"turn_every_beats": 1, "turn_vyaw": 0.0})
    beats = [type("Beat", (), {"time_s": i * 0.5, "strength": 1.0})() for i in range(4)]
    assert all(event.kind != "turn_pulse" for event in build_choreography(beats, config))


def test_config_out_of_range_is_a_value_error_not_a_silent_clamp():
    for bad in ({"action_every_beats": 0}, {"turn_vyaw": 2.0},
                {"frame_ms": 5}, {"action_cycle": "wave,not_an_action"},
                {"fallback_bpm": 1000}):
        with pytest.raises(BeatSpecError):
            normalise_config(bad)


# ── framing, safety payloads, dashboard contract ─────────────────────────────

def test_audio_frames_have_a_fixed_duration_and_the_tail_is_padded():
    pcm = b"\x01\x00" * (FRAME_SAMPLES + 7)
    frames = list(iter_audio_frames(pcm))
    assert len(frames) == 2
    assert all(len(frame) == FRAME_SAMPLES * 2 for frame in frames)
    assert frames[1].endswith(b"\x00\x00")


def test_protection_mode_answer_admits_no_command_was_sent():
    result = protection_result("beat_performance", 26)
    assert result["protection"] is True
    assert result["command_sent"] is False
    assert result["current_workmode"] == 26
    assert result["state"] == "error"


def test_status_payload_is_flat_and_reports_unknown_as_null():
    payload = status_payload(
        state="idle", audio_file=None, beat_count=0, beat_source="none",
        event_count=0, events_fired=0, action_count=0, turn_count=0,
        frames_published=0, duration_s=0.0, elapsed_s=0.0,
        last_action=None, last_action_drift_ms=None,
        workmode=None, protection=False, last_error=None,
    )
    assert all(not isinstance(value, (dict, list)) for value in payload.values())
    assert payload["workmode"] is None
    assert payload["last_action_drift_ms"] is None
    assert math.isfinite(payload["duration_s"])
