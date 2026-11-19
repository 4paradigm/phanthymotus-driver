"""Tests for the alarm-speech synthesis — no ROS, no YOLO."""

import sys
import wave
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # noetix/bumi

from fall_alarm.speech import (  # noqa: E402
    SAMPLE_RATE,
    load_wav_pcm,
    make_chime,
    SpeechSynthesizer,
)


def test_make_chime_is_16k_mono_s16le():
    pcm = make_chime(duration_s=0.3)
    assert len(pcm) % 2 == 0
    samples = np.frombuffer(pcm, dtype="<i2")
    assert len(samples) == int(0.3 * SAMPLE_RATE)
    assert int(samples.max()) > 0  # has amplitude
    assert int(samples.min()) < 0  # oscillates


def _write_wav(path, *, rate, channels, sampwidth, samples):
    with wave.open(str(path), "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(sampwidth)
        w.setframerate(rate)
        w.writeframes(samples.tobytes())


def test_load_wav_converts_stereo_8k_to_mono_16k(tmp_path):
    rate = 8000
    n = 1600  # 0.2 s
    t = np.linspace(0, 0.2, n, endpoint=False)
    left = (np.sin(2 * np.pi * 440 * t) * 12000).astype("<i2")
    right = (np.sin(2 * np.pi * 440 * t) * 12000).astype("<i2")
    interleaved = np.empty(2 * n, dtype="<i2")
    interleaved[0::2] = left
    interleaved[1::2] = right

    path = tmp_path / "stereo8k.wav"
    _write_wav(path, rate=rate, channels=2, sampwidth=2, samples=interleaved)

    pcm = load_wav_pcm(path)
    samples = np.frombuffer(pcm, dtype="<i2")
    assert len(samples) == int(n * SAMPLE_RATE / rate)  # resampled to 16 kHz
    assert np.abs(samples).max() > 5000


def test_synthesize_uses_wav_when_present(tmp_path):
    path = tmp_path / "fall_alarm.wav"
    mono = (np.sin(np.linspace(0, 10, 1600)) * 8000).astype("<i2")
    _write_wav(path, rate=SAMPLE_RATE, channels=1, sampwidth=2, samples=mono)

    synth = SpeechSynthesizer(wav_path=str(path))
    pcm = synth.synthesize()
    assert synth.source == "wav"
    assert np.frombuffer(pcm, dtype="<i2").shape[0] == 1600


def test_synthesize_falls_back_to_chime_when_wav_missing():
    synth = SpeechSynthesizer(wav_path="/nonexistent/fall_alarm.wav")
    pcm = synth.synthesize()
    assert synth.source == "chime"
    assert len(pcm) > 0
