"""Alarm speech -> 16 kHz mono S16LE PCM bytes.

The alarm phrase is fixed and short, so the primary source is a pre-recorded
WAV bundled with the card — deterministic, offline, and no TTS engine on the
robot. Two behaviours keep the reminder audible without that asset:

  1. a pre-recorded WAV at ``alarm_wav_path`` (primary);
  2. a short attention chime, synthesized here with no dependencies, so the
     card still makes a sound and logs a clear message that the speech asset
     is missing.

Run ``scripts/generate_alarm_wav.py`` once (needs ``edge-tts`` + ``ffmpeg`` +
network) to produce the WAV asset for any phrase.
"""

from __future__ import annotations

import math
from pathlib import Path

SAMPLE_RATE = 16000
ALARM_TEXT = "检测到有人摔倒，请注意安全"
# AudioChunk chunking for the ROS wire (bytes per message = 128 ms @ 16 kHz).
CHUNK_BYTES = 4096


def resample(x, sr_old, sr_new):
    """Linear resample — good enough for a short, fixed alarm phrase."""
    import numpy as np
    if sr_old == sr_new:
        return x
    n = max(1, int(round(len(x) * sr_new / sr_old)))
    idx = np.linspace(0.0, float(len(x) - 1), n)
    return np.interp(idx, np.arange(len(x), dtype=float), x)


def load_wav_pcm(path) -> bytes:
    """Load a WAV as 16 kHz mono S16LE PCM bytes, converting on the fly."""
    import wave
    import numpy as np

    with wave.open(str(path), "rb") as w:
        sr = w.getframerate()
        ch = w.getnchannels()
        sw = w.getsampwidth()
        raw = w.readframes(w.getnframes())

    if sw == 2:
        samples = np.frombuffer(raw, dtype="<i2").astype(np.float32)
    elif sw == 1:
        samples = (np.frombuffer(raw, dtype=np.uint8).astype(np.float32) - 128.0) * 256.0
    elif sw == 4:
        samples = (np.frombuffer(raw, dtype="<i4").astype(np.float32) / 32768.0)
    else:
        raise ValueError(f"unsupported WAV sample width {sw}")

    if ch > 1:
        samples = samples.reshape(-1, ch).mean(axis=1)

    samples = resample(samples, sr, SAMPLE_RATE)
    samples = np.clip(samples, -32768.0, 32767.0).astype("<i2")
    return samples.tobytes()


def make_chime(duration_s=0.9, volume=0.5) -> bytes:
    """A short three-tone ascending chime (sine), 16 kHz mono S16LE."""
    import numpy as np

    freqs = (660.0, 880.0, 1320.0)
    tone_s = duration_s / len(freqs)
    n = int(round(tone_s * SAMPLE_RATE))
    t = np.linspace(0.0, tone_s, n, endpoint=False)
    attack = np.clip(t / 0.02, 0.0, 1.0)
    release = np.clip((tone_s - t) / 0.06, 0.0, 1.0)
    env = np.minimum(attack, release)

    pieces = [(np.sin(2 * math.pi * f * t) * env).astype(np.float32) for f in freqs]
    pcm = np.concatenate(pieces) * (volume * 32767.0)
    return np.clip(pcm, -32768.0, 32767.0).astype("<i2").tobytes()


class SpeechSynthesizer:
    """Turn the alarm phrase into PCM. WAV first, chime as the last resort."""

    def __init__(self, wav_path=None, text=ALARM_TEXT):
        self.wav_path = wav_path
        self.text = text
        self._pcm = None
        self._used = None

    @property
    def source(self) -> str | None:
        """Which backend produced the cached PCM: ``"wav"`` or ``"chime"``."""
        return self._used

    def synthesize(self) -> bytes:
        if self._pcm is not None:
            return self._pcm
        if self.wav_path and Path(self.wav_path).exists():
            self._pcm = load_wav_pcm(self.wav_path)
            self._used = "wav"
        else:
            self._pcm = make_chime()
            self._used = "chime"
        return self._pcm

    def reset(self) -> None:
        self._pcm = None
        self._used = None
