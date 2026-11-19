"""Traffic-light speech -> 16 kHz mono S16LE PCM bytes.

Three short fixed phrases (one per light color) are spoken through the speaker
card. The primary source is pre-recorded WAVs bundled with the card; without an
asset the card falls back to a per-color chime (red=low, yellow=mid, green=high)
so it still signals audibly and logs that the speech asset is missing.

Run ``scripts/generate_speech_wav.py`` once (needs ``edge-tts`` + ``ffmpeg`` +
network) to produce the WAV assets.
"""

from __future__ import annotations

import math
from pathlib import Path

SAMPLE_RATE = 16000
# AudioChunk chunking for the ROS wire (bytes per message = 128 ms @ 16 kHz).
CHUNK_BYTES = 4096

DEFAULT_TEXTS = {
    "red": "红灯，请等待",
    "yellow": "黄灯，请注意",
    "green": "绿灯，请通行",
}

# Fallback chime tone per color (Hz), so a blind user can still distinguish
# states without the pre-recorded speech assets.
_CHIME_FREQS = {"red": 440.0, "yellow": 660.0, "green": 880.0}


def resample(x, sr_old, sr_new):
    """Linear resample — good enough for a short, fixed spoken phrase."""
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


def make_chime(duration_s=0.6, volume=0.5, freq=660.0) -> bytes:
    """A short single-tone chime (sine), 16 kHz mono S16LE."""
    import numpy as np

    n = int(round(duration_s * SAMPLE_RATE))
    t = np.linspace(0.0, duration_s, n, endpoint=False)
    attack = np.clip(t / 0.02, 0.0, 1.0)
    release = np.clip((duration_s - t) / 0.08, 0.0, 1.0)
    env = np.minimum(attack, release)
    tone = (np.sin(2 * math.pi * freq * t) * env).astype(np.float32)
    pcm = tone * (volume * 32767.0)
    return np.clip(pcm, -32768.0, 32767.0).astype("<i2").tobytes()


class SpeechSynthesizer:
    """Turn a light color into PCM. WAV first, chime as the last resort."""

    def __init__(self, wav_paths=None, texts=None):
        self._wav_paths = dict(wav_paths or {})
        self.texts = dict(texts or DEFAULT_TEXTS)
        self._cache: dict[str, bytes] = {}
        self._used: dict[str, str] = {}

    def phrase(self, color: str) -> str:
        """The spoken phrase for a color (used in the JSON event text too)."""
        return self.texts.get(color, color)

    def source(self, color: str) -> str | None:
        """Which backend produced the cached PCM: ``"wav"`` or ``"chime"``."""
        return self._used.get(color)

    def synthesize(self, color: str) -> bytes:
        if color in self._cache:
            return self._cache[color]
        wav_path = self._wav_paths.get(color)
        if wav_path and Path(wav_path).exists():
            pcm = load_wav_pcm(wav_path)
            self._used[color] = "wav"
        else:
            pcm = make_chime(freq=_CHIME_FREQS.get(color, 660.0))
            self._used[color] = "chime"
        self._cache[color] = pcm
        return pcm

    def reset(self) -> None:
        self._cache = {}
        self._used = {}
