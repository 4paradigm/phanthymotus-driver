"""Bumi head microphone delay estimation and two-position direction calibration."""

import math
from collections import deque

import numpy as np


class SoundActivityGate:
    """Accept sounds clearly louder than the recent microphone background."""

    def __init__(self):
        self._levels = deque(maxlen=50)
        self._rising_frames = 0
        self._hold_samples = 0

    def accepts(self, audio) -> bool:
        samples = np.asarray(audio, dtype=np.float32)
        if samples.size == 0 or samples.size % 8:
            return False
        head = samples.reshape(-1, 8)[:, :4]
        head = head - head.mean(axis=0)
        level = float(np.median(np.sqrt(np.mean(head * head, axis=0))))
        background = (float(np.percentile(self._levels, 20))
                      if len(self._levels) >= 6 else None)
        self._levels.append(level)
        if background is None:
            return False
        # 去直流后用连续两帧识别较轻的说话声；短暂停顿只保留状态，不输出底噪角度。
        rising = level >= max(10.0, background * 1.8)
        self._rising_frames = self._rising_frames + 1 if rising else 0
        if self._rising_frames >= 2 or level >= max(15.0, background * 3.0):
            self._hold_samples = 9600  # 允许约 0.6 秒语音停顿。
            return True
        if self._hold_samples > 0 and level >= max(10.0, background * 1.25):
            self._hold_samples = 9600
            return True
        self._hold_samples = max(0, self._hold_samples - len(head))
        return False


def is_voiced_audio(audio, channels: int, sample_rate: int) -> bool:
    """Reject silence, broad noise and a single tone before saving calibration."""
    samples = np.asarray(audio, dtype=np.float64)
    if channels != 8 or sample_rate != 16000 or samples.size < channels * 16000:
        return False
    voice = samples.reshape(-1, channels)[:, 0]
    voice = voice - voice.mean()
    if np.sqrt(np.mean(voice * voice)) < 10:
        return False
    power = np.abs(np.fft.rfft(voice * np.hanning(len(voice)))) ** 2
    frequencies = np.fft.rfftfreq(len(voice), 1 / sample_rate)
    band = power[(frequencies >= 100) & (frequencies <= 3500)]
    if band.sum() < 0.65 * power.sum():
        return False
    flatness = np.exp(np.mean(np.log(band + 1))) / np.mean(band + 1)
    return flatness < 0.45 and band.max() < 0.5 * band.sum()


def estimate_signature(audio, channels: int, sample_rate: int):
    """Return channels 1-3 delays against channel 0, in samples."""
    samples = np.asarray(audio, dtype=np.float64)
    if channels != 8 or sample_rate != 16000 or samples.size < channels * 1024:
        return None
    if samples.size % channels:
        return None
    head = samples.reshape(-1, channels)[:, :4]
    head = head - head.mean(axis=0)
    if np.min(np.sqrt(np.mean(head * head, axis=0))) < 10:
        return None

    window = np.hanning(len(head))[:, None]
    spectrum = np.fft.rfft(head * window, n=2 * len(head), axis=0)
    max_lag = min(round(sample_rate * 0.0015), len(head) - 1)
    delays = []
    for index in range(1, 4):
        cross = spectrum[:, index] * spectrum[:, 0].conj()
        correlation = np.fft.irfft(cross / np.maximum(np.abs(cross), 1e-12), n=2 * len(head))
        nearby = np.concatenate((correlation[-max_lag:], correlation[:max_lag + 1]))
        if np.max(nearby) < 0.12:
            return None
        peak = int(np.argmax(nearby))
        offset = 0.0
        if 0 < peak < len(nearby) - 1:
            left, middle, right = nearby[peak - 1:peak + 2]
            curvature = left - 2 * middle + right
            if abs(curvature) > 1e-12:
                offset = 0.5 * (left - right) / curvature
        delays.append(float(peak + offset - max_lag))
    return tuple(delays)


def estimate_angle(signature, front, right):
    """Clockwise degrees from robot front; return None for ambiguous signatures."""
    basis = np.column_stack((front, right)).astype(np.float64)
    measured = np.asarray(signature, dtype=np.float64)
    if basis.shape != (3, 2) or measured.shape != (3,):
        return None
    if not np.all(np.isfinite(basis)) or not np.all(np.isfinite(measured)):
        return None
    if np.linalg.cond(basis) > 20 or np.linalg.norm(measured) < 0.5:
        return None
    weights, *_ = np.linalg.lstsq(basis, measured, rcond=None)
    if np.linalg.norm(basis @ weights - measured) > 0.35 * np.linalg.norm(measured):
        return None
    return round(math.degrees(math.atan2(weights[1], weights[0]))) % 360
