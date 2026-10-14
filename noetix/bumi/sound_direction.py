"""Bumi head microphone delay estimation and two-position direction calibration."""

import math
from collections import deque

import numpy as np

MAX_BASIS_CONDITION = 20


class SoundActivityGate:
    """Accept sounds clearly louder than the recent microphone background."""

    def __init__(self, onset_level=10.0, onset_ratio=2.0,
                 burst_level=15.0, burst_ratio=4.0):
        self._levels = deque(maxlen=50)
        self._rising_frames = 0
        self._hold_samples = 0
        self._onset_level = onset_level
        self._onset_ratio = onset_ratio
        self._burst_level = burst_level
        self._burst_ratio = burst_ratio

    def accepts(self, audio) -> bool:
        samples = np.asarray(audio, dtype=np.float32)
        if samples.size == 0 or samples.size % 8:
            return False
        head = samples.reshape(-1, 8)[:, :4]
        head = head - head.mean(axis=0)
        level = float(np.median(np.sqrt(np.mean(head * head, axis=0))))
        background = (float(np.percentile(self._levels, 20))
                      if len(self._levels) >= 6 else None)
        if background is None:
            self._levels.append(level)
            return False
        # 去直流后用连续两帧识别较轻的说话声；短暂停顿只保留状态，不输出底噪角度。
        rising = level >= max(self._onset_level, background * self._onset_ratio)
        self._rising_frames = self._rising_frames + 1 if rising else 0
        if self._rising_frames >= 2 or level >= max(
                self._burst_level, background * self._burst_ratio):
            self._hold_samples = 9600  # 允许约 0.6 秒语音停顿。
            return True
        if self._hold_samples > 0 and level >= max(10.0, background * 1.5):
            self._hold_samples = 9600
            return False  # 只维持语音状态，停顿时的风扇声不能刷新角度。
        if not rising:
            # 只把非发声帧计入背景：持续背景噪声仍会更新底噪，
            # 但持续说话不能把自己抬成背景，否则同电平发声会永远过不了门。
            self._levels.append(level)
        self._hold_samples = max(0, self._hold_samples - len(head))
        return False


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
    """Counter-clockwise degrees from robot front (REP-103: left positive, same sign as vyaw).

    0 = front, 90 = left, 270 (=-90) = right; return None for ambiguous signatures.
    """
    basis = np.column_stack((front, right)).astype(np.float64)
    measured = np.asarray(signature, dtype=np.float64)
    if basis.shape != (3, 2) or measured.shape != (3,):
        return None
    if not np.all(np.isfinite(basis)) or not np.all(np.isfinite(measured)):
        return None
    if np.linalg.cond(basis) > MAX_BASIS_CONDITION or np.linalg.norm(measured) < 0.5:
        return None
    weights, *_ = np.linalg.lstsq(basis, measured, rcond=None)
    if np.linalg.norm(basis @ weights - measured) > 0.35 * np.linalg.norm(measured):
        return None
    # atan2(right, front) 是顺时针角；取反得 REP-103 的逆时针（左正）角。
    return round(-math.degrees(math.atan2(weights[1], weights[0]))) % 360
