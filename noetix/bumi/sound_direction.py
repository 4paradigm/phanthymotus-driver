"""Bumi head microphone delay estimation and two-position direction calibration."""

import math

import numpy as np


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
        delays.append(float(np.argmax(nearby) - max_lag))
    return tuple(delays)


def estimate_angle(signature, front, right):
    """Clockwise degrees from robot front; return None for ambiguous signatures."""
    basis = np.column_stack((front, right)).astype(np.float64)
    measured = np.asarray(signature, dtype=np.float64)
    if basis.shape != (3, 2) or measured.shape != (3,):
        return None
    if np.linalg.cond(basis) > 20 or np.linalg.norm(measured) < 0.5:
        return None
    weights, *_ = np.linalg.lstsq(basis, measured, rcond=None)
    if np.linalg.norm(basis @ weights - measured) > 0.35 * np.linalg.norm(measured):
        return None
    return round(math.degrees(math.atan2(weights[1], weights[0]))) % 360
