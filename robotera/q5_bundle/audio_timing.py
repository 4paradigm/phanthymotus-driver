"""Thread-safe PCM transport timing facts; no acoustic playback claims."""

from __future__ import annotations

import threading
import time
import math


class PcmWriteTimeline:
    def __init__(self, sample_rate_hz: int, channels: int = 1, sample_width_bytes: int = 2):
        self.sample_rate_hz = int(sample_rate_hz)
        self.channels = int(channels)
        self.sample_width_bytes = int(sample_width_bytes)
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self.frames_received = 0
            self.frames_written = 0
            self.bytes_received = 0
            self.bytes_written = 0
            self.first_received_monotonic_ns = None
            self.first_write_completed_monotonic_ns = None
            self.last_write_completed_monotonic_ns = None

    def mark_received(self, byte_count: int, now_ns: int | None = None) -> None:
        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        with self._lock:
            self.frames_received += 1
            self.bytes_received += max(0, int(byte_count))
            if self.first_received_monotonic_ns is None:
                self.first_received_monotonic_ns = now_ns

    def mark_write_completed(self, byte_count: int, now_ns: int | None = None) -> None:
        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        with self._lock:
            self.frames_written += 1
            self.bytes_written += max(0, int(byte_count))
            if self.first_write_completed_monotonic_ns is None:
                self.first_write_completed_monotonic_ns = now_ns
            self.last_write_completed_monotonic_ns = now_ns

    def snapshot(self) -> dict:
        with self._lock:
            sample_bytes = self.channels * self.sample_width_bytes
            samples_written = self.bytes_written // sample_bytes if sample_bytes else 0
            return {
                "clock_domain": "host_monotonic",
                "first_pcm_received_monotonic_ns": self.first_received_monotonic_ns,
                "first_pcm_write_completed_monotonic_ns": self.first_write_completed_monotonic_ns,
                "last_pcm_write_completed_monotonic_ns": self.last_write_completed_monotonic_ns,
                "frames_received": self.frames_received,
                "frames_written": self.frames_written,
                "bytes_received": self.bytes_received,
                "bytes_written": self.bytes_written,
                "pcm_samples_written": samples_written,
                "pcm_duration_written_s": round(
                    samples_written / self.sample_rate_hz, 6) if self.sample_rate_hz else 0.0,
                "acoustic_t0_trusted": False,
                "acoustic_t0_rejection_reason": "no_hardware_playback_cursor",
            }
