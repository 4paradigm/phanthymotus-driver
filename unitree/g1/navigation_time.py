"""Clock-domain normalization helpers for the G1 navigation sensor bridge."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import threading
import time

NSEC_PER_SEC = 1_000_000_000
WINDOW_NS = 30 * NSEC_PER_SEC
WINDOW_CAPACITY = 12000


def stamp_to_ns(sec: int, nanosec: int) -> int:
    if not 0 <= int(nanosec) < NSEC_PER_SEC:
        raise ValueError(f"nanosec out of range: {nanosec}")
    return int(sec) * NSEC_PER_SEC + int(nanosec)


def split_ns(timestamp_ns: int) -> tuple[int, int]:
    return divmod(int(timestamp_ns), NSEC_PER_SEC)


@dataclass(frozen=True)
class ClockSnapshot:
    ready: bool
    samples: int
    offset_ns: int | None
    residual_ns: int | None
    resets: int
    rejected: int
    pending_reset_samples: int

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class NavigationClockSnapshot(ClockSnapshot):
    stale_observations: int = 0
    window_ns: int = WINDOW_NS
    window_capacity: int = WINDOW_CAPACITY
    window_evicted: int = 0
    capacity_untruncated: bool = True
    epoch_elapsed_seconds: float = 0.0
    retained_sample_span_seconds: float = 0.0
    window_filled: bool = False
    window_quality: str = "warming"


class NavigationClockOffsetEstimator:
    """Shared offset, 30s rolling minimum and 50-us/s post-warmup slew.

    All accepted samples in the last 30 seconds are retained unless capped.
    Status reports capped history separately from retained time span; neither
    implies continuous inputs or proves that minimum latency was observed.
    A rising 20-us/s offset can naturally lag by roughly 600us.
    """

    def __init__(self, *, warmup_samples: int = 32,
                 reset_threshold_ns: int = NSEC_PER_SEC,
                 reset_confirm_samples: int = 8) -> None:
        if not 1 <= warmup_samples <= 12000:
            raise ValueError("invalid warmup configuration")
        if reset_threshold_ns < 1 or not 2 <= reset_confirm_samples <= 12000:
            raise ValueError("invalid reset configuration")
        self._warmup_samples = warmup_samples
        self._reset_threshold_ns = reset_threshold_ns
        self._reset_confirm_samples = reset_confirm_samples
        self._candidates: deque[tuple[int, int]] = deque()
        self._minima: deque[tuple[int, int]] = deque()
        self._window_evicted = 0
        self._last_evicted_mono_ns: int | None = None
        self._first_accepted_mono_ns: int | None = None
        self._pending_reset: list[int] = []
        self._offset_ns: int | None = None
        self._residual_ns: int | None = None
        self._last_mono_ns: int | None = None
        self._slew_remainder = 0
        self._ready = False
        self._resets = 0
        self._rejected = 0
        self._stale_observations = 0
        self._lock = threading.Lock()

    def correct_observation(self, source_ns: int, host_arrival_ns: int,
                            monotonic_ns: int | None = None) -> int | None:
        source_ns, host_arrival_ns = int(source_ns), int(host_arrival_ns)
        mono = time.monotonic_ns() if monotonic_ns is None else int(monotonic_ns)
        if source_ns <= 0 or host_arrival_ns <= 0 or mono < 0:
            raise ValueError("timestamps must be positive (monotonic may be zero)")
        candidate = host_arrival_ns - source_ns
        with self._lock:
            if self._last_mono_ns is not None and mono <= self._last_mono_ns:
                self._rejected += 1
                self._stale_observations += 1
                return None
            dt = 0 if self._last_mono_ns is None else mono - self._last_mono_ns
            self._last_mono_ns = mono
            self._expire_locked(mono)
            if self._offset_ns is not None:
                delta = candidate - self._offset_ns
                if abs(delta) > self._reset_threshold_ns:
                    if self._pending_reset:
                        previous = self._pending_reset[0]
                        same_direction = (candidate - self._offset_ns) * (previous - self._offset_ns) > 0
                        low = min(min(self._pending_reset), candidate)
                        high = max(max(self._pending_reset), candidate)
                        if not same_direction or high - low >= self._reset_threshold_ns:
                            self._pending_reset.clear()
                    self._pending_reset.append(candidate)
                    self._rejected += 1
                    self._residual_ns = delta
                    if len(self._pending_reset) >= self._reset_confirm_samples:
                        self._reset_locked(min(self._pending_reset), mono)
                    return None
            self._pending_reset.clear()
            self._accept_locked(candidate, mono, dt)
            return source_ns + self._offset_ns if self._ready else None

    def snapshot(self) -> ClockSnapshot:
        with self._lock:
            incomplete = (self._last_evicted_mono_ns is not None and
                          self._last_mono_ns - self._last_evicted_mono_ns <= WINDOW_NS)
            elapsed_ns = (0 if self._first_accepted_mono_ns is None else
                          self._last_mono_ns - self._first_accepted_mono_ns)
            retained_span_ns = (self._candidates[-1][0] - self._candidates[0][0]
                                if self._candidates else 0)
            # A 100ms boundary tolerance only describes sample extent; it does
            # not certify continuous arrivals or adequate low-latency samples.
            filled = retained_span_ns >= WINDOW_NS - 100_000_000
            quality = ("reset_pending" if self._pending_reset else "incomplete" if incomplete
                       else "sample_span_full" if filled else "warming")
            return NavigationClockSnapshot(
                ready=self._ready and not self._pending_reset,
                samples=len(self._candidates), offset_ns=self._offset_ns,
                residual_ns=self._residual_ns, resets=self._resets, rejected=self._rejected,
                pending_reset_samples=len(self._pending_reset),
                stale_observations=self._stale_observations, window_evicted=self._window_evicted,
                capacity_untruncated=not incomplete, epoch_elapsed_seconds=elapsed_ns / NSEC_PER_SEC,
                retained_sample_span_seconds=retained_span_ns / NSEC_PER_SEC,
                window_filled=filled, window_quality=quality)

    def _expire_locked(self, mono: int) -> None:
        while self._candidates and self._candidates[0][0] < mono - WINDOW_NS:
            removed = self._candidates.popleft()
            if self._minima and self._minima[0] == removed:
                self._minima.popleft()

    def _accept_locked(self, candidate: int, mono: int, dt: int) -> None:
        was_ready = self._ready
        if len(self._candidates) >= WINDOW_CAPACITY:
            removed = self._candidates.popleft()
            if self._minima and self._minima[0] == removed:
                self._minima.popleft()
            self._window_evicted += 1
            self._last_evicted_mono_ns = removed[0]
        if self._first_accepted_mono_ns is None:
            self._first_accepted_mono_ns = mono
        item = (mono, candidate)
        self._candidates.append(item)
        while self._minima and self._minima[-1][1] > candidate:
            self._minima.pop()
        self._minima.append(item)
        target = self._minima[0][1]
        self._ready = was_ready or len(self._candidates) >= self._warmup_samples
        if not was_ready:
            self._offset_ns = target
        else:
            budget, self._slew_remainder = divmod(dt * 50_000 + self._slew_remainder,
                                                  NSEC_PER_SEC)
            delta = target - self._offset_ns
            self._offset_ns += max(-budget, min(budget, delta))
        self._residual_ns = candidate - self._offset_ns

    def _reset_locked(self, candidate: int, mono: int) -> None:
        self._candidates.clear()
        self._minima.clear()
        self._pending_reset.clear()
        self._last_evicted_mono_ns = None
        self._first_accepted_mono_ns = None
        self._offset_ns = None
        self._slew_remainder = 0
        self._ready = False
        self._resets += 1
        self._accept_locked(candidate, mono, 0)
