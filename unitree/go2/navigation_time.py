"""Clock-domain normalization helpers for the G1 navigation sensor bridge.

Unitree's MID360 publishes LiDAR and IMU headers in a shared sensor clock that
is not guaranteed to match the Jetson system clock.  Estimators and planners
must still receive ROS timestamps in one coherent clock domain.  This module is
kept free of ROS dependencies so the correction policy can be unit tested on a
development machine.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import threading


NSEC_PER_SEC = 1_000_000_000


def stamp_to_ns(sec: int, nanosec: int) -> int:
    """Convert a ROS-style ``sec``/``nanosec`` pair to integer nanoseconds."""
    if not 0 <= int(nanosec) < NSEC_PER_SEC:
        raise ValueError(f"nanosec out of range: {nanosec}")
    return int(sec) * NSEC_PER_SEC + int(nanosec)


def split_ns(timestamp_ns: int) -> tuple[int, int]:
    """Split integer nanoseconds into a normalized ROS timestamp pair."""
    sec, nanosec = divmod(int(timestamp_ns), NSEC_PER_SEC)
    return sec, nanosec


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


class ClockOffsetEstimator:
    """Estimate ``host_time - sensor_time`` from callback arrival times.

    Transport and scheduling latency is non-negative, so the minimum candidate
    in a rolling window is a better offset estimate than an average.  A single
    late callback is rejected instead of shifting every subsequent timestamp.
    Persistent jumps are treated as a source-clock reset and require a new
    warm-up period.
    """

    def __init__(
        self,
        *,
        warmup_samples: int = 32,
        window_samples: int = 400,
        reset_threshold_ns: int = NSEC_PER_SEC,
        reset_confirm_samples: int = 8,
    ) -> None:
        if warmup_samples < 1:
            raise ValueError("warmup_samples must be positive")
        if window_samples < warmup_samples:
            raise ValueError("window_samples must be >= warmup_samples")
        if reset_threshold_ns < 1:
            raise ValueError("reset_threshold_ns must be positive")
        if reset_confirm_samples < 2:
            raise ValueError("reset_confirm_samples must be >= 2")

        self._warmup_samples = warmup_samples
        self._reset_threshold_ns = reset_threshold_ns
        self._reset_confirm_samples = reset_confirm_samples
        self._candidates: deque[int] = deque(maxlen=window_samples)
        self._pending_reset: list[int] = []
        self._offset_ns: int | None = None
        self._residual_ns: int | None = None
        self._ready = False
        self._resets = 0
        self._rejected = 0
        self._lock = threading.Lock()

    def correct_observation(self, source_ns: int, host_arrival_ns: int) -> int | None:
        """Observe one sample and return its corrected host timestamp when ready.

        ``None`` means the sample must not be published: either the initial
        offset is still warming up or the observation looks like a clock jump.
        """
        source_ns = int(source_ns)
        host_arrival_ns = int(host_arrival_ns)
        if source_ns <= 0 or host_arrival_ns <= 0:
            raise ValueError("timestamps must be positive")

        candidate = host_arrival_ns - source_ns
        with self._lock:
            if self._offset_ns is None:
                self._accept_locked(candidate)
            else:
                delta = candidate - self._offset_ns
                if delta < -self._reset_threshold_ns:
                    # The source clock jumped forward.  Re-baseline immediately;
                    # normal callback latency cannot make this candidate smaller.
                    self._reset_locked(candidate)
                elif delta > self._reset_threshold_ns:
                    # This may be one delayed callback or a source clock that
                    # jumped backwards.  Require consecutive evidence so a large
                    # point-cloud callback cannot reset the IMU clock domain.
                    self._pending_reset.append(candidate)
                    self._rejected += 1
                    self._residual_ns = delta
                    if len(self._pending_reset) >= self._reset_confirm_samples:
                        self._reset_locked(min(self._pending_reset))
                    return None
                else:
                    self._pending_reset.clear()
                    self._accept_locked(candidate)

            if not self._ready:
                return None
            return source_ns + int(self._offset_ns)

    def snapshot(self) -> ClockSnapshot:
        with self._lock:
            return ClockSnapshot(
                ready=self._ready,
                samples=len(self._candidates),
                offset_ns=self._offset_ns,
                residual_ns=self._residual_ns,
                resets=self._resets,
                rejected=self._rejected,
                pending_reset_samples=len(self._pending_reset),
            )

    def _accept_locked(self, candidate: int) -> None:
        self._candidates.append(candidate)
        self._offset_ns = min(self._candidates)
        self._residual_ns = candidate - self._offset_ns
        self._ready = len(self._candidates) >= self._warmup_samples

    def _reset_locked(self, candidate: int) -> None:
        self._candidates.clear()
        self._pending_reset.clear()
        self._offset_ns = None
        self._residual_ns = None
        self._ready = False
        self._resets += 1
        self._accept_locked(candidate)
