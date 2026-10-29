"""Injected clock.

Same shape and same reason as `common/control/sink.py`'s injected clock: a 24
second navigation leg has to be a real 24 seconds on a robot and a handful of
microseconds in a test, and nothing above this file should know which it is.

`FakeClock` is also what keeps `step()` faster than real time once a physics
backend is attached — see the plan's "three preconditions".
"""

from __future__ import annotations

import threading
import time


class Clock:
    """Interface. `now()` is monotonic seconds; `sleep()` may return early."""

    def now(self) -> float:  # pragma: no cover - interface
        raise NotImplementedError

    def sleep(self, seconds: float) -> None:  # pragma: no cover - interface
        raise NotImplementedError


class RealClock(Clock):
    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


class FakeClock(Clock):
    """Manually advanced. `sleep` advances instead of blocking.

    Thread-safe so a test may run the real tick thread against it, but the
    normal use is single-threaded: call `world.step(dt)` directly.
    """

    def __init__(self, start: float = 0.0):
        self._t = float(start)
        self._lock = threading.RLock()

    def now(self) -> float:
        with self._lock:
            return self._t

    def sleep(self, seconds: float) -> None:
        self.advance(seconds)

    def advance(self, seconds: float) -> float:
        with self._lock:
            self._t += max(0.0, float(seconds))
            return self._t
