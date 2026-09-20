"""Bounded, spawn-safe publication gate for a single sensor output group."""

import multiprocessing
import time
from contextlib import contextmanager


class OutputGate:
    """Stop waits for in-flight publish; epochs invalidate queued old frames.

    Locks are never held during conversion/encoding. A stuck DDS publish makes
    stop fail explicitly rather than reporting idle or killing sibling streams.
    """

    def __init__(self, enabled=True):
        ctx = multiprocessing.get_context("spawn")
        self._lock = ctx.RLock()
        self._values = ctx.Array("d", [int(enabled), 0, 0, 0], lock=False)

    @contextmanager
    def _guard(self):
        if not self._lock.acquire(timeout=2.0):
            raise TimeoutError("sensor_output_transition_timeout")
        try:
            yield
        finally:
            self._lock.release()

    def set_enabled(self, enabled):
        with self._guard():
            if bool(self._values[0]) != bool(enabled):
                self._values[0] = int(enabled)
                self._values[1] += 1
                self._values[2] = 0
                self._values[3] = 0

    def token(self):
        # Acquisition callbacks must never wait behind a DDS publish or a
        # lifecycle transition. Losing one sample is preferable to starving
        # the shared clock, IMU callback, or safety acquisition.
        if not self._lock.acquire(block=False):
            return None
        try:
            return int(self._values[1]) if self._values[0] else None
        finally:
            self._lock.release()

    def acquisition_requested(self):
        # Advisory single-value read, used only to pause/resume acquisition.
        # Actual publication always rechecks enabled + epoch under the lock.
        return bool(self._values[0])

    def publish(self, token, publisher, message, record=True):
        with self._guard():
            if token is None or not self._values[0] or token != self._values[1]:
                return False
            publisher.publish(message)
            if record:
                self._values[2] += 1
                self._values[3] = time.monotonic()
            return True

    def status(self, max_age_ms=2000):
        with self._guard():
            enabled, epoch, count, last = self._values[:]
        age = (time.monotonic() - last) * 1000 if last else None
        ready = bool(enabled and age is not None and age <= max_age_ms)
        return {
            "enabled": bool(enabled), "epoch": int(epoch),
            "published": int(count), "publish_age_ms": age, "ready": ready,
            "state": "idle" if not enabled else ("ready" if ready else "not_ready"),
        }


def run_navigation_worker(config, namespace, network_iface, gates):
    # Spawn target stays free of ROS/SDK imports until logging is installed.
    from common import logsafe
    logsafe.install(check_fd=False)
    from navigation_sensor_bridge_main import run
    run(config, namespace, network_iface, gates)
