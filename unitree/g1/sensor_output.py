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
        # Publication and lifecycle transitions are serialized across processes.
        # This lock must not be reentrant: a stop called from inside publish must
        # not succeed before that publication has returned.
        self._lock = ctx.Lock()
        self._state_lock = ctx.Lock()
        self._values = ctx.Array("d", [int(enabled), 0, 0, 0], lock=False)

    @staticmethod
    @contextmanager
    def _acquire(lock):
        if not lock.acquire(timeout=2.0):
            raise TimeoutError("sensor_output_transition_timeout")
        try:
            yield
        finally:
            lock.release()

    def _guard(self):
        return self._acquire(self._lock)

    def set_enabled(self, enabled):
        # Lock order is always publication lock, then state lock. A successful
        # stop cannot overtake a DDS call that has already been admitted.
        with self._guard():
            with self._acquire(self._state_lock):
                if bool(self._values[0]) != bool(enabled):
                    self._values[0] = int(enabled)
                    self._values[1] += 1
                    self._values[2] = 0
                    self._values[3] = 0

    def token(self):
        # The state lock is never held during DDS publication or conversion.
        # A brief transition can still make a callback drop rather than block.
        if not self._state_lock.acquire(block=False):
            return None
        try:
            return int(self._values[1]) if self._values[0] else None
        finally:
            self._state_lock.release()

    def acquisition_requested(self):
        # Advisory single-value read, used only to pause/resume acquisition.
        # Actual publication always rechecks enabled + epoch under the lock.
        return bool(self._values[0])

    def publish(self, token, publisher, message, record=True):
        with self._guard():
            with self._acquire(self._state_lock):
                if token is None or not self._values[0] or token != self._values[1]:
                    return False
            # Keep the publication lock, but never the state lock, over DDS.
            # Transitions cannot change epoch until this call and accounting end.
            publisher.publish(message)
            if record:
                with self._acquire(self._state_lock):
                    self._values[2] += 1
                    self._values[3] = time.monotonic()
            return True

    def status(self, max_age_ms=2000):
        with self._acquire(self._state_lock):
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
