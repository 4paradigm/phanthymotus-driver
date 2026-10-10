"""ROS receive-loop health. Importable without ROS for fault-injection tests."""
import logging
import threading
import time

LOG = logging.getLogger(__name__)


class SpinHealth:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.running = False
        self.heartbeat = None
        self.invalid_handles = 0
        self.last_error = None
        self.fatal = False
        self._last_log = float('-inf')

    def beat(self):
        with self.lock:
            self.running = True
            self.heartbeat = self.clock()

    def error(self, exc, *, recoverable):
        with self.lock:
            self.last_error = f'{type(exc).__name__}: {exc}'
            self.fatal = not recoverable
            if recoverable:
                self.invalid_handles += 1
            now = self.clock()
            emit = now - self._last_log >= 2.
            if emit:
                self._last_log = now
        if emit:
            LOG.exception('ROS receive loop error; recoverable=%s', recoverable)

    def stopped(self):
        with self.lock:
            self.running = False

    def status(self):
        with self.lock:
            age = None if self.heartbeat is None else max(0., self.clock()-self.heartbeat)
            return {'running': self.running, 'healthy': bool(self.running and not self.fatal and age is not None and age < 1.),
                    'heartbeat_age_ms': None if age is None else age*1000.,
                    'invalid_handle_count': self.invalid_handles,
                    'last_error': self.last_error, 'fatal': self.fatal}


def supervised_spin(executor, ok, health, invalid_handle, shutdown_errors=(), sleep=time.sleep):
    try:
        while ok():
            try:
                executor.spin_once(timeout_sec=.1)
                health.beat()
            except shutdown_errors:
                break
            except invalid_handle as exc:
                health.error(exc, recoverable=True)
                # Bounded retry, never a tight exception loop. A successful spin
                # alone refreshes the heartbeat; repeated failures become unhealthy.
                sleep(.05)
            except Exception as exc:
                health.error(exc, recoverable=False)
                break
    finally:
        health.stopped()
