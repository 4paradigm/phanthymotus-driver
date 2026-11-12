"""One arm owner across actuator cards, including recording and paused queues."""

import threading


class ArmOwnership:
    def __init__(self):
        self._lock = threading.RLock()
        self._owner = None
        self._lease = None

    @property
    def owner(self):
        with self._lock:
            return self._owner

    def check(self, owner):
        with self._lock:
            if self._owner not in (None, owner):
                raise RuntimeError(f"arm is occupied by {self._owner}")

    def acquire(self, owner):
        with self._lock:
            self.check(owner)
            self._owner, self._lease = owner, object()
            return self._lease

    def release(self, lease):
        with self._lock:
            # A cancelled worker must not release its replacement's reservation.
            if self._lease is lease:
                self._owner = self._lease = None
