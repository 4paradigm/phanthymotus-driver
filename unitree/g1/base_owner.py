"""In-process ownership bookkeeping for a G1 base, without hardware access.

This module is not connected to any motion source. A grant only authorizes a
caller to enqueue work; it neither starts nor stops a robot. Callers must fence
queued work again at the actual execution boundary, serialize it with stopping,
and handle commands already sent or in flight. No lock here crosses a process.
Tokens prevent accidental stale reuse, not malicious impersonation. Each new
instance starts idle: a restart must independently establish a safe hardware
and previous-process state before granting ownership; an epoch is not a stop.

Typical use: acquire(owner) -> submit(grant, enqueue) -> begin_stop() -> send
one stop outside this module -> finish_stop(stop_token, success=...). Success
means the external stop-command operation succeeded, not physical standstill.
A failed/unknown result enters fault. To recover, begin_stop() again, complete
that retry successfully, then explicitly reset_fault(); old grants stay invalid.
Preemption only begins stopping; a contender must acquire again after completion.
"""

from dataclasses import dataclass
from threading import Lock
from typing import Callable, Optional
from uuid import uuid4


@dataclass(frozen=True)
class Grant:
    owner: str
    generation: int
    epoch: str


@dataclass(frozen=True)
class StopToken:
    generation: int
    epoch: str


@dataclass(frozen=True)
class OwnerSnapshot:
    state: str
    owner: Optional[str]
    generation: int
    fault_stop_succeeded: bool


class BaseOwner:
    """Thread-safe, fail-closed state machine; no priority or I/O policy."""

    def __init__(self):
        self._lock = Lock()
        self._epoch = uuid4().hex
        self._state = "idle"
        self._generation = 0
        self._grant = None
        self._stop_token = None
        self._recovering = False
        self._fault_stop_succeeded = False

    def snapshot(self) -> OwnerSnapshot:
        with self._lock:
            return OwnerSnapshot(self._state, self._grant.owner if self._grant else None,
                                 self._generation, self._fault_stop_succeeded)

    def acquire(self, owner: str) -> Optional[Grant]:
        """Acquire only when idle; busy, stopping, and fault never grant."""
        if not isinstance(owner, str) or not owner.strip():
            raise ValueError("owner must be a nonempty string")
        with self._lock:
            if self._state != "idle":
                return None
            self._generation += 1
            self._grant = Grant(owner, self._generation, self._epoch)
            self._state = "owned"
            return self._grant

    def is_current(self, grant: Grant) -> bool:
        """Point-in-time check only; does not protect a later hardware write."""
        with self._lock:
            return self._state == "owned" and self._grant == grant

    def submit(self, grant: Grant, enqueue: Callable[[], None]) -> bool:
        """Atomically validate a grant and call a short, nonblocking enqueue.

        The callback must not perform I/O, block, or reenter this object. The
        queued item must carry the grant for a final execution-side fence. A
        callback exception propagates; it does not imply enqueue or execution.
        """
        with self._lock:
            if self._state != "owned" or self._grant != grant:
                return False
            enqueue()
            return True

    def begin_stop(self) -> Optional[StopToken]:
        """Revoke first, then return a token for a caller to send stop externally.

        Any caller may request a stop, including a competing source or safety
        path. No contender is granted by this operation. In fault this begins a
        stop retry; stopping/idle cannot start a second stop via this method.
        """
        with self._lock:
            if self._state not in ("owned", "fault"):
                return None
            self._recovering = self._state == "fault"
            self._fault_stop_succeeded = False
            self._generation += 1
            self._grant = None
            self._stop_token = StopToken(self._generation, self._epoch)
            self._state = "stopping"
            return self._stop_token

    def finish_stop(self, token: StopToken, *, success: bool) -> bool:
        """Record this stop's external result; stale tokens cannot change state.

        Only literal True reports successful stop-command issuance/result;
        failure or uncertainty must be reported as False. A successful retry
        remains faulted until reset_fault() is explicitly called.
        """
        if type(success) is not bool:
            raise ValueError("success must be a bool")
        with self._lock:
            if self._state != "stopping" or self._stop_token != token:
                return False
            self._stop_token = None
            if not success:
                self._state = "fault"
                self._fault_stop_succeeded = False
            elif self._recovering:
                self._state = "fault"
                self._fault_stop_succeeded = True
            else:
                self._state = "idle"
            self._recovering = False
            return True

    def reset_fault(self) -> bool:
        """Explicitly clear fault only after a successful stop retry."""
        with self._lock:
            if self._state != "fault" or not self._fault_stop_succeeded:
                return False
            self._fault_stop_succeeded = False
            self._state = "idle"
            return True
