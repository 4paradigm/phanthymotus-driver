"""Independent G1 velocity transport, not a robot safety controller.

All deadlines use time.monotonic() seconds. A single writer serializes SDK
Begin* calls; their returned wait closures run outside the writer in bounded
workers, including on shutdown. SDK writes can block indefinitely: a stop cannot
preempt a write already committed to that writer. ACKs do not prove physical
motion or stopping, and no cross-card arbitration is provided.
"""
from __future__ import annotations

import math
import threading
import time


class LocoServoTransport:
    """Latest-only, asynchronous transport with a reserved stop ACK slot.

    client implements BeginVelocity(vx, vy, omega, duration) and BeginStopMove(),
    returning zero-argument wait closures whose result is the SDK status code.
    Construction starts daemon threads. Public methods never wait for SDK replies.
    """

    def __init__(self, client, *, watchdog_s=0.3, max_outstanding=4):
        if not math.isfinite(watchdog_s) or watchdog_s <= 0:
            raise ValueError("watchdog_s must be positive and finite")
        if isinstance(max_outstanding, bool) or not isinstance(max_outstanding, int) or max_outstanding < 2:
            raise ValueError("max_outstanding must be an integer >= 2")
        self._client = client
        self._watchdog_s = watchdog_s
        self._max_moves = max_outstanding - 1
        self._cv = threading.Condition()
        self._generation = 0
        self._paused = False
        self._closed = False
        self._closing = False
        self._pending = None
        self._stop_pending = False
        self._stop_inflight = False
        self._stop_status = "ok"
        self._move_outstanding = 0
        self._watch_deadline = None
        self._last_error = None
        self._stop_reason = None
        threading.Thread(target=self._writer, name="g1-servo-writer", daemon=True).start()
        threading.Thread(target=self._watchdog, name="g1-servo-watchdog", daemon=True).start()

    def submit_velocity(self, vx, vy, omega, duration, *, deadline):
        """Replace pending velocity; return False if paused, closed or expired."""
        vals = (vx, vy, omega, duration, deadline)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals) or duration <= 0:
            raise ValueError("velocity, duration and deadline must be finite; duration > 0")
        with self._cv:
            now = time.monotonic()
            if self._closed or self._closing or self._paused or self._stop_status != "ok" or deadline <= now:
                return False
            self._pending = (self._generation, vx, vy, omega, duration, deadline)
            # Queuing a replacement must not extend the lease of a command
            # already sent while the writer is congested.
            candidate = min(deadline, now + self._watchdog_s)
            self._watch_deadline = (candidate if self._watch_deadline is None
                                    else min(self._watch_deadline, candidate))
            self._cv.notify_all()
            return True

    def _request_stop_locked(self, reason="requested"):
        self._stop_reason = reason
        self._generation += 1
        self._paused = True
        self._pending = None
        self._watch_deadline = None
        self._stop_pending = True
        self._stop_status = "pending"
        self._cv.notify_all()

    def pause(self):
        """Invalidate pending motion and asynchronously request a stop."""
        with self._cv:
            if self._closed:
                return False
            self._request_stop_locked()
            return True

    stop = pause

    def resume(self):
        """Re-enable submissions only after the current stop returned success."""
        with self._cv:
            if self._closed or self._closing or self._stop_status != "ok" or self._stop_pending or self._stop_inflight:
                return False
            self._paused = False
            return True

    def close(self):
        """Request a final stop asynchronously; a failed stop remains retryable."""
        with self._cv:
            if not self._closed and not self._closing:
                self._closing = True
                self._request_stop_locked()

    def wait_stop(self, timeout):
        """Wait boundedly for an already requested stop's SDK response."""
        end = time.monotonic() + timeout
        with self._cv:
            while self._stop_status == "pending":
                remaining = end - time.monotonic()
                if remaining <= 0:
                    break
                self._cv.wait(remaining)
            return self._stop_status

    def status(self):
        with self._cv:
            return {"generation": self._generation, "paused": self._paused,
                    "closed": self._closed, "closing": self._closing, "stop_status": self._stop_status,
                    "stop_reason": self._stop_reason,
                    "pending_velocity": self._pending is not None,
                    "outstanding": self._move_outstanding + int(self._stop_inflight),
                    "last_error": self._last_error}

    def _watchdog(self):
        with self._cv:
            while not self._closed:
                if self._watch_deadline is None:
                    self._cv.wait()
                    continue
                remaining = self._watch_deadline - time.monotonic()
                if remaining <= 0:
                    self._request_stop_locked("watchdog")
                else:
                    self._cv.wait(remaining)

    def _writer(self):
        while True:
            with self._cv:
                while True:
                    if self._stop_pending and not self._stop_inflight:
                        gen = self._generation
                        self._stop_pending = False
                        self._stop_inflight = True
                        kind, args = "stop", ()
                        break
                    if (not self._paused and not self._closed and self._pending is not None
                            and self._move_outstanding < self._max_moves):
                        gen, vx, vy, omega, duration, deadline = self._pending
                        self._pending = None
                        if gen != self._generation or deadline <= time.monotonic():
                            continue
                        self._move_outstanding += 1
                        self._watch_deadline = min(deadline, time.monotonic() + self._watchdog_s)
                        self._cv.notify_all()
                        kind, args = "move", (vx, vy, omega, duration)
                        break
                    if self._closed and not self._stop_pending and not self._stop_inflight:
                        return
                    self._cv.wait()
            # The write is committed above. Pause remains nonblocking even if SDK
            # Begin* blocks; it cannot cancel this already committed write.
            try:
                wait = (self._client.BeginStopMove() if kind == "stop"
                        else self._client.BeginVelocity(*args))
                if not callable(wait):
                    raise TypeError("SDK Begin* did not return a wait closure")
            except Exception as exc:
                self._complete(kind, gen, None, exc)
                continue
            threading.Thread(target=self._consume, args=(kind, gen, wait),
                             name="g1-servo-ack", daemon=True).start()

    def _consume(self, kind, gen, wait):
        try:
            code = wait()
            self._complete(kind, gen, code, None)
        except Exception as exc:
            self._complete(kind, gen, None, exc)

    def _complete(self, kind, gen, code, error):
        with self._cv:
            if kind == "stop":
                self._stop_inflight = False
            else:
                self._move_outstanding -= 1
            if gen == self._generation:
                if error is not None or code != 0:
                    self._last_error = str(error) if error is not None else f"SDK returned {code}"
                    if kind == "stop":
                        self._stop_status = "failed"
                        self._closing = False
                    elif not self._closed:
                        self._request_stop_locked("move_failed")
                elif kind == "stop":
                    self._stop_status = "ok"
                    self._last_error = None
                    if self._closing:
                        self._closing = False
                        self._closed = True
            self._cv.notify_all()


def create_g1_loco_servo_transport(*, watchdog_s=0.3, max_outstanding=4, sdk_timeout_s=1.0):
    """Make a dedicated SDK client after the G1 ChannelFactory is initialized.

    This does not initialize DDS, change control mode, or connect this transport
    to a card. SDK timeout bounds response waits, not a potentially blocked write.
    """
    if not math.isfinite(sdk_timeout_s) or sdk_timeout_s <= 0:
        raise ValueError("sdk_timeout_s must be positive and finite")
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    client = LocoClient()
    client.SetTimeout(sdk_timeout_s)
    client.Init()
    return LocoServoTransport(client, watchdog_s=watchdog_s, max_outstanding=max_outstanding)
