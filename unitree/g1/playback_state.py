"""Observe the G1 body audio player's state over Unitree DDS.

The public ``AudioClient`` can submit and stop audio, but it has no status
method.  G1 firmware publishes the global player state on ``rt/audio_msg`` as
JSON (``{"play_state": 1}`` while active and ``{"play_state": 0}`` while
idle).  This module turns that event stream into a conservative stable-idle
signal for the serialized speaker stream.
"""

from __future__ import annotations

from collections import deque
import json
import threading
import time
from typing import Callable


class G1PlaybackStateMonitor:
    """Track G1 audio-service transitions and wait for stable idle."""

    def __init__(self, topic: str = "rt/audio_msg", *, available: bool = False,
                 connect_error: str = "not_connected", monotonic=None,
                 history_limit: int = 256, idle_settle_sec: float = 0.35):
        self.topic = topic
        self._monotonic = monotonic or time.monotonic
        self._cv = threading.Condition(threading.RLock())
        self._events = deque(maxlen=max(16, int(history_limit)))
        self._event_seq = 0
        self._current_state: int | None = None
        self._last_event_ts: float | None = None
        self._available = bool(available)
        self._connect_error = "" if available else connect_error
        self._idle_settle_sec = max(0.0, float(idle_settle_sec))
        self._subscriber = None

    @classmethod
    def connect_dds(cls, topic: str = "rt/audio_msg", *, monotonic=None,
                    idle_settle_sec: float = 0.35):
        """Subscribe without making speaker startup fail if DDS is absent."""
        monitor = cls(
            topic,
            monotonic=monotonic,
            idle_settle_sec=idle_settle_sec,
        )
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

            subscriber = ChannelSubscriber(topic, String_)
            # queueLen=0 dispatches directly from the DDS callback and avoids
            # adding another ordering boundary between player state and RPC.
            subscriber.Init(monitor.on_dds_message, 0)
            monitor._subscriber = subscriber
            with monitor._cv:
                monitor._available = True
                monitor._connect_error = ""
        except Exception as exc:
            with monitor._cv:
                monitor._available = False
                monitor._connect_error = (
                    f"dds_subscribe_failed:{type(exc).__name__}:{exc}"
                )
        return monitor

    @property
    def available(self) -> bool:
        with self._cv:
            return self._available

    def checkpoint(self) -> int:
        """Return the latest event sequence before starting a new stream."""
        with self._cv:
            return self._event_seq

    def on_dds_message(self, msg) -> None:
        raw = getattr(msg, "data", None)
        if raw is None:
            raw = getattr(msg, "data_", None)
        self.on_raw_message(raw)

    def on_raw_message(self, raw) -> bool:
        """Consume one DDS JSON payload; return whether it was play_state."""
        if not isinstance(raw, str):
            return False
        try:
            payload = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            return False
        if not isinstance(payload, dict):
            return False
        state = payload.get("play_state")
        if isinstance(state, bool) or state not in (0, 1):
            return False

        with self._cv:
            self._event_seq += 1
            timestamp = self._monotonic()
            self._current_state = int(state)
            self._last_event_ts = timestamp
            self._events.append((self._event_seq, int(state), timestamp))
            self._cv.notify_all()
        return True

    def _evaluate_locked(self, *, after_seq: int, idle_after_seq: int,
                         not_before: float) -> dict:
        playing = None
        idle = None
        for sequence, state, timestamp in self._events:
            if sequence <= after_seq:
                continue
            if state == 1:
                # G1 can briefly report 0→1→0 while draining its final PCM.
                # A later playing event therefore invalidates an earlier idle.
                playing = (sequence, timestamp)
                idle = None
            elif (playing is not None
                  and sequence > playing[0]
                  and sequence > idle_after_seq):
                idle = (sequence, timestamp)

        now = self._monotonic()
        if idle is not None:
            stable_since = max(idle[1], float(not_before))
            settled_for = now - stable_since
            if settled_for >= self._idle_settle_sec:
                return {
                    "state": "completed",
                    "reason": "",
                    "playing_seq": playing[0],
                    "idle_seq": idle[0],
                    "playing_ts": playing[1],
                    "idle_ts": idle[1],
                    "settled_for_sec": settled_for,
                }
            return {
                "state": "waiting",
                "reason": "idle_not_stable",
                "playing_seq": playing[0],
                "idle_seq": idle[0],
                "settle_remaining_sec": max(
                    0.0, self._idle_settle_sec - settled_for,
                ),
            }

        return {
            "state": "waiting",
            "reason": "playing" if playing is not None else "start_not_seen",
            "playing_seq": playing[0] if playing else None,
            "idle_seq": None,
        }

    def wait_for_stable_idle(self, *, after_seq: int,
                             idle_after_seq: int | None = None,
                             not_before: float,
                             timeout: float,
                             cancelled: Callable[[], bool] | None = None,
                             poll_interval: float = 0.05) -> dict:
        """Wait for a post-checkpoint playing→stable-idle transition.

        ``not_before`` is the local deadline of the final PCM block.  DDS and
        AudioClient RPC replies are independent channels, so completion is not
        gated on RPC acknowledgement ordering.  ``idle_after_seq`` is captured
        immediately before sending the final block: it rejects an earlier
        block's idle, while retaining a valid final idle that races ahead of
        the final RPC reply.
        """
        if cancelled is None:
            cancelled = lambda: False
        with self._cv:
            if not self._available:
                return {
                    "state": "error",
                    "reason": self._connect_error or "play_state_unavailable",
                }

        deadline = self._monotonic() + max(0.0, float(timeout))
        idle_gate = (
            int(after_seq)
            if idle_after_seq is None else int(idle_after_seq)
        )
        while True:
            if cancelled():
                return {"state": "interrupted", "reason": "interrupted"}
            with self._cv:
                result = self._evaluate_locked(
                    after_seq=int(after_seq),
                    idle_after_seq=idle_gate,
                    not_before=float(not_before),
                )
                if result["state"] == "completed":
                    return result
                remaining = deadline - self._monotonic()
                if remaining <= 0:
                    return {
                        "state": "timeout",
                        "reason": (
                            "play_state_idle_timeout"
                            if result.get("playing_seq") is not None
                            else "play_state_start_timeout"
                        ),
                        "playing_seq": result.get("playing_seq"),
                        "idle_seq": result.get("idle_seq"),
                    }
                wait_for = min(max(0.001, poll_interval), remaining)
                settle_remaining = result.get("settle_remaining_sec")
                if settle_remaining is not None:
                    wait_for = min(wait_for, max(0.001, settle_remaining))
                self._cv.wait(timeout=wait_for)

    def status(self) -> dict:
        with self._cv:
            return {
                "available": self._available,
                "topic": self.topic,
                "current_state": self._current_state,
                "event_seq": self._event_seq,
                "last_event_ts": self._last_event_ts,
                "idle_settle_sec": self._idle_settle_sec,
                "connect_error": self._connect_error,
            }

    def close(self) -> None:
        subscriber = self._subscriber
        self._subscriber = None
        if subscriber is not None:
            try:
                subscriber.Close()
            except Exception:
                pass
        with self._cv:
            self._available = False
            self._cv.notify_all()
