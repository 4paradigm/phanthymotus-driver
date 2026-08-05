"""G1 body-speaker playback state observed from Unitree DDS.

The G1 controller publishes JSON messages on ``rt/audio_msg`` with
``{"play_state": 1}`` when its body-speaker player starts and
``{"play_state": 0}`` when the player becomes idle.  Unitree's public
``AudioClient`` does not expose that state through its RPC API, so the speaker
driver correlates the global state transitions with its serialized PCM
submissions.
"""

from __future__ import annotations

from collections import deque
import json
import threading
import time
from typing import Callable


class G1PlaybackStateMonitor:
    """Track body-speaker state transitions and correlate one serial stream."""

    def __init__(self, topic: str = "rt/audio_msg", *, available: bool = False,
                 connect_error: str = "not_connected", monotonic=None,
                 history_limit: int = 256, ready: bool | None = None,
                 idle_settle_sec: float = 0.1):
        self.topic = topic
        self._monotonic = monotonic or time.monotonic
        self._cv = threading.Condition(threading.RLock())
        self._events = deque(maxlen=max(8, int(history_limit)))
        self._observations: dict[tuple[int, str], dict[str, int]] = {}
        self._event_seq = 0
        self._current_state: int | None = None
        self._last_event_ts: float | None = None
        self._available = bool(available)
        self._connect_error = "" if available else connect_error
        self._ready = bool(available) if ready is None else bool(ready)
        self._matched_publishers = 1 if self._ready else 0
        self._idle_settle_sec = max(0.0, float(idle_settle_sec))
        self._subscriber = None

    @classmethod
    def connect_dds(cls, topic: str = "rt/audio_msg", *, monotonic=None):
        """Create the real Unitree DDS subscriber without failing bundle load."""
        monitor = cls(topic, monotonic=monotonic)
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

            subscriber = ChannelSubscriber(topic, String_)
            # play_state is sparse and ordering-sensitive. Handle it directly
            # in the DDS callback instead of adding a second SDK queue whose
            # scheduling could move a pre-ACK idle event past the ACK gate.
            subscriber.Init(
                monitor.on_dds_message, 0,
                monitor.on_subscription_matched,
            )
            monitor._subscriber = subscriber
            with monitor._cv:
                monitor._available = True
                monitor._connect_error = ""
        except Exception as exc:
            with monitor._cv:
                monitor._available = False
                monitor._ready = False
                monitor._matched_publishers = 0
                monitor._connect_error = (
                    f"dds_subscribe_failed:{type(exc).__name__}:{exc}"
                )
        return monitor

    @property
    def available(self) -> bool:
        with self._cv:
            return self._available

    def checkpoint(self) -> int:
        """Return an event sequence checkpoint captured before a PCM submit."""
        with self._cv:
            return self._event_seq

    def on_subscription_matched(self, current_count: int) -> None:
        with self._cv:
            self._matched_publishers = max(0, int(current_count))
            self._ready = self._matched_publishers > 0
            self._cv.notify_all()

    def wait_until_ready(self, *, timeout: float,
                         cancelled: Callable[[], bool] | None = None,
                         poll_interval: float = 0.05) -> dict:
        """Wait until DDS discovery has matched the firmware state writer."""
        if cancelled is None:
            cancelled = lambda: False
        deadline = self._monotonic() + max(0.0, float(timeout))
        while True:
            if cancelled():
                return {"state": "interrupted", "reason": "interrupted"}
            with self._cv:
                if not self._available:
                    return {
                        "state": "error",
                        "reason": self._connect_error or "play_state_unavailable",
                    }
                if self._ready:
                    return {"state": "ready", "reason": ""}
                remaining = deadline - self._monotonic()
                if remaining <= 0:
                    return {
                        "state": "timeout",
                        "reason": "play_state_publisher_timeout",
                    }
                self._cv.wait(timeout=min(max(0.001, poll_interval), remaining))

    def note_submission(self, key: tuple[int, str], checkpoint: int) -> None:
        """Record a successful PCM submission and its pre-submit checkpoint."""
        with self._cv:
            # A previous block can become idle after the caller's pre-submit
            # checkpoint but before PlayStream acknowledges the new block.  An
            # idle from that window belongs to the previous block and must not
            # complete the utterance.  Requiring the final idle to follow the
            # successful acknowledgement checkpoint fails closed for that race.
            acknowledged_seq = max(int(checkpoint), self._event_seq)
            observation = self._observations.get(key)
            if observation is None:
                observation = {
                    "first_after_seq": int(checkpoint),
                    "idle_after_seq": acknowledged_seq,
                }
                self._observations[key] = observation
            else:
                observation["idle_after_seq"] = acknowledged_seq
            self._cv.notify_all()

    def forget(self, key: tuple[int, str]) -> None:
        with self._cv:
            self._observations.pop(key, None)

    def on_dds_message(self, msg) -> None:
        raw = getattr(msg, "data", None)
        if raw is None:
            # Some older generated Unitree IDL bindings use a trailing suffix.
            raw = getattr(msg, "data_", None)
        self.on_raw_message(raw)

    def on_raw_message(self, raw) -> bool:
        """Accept one JSON String_ payload; return whether it was play state."""
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

    def _evaluate_locked(self, key: tuple[int, str]) -> dict:
        observation = self._observations.get(key)
        if observation is None:
            return {
                "state": "error",
                "reason": "play_state_observation_missing",
            }

        first_after = observation["first_after_seq"]
        idle_after = observation["idle_after_seq"]
        started = None
        idle = None
        for sequence, state, timestamp in self._events:
            if sequence <= first_after:
                continue
            if state == 1:
                # A later playing transition invalidates an earlier idle
                # candidate. This covers a delayed start notification for the
                # final block without losing the initial stream start.
                started = (sequence, timestamp)
                idle = None
                continue
            if (started is not None and sequence > idle_after
                    and sequence > started[0] and state == 0):
                idle = (sequence, timestamp)

        if idle is not None:
            settled_for = self._monotonic() - idle[1]
            if settled_for >= self._idle_settle_sec:
                return {
                    "state": "completed",
                    "reason": "",
                    "playing_seq": started[0],
                    "idle_seq": idle[0],
                    "playing_ts": started[1],
                    "idle_ts": idle[1],
                }

        return {
            "state": "waiting",
            "reason": "",
            "playing_seq": started[0] if started else None,
            "idle_seq": idle[0] if idle else None,
        }

    def wait_for_completion(self, key: tuple[int, str], *, timeout: float,
                            cancelled: Callable[[], bool] | None = None,
                            poll_interval: float = 0.05) -> dict:
        """Wait for playing→idle after the first and last PCM submissions."""
        if cancelled is None:
            cancelled = lambda: False
        with self._cv:
            if not self._available:
                return {
                    "state": "error",
                    "reason": self._connect_error or "play_state_unavailable",
                }

        deadline = self._monotonic() + max(0.0, float(timeout))
        while True:
            if cancelled():
                return {"state": "interrupted", "reason": "interrupted"}
            with self._cv:
                result = self._evaluate_locked(key)
                if result["state"] != "waiting":
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
                    }
                self._cv.wait(timeout=min(max(0.001, poll_interval), remaining))

    def status(self) -> dict:
        with self._cv:
            return {
                "available": self._available,
                "ready": self._ready,
                "matched_publishers": self._matched_publishers,
                "topic": self.topic,
                "current_state": self._current_state,
                "event_seq": self._event_seq,
                "last_event_ts": self._last_event_ts,
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
            self._ready = False
            self._matched_publishers = 0
            self._cv.notify_all()
