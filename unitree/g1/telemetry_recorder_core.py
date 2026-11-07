"""Bounded, receive-time telemetry clips. No ROS, SDK, or motion commands.

The clock domain is the receiving process's monotonic clock. Original payloads
are preserved, but their timestamps are never silently treated as synchronized.
"""
from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import threading
import time
from uuid import uuid4


def number(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be in [{low}, {high}]")
    return value


def utc_stamp(seconds):
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat()


def _reject_constant(value):
    raise ValueError(f"non-finite JSON value: {value}")


@dataclass(frozen=True)
class Sample:
    source: str
    mono: float
    utc: str
    payload: str
    size: int


@dataclass
class Clip:
    id: str
    label: str
    trigger: float
    utc: str
    pre_s: float
    post_s: float
    samples: list = field(default_factory=list)
    size: int = 0
    truncated: bool = False


class Recorder:
    """Thread-safe ring and a bounded number of pending clips.

    Callbacks only validate/buffer. A separate worker drains completed clips and
    writes them. Bounds are serialized payload bytes and entry counts, not an
    assertion about Python's exact RSS. All data retained in clips is bounded too.
    """
    def __init__(self, sources, *, buffer_s=30, max_buffer_bytes=4_000_000,
                 max_samples=10000, max_message_bytes=65536,
                 max_clip_bytes=4_000_000, max_pending=2,
                 auto_events=(), cooldown_s=10, pre_s=10, post_s=5,
                 clock=time.monotonic, wall=time.time):
        if not sources or len(sources) > 8:
            raise ValueError("one to eight sources required")
        self.sources = json.loads(json.dumps(sources, allow_nan=False))
        self.buffer_s = number(buffer_s, 1, 120, "buffer_s")
        for key, limits in {
            "max_buffer_bytes": (1024, 32_000_000),
            "max_samples": (1, 50000), "max_message_bytes": (128, 262144),
            "max_clip_bytes": (1024, 8_000_000), "max_pending": (1, 4),
        }.items():
            value = locals()[key]
            if type(value) is not int:
                raise ValueError(f"{key} must be an integer")
            setattr(self, key, number(value, *limits, key))
        self.default_pre = number(pre_s, 0, self.buffer_s, "pre_s")
        self.default_post = number(post_s, 0, 30, "post_s")
        self.cooldown_s = number(cooldown_s, 1, 120, "cooldown_s")
        for spec in self.sources.values():
            if not isinstance(spec, dict) or not isinstance(spec.get("topic"), str):
                raise ValueError("each source needs a topic")
            if not spec.get("event_stream", False):
                number(spec.get("stale_s"), .01, 300, "stale_s")
        self.auto_events = frozenset(auto_events)
        self._clock, self._wall = clock, wall
        self._lock = threading.RLock()
        self._ring = deque()
        self._bytes = 0
        self._pending = {}
        self._latest = {}
        self._received = Counter()
        self._rejected = Counter()
        self._evicted = 0
        self._auto_suppressed = 0
        self._last_auto = -math.inf
        self._running = False

    def start(self):
        with self._lock:
            self._running = True

    def _prune(self, now):
        while self._ring and (
            self._ring[0].mono < now - self.buffer_s
            or self._bytes > self.max_buffer_bytes
            or len(self._ring) > self.max_samples
        ):
            self._bytes -= self._ring.popleft().size
            self._evicted += 1

    def ingest_json(self, source, text):
        with self._lock:
            if not self._running:
                return False
            if source not in self.sources:
                self._rejected["unknown_source"] += 1
                return False
            if not isinstance(text, str) or len(text) > self.max_message_bytes:
                self._rejected["message_size"] += 1
                return False
            try:
                if len(text.encode("utf-8")) > self.max_message_bytes:
                    raise ValueError("oversized UTF-8")
                payload = json.loads(text, parse_constant=_reject_constant)
                if not isinstance(payload, dict):
                    raise ValueError("expected JSON object")
                frozen = json.dumps(payload, ensure_ascii=False, allow_nan=False,
                                    separators=(",", ":"))
            except (ValueError, TypeError, RecursionError, UnicodeError):
                self._rejected["invalid_json"] += 1
                return False
            now = self._clock()
            sample = Sample(source, now, utc_stamp(self._wall()), frozen,
                            len(frozen.encode("utf-8")) + 192)
            self._received[source] += 1
            self._latest[source] = now
            self._ring.append(sample)
            self._bytes += sample.size
            self._prune(now)
            for clip in self._pending.values():
                if now <= clip.trigger + clip.post_s:
                    if clip.size + sample.size <= self.max_clip_bytes and len(clip.samples) < self.max_samples:
                        clip.samples.append(sample)
                        clip.size += sample.size
                    else:
                        clip.truncated = True
            if source == "motion_events" and isinstance(payload.get("type"), str) and payload["type"] in self.auto_events:
                if now - self._last_auto >= self.cooldown_s and len(self._pending) < self.max_pending:
                    self._capture("event: " + str(payload["type"]), now,
                                  self.default_pre, self.default_post)
                    self._last_auto = now
                else:
                    self._auto_suppressed += 1
            return True

    def _capture(self, label, now, pre, post):
        self._prune(now)
        clip = Clip(uuid4().hex, label, now, utc_stamp(self._wall()), pre, post)
        candidates = [s for s in self._ring if now - pre <= s.mono <= now]
        # Keep the most recent pre-trigger evidence if a size cap is hit.
        for sample in reversed(candidates):
            if clip.size + sample.size > self.max_clip_bytes or len(clip.samples) >= self.max_samples:
                clip.truncated = True
                break
            clip.samples.append(sample)
            clip.size += sample.size
        clip.samples.reverse()
        self._pending[clip.id] = clip
        return {"clip_id": clip.id, "status": "capturing",
                "pre_s": pre, "post_s": post, "label": label}

    def capture(self, label="manual", pre_s=None, post_s=None):
        if not isinstance(label, str) or not 1 <= len(label) <= 120:
            raise ValueError("label must contain 1 to 120 characters")
        pre = self.default_pre if pre_s is None else number(pre_s, 0, self.buffer_s, "pre_s")
        post = self.default_post if post_s is None else number(post_s, 0, 30, "post_s")
        with self._lock:
            if not self._running:
                raise ValueError("recorder is not started")
            if len(self._pending) >= self.max_pending:
                raise ValueError("pending clip limit reached; wait for completion")
            return self._capture(label, self._clock(), pre, post)

    def _detach(self, force):
        with self._lock:
            now = self._clock()
            ready = []
            for key, clip in list(self._pending.items()):
                if force or now >= clip.trigger + clip.post_s:
                    ready.append((self._pending.pop(key), now,
                                  now >= clip.trigger + clip.post_s))
            return ready

    def drain_ready(self, force=False):
        return [self._document(clip, now, complete)
                for clip, now, complete in self._detach(force)]

    def stop(self):
        with self._lock:
            self._running = False
            documents = self.drain_ready(force=True)
            self._ring.clear()
            self._bytes = 0
            self._latest.clear()
            return documents

    def status(self):
        with self._lock:
            now = self._clock()
            self._prune(now)
            source_status = {}
            for name, spec in self.sources.items():
                last = self._latest.get(name)
                age = None if last is None else max(0, now - last)
                state = ("no_events_observed" if last is None else "event_observed") if spec.get("event_stream") else (
                    "never_received" if last is None else "stale" if age > spec["stale_s"] else "receiving")
                source_status[name] = {"state": state, "receive_age_s": age,
                                       "received": self._received[name], **spec}
            return {"recording": self._running, "sources": source_status,
                    "buffer_samples": len(self._ring), "buffer_payload_budget_bytes": self._bytes,
                    "evicted_samples": self._evicted, "rejected_messages": dict(self._rejected),
                    "auto_suppressed": self._auto_suppressed,
                    "pending": [{"clip_id": c.id, "label": c.label,
                                 "remaining_s": max(0, c.trigger + c.post_s - now)}
                                for c in self._pending.values()]}

    def _document(self, clip, end, complete):
        with self._lock:
            receiver_totals = {"scope": "process cumulative, not clip-local",
                               "rejected_messages": dict(self._rejected),
                               "ring_evicted_samples": self._evicted,
                               "auto_suppressed": self._auto_suppressed}
        quality = {}
        for name, spec in self.sources.items():
            times = [s.mono - clip.trigger for s in clip.samples if s.source == name]
            gaps = [b - a for a, b in zip(times, times[1:])]
            prior = [t for t in times if t <= 0]
            trigger_age = -prior[-1] if prior else None
            quality[name] = {
                "sample_count": len(times), "first_receive_offset_s": times[0] if times else None,
                "last_receive_offset_s": times[-1] if times else None,
                "max_observed_gap_s": max(gaps) if gaps else None,
                "trigger_receive_age_s": trigger_age,
                "trigger_state": ("event_stream" if spec.get("event_stream") else
                                  "unknown" if trigger_age is None else
                                  "stale" if trigger_age > spec["stale_s"] else "recent"),
            }
        return {"schema": "g1.telemetry_clip/1", "clip_id": clip.id,
                "label": clip.label, "trigger_utc": clip.utc,
                "requested_pre_s": clip.pre_s, "requested_post_s": clip.post_s,
                "post_window_elapsed": complete, "interrupted": not complete,
                "truncated": clip.truncated, "sources": self.sources, "quality": quality,
                "receiver_totals_at_export": receiver_totals,
                "alignment": "receiver monotonic time; not hardware synchronization",
                "limitations": ["No physical fault diagnosis or safety certification",
                                "Missing/event-only streams do not prove health or failure",
                                "Raw payload units and actual joint mapping require site validation"],
                "samples": [{"source": s.source, "receive_offset_s": s.mono - clip.trigger,
                             "receive_utc": s.utc, "payload": json.loads(s.payload)}
                            for s in clip.samples]}
