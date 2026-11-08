"""Watch7 shared, bounded telemetry state; never synthesizes health readings.

Wire validators are copied unchanged from the existing, tested research gateway.
ROS2 publishers live in the card adapters, not in the transport.
"""
from __future__ import annotations

import threading
import time
from collections import OrderedDict
from typing import Callable

from protocol_v1 import validate_frame
from protocol_v2 import validate_v2

MAX_AGE_MS = 15_000
MAX_SEQUENCES = 128


class DeviceStore:
    def __init__(
        self,
        *,
        on_heart: Callable[[dict], None] | None = None,
        on_device: Callable[[dict], None] | None = None,
        allow_simulated: bool = False,
        wall_ms: Callable[[], int] | None = None,
        monotonic_ms: Callable[[], int] | None = None,
    ):
        self._lock = threading.RLock()
        self._wall = wall_ms or (lambda: int(time.time() * 1000))
        self._mono = monotonic_ms or (lambda: int(time.monotonic() * 1000))
        self._allow_simulated = allow_simulated
        self._on_heart = on_heart or (lambda payload: None)
        self._on_device = on_device or (lambda payload: None)
        self._seq: OrderedDict[tuple, int] = OrderedDict()
        self._v1: tuple[dict, int] | None = None
        self._v2_heart: tuple[dict, int] | None = None
        self._state: tuple[dict, int] | None = None

    def _accept_seq(self, key: tuple, seq: int) -> bool:
        if seq <= self._seq.get(key, -1):
            return False
        self._seq[key] = seq
        self._seq.move_to_end(key)
        while len(self._seq) > MAX_SEQUENCES:
            self._seq.popitem(last=False)
        return True

    def _is_fresh(self, record: tuple[dict, int] | None) -> bool:
        return record is not None and 0 <= self._mono() - record[1] <= MAX_AGE_MS

    def _blocked_by_wear(self, session: str) -> bool:
        if not self._is_fresh(self._state):
            return False
        state = self._state[0]
        if state["session_id"] != session:
            return True  # current device heartbeat belongs to another session
        return state["wear_state"] == "off_body" or not state["online"]

    def ingest_v1(self, frame: object) -> dict:
        try:
            clean = validate_frame(frame, now_ms=self._wall())
        except ValueError as exc:
            return {"ok": False, "reason": "bad_input", "detail": str(exc)}
        if clean["source"] != "watch7_real" and not self._allow_simulated:
            return {"ok": False, "reason": "simulated_disabled"}
        with self._lock:
            key = ("v1", clean["device_id"], clean["session_id"])
            if not self._accept_seq(key, clean["seq"]):
                return {"ok": False, "reason": "duplicate_or_out_of_order"}
            if self._blocked_by_wear(clean["session_id"]):
                return {"ok": False, "reason": "off_body_or_wrong_session"}
            self._v1 = (dict(clean), self._mono())
            event = {
                "timestamp_ms": clean["captured_at_ms"],
                "heart_rate_bpm": clean["bpm"], "unit": "bpm",
                "quality": "valid", "source": clean["source"],
                "session_id": clean["session_id"],
                "seq": clean["seq"], "protocol_version": 1,
                "wear_state": "unknown", "fresh": True,
            }
        self._on_heart(event)
        return {"ok": True, "seq": clean["seq"]}

    def ingest_v2(self, frame: object) -> dict:
        try:
            _, clean = validate_v2(frame, now_ms=self._wall())
        except ValueError as exc:
            return {"ok": False, "reason": "bad_input", "detail": str(exc)}
        if clean["source"] != "watch7_real" and not self._allow_simulated:
            return {"ok": False, "reason": "simulated_disabled"}
        # Waveform transfer/indexing is deferred to a later, separately reviewed PR.
        if clean["kind"] not in {"summary", "device_state"}:
            return {"ok": False, "reason": "kind_not_enabled"}
        event_type = None
        event = None
        with self._lock:
            key = ("v2", clean["device_id"], clean["session_id"], clean["kind"])
            if not self._accept_seq(key, clean["seq"]):
                return {"ok": False, "reason": "duplicate_or_out_of_order"}
            arrival = self._mono()
            if clean["kind"] == "device_state":
                self._state = (dict(clean), arrival)
                event_type = "device"
                event = {
                    "timestamp_ms": clean["captured_at_ms"], "source": clean["source"],
                    "session_id": clean["session_id"], "seq": clean["seq"],
                    "wear_state": clean["wear_state"], "online": clean["online"],
                    "collecting": clean["collecting"], "fresh": True,
                }
                for field in ("battery_pct", "charging", "heart_permission", "app_version"):
                    if field in clean:
                        event[field] = clean[field]
            else:
                metric = clean["metrics"].get("heart_rate_bpm")
                if (metric is not None and metric["quality"] == "valid"
                        and metric["value"] is not None
                        and metric["age_ms"] <= MAX_AGE_MS
                        and clean["wear_state"] != "off_body"
                        and not self._blocked_by_wear(clean["session_id"])):
                    self._v2_heart = (dict(clean), arrival)
                    event_type = "heart"
                    event = {
                        "timestamp_ms": clean["captured_at_ms"] - metric["age_ms"],
                        "heart_rate_bpm": metric["value"], "unit": "bpm",
                        "quality": metric["quality"], "source": clean["source"],
                        "session_id": clean["session_id"],
                        "seq": clean["seq"], "protocol_version": 2,
                        "provider": metric["provider"],
                        "wear_state": clean["wear_state"], "fresh": True,
                    }
        if event_type == "heart":
            self._on_heart(event)
        elif event_type == "device":
            self._on_device(event)
        return {"ok": True, "kind": clean["kind"], "seq": clean["seq"]}

    def heart_snapshot(self) -> dict:
        with self._lock:
            # Prefer v2 when present; fall back to the already validated legacy v1.
            for record, version in ((self._v2_heart, 2), (self._v1, 1)):
                if not self._is_fresh(record):
                    continue
                clean, arrived = record
                session = clean["session_id"]
                if self._blocked_by_wear(session):
                    continue
                if version == 2:
                    metric = clean["metrics"].get("heart_rate_bpm", {})
                    if (clean["wear_state"] == "off_body"
                            or metric.get("quality") != "valid"
                            or metric.get("age_ms", MAX_AGE_MS + 1) > MAX_AGE_MS):
                        continue
                    age = self._mono() - arrived + metric["age_ms"]
                    value = metric["value"]
                else:
                    age = self._mono() - arrived
                    value = clean["bpm"]
                if age <= MAX_AGE_MS:
                    return {
                        "available": True, "fresh": True, "heart_rate_bpm": value,
                        "unit": "bpm", "sample_age_ms": age,
                        "source": clean["source"], "session_id": session,
                        "protocol_version": version,
                        "timestamp_ms": clean["captured_at_ms"],
                    }
            return {
                "available": False, "fresh": False, "heart_rate_bpm": None,
                "unit": "bpm", "reason": "no_fresh_valid_heart_rate",
            }

    def device_snapshot(self) -> dict:
        with self._lock:
            if self._is_fresh(self._state):
                clean, arrived = self._state
                return {
                    "available": True, "fresh": True,
                    "online": bool(clean["online"]), "collecting": bool(clean["collecting"]),
                    "wear_state": clean["wear_state"],
                    "battery_pct": clean.get("battery_pct"),
                    "source": clean["source"], "session_id": clean["session_id"],
                    "heartbeat_age_ms": self._mono() - arrived,
                    "timestamp_ms": clean["captured_at_ms"],
                    "app_version": clean.get("app_version"),
                }
            if self._is_fresh(self._v1):
                clean, arrived = self._v1
                return {
                    "available": True, "fresh": True, "online": True,
                    "collecting": True, "wear_state": "unknown",
                    "battery_pct": None, "source": clean["source"],
                    "session_id": clean["session_id"],
                    "heartbeat_age_ms": self._mono() - arrived,
                    "timestamp_ms": clean["captured_at_ms"],
                }
            return {
                "available": False, "fresh": False,
                "online": False, "collecting": False,
                "wear_state": "unknown", "battery_pct": None,
                "reason": "no_fresh_watch7_heartbeat",
            }
