"""Model-free region occupancy from the existing VOP JSON contract.

Coordinates are VOP centre-relative [-1, 1], not pixels or metres. This is
aggregate occupancy, not identity tracking or a count of people entering.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math


SCHEMA = "motus.region_presence/1"


def qr_guidance(data):
    """Inspect one camera JPEG without reading the contents of the code."""
    import cv2
    from common.image_quality import QualityConfig, decode_jpeg

    gray = cv2.cvtColor(decode_jpeg(data), cv2.COLOR_BGR2GRAY)
    height, width = gray.shape
    found, points = cv2.QRCodeDetector().detect(gray)
    sample = gray
    hints = []
    size_ratio = None
    if found:
        corners = points.reshape(4, 2)
        x, y = corners.mean(axis=0)
        if y < height / 3:
            hints.append("请把二维码往下移一些。")
        elif y > height * 2 / 3:
            hints.append("请把二维码往上移一些。")
        if x < width / 3:
            hints.append("请把二维码往右移一些。")
        elif x > width * 2 / 3:
            hints.append("请把二维码往左移一些。")
        left, top, w, h = cv2.boundingRect(corners)
        sample = gray[max(0, top):top+h, max(0, left):left+w]
        size_ratio = min(w, h) / min(width, height)
        if size_ratio < 0.15:
            hints.append("二维码在画面里太小，请把它靠近镜头一些。")
    brightness = float(sample.mean())
    thresholds = QualityConfig()
    if brightness > thresholds.max_brightness:
        hints.append("画面太亮，请避开反光，或把展示二维码的屏幕调暗一些。")
    elif brightness < thresholds.min_brightness:
        hints.append("画面太暗，请增加照明，或把展示二维码的屏幕调亮一些。")
    if not found and not hints:
        hints.append("请把完整的二维码放在画面中央。")
    framing_ok = bool(found) and not hints
    if framing_ok:
        hints.append("二维码已对齐，请保持不动。")
    return {"state": "analyzed", "qr_located": bool(found), "hints": hints,
            "framing_ok": framing_ok, "size_ratio": size_ratio,
            "mean_brightness": round(brightness, 1)}


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    if not low <= value <= high or not math.isfinite(value):
        raise ValueError(f"{name} must be in [{low}, {high}]")
    return float(value)


@dataclass(frozen=True)
class PresenceConfig:
    target_name: str = "person"
    min_confidence: float = .5
    x_min: float = -1.
    x_max: float = 1.
    y_min: float = -1.
    y_max: float = 1.
    enter_hold_s: float = .6
    leave_hold_s: float = 1.5
    stale_after_s: float = 3.
    dwell_alert_s: float = 0.
    capacity_limit: int = 0

    def __post_init__(self):
        if (not isinstance(self.target_name, str) or not self.target_name.strip()
                or len(self.target_name) > 128):
            raise ValueError("target_name must be a non-empty VOP class name (max 128 chars)")
        for key in ("x_min", "x_max", "y_min", "y_max"):
            number(getattr(self, key), key, -1, 1)
        number(self.min_confidence, "min_confidence", 0, 1)
        for key in ("enter_hold_s", "leave_hold_s"):
            number(getattr(self, key), key, 0, 60)
        number(self.stale_after_s, "stale_after_s", .2, 120)
        number(self.dwell_alert_s, "dwell_alert_s", 0, 86400)
        if type(self.capacity_limit) is not int or not 0 <= self.capacity_limit <= 256:
            raise ValueError("capacity_limit must be an integer in [0, 256]")
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("ROI requires x_min < x_max and y_min < y_max")

    def updated(self, changes):
        unknown = set(changes) - set(asdict(self))
        if unknown:
            raise ValueError(f"Unknown configuration fields: {sorted(unknown)}")
        return replace(self, **changes)


def _observe_targets(payload, config):
    """Validate the entire VOP frame before interpreting an empty result."""
    if not isinstance(payload, dict):
        raise ValueError("Expected a VOP JSON object")
    stamp = number(payload.get("timestamp"), "VOP timestamp", 0, 1e12)
    objects = payload.get("objects")
    if not isinstance(objects, list) or len(objects) > 256:
        raise ValueError("VOP objects must be a list of at most 256 detections")
    count = payload.get("count")
    if type(count) is not int or count != len(objects):
        raise ValueError("VOP count must equal len(objects)")
    matches = 0
    positions = []
    for obj in objects:
        if not isinstance(obj, dict) or not isinstance(obj.get("name"), str) or not obj["name"].strip():
            raise ValueError("Each VOP object requires a class name")
        score = number(obj.get("confidence"), "confidence", 0, 1)
        position = obj.get("position")
        if not isinstance(position, list) or len(position) != 2:
            raise ValueError("VOP position must be [x, y] in centre-relative [-1, 1] coordinates")
        x, y = (number(v, "position", -1, 1) for v in position)
        if (obj["name"].strip().casefold() == config.target_name.strip().casefold()
                and score >= config.min_confidence):
            positions.append((x, y))
            if config.x_min <= x <= config.x_max and config.y_min <= y <= config.y_max:
                matches += 1
    return stamp, matches, positions


def count_matches(payload, config):
    stamp, count, _ = _observe_targets(payload, config)
    return stamp, count


def position_guidance(positions, config):
    if positions is None:
        return {"state": "unavailable", "hints": []}
    if not positions:
        return {"state": "not_found", "hints": ["请让目标进入画面。"]}
    if len(positions) > 1:
        return {"state": "multiple", "hints": ["画面里有多个同类目标，请单独展示要调整的目标。"]}
    x, y = positions[0]
    dx = (x - config.x_min) / (config.x_max - config.x_min)
    dy = (y - config.y_min) / (config.y_max - config.y_min)
    hints = []
    if dy < 1/3:
        hints.append("请让目标在画面中往下移一些。")
    elif dy > 2/3:
        hints.append("请让目标在画面中往上移一些。")
    if dx < 1/3:
        hints.append("请让目标在画面中往右移一些。")
    elif dx > 2/3:
        hints.append("请让目标在画面中往左移一些。")
    return {"state": "adjust" if hints else "aligned", "position": [x, y], "hints": hints}


class PresenceMonitor:
    """Caller supplies a monotonic clock. Only new source frames confirm dwell."""

    def __init__(self, config=None):
        self.config = config or PresenceConfig()
        self.stable_state = "unknown"
        self.state = "no_data"
        self.last_received = self.last_source_stamp = None
        self.detected_count = None
        self.positions = None
        self.candidate = self.candidate_since = None
        self.samples = self.sequence = 0
        self.message = None
        self.occupied_since = None
        self.dwell_alerted = False
        self.over_capacity = self.capacity_candidate = self.capacity_since = None

    def _capacity_events(self, count, now):
        if not self.config.capacity_limit:
            return []
        above = count > self.config.capacity_limit
        if above != self.capacity_candidate:
            self.capacity_candidate, self.capacity_since = above, now
            return []
        hold = self.config.enter_hold_s if above else self.config.leave_hold_s
        if above == self.over_capacity or now - self.capacity_since < hold:
            return []
        previously_above = self.over_capacity
        self.over_capacity = above
        if not above and not previously_above:
            return []
        name = "capacity_exceeded" if above else "capacity_restored"
        return [{**self._event(name, self.stable_state), "capacity_limit": self.config.capacity_limit,
                 "over_capacity": above}]

    def occupied_seconds(self):
        if self.occupied_since is None or self.state != "ready":
            return None
        return self.last_received - self.occupied_since

    def _event(self, event, previous):
        self.sequence += 1
        return {"schema": SCHEMA, "event": event, "sequence": self.sequence,
                "priority": 1, "previous_state": previous,
                "state": self.state, "stable_state": self.stable_state,
                "present": {"occupied": True, "empty": False}.get(self.stable_state),
                "detected_count": self.detected_count,
                "occupied_s": self.occupied_seconds(),
                "target_name": self.config.target_name,
                "source_timestamp": self.last_source_stamp, "message": self.message}

    def unavailable(self, reason, message=None):
        if self.state == reason:
            return []
        previous = self.stable_state
        self.stable_state, self.state = "unknown", reason
        self.positions = None
        self.occupied_since, self.dwell_alerted = None, False
        self.over_capacity = self.capacity_candidate = self.capacity_since = None
        self.candidate = self.candidate_since = None
        self.samples = 0
        self.detected_count = None
        self.message = message
        return [self._event("unavailable", previous)]

    def poll(self, now):
        if (self.last_received is not None and self.state != "invalid_data"
                and now - self.last_received > self.config.stale_after_s):
            return self.unavailable("stale", "No new VOP publication within stale_after_s")
        return []

    def ingest(self, payload, now):
        events = self.poll(now)
        try:
            stamp, count, positions = _observe_targets(payload, self.config)
        except ValueError as error:
            return events + self.unavailable("invalid_data", str(error))
        # Replayed DDS messages must not refresh freshness or confirm dwell.
        # Rebind/start resets this publication-clock baseline.
        if self.last_source_stamp is not None and stamp <= self.last_source_stamp:
            return events
        self.last_source_stamp, self.last_received = stamp, now
        self.positions = positions
        self.detected_count, self.message = count, None
        desired = "occupied" if count else "empty"
        if desired != self.candidate:
            self.candidate, self.candidate_since, self.samples = desired, now, 1
        else:
            self.samples += 1
        if desired == self.stable_state:
            self.state = "ready"
        else:
            self.state = "stabilizing"
            dwell = self.config.enter_hold_s if count else self.config.leave_hold_s
            if self.samples >= 2 and now - self.candidate_since >= dwell:
                previous = self.stable_state
                self.stable_state, self.state = desired, "ready"
                self.occupied_since = now if count else None
                self.dwell_alerted = False
                event = ("initialized" if previous == "unknown" else
                         "appeared" if count else "disappeared")
                events.append(self._event(event, previous))
        elapsed = self.occupied_seconds()
        if (elapsed is not None and self.config.dwell_alert_s > 0
                and elapsed >= self.config.dwell_alert_s and not self.dwell_alerted):
            self.dwell_alerted = True
            events.append(self._event("dwell_reached", self.stable_state))
        events.extend(self._capacity_events(count, now))
        return events

    def snapshot(self, now):
        return {"schema": SCHEMA, "state": self.state,
                "stable_state": self.stable_state,
                "present": ({"occupied": True, "empty": False}.get(self.stable_state)
                            if self.state == "ready" else None),
                "candidate": self.candidate, "confirming_samples": self.samples,
                "position_guidance": position_guidance(self.positions, self.config),
                "capacity_limit": self.config.capacity_limit, "over_capacity": self.over_capacity,
                "occupied_s": self.occupied_seconds(),
                "detected_count": self.detected_count,
                "target_name": self.config.target_name,
                "source_timestamp": self.last_source_stamp,
                "received_age_s": (round(max(0., now - self.last_received), 4)
                                   if self.last_received is not None else None),
                "freshness_basis": "local_receive_time_of_new_publication",
                "capture_time_verified": False, "message": self.message}
