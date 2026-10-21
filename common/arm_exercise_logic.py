"""Camera-relative dumbbell curl and lateral raise counters for pose skeletons.

These prototype thresholds use COCO-17 image coordinates. A caller must feed
frames in timestamp order and serialize calls to ``process`` and ``reset``.
The camera should show the upper body throughout each repetition; a front or
slightly oblique view is needed for lateral raises.
"""

from __future__ import annotations

import math
import time
import uuid
from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Any

from common.squat_logic import COCO_NAMES


EXERCISES = ("dumbbell_curl", "lateral_raise")

DEFAULT_CONFIG = {
    "min_visibility": 0.30,
    "initial_dwell_s": 0.35,
    "peak_dwell_s": 0.07,
    "return_dwell_s": 0.07,
    "min_rep_s": 0.45,
    "max_rep_s": 12.0,
    "smoothing_frames": 1,
    "max_frame_gap_s": 0.50,
    "lost_grace_s": 0.50,
    "switch_after_s": 1.0,
    "curl_rest_min_deg": 145.0,
    "curl_start_max_deg": 130.0,
    "curl_peak_max_deg": 105.0,
    "curl_upper_arm_vertical_min_ratio": 0.45,
    "curl_elbow_shift_max_ratio": 0.65,
    "curl_rest_wrist_drop_min_ratio": 0.35,
    "curl_peak_wrist_drop_max_ratio": 0.20,
    "lateral_shoulder_span_min_ratio": 0.45,
    "lateral_elbow_min_deg": 125.0,
    "lateral_rest_wrist_drop_min_ratio": 0.60,
    "lateral_start_wrist_drop_max_ratio": 0.45,
    "lateral_peak_wrist_level_max_ratio": 0.35,
    "lateral_peak_wrist_above_max_ratio": 0.55,
    "lateral_start_reach_min_ratio": 0.25,
    "lateral_peak_reach_min_ratio": 0.63,
    "lateral_peak_window_s": 0.60,
}


@dataclass(frozen=True)
class _Candidate:
    person_id: Any
    bbox: tuple[float, float, float, float]
    points: dict[str, tuple[float, float, float]]
    score: float
    readable: bool
    quality: float
    preferred_side: str | None


def _finite(value: Any, fallback: float = 0.0) -> float:
    try:
        number = float(value)
    except (ValueError, TypeError):
        return fallback
    return number if math.isfinite(number) else fallback


def _distance(a: tuple, b: tuple) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _angle(a: tuple, vertex: tuple, b: tuple) -> float | None:
    ax, ay = a[0] - vertex[0], a[1] - vertex[1]
    bx, by = b[0] - vertex[0], b[1] - vertex[1]
    divisor = math.hypot(ax, ay) * math.hypot(bx, by)
    if divisor < 1e-6:
        return None
    return math.degrees(math.acos(max(-1.0, min(1.0,
        (ax * bx + ay * by) / divisor))))


def _center(box: tuple[float, float, float, float]) -> tuple[float, float]:
    return ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)


def _iou(a: tuple[float, float, float, float],
         b: tuple[float, float, float, float]) -> float:
    width = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    height = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    overlap = width * height
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return overlap / max(1.0, area_a + area_b - overlap)


def _safe_id(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, float) and math.isfinite(value):
        return value
    return str(value)


def _summary(candidate: _Candidate) -> dict:
    return {"id": _safe_id(candidate.person_id), "bbox": list(candidate.bbox),
            "score": candidate.score, "quality": candidate.quality,
            "readable": candidate.readable, "side": candidate.preferred_side}


def _torso_scale(points: dict[str, tuple[float, float, float]], side: str,
                 min_visibility: float) -> float | None:
    shoulder = points.get(f"{side}_shoulder")
    elbow = points.get(f"{side}_elbow")
    if (shoulder is None or elbow is None or
            min(shoulder[2], elbow[2]) < min_visibility):
        return None
    upper_arm = _distance(shoulder, elbow)
    if upper_arm < 1:
        return None
    hip = points.get(f"{side}_hip")
    if hip is not None and hip[2] >= min_visibility:
        torso = _distance(shoulder, hip)
        if torso >= 1:
            return torso
    return 2.0 * upper_arm


class ArmExerciseCounter:
    """Count complete upper-body movement cycles for one locked person.

    ``dumbbell_curl`` chooses the arm that begins moving and holds it for one
    repetition. ``lateral_raise`` requires both arms.
    ``snapshot`` is authoritative; events contain sparse absolute totals.
    """

    def __init__(self, exercise: str, config: dict | None = None):
        if exercise not in EXERCISES:
            raise ValueError(f"unsupported arm exercise: {exercise}")
        self.exercise = exercise
        self.config = dict(DEFAULT_CONFIG)
        if config:
            unknown = set(config) - set(self.config)
            if unknown:
                raise ValueError(f"unknown arm exercise configuration: {sorted(unknown)}")
            self.config.update(config)
        for key, value in self.config.items():
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0):
                raise ValueError(f"{key} must be a nonnegative finite number")
        self.config["smoothing_frames"] = int(self.config["smoothing_frames"])
        if self.config["smoothing_frames"] < 1:
            raise ValueError("smoothing_frames must be at least 1")
        if self.config["switch_after_s"] < self.config["lost_grace_s"]:
            raise ValueError("switch_after_s must be >= lost_grace_s")
        if self.config["curl_peak_max_deg"] >= self.config["curl_rest_min_deg"]:
            raise ValueError("curl peak angle must be below rest angle")
        if self.config["min_rep_s"] > self.config["max_rep_s"]:
            raise ValueError("min_rep_s must be <= max_rep_s")
        self.reset()

    def reset(self) -> dict:
        self.session_id = uuid.uuid4().hex
        self.seq = 0
        self.count_total = 0
        self.target_id = None
        self._target_box: tuple[float, float, float, float] | None = None
        self._side: str | None = None
        self._state = "waiting_target"
        self._target_state = "none"
        self._calibrated = False
        self._last_frame_time: float | None = None
        self._last_seen_time: float | None = None
        self._loss_since: float | None = None
        self._loss_reported = False
        self._needs_rearm = False
        self._paused_state: str | None = None
        self._rest_since: float | None = None
        self._peak_since: float | None = None
        self._return_since: float | None = None
        self._motion_started: float | None = None
        self._recent: deque[dict[str, float]] = deque(
            maxlen=self.config["smoothing_frames"])
        self._recent_by_side = {
            side: deque(maxlen=self.config["smoothing_frames"])
            for side in ("left", "right")
        }
        self._rest_ready_sides: set[str] = set()
        self._lateral_peak_since: dict[str, float | None] = {
            "left": None, "right": None}
        self._lateral_peak_confirmed: dict[str, float | None] = {
            "left": None, "right": None}
        self._debug_sample: dict[str, float] | None = None
        self._debug_selected: dict | None = None
        self._debug_candidates: list[dict] = []
        self._debug_reason = "waiting_for_readable_person"
        self._debug_frame_gap_s: float | None = None
        return self.snapshot()

    @property
    def last_frame_time(self) -> float | None:
        return self._last_frame_time

    def snapshot(self) -> dict:
        return {
            "session_id": self.session_id, "seq": self.seq,
            "count_total": self.count_total, "target_id": self.target_id,
            "target_state": self._target_state,
            "candidate_count": len(self._debug_candidates),
            "state": self._state, "side": self._side,
            "calibrated": self._calibrated,
            "last_seen_time": self._last_seen_time,
        }

    def diagnostics(self) -> dict:
        sample = self._debug_sample or {}
        checks = self._checks(sample) if sample else {}
        selected = self._debug_selected
        now = self._last_frame_time
        def elapsed(since: float | None) -> float | None:
            return max(0.0, now - since) if now is not None and since is not None else None

        return {
            "frame_time": now,
            "frame_gap_s": self._debug_frame_gap_s,
            "frame_gap_exceeded": (self._debug_frame_gap_s > self.config["max_frame_gap_s"]
                                   if self._debug_frame_gap_s is not None else None),
            "state": self._state, "target_state": self._target_state,
            "needs_rearm": self._needs_rearm,
            "paused_state": self._paused_state,
            "rest_ready_sides": sorted(self._rest_ready_sides),
            "selected_target": {
                "id": _safe_id(self.target_id),
                "bbox": (list(selected["bbox"]) if selected is not None else
                         list(self._target_box) if self._target_box else None),
                "quality": selected["quality"] if selected else None,
                "readable": selected["readable"] if selected else None,
                "side": self._side, "present": selected is not None,
            },
            "candidate_count": len(self._debug_candidates),
            "candidates": sorted(self._debug_candidates,
                                 key=lambda item: item["quality"], reverse=True)[:8],
            "measurements": {"baseline": None, **self._measurement_template(), **sample},
            "checks": checks,
            "timing": {
                "calibration_elapsed_s": elapsed(self._rest_since)
                    if not self._calibrated else None,
                "motion_elapsed_s": elapsed(self._motion_started),
                "peak_dwell_elapsed_s": elapsed(self._peak_since),
                "return_dwell_elapsed_s": elapsed(self._return_since),
            },
            "lateral_peak_confirmed": dict(self._lateral_peak_confirmed),
            "thresholds": dict(self.config), "reason": self._debug_reason,
        }

    def _measurement_template(self) -> dict:
        if self.exercise == "dumbbell_curl":
            return {key: None for key in (
                "elbow_angle_deg", "upper_arm_vertical_ratio",
                "elbow_shift_ratio", "wrist_rise_ratio")}
        return {f"{side}_{key}": None for side in ("left", "right")
                for key in ("elbow_angle_deg", "wrist_drop_ratio",
                            "outward_reach_ratio")} | {"shoulder_span_ratio": None}

    def _checks(self, sample: dict[str, float]) -> dict[str, bool]:
        cfg = self.config
        if self.exercise == "dumbbell_curl":
            arm_stable = (sample["upper_arm_vertical_ratio"] >=
                          cfg["curl_upper_arm_vertical_min_ratio"] and
                          sample["elbow_shift_ratio"] <=
                          cfg["curl_elbow_shift_max_ratio"])
            rest = (arm_stable and
                    sample["elbow_angle_deg"] >= cfg["curl_rest_min_deg"] and
                    sample["wrist_rise_ratio"] <=
                    -cfg["curl_rest_wrist_drop_min_ratio"])
            start = (arm_stable and
                     sample["elbow_angle_deg"] <= cfg["curl_start_max_deg"])
            peak = (arm_stable and
                    sample["elbow_angle_deg"] <= cfg["curl_peak_max_deg"] and
                    sample["wrist_rise_ratio"] >=
                    -cfg["curl_peak_wrist_drop_max_ratio"])
            return {"arm_stable": arm_stable, "rest": rest,
                    "start": start, "peak": peak}
        bilateral = all(
            sample[f"{side}_elbow_angle_deg"] >= cfg["lateral_elbow_min_deg"]
            for side in ("left", "right"))
        front_view = sample["shoulder_span_ratio"] >= cfg[
            "lateral_shoulder_span_min_ratio"]
        rest = (bilateral and front_view and all(
            sample[f"{side}_wrist_drop_ratio"] >=
            cfg["lateral_rest_wrist_drop_min_ratio"]
            for side in ("left", "right")))
        start = (bilateral and front_view and any(
            sample[f"{side}_wrist_drop_ratio"] <=
                cfg["lateral_start_wrist_drop_max_ratio"] and
            sample[f"{side}_outward_reach_ratio"] >=
                cfg["lateral_start_reach_min_ratio"]
            for side in ("left", "right")))
        raised = [
            -cfg["lateral_peak_wrist_above_max_ratio"] <=
                sample[f"{side}_wrist_drop_ratio"] <=
                cfg["lateral_peak_wrist_level_max_ratio"] and
            sample[f"{side}_outward_reach_ratio"] >=
                cfg["lateral_peak_reach_min_ratio"]
            for side in ("left", "right")]
        return {"bilateral_visible": True, "arms_extended": bilateral,
                "front_view": front_view, "rest": rest, "start": start,
                "left_raised": raised[0], "right_raised": raised[1],
                "peak": bilateral and front_view and all(raised)}

    def process(self, payload: dict) -> list[dict]:
        if not isinstance(payload, dict):
            return []
        timestamp = _finite(payload.get("timestamp"), time.time())
        if self._last_frame_time is not None and timestamp <= self._last_frame_time:
            self._debug_reason = "non_monotonic_timestamp"
            return []
        gap = (timestamp - self._last_frame_time
               if self._last_frame_time is not None else 0.0)
        self._debug_frame_gap_s = gap if self._last_frame_time is not None else None
        self._last_frame_time = timestamp
        self._debug_sample = None
        self._debug_selected = None
        if gap > self.config["max_frame_gap_s"]:
            self._cancel_partial(require_rearm=True)
            self._recent.clear()
            for recent in self._recent_by_side.values():
                recent.clear()
            self._paused_state = None
        candidates = self._candidates(payload)
        self._debug_candidates = [_summary(candidate) for candidate in candidates]
        events: list[dict] = []
        if self.target_id is None:
            best = self._best(candidates)
            if best is not None:
                events.append(self._begin_session(best, timestamp, changed=False))
                self._advance(best, timestamp, events)
            else:
                self._debug_reason = "waiting_for_readable_person"
            return events

        matched = self._match(candidates, timestamp)
        self._debug_selected = _summary(matched) if matched is not None else None
        if matched is not None and self._usable(matched):
            self._reacquire(matched, timestamp, events)
            self._advance(matched, timestamp, events)
            return events

        if self._loss_since is None:
            self._loss_since = (self._last_seen_time if self._last_seen_time is not None
                                else timestamp)
            self._paused_state = self._state
            # Dwell needs consecutive observed poses; missing time must not
            # confirm an unseen calibration, peak, or return.
            self._rest_since = None
            self._peak_since = None
            self._return_since = None
            for side in ("left", "right"):
                self._lateral_peak_since[side] = None
            self._recent.clear()
            for recent in self._recent_by_side.values():
                recent.clear()
        if timestamp - self._loss_since >= self.config["lost_grace_s"]:
            self._cancel_partial(require_rearm=True)
            self._paused_state = None
        self._target_state = "missing"
        self._state = "paused"
        self._debug_reason = "target_unreadable" if matched else "target_missing"
        if (not self._loss_reported and
                timestamp - self._loss_since >= self.config["lost_grace_s"]):
            self._loss_reported = True
            events.append(self._emit("target_lost", timestamp))
        if timestamp - self._loss_since >= self.config["switch_after_s"]:
            # Keep the same person when its detector ID survives; any partial
            # curl was already cancelled before another arm can rearm.
            best = self._best([
                candidate for candidate in candidates
                if candidate.person_id != self.target_id or self._usable(candidate)
            ])
            if best is not None:
                if best.person_id == self.target_id:
                    self._reacquire(best, timestamp, events)
                else:
                    events.append(self._begin_session(best, timestamp, changed=True))
                self._advance(best, timestamp, events)
        return events

    def _candidates(self, payload: dict) -> list[_Candidate]:
        names = payload.get("keypoint_names") or COCO_NAMES
        persons = payload.get("persons") or []
        if not isinstance(names, (list, tuple)) or not isinstance(persons, list):
            return []
        indexes = {name: index for index, name in enumerate(names)
                   if isinstance(name, str)}
        image = payload.get("image_size") or (0, 0)
        width = _finite(image[0]) if isinstance(image, (list, tuple)) and len(image) > 1 else 0
        height = _finite(image[1]) if isinstance(image, (list, tuple)) and len(image) > 1 else 0
        result: list[_Candidate] = []
        for person in persons:
            if not isinstance(person, dict) or person.get("id") is None:
                continue
            raw_box, raw_points = person.get("bbox"), person.get("keypoints")
            if (not isinstance(raw_box, (list, tuple)) or len(raw_box) != 4
                    or not isinstance(raw_points, (list, tuple))):
                continue
            try:
                bbox = tuple(float(value) for value in raw_box)
            except (TypeError, ValueError):
                continue
            if (not all(math.isfinite(value) for value in bbox) or
                    bbox[2] <= bbox[0] or bbox[3] <= bbox[1]):
                continue
            points: dict[str, tuple[float, float, float]] = {}
            for side in ("left", "right"):
                for joint in ("shoulder", "elbow", "wrist", "hip"):
                    name = f"{side}_{joint}"
                    index = indexes.get(name)
                    if index is None or index >= len(raw_points):
                        continue
                    raw = raw_points[index]
                    if not isinstance(raw, (list, tuple)) or len(raw) < 3:
                        continue
                    try:
                        x, y, visibility = map(float, raw[:3])
                    except (TypeError, ValueError):
                        continue
                    if not all(math.isfinite(value) for value in (x, y, visibility)):
                        continue
                    inside = ((width <= 0 or 0 <= x < width) and
                              (height <= 0 or 0 <= y < height))
                    points[name] = (x, y, max(0.0, min(1.0, visibility))
                                    if inside else 0.0)
            side_quality = {}
            for side in ("left", "right"):
                confidence = [points.get(f"{side}_{joint}", (0, 0, 0))[2]
                              for joint in ("shoulder", "elbow", "wrist")]
                if min(confidence) >= self.config["min_visibility"]:
                    side_quality[side] = sum(confidence) / 3
            preferred = max(side_quality, key=side_quality.get) if side_quality else None
            readable = (bool(side_quality) if self.exercise == "dumbbell_curl"
                        else len(side_quality) == 2)
            if readable and self.exercise == "lateral_raise":
                left_shoulder = points["left_shoulder"]
                right_shoulder = points["right_shoulder"]
                scales = [_torso_scale(points, side, self.config["min_visibility"])
                          for side in ("left", "right")]
                valid_scales = [scale for scale in scales if scale is not None]
                torso = median(valid_scales) if len(valid_scales) == 2 else 0
                readable = (torso >= 1 and
                            abs(left_shoulder[0] - right_shoulder[0]) / torso >=
                            self.config["lateral_shoulder_span_min_ratio"])
            coverage = sum(1 for side in ("left", "right")
                           for joint in ("shoulder", "elbow", "wrist")
                           if points.get(f"{side}_{joint}", (0, 0, 0))[2] >=
                           self.config["min_visibility"]) / 6
            area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
            size = min(1.0, math.sqrt(area / max(width * height, 1.0)) * 2)
            cx, cy = _center(bbox)
            centrality = (max(0.0, 1 - math.hypot(
                (cx - width / 2) / (width / 2),
                (cy - height / 2) / (height / 2)) / math.sqrt(2))
                if width > 0 and height > 0 else 0.5)
            confidence = max(0.0, min(1.0, _finite(person.get("score"))))
            required_quality = (max(side_quality.values()) if side_quality else 0.0)
            if self.exercise == "lateral_raise" and len(side_quality) == 2:
                required_quality = min(side_quality.values())
            quality = (0.60 * required_quality + 0.15 * coverage + 0.15 * size
                       + 0.05 * centrality + 0.05 * confidence)
            result.append(_Candidate(person["id"], bbox, points, confidence,
                                     readable, quality, preferred))
        return result

    @staticmethod
    def _best(candidates: list[_Candidate]) -> _Candidate | None:
        return max((candidate for candidate in candidates if candidate.readable),
                   key=lambda candidate: candidate.quality, default=None)

    def _usable(self, candidate: _Candidate) -> bool:
        if not candidate.readable:
            return False
        if self.exercise == "dumbbell_curl":
            active_side = (self._state in ("moving", "peak", "returning") or
                           self._state == "paused" and
                           self._paused_state in ("moving", "peak", "returning")
                           and not self._needs_rearm)
            if active_side:
                return (self._side is not None and
                        self._measure_curl(candidate, self._side) is not None)
            return any(self._measure_curl(candidate, side) is not None
                       for side in ("left", "right"))
        return self._measure_lateral(candidate) is not None

    def _match(self, candidates: list[_Candidate], timestamp: float) -> _Candidate | None:
        old = self._target_box
        if old is None:
            return None
        old_center = _center(old)
        for candidate in candidates:
            if (candidate.person_id == self.target_id and
                    _distance(_center(candidate.bbox), old_center) <=
                    0.9 * max(1.0, old[3] - old[1])):
                return candidate
        # Only relink an expired detector ID when this is the sole person and
        # the box still overlaps closely. Never transfer a workout across two
        # visible people on spatial proximity alone.
        if (self._last_seen_time is None or
                timestamp - self._last_seen_time > self.config["lost_grace_s"] or
                len(candidates) != 1):
            return None
        candidate = candidates[0]
        if (_iou(candidate.bbox, old) >= 0.45 and
                _distance(_center(candidate.bbox), old_center) <=
                0.25 * max(1.0, old[3] - old[1])):
            return candidate
        return None

    def _emit(self, kind: str, timestamp: float, **extra: Any) -> dict:
        self.seq += 1
        return {"type": kind, "session_id": self.session_id,
                "seq": self.seq, "count_total": self.count_total,
                "event_time": timestamp, "target_id": self.target_id,
                "target_state": self._target_state, "state": self._state,
                **extra}

    def _begin_session(self, candidate: _Candidate, timestamp: float,
                       *, changed: bool) -> dict:
        previous = (self.session_id, self.target_id, self.count_total)
        if changed:
            self.session_id = uuid.uuid4().hex
        self.seq = 0
        self.count_total = 0
        self.target_id = candidate.person_id
        self._target_box = candidate.bbox
        self._side = candidate.preferred_side if self.exercise == "dumbbell_curl" else None
        self._state = "calibrating"
        self._target_state = "tracking"
        self._calibrated = False
        self._last_seen_time = timestamp
        self._loss_since = None
        self._loss_reported = False
        self._needs_rearm = False
        self._paused_state = None
        self._rest_since = None
        self._recent.clear()
        for recent in self._recent_by_side.values():
            recent.clear()
        self._rest_ready_sides.clear()
        self._debug_selected = _summary(candidate)
        self._cancel_partial()
        self._debug_reason = "target_changed" if changed else "target_locked"
        extra = ({"previous_session_id": previous[0],
                  "previous_target_id": previous[1],
                  "previous_count_total": previous[2]} if changed else {})
        return self._emit("target_changed" if changed else "session_started",
                          timestamp, **extra)

    def _reacquire(self, candidate: _Candidate, timestamp: float,
                   events: list[dict]) -> None:
        was_missing = self._loss_since is not None
        self.target_id = candidate.person_id
        self._target_box = candidate.bbox
        self._target_state = "tracking"
        if was_missing:
            short_loss = (timestamp - self._loss_since < self.config["lost_grace_s"]
                          and self._paused_state is not None and not self._needs_rearm)
            if short_loss:
                self._state = self._paused_state
            else:
                self._state = "rearming" if self._calibrated else "calibrating"
                self._needs_rearm = self._calibrated
                self._rest_since = None
                self._recent.clear()
                for recent in self._recent_by_side.values():
                    recent.clear()
                self._rest_ready_sides.clear()
            self._paused_state = None
        if self._loss_reported:
            events.append(self._emit("target_reacquired", timestamp))
        self._loss_since = None
        self._loss_reported = False

    def _measure_curl(self, candidate: _Candidate,
                      side: str) -> dict[str, float] | None:
        points = [candidate.points.get(f"{side}_{joint}")
                  for joint in ("shoulder", "elbow", "wrist")]
        if any(point is None or point[2] < self.config["min_visibility"]
               for point in points):
            return None
        shoulder, elbow, wrist = points
        elbow_angle = _angle(shoulder, elbow, wrist)
        upper_length = _distance(shoulder, elbow)
        forearm_length = _distance(elbow, wrist)
        torso_length = _torso_scale(candidate.points, side,
                                    self.config["min_visibility"])
        if (elbow_angle is None or min(upper_length, forearm_length,
                                     torso_length or 0) < 1):
            return None
        return {
            "elbow_angle_deg": elbow_angle,
            "upper_arm_vertical_ratio": (elbow[1] - shoulder[1]) / upper_length,
            "elbow_shift_ratio": abs(elbow[0] - shoulder[0]) / torso_length,
            "wrist_rise_ratio": (elbow[1] - wrist[1]) / forearm_length,
        }

    def _measure_lateral(self, candidate: _Candidate) -> dict[str, float] | None:
        result: dict[str, float] = {}
        shoulder_points: dict[str, tuple] = {}
        torso_lengths: list[float] = []
        arm_positions: dict[str, tuple[tuple, tuple, float]] = {}
        for side in ("left", "right"):
            points = [candidate.points.get(f"{side}_{joint}")
                      for joint in ("shoulder", "elbow", "wrist")]
            if any(point is None or point[2] < self.config["min_visibility"]
                   for point in points):
                return None
            shoulder, elbow, wrist = points
            elbow_angle = _angle(shoulder, elbow, wrist)
            arm_length = _distance(shoulder, elbow) + _distance(elbow, wrist)
            torso_length = _torso_scale(candidate.points, side,
                                        self.config["min_visibility"])
            if elbow_angle is None or min(arm_length, torso_length or 0) < 1:
                return None
            shoulder_points[side] = shoulder
            torso_lengths.append(torso_length)
            arm_positions[side] = (shoulder, wrist, arm_length)
            result[f"{side}_elbow_angle_deg"] = elbow_angle
            result[f"{side}_wrist_drop_ratio"] = (
                wrist[1] - shoulder[1]) / arm_length
        midpoint_x = (shoulder_points["left"][0] +
                      shoulder_points["right"][0]) / 2
        for side, (shoulder, wrist, arm_length) in arm_positions.items():
            direction = 1 if shoulder[0] > midpoint_x else -1
            result[f"{side}_outward_reach_ratio"] = (
                direction * (wrist[0] - shoulder[0]) / arm_length)
        result["shoulder_span_ratio"] = (
            abs(shoulder_points["left"][0] - shoulder_points["right"][0])
            / max(1.0, median(torso_lengths)))
        return result

    def _advance(self, candidate: _Candidate, timestamp: float,
                 events: list[dict]) -> None:
        self._last_seen_time = timestamp
        self._target_box = candidate.bbox
        self._debug_selected = _summary(candidate)
        if self.exercise == "dumbbell_curl":
            samples: dict[str, dict[str, float]] = {}
            active_rep = self._state in ("moving", "peak", "returning")
            for side in ("left", "right"):
                measured = self._measure_curl(candidate, side)
                recent = self._recent_by_side[side]
                if measured is None:
                    recent.clear()
                    if active_rep and side != self._side:
                        self._rest_ready_sides.discard(side)
                    continue
                recent.append(measured)
                samples[side] = {key: median(item[key] for item in recent)
                                 for key in measured}
                if self._checks(samples[side])["rest"]:
                    self._rest_ready_sides.add(side)
                elif active_rep and side != self._side:
                    # The other arm must still be at rest during this rep to
                    # be eligible for the next one. An old rest observation
                    # cannot turn a held curl into a second repetition.
                    self._rest_ready_sides.discard(side)
            if self._state == "ready":
                started = [side for side, value in samples.items()
                           if side in self._rest_ready_sides and
                           self._checks(value)["start"]]
                if started:
                    # A clearer idle arm is not necessarily the arm being
                    # curled. Choose from observed motion, then hold that arm
                    # until the repetition returns to rest.
                    self._side = min(started,
                                     key=lambda side: samples[side]["elbow_angle_deg"])
                    self._debug_sample = samples[self._side]
                    self._step(self._debug_sample, timestamp, events)
                else:
                    display_side = (self._side if self._side in samples else
                                    candidate.preferred_side)
                    self._debug_sample = samples.get(display_side)
                    self._debug_reason = "waiting_for_movement"
                return
            if self._state in ("calibrating", "rearming"):
                rest_sides = [side for side, value in samples.items()
                              if self._checks(value)["rest"]]
                if rest_sides:
                    self._side = (candidate.preferred_side
                                  if candidate.preferred_side in rest_sides else
                                  rest_sides[0])
                elif self._side not in samples:
                    self._side = (candidate.preferred_side
                                  if candidate.preferred_side in samples else
                                  next(iter(samples), None))
            sample = samples.get(self._side)
        else:
            sample = self._measure_lateral(candidate)
        if sample is None:
            self._cancel_partial(require_rearm=True)
            self._state = "rearming" if self._calibrated else "calibrating"
            self._debug_reason = "target_geometry_invalid"
            return
        if self.exercise == "lateral_raise":
            self._recent.append(sample)
            sample = {key: median(item[key] for item in self._recent)
                      for key in sample}
        self._debug_sample = sample
        self._step(sample, timestamp, events)

    def _track_lateral_peaks(self, checks: dict[str, bool], timestamp: float) -> bool:
        for side in ("left", "right"):
            raised = (checks["arms_extended"] and checks["front_view"] and
                      checks[f"{side}_raised"])
            if not raised:
                self._lateral_peak_since[side] = None
                continue
            if self._lateral_peak_since[side] is None:
                self._lateral_peak_since[side] = timestamp
            if timestamp - self._lateral_peak_since[side] >= self.config["peak_dwell_s"]:
                self._lateral_peak_confirmed[side] = timestamp
        left = self._lateral_peak_confirmed["left"]
        right = self._lateral_peak_confirmed["right"]
        return (left is not None and right is not None and
                abs(left - right) <= self.config["lateral_peak_window_s"] and
                timestamp - min(left, right) <= self.config["lateral_peak_window_s"])

    def _step(self, sample: dict[str, float], timestamp: float,
              events: list[dict]) -> None:
        checks = self._checks(sample)
        rest, start, peak = (checks["rest"], checks["start"], checks["peak"])
        if not self._calibrated or self._state == "rearming":
            if not rest:
                self._rest_since = None
                self._debug_reason = ("waiting_for_rest_to_rearm" if self._calibrated
                                      else "calibration_pose_invalid")
                return
            if self._rest_since is None:
                self._rest_since = timestamp
            if timestamp - self._rest_since < self.config["initial_dwell_s"]:
                self._debug_reason = "calibration_waiting_still"
                return
            first_ready = not self._calibrated
            self._calibrated = True
            self._needs_rearm = False
            self._state = "ready"
            self._rest_since = None
            self._debug_reason = "baseline_ready" if first_ready else "rearmed"
            events.append(self._emit("ready_to_exercise", timestamp,
                                     calibrated=True, rearmed=not first_ready))
            return
        if self._state == "ready":
            if start:
                self._clear_lateral_peaks()
                self._state = "moving"
                self._motion_started = timestamp
                if self.exercise == "lateral_raise":
                    self._track_lateral_peaks(checks, timestamp)
                    self._peak_since = None
                else:
                    self._peak_since = timestamp if peak else None
                self._debug_reason = "movement_started"
            else:
                self._debug_reason = "waiting_for_movement"
            return
        if (self._motion_started is not None and
                timestamp - self._motion_started > self.config["max_rep_s"]):
            self._cancel_partial(require_rearm=True)
            self._state = "rearming"
            self._debug_reason = "movement_timeout"
            return
        if self._state == "moving":
            peak_reached = (self._track_lateral_peaks(checks, timestamp)
                            if self.exercise == "lateral_raise" else peak)
            if rest:
                self._cancel_partial()
                self._state = "ready"
                self._debug_reason = "incomplete_returned_to_rest"
            elif peak_reached:
                if self.exercise == "lateral_raise":
                    self._state = "peak"
                    self._debug_reason = "peak_confirmed"
                    return
                if self._peak_since is None:
                    self._peak_since = timestamp
                if timestamp - self._peak_since >= self.config["peak_dwell_s"]:
                    self._state = "peak"
                    self._debug_reason = "peak_confirmed"
                else:
                    self._debug_reason = "waiting_for_peak_dwell"
            else:
                self._peak_since = None
                self._debug_reason = "waiting_for_peak"
            return
        if self._state == "peak":
            if not peak:
                self._state = "returning"
                self._return_since = timestamp if rest else None
                self._debug_reason = "return_started"
            else:
                self._debug_reason = "peak_held"
            return
        if self._state == "returning":
            if peak:
                self._state = "peak"
                self._return_since = None
                self._debug_reason = "returned_to_peak"
                return
            if not rest:
                self._return_since = None
                self._debug_reason = "waiting_for_full_return"
                return
            if self._return_since is None:
                self._return_since = timestamp
            if (timestamp - self._return_since >= self.config["return_dwell_s"]
                    and self._motion_started is not None and
                    timestamp - self._motion_started >= self.config["min_rep_s"]):
                duration = timestamp - self._motion_started
                self.count_total += 1
                self._state = "ready"
                self._cancel_partial()
                self._debug_reason = "rep_completed"
                events.append(self._emit("rep_completed", timestamp,
                                         rep_duration_s=duration))
            elif timestamp - self._return_since < self.config["return_dwell_s"]:
                self._debug_reason = "waiting_for_return_dwell"
            else:
                self._debug_reason = "rep_too_fast"

    def _cancel_partial(self, *, require_rearm: bool = False) -> None:
        self._motion_started = None
        self._peak_since = None
        self._return_since = None
        self._clear_lateral_peaks()
        if require_rearm:
            self._rest_ready_sides.clear()
            if self._calibrated:
                self._needs_rearm = True
                self._rest_since = None
                self._state = "rearming"

    def _clear_lateral_peaks(self) -> None:
        for side in ("left", "right"):
            self._lateral_peak_since[side] = None
            self._lateral_peak_confirmed[side] = None
