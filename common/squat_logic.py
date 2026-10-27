"""Local, camera-relative squat counting for pose ``/poses/skeleton`` frames.

This module has no ROS or model dependency. Its thresholds are prototype
starting values, not measurements validated on every robot camera. A caller must
feed frames in timestamp order and serialize calls to ``process``/``reset``.
"""

from __future__ import annotations

import math
import time
import uuid
from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Any


COCO_NAMES = (
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
)

# Explicit defaults make camera-footage calibration possible without changing
# the state machine. Angle values are degrees; ratios use standing leg length.
DEFAULT_CONFIG = {
    "min_visibility": 0.30,
    "baseline_dwell_s": 2.0,
    "baseline_knee_min_deg": 115.0,
    "baseline_hip_min_deg": 140.0,
    "baseline_angle_jitter_deg": 12.0,
    "baseline_position_jitter_ratio": 0.08,
    "reanchor_dwell_s": 0.8,
    "reanchor_knee_min_deg": 150.0,
    "reanchor_hip_min_deg": 150.0,
    "start_knee_drop_deg": 12.0,
    "start_hip_drop_deg": 8.0,
    "start_hip_drop_ratio": 0.025,
    "bottom_knee_drop_deg": 40.0,
    "bottom_hip_drop_deg": 25.0,
    "bottom_hip_drop_ratio": 0.10,
    "bottom_dwell_s": 0.15,
    "top_restore_deg": 12.0,
    "top_hip_restore_ratio": 0.08,
    "top_dwell_s": 0.20,
    "min_rep_s": 0.60,
    "ankle_motion_max_ratio": 0.25,
    "smoothing_frames": 3,
    "max_frame_gap_s": 0.50,
    "lost_grace_s": 0.50,
    "switch_after_s": 1.0,
}


@dataclass(frozen=True)
class _Sample:
    knee: float
    hip: float
    hip_y: float
    ankle_x: float
    ankle_y: float
    leg_scale: float


@dataclass(frozen=True)
class _Candidate:
    person_id: Any
    bbox: tuple[float, float, float, float]
    points: dict[str, tuple[float, float, float]]
    score: float
    readable: bool
    quality: float


def _finite(value: Any, fallback: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return fallback
    return number if math.isfinite(number) else fallback


def _angle(a: tuple[float, float], vertex: tuple[float, float],
           b: tuple[float, float]) -> float | None:
    ax, ay = a[0] - vertex[0], a[1] - vertex[1]
    bx, by = b[0] - vertex[0], b[1] - vertex[1]
    length = math.hypot(ax, ay) * math.hypot(bx, by)
    if length < 1e-6:
        return None
    cosine = min(1.0, max(-1.0, (ax * bx + ay * by) / length))
    return math.degrees(math.acos(cosine))


def _center(box: tuple[float, float, float, float]) -> tuple[float, float]:
    return ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)


def _box_height(box: tuple[float, float, float, float]) -> float:
    return max(1.0, box[3] - box[1])


def _box_iou(a: tuple[float, float, float, float],
             b: tuple[float, float, float, float]) -> float:
    width = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    height = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = width * height
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return intersection / max(area_a + area_b - intersection, 1.0)


def _safe_id(value: Any) -> str | int | float | bool | None:
    """Keep diagnostic IDs serializable even for direct, non-JSON callers."""
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, float) and math.isfinite(value):
        return value
    return str(value)


def _candidate_summary(candidate: _Candidate) -> dict:
    return {
        "id": _safe_id(candidate.person_id),
        "bbox": list(candidate.bbox),
        "score": candidate.score,
        "quality": candidate.quality,
        "readable": candidate.readable,
    }


class SquatCounter:
    """Count complete stand–squat–stand cycles for one locked person.

    ``process`` returns only sparse events. ``snapshot`` is the authoritative
    current total; it must be used to reconcile delayed decision-core events.
    A target switch starts a new session with total zero.
    """

    def __init__(self, config: dict | None = None):
        self.config = dict(DEFAULT_CONFIG)
        if config:
            unknown = set(config) - set(self.config)
            if unknown:
                raise ValueError(f"unknown squat configuration: {sorted(unknown)}")
            self.config.update(config)
        for key, value in self.config.items():
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be a nonnegative finite number")
        self.config["smoothing_frames"] = int(self.config["smoothing_frames"])
        if self.config["smoothing_frames"] < 1:
            raise ValueError("smoothing_frames must be at least 1")
        if self.config["switch_after_s"] < self.config["lost_grace_s"]:
            raise ValueError("switch_after_s must be >= lost_grace_s")
        self.reset()

    def reset(self) -> dict:
        """Clear the counter and target; the next readable person starts a session."""
        self.session_id = uuid.uuid4().hex
        self.seq = 0
        self.count_total = 0
        self.target_id = None
        self._target_box = None
        self._side = None
        self._baseline: _Sample | None = None
        self._baseline_samples: list[tuple[float, _Sample]] = []
        self._reanchor_samples: list[tuple[float, _Sample]] = []
        self._recent: deque[_Sample] = deque(maxlen=self.config["smoothing_frames"])
        self._state = "waiting_target"
        self._target_state = "none"
        self._last_frame_time = None
        self._last_seen_time = None
        self._loss_since = None
        self._loss_reported = False
        self._motion_started = None
        self._bottom_since = None
        self._top_since = None
        self._needs_rearm = False
        self._debug_sample: _Sample | None = None
        self._debug_selected_candidate: dict | None = None
        self._debug_candidates: list[dict] = []
        self._debug_reason = "waiting_for_readable_person"
        self._debug_frame_gap_s: float | None = None
        return self.snapshot()

    def snapshot(self) -> dict:
        """Return the latest count and tracking state without advancing time."""
        return {
            "session_id": self.session_id,
            "seq": self.seq,
            "count_total": self.count_total,
            "target_id": self.target_id,
            "target_state": self._target_state,
            "candidate_count": len(self._debug_candidates),
            "state": self._state,
            "side": self._side,
            "calibrated": self._baseline is not None,
            "last_seen_time": self._last_seen_time,
        }

    @property
    def last_frame_time(self) -> float | None:
        """Timestamp of the last pose frame accepted by the counter."""
        return self._last_frame_time

    def diagnostics(self) -> dict:
        """Return a JSON-safe, read-only view of the latest pose decision.

        Values describe the latest processed frame, not a new measurement.
        Missing measurements and checks are ``None``. Angles are degrees,
        pixel quantities use the pose image coordinates, and movement ratios
        are relative to the standing leg scale. These prototype thresholds
        need calibration on footage from the intended camera.
        """
        sample = self._debug_sample
        base = self._baseline
        cfg = self.config
        def sample_view(value: _Sample | None) -> dict | None:
            if value is None:
                return None
            return {
                "knee_angle_deg": value.knee,
                "hip_angle_deg": value.hip,
                "hip_y_px": value.hip_y,
                "ankle_x_px": value.ankle_x,
                "ankle_y_px": value.ankle_y,
                "leg_scale_px": value.leg_scale,
            }

        checks: dict[str, bool | None] = {
            key: None for key in (
                "baseline_pose", "start_knee_drop", "start_hip_drop",
                "start_hip_descent", "moving_down", "bottom_knee_drop",
                "bottom_hip_drop", "bottom_hip_descent", "bottom",
                "top_knee_restored", "top_hip_restored",
                "top_height_restored", "top", "ankle_stable",
            )
        }
        knee_drop = hip_drop = descent = ankle_motion = None
        if sample is not None:
            checks["baseline_pose"] = (
                sample.knee >= cfg["baseline_knee_min_deg"]
                and sample.hip >= cfg["baseline_hip_min_deg"]
                and sample.hip_y < sample.ankle_y
            )
        if sample is not None and base is not None:
            scale = max(1.0, base.leg_scale)
            knee_drop = base.knee - sample.knee
            hip_drop = base.hip - sample.hip
            descent = (sample.hip_y - base.hip_y) / scale
            ankle_motion = math.dist(
                (sample.ankle_x, sample.ankle_y),
                (base.ankle_x, base.ankle_y),
            ) / scale
            checks.update({
                "start_knee_drop": knee_drop >= cfg["start_knee_drop_deg"],
                "start_hip_drop": hip_drop >= cfg["start_hip_drop_deg"],
                "start_hip_descent": descent >= cfg["start_hip_drop_ratio"],
                "bottom_knee_drop": knee_drop >= cfg["bottom_knee_drop_deg"],
                "bottom_hip_drop": hip_drop >= cfg["bottom_hip_drop_deg"],
                "bottom_hip_descent": descent >= cfg["bottom_hip_drop_ratio"],
                "top_knee_restored": abs(knee_drop) <= cfg["top_restore_deg"],
                "top_hip_restored": abs(hip_drop) <= cfg["top_restore_deg"],
                "top_height_restored": abs(descent) <= cfg["top_hip_restore_ratio"],
                "ankle_stable": ankle_motion <= cfg["ankle_motion_max_ratio"],
            })
            checks["moving_down"] = sum((
                checks["start_knee_drop"], checks["start_hip_drop"],
                checks["start_hip_descent"],
            )) >= 2
            checks["bottom"] = all((
                checks["bottom_knee_drop"], checks["bottom_hip_drop"],
                checks["bottom_hip_descent"],
            ))
            checks["top"] = all((
                checks["top_knee_restored"], checks["top_hip_restored"],
                checks["top_height_restored"],
            ))

        selected = self._debug_selected_candidate
        selected_target = {
            "id": _safe_id(self.target_id),
            "bbox": (list(selected["bbox"]) if selected is not None else
                     list(self._target_box) if self._target_box is not None else None),
            "quality": selected["quality"] if selected is not None else None,
            "readable": selected["readable"] if selected is not None else None,
            "side": self._side,
            "present": selected is not None,
        }
        now = self._last_frame_time
        def elapsed(start: float | None) -> float | None:
            return max(0.0, now - start) if now is not None and start is not None else None

        return {
            "frame_time": now,
            "frame_gap_s": self._debug_frame_gap_s,
            "frame_gap_exceeded": (self._debug_frame_gap_s > cfg["max_frame_gap_s"]
                                   if self._debug_frame_gap_s is not None else None),
            "state": self._state,
            "target_state": self._target_state,
            "needs_rearm": self._needs_rearm,
            "selected_target": selected_target,
            "candidate_count": len(self._debug_candidates),
            "candidates": [{**candidate, "bbox": list(candidate["bbox"])}
                           for candidate in sorted(
                               self._debug_candidates,
                               key=lambda c: c["quality"], reverse=True)[:8]],
            "measurements": {
                **(sample_view(sample) or {
                    "knee_angle_deg": None, "hip_angle_deg": None,
                    "hip_y_px": None, "ankle_x_px": None,
                    "ankle_y_px": None, "leg_scale_px": None,
                }),
                "baseline": sample_view(base),
                "baseline_knee_angle_deg": base.knee if base is not None else None,
                "baseline_hip_angle_deg": base.hip if base is not None else None,
                "baseline_hip_y_px": base.hip_y if base is not None else None,
                "baseline_leg_scale_px": base.leg_scale if base is not None else None,
                "knee_drop_deg": knee_drop,
                "hip_drop_deg": hip_drop,
                "hip_descent_ratio": descent,
                "ankle_motion_ratio": ankle_motion,
            },
            "checks": checks,
            "timing": {
                "calibration_elapsed_s": elapsed(self._baseline_samples[0][0])
                if self._baseline_samples else None,
                "calibration_samples": len(self._baseline_samples),
                "reanchor_elapsed_s": elapsed(self._reanchor_samples[0][0])
                if self._reanchor_samples else None,
                "reanchor_samples": len(self._reanchor_samples),
                "motion_elapsed_s": elapsed(self._motion_started),
                "bottom_dwell_elapsed_s": elapsed(self._bottom_since),
                "top_dwell_elapsed_s": elapsed(self._top_since),
            },
            "thresholds": dict(cfg),
            "reason": self._debug_reason,
        }

    def process(self, payload: dict) -> list[dict]:
        """Consume one skeleton frame and return completed semantic events."""
        if not isinstance(payload, dict):
            return []
        timestamp = _finite(payload.get("timestamp"), time.time())
        if self._last_frame_time is not None and timestamp <= self._last_frame_time:
            self._debug_reason = "non_monotonic_timestamp"
            return []
        self._debug_sample = None
        self._debug_selected_candidate = None
        self._debug_reason = "waiting_for_readable_person"
        gap = (timestamp - self._last_frame_time
               if self._last_frame_time is not None else 0.0)
        self._debug_frame_gap_s = gap if self._last_frame_time is not None else None
        self._last_frame_time = timestamp
        if gap > self.config["max_frame_gap_s"]:
            self._cancel_partial(require_stand=True)
            self._recent.clear()
            self._baseline_samples.clear()
            self._reanchor_samples.clear()

        candidates = self._candidates(payload)
        self._debug_candidates = [_candidate_summary(candidate) for candidate in
                                  candidates]
        events: list[dict] = []
        if self.target_id is None:
            best = self._best_readable(candidates)
            if best is not None:
                events.append(self._begin_session(best, timestamp, changed=False))
                self._advance_person(best, timestamp, events)
            return events

        matched = self._match_target(candidates, timestamp)
        self._debug_selected_candidate = (_candidate_summary(matched)
                                          if matched is not None else None)
        if matched is not None and matched.readable:
            self._reacquire(matched, timestamp, events)
            self._advance_person(matched, timestamp, events)
            return events

        if self._loss_since is None:
            self._loss_since = (self._last_seen_time if self._last_seen_time is not None
                                else timestamp)
        self._cancel_partial(require_stand=True)
        self._recent.clear()
        self._baseline_samples.clear()
        self._reanchor_samples.clear()
        self._target_state = "missing"
        self._state = "paused"
        self._debug_reason = ("target_unreadable" if matched is not None
                              else "target_missing")
        if (not self._loss_reported
                and timestamp - self._loss_since >= self.config["lost_grace_s"]):
            self._loss_reported = True
            events.append(self._emit("target_lost", timestamp))
        if timestamp - self._loss_since >= self.config["switch_after_s"]:
            best = self._best_readable(candidates)
            if best is not None:
                if best.person_id == self.target_id:
                    # A subject can move beyond the old box while missing.
                    # The same tracked ID must not start a new count session.
                    self._reacquire(best, timestamp, events)
                else:
                    events.append(self._begin_session(best, timestamp, changed=True))
                self._advance_person(best, timestamp, events)
        return events

    def _reacquire(self, candidate: _Candidate, timestamp: float,
                   events: list[dict]) -> None:
        if self._loss_since is not None:
            if (self._baseline is not None and self._target_box is not None
                    and math.dist(_center(candidate.bbox),
                                  _center(self._target_box)) >
                    self.config["ankle_motion_max_ratio"] * self._baseline.leg_scale):
                # The old ankle position cannot validate a subject who moved
                # while out of view. Relearn standing pose but retain the count.
                self._baseline = None
                self._baseline_samples.clear()
                self._reanchor_samples.clear()
                self._recent.clear()
                self._side = None
                self._needs_rearm = False
            self._state = ("calibrating" if self._baseline is None else
                           "rearming" if self._needs_rearm else "top")
        self.target_id = candidate.person_id
        self._target_box = candidate.bbox
        self._target_state = "tracking"
        if self._loss_reported:
            events.append(self._emit("target_reacquired", timestamp))
        self._loss_since = None
        self._loss_reported = False

    def _emit(self, kind: str, timestamp: float, **extra: Any) -> dict:
        self.seq += 1
        return {
            "type": kind,
            "session_id": self.session_id,
            "seq": self.seq,
            "count_total": self.count_total,
            "event_time": timestamp,
            "target_id": self.target_id,
            "target_state": self._target_state,
            "state": self._state,
            **extra,
        }

    def _begin_session(self, candidate: _Candidate, timestamp: float,
                       *, changed: bool) -> dict:
        previous_session = self.session_id if changed else None
        previous_target = self.target_id if changed else None
        previous_count = self.count_total if changed else None
        if changed:
            self.session_id = uuid.uuid4().hex
        self.seq = 0
        self.count_total = 0
        self.target_id = candidate.person_id
        self._target_box = candidate.bbox
        self._side = None
        self._baseline = None
        self._baseline_samples.clear()
        self._reanchor_samples.clear()
        self._recent.clear()
        self._state = "calibrating"
        self._target_state = "tracking"
        self._last_seen_time = timestamp
        self._loss_since = None
        self._loss_reported = False
        self._needs_rearm = False
        self._debug_selected_candidate = _candidate_summary(candidate)
        self._debug_reason = "target_changed" if changed else "target_locked"
        self._cancel_partial()
        return self._emit(
            "target_changed" if changed else "session_started", timestamp,
            **({"previous_session_id": previous_session,
                "previous_target_id": previous_target,
                "previous_count_total": previous_count} if changed else {}),
        )

    def _candidates(self, payload: dict) -> list[_Candidate]:
        names = payload.get("keypoint_names") or COCO_NAMES
        if not isinstance(names, (list, tuple)):
            return []
        indexes = {name: index for index, name in enumerate(names)
                   if isinstance(name, str)}
        image = payload.get("image_size") or (0, 0)
        width = _finite(image[0]) if isinstance(image, (list, tuple)) and len(image) > 1 else 0
        height = _finite(image[1]) if isinstance(image, (list, tuple)) and len(image) > 1 else 0
        persons = payload.get("persons") or []
        if not isinstance(persons, list):
            return []
        out: list[_Candidate] = []
        for person in persons:
            if not isinstance(person, dict) or person.get("id") is None:
                continue
            raw_box = person.get("bbox")
            raw_points = person.get("keypoints")
            if not isinstance(raw_box, (list, tuple)) or len(raw_box) != 4:
                continue
            if not isinstance(raw_points, (list, tuple)):
                continue
            try:
                bbox = tuple(float(v) for v in raw_box)
            except (TypeError, ValueError):
                continue
            if (not all(math.isfinite(v) for v in bbox)
                    or bbox[2] <= bbox[0] or bbox[3] <= bbox[1]):
                continue
            points: dict[str, tuple[float, float, float]] = {}
            for side in ("left", "right"):
                for joint in ("shoulder", "hip", "knee", "ankle"):
                    name = f"{side}_{joint}"
                    index = indexes.get(name)
                    if index is None or index >= len(raw_points):
                        continue
                    raw = raw_points[index]
                    if not isinstance(raw, (list, tuple)) or len(raw) < 3:
                        continue
                    try:
                        x, y, visibility = (float(raw[0]), float(raw[1]), float(raw[2]))
                    except (TypeError, ValueError):
                        continue
                    if all(math.isfinite(v) for v in (x, y, visibility)):
                        # The detector may extrapolate a high-confidence joint
                        # outside a cropped frame. It cannot support counting.
                        inside = (width <= 0 or 0 <= x < width) and (
                            height <= 0 or 0 <= y < height)
                        points[name] = (
                            x, y, max(0.0, min(1.0, visibility)) if inside else 0.0)
            side_quality = []
            for side in ("left", "right"):
                values = [points.get(f"{side}_{joint}", (0, 0, 0))[2]
                          for joint in ("shoulder", "hip", "knee", "ankle")]
                if min(values) >= self.config["min_visibility"]:
                    side_quality.append(sum(values) / 4)
            readable = bool(side_quality)
            coverage = sum(1 for point in points.values()
                           if point[2] >= self.config["min_visibility"]) / 8
            box_area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
            size = min(1.0, math.sqrt(box_area / max(width * height, 1.0)) * 2)
            cx, cy = _center(bbox)
            if width > 0 and height > 0:
                distance = math.hypot((cx - width / 2) / (width / 2),
                                      (cy - height / 2) / (height / 2))
                centrality = max(0.0, 1.0 - distance / math.sqrt(2))
            else:
                centrality = 0.5
            confidence = max(0.0, min(1.0, _finite(person.get("score"))))
            quality = (0.60 * (max(side_quality) if side_quality else 0.0)
                       + 0.15 * coverage + 0.15 * size
                       + 0.05 * centrality + 0.05 * confidence)
            out.append(_Candidate(person["id"], bbox, points, confidence,
                                  readable, quality))
        return out

    @staticmethod
    def _best_readable(candidates: list[_Candidate]) -> _Candidate | None:
        return max((c for c in candidates if c.readable),
                   key=lambda c: c.quality, default=None)

    def _match_target(self, candidates: list[_Candidate],
                      timestamp: float) -> _Candidate | None:
        old_box = self._target_box
        if old_box is None:
            return None
        old_center = _center(old_box)
        for candidate in candidates:
            if candidate.person_id != self.target_id:
                continue
            distance = math.dist(_center(candidate.bbox), old_center)
            if distance <= 0.9 * _box_height(old_box):
                return candidate
        # Pose's track id can expire after a short occlusion. A tight spatial
        # match lets the same subject retain their count without following a
        # different person across the scene.
        if (self._last_seen_time is None
                or timestamp - self._last_seen_time > self.config["lost_grace_s"]):
            return None
        if len(candidates) != 1:
            return None
        matches = []
        for candidate in candidates:
            distance = math.dist(_center(candidate.bbox), old_center)
            if (_box_iou(candidate.bbox, old_box) >= 0.45
                    and distance <= 0.25 * _box_height(old_box)):
                matches.append(candidate)
        return max(matches, key=lambda c: _box_iou(c.bbox, old_box), default=None)

    def _measure(self, candidate: _Candidate, side: str) -> _Sample | None:
        points = [candidate.points.get(f"{side}_{joint}")
                  for joint in ("shoulder", "hip", "knee", "ankle")]
        if any(point is None or point[2] < self.config["min_visibility"]
               for point in points):
            return None
        shoulder, hip, knee, ankle = points
        knee_angle = _angle(hip, knee, ankle)
        hip_angle = _angle(shoulder, hip, knee)
        if knee_angle is None or hip_angle is None:
            return None
        return _Sample(knee_angle, hip_angle, hip[1], ankle[0], ankle[1],
                       max(1.0, math.dist(hip[:2], ankle[:2])))

    def _advance_person(self, candidate: _Candidate, timestamp: float,
                        events: list[dict]) -> None:
        self._last_seen_time = timestamp
        self._target_box = candidate.bbox
        self._debug_selected_candidate = _candidate_summary(candidate)
        if self._side is None:
            sides = [(self._measure(candidate, side), side)
                     for side in ("left", "right")]
            available = [(sample, side) for sample, side in sides if sample]
            if not available:
                self._debug_reason = "target_geometry_invalid"
                return
            self._side = max(available, key=lambda item: min(
                candidate.points[f"{item[1]}_{joint}"][2]
                for joint in ("shoulder", "hip", "knee", "ankle")))[1]
        sample = self._measure(candidate, self._side)
        if sample is None:
            other = "right" if self._side == "left" else "left"
            sample = self._measure(candidate, other)
            if sample is None:
                self._cancel_partial(require_stand=True)
                self._recent.clear()
                self._baseline_samples.clear()
                self._reanchor_samples.clear()
                self._state = "rearming" if self._baseline is not None else "calibrating"
                self._debug_reason = "target_joint_unreadable"
                return
            # Do not compare one leg against the other leg's baseline.
            self._side = other
            self._baseline = None
            self._baseline_samples.clear()
            self._reanchor_samples.clear()
            self._recent.clear()
            self._cancel_partial()
            self._state = "calibrating"
        self._recent.append(sample)
        smooth = _Sample(*(median(getattr(s, field) for s in self._recent)
                           for field in _Sample.__dataclass_fields__))
        self._debug_sample = smooth
        if self._baseline is None:
            self._calibrate(smooth, timestamp, events)
            return
        self._step(smooth, timestamp, events)

    def _calibrate(self, sample: _Sample, timestamp: float,
                   events: list[dict]) -> None:
        self._state = "calibrating"
        if (sample.knee < self.config["baseline_knee_min_deg"]
                or sample.hip < self.config["baseline_hip_min_deg"]
                or sample.hip_y >= sample.ankle_y):
            self._baseline_samples.clear()
            self._debug_reason = "calibration_pose_invalid"
            return
        self._baseline_samples.append((timestamp, sample))
        if len(self._baseline_samples) < 3:
            self._debug_reason = "calibration_collecting"
            return
        if timestamp - self._baseline_samples[0][0] < self.config["baseline_dwell_s"]:
            self._debug_reason = "calibration_waiting_still"
            return
        samples = [s for _, s in self._baseline_samples]
        scale = max(1.0, median(s.leg_scale for s in samples))
        if (max(s.knee for s in samples) - min(s.knee for s in samples)
                > self.config["baseline_angle_jitter_deg"]):
            self._baseline_samples[:] = self._baseline_samples[-1:]
            self._debug_reason = "calibration_knee_unstable"
            return
        if (max(s.hip for s in samples) - min(s.hip for s in samples)
                > self.config["baseline_angle_jitter_deg"]):
            self._baseline_samples[:] = self._baseline_samples[-1:]
            self._debug_reason = "calibration_hip_unstable"
            return
        allowed = self.config["baseline_position_jitter_ratio"] * scale
        if max(s.hip_y for s in samples) - min(s.hip_y for s in samples) > allowed:
            self._baseline_samples[:] = self._baseline_samples[-1:]
            self._debug_reason = "calibration_hip_position_unstable"
            return
        if (math.dist((max(s.ankle_x for s in samples), max(s.ankle_y for s in samples)),
                      (min(s.ankle_x for s in samples), min(s.ankle_y for s in samples)))
                > allowed):
            self._baseline_samples[:] = self._baseline_samples[-1:]
            self._debug_reason = "calibration_ankle_position_unstable"
            return
        self._baseline = _Sample(*(median(getattr(s, field) for s in samples)
                                   for field in _Sample.__dataclass_fields__))
        self._baseline_samples.clear()
        self._needs_rearm = False
        self._state = "top"
        self._debug_reason = "baseline_ready"
        events.append(self._emit("ready_to_squat", timestamp, calibrated=True))

    def _cancel_partial(self, *, require_stand: bool = False) -> None:
        self._motion_started = None
        self._bottom_since = None
        self._top_since = None
        if require_stand and self._baseline is not None:
            self._needs_rearm = True
            self._state = "rearming"
            return
        if self._baseline is not None and self._state in ("descent", "bottom", "ascent"):
            self._state = "top"

    def _try_reanchor(self, sample: _Sample, timestamp: float,
                      events: list[dict] | None) -> bool:
        """Recover a stale position baseline from a stable, upright subject.

        This only runs outside a valid repetition or after a moved ankle has
        canceled one. The existing total and session are kept.
        """
        cfg = self.config
        if (sample.knee < cfg["reanchor_knee_min_deg"]
                or sample.hip < cfg["reanchor_hip_min_deg"]
                or sample.hip_y >= sample.ankle_y):
            self._reanchor_samples.clear()
            return False
        self._reanchor_samples.append((timestamp, sample))
        samples = [item for _, item in self._reanchor_samples]
        scale = max(1.0, median(item.leg_scale for item in samples))
        position_limit = cfg["baseline_position_jitter_ratio"] * scale
        if (max(item.knee for item in samples) - min(item.knee for item in samples)
                > cfg["baseline_angle_jitter_deg"]
                or max(item.hip for item in samples) - min(item.hip for item in samples)
                > cfg["baseline_angle_jitter_deg"]
                or max(item.hip_y for item in samples) - min(item.hip_y for item in samples)
                > position_limit
                or math.dist(
                    (max(item.ankle_x for item in samples),
                     max(item.ankle_y for item in samples)),
                    (min(item.ankle_x for item in samples),
                     min(item.ankle_y for item in samples)),
                ) > position_limit):
            self._reanchor_samples[:] = self._reanchor_samples[-1:]
            self._debug_reason = "reanchor_waiting_still"
            return False
        if (len(samples) < 3 or
                timestamp - self._reanchor_samples[0][0] < cfg["reanchor_dwell_s"]):
            self._debug_reason = "reanchor_collecting"
            return False
        self._baseline = _Sample(*(median(getattr(item, field) for item in samples)
                                   for field in _Sample.__dataclass_fields__))
        self._reanchor_samples.clear()
        self._recent.clear()
        self._recent.append(sample)
        self._cancel_partial()
        self._needs_rearm = False
        self._state = "top"
        self._debug_reason = "baseline_reanchored"
        if events is not None:
            events.append(self._emit("ready_to_squat", timestamp,
                                     calibrated=True, reanchored=True))
        return True

    def _step(self, sample: _Sample, timestamp: float,
              events: list[dict] | None) -> None:
        base = self._baseline
        scale = max(1.0, base.leg_scale)
        ankle_motion = math.dist((sample.ankle_x, sample.ankle_y),
                                 (base.ankle_x, base.ankle_y)) / scale
        if ankle_motion > self.config["ankle_motion_max_ratio"]:
            self._cancel_partial(require_stand=True)
            self._debug_reason = "ankle_motion_exceeded"
            self._try_reanchor(sample, timestamp, events)
            return
        knee_drop = base.knee - sample.knee
        hip_drop = base.hip - sample.hip
        descent = (sample.hip_y - base.hip_y) / scale
        top = (abs(knee_drop) <= self.config["top_restore_deg"]
               and abs(hip_drop) <= self.config["top_restore_deg"]
               and abs(descent) <= self.config["top_hip_restore_ratio"])
        bottom = (knee_drop >= self.config["bottom_knee_drop_deg"]
                  and hip_drop >= self.config["bottom_hip_drop_deg"]
                  and descent >= self.config["bottom_hip_drop_ratio"])
        moving_down = sum((knee_drop >= self.config["start_knee_drop_deg"],
                           hip_drop >= self.config["start_hip_drop_deg"],
                           descent >= self.config["start_hip_drop_ratio"])) >= 2

        if self._state == "rearming":
            if not top:
                self._top_since = None
                self._debug_reason = "waiting_for_top_to_rearm"
                self._try_reanchor(sample, timestamp, events)
                return
            self._reanchor_samples.clear()
            if self._top_since is None:
                self._top_since = timestamp
            if timestamp - self._top_since >= self.config["top_dwell_s"]:
                self._needs_rearm = False
                self._top_since = None
                self._state = "top"
                self._debug_reason = "rearmed"
            else:
                self._debug_reason = "waiting_for_top_dwell_to_rearm"
            return
        if self._state == "top":
            if moving_down:
                self._reanchor_samples.clear()
                self._state = "descent"
                self._motion_started = timestamp
                self._bottom_since = timestamp if bottom else None
                self._debug_reason = "descent_started"
            else:
                self._debug_reason = "waiting_for_descent"
                if not top:
                    self._try_reanchor(sample, timestamp, events)
                else:
                    self._reanchor_samples.clear()
            return
        if self._state == "descent":
            if top:
                self._cancel_partial()
                self._debug_reason = "incomplete_squat_returned_top"
            elif bottom:
                if self._bottom_since is None:
                    self._bottom_since = timestamp
                if timestamp - self._bottom_since >= self.config["bottom_dwell_s"]:
                    self._state = "bottom"
                    self._debug_reason = "bottom_confirmed"
                else:
                    self._debug_reason = "waiting_for_bottom_dwell"
            else:
                self._bottom_since = None
                self._debug_reason = "waiting_for_bottom"
            return
        if self._state == "bottom":
            if not bottom:
                self._state = "ascent"
                self._top_since = timestamp if top else None
                self._debug_reason = "ascent_started"
            else:
                self._debug_reason = "bottom_held"
            return
        if self._state == "ascent":
            if bottom:
                self._state = "bottom"
                self._top_since = None
                self._debug_reason = "returned_to_bottom"
                return
            if not top:
                self._top_since = None
                self._debug_reason = "waiting_for_full_stand"
                return
            if self._top_since is None:
                self._top_since = timestamp
            if (timestamp - self._top_since >= self.config["top_dwell_s"]
                    and self._motion_started is not None
                    and timestamp - self._motion_started >= self.config["min_rep_s"]):
                rep_duration_s = timestamp - self._motion_started
                self.count_total += 1
                self._state = "top"
                self._cancel_partial()
                self._reanchor_samples.clear()
                if (sample.knee >= self.config["reanchor_knee_min_deg"]
                        and sample.hip >= self.config["reanchor_hip_min_deg"]):
                    # Follow small changes in the verified top pose. Freeze
                    # the baseline throughout the down/up part of each rep.
                    self._baseline = _Sample(*(
                        0.8 * getattr(base, field) + 0.2 * getattr(sample, field)
                        for field in _Sample.__dataclass_fields__))
                self._debug_reason = "rep_completed"
                if events is not None:
                    events.append(self._emit("rep_completed", timestamp,
                                             rep_duration_s=rep_duration_s))
            elif timestamp - self._top_since < self.config["top_dwell_s"]:
                self._debug_reason = "waiting_for_top_dwell"
            else:
                self._debug_reason = "rep_too_fast"
