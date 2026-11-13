"""Bounded source-frame matching and per-housing temporal evidence, without ROS."""
from collections import OrderedDict, deque
from copy import deepcopy
from dataclasses import dataclass, field
import math
import time

import numpy as np

from .color_classifier import classify_color
from .schemas import Config, bbox_of, stamp_ns


def overlap(a, b):
    area = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
    total = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - area
    return area / total if total else 0.


def jpeg_size(data):
    """Read JPEG SOF before decoding, bounding even highly compressed images."""
    if data[:2] != b"\xff\xd8":
        return None
    cursor = 2
    while cursor+4 <= len(data):
        if data[cursor] != 255:
            return None
        while cursor < len(data) and data[cursor] == 255:
            cursor += 1
        if cursor >= len(data):
            return None
        marker = data[cursor]
        cursor += 1
        if marker in (0xDA, 0xD9):
            return None
        if cursor+2 > len(data):
            return None
        length = int.from_bytes(data[cursor:cursor+2], "big")
        if length < 2 or cursor+length > len(data):
            return None
        if marker in (0xC0, 0xC1, 0xC2):
            if length < 8:
                return None
            height = int.from_bytes(data[cursor+3:cursor+5], "big")
            width = int.from_bytes(data[cursor+5:cursor+7], "big")
            return (width, height) if 0 < width <= 3840 and 0 < height <= 2160 else None
        cursor += length
    return None


@dataclass
class Track:
    signal_id: str
    bbox: list
    last_seen: float
    history: deque = field(default_factory=lambda: deque(maxlen=5))


class TrafficLightProcessor:
    def __init__(self, config=None, *, clock=time.monotonic, ros_clock=time.time_ns,
                 wall_clock=time.time, not_before_ns=0):
        self.config = Config.from_dict(config)
        self.clock, self.ros_clock, self.wall_clock = clock, ros_clock, wall_clock
        self.not_before_ns = not_before_ns
        self.frames = OrderedDict()
        self.pending = OrderedDict()
        self.tracks = {}
        self._next_id = 0
        self._stream = None
        self._last_stamp = 0
        self._seen = deque(maxlen=128)
        self._retired_streams = deque(maxlen=16)
        self._ambiguous_stamps = deque(maxlen=128)
        self.last_image_at = self.last_detection_at = None
        self.last_image_stamp = self.last_detection_stamp = None
        self.last_sync_at = None
        self.last_error = None
        self._latest_at = None
        self._latest = self._result("waiting", reason="waiting_for_source_frame")

    def _result(self, state, *, signals=None, stamp=None, reason=None):
        return {"state": state, "timestamp": self.wall_clock(),
                "frame_timestamp": stamp / 10**9 if stamp else None,
                "frame_stamp": ({"sec": stamp//10**9, "nanosec": stamp % 10**9} if stamp else None),
                "frame_timestamp_semantics": "ros_header_stamp" if stamp else "unavailable",
                "signals": signals or [], "reason": reason}

    def _prune(self, now):
        for stamp, (_, received) in list(self.frames.items()):
            if now-received > self.config.max_age_s:
                del self.frames[stamp]
        for identity, (_, received) in list(self.pending.items()):
            if now-received > self.config.max_age_s:
                del self.pending[identity]
        for key, track in list(self.tracks.items()):
            if now-track.last_seen >= self.config.track_ttl_s:
                del self.tracks[key]

    def image(self, data, stamp):
        now, source = self.clock(), stamp_ns(stamp)
        self._prune(now)
        if (source is None or source < self.not_before_ns or not data
                or len(data) > self.config.max_image_bytes):
            return []
        age = (self.ros_clock()-source)/10**9
        if not 0 <= age <= self.config.max_age_s:
            return []
        self.last_image_at, self.last_image_stamp = now, source
        # Same stamped image is one exposure; never refresh its receive age.
        if source in self.frames and self.frames[source][0] != bytes(data):
            # Conflicting JPEGs with an identical ROS stamp have no reliable join.
            self.frames.pop(source)
            self._ambiguous_stamps.append(source)
            if stamp_ns(self._latest.get("frame_stamp")) == source:
                for track in self.tracks.values():
                    track.history.clear()
                self.last_sync_at = None
                self._latest_at = now
                self._latest = self._result("waiting", reason="ambiguous_source_stamp")
                return [deepcopy(self._latest)]
            return []
        if source in self._ambiguous_stamps:
            return []
        if source not in self.frames:
            self.frames[source] = (bytes(data), now)
        while (len(self.frames) > self.config.cache_size
               or sum(len(frame[0]) for frame in self.frames.values()) > self.config.max_cache_bytes):
            self.frames.popitem(last=False)
        results = []
        for identity, (payload, received) in list(self.pending.items()):
            if stamp_ns(payload.get("source_stamp")) == source:
                del self.pending[identity]
                results.append(self._process(payload, received))
        return results

    @staticmethod
    def _identity(payload):
        return (payload.get("stream_id"), stamp_ns(payload.get("source_stamp")))

    def detection(self, payload):
        if not isinstance(payload, dict) or not isinstance(payload.get("objects"), list):
            return self.error("invalid_vop_json")
        if len(payload["objects"]) > 256:
            return self.error("too_many_detections")
        return self._process(payload, self.clock())

    def error(self, reason):
        self.last_error = reason
        self._latest_at = self.clock()
        self._latest = self._result("error", reason=reason)
        return deepcopy(self._latest)

    def _unknown_signals(self, objects, reason):
        return [{"signal_id": None, "bbox": bbox_of(obj.get("bbox")),
                 "position": self._position(obj), "detection_confidence": self._confidence(obj),
                 "color": "unknown", "color_confidence": 0., "symbol": "unknown",
                 "symbol_confidence": 0., "stability": "unstable", "observations": 0,
                 "color_stability": "unstable", "symbol_stability": "unstable",
                 "bulbs": [], "reason": reason} for obj in objects]

    @staticmethod
    def _position(obj):
        value = obj.get("position")
        if (isinstance(value, list) and len(value) == 2
                and all(type(v) in (int, float) and math.isfinite(v) for v in value)):
            return {"x": value[0], "y": value[1]}
        return None

    @staticmethod
    def _confidence(obj):
        value = obj.get("confidence")
        return value if type(value) in (int, float) and math.isfinite(value) and 0 <= value <= 1 else None

    def _associate(self, detections):
        """Inherit evidence only on isolated one-detection/one-track edges.

        Build the entire candidate graph against the previous frame before
        allocating anything. Removing a claimant during greedy assignment can
        turn a duplicate's second choice into a false unique match.
        """
        candidates = {}
        claimants = {key: [] for key in self.tracks}
        for index, (box, confidence) in enumerate(detections):
            if box is None or confidence is None or confidence < self.config.min_detection_confidence:
                continue
            candidates[index] = [key for key, track in self.tracks.items()
                                 if overlap(box, track.bbox) >= .3]
            for key in candidates[index]:
                claimants[key].append(index)
        matches, affected = {}, set()
        for index, keys in candidates.items():
            if len(keys) == 1 and len(claimants[keys[0]]) == 1:
                matches[index] = keys[0]
            else:
                # Every non-isolated component is ambiguous. Retire all its
                # old endpoints so the next frame cannot revive stale votes.
                affected.update(keys)
        for key in affected:
            del self.tracks[key]
        return matches

    def _process(self, payload, received):
        now = self.clock()
        self._prune(now)
        objects = [obj for obj in payload["objects"] if isinstance(obj, dict)
                   and obj.get("name") == "traffic light"][:self.config.max_tracks]
        source = stamp_ns(payload.get("source_stamp"))
        stream = payload.get("stream_id")
        if stream is not None and not isinstance(stream, str):
            return self.error("invalid_stream_id")
        identity = self._identity(payload)
        if identity in self._seen:
            return self.latest()
        self.last_detection_at, self.last_detection_stamp = now, source
        problem = None
        if stream in self._retired_streams:
            problem = "retired_stream"
        elif source in self._ambiguous_stamps:
            problem = "ambiguous_source_stamp"
        elif (source is None or payload.get("source_mode", "stream") != "stream"
                or payload.get("temporal_eligible") is False
                or payload.get("source_clock_status") in ("regressed", "zero", "unavailable")):
            problem = "source_provenance_unavailable"
        elif source < self.not_before_ns:
            problem = "before_observation_start"
        elif not 0 <= (self.ros_clock()-source)/10**9 <= self.config.max_age_s:
            problem = "source_frame_stale_or_future"
        elif now-received > self.config.max_age_s:
            problem = "detection_expired"
        elif stream == self._stream and source <= self._last_stamp:
            problem = "source_stamp_regressed"
        frame = self.frames.get(source)
        if not problem and frame is None:
            problem = "matching_image_unavailable"
            self.pending[identity] = (deepcopy(payload), received)
            while len(self.pending) > 8:
                self.pending.popitem(last=False)
        if problem:
            self._latest_at = now
            state = "stale" if problem in ("source_frame_stale_or_future", "detection_expired",
                                           "before_observation_start", "source_stamp_regressed") else "waiting"
            self._latest = self._result(state, signals=self._unknown_signals(objects, problem), reason=problem)
            return deepcopy(self._latest)
        if self._stream != stream:
            if self._stream is not None:
                self._retired_streams.append(self._stream)
            self.tracks.clear()
            self._seen.clear()
            self._stream = stream
        self._last_stamp = source
        self._seen.append(identity)
        self.last_error = None
        import cv2
        if jpeg_size(frame[0]) is None:
            return self.error("invalid_or_oversized_jpeg")
        pixels = cv2.imdecode(np.frombuffer(frame[0], np.uint8), cv2.IMREAD_COLOR)
        if pixels is None or pixels.shape[0] > 2160 or pixels.shape[1] > 3840:
            return self.error("invalid_rgb_image")
        self.last_sync_at = now
        detections = [(bbox_of(obj.get("bbox")), self._confidence(obj)) for obj in objects]
        matches = self._associate(detections)
        available = set(self.tracks)
        signals = []
        for index, obj in enumerate(objects):
            box, confidence = detections[index]
            if box is None or confidence is None or confidence < self.config.min_detection_confidence:
                signals.extend(self._unknown_signals([obj], "invalid_or_weak_detection"))
                continue
            key = matches.get(index)
            if key is None:
                if len(self.tracks) >= self.config.max_tracks:
                    signals.extend(self._unknown_signals([obj], "track_capacity"))
                    continue
                self._next_id += 1
                key = f"signal_{self._next_id:03d}"
                self.tracks[key] = Track(key, box, now)
            else:
                available.remove(key)
            track = self.tracks[key]
            evidence = classify_color(pixels, box).to_dict()
            track.history.append(evidence)
            track.bbox, track.last_seen = box, now
            result = self._vote(evidence, track.history)
            # Bulbs within one housing retain independent evidence, including simultaneous colours.
            for bulb in result["bulbs"]:
                previous = []
                for sample in track.history:
                    matched = [b for b in sample["bulbs"] if overlap(b["bbox"], bulb["bbox"]) >= .3]
                    previous.append(matched[0] if len(matched) == 1 else {})
                voted = self._vote(bulb, previous)
                bulb.update({k: voted[k] for k in ("color", "color_confidence", "symbol",
                                                   "symbol_confidence", "stability")})
            signals.append({"signal_id": key, "bbox": box, "position": self._position(obj),
                            "detection_confidence": confidence, **result,
                            "observations": len(track.history)})
        # Missing targets cannot lend votes to a later reappearance.
        for key in available:
            self.tracks[key].history.clear()
        self._latest_at = now
        self._latest = self._result("ok" if signals else "no_signal", signals=signals, stamp=source)
        self._latest.update(stream_id=stream, frame_seq=payload.get("frame_seq"))
        result = self.latest()
        if result["state"] == "stale":
            for track in self.tracks.values():
                track.history.clear()
        return result

    @staticmethod
    def _vote(current, history):
        result = deepcopy(current)
        stable = True
        for field in ("color", "symbol"):
            label = current.get(field, "unknown")
            votes = sum(item.get(field) == label for item in history)
            if label == "unknown" or votes < 3:
                result[field], result[field+"_confidence"] = "unknown", 0.
                result[field+"_stability"] = "unstable"
                stable = False
            else:
                result[field+"_stability"] = "stable"
        result["stability"] = "stable" if stable else "unstable"
        return result

    def latest(self):
        now = self.clock()
        self._prune(now)
        result = deepcopy(self._latest)
        source = stamp_ns(result.get("frame_stamp"))
        expired = source is not None and not 0 <= (self.ros_clock()-source)/10**9 <= self.config.max_age_s
        if expired or (self._latest_at is not None and now-self._latest_at > self.config.max_age_s):
            result["state"], result["reason"] = "stale", "observation_expired"
            for signal in result["signals"]:
                for item in [signal, *signal.get("bulbs", [])]:
                    item.update(color="unknown", color_confidence=0., symbol="unknown",
                                symbol_confidence=0., stability="unstable",
                                color_stability="unstable", symbol_stability="unstable")
        return result

    def info(self):
        now = self.clock()
        self._prune(now)
        latest = self.latest()
        return {"last_image_time": self.last_image_stamp/10**9 if self.last_image_stamp else None,
                "last_detection_time": self.last_detection_stamp/10**9 if self.last_detection_stamp else None,
                "has_synchronized_frame": (latest["state"] in ("ok", "no_signal")
                    and self.last_sync_at is not None and now-self.last_sync_at <= self.config.max_age_s),
                "cache_frames": len(self.frames), "cache_limit": self.config.cache_size,
                "cache_bytes": sum(len(frame[0]) for frame in self.frames.values()),
                "pending_detections": len(self.pending), "tracks": len(self.tracks),
                "error": self.last_error}
