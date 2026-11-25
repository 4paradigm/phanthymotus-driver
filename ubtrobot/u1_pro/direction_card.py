"""Read-only U1 SDK DOA bridge; no motion or ASR implementation.

The vendor DoaEvent documents degrees in [-180, 180), left negative.
Receipt time is diagnostic only: no verified shared audio capture clock exists.
"""
from __future__ import annotations

import json
import math
import threading
import time

from common.vendor_runtime import action_schema, tool
from native_direction import NativeDirectionSource
from head_yaw import wrap_degrees

DOA_TOPIC = "/robo/audio/subscribe/doa_event"
ANGLE_CONVENTION = "robot_front_0_left_positive_ccw"


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be in [{low}, {high}]")
    return value


def parse_doa(event):
    """Decode the confirmed String JSON transport, including SDK data envelopes."""
    if hasattr(event, "data"):
        event = event.data
    for _ in range(6):
        if isinstance(event, str):
            if len(event) > 16384:
                raise ValueError("DOA event too large")
            event = json.loads(event)
        elif isinstance(event, dict) and "azimuth" not in event and "data" in event:
            event = event["data"]
        else:
            break
    if not isinstance(event, dict):
        raise ValueError("DOA event must contain an object")
    angle = number(event.get("azimuth"), "azimuth", -180, 180)
    if angle == 180:
        raise ValueError("azimuth must be less than 180")
    confidence = event.get("confidence")
    if confidence is not None:
        # Firmware v0.9.2 emits signed scores such as -1.0812 and 4.31 despite the message
        # comment describing [0,1]. Preserve the vendor score, never clamp it
        # or reject an otherwise valid direction as a malformed probability.
        confidence = number(confidence, "confidence", -math.inf, math.inf)
    if "valid" in event and not isinstance(event["valid"], bool):
        raise ValueError("valid must be boolean")
    return angle, confidence, event.get("valid", True)


def platform_angle(azimuth):
    """Vendor left-negative [-180,180) -> platform left-positive (-180,180]."""
    azimuth = number(azimuth, "azimuth", -180, 180)
    if azimuth == 180:
        raise ValueError("azimuth must be less than 180")
    return -azimuth if azimuth else 0.0


class SoundDirectionPlugin:
    PREFIX = "sound_direction"
    STALE_AFTER = 2.0
    HEARTBEAT = 0.5

    def __init__(self, nodes):
        self.nodes = nodes
        self.topic = f"/{nodes.namespace}/sound_direction"
        self.running = False
        self._lock = threading.RLock()
        self._subscription = self._publisher = self._timer = None
        self._latest = None
        self._received = None
        self._error = ""
        self._events_received = 0
        self._generation = 0
        config = vars(nodes).get("config", {})
        self.backend = config.get("sound_direction", {}).get("source", "sdk") if isinstance(config, dict) else "sdk"
        if self.backend not in ("sdk", "native"):
            raise ValueError("sound_direction.source must be sdk or native")
        self._native = None

    def get_tool(self):
        return tool(self.PREFIX, "sensor",
                    "U1 Pro 只读声源方位：前方 0°、左正右负。无可靠音频采集窗口时 ASR 方向未知。",
                    action_schema({"start": ([], "开始只读订阅"),
                                   "stop": ([], "停止订阅并清除方位"),
                                   "info": ([], "读取方位、有效状态和时间关联限制")}, {}),
                    topic_out=[{"topic": self.topic, "format": "sensor/sound-direction"}])

    def _observation(self):
        age = None if self._received is None else max(0, time.monotonic() - self._received)
        state = ("no_event" if not self.running or age is None else
                 "stale" if age >= self.STALE_AFTER else
                 "no_event" if self._error or not self._latest["valid"] else "fresh")
        result = {"state": state, "angle_deg": None,
                  "angle_convention": ANGLE_CONVENTION,
                  "audio_window_start_us": 0, "audio_window_end_us": 0,
                  "timestamp_ms": None if not self._latest else self._latest["received_at_ms"],
                  "timestamp_basis": "receipt_time_only",
                  "association_state": "unknown",
                  "association_reason": "SDK DOA has no verified shared audio capture window",
                  "source_backend": self.backend,
                  "reference_frame": "vendor DOA front; physical installation alignment requires acceptance",
                  "age_ms": None if age is None else round(age * 1000),
                  "error": self._error}
        if state == "fresh":
            result["angle_deg"] = self._latest["angle_deg"]
            if self._latest["confidence"] is not None:
                result["confidence"] = self._latest["confidence"]
                result["confidence_semantics"] = "vendor_raw_score_not_probability"
        if self._latest and "frame_compensation" in self._latest:
            compensation = self._latest["frame_compensation"]
            result["head_angle_deg"] = self._latest["angle_deg"] if state == "fresh" else None
            result["frame_compensation"] = compensation
            result["reference_frame"] = "torso_front"
            if state == "fresh":
                if compensation.get("state") == "estimated":
                    result["angle_deg"] = wrap_degrees(
                        self._latest["angle_deg"] + compensation["head_yaw_deg"])
                else:
                    result["state"] = "no_event"
                    result["angle_deg"] = None
                    result.pop("confidence", None)
                    result.pop("confidence_semantics", None)
        return result

    def _info(self):
        observation = self._observation()
        return {"state": "running" if self.running else "idle",
                "sound_direction": observation,
                "valid": observation["state"] == "fresh",
                "sdk_events_received": self._events_received,
                "source_topic": "/phanthy/u1_doa/event" if self.backend == "native" else DOA_TOPIC,
                "source_domain": 43 if self.backend == "native" else 20,
                "source_backend": self.backend,
                "source_status": self._native.info() if self._native is not None else None,
                "stale_after_seconds": self.STALE_AFTER,
                "source_dependency": ("Private vendor DOA worker required; no wakeup or motion behavior" if self.backend == "native"
                                      else "SDK DOA may depend on vendor wakeup; this card never changes behavior switches"),
                "topic_out": self.get_tool()["topic_out"]}

    def _publish(self):
        if self._publisher is not None:
            message = self.nodes.String()
            message.data = json.dumps(self._observation(), allow_nan=False)
            self._publisher.publish(message)

    def start(self):
        with self._lock:
            if self.running:
                return self._info()
            if any(resource is not None for resource in (self._publisher, self._subscription, self._timer, self._native)):
                # Retry incomplete cleanup before allocating a new session.
                self.stop()
            self._latest = self._received = None
            self._events_received = 0
            self._error = ""
            self._generation += 1
            generation = self._generation
            try:
                self._publisher = self.nodes.core.create_publisher(self.nodes.String, self.topic, 10)
                # Mark running before subscribing: a ready executor may deliver immediately.
                self.running = True
                if self.backend == "native":
                    self._native = NativeDirectionSource(self.nodes)
                    self._native.start()
                else:
                    self._subscription = self.nodes.robot.create_subscription(
                        self.nodes.String, DOA_TOPIC,
                        lambda event: self._on_event(event, generation), self.nodes._sensor_qos)
                self._timer = self.nodes.robot.create_timer(self.HEARTBEAT, self._tick)
                self._publish()
            except Exception:
                self.stop()
                raise
            return self._info()

    def stop(self):
        with self._lock:
            self.running = False
            self._generation += 1
            self._latest = self._received = None
            self._error = ""
            errors = []
            try:
                self._publish()
            except Exception as exc:
                errors.append(exc)
            if self._native is not None:
                try:
                    self._native.stop()
                except Exception as exc:
                    errors.append(exc)
                else:
                    self._native = None
            for owner, attribute, destroy in (
                (self.nodes.robot, "_timer", "destroy_timer"),
                (self.nodes.robot, "_subscription", "destroy_subscription"),
                (self.nodes.core, "_publisher", "destroy_publisher"),
            ):
                resource = getattr(self, attribute)
                if resource is not None:
                    try:
                        getattr(owner, destroy)(resource)
                    except Exception as exc:
                        errors.append(exc)
                    else:
                        setattr(self, attribute, None)
            if errors:
                raise RuntimeError(f"sound direction cleanup failed: {errors[0]}") from errors[0]
            return self._info()

    def _on_event(self, event, generation=None, received=None, received_ms=None):
        with self._lock:
            if not self.running or (generation is not None and generation != self._generation):
                return
            self._events_received += 1
            try:
                angle, confidence, valid = parse_doa(event)
                # Do not smooth across separate speakers or retain the last valid
                # direction on parse failure. SDK receipt time never becomes an
                # audio window, even if an undocumented timestamp is present.
                self._latest = {"angle_deg": platform_angle(angle), "confidence": confidence,
                                "valid": valid, "received_at_ms": received_ms if received_ms is not None else time.time_ns() // 1_000_000}
                if self.backend == "native" and isinstance(event, dict) and "frame_compensation" in event:
                    # Freeze this event's pose. Heartbeats must never reapply a newer head angle.
                    self._latest["frame_compensation"] = dict(event["frame_compensation"])
                self._error = ""
            except (ValueError, TypeError, RecursionError) as exc:
                self._latest = {"valid": False, "received_at_ms": time.time_ns() // 1_000_000}
                self._error = str(exc)[:256]
            self._received = time.monotonic() if received is None else received
            self._publish()

    def _tick(self):
        with self._lock:
            if self.running:
                if self._native is not None:
                    for event, received, received_ms in self._native.drain():
                        if event is None:
                            self._latest = self._received = None
                            self._error = ""
                        else:
                            self._on_event(event, received=received, received_ms=received_ms)
                    if self._native.error:
                        self._latest = self._received = None
                        self._error = self._native.error
                # Repeat terminal states for late subscribers and lost packets.
                # The original receipt timestamp stays fixed on every heartbeat.
                self._publish()

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            with self._lock:
                return self._info()
        return None
