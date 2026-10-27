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

DOA_TOPIC = "/robo/audio/subscribe/doa_event"
ANGLE_CONVENTION = "robot_front_0_left_positive_ccw"


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
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
        confidence = number(confidence, "confidence", 0, 1)
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
                  "reference_frame": "vendor DOA front; physical installation alignment requires acceptance",
                  "age_ms": None if age is None else round(age * 1000),
                  "error": self._error}
        if state == "fresh":
            result["angle_deg"] = self._latest["angle_deg"]
            if self._latest["confidence"] is not None:
                result["confidence"] = self._latest["confidence"]
        return result

    def _info(self):
        observation = self._observation()
        return {"state": "running" if self.running else "idle",
                "sound_direction": observation,
                "valid": observation["state"] == "fresh",
                "sdk_events_received": self._events_received,
                "source_topic": DOA_TOPIC, "source_domain": 20,
                "stale_after_seconds": self.STALE_AFTER,
                "source_dependency": "DOA production may depend on vendor wakeup; this card never changes behavior switches",
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
            self._latest = self._received = None
            self._events_received = 0
            self._error = ""
            self._generation += 1
            generation = self._generation
            try:
                self._publisher = self.nodes.core.create_publisher(self.nodes.String, self.topic, 10)
                # Mark running before subscribing: a ready executor may deliver immediately.
                self.running = True
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
            self._publish()
            for owner, attribute, destroy in (
                (self.nodes.robot, "_timer", "destroy_timer"),
                (self.nodes.robot, "_subscription", "destroy_subscription"),
                (self.nodes.core, "_publisher", "destroy_publisher"),
            ):
                resource = getattr(self, attribute)
                if resource is not None:
                    getattr(owner, destroy)(resource)
                    setattr(self, attribute, None)
            return self._info()

    def _on_event(self, event, generation=None):
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
                                "valid": valid, "received_at_ms": time.time_ns() // 1_000_000}
                self._error = ""
            except (ValueError, TypeError, RecursionError) as exc:
                self._latest = {"valid": False, "received_at_ms": time.time_ns() // 1_000_000}
                self._error = str(exc)[:256]
            self._received = time.monotonic()
            self._publish()

    def _tick(self):
        with self._lock:
            if self.running:
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
