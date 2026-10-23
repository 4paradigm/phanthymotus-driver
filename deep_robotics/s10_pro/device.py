"""Independently managed S10 Pro monitor cards with the M20 card structure.

The bridge owns robot subscriptions.  This module only reads bridge snapshots
and publishes canvas data; it never imports or instantiates a motion SDK.
"""

from __future__ import annotations

import copy
import math
import threading
import time

try:
    from .reader import source_specs
    from .parameters import (sensor_defaults, sensor_config_schema, validate_sensor_settings,
                             normalize_config_updates, validate_empty_arguments, SENSOR_TITLES, config_overlap_constraints)
    from .monitoring import (MOTION_CAPABILITY, DEVICE_ACTIONS, DEVICE_REASON, ErrorObservations,
                             capabilities_snapshot, device_summary, estop_summary, motion_summary)
except ImportError:  # Direct driver execution; reader itself imports no ROS here.
    from reader import source_specs
    from parameters import (sensor_defaults, sensor_config_schema, validate_sensor_settings,
                            normalize_config_updates, validate_empty_arguments, SENSOR_TITLES, config_overlap_constraints)
    from monitoring import (MOTION_CAPABILITY, DEVICE_ACTIONS, DEVICE_REASON, ErrorObservations,
                            capabilities_snapshot, device_summary, estop_summary, motion_summary)


CARDS = {
    "state": (SENSOR_TITLES["state"], ("battery", "motion_info", "hard_estop", "exception", "fault_event", "fault_aggregator"), 1.0),
    "joints": (SENSOR_TITLES["joints"], ("joints",), 5.0),
    "imu": ("IMU", ("imu",), 5.0),
    "odometry": (SENSOR_TITLES["odometry"], ("odometry",), 5.0),
    "lidar": (SENSOR_TITLES["lidar"], ("lidar",), 2.0),
    "motion_info": (SENSOR_TITLES["motion_info"], ("motion_info",), 5.0),
    "device_status": (SENSOR_TITLES["device_status"], ("battery", "joints", "cpu_status", "memory_status", "filesystem_status", "network_status"), 1.0),
    "hard_estop": (SENSOR_TITLES["hard_estop"], ("hard_estop",), 5.0),
    "errors": (SENSOR_TITLES["errors"], ("exception", "fault_event", "fault_aggregator"), 1.0),
    "motion_events": (SENSOR_TITLES["motion_events"], (), 2.0),
    "capabilities": (SENSOR_TITLES["capabilities"], (), 1.0),
    "ros_graph": (SENSOR_TITLES["ros_graph"], ("ros_graph",), 0.5),
    "device": (SENSOR_TITLES["device"], (), 1.0),
}
SOURCE_TOPICS = {
    "battery": "/BATTERY_DATA", "motion_info": "/MOTION_INFO",
    "joints": "/JOINTS_DATA_10HZ", "imu": "/IMU",
    "odometry": "/ODOM", "lidar": "/LIDAR/POINTS",
    "hard_estop": "/HES_STATUS", "exception": "/EXCEPTION_NOTIFICATION",
    "fault_event": "/fault_event", "fault_aggregator": "/fault_aggregator", "ros_graph": "ros2://graph",
    "cpu_status": "/cpu_status", "memory_status": "/memory_status",
    "filesystem_status": "/filesystem_status", "network_status": "/network_status",
}
ERROR_SOURCES = ("exception", "fault_event", "fault_aggregator")
RESOURCE_SOURCES = ("cpu_status", "memory_status", "filesystem_status", "network_status")
LOCAL_CARDS = ("motion_events", "capabilities", "device")


def _json_value(value):
    """Keep raw message structure, with invalid floating point values explicit."""
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return copy.deepcopy(value)


class ReadOnlyCard:
    def __init__(self, name, config, namespace, bridge, publish, source_topics=None, source_types=None, motion=None):
        self.PREFIX = self.name = name
        self.title, self.sources, default_hz = CARDS[name]
        self._bridge, self._publish = bridge, publish
        self._motion = motion
        self._errors = ErrorObservations()
        self._source_topics = {**SOURCE_TOPICS, **(source_topics or {})}
        self._source_types = {key: value["type"] for key, value in source_specs({}).items()}
        self._source_types.update(source_types or {})
        base = "/" + "/".join(part for part in (str(namespace).strip("/"), "s10_pro") if part)
        # The action card owns /motion_events. This independent view must not
        # add a second producer to that existing event/ACP-related stream.
        self.topic = f"{base}/motion_events_view" if name == "motion_events" else f"{base}/{name}"
        self.info_topic = f"{base}/lidar_info" if name == "lidar" else None
        self._defaults = self._validate({**sensor_defaults(name), **dict(config or {})})
        self._settings = dict(self._defaults)
        self._lock = threading.RLock()
        # Serializes bridge leases, but never blocks stop on an in-flight start.
        self._lease_lock = threading.Lock()
        self._lease_generation = None
        self._generation = 0
        self._config_revision = 0
        self._state = "idle"
        self._thread = None
        self._cancel = None
        self._last_error = None
        self._last_error_detail = None
        self._last_binary_sequence = None

    def _validate(self, settings):
        return validate_sensor_settings(self.name, settings)

    def _ports(self):
        ports = [{"topic": self.topic, "format": "sensor/pointcloud" if self.name == "lidar" else "data/json"}]
        if self.info_topic:
            ports.append({"topic": self.info_topic, "format": "data/json"})
        return ports

    def get_tool(self):
        with self._lock:
            config_schema = sensor_config_schema(self.name, self._settings)
        properties = copy.deepcopy(config_schema["properties"])
        routing = {"instance_id": {"type": "string", "minLength": 1, "pattern": r"\S", "description": "Internal card instance metadata."},
                   "_tool_name": {"type": "string", "minLength": 1, "pattern": r"\S", "description": "Internal tool routing metadata."}}
        description = f"Read-only S10 Pro {self.title} data. Stopping releases only this card's data stream."
        if self.name == "device":
            description = f"S10 device-management availability. Lights, mode, sleep, wake and automatic sleep are unavailable. {DEVICE_REASON}"
        elif self.name == "motion_events":
            description = "Independent read-only view of existing application motion events. Starting or stopping this view never starts, stops or notifies a motion task."
        return {
            "name": self.name, "title": f"S10 Pro · {self.title}", "type": "actuator" if self.name == "device" else "sensor", "multiInstance": False,
            "description": description,
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["info", "start", "stop", "status", "config"],
                           "description": "Required operation; no action is selected by default."},
                **properties, **routing, "config": copy.deepcopy(config_schema)},
                "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "info": {"params": [], "description": "Read card status without starting a stream."},
                    "status": {"params": [], "description": "Read current status and source freshness."},
                    "stop": {"params": [], "description": "Release only this card's stream."},
                    "start": {"params": list(properties), "description": "Start this card's read-only observations."},
                    "config": {"params": list(properties), "description": "Apply validated settings for this card."}},
                "allOf": [{"if": {"properties": {"action": {"enum": ["info", "status", "stop"]}},
                                  "required": ["action"]},
                           "then": {"not": {"anyOf": [{"required": [key]} for key in [*properties, "config"]]}}}]
                    + config_overlap_constraints(properties)},
            "configSchema": config_schema,
            "topic_in": [], "topic_out": self._ports(),
            **({"unavailable_actions": list(DEVICE_ACTIONS), "availability_reason": DEVICE_REASON} if self.name == "device" else {}),
        }

    def get_tools(self):
        return [self.get_tool()]

    def _snapshot(self, source):
        try:
            snapshot = self._bridge.snapshot(source) or {}
            if not isinstance(snapshot, dict):
                raise TypeError("bridge snapshot is not an object")
        except Exception as exc:
            snapshot = {"error": "Sensor snapshot unavailable.", "diagnostic_error": str(exc)}
        data = snapshot.get("data")
        data = _json_value(data) if isinstance(data, dict) else None
        binary = snapshot.get("binary")
        has_data = data is not None or (source == "lidar" and isinstance(binary, bytes) and bool(binary))
        age = snapshot.get("age_s")
        if isinstance(age, bool) or not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
            age = None
        received_at = snapshot.get("received_at_ms")
        fresh = bool(has_data and age is not None and age <= self._settings["stale_after_s"]
                     and snapshot.get("fresh", False) and not snapshot.get("error"))
        header = data.get("header") if isinstance(data, dict) else None
        stamp = header.get("stamp") if isinstance(header, dict) else None
        source_info = {
            "topic": self._source_topics.get(source), "message_type": self._source_types.get(source),
            "received": has_data,
            "received_at_ms": received_at, "age_s": age,
            "fresh": fresh, "stale": has_data and not fresh,
            "stamp": stamp, "error": "Sensor source unavailable." if snapshot.get("error") else None,
            "diagnostic_error": snapshot.get("diagnostic_error", snapshot.get("error")),
            "sequence": snapshot.get("sequence", 0),
        }
        return data, source_info, binary

    def _payload(self):
        if self.name in LOCAL_CARDS:
            return self._local_payload(), {}
        values = {source: self._snapshot(source) for source in self.sources}
        source_info = {source: value[1] for source, value in values.items()}
        # Candidate fault streams are event driven with unverified heartbeat
        # semantics. They cannot determine the state card's online indicator.
        required = (("battery", "motion_info", "hard_estop") if self.name == "state" else
                    ("battery", "joints") if self.name == "device_status" else self.sources)
        fresh = all(source_info[source]["fresh"] for source in required)
        connected = any(source_info[source]["fresh"] for source in required)
        received = any(source_info[source]["received"] for source in required)
        payload = {
            "card": self.name, "read_only": True, "timestamp_ms": int(time.time() * 1000),
            "connected": connected, "fresh": fresh, "stale": received and not fresh,
            "status": "live" if fresh else "partial" if connected else "stale" if received else "no_data",
            "source": source_info,
            "data": ({source: value[0] for source, value in values.items()}
                     if len(self.sources) != 1 else values[self.sources[0]][0]),
        }
        if self.name == "state":
            payload["capabilities"] = {"motion": dict(MOTION_CAPABILITY)}
            payload["units"] = {"battery": {"voltage": None, "current": None, "capacity": None,
                                                "battery_level": None, "temperature": None},
                                "motion_info": None}
            payload["state_code_meaning"] = "unknown"
            payload["summary"] = {
                "online": connected, "all_required_sources_fresh": fresh,
                "required_sources": list(required),
                "motion": motion_summary(values["motion_info"][0], source_info["motion_info"]),
                "hardware_emergency_stop": estop_summary(values["hard_estop"][0], source_info["hard_estop"]),
                "device": device_summary(values["battery"][0], None),
                "errors": self._error_summary({key: values[key] for key in ERROR_SOURCES}),
            }
        elif self.name == "motion_info":
            payload["summary"] = motion_summary(payload["data"], source_info["motion_info"])
        elif self.name == "hard_estop":
            payload["summary"] = estop_summary(payload["data"], source_info["hard_estop"])
        elif self.name == "device_status":
            payload["summary"] = device_summary(values["battery"][0], values["joints"][0],
                                                {key: values[key] for key in RESOURCE_SOURCES},
                                                self._settings["stale_after_s"])
        elif self.name == "errors":
            payload["summary"] = self._error_summary(values)
            payload["health"] = payload["summary"]["current_fault_state"]
        elif self.name == "ros_graph":
            payload["summary"] = {
                "discovery_only": True, "control_ownership": "unknown",
                "note": "Discovered topics and publishers do not prove valid data, hardware availability or command authority.",
            }
        elif self.name == "joints":
            data = payload["data"] or {}
            inner = data.get("data")
            records = inner.get("joints_data", []) if isinstance(inner, dict) else []
            if not isinstance(records, list):
                records = []
            payload["joint_count"] = len(records)
            payload["joint_labels"] = [self._joint_label(item.get("name")) if isinstance(item, dict) else None
                                       for item in records]
            payload["units"] = {"position": None, "velocity": None, "torque": None, "temperature": None}
            payload["status_word_meaning"] = "unknown"
        elif self.name in ("imu", "odometry"):
            payload["message_type"] = self._source_types[self.sources[0]]
            payload["frame_contract"] = "raw_ros_message"
        return payload, values

    def _error_summary(self, values):
        with self._lock:
            return self._errors.update(values)

    def _local_payload(self):
        payload = {
            "card": self.name, "read_only": True, "timestamp_ms": int(time.time() * 1000),
            "connected": False, "fresh": False, "stale": False,
            "source": {}, "live_robot_data": False, "status": "informational",
        }
        if self.name == "capabilities":
            payload["data"] = capabilities_snapshot()
        elif self.name == "device":
            payload.update(status="locked", execution_available=False,
                           data=capabilities_snapshot()["device"])
        else:
            payload.update(status="unavailable", data=None)
            if self._motion is None:
                payload["reason"] = "The application motion-event provider is not connected."
                return payload
            try:
                # status() is the existing passive observation API. Do not call
                # start/stop/cancel/dispatch, access task stores, or emit ACP.
                state = self._motion.status()
                if not isinstance(state, dict) or not isinstance(state.get("events"), list):
                    raise TypeError("motion-event provider returned an invalid snapshot")
                events = _json_value(state["events"][-64:])
                last = events[-1] if events else None
                stamp = last.get("timestamp_ms") if isinstance(last, dict) else None
                age = ((payload["timestamp_ms"] - stamp) / 1000
                       if type(stamp) is int and stamp <= payload["timestamp_ms"] else None)
                payload.update(status="observing", data={
                    "events": events, "event_count": len(events), "history_limit": 64,
                    "last_event_age_s": age, "motion_card_state": state.get("state"),
                    "motion_phase": state.get("phase"),
                    "event_source_topic": getattr(self._motion, "events_topic", None),
                    "completion_notifications_emitted_by_view": 0,
                    "note": "Existing application event history; no events does not establish robot state. Event timestamps are preserved.",
                })
            except Exception:
                payload["reason"] = "The application motion-event snapshot is unavailable."
        return payload

    @staticmethod
    def _joint_label(name):
        if isinstance(name, (list, tuple)):
            try:
                return bytes(name).split(b"\0", 1)[0].decode("ascii")
            except (ValueError, TypeError, UnicodeDecodeError):
                return None
        return name if isinstance(name, str) else None

    def _reader_status(self):
        try:
            status = self._bridge.status() or {}
            return {"reader_running": bool(status.get("reader_running", False)),
                    "last_error": "Sensor reader unavailable." if status.get("last_error") else None,
                    "diagnostic_error": status.get("last_error")}
        except Exception as exc:
            return {"reader_running": False, "last_error": "Sensor reader status unavailable.", "diagnostic_error": str(exc)}

    def status(self):
        with self._lock:
            payload, _ = self._payload()
            # A stopped card must not claim a live link merely because another
            # card is keeping the shared reader and its cache alive.
            active = self._state == "running"
            result = {key: payload[key] for key in ("source", "fresh", "stale", "status")}
            result.update(state=self._state, connected=active and payload["connected"],
                          enabled=self._settings["enabled"], read_only=True,
                          topic_in=[], topic_out=self._ports(), settings=dict(self._settings),
                          last_error=self._last_error, diagnostic_error=self._last_error_detail, reader=self._reader_status())
            for key in ("data", "summary", "health", "live_robot_data", "reason", "execution_available"):
                if key in payload:
                    result[key] = payload[key]
            if self.name == "state":
                result["capabilities"] = {"motion": dict(MOTION_CAPABILITY)}
            return result

    def _overrides(self, options):
        return normalize_config_updates({} if options is None else options, sensor_defaults(self.name))

    def configure(self, options):
        overrides = self._overrides(options)
        with self._lock:
            defaults = {**self._settings, **overrides}
            settings = self._validate(defaults)
            self._defaults, self._settings = defaults, settings
            self._config_revision += 1
            config_revision = self._config_revision
            generation = self._generation
            active = self._state in ("running", "starting")
        if not settings["enabled"]:
            return self.stop(_expected_config_revision=config_revision)
        if active:
            # Updating one owner's lease changes reader rates/point limits,
            # without releasing another card or restarting a robot process.
            with self._lease_lock:
                with self._lock:
                    if generation != self._generation or self._state != "running":
                        return self.status()
                    # Another configuration request can overtake this one
                    # before the lease lock is acquired.  Always apply the
                    # latest committed settings, never the older request's
                    # captured dictionary.
                    settings = dict(self._settings)
                try:
                    if self.sources:
                        self._bridge.acquire(self.name, list(self.sources), dict(settings))
                        self._lease_generation = generation
                    with self._lock:
                        canceled = generation != self._generation or self._state != "running"
                    if canceled:
                        if self.sources:
                            self._bridge.release(self.name)
                        self._lease_generation = None
                except Exception as exc:
                    with self._lock:
                        if generation == self._generation:
                            self._generation += 1
                            self._cancel.set()
                            self._state, self._last_error = "error", "Sensor reader unavailable."
                            self._last_error_detail = str(exc)
                    try:
                        if self.sources:
                            self._bridge.release(self.name)
                    except Exception:
                        pass
                    self._lease_generation = None
        return self.status()

    def start(self, options=None):
        overrides = self._overrides(options)
        with self._lock:
            settings = self._validate({**self._settings, **overrides})
        with self._lock:
            if self._state in ("running", "starting"):
                return self.status()
            self._settings = settings
            if not settings["enabled"]:
                self._state = "disabled"
                return self.status()
            self._generation += 1
            generation = self._generation
            self._state, self._last_error = "starting", None
            self._last_error_detail = None
            self._cancel = cancel = threading.Event()
            self._last_binary_sequence = None
        with self._lease_lock:
            with self._lock:
                if generation != self._generation or cancel.is_set():
                    return self.status()
            try:
                if self.sources:
                    self._bridge.acquire(self.name, list(self.sources), dict(settings))
                    self._lease_generation = generation
                with self._lock:
                    if generation == self._generation and not cancel.is_set():
                        self._state = "running"
                        self._thread = threading.Thread(target=self._run, args=(generation, cancel),
                                                        name=f"s10-pro-{self.name}", daemon=True)
                        self._thread.start()
                        return self.status()
                # stop() canceled this start while bridge.acquire was blocked.
                if self.sources:
                    self._bridge.release(self.name)
                self._lease_generation = None
            except Exception as exc:
                try:
                    if self.sources:
                        self._bridge.release(self.name)
                except Exception:
                    pass
                self._lease_generation = None
                with self._lock:
                    if generation == self._generation:
                        self._state, self._last_error = "error", "Sensor reader unavailable."
                        self._last_error_detail = str(exc)
            return self.status()

    def _release_through(self, stopped_generation):
        with self._lease_lock:
            if self._lease_generation is not None and self._lease_generation <= stopped_generation:
                try:
                    self._bridge.release(self.name)
                except Exception as exc:
                    with self._lock:
                        self._last_error = "Sensor stream operation failed."
                        self._last_error_detail = str(exc)
                self._lease_generation = None

    def stop(self, *, _expected_config_revision=None):
        with self._lock:
            if (_expected_config_revision is not None
                    and _expected_config_revision != self._config_revision):
                return self.status()
            stopped_generation = self._generation
            self._generation += 1
            if self._cancel is not None:
                self._cancel.set()
            self._state = "idle"
            thread = self._thread
            self._thread = None
        # If acquire is pending, its canceled generation releases itself.
        if self._lease_lock.acquire(blocking=False):
            try:
                leased = self._lease_generation is not None and self._lease_generation <= stopped_generation
                if leased:
                    try:
                        self._bridge.release(self.name)
                    except Exception as exc:
                        with self._lock:
                            self._last_error = "Sensor stream operation failed."
                            self._last_error_detail = str(exc)
                    self._lease_generation = None
            finally:
                self._lease_lock.release()
        else:
            # Also cover the tiny interval after start/config's success check
            # but before its lease lock is released.  A canceled acquire does
            # its own cleanup; this deferred release is safe and idempotent.
            threading.Thread(target=self._release_through, args=(stopped_generation,),
                             name=f"s10-pro-{self.name}-release", daemon=True).start()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        return self.status()

    def _run(self, generation, cancel):
        while not cancel.is_set():
            try:
                payload, values = self._payload()
                with self._lock:
                    if generation != self._generation or self._state != "running" or cancel.is_set():
                        return
                    if self.name == "lidar":
                        _, source, binary = values["lidar"]
                        sequence = source["sequence"]
                        should_send = (source["fresh"] and isinstance(binary, bytes) and bool(binary)
                                       and isinstance(sequence, int) and not isinstance(sequence, bool)
                                       and (self._last_binary_sequence is None
                                            or sequence > self._last_binary_sequence))
                        if should_send:
                            self._publish(self.topic, "sensor/pointcloud", binary)
                            self._last_binary_sequence = sequence
                        payload["binary_published"] = should_send
                        self._publish(self.info_topic, "data/json", payload)
                    else:
                        self._publish(self.topic, "data/json", payload)
                    self._last_error = None
                    self._last_error_detail = None
            except Exception as exc:
                with self._lock:
                    if generation == self._generation:
                        self._last_error = "Sensor stream operation failed."
                        self._last_error_detail = str(exc)
            cancel.wait(1.0 / self._settings["refresh_hz"])

    def dispatch(self, action, args):
        if self.name == "device" and action in DEVICE_ACTIONS:
            return {"state": "unavailable", "status": "unavailable", "action": action,
                    "accepted": False, "command_sent": False, "reason": DEVICE_REASON}
        if action == "start":
            return self.start(args)
        if action == "stop":
            validate_empty_arguments(args)
            return self.stop()
        if action == "config":
            return self.configure(args)
        if action in ("info", "status"):
            validate_empty_arguments(args)
            return self.status()
        return None


def build_plugins(config, namespace, bridge, publish, *, motion=None, motion_bridge=None):
    """Keep the five legacy IDs, then add idle M20-structure monitor cards."""
    plugins = (config or {}).get("plugins", {})
    specs = source_specs(config or {})
    return [ReadOnlyCard(name, plugins.get(name, {}), namespace,
                         motion_bridge if name in ("motion_info", "hard_estop") and motion_bridge is not None else bridge, publish,
                         source_topics={key: item["topic"] for key, item in specs.items()},
                         source_types={key: item["type"] for key, item in specs.items()}, motion=motion) for name in CARDS]
