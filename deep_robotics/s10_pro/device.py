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
    from .parameters import sensor_defaults, validate_sensor_settings, normalize_config_updates, validate_empty_arguments
    from .s10_skeleton import model_description, model_joint_name, model_angle
except ImportError:
    from reader import source_specs
    from parameters import sensor_defaults, validate_sensor_settings, normalize_config_updates, validate_empty_arguments
    from s10_skeleton import model_description, model_joint_name, model_angle

CARDS = {
    "battery": ("Battery", ("battery",), 1.0),
    "joints": ("Joint Feedback", ("joints", "imu"), 10.0),
    "lidar_cloud": ("LiDAR Point Cloud", ("lidar",), 10.0),
    "imu": ("IMU", ("imu",), 20.0),
}
MONITOR_CARDS = tuple(CARDS)
SOURCE_TOPICS = {key: spec["topic"] for key, spec in source_specs({}).items()}


def _finite(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def _axis(data, fields):
    data = data if isinstance(data, dict) else {}
    return [_finite(data.get(key)) for key in fields]


def _monitor_output(name, source):
    if name == "battery":
        records = source.get("data") if isinstance(source, dict) else None
        item = records[0] if isinstance(records, list) and records and isinstance(records[0], dict) else {}
        voltage = _finite(item.get("voltage"))
        temperatures = item.get("battery_temperature")
        return {"soc": _finite(item.get("battery_level")), "soh": None,
                "current": _finite(item.get("current")),
                "voltage": [voltage] if voltage is not None else [], "cell_vol": [],
                "temperature": [_finite(v) for v in temperatures] if isinstance(temperatures, list) else [],
                "cycle": _finite(item.get("cycles"))}
    if name == "joints":
        source = source if isinstance(source, dict) else {}
        joint_data = source.get("joints")
        inner = joint_data.get("data") if isinstance(joint_data, dict) else None
        records = inner.get("joints_data") if isinstance(inner, dict) else None
        records = records if isinstance(records, list) else []
        joints = []
        for idx, record in enumerate(records):
            record = record if isinstance(record, dict) else {}
            temps = [_finite(record.get(key)) for key in ("motion_temp", "driver_temp")]
            joint_name = model_joint_name(idx, record)
            raw_position = _finite(record.get("position"))
            entry = {"idx": idx,
                     "q": model_angle(idx, raw_position) if joint_name else raw_position,
                     "dq": _finite(record.get("velocity")), "tau": _finite(record.get("torque")),
                     "temp": [value for value in temps if value is not None]}
            if joint_name:
                entry["name"] = joint_name
            joints.append(entry)
        imu = source.get("imu")
        orientation = imu.get("orientation") if isinstance(imu, dict) else None
        return {"joints": joints, "imu_quat": _axis(orientation, ("w", "x", "y", "z"))}
    if name == "imu":
        source = source if isinstance(source, dict) else {}
        orientation = source.get("orientation")
        x, y, z, w = _axis(orientation, ("x", "y", "z", "w"))
        rpy = [None, None, None]
        if all(value is not None for value in (x, y, z, w)):
            roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            pitch_value = 2 * (w * y - z * x)
            pitch = math.asin(max(-1.0, min(1.0, pitch_value)))
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            rpy = [roll, pitch, yaw]
        header = source.get("header")
        stamp = header.get("stamp") if isinstance(header, dict) else None
        valid_stamp = (isinstance(stamp, dict)
                       and type(stamp.get("sec")) is int
                       and type(stamp.get("nanosec")) is int
                       and 0 <= stamp["nanosec"] < 1_000_000_000)
        # Preserve the sensor's ROS timestamp, never substitute wall-clock time.
        output_header = {"stamp": {"sec": stamp["sec"], "nanosec": stamp["nanosec"]}
                         if valid_stamp else None}
        return {"header": output_header, "quaternion": [w, x, y, z],
                "gyroscope": _axis(source.get("angular_velocity"), ("x", "y", "z")),
                "accelerometer": _axis(source.get("linear_acceleration"), ("x", "y", "z")),
                "rpy": rpy, "temperature": None}
    raise ValueError("Unsupported monitor card")


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
    def __init__(self, name, config, namespace, bridge, publish, source_topics=None,
                 source_types=None):
        self.PREFIX = self.name = name
        self.title, self.sources, _ = CARDS[name]
        self._bridge, self._publish = bridge, publish
        self._source_topics = {**SOURCE_TOPICS, **(source_topics or {})}
        self._source_types = {key: value["type"] for key, value in source_specs({}).items()}
        self._source_types.update(source_types or {})
        base = "/" + "/".join(part for part in (str(namespace).strip("/"), "s10_pro") if part)
        self.topic = f"{base}/{name}"
        self.joint_detail_topic = f"{base}/joints_info" if name == "joints" else None
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
        if self.joint_detail_topic:
            return [{"topic": self.joint_detail_topic, "format": "data/json"},
                    {"topic": self.topic, "format": "sensor/skeleton"}]
        return [{"topic": self.topic,
                 "format": "sensor/pointcloud" if self.name == "lidar_cloud" else "data/json"}]

    def get_tool(self):
        output = {"name": self.name, "title": f"S10 Pro · {self.title}", "type": "sensor", "multiInstance": False,
                  "description": f"S10 Pro {self.title} read-only monitor.",
                  "inputSchema": {"type": "object", "properties": {}},
                  "topic_in": [], "topic_out": self._ports()}
        if self.name == "lidar_cloud":
            output["configSchema"] = {"type": "object", "properties": {}}
        return output

    def get_tools(self):
        tools = [self.get_tool()]
        if self.name == "joints":
            tools.append({"name": "model", "title": "S10 Pro · Model", "type": "resource",
                          "description": "Read-only S10 Pro 16-joint skeleton model.",
                          "inputSchema": {"type": "object", "properties": {},
                                          "additionalProperties": False}})
        return tools

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
        values = {source: self._snapshot(source) for source in self.sources}
        source_info = {source: value[1] for source, value in values.items()}
        fresh = all(item["fresh"] for item in source_info.values())
        connected = any(item["fresh"] for item in source_info.values())
        received = any(item["received"] for item in source_info.values())
        return {
            "card": self.name, "read_only": True, "timestamp_ms": int(time.time() * 1000),
            "connected": connected, "fresh": fresh, "stale": received and not fresh,
            "status": "live" if fresh else "partial" if connected else "stale" if received else "no_data",
            "source": source_info,
            "data": ({source: value[0] for source, value in values.items()}
                     if len(self.sources) != 1 else values[self.sources[0]][0]),
        }, values


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
            result["data"] = payload["data"]
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
                    if self.name == "lidar_cloud":
                        _, source, binary = values["lidar"]
                        sequence = source["sequence"]
                        if (source["fresh"] and isinstance(binary, bytes) and binary
                                and type(sequence) is int and (self._last_binary_sequence is None
                                or sequence > self._last_binary_sequence)):
                            self._publish(self.topic, "sensor/pointcloud", binary)
                            self._last_binary_sequence = sequence
                    elif payload["fresh"]:
                        content = _monitor_output(self.name, payload["data"])
                        if self.joint_detail_topic:
                            rows = [{"joint": joint.get("name", joint["idx"]),
                                     "angle_rad": joint.get("q"), "speed": joint.get("dq"),
                                     "torque": joint.get("tau"), "temperature": joint.get("temp")}
                                    for joint in content["joints"]]
                            self._publish(self.joint_detail_topic, "data/json", {"joints": rows})
                            self._publish(self.topic, "sensor/skeleton", content)
                        else:
                            self._publish(self.topic, "data/json", content)
                    self._last_error = None
                    self._last_error_detail = None
            except Exception as exc:
                with self._lock:
                    if generation == self._generation:
                        self._last_error = "Sensor stream operation failed."
                        self._last_error_detail = str(exc)
            cancel.wait(1.0 / self._settings["refresh_hz"])

    def dispatch(self, action, args):
        # Empty-schema MCP calls are routed by DriverBundle using the tool name.
        # Explicit lifecycle actions remain available for the platform.
        if self.name in MONITOR_CARDS and action == self.name:
            validate_empty_arguments(args)
            return self.start({})
        if self.name == "joints" and action == "model":
            validate_empty_arguments(args)
            return model_description()
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


def build_plugins(config, namespace, bridge, publish, *, public_only=True):
    """Construct exactly four subscription-only monitors and their model resource."""
    plugins = (config or {}).get("plugins", {})
    specs = source_specs(config or {})
    return [ReadOnlyCard(name, plugins.get(name, {}), namespace, bridge, publish,
                         source_topics={key: item["topic"] for key, item in specs.items()},
                         source_types={key: item["type"] for key, item in specs.items()})
            for name in MONITOR_CARDS]
