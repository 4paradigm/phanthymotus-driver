"""Five independently managed, read-only S10 Pro canvas sensors.

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
except ImportError:  # Direct driver execution; reader itself imports no ROS here.
    from reader import source_specs


CARDS = {
    "state": ("状态", ("battery", "motion_info"), 1.0),
    "joints": ("关节反馈", ("joints",), 5.0),
    "imu": ("IMU", ("imu",), 5.0),
    "odometry": ("里程计", ("odometry",), 5.0),
    "lidar": ("雷达点云", ("lidar",), 2.0),
}
SOURCE_TOPICS = {
    "battery": "/BATTERY_DATA", "motion_info": "/MOTION_INFO",
    "joints": "/JOINTS_DATA_10HZ", "imu": "/IMU",
    "odometry": "/ODOM", "lidar": "/LIDAR/POINTS",
}
MOTION_CAPABILITY = {
    "supported": False,
    "status": "requires_vendor_contract",
    "description": "本驱动只读；运动控制需要本机版本对应的厂家接口约定。",
}


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
    def __init__(self, name, config, namespace, bridge, publish, source_topics=None, source_types=None):
        self.PREFIX = self.name = name
        self.title, self.sources, default_hz = CARDS[name]
        self._bridge, self._publish = bridge, publish
        self._source_topics = {**SOURCE_TOPICS, **(source_topics or {})}
        self._source_types = {key: value["type"] for key, value in source_specs({}).items()}
        self._source_types.update(source_types or {})
        base = "/" + "/".join(part for part in (str(namespace).strip("/"), "s10_pro") if part)
        self.topic = f"{base}/{name}"
        self.info_topic = f"{base}/lidar_info" if name == "lidar" else None
        self._defaults = {"enabled": True, "refresh_hz": default_hz, "stale_after_s": 5.0}
        if name == "lidar":
            self._defaults["max_points"] = 6000
        self._defaults.update(dict(config or {}))
        self._settings = self._validate(self._defaults)
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
        self._last_binary_sequence = None

    def _validate(self, settings):
        result = {key: settings[key] for key in ("enabled", "refresh_hz", "stale_after_s")}
        if not isinstance(result["enabled"], bool):
            raise ValueError("enabled must be a boolean")
        for key, minimum, maximum in (("refresh_hz", 0.1, 30.0), ("stale_after_s", 0.1, 300.0)):
            value = result[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{key} must be a finite number")
            if not math.isfinite(value) or not minimum <= value <= maximum:
                raise ValueError(f"{key} must be between {minimum} and {maximum}")
            result[key] = float(value)
        if self.name == "lidar":
            value = settings.get("max_points", 6000)
            if isinstance(value, bool) or not isinstance(value, int) or not 100 <= value <= 40000:
                raise ValueError("max_points must be an integer between 100 and 40000")
            result["max_points"] = value
        return result

    def _ports(self):
        ports = [{"topic": self.topic, "format": "sensor/pointcloud" if self.name == "lidar" else "data/json"}]
        if self.info_topic:
            ports.append({"topic": self.info_topic, "format": "data/json"})
        return ports

    def get_tool(self):
        properties = {
            "enabled": {"type": "boolean", "title": "启用", "default": self._defaults["enabled"]},
            "refresh_hz": {"type": "number", "title": "显示频率 (Hz)", "minimum": 0.1,
                           "maximum": 30, "default": self._defaults["refresh_hz"],
                           "description": "画布数据刷新频率，不是机器人控制频率。"},
            "stale_after_s": {"type": "number", "title": "数据过期时间 (秒)", "minimum": 0.1,
                              "maximum": 300, "default": self._defaults["stale_after_s"]},
        }
        if self.name == "lidar":
            properties["max_points"] = {"type": "integer", "title": "最大显示点数", "minimum": 100,
                                        "maximum": 40000, "default": self._defaults["max_points"]}
        return {
            "name": self.name, "title": f"S10 Pro · {self.title}", "type": "sensor", "multiInstance": False,
            "description": f"S10 Pro {self.title}只读数据；停止仅关闭本卡数据流，不控制机器人。",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["info", "start", "stop", "status", "config"]},
                **copy.deepcopy(properties),
                "config": {"type": "object", "properties": copy.deepcopy(properties), "additionalProperties": False}},
                "required": ["action"], "additionalProperties": False},
            "configSchema": {"type": "object", "properties": properties, "additionalProperties": False},
            "topic_in": [], "topic_out": self._ports(),
        }

    def get_tools(self):
        return [self.get_tool()]

    def _snapshot(self, source):
        try:
            snapshot = self._bridge.snapshot(source) or {}
            if not isinstance(snapshot, dict):
                raise TypeError("bridge snapshot is not an object")
        except Exception as exc:
            snapshot = {"error": str(exc)}
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
            "topic": self._source_topics[source], "message_type": self._source_types[source],
            "received": has_data,
            "received_at_ms": received_at, "age_s": age,
            "fresh": fresh, "stale": has_data and not fresh,
            "stamp": stamp, "error": snapshot.get("error"),
            "sequence": snapshot.get("sequence", 0),
        }
        return data, source_info, binary

    def _payload(self):
        values = {source: self._snapshot(source) for source in self.sources}
        source_info = {source: value[1] for source, value in values.items()}
        fresh = all(item["fresh"] for item in source_info.values())
        connected = any(item["fresh"] for item in source_info.values())
        received = any(item["received"] for item in source_info.values())
        payload = {
            "card": self.name, "read_only": True, "timestamp_ms": int(time.time() * 1000),
            "connected": connected, "fresh": fresh, "stale": received and not fresh,
            "status": "live" if fresh else "partial" if connected else "stale" if received else "no_data",
            "source": source_info,
            "data": ({source: value[0] for source, value in values.items()}
                     if self.name == "state" else values[self.sources[0]][0]),
        }
        if self.name == "state":
            payload["capabilities"] = {"motion": dict(MOTION_CAPABILITY)}
            payload["units"] = {"battery": {"voltage": None, "current": None, "capacity": None,
                                                "battery_level": None, "temperature": None},
                                "motion_info": None}
            payload["state_code_meaning"] = "unknown"
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
                    "last_error": status.get("last_error")}
        except Exception as exc:
            return {"reader_running": False, "last_error": str(exc)}

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
                          last_error=self._last_error, reader=self._reader_status())
            if self.name == "state":
                result["capabilities"] = {"motion": dict(MOTION_CAPABILITY)}
            return result

    @staticmethod
    def _overrides(options):
        overrides = dict(options or {})
        overrides.pop("_tool_name", None)
        overrides.pop("instance_id", None)
        if isinstance(overrides.get("config"), dict):
            overrides = {**overrides.pop("config"), **overrides}
        return overrides

    def configure(self, options):
        overrides = self._overrides(options)
        with self._lock:
            defaults = dict(self._defaults)
            defaults.update({key: value for key, value in overrides.items() if key in defaults})
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
                    self._bridge.acquire(self.name, list(self.sources), dict(settings))
                    self._lease_generation = generation
                    with self._lock:
                        canceled = generation != self._generation or self._state != "running"
                    if canceled:
                        self._bridge.release(self.name)
                        self._lease_generation = None
                except Exception as exc:
                    with self._lock:
                        if generation == self._generation:
                            self._generation += 1
                            self._cancel.set()
                            self._state, self._last_error = "error", str(exc)
                    try:
                        self._bridge.release(self.name)
                    except Exception:
                        pass
                    self._lease_generation = None
        return self.status()

    def start(self, options=None):
        overrides = self._overrides(options)
        settings = dict(self._defaults)
        settings.update({key: value for key, value in overrides.items() if key in settings})
        settings = self._validate(settings)
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
            self._cancel = cancel = threading.Event()
            self._last_binary_sequence = None
        with self._lease_lock:
            with self._lock:
                if generation != self._generation or cancel.is_set():
                    return self.status()
            try:
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
                self._bridge.release(self.name)
                self._lease_generation = None
            except Exception as exc:
                try:
                    self._bridge.release(self.name)
                except Exception:
                    pass
                self._lease_generation = None
                with self._lock:
                    if generation == self._generation:
                        self._state, self._last_error = "error", str(exc)
            return self.status()

    def _release_through(self, stopped_generation):
        with self._lease_lock:
            if self._lease_generation is not None and self._lease_generation <= stopped_generation:
                try:
                    self._bridge.release(self.name)
                except Exception as exc:
                    with self._lock:
                        self._last_error = str(exc)
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
                            self._last_error = str(exc)
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
            except Exception as exc:
                with self._lock:
                    if generation == self._generation:
                        self._last_error = str(exc)
            cancel.wait(1.0 / self._settings["refresh_hz"])

    def dispatch(self, action, args):
        if action == "start":
            return self.start(args)
        if action == "stop":
            return self.stop()
        if action == "config":
            return self.configure(args)
        if action in ("info", "status"):
            return self.status()
        return None


def build_plugins(config, namespace, bridge, publish):
    """Build exactly five idle sensors; acquiring a source requires start."""
    plugins = (config or {}).get("plugins", {})
    specs = source_specs(config or {})
    return [ReadOnlyCard(name, plugins.get(name, {}), namespace, bridge, publish,
                         source_topics={key: item["topic"] for key, item in specs.items()},
                         source_types={key: item["type"] for key, item in specs.items()}) for name in CARDS]
