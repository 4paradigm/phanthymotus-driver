#!/usr/bin/env python3
"""Read-only S10 Pro ROS 2 adapter, isolated from the platform DDS process.

The parent supplies JSON control records on stdin.  Only subscriptions are
created; this module never imports a vendor controller or calls a ROS service.
stdout is an NDJSON data channel.  A bounded writer retains only the newest
pending record for each source, so a slow parent cannot grow a stale queue.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import queue
import signal
import sys
import threading
import time
from collections import OrderedDict
from collections.abc import Mapping
from pathlib import Path

try:
    from .pointcloud import convert_pointcloud
except ImportError:  # Direct execution inside the independent reader process.
    from pointcloud import convert_pointcloud


DEFAULT_SOURCES = {
    "battery": {"topic": "/BATTERY_DATA", "type": "drdds/msg/BatteryData"},
    "motion_info": {"topic": "/MOTION_INFO", "type": "drdds/msg/MotionInfo"},
    "joints": {"topic": "/JOINTS_DATA_10HZ", "type": "drdds/msg/JointsData"},
    "imu": {"topic": "/IMU", "type": "sensor_msgs/msg/Imu"},
    "odometry": {"topic": "/ODOM", "type": "nav_msgs/msg/Odometry"},
    "lidar": {"topic": "/LIDAR/POINTS", "type": "sensor_msgs/msg/PointCloud2"},
}
DEFAULT_RATES_HZ = {
    "battery": 2.0, "motion_info": 5.0, "joints": 5.0,
    "imu": 5.0, "odometry": 5.0, "lidar": 3.0,
}
MAX_CONTROL_BYTES = 16 * 1024
MAX_POINTS = 40000  # The existing canvas renderer's maximum display size.


def log(message: str) -> None:
    print(f"[s10-reader] {message}", file=sys.stderr, flush=True)


def to_json_safe(value, depth: int = 0):
    """Convert ROS scalar/array/message fields without depending on ROS imports."""
    if depth > 32:
        raise ValueError("message nesting exceeds 32 levels")
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {str(k): to_json_safe(v, depth + 1) for k, v in value.items()}
    if isinstance(value, (list, tuple, bytes, bytearray, memoryview)):
        return [to_json_safe(v, depth + 1) for v in value]
    if hasattr(value, "tolist"):
        return to_json_safe(value.tolist(), depth + 1)
    if hasattr(value, "get_fields_and_field_types"):
        return {
            name: to_json_safe(getattr(value, name), depth + 1)
            for name in value.get_fields_and_field_types()
        }
    slots = getattr(value, "__slots__", None)
    if slots is not None:
        return {
            str(name).lstrip("_"): to_json_safe(getattr(value, name), depth + 1)
            for name in slots if hasattr(value, name)
        }
    raise TypeError(f"unsupported message value type: {type(value).__name__}")


def load_config(path: str | Path) -> dict:
    """Load declarative configuration; no shell setup scripts are sourced."""
    import yaml

    with Path(path).open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if config is None:
        return {}
    if not isinstance(config, dict):
        raise ValueError("config must be a mapping")
    return config


def source_specs(config: dict) -> dict:
    """Resolve fixed read-only sources, with optional declarative topic overrides."""
    topics = config.get("topics", {})
    message_types = config.get("message_types", {})
    if not isinstance(topics, dict) or not isinstance(message_types, dict):
        raise ValueError("topics and message_types must be mappings")
    result = {}
    for key, defaults in DEFAULT_SOURCES.items():
        entry = topics.get(key, defaults["topic"])
        if isinstance(entry, str):
            item = {"topic": entry, "type": message_types.get(key, defaults["type"])}
        elif isinstance(entry, dict):
            item = {
                "topic": entry.get("topic", defaults["topic"]),
                "type": entry.get("type", message_types.get(key, defaults["type"])),
            }
        else:
            raise ValueError(f"invalid topic configuration for {key}")
        if not isinstance(item["topic"], str) or not item["topic"].startswith("/"):
            raise ValueError(f"{key} topic must be an absolute ROS topic")
        if not isinstance(item["type"], str) or "/msg/" not in item["type"]:
            raise ValueError(f"{key} type must be a ROS message type")
        result[key] = item
    return result


def normalize_control(command: dict, *, previous: dict | None = None) -> dict:
    """Validate a data-only control record before it can change subscriptions."""
    if not isinstance(command, dict):
        raise ValueError("control record must be a JSON object")
    if set(command) - {"active", "rates_hz", "max_points"}:
        raise ValueError("unsupported control fields")
    active = command.get("active")
    if not isinstance(active, list) or any(not isinstance(k, str) for k in active):
        raise ValueError("active must be a list of source names")
    if len(active) != len(set(active)) or set(active) - set(DEFAULT_SOURCES):
        raise ValueError("active contains duplicate or unsupported source names")
    previous = previous or {}
    rates = dict(previous.get("rates_hz", DEFAULT_RATES_HZ))
    overrides = command.get("rates_hz", {})
    if not isinstance(overrides, dict) or set(overrides) - set(DEFAULT_SOURCES):
        raise ValueError("rates_hz must contain supported source names")
    for key, rate in overrides.items():
        if isinstance(rate, bool) or not isinstance(rate, (float, int)):
            raise ValueError(f"invalid publish rate for {key}")
        if not math.isfinite(rate) or not 0 < rate <= 200:
            raise ValueError(f"publish rate for {key} must be between 0 and 200 Hz")
        rates[key] = float(rate)
    max_points = command.get("max_points", previous.get("max_points", 6000))
    if isinstance(max_points, bool) or not isinstance(max_points, int) or not 1 <= max_points <= MAX_POINTS:
        raise ValueError(f"max_points must be an integer between 1 and {MAX_POINTS}")
    return {"active": list(active), "rates_hz": rates, "max_points": max_points}


class LatestEventWriter:
    """One pending event per known key, plus at most one in-flight line.

    A line already being written cannot be retracted.  The parent must also gate
    by its active sources and check received_monotonic after a delayed read.
    """

    def __init__(self, stream, *, logger=log, autostart: bool = True):
        self._stream = stream
        self._logger = logger
        self._condition = threading.Condition()
        self._pending: OrderedDict[str, dict] = OrderedDict()
        self._closed = False
        self.failed = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="s10-reader-output")
        if autostart:
            self.start()

    def start(self):
        self._thread.start()

    @property
    def pending_count(self) -> int:
        with self._condition:
            return len(self._pending)

    def submit(self, event: dict) -> bool:
        key = event.get("key")
        if key not in DEFAULT_SOURCES:
            raise ValueError("unsupported output source")
        with self._condition:
            if self._closed or self.failed.is_set():
                return False
            # Replacing does not move the key: a high-rate source cannot starve
            # pending battery or health data by repeatedly jumping the queue.
            self._pending[key] = event
            self._condition.notify()
        return True

    def discard(self, keys):
        if isinstance(keys, str):
            keys = [keys]
        with self._condition:
            for key in keys:
                self._pending.pop(key, None)

    def close(self, timeout: float = 1.0):
        with self._condition:
            self._closed = True
            self._pending.clear()
            self._condition.notify_all()
        if self._thread.ident is not None:
            self._thread.join(timeout)

    def _run(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._closed or self._pending)
                    if self._closed:
                        return
                    _, event = self._pending.popitem(last=False)
                line = json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
                self._stream.write(line + "\n")
                self._stream.flush()
        except Exception as exc:
            self.failed.set()
            self._logger(f"output closed ({type(exc).__name__})")


class SubscriptionReader:
    """Single-threaded subscription owner; dependencies are injected for tests."""

    def __init__(self, node, specs, message_getter, emit, qos, *, purge=None,
                 monotonic=time.monotonic, wall_time=time.time, logger=log):
        self.node = node
        self.specs = specs
        self._message_getter = message_getter
        self._emit = emit
        self._purge = purge or (lambda keys: None)
        self._qos = qos
        self._monotonic = monotonic
        self._wall_time = wall_time
        self._logger = logger
        self.control = normalize_control({"active": []})
        self.subscriptions = {}
        self._generations = {key: 0 for key in DEFAULT_SOURCES}
        self._last_emitted = {}
        self._last_attempt = {}
        self._last_error = {}
        self.closed = False

    def apply_control(self, command: dict):
        normalized = normalize_control(command, previous=self.control)
        old = set(self.control["active"])
        new = set(normalized["active"])
        self.control = normalized
        for key in old - new:
            self._generations[key] += 1
            subscription = self.subscriptions.pop(key, None)
            if subscription is not None:
                self.node.destroy_subscription(subscription)
            self._last_emitted.pop(key, None)
            self._last_attempt.pop(key, None)
        self._purge(old - new)
        self.reconcile()

    def reconcile(self):
        if self.closed:
            return
        now = self._monotonic()
        for key in self.control["active"]:
            if key in self.subscriptions:
                continue
            if now - self._last_attempt.get(key, -math.inf) < 2.0:
                continue
            self._last_attempt[key] = now
            self._generations[key] += 1
            generation = self._generations[key]
            try:
                spec = self.specs[key]
                message_type = self._message_getter(spec["type"])
                subscription = self.node.create_subscription(
                    message_type, spec["topic"],
                    lambda message, k=key, g=generation: self._on_message(k, g, message),
                    self._qos,
                )
                self.subscriptions[key] = subscription
            except Exception as exc:
                # Retry missing types without taking unrelated cards offline.
                self._report_error(key, "subscription", exc)

    def _report_error(self, key, operation, exc):
        now = self._monotonic()
        token = (key, operation)
        if now - self._last_error.get(token, -math.inf) >= 5.0:
            # Avoid message bodies and arbitrary exception strings in logs.
            self._logger(f"{key} {operation} failed ({type(exc).__name__})")
            self._last_error[token] = now

    def _on_message(self, key, generation, message):
        if self.closed or key not in self.control["active"] or generation != self._generations[key]:
            return
        received = self._monotonic()
        rate = self.control["rates_hz"][key]
        if received - self._last_emitted.get(key, -math.inf) < 1.0 / rate:
            return
        received_at_ms = int(self._wall_time() * 1000)
        try:
            event = {"key": key, "received_monotonic": received, "received_at_ms": received_at_ms}
            if key == "lidar":
                binary, metadata = convert_pointcloud(message, max_points=self.control["max_points"])
                event["data"] = to_json_safe(metadata)
                event["binary_base64"] = base64.b64encode(binary).decode("ascii")
            else:
                event["data"] = to_json_safe(message)
            if self._emit(event) is not False:
                self._last_emitted[key] = received
        except Exception as exc:
            self._report_error(key, "decode", exc)

    def close(self):
        if self.closed:
            return
        self.closed = True
        for subscription in list(self.subscriptions.values()):
            self.node.destroy_subscription(subscription)
        self.subscriptions.clear()
        self._purge(set(DEFAULT_SOURCES))


def read_controls(stream, latest: queue.Queue, eof: threading.Event, *, logger=log):
    """Read bounded data records; the ROS thread applies only the latest record."""
    previous = None
    try:
        while True:
            line = stream.readline(MAX_CONTROL_BYTES + 1)
            if not line:
                break
            if len(line) > MAX_CONTROL_BYTES:
                while line and not line.endswith("\n"):
                    line = stream.readline(MAX_CONTROL_BYTES + 1)
                logger("control record rejected (too long)")
                continue
            try:
                command = normalize_control(json.loads(line), previous=previous)
            except (ValueError, TypeError, OverflowError, RecursionError):
                logger("control record rejected (invalid schema)")
                continue
            previous = command
            while True:
                try:
                    latest.put_nowait(command)
                    break
                except queue.Full:
                    try:
                        latest.get_nowait()
                    except queue.Empty:
                        pass
    except Exception as exc:
        logger(f"control input closed ({type(exc).__name__})")
    finally:
        eof.set()


def import_ros(specs, *, validate_messages=False):
    """Imports only.  In particular, --self-check never creates a participant."""
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    if validate_messages:
        for spec in specs.values():
            get_message(spec["type"])
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
    return rclpy, get_message, qos


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.yaml"))
    parser.add_argument("--self-check", action="store_true", help="check imports without starting ROS")
    args = parser.parse_args(argv)
    try:
        specs = source_specs(load_config(args.config))
        if args.self_check:
            import_ros(specs, validate_messages=True)
            print(json.dumps({"self_check": "ok", "topics": specs}, ensure_ascii=False))
            return 0
    except Exception as exc:
        log(f"configuration/import check failed ({type(exc).__name__})")
        return 1

    stopping = threading.Event()
    eof = threading.Event()
    controls = queue.Queue(maxsize=1)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda _signum, _frame: stopping.set())
    threading.Thread(target=read_controls, args=(sys.stdin, controls, eof),
                     daemon=True, name="s10-reader-control").start()
    initial = None
    while not stopping.is_set():
        try:
            initial = controls.get(timeout=0.1)
            break
        except queue.Empty:
            if eof.is_set():
                return 0
    if initial is None:
        return 0

    # Preserve a private data fd, then route any ROS/library console output to
    # stderr.  No log line from a native dependency can corrupt the NDJSON pipe.
    output = os.fdopen(os.dup(sys.stdout.fileno()), "w", encoding="utf-8", buffering=1)
    sys.stdout.flush()
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    writer = LatestEventWriter(output)
    rclpy = node = reader = None
    initialized = False
    try:
        rclpy, get_message, qos = import_ros(specs)
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node("s10_pro_readonly_reader", enable_rosout=False,
                                 start_parameter_services=False)
        reader = SubscriptionReader(node, specs, get_message, writer.submit, qos, purge=writer.discard)
        reader.apply_control(initial)
        while not stopping.is_set() and not writer.failed.is_set() and rclpy.ok():
            try:
                reader.apply_control(controls.get_nowait())
            except queue.Empty:
                pass
            if eof.is_set() and controls.empty():
                break  # The parent closed stdin; do not leave an orphan reader.
            reader.reconcile()
            rclpy.spin_once(node, timeout_sec=0.1)
        return 1 if writer.failed.is_set() else 0
    except Exception as exc:
        log(f"reader stopped ({type(exc).__name__})")
        return 1
    finally:
        if reader is not None:
            reader.close()
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.try_shutdown()
        writer.close()
        # A blocked daemon writer owns this fd until process exit; closing it
        # here could race a partial JSON line.  Process teardown closes the fd.


if __name__ == "__main__":
    raise SystemExit(main())
