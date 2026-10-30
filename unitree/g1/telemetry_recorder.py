"""Motus card: subscribe to existing JSON state streams, capture local clips.

ROS dependencies are imported only by the subscription adapter at start time.
No Unitree control client is constructed, and this module never publishes ROS.
"""
from __future__ import annotations
from collections import OrderedDict
import json
import re
import ssl
import threading
import time
import urllib.parse
import urllib.request

from telemetry_recorder_core import Recorder
from telemetry_recorder_store import ClipStore


def notify_completion(action_id, status, result):
    """Agent Core ACP contract; network errors are surfaced by the caller.

    The deployed Core uses a local self-signed certificate. Follow the existing
    G1 callback convention for loopback only; remote HTTPS retains verification.
    """
    import os as _os
    url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme not in ("http", "https") or not parsed.hostname:
        raise ValueError("AGENT_CORE_URL must be an HTTP(S) origin")
    if parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in ("", "/"):
        raise ValueError("AGENT_CORE_URL must be an origin without credentials, query or path")
    context = None
    if parsed.scheme == "https":
        context = ssl.create_default_context()
        if parsed.hostname.lower() in ("localhost", "127.0.0.1", "::1"):
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
    payload = json.dumps({"action_id": action_id, "status": status,
                          "result": result, "tool": "telemetry_recorder",
                          "ts": time.time()}, allow_nan=False).encode("utf-8")
    request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                     headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(request, timeout=5, context=context) as response:
        response.read(1024)


def create_ros_subscriber(sources, callback, executor):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    if not rclpy.ok():
        raise RuntimeError("rclpy is not initialized")
    node = Node("g1_telemetry_recorder")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=100,
                     durability=DurabilityPolicy.VOLATILE)
    try:
        for source, spec in sources.items():
            node.create_subscription(String, spec["topic"],
                                     lambda msg, key=source: callback(key, msg.data), qos)
        executor.add_node(node)
    except Exception:
        node.destroy_node()
        raise

    def close():
        try:
            executor.remove_node(node)
        finally:
            node.destroy_node()
    return close


class TelemetryRecorderPlugin:
    PREFIX = "telemetry_recorder"
    CONFIG_KEYS = frozenset({
        "enabled", "output_dir", "buffer_s", "pre_s", "post_s", "cooldown_s",
        "max_buffer_bytes", "max_samples", "max_message_bytes", "max_clip_bytes",
        "max_pending", "max_disk_bytes", "max_clips", "auto_events",
        "imu_topic", "joints_topic", "battery_topic", "motion_events_topic",
        "imu_stale_s", "joints_stale_s", "battery_stale_s",
    })

    def __init__(self, plugin_config, namespace, executor, *,
                 subscriber_factory=create_ros_subscriber, completion_notifier=notify_completion):
        self._cfg = dict(plugin_config)
        self._ns, self._executor = namespace.strip("/"), executor
        self._factory = subscriber_factory
        self._notifier = completion_notifier
        self._life_lock = threading.RLock()
        self._result_lock = threading.Lock()
        self._thread = None
        self._close_subscriber = None
        self._stop_event = threading.Event()
        self._results = OrderedDict()
        self._actions = {}
        self._last_error = None
        self._last_completion_error = None
        self._completion_failures = 0
        self._state = "idle"
        self._build()

    def _build(self):
        unknown = set(self._cfg) - self.CONFIG_KEYS
        if unknown:
            raise ValueError("unknown recorder config: " + ", ".join(sorted(unknown)))
        topics = {
            "imu": f"/{self._ns}/state/imu", "joints": f"/{self._ns}/state/joints",
            "battery": f"/{self._ns}/state/battery",
            "motion_events": f"/{self._ns}/safety/motion_events",
        }
        sources = {}
        for name, default in topics.items():
            topic = self._cfg.get(name + "_topic", default)
            if not isinstance(topic, str) or not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*", topic):
                raise ValueError(f"invalid absolute ROS topic for {name}")
            sources[name] = {"topic": topic, "event_stream": name == "motion_events"}
            if name != "motion_events":
                sources[name]["stale_s"] = self._cfg.get(name + "_stale_s", {"imu": .5, "joints": .5, "battery": 3}[name])
        if len(set(s["topic"] for s in sources.values())) != len(sources):
            raise ValueError("source topics must be distinct")
        auto = self._cfg.get("auto_events", [])
        if not isinstance(auto, list) or len(auto) > 16 or any(
                not isinstance(x, str) or not re.fullmatch(r"[a-z_]{1,64}", x) for x in auto):
            raise ValueError("auto_events must be a list of event type names")
        keys = ("buffer_s", "pre_s", "post_s", "cooldown_s", "max_buffer_bytes",
                "max_samples", "max_message_bytes", "max_clip_bytes", "max_pending")
        self.core = Recorder(sources, auto_events=auto, **{k: self._cfg[k] for k in keys if k in self._cfg})
        self.store = ClipStore(self._cfg.get("output_dir", "/opt/phanthy-motus/data/telemetry_recorder"),
                               self._cfg.get("max_disk_bytes", 64_000_000), self._cfg.get("max_clips", 100))

    def get_tool(self):
        actions = {
            "capture": {"params": ["label", "pre_s", "post_s"],
                        "description": "Save telemetry before/after a marker. Returns action_id immediately; Agent Core receives completion after saving, failure or interruption."},
            "status": {"params": [], "description": "Check actual source receive ages, missing streams, pending clips and storage errors."},
            "list_clips": {"params": ["limit"], "description": "List local saved clip metadata; no upload or file download."},
            "get_clip": {"params": ["clip_id"], "description": "Check capture/save state and coverage; returns local file paths after saving."},
        }
        config = {
            "output_dir": {"type": "string", "description": "Dedicated persistent directory on the driver host; confirm bind mount with master."},
            "buffer_s": {"type": "number", "minimum": 1, "maximum": 120},
            "pre_s": {"type": "number", "minimum": 0, "maximum": 120},
            "post_s": {"type": "number", "minimum": 0, "maximum": 30},
            "max_disk_bytes": {"type": "integer", "minimum": 1024, "maximum": 1000000000},
            "auto_events": {"type": "array", "items": {"type": "string"},
                            "description": "Opt-in SmartMotion event types; empty disables automatic capture."},
        }
        config.update({name + "_topic": {"type": "string", "default": spec["topic"]}
                       for name, spec in self.core.sources.items()})
        return {"name": self.PREFIX, "type": "actuator", "multiInstance": False,
                "description": "G1 read-only telemetry recorder and event review. Local file operations only; never commands robot motion. Readiness does not prove data health.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": list(actions) + ["start", "stop", "info"]},
                    "label": {"type": "string", "maxLength": 120, "default": "manual"},
                    "pre_s": {"type": "number", "minimum": 0, "maximum": self.core.buffer_s},
                    "post_s": {"type": "number", "minimum": 0, "maximum": 30},
                    "limit": {"type": "integer", "minimum": 1, "maximum": 20, "default": 10},
                    "clip_id": {"type": "string", "pattern": "^[0-9a-f]{32}$"},
                }, "required": ["action"], "x-action-params": actions,
                    "x-completion": {"actions": ["capture"], "timeout": 90},
                    "x-resource": "telemetry_storage"},
                "configSchema": {"type": "object", "properties": config}}

    def _remember(self, result):
        with self._result_lock:
            # A zero-post clip may finish on the writer before capture returns.
            # Never let its later "capturing" acknowledgement erase completion.
            rank = {"capturing": 0, "writing": 1, "saved": 2, "failed": 2}
            previous = self._results.get(result["clip_id"])
            if previous and rank[previous["status"]] > rank[result["status"]]:
                return
            action_id = self._actions.get(result["clip_id"])
            if action_id:
                result = {**result, "action_id": action_id}
            self._results[result["clip_id"]] = result
            self._results.move_to_end(result["clip_id"])
            while len(self._results) > 50:
                self._results.popitem(last=False)

    def _complete(self, clip_id, status, result):
        # Claim once, before network I/O. Automatic event clips have no ACP
        # caller and no entry here, so never emit a synthetic completion.
        with self._result_lock:
            action_id = self._actions.pop(clip_id, None)
        if action_id is None:
            return
        try:
            self._notifier(action_id, status, result)
        except Exception as exc:
            self._last_completion_error = str(exc)[:500]
            self._completion_failures += 1

    def _save(self, docs):
        for doc in docs:
            self._remember({"clip_id": doc["clip_id"], "status": "writing"})
            try:
                result = self.store.write(doc)
            except Exception as exc:
                self._last_error = str(exc)[:500]
                result = {"clip_id": doc["clip_id"], "status": "failed", "error": self._last_error}
            self._remember(result)
            status = ("error" if result["status"] == "failed" or self._state == "error" else
                      "cancelled" if doc["interrupted"] else "completed")
            self._complete(doc["clip_id"], status, result)

    def _worker(self):
        try:
            while not self._stop_event.wait(.05):
                self._save(self.core.drain_ready())
        except Exception as exc:
            self._last_error = str(exc)[:500]
            self._state = "error"
        finally:
            try:
                self._save(self.core.stop())
            except Exception as exc:
                self._last_error = str(exc)[:500]
                self._state = "error"
            finally:
                # Even an unexpected export failure must not leave a caller's
                # action pending forever. The Core's 90s timeout remains the
                # fallback for failed network delivery or a terminated process.
                with self._result_lock:
                    unresolved = list(self._actions)
                for clip_id in unresolved:
                    result = {"clip_id": clip_id, "status": "failed",
                              "error": self._last_error or "recorder stopped before exporting clip"}
                    self._remember(result)
                    self._complete(clip_id, "error", result)

    def start(self):
        with self._life_lock:
            if self._thread is not None and self._thread.is_alive():
                if self._stop_event.is_set():
                    return {"state": "error", "error": "previous writer is still stopping"}
                return self.info()
            self._last_error = None
            try:
                if self._close_subscriber:
                    self._close_subscriber()
                    self._close_subscriber = None
                self.store.prepare()
                self.core.start()
                self._close_subscriber = self._factory(self.core.sources, self.core.ingest_json, self._executor)
                self._stop_event.clear()
                self._thread = threading.Thread(target=self._worker, name="g1-recorder-writer", daemon=True)
                self._thread.start()
                self._state = "ready"
            except Exception as exc:
                self.core.stop()
                if self._close_subscriber:
                    try:
                        self._close_subscriber()
                    except Exception as close_exc:
                        self._last_error = str(close_exc)[:500]
                    self._close_subscriber = None
                self._state = "error"
                self._last_error = str(exc)[:500]
                return {"state": "error", "error": self._last_error}
            return self.info()

    def stop(self):
        with self._life_lock:
            self._stop_event.set()
            close_error = None
            if self._close_subscriber:
                try:
                    self._close_subscriber()
                except Exception as exc:
                    close_error = str(exc)[:500]
                self._close_subscriber = None
            if self._thread is not None:
                self._thread.join(timeout=3)
                if self._thread.is_alive():
                    self._state = "stopping"
                    return {"state": "error", "error": "writer is still stopping; restart refused"}
            self._state = "idle"
            return {"state": "idle", "last_storage_or_worker_error": self._last_error,
                    **({"error": close_error} if close_error else {})}

    def info(self):
        return {"state": self._state, **self.core.status(),
                "last_storage_or_worker_error": self._last_error,
                "last_completion_error": self._last_completion_error,
                "completion_delivery_failures": self._completion_failures,
                "output_dir": str(self.store.root),
                "data_health_is_separate_from_lifecycle": True}

    def dispatch(self, action, args):
        try:
            if action == "start":
                return self.start()
            if action == "stop":
                return self.stop()
            if action in ("info", "status"):
                return self.info()
            if action == "config":
                with self._life_lock:
                    if self._thread is not None and self._thread.is_alive():
                        raise ValueError("stop recorder before reconfiguring")
                    new = {k: v for k, v in args.items() if k not in ("_tool_name", "action")}
                    old = self._cfg
                    self._cfg = {**old, **new}
                    try:
                        self._build()
                    except Exception:
                        self._cfg = old
                        self._build()
                        raise
                    return {"state": "idle", "configured": True}
            if action == "capture":
                with self._life_lock:
                    if self._stop_event.is_set():
                        raise ValueError("recorder is stopping or stopped")
                    # _save first takes this lock through _remember. Register
                    # correlation before the writer can finalize a zero-post
                    # clip, including a callback preceding dispatch's return.
                    with self._result_lock:
                        result = self.core.capture(args.get("label", "manual"), args.get("pre_s"), args.get("post_s"))
                        action_id = "telemetry_capture_" + result["clip_id"]
                        self._actions[result["clip_id"]] = action_id
                        result = {**result, "action_id": action_id, "state": "capturing"}
                    self._remember(result)
                    return result
            if action == "list_clips":
                return {"clips": self.store.list(args.get("limit", 10))}
            if action == "get_clip":
                key = args.get("clip_id")
                for pending in self.core.status()["pending"]:
                    if pending["clip_id"] == key:
                        return {**pending, "status": "capturing"}
                with self._result_lock:
                    result = self._results.get(key)
                return dict(result) if result is not None else self.store.get(key)
            return {"error": f"unknown action: {action}"}
        except (ValueError, OSError, TypeError, KeyError) as exc:
            return {"error": str(exc)[:500]}
