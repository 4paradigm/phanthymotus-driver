"""ImageQualityGuard's Bumi processor and strict system lifecycle protocol.

The default transport subscribes to the existing RGB stream on the bundle's
shared ROS2 executor. Construction and bundle boot do not acquire resources.
The internal runtime seam is NOT an MCP parameter or a new framework interface:

    runtime_factory(namespace=..., executor=...) -> inert runtime
    runtime.start(input_topic=..., output_topic=..., run_id=...,
                  analysis_hz=..., run_if_current=...)
    runtime.close()

The factory must not allocate resources before returning. The plugin records
ownership before start, and close must reclaim partial starts and be retryable.
A runtime callback uses run_if_current(operation), which executes analysis
and publication only for the current running generation. External start/stop
require a Core-issued ticket; local process shutdown uses a separate method.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict
import re
import threading
from uuid import UUID, uuid4

from image_quality import ImageQualityAnalyzer, QualityConfig


_TOPIC_PATTERN = r"/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*"
_CONTEXT_KEYS = {"instance_id", "_tool_name", "_trace_id", "camera_info", "input_bindings"}
_INPUT_KEYS = {"input_topic", "input_topics"}
_TERMINAL = {"closed", "cancelled", "failed"}
LIFECYCLE = {"schema": "motus.lifecycle/1", "id_parameter": "run_id",
             "required_for": ["start", "stop"]}


def _uuid(value):
    if not isinstance(value, str):
        raise ValueError("UUID must be a canonical lowercase string")
    try:
        valid = str(UUID(value)) == value
    except (ValueError, AttributeError):
        valid = False
    if not valid:
        raise ValueError("UUID must be canonical lowercase")
    return value


def _run_epoch(value):
    if value is None or value == "":
        raise ValueError("MISSING_RUN_ID")
    if not isinstance(value, str) or len(value) != 73 or value.count(".") != 1:
        raise ValueError("INVALID_RUN_ID")
    try:
        epoch, run = value.split(".")
        _uuid(epoch)
        _uuid(run)
    except ValueError as exc:
        raise ValueError("INVALID_RUN_ID") from exc
    return epoch


def _runtime_factory(**kwargs):
    from image_quality_runtime import ImageQualityRosRuntime
    return ImageQualityRosRuntime(**kwargs)


def _config_schema():
    defaults = asdict(QualityConfig())
    integer_ranges = {"dark_pixel_threshold": (0, 254), "bright_pixel_threshold": (1, 255),
                      "confirm_samples": (1, 1000), "recover_samples": (1, 1000)}
    titles = {
        "dark_pixel_threshold": "暗像素灰度上限", "bright_pixel_threshold": "亮像素灰度下限",
        "low_light_mean_threshold": "低光灰度均值上限", "low_light_ratio_threshold": "低光暗像素比例下限",
        "overexposed_mean_threshold": "过曝灰度均值下限", "overexposed_ratio_threshold": "过曝亮像素比例下限",
        "blur_variance_threshold": "疑似模糊的拉普拉斯方差上限", "analysis_hz": "分析频率（Hz）",
        "confirm_samples": "异常连续确认次数", "recover_samples": "恢复连续确认次数",
    }
    properties = {}
    for name, default in defaults.items():
        item = {"type": "integer" if name in integer_ranges else "number",
                "title": titles[name], "description": titles[name] + "；工程初值，未经真机标定",
                "default": default, "scope": "shared"}
        if name in integer_ranges:
            item.update(minimum=integer_ranges[name][0], maximum=integer_ranges[name][1])
        elif name.endswith("ratio_threshold"):
            item.update(exclusiveMinimum=0, maximum=1)
        elif name.endswith("mean_threshold"):
            item.update(minimum=0, maximum=255)
        elif name == "analysis_hz":
            item.update(exclusiveMinimum=0, maximum=30)
        else:
            item.update(minimum=0)
        properties[name] = item
    return {"type": "object", "properties": properties, "additionalProperties": False}


class ImageQualityGuardPlugin:
    PREFIX = "image_quality_guard"

    def __init__(self, plugin_config: dict | None, namespace: str, executor, *,
                 runtime_factory=None, driver_boot_id=None, history_capacity=4096):
        if plugin_config is not None and not isinstance(plugin_config, Mapping):
            raise ValueError("plugin_config must be a mapping")
        config = dict(plugin_config or {})
        if "enabled" in config and type(config.pop("enabled")) is not bool:
            raise ValueError("enabled must be a boolean")
        if not isinstance(namespace, str):
            raise ValueError("namespace must be a string")
        if runtime_factory is not None and not callable(runtime_factory):
            raise ValueError("runtime_factory must be callable")
        self._analyzer = ImageQualityAnalyzer(config)
        self._namespace, self._executor = namespace, executor
        if type(history_capacity) is not int or history_capacity < 1:
            raise ValueError("history_capacity must be a positive integer")
        self._runtime_factory = runtime_factory or _runtime_factory
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.Lock()
        self._generation = 0
        self._runtime = None
        self._state = "idle"
        self._input_topic = ""
        self._instance_id = None
        self._last_error = None
        self._driver_boot_id = _uuid(driver_boot_id) if driver_boot_id else str(uuid4())
        self._epoch = None
        self._last_bind = None
        self._run_id = None
        self._history = {}
        self._history_capacity = history_capacity
        self._shutdown = False

    def get_tool(self) -> dict:
        config = _config_schema()
        properties = {
            "action": {"type": "string", "enum": ["start", "stop", "info", "config"]},
            "run_id": {"type": "string", "pattern": "^[0-9a-f-]{36}\\.[0-9a-f-]{36}$",
                       "description": "Core 签发的系统运行票据；不是用户配置或 LLM 参数"},
            "input_topic": {"type": "string", "pattern": "^" + _TOPIC_PATTERN + "$",
                            "description": "画布连接的唯一 RGB JPEG Topic"},
            "input_topics": {"type": "array", "items": {
                "type": "string", "pattern": "^" + _TOPIC_PATTERN + "$"},
                             "description": "框架输入列表；仅接受同一路输入"},
        }
        properties.update({name: {k: v for k, v in item.items() if k != "scope"}
                           for name, item in config["properties"].items()})
        return {
            "name": self.PREFIX, "type": "processor", "multiInstance": False,
            "x-lifecycle": {**LIFECYCLE, "required_for": list(LIFECYCLE["required_for"])},
            "description": "分析现有 RGB JPEG 输入的低光、过曝和疑似模糊，输出多帧确认的质量状态。",
            "inputSchema": {"type": "object", "properties": properties, "required": ["action"],
                            "x-action-params": {
                                "start": {"params": ["input_topic"], "description": "启动已连接的图像质量监测"},
                                "stop": {"params": [], "description": "停止本卡片并释放资源"},
                                "info": {"params": ["input_topic"], "description": "查询状态或预览连线后的输出 Topic"},
                                "config": {"params": list(config["properties"]),
                                           "description": "停止后更新图像质量配置"}}},
            "configSchema": config,
            "topic_in": [{"format": "image/jpeg", "desc": "现有 RGB JPEG 图像流"}],
            "topic_out": [{"format": "data/json", "desc": "motus.image_quality/1 图像质量结果"}],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self) -> None:
        """Bundle boot hook: wait for the canvas to supply an input topic."""

    def stop(self, run_id=None) -> dict:
        return self._stop(run_id)

    @staticmethod
    def _resolve_input(args, *, required=False):
        topics = []
        if "input_topic" in args and args["input_topic"] is not None:
            topics.append(args["input_topic"])
        if "input_topics" in args:
            if not isinstance(args["input_topics"], list):
                raise ValueError("input_topics must be a list")
            topics.extend(args["input_topics"])
        unique = []
        for topic in topics:
            if not isinstance(topic, str) or not re.fullmatch(_TOPIC_PATTERN, topic):
                raise ValueError("input must be an absolute Topic with nonempty alphanumeric/underscore tokens")
            if topic not in unique:
                unique.append(topic)
        if len(unique) > 1:
            raise ValueError("exactly one input Topic is supported")
        if required and not unique:
            raise ValueError("missing input_topic; connect the RGB JPEG output on the canvas")
        return unique[0] if unique else ""

    def _failure(self, code, message, requested=None):
        # An invalid request must not turn a healthy running instance into error.
        with self._lock:
            return {"state": "error", "code": code, "error": message,
                    "lifecycle_state": self._state, "requested_run_id": requested,
                    "active_run_id": self._run_id}

    def _info_locked(self, preview="", requested=None):
        topic = self._input_topic or preview
        output = f"{topic}/{self.PREFIX}" if topic else ""
        quality = self._analyzer.snapshot()
        result = {"state": self._state, "input_topic": topic, "output_topic": output,
                "topic_in": [{"topic": topic, "format": "image/jpeg"}] if topic else [],
                "topic_out": [{"topic": output, "format": "data/json"}] if output else [],
                "instance_id": self._instance_id, "config": asdict(self._analyzer.config),
                "quality_state": quality["state"], "quality": quality,
                "runtime_configured": self._runtime_factory is not None,
                "resources_pending": self._runtime is not None, "error": self._last_error,
                "pending_operations": self._state in {"starting", "stopping"},
                "driver_boot_id": self._driver_boot_id, "admission_epoch": self._epoch,
                "active_run_id": self._run_id, "run_id": self._run_id,
                "history_count": len(self._history), "history_capacity": self._history_capacity,
                "shutdown": self._shutdown}
        if requested is not None:
            result.update(requested_run_id=requested,
                          requested_run_state=self._history.get(requested, {}).get("state", "unknown"))
        return result

    def lifecycle_status(self):
        with self._lock:
            return self._info_locked()

    def bind_epoch(self, *, driver_boot_id, expected_epoch, proposed_epoch):
        try:
            _uuid(driver_boot_id)
            _uuid(proposed_epoch)
            if expected_epoch is not None:
                _uuid(expected_epoch)
        except ValueError as exc:
            return self._failure("INVALID_ARGUMENT", str(exc))
        with self._lifecycle_lock:
            with self._lock:
                if self._shutdown:
                    return self._failure("SHUTDOWN", "driver is shutting down")
                if driver_boot_id != self._driver_boot_id:
                    return self._failure("DRIVER_BOOT_MISMATCH", "read status for the current driver boot")
                request = (expected_epoch, proposed_epoch)
                if self._epoch == proposed_epoch and self._last_bind == request:
                    return self._info_locked()
                if self._epoch != expected_epoch:
                    return self._failure("EPOCH_MISMATCH", "binding was superseded")
                if proposed_epoch == self._epoch:
                    return self._failure("INVALID_ARGUMENT", "epoch must be fresh")
                if (self._run_id is not None or self._runtime is not None
                        or self._state in {"starting", "stopping"}):
                    return self._failure("BUSY", "all resources must be confirmed stopped before rotation")
                self._epoch = proposed_epoch
                self._last_bind = request
                self._history.clear()
                self._generation += 1
                self._state, self._last_error = "idle", None
                self._input_topic, self._instance_id = "", None
                return self._info_locked()

    def _validate_run_locked(self, run_id):
        try:
            epoch = _run_epoch(run_id)
        except ValueError as exc:
            return self._failure(str(exc), str(exc), run_id)
        if self._epoch is None:
            return self._failure("ADMISSION_REQUIRED", "bind a Core session first", run_id)
        if epoch != self._epoch:
            return self._failure("STALE_RUN", "admission session is no longer current", run_id)
        return None

    def _record_locked(self, run_id, state, topic="", instance_id=None):
        if run_id not in self._history and len(self._history) >= self._history_capacity:
            return self._failure("RUN_HISTORY_FULL", "rotate idle session; history is never evicted", run_id)
        self._history[run_id] = {"state": state, "input_topic": topic, "instance_id": instance_id}
        return None

    def info(self, input_topic: str | None = None, run_id=None) -> dict:
        try:
            preview = self._resolve_input({"input_topic": input_topic})
        except ValueError as exc:
            return self._failure("INVALID_ARGUMENT", str(exc))
        with self._lock:
            result = self._info_locked(preview, run_id)
            runtime = self._runtime
        if runtime is not None and callable(getattr(runtime, "info", None)):
            try:
                result["runtime"] = runtime.info()
            except Exception as exc:
                result["runtime"] = {"error": str(exc)[:400]}
        return result

    def dispatch(self, action: str, args: dict | None = None) -> dict | None:
        if not isinstance(action, str) or action not in {"start", "stop", "info", "config", self.PREFIX}:
            return None
        if args is not None and not isinstance(args, Mapping):
            return self._failure("INVALID_ARGUMENT", "arguments must be a mapping")
        args = dict(args or {})
        allowed = _CONTEXT_KEYS | (_INPUT_KEYS | {"run_id"}
                    if action in {"start", "stop", "info", self.PREFIX} else set())
        if action == "config":
            allowed |= set(asdict(self._analyzer.config))
        unknown = set(args) - allowed
        if unknown:
            return self._failure("INVALID_ARGUMENT", "unknown arguments: " + ", ".join(sorted(map(str, unknown))))
        if args.get("instance_id") is not None and not isinstance(args["instance_id"], str):
            return self._failure("INVALID_ARGUMENT", "instance_id must be a string")
        try:
            if action == "start":
                return self._start(args.get("run_id"), self._resolve_input(args, required=True), args.get("instance_id"))
            if action == "stop":
                # Canvas disconnection includes the binding it is removing.
                return self._stop(args.get("run_id"), self._resolve_input(args), args.get("instance_id"))
            if action == "config":
                return self._configure({key: value for key, value in args.items() if key not in _CONTEXT_KEYS})
            return self.info(self._resolve_input(args) or None, args.get("run_id"))
        except (TypeError, ValueError) as exc:
            return self._failure("INVALID_ARGUMENT", str(exc))

    def _configure(self, changes):
        with self._lock:
            candidate = QualityConfig.from_mapping({**asdict(self._analyzer.config), **changes})
            if candidate == self._analyzer.config:
                return self._info_locked()
            if self._run_id is not None or self._runtime is not None or self._state in {"starting", "running", "stopping"}:
                return self._failure("STOP_REQUIRED", "stop the Card before changing configuration")
            self._analyzer = ImageQualityAnalyzer(candidate)
            self._generation += 1
            self._state, self._last_error = "idle", None
            self._input_topic, self._instance_id = "", None
            return self._info_locked()

    def _run_if_current(self, token, run_id, operation):
        with self._lock:
            if token != self._generation or run_id != self._run_id or self._state != "running" or self._shutdown:
                return False
            operation(self._analyzer)
            return True

    def _release(self, runtime):
        """Release this captured owner, never a runtime from a later start.

        Caller holds the lifecycle lock, without the state lock. New starts
        cannot acquire resources until close finishes, even if another stop
        supersedes the caller's operation token during close.
        """
        try:
            runtime.close()
        except Exception as exc:
            return f"runtime cleanup failed: {exc}"[:400]
        with self._lock:
            if self._runtime is runtime:
                self._runtime = None
        return None

    def _start(self, run_id, topic, instance_id):
        with self._lock:
            error = self._validate_run_locked(run_id)
            if error:
                return error
            if self._shutdown:
                return self._failure("SHUTDOWN", "local shutdown has closed admission", run_id)
            record = self._history.get(run_id)
            if record and record["state"] in _TERMINAL:
                return self._failure("STALE_RUN", "terminated runs cannot restart", run_id)
            if self._run_id == run_id:
                if (topic, instance_id) != (self._input_topic, self._instance_id):
                    return self._failure("INVALID_ARGUMENT", "run binding is immutable", run_id)
                if self._state in {"starting", "running"}:
                    return self._info_locked()
            if self._run_id is not None or self._runtime is not None or self._state in {"starting", "running", "stopping"}:
                return self._failure("BUSY", "stop the existing instance before starting another")
            error = self._record_locked(run_id, "starting", topic, instance_id)
            if error:
                return error
            self._generation += 1
            token = self._generation
            self._run_id = run_id
            self._input_topic, self._instance_id = topic, instance_id
            self._analyzer.reset()
            self._last_error = None
            self._state = "starting"

        with self._lifecycle_lock:
            with self._lock:
                if token != self._generation or self._state != "starting":
                    return self._failure("START_CANCELLED", "start was cancelled")
            runtime = None
            failure = None
            try:
                runtime = self._runtime_factory(namespace=self._namespace, executor=self._executor)
                if not callable(getattr(runtime, "close", None)):
                    raise ValueError("runtime must provide close() and remain inert until start()")
                with self._lock:
                    self._runtime = runtime
                    current = token == self._generation and self._state == "starting"
                if not callable(getattr(runtime, "start", None)):
                    raise ValueError("runtime must provide start()")
                if current:
                    runtime.start(input_topic=topic, output_topic=f"{topic}/{self.PREFIX}",
                                  run_id=run_id, analysis_hz=self._analyzer.config.analysis_hz,
                                  run_if_current=lambda operation: self._run_if_current(token, run_id, operation))
                with self._lock:
                    if token == self._generation and self._state == "starting":
                        self._state = "running"
                        self._history[run_id]["state"] = "running"
                        return self._info_locked()
            except Exception as exc:
                failure = f"runtime start failed: {exc}"[:400]

            with self._lock:
                owned = runtime is not None and self._runtime is runtime
            cleanup_error = self._release(runtime) if owned else None
            with self._lock:
                if cleanup_error:
                    if token == self._generation:
                        self._state, self._last_error = "error", cleanup_error
                    return self._failure("CLEANUP_FAILED", cleanup_error)
                if token == self._generation and self._state == "starting":
                    self._history[run_id]["state"] = "failed"
                    self._run_id = None
                    self._state, self._last_error = "error", failure
                    return self._failure("START_FAILED", failure or "runtime start failed")
                return self._failure("START_CANCELLED", "start was cancelled")

    def _stop_target_matches(self, topic, instance_id):
        # Admission filter only: Topic/instance labels can repeat across starts.
        # Caller holds _lock. The operation token below owns the actual stop.
        return ((not topic or not self._input_topic or topic == self._input_topic)
                and (instance_id is None or self._instance_id is None
                     or instance_id == self._instance_id))

    def _stop(self, run_id, expected_topic="", expected_instance_id=None):
        with self._lock:
            error = self._validate_run_locked(run_id)
            if error:
                return error
            record = self._history.get(run_id)
            if self._run_id != run_id:
                if self._run_id is not None:
                    # A cancellation can arrive before its start while another
                    # run still owns the slot. Remember it without touching that
                    # owner, so the delayed start cannot later revive this ID.
                    if record is None:
                        error = self._record_locked(run_id, "cancelled")
                        if error:
                            return error
                    code = "STALE_RUN" if record and record["state"] in _TERMINAL else "RUN_ID_MISMATCH"
                    return self._failure(code, "stop does not target the active run", run_id)
                if record is None:
                    error = self._record_locked(run_id, "cancelled")
                    if error:
                        return error
                result = self._info_locked(requested=run_id)
                # This acknowledges the requested terminal run. A failed start
                # may leave a diagnostic error state after all resources were
                # released; it must not keep Core's slot owner busy forever.
                # Do not mutate the global state for a delayed old request.
                result.update(state="idle", code="ALREADY_STOPPED", error=None,
                              diagnostic_state=self._state)
                return result
            if not self._stop_target_matches(expected_topic, expected_instance_id):
                return self._failure("INVALID_ARGUMENT", "stop target does not match the current binding")
            self._generation += 1
            token = self._generation
            self._state = "stopping"
            self._history[run_id]["state"] = "stopping"
        with self._lifecycle_lock:
            with self._lock:
                if token != self._generation or run_id != self._run_id:
                    # A newer operation owns the lifecycle. This is its current
                    # snapshot, not permission for this old stop to control it.
                    return self._info_locked()
                runtime = self._runtime
            cleanup_error = self._release(runtime) if runtime is not None else None
            with self._lock:
                if token != self._generation or run_id != self._run_id:
                    # A newer stop keeps stopping until it completes. Do not
                    # open an idle/start window or overwrite its state/error.
                    if cleanup_error:
                        return self._failure("CLEANUP_FAILED", cleanup_error)
                    return self._info_locked()
                if cleanup_error:
                    self._state, self._last_error = "error", cleanup_error
                    return self._failure("CLEANUP_FAILED", cleanup_error)
                self._input_topic, self._instance_id = "", None
                self._history[run_id]["state"] = "closed"
                self._run_id = None
                self._state, self._last_error = "idle", None
                self._analyzer.reset()
                return self._info_locked()

    def shutdown(self):
        """Local process exit only. Never exposed as an MCP action."""
        with self._lock:
            self._shutdown = True
            self._generation += 1
            token = self._generation
            self._state = "stopping"
        with self._lifecycle_lock:
            with self._lock:
                if token != self._generation:
                    return self._info_locked()
                runtime = self._runtime
            error = self._release(runtime) if runtime is not None else None
            with self._lock:
                if token != self._generation:
                    return self._info_locked()
                if error:
                    self._state, self._last_error = "error", error
                    return self._failure("CLEANUP_FAILED", error)
                if self._run_id is not None:
                    self._history[self._run_id]["state"] = "closed"
                self._run_id = None
                self._input_topic, self._instance_id = "", None
                self._state, self._last_error = "idle", None
                self._analyzer.reset()
                return self._info_locked()
