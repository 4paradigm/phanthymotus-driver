"""LinkerHand L20 Tactile Sensing & Matrix Touch Sensor Card.

Provides real-time streaming and polling of:
1. 5-finger scalar fingertip pressures (0–255)
2. 5-finger 12x6 tactile matrix taxel arrays
3. Freshness/error metadata and contact flags (no calibrated force claims)

Card Type: "sensor"
Streams to topic: /{namespace}/hand/{side}/tactile as "sensor/tactile"
"""

from __future__ import annotations

import json
import logging
import math
import re
import threading
import time
import uuid
from typing import Any, Optional

from common.vendor_runtime import action_schema, tool
from can_client import FINGER_NAMES
from hand_device import HandPlugin, _SingleHandController


logger = logging.getLogger("linkerhand.tactile")


def _number(value, name, lower, upper):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{name} 必须是 {lower}–{upper} 之间的数值")
    return float(value)


class TactilePlugin:
    PREFIX = "hand_tactile"

    def __init__(
        self,
        hand: HandPlugin | None,
        config: dict,
        global_config: dict | None = None,
        ros2: Any = None,
        namespace: str = "",
    ):
        self.hand = hand
        self.config = config or {}
        self.global_config = dict(global_config or (hand.config if hand else {}))
        self._ros2 = ros2
        self.namespace = (namespace or "").strip("/")

        self.default_side = str(self.global_config.get("side", "left"))
        self.default_channel = str(self.global_config.get("can_channel", "can0"))
        if self.default_side not in ("left", "right"):
            raise ValueError("side 必须是 left 或 right")
        if not re.fullmatch(r"[A-Za-z0-9_.:-]+", self.default_channel):
            raise ValueError("can_channel 必须是有效的 CAN 接口名称")

        self.poll_hz = _number(self.config.get("poll_hz", 20.0), "poll_hz", 1, 50)
        self.matrix_interval_s = _number(self.config.get("matrix_request_interval_s", 0.05), "matrix_request_interval_s", 0.01, 60)
        self.tactile_timeout_s = _number(self.config.get("tactile_timeout_s", 0.25), "tactile_timeout_s", 0.02, 5)
        self.matrix_timeout_s = _number(self.config.get("matrix_timeout_s", 0.25), "matrix_timeout_s", 0.02, 5)
        self.stale_after_s = _number(self.config.get("stale_after_s", 1.0), "stale_after_s", 0.05, 30)
        self.contact_threshold = self.config.get("contact_threshold", 8)
        if isinstance(self.contact_threshold, bool) or not isinstance(self.contact_threshold, int) or not 1 <= self.contact_threshold <= 255:
            raise ValueError("contact_threshold 必须是 1–255 之间的整数")
        self.auto_start = self.config.get("auto_start", True)
        self.matrix_enabled = self.config.get("matrix_enabled", True)
        if not isinstance(self.auto_start, bool) or not isinstance(self.matrix_enabled, bool):
            raise ValueError("auto_start 和 matrix_enabled 必须是布尔值")

        self._lock = threading.RLock()
        self._bindings: dict[str, str] = {}
        self._channels: dict[str, str] = {}
        self._streams: dict[tuple[str, str], TactilePlugin] = {}
        self._stream_targets: dict[str, tuple[str, str]] = {}
        self._sample_lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._latest_data: dict[str, Any] = {}
        self._last_matrix_req_time: float = 0.0
        self._matrix_cache: dict[tuple[str, str], dict] = {}
        self._sequence = 0
        self._published_frames = 0
        self._last_publish_time = None
        self._last_publish_error = None
        self._last_publisher_attempt = 0.0
        # ROS 2 Publisher attributes
        self._node = None
        self._pub = None

        # Resolve topics
        self._topic = self._target_topic(self.default_side, self.default_channel)

    # ── Topic Metadata ────────────────────────────────────────────────

    def _target_topic(self, side: str, channel: str) -> str:
        # Keep can0 paths compatible; other adapters must not share its topic.
        base_channel = str(self.global_config.get("can_channel", "can0"))
        adapter = "" if channel == base_channel else "/can_" + channel.encode().hex()
        return f"/{self.namespace}/hand/{side}{adapter}/tactile".replace("//", "/")

    def topic_out(self, side: str | None = None, channel: str | None = None) -> list[dict]:
        topic = self._target_topic(side or self.default_side, channel or self.default_channel)
        return [{"topic": topic, "format": "sensor/tactile"}]

    # ── Card Definition (Schema) ──────────────────────────────────────

    def get_tool(self) -> dict:
        schema = action_schema(
            {
                "info": ([], "获取触觉传感器卡片信息、当前绑定状态及 topic_out"),
                "get_state": (["bound_side"], "应用所选左右手到当前采集流，并读取五指新鲜压力与完整阵列快照"),
                "start": ([], "启动触觉数据后台连续采集与流式推送"),
                "stop": ([], "停止触觉数据流推送"),
                "config": (["bound_side"], "按提供的字段更新实例左右手／CAN 接口或共享采集频率"),
            },
            {
                "bound_side": {
                    "type": "string",
                    "enum": ["left", "right"],
                    "description": "选择本卡片实际采集的左手或右手，触觉数据流同步切换",
                },
            },
        )

        definition = tool(
            self.PREFIX,
            "sensor",
            "LinkerHand L20 五指点阵触觉与指尖压力传感器数据流与状态读取",
            schema,
            topic_out=self.topic_out(),
        )
        definition["title"] = "L20 触觉感知"
        definition["multiInstance"] = True
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "can_channel": {
                    "type": "string",
                    "default": self.default_channel,
                    "scope": "instance",
                    "title": "高级：CAN 接口",
                    "description": "通常保持 can0；仅连接到其他 CAN 适配器时修改，例如 can1。不是 CAN ID。",
                },
                "poll_hz": {
                    "type": "number",
                    "default": self.poll_hz,
                    "minimum": 1.0,
                    "maximum": 50.0,
                    "title": "后台采集频率（共享）",
                },
            },
        }
        return definition

    # ── Lifecycle ────────────────────────────────────────────────────

    def start(self) -> None:
        """Start lifecycle hook invoked by DriverBundle."""
        if self.auto_start:
            self._start_streaming()

    def stop(self) -> None:
        """Stop lifecycle hook invoked by DriverBundle."""
        with self._lifecycle_lock:
            for stream in list(self._streams.values()):
                if stream is not self:
                    stream._stop_streaming()
            self._stop_streaming()
            self._streams.clear()
            self._stream_targets.clear()

    # ── Controller & CAN Client Access ────────────────────────────────

    def _resolve_target(self, args: dict) -> tuple[str, str]:
        instance_id = str(args.get("instance_id", ""))
        with self._lock:
            side = args.get("bound_side") or self._bindings.get(instance_id) or self.default_side
            channel = args.get("can_channel") or self._channels.get(instance_id) or self.default_channel
        if side not in ("left", "right"):
            raise ValueError("bound_side 必须是 left 或 right")
        if not isinstance(channel, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]+", channel):
            raise ValueError("can_channel 必须是有效的 CAN 接口名称")
        return side, channel

    def _get_controller(self, side: str, channel: str) -> Optional[_SingleHandController]:
        if self.hand is None:
            return None
        if isinstance(self.hand, HandPlugin):
            return self.hand.get_controller_for_side(side, channel)
        if hasattr(self.hand, "get_controller_for_side"):
            return self.hand.get_controller_for_side(side, channel)
        return None

    def _running(self) -> bool:
        return self._worker_thread is not None and self._worker_thread.is_alive()

    def _select_stream(self, instance_id: str, side: str, channel: str) -> TactilePlugin:
        """One worker per physical target, with independent card ownership."""
        with self._lifecycle_lock:
            target = (side, channel)
            previous = self._stream_targets.get(instance_id)
            stream = self._streams.get(target)
            if stream is None:
                if target == (self.default_side, self.default_channel):
                    stream = self
                else:
                    stream = TactilePlugin(
                        self.hand, {**self.config, "auto_start": False, "poll_hz": self.poll_hz},
                        {**self.global_config, "side": side, "can_channel": channel},
                        ros2=self._ros2, namespace=self.namespace,
                    )
                    # Derive paths using the parent's original default adapter.
                    stream._topic = self._target_topic(side, channel)
                self._streams[target] = stream
            self._stream_targets[instance_id] = target
            with self._lock:
                self._bindings[instance_id] = side
                self._channels[instance_id] = channel
            if previous is not None and previous != target and previous not in self._stream_targets.values():
                old = self._streams.pop(previous, None)
                if old is not None:
                    old._stop_streaming()
            # Replace the unclaimed auto-start stream when a card selects another hand.
            default = (self.default_side, self.default_channel)
            if target != default and default not in self._stream_targets.values():
                self._stop_streaming()
                self._streams.pop(default, None)
            stream._start_streaming()
            return stream

    def _stop_instance_stream(self, instance_id: str) -> None:
        with self._lifecycle_lock:
            target = self._stream_targets.pop(instance_id, None)
            # An unowned stop must not shut down the auto-start default stream.
            if target is not None and target not in self._stream_targets.values():
                stream = self._streams.pop(target, None)
                if stream is not None:
                    stream._stop_streaming()

    # ── Data Reading ──────────────────────────────────────────────────

    def _read_tactile_snapshot(self, side: str, channel: str, *, cancel=None) -> dict[str, Any]:
        # Serialize direct queries with background acquisition. The CAN client's
        # fresh-query APIs require new replies, never a previous cache value.
        with self._sample_lock:
            errors = {}
            touch_scalar = {name: None for name in FINGER_NAMES}
            scalar_age = {name: None for name in FINGER_NAMES}
            scalar_valid = False
            scalar_time = None
            client = None
            try:
                ctrl = self._get_controller(side, channel)
                if ctrl is None:
                    raise RuntimeError(f"找不到 {side}/{channel} 的手部控制器")
                client = ctrl.get()
                snapshot = client.get_fresh_tactile(timeout_s=self.tactile_timeout_s, cancel=cancel)
                values = snapshot.get("touch_scalar", {})
                if any(isinstance(values.get(name), bool) or not isinstance(values.get(name), int)
                       or not 0 <= values[name] <= 255 for name in FINGER_NAMES):
                    raise ValueError("本轮五指压力不完整或不在 0–255 范围内")
                touch_scalar = {name: values[name] for name in FINGER_NAMES}
                scalar_age = dict(snapshot.get("scalar_age_s", scalar_age))
                scalar_time = time.time()
                scalar_valid = True
            except InterruptedError:
                raise
            except Exception as exc:
                errors["scalar"] = str(exc)

            key = (side, channel)
            entry = self._matrix_cache.setdefault(key, {})
            matrix_fresh = False
            due = time.monotonic() - entry.get("requested_at", -float("inf")) >= self.matrix_interval_s
            if self.matrix_enabled and client is not None and due:
                entry["requested_at"] = time.monotonic()
                self._last_matrix_req_time = entry["requested_at"]
                try:
                    if cancel is not None and cancel.is_set():
                        raise InterruptedError("触觉采集已停止")
                    matrix = client.get_fresh_matrix_touch(timeout_s=self.matrix_timeout_s)
                    if any(not isinstance(matrix.get(name), list) or len(matrix[name]) != 12 or
                           any(not isinstance(row, list) or len(row) != 6 or
                               any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 255 for v in row)
                               for row in matrix[name]) for name in FINGER_NAMES):
                        raise ValueError("本轮阵列缺少完整的五指 12×6 数据")
                    entry.update(matrix={name: [list(row) for row in matrix[name]] for name in FINGER_NAMES},
                                 received_at=time.monotonic(), timestamp=time.time())
                    entry.pop("error", None)
                    matrix_fresh = True
                except InterruptedError:
                    raise
                except Exception as exc:
                    entry["error"] = str(exc)
            matrix_age = max(0.0, time.monotonic() - entry["received_at"]) if "received_at" in entry else None
            matrix_valid = self.matrix_enabled and matrix_age is not None and matrix_age <= self.stale_after_s and "error" not in entry
            matrix = entry["matrix"] if matrix_valid else {name: None for name in FINGER_NAMES}
            if self.matrix_enabled and not matrix_valid:
                errors["matrix"] = entry.get("error", "尚未收到完整的新阵列数据")
            matrix_sums = {name: sum(map(sum, matrix[name])) if matrix_valid else None for name in FINGER_NAMES}
            contact_flags = {
                name: ((scalar_valid and touch_scalar[name] >= self.contact_threshold) or
                       (matrix_valid and matrix_sums[name] > 0)) if scalar_valid or matrix_valid else None
                for name in FINGER_NAMES
            }
            # A matrix query takes time. Scalar age includes that time, rather
            # than relabeling an earlier scalar reply with the publication time.
            if scalar_time is not None:
                extra_age = max(0.0, time.time() - scalar_time)
                scalar_age = {name: scalar_age.get(name) + extra_age if scalar_age.get(name) is not None else extra_age
                              for name in FINGER_NAMES}
                if max(scalar_age.values()) > self.stale_after_s:
                    scalar_valid = False
                    errors["scalar"] = "标量读数已过期"
                    touch_scalar = {name: None for name in FINGER_NAMES}
                    contact_flags = {name: matrix_sums[name] > 0 if matrix_valid else None for name in FINGER_NAMES}
            with self._lock:
                self._sequence += 1
                sequence = self._sequence
            return {
                "state": "error" if not scalar_valid else "degraded" if errors else "ok",
                "side": side, "bound_side": side, "can_channel": channel,
                "timestamp": time.time(), "sequence": sequence,
                "scalar_timestamp": scalar_time, "scalar_valid": scalar_valid,
                "scalar_age_s": scalar_age, "touch_scalar": touch_scalar,
                "matrix_timestamp": entry.get("timestamp"), "matrix_valid": matrix_valid,
                "matrix_fresh": matrix_fresh, "matrix_age_s": matrix_age,
                "touch_matrix": matrix, "matrix_sums": matrix_sums,
                "contact_flags": contact_flags, "errors": errors,
                "normal_force": {name: None for name in FINGER_NAMES},
                "tangential_force": {name: None for name in FINGER_NAMES},
                "tangential_dir": {name: None for name in FINGER_NAMES},
                "force_fields_valid": False,
            }

    # ── ROS 2 Publishing & Streaming Thread ──────────────────────────

    def _setup_ros_publisher(self) -> bool:
        self._last_publisher_attempt = time.monotonic()
        if self._ros2 is None or not hasattr(self._ros2, "ctx_core"):
            self._last_publish_error = "没有可用的 ROS 2 Core 上下文，仅能本地读取"
            return False
        if self._pub is not None:
            return True
        node = None
        try:
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE,
            )
            node_name = f"hand_tactile_{self.default_side}_{uuid.uuid4().hex[:8]}"
            node = Node(node_name, context=self._ros2.ctx_core)
            pub = node.create_publisher(String, self._topic, qos)

            self._ros2.executor_core.add_node(node)
            self._node = node
            self._pub = pub
            self._last_publish_error = None
            print(f"[tactile] ROS2 publisher ready: {self._topic}", flush=True)
            return True
        except Exception as exc:
            if node is not None:
                try:
                    self._ros2.executor_core.remove_node(node)
                    node.destroy_node()
                except Exception:
                    pass
            self._last_publish_error = str(exc)
            logger.warning("Failed to setup ROS2 tactile publisher: %s", exc)
            return False

    def _teardown_ros_publisher(self) -> None:
        node, self._node = self._node, None
        self._pub = None
        if node is None or self._ros2 is None:
            return
        try:
            self._ros2.executor_core.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass

    def _start_streaming(self) -> None:
        with self._lifecycle_lock:
            if self._worker_thread is not None and self._worker_thread.is_alive():
                return
            self._stop_event.clear()
            self._setup_ros_publisher()
            self._worker_thread = threading.Thread(
                target=self._stream_loop,
                daemon=True,
                name="tactile_stream_worker",
            )
            try:
                self._worker_thread.start()
            except Exception:
                self._stop_event.set()
                self._worker_thread = None
                self._teardown_ros_publisher()
                raise

    def _stop_streaming(self) -> None:
        with self._lifecycle_lock:
            self._stop_event.set()
            thread = self._worker_thread
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=self.tactile_timeout_s + self.matrix_timeout_s + 1.0)
                if thread.is_alive():
                    raise RuntimeError("触觉采集尚未停止；保留线程及发布器，避免重复启动")
            self._worker_thread = None
            self._teardown_ros_publisher()

    def _stream_loop(self) -> None:
        side = self.default_side
        channel = self.default_channel
        consecutive_errors = 0

        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            try:
                data = self._read_tactile_snapshot(side, channel, cancel=self._stop_event)
                with self._lock:
                    self._latest_data = data
                if self._pub is None and time.monotonic() - self._last_publisher_attempt >= 2.0:
                    self._setup_ros_publisher()
                if self._pub is not None and not self._stop_event.is_set():
                    from std_msgs.msg import String
                    msg = String()
                    msg.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
                    self._pub.publish(msg)

                    with self._lock:
                        self._published_frames += 1
                        self._last_publish_time = time.time()
                        self._last_publish_error = None
                if consecutive_errors:
                    logger.info("Tactile stream loop recovered after %d consecutive errors", consecutive_errors)
                    consecutive_errors = 0
            except InterruptedError:
                if self._stop_event.is_set():
                    break
                consecutive_errors += 1
                if consecutive_errors == 1 or consecutive_errors % 100 == 0:
                    logger.warning("Tactile acquisition interrupted unexpectedly (consecutive errors: %d)", consecutive_errors)
            except Exception as exc:
                self._last_publish_error = str(exc)
                consecutive_errors += 1
                if consecutive_errors == 1 or consecutive_errors % 100 == 0:
                    logger.warning("Error in tactile stream loop (consecutive errors: %d): %s", consecutive_errors, exc)
                self._teardown_ros_publisher()

            elapsed = time.monotonic() - loop_start
            sleep_time = max(0.002, 1.0 / self.poll_hz - elapsed)
            self._stop_event.wait(timeout=sleep_time)

    # ── MCP Dispatcher ────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict:
        instance_id = str(args.get("instance_id", ""))
        side, channel = self._resolve_target(args)

        if action == "config":
            bound = args.get("bound_side")
            if "bound_side" in args and bound not in ("left", "right"):
                raise ValueError("bound_side 必须是 left 或 right")
            rate = _number(args.get("poll_hz", self.poll_hz), "poll_hz", 1, 50)
            with self._lock:
                # Core applies shared settings before instance settings. A
                # frequency-only config must not require or replace a binding.
                if "bound_side" in args:
                    self._bindings[instance_id] = bound
                if "can_channel" in args:
                    self._channels[instance_id] = channel
                self.poll_hz = rate
            with self._lifecycle_lock:
                for stream in self._streams.values():
                    stream.poll_hz = rate
                if instance_id in self._stream_targets and ("bound_side" in args or "can_channel" in args):
                    self._select_stream(instance_id, side, channel)
            return {"state": "configured", "bound_side": side, "can_channel": channel,
                    "stream_side": side, "stream_can_channel": channel,
                    "topic_out": self.topic_out(side, channel)}

        if action == "info":
            with self._lifecycle_lock:
                stream = self._streams.get((side, channel))
                if stream is None and (side, channel) == (self.default_side, self.default_channel):
                    stream = self
                running = stream is not None and stream._running()
                latest = dict(stream._latest_data) if stream is not None else {}
            try:
                subscribers = stream._pub.get_subscription_count() if stream is not None and stream._pub is not None else 0
            except Exception:
                subscribers = 0
            return {
                "state": "running" if running else "idle",
                "bound_side": side,
                "can_channel": channel,
                "topic_out": self.topic_out(side, channel),
                "poll_hz": self.poll_hz,
                "stream_side": side,
                "stream_can_channel": channel,
                "publisher_ready": stream is not None and stream._pub is not None,
                "subscriber_count": subscribers,
                "published_frames": stream._published_frames if stream is not None else 0,
                "last_publish_time": stream._last_publish_time if stream is not None else None,
                "publish_error": stream._last_publish_error if stream is not None else None,
                "sample_state": latest.get("state", "waiting"),
                "sample_age_s": max(0.0, time.time() - latest["timestamp"]) if latest else None,
                "sample_errors": latest.get("errors", {}),
            }

        if action == "start":
            self._select_stream(instance_id, side, channel)
            return self.dispatch("info", args)

        if action == "stop":
            self._stop_instance_stream(instance_id)
            return {
                "state": "idle",
                "bound_side": side,
                "topic_out": self.topic_out(side, channel),
            }

        if action == "get_state":
            with self._lifecycle_lock:
                running = self._running() or any(s._running() for s in self._streams.values())
                if running:
                    reader = self._select_stream(instance_id, side, channel)
                else:
                    reader = self
                    with self._lock:
                        self._bindings[instance_id] = side
                        self._channels[instance_id] = channel
            # A manual read requests new scalar replies too; do not silently
            # return the background cache with an earlier sensor age.
            latest = reader._read_tactile_snapshot(side, channel)
            return {
                "action": "get_state",
                **latest,
                "topic_out": self.topic_out(side, channel),
                "sample_age_s": max(0.0, time.time() - latest["timestamp"]),
            }

        return {"state": "error", "message": f"Unknown action: {action}"}
