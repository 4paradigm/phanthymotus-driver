"""LinkerHand L20 Lite Tactile Sensing & Matrix Touch Sensor Card.

Mirrors the L20 ``hand_tactile`` card, but reads over the USB-RS485 Modbus
link shared with the hand card (the Lite has no CAN bus):
1. 5-finger Tip: mean pressure across the 72 displayed cells (raw sum as touch_sum)
2. 5-finger 12x6 tactile matrix taxel arrays (first 72 cells, row-major)
3. All 96 raw cells per finger as ``touch_cells`` (not drawn by Core)
4. Freshness/error metadata and contact flags (no calibrated force claims)

Card Type: "sensor"
Streams to topic: /{namespace}/l20_lite/hand/{side}/tactile as "sensor/tactile"
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
import uuid
from typing import Any

from common.vendor_runtime import action_schema, tool
from modbus_client import FINGER_IDS, TACTILE_CELL_COUNT


logger = logging.getLogger("linkerhand.l20_lite.tactile")

FINGER_NAMES = list(FINGER_IDS)
MATRIX_ROWS, MATRIX_COLS = 12, 6
MATRIX_CELLS = MATRIX_ROWS * MATRIX_COLS


def _number(value, name, lower, upper):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{name} 必须是 {lower}–{upper} 之间的数值")
    return float(value)


def _matrix(cells: list[int]) -> list[list[int]]:
    # The vendor host app draws the first 72 cells as 12 rows of 6. Registers
    # are 16-bit, but the L20 matrix schema is 0–255, so clip each cell to 255
    # (touch_cells keeps the raw values).
    return [[min(v, 255) for v in cells[row * MATRIX_COLS:(row + 1) * MATRIX_COLS]]
            for row in range(MATRIX_ROWS)]


class LiteTactilePlugin:
    PREFIX = "hand_tactile"

    def __init__(
        self,
        hand: Any,
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

        self.default_side = str(self.global_config.get("side", "right"))
        if self.default_side not in ("left", "right"):
            raise ValueError("side 必须是 left 或 right")

        # One round is 5 x (select + 201-byte read) on 115200 baud, about 150 ms.
        self.poll_hz = _number(self.config.get("poll_hz", 10.0), "poll_hz", 1, 10)
        self.matrix_interval_s = _number(self.config.get("matrix_request_interval_s", 0.05), "matrix_request_interval_s", 0.01, 60)
        self.tactile_timeout_s = _number(self.config.get("tactile_timeout_s", 1.0), "tactile_timeout_s", 0.02, 5)
        self.matrix_timeout_s = _number(self.config.get("matrix_timeout_s", 0.25), "matrix_timeout_s", 0.02, 5)
        self.stale_after_s = _number(self.config.get("stale_after_s", 1.0), "stale_after_s", 0.05, 30)
        # The vendor app waits 30 ms after selecting; hardware answers without
        # it, and reg 0 still proves which finger the reply belongs to.
        self.select_settle_s = _number(self.config.get("select_settle_s", 0.0), "select_settle_s", 0, 0.5)
        self.contact_threshold = self.config.get("contact_threshold", 8)
        if isinstance(self.contact_threshold, bool) or not isinstance(self.contact_threshold, int) or not 1 <= self.contact_threshold <= 255:
            raise ValueError("contact_threshold 必须是 1–255 之间的整数")
        self.auto_start = self.config.get("auto_start", True)
        self.matrix_enabled = self.config.get("matrix_enabled", True)
        if not isinstance(self.auto_start, bool) or not isinstance(self.matrix_enabled, bool):
            raise ValueError("auto_start 和 matrix_enabled 必须是布尔值")

        self._lock = threading.RLock()
        self._bindings: dict[str, str] = {}
        self._streams: dict[str, LiteTactilePlugin] = {}
        self._stream_targets: dict[str, str] = {}
        self._sample_lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._latest_data: dict[str, Any] = {}
        self._last_matrix_req_time: float = 0.0
        self._matrix_cache: dict[str, dict] = {}
        self._sequence = 0
        self._published_frames = 0
        self._last_publish_time = None
        self._last_publish_error = None
        self._last_publisher_attempt = 0.0
        # ROS 2 Publisher attributes
        self._node = None
        self._pub = None

        # Resolve topics
        self._topic = self._target_topic(self.default_side)

    # ── Topic Metadata ────────────────────────────────────────────────

    def _target_topic(self, side: str) -> str:
        # The l20_lite segment keeps this apart from an L20 driver on the same robot.
        return f"/{self.namespace}/l20_lite/hand/{side}/tactile".replace("//", "/")

    def topic_out(self, side: str | None = None) -> list[dict]:
        topic = self._target_topic(side or self.default_side)
        return [{"topic": topic, "format": "sensor/tactile"}]

    # ── Card Definition (Schema) ──────────────────────────────────────

    def get_tool(self) -> dict:
        schema = action_schema(
            {
                "info": ([], "获取触觉传感器卡片信息、当前绑定状态及 topic_out"),
                "get_state": ([], "读取当前绑定手的一次五指触觉快照"),
                "start": ([], "启动触觉数据后台连续采集与流式推送"),
                "stop": ([], "停止触觉数据流推送"),
                "config": ([], "按提供的字段更新实例左右手或共享采集频率"),
            },
            {},
        )

        definition = tool(
            self.PREFIX,
            "sensor",
            "LinkerHand L20 Lite 五指点阵触觉与指尖压力传感器数据流与状态读取",
            schema,
            topic_out=self.topic_out(),
        )
        definition["title"] = "L20 Lite 触觉感知"
        definition["multiInstance"] = True
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "bound_side": {
                    "type": "string",
                    "enum": ["left", "right"],
                    "default": self.default_side,
                    "scope": "instance",
                    "title": "采集左右手",
                    "description": "选择本卡片采集的左手或右手；启动智能控制后自动采集，切换后请重新打开数据流。",
                },
                "poll_hz": {
                    "type": "number",
                    "default": self.poll_hz,
                    "minimum": 1.0,
                    "maximum": 10.0,
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

    # ── Target Resolution ─────────────────────────────────────────────

    def _resolve_target(self, args: dict) -> str:
        instance_id = str(args.get("instance_id", ""))
        with self._lock:
            side = args.get("bound_side") or self._bindings.get(instance_id) or self.default_side
        if side not in ("left", "right"):
            raise ValueError("bound_side 必须是 left 或 right")
        return side

    def _running(self) -> bool:
        return self._worker_thread is not None and self._worker_thread.is_alive()

    def _select_stream(self, instance_id: str, side: str) -> LiteTactilePlugin:
        """One worker per hand, with independent card ownership."""
        with self._lifecycle_lock:
            if not instance_id:
                # Default / anonymous stream is explicitly single-instance.
                # Reject overwriting an active anonymous stream with a different target.
                current_target = self._stream_targets.get("")
                if current_target is not None and current_target != side:
                    raise ValueError(
                        f"默认触觉流为单实例（当前已绑定 {current_target}）；"
                        f"启动或切换不同目标 ({side}) 必须指定非空 instance_id"
                    )
            previous = self._stream_targets.get(instance_id)
            stream = self._streams.get(side)
            if stream is None:
                if side == self.default_side:
                    stream = self
                else:
                    stream = LiteTactilePlugin(
                        self.hand, {**self.config, "auto_start": False, "poll_hz": self.poll_hz},
                        {**self.global_config, "side": side},
                        ros2=self._ros2, namespace=self.namespace,
                    )
            other_targets = {target for owner, target in self._stream_targets.items()
                             if owner != instance_id}
            old_sides = {previous, self.default_side} - other_targets - {None, side}
            was_running = stream._running()
            stream._start_streaming()
            try:
                for old_side in old_sides:
                    old = self if old_side == self.default_side else self._streams[old_side]
                    old._stop_streaming()
                    self._streams.pop(old_side, None)
            except Exception:
                if not was_running:
                    try:
                        stream._stop_streaming()
                    except Exception:
                        self._streams[side] = stream
                        logger.exception("Failed to stop tactile stream after unsuccessful rebind")
                raise
            self._streams[side] = stream
            self._stream_targets[instance_id] = side
            with self._lock:
                self._bindings[instance_id] = side
            return stream

    def _stop_instance_stream(self, instance_id: str) -> None:
        with self._lifecycle_lock:
            target = self._stream_targets.get(instance_id)
            if target is None:
                # The lifecycle starts the default stream before any card claims it.
                if not instance_id and self.default_side not in self._stream_targets.values():
                    self._stop_streaming()
                return
            other_targets = (side for owner, side in self._stream_targets.items()
                             if owner != instance_id)
            if target not in other_targets:
                stream = self._streams.get(target)
                if stream is not None:
                    stream._stop_streaming()
                    del self._streams[target]
            del self._stream_targets[instance_id]

    # ── Data Reading ──────────────────────────────────────────────────

    def _read_tactile_snapshot(self, side: str, *, cancel=None) -> dict[str, Any]:
        # Serialize direct queries with background acquisition. Every snapshot
        # is a fresh RS485 read, never a previous cache value.
        with self._sample_lock:
            errors = {}
            touch_scalar = {name: None for name in FINGER_NAMES}
            touch_sum = {name: None for name in FINGER_NAMES}
            touch_cells = {name: None for name in FINGER_NAMES}
            scalar_age = {name: None for name in FINGER_NAMES}
            scalar_valid = False
            scalar_time = None
            serial_port = None
            cells = None
            read_at = None
            try:
                if self.hand is None or not hasattr(self.hand, "read_tactile"):
                    raise RuntimeError(f"找不到 {side} 的手部控制器")
                serial_port, results = self.hand.read_tactile(
                    side, FINGER_NAMES, budget_s=self.tactile_timeout_s,
                    settle_s=self.select_settle_s, cancel=cancel,
                )
                failed = {name: results.get(name, "未读取") for name in FINGER_NAMES
                          if not isinstance(results.get(name), tuple)}
                if failed:
                    raise ValueError("本轮五指压力不完整：" + "；".join(f"{name}: {exc}" for name, exc in failed.items()))
                cells = {name: results[name][0] for name in FINGER_NAMES}
                if any(len(cells[name]) != TACTILE_CELL_COUNT or
                       any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 0xFFFF for v in cells[name])
                       for name in FINGER_NAMES):
                    raise ValueError(f"本轮五指压力不是完整的 {TACTILE_CELL_COUNT} 点数据")
                read_at = {name: results[name][1] for name in FINGER_NAMES}
                # Average the raw 12x6 pressure cells, then round and cap
                # the scalar for the Core renderer's 0–255 integer scale.
                touch_scalar = {
                    name: min(255, (sum(cells[name][:MATRIX_CELLS]) + MATRIX_CELLS // 2) // MATRIX_CELLS)
                    for name in FINGER_NAMES
                }
                touch_sum = {name: sum(cells[name]) for name in FINGER_NAMES}
                # Core draws only the 12x6 touch_matrix; keep the full reply here.
                touch_cells = {name: list(cells[name]) for name in FINGER_NAMES}
                scalar_age = {name: max(0.0, time.monotonic() - read_at[name]) for name in FINGER_NAMES}
                scalar_time = time.time()
                scalar_valid = True
            except InterruptedError:
                raise
            except Exception as exc:
                cells = None
                errors["scalar"] = str(exc)

            entry = self._matrix_cache.setdefault(side, {})
            matrix_fresh = False
            due = time.monotonic() - entry.get("requested_at", -float("inf")) >= self.matrix_interval_s
            if self.matrix_enabled and serial_port is not None and due:
                entry["requested_at"] = time.monotonic()
                self._last_matrix_req_time = entry["requested_at"]
                # The matrix arrives in the same register reply as the pressure.
                if cells is not None:
                    entry.update(matrix={name: _matrix(cells[name]) for name in FINGER_NAMES},
                                 received_at=min(read_at.values()), timestamp=time.time())
                    entry.pop("error", None)
                    matrix_fresh = True
                else:
                    entry["error"] = errors["scalar"]
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
            # Five sequential finger reads take time. Scalar age is measured
            # from each finger's own reply, rather than the publication time.
            if scalar_time is not None and max(scalar_age.values()) > self.stale_after_s:
                scalar_valid = False
                errors["scalar"] = "标量读数已过期"
                touch_scalar = {name: None for name in FINGER_NAMES}
                touch_sum = {name: None for name in FINGER_NAMES}
                touch_cells = {name: None for name in FINGER_NAMES}
                contact_flags = {name: matrix_sums[name] > 0 if matrix_valid else None for name in FINGER_NAMES}
            with self._lock:
                self._sequence += 1
                sequence = self._sequence
            return {
                "state": "error" if not scalar_valid else "degraded" if errors else "ok",
                "model": "l20_lite",
                "side": side, "bound_side": side, "serial_port": serial_port,
                "timestamp": time.time(), "sequence": sequence,
                "scalar_timestamp": scalar_time, "scalar_valid": scalar_valid,
                "scalar_age_s": scalar_age, "touch_scalar": touch_scalar, "touch_sum": touch_sum,
                "touch_cells": touch_cells, "cell_count": TACTILE_CELL_COUNT,
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
        consecutive_errors = 0

        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            try:
                data = self._read_tactile_snapshot(side, cancel=self._stop_event)
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

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = str(args.get("instance_id", ""))

        if action == "config":
            bound = args.get("bound_side")
            if "bound_side" in args and bound not in ("left", "right"):
                raise ValueError("bound_side 必须是 left 或 right")
            with self._lifecycle_lock:
                rate = _number(args.get("poll_hz", self.poll_hz), "poll_hz", 1, 10)
                side = self._resolve_target(args)
                if "bound_side" in args:
                    if instance_id in self._stream_targets:
                        self._select_stream(instance_id, side)
                    else:
                        with self._lock:
                            self._bindings[instance_id] = side
                # Core can apply shared settings without an instance binding.
                self.poll_hz = rate
                for stream in self._streams.values():
                    stream.poll_hz = rate
            return {"state": "configured", "bound_side": side, "stream_side": side,
                    "topic_out": self.topic_out(side)}

        if action not in ("info", "start", "stop", "get_state"):
            return None

        side = self._resolve_target(args)
        if action == "info":
            with self._lifecycle_lock:
                stream = self._streams.get(side)
                if stream is None and side == self.default_side:
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
                "serial_port": latest.get("serial_port"),
                "topic_out": self.topic_out(side),
                "poll_hz": self.poll_hz,
                "stream_side": side,
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
            self._select_stream(instance_id, side)
            return self.dispatch("info", args)

        if action == "stop":
            self._stop_instance_stream(instance_id)
            return {
                "state": "idle",
                "bound_side": side,
                "topic_out": self.topic_out(side),
            }

        if action == "get_state":
            with self._lifecycle_lock:
                running = self._running() or any(s._running() for s in self._streams.values())
                if running:
                    reader = self._select_stream(instance_id, side)
                else:
                    reader = self
                    with self._lock:
                        self._bindings[instance_id] = side
            # A manual read requests new replies too; do not silently return
            # the background cache with an earlier sensor age.
            latest = reader._read_tactile_snapshot(side)
            return {
                "action": "get_state",
                **latest,
                "topic_out": self.topic_out(side),
                "sample_age_s": max(0.0, time.time() - latest["timestamp"]),
            }
