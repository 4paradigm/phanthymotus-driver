"""Bumi ROS 2 processor card for food instance segmentation.

The MCP tool controls and describes the processor.  Image frames remain on
ROS 2 DDS: a depth-one callback stores only the newest compressed frame and a
single worker performs model inference.  Importing this module does not import
Ultralytics, load a model, or download weights.
"""

from __future__ import annotations

import json
import math
import re
import threading
import time
from types import SimpleNamespace
from typing import Any, Callable

import numpy as np

try:  # package import in tests/development
    from .food_segmentation_backend import FoodSegmentationPipeline
    from .food_segmentation_contract import (
        SegmentationLimitError,
        decode_binary_rle,
    )
except ImportError:  # flat /work layout used by the Bumi Docker image
    from food_segmentation_backend import FoodSegmentationPipeline
    from food_segmentation_contract import SegmentationLimitError, decode_binary_rle


SCHEMA_VERSION = "food-segmentation/1"
DEFAULT_MAX_RATE_HZ = 2.0
DEFAULT_INPUT_TIMEOUT_S = 2.0
DEFAULT_STOP_TIMEOUT_S = 10.0
DEFAULT_JPEG_QUALITY = 85
DEFAULT_MAX_COMPRESSED_BYTES = 8 * 1024 * 1024
DEFAULT_MAX_IMAGE_PIXELS = 2_500_000
DEFAULT_MAX_INSTANCES = 50
DEFAULT_MAX_TOTAL_RLE_RUNS = 500_000
DEFAULT_MAX_METADATA_BYTES = 4 * 1024 * 1024
_TOPIC_RE = re.compile(r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$")
_COLORS = (
    (40, 180, 240), (240, 120, 40), (80, 210, 100),
    (210, 80, 180), (180, 210, 40), (80, 100, 230),
)


def _number(config: dict, name: str, default: float,
            minimum: float, maximum: float) -> float:
    value = config.get(name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"food_segmentation.{name} must be a number")
    value = float(value)
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(
            f"food_segmentation.{name} must be in [{minimum:g}, {maximum:g}]"
        )
    return value


def _integer(config: dict, name: str, default: int,
             minimum: int, maximum: int) -> int:
    value = config.get(name, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"food_segmentation.{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ValueError(
            f"food_segmentation.{name} must be in [{minimum}, {maximum}]"
        )
    return value


class FrameLimitError(ValueError):
    """A frame exceeded a configured resource or output limit."""


def _jpeg_dimensions(data: bytes) -> tuple[int, int] | None:
    """Read JPEG SOF dimensions without allocating the decoded image."""

    if len(data) < 4 or data[:2] != b"\xff\xd8":
        return None
    offset = 2
    sof_markers = {
        0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
        0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF,
    }
    while offset < len(data):
        while offset < len(data) and data[offset] != 0xFF:
            offset += 1
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            return None
        marker = data[offset]
        offset += 1
        if marker in (0xD9, 0xDA):
            return None
        if marker == 0x01 or 0xD0 <= marker <= 0xD7:
            continue
        if offset + 2 > len(data):
            return None
        segment_length = int.from_bytes(data[offset:offset + 2], "big")
        if segment_length < 2 or offset + segment_length > len(data):
            return None
        if marker in sof_markers:
            if segment_length < 7:
                return None
            height = int.from_bytes(data[offset + 3:offset + 5], "big")
            width = int.from_bytes(data[offset + 5:offset + 7], "big")
            if width <= 0 or height <= 0:
                return None
            return width, height
        offset += segment_length
    return None


def _json_dumps_limited(value: Any, max_bytes: int) -> str:
    """Serialize JSON incrementally and stop before exceeding the byte budget."""

    encoder = json.JSONEncoder(separators=(",", ":"), allow_nan=False)
    parts = []
    total = 0
    for part in encoder.iterencode(value):
        total += len(part.encode("utf-8"))
        if total > max_bytes:
            raise FrameLimitError(
                "metadata JSON exceeds max_metadata_bytes "
                f"({total} > {max_bytes})")
        parts.append(part)
    return "".join(parts)


def _topic(configured: Any, default: str, namespace: str) -> str:
    value = default if configured in (None, "") else configured
    if not isinstance(value, str):
        raise ValueError("food segmentation topics must be strings")
    value = value.replace("{namespace}", namespace).strip()
    if not value.startswith("/"):
        value = "/" + value
    if not _TOPIC_RE.fullmatch(value):
        raise ValueError(f"invalid ROS 2 topic name: {value!r}")
    return value


def _default_runtime_factory():
    # Deliberately delayed: a disabled/discovered card must not require the
    # AGPL Ultralytics package or trigger its checkpoint resolution behavior.
    try:
        from .food_segmentation_ultralytics import UltralyticsSegmentationRuntime
    except ImportError:
        from food_segmentation_ultralytics import UltralyticsSegmentationRuntime
    return UltralyticsSegmentationRuntime()


def _default_ros_bindings():
    """Import ROS only when an enabled card is actually started."""

    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    def qos_profile():
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

    return SimpleNamespace(
        Node=Node,
        String=String,
        CompressedImage=CompressedImage,
        qos_profile=qos_profile,
        callback_group=MutuallyExclusiveCallbackGroup,
    )


class _OpenCvCodec:
    """Small delayed OpenCV boundary, replaceable by ROS-free unit tests."""

    @staticmethod
    def decode(data: bytes) -> np.ndarray:
        import cv2

        encoded = np.frombuffer(data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("JPEG payload could not be decoded")
        return image

    @staticmethod
    def encode(image: np.ndarray, quality: int) -> bytes:
        import cv2

        ok, encoded = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        )
        if not ok:
            raise RuntimeError("overlay JPEG encoding failed")
        return encoded.tobytes()

    @staticmethod
    def rectangle(image, point1, point2, color) -> None:
        import cv2
        cv2.rectangle(image, point1, point2, color, 2)

    @staticmethod
    def label(image, text, origin, color) -> None:
        import cv2
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
            0.5, color, 2, cv2.LINE_AA,
        )


class FoodSegmentationPlugin:
    """Single-worker, latest-frame-only Food Segmentation processor card."""

    PREFIX = "food_segmentation"

    def __init__(
        self,
        plugin_config: dict,
        namespace: str,
        executor,
        *,
        runtime_factory: Callable[[], Any] | None = None,
        ros_bindings=None,
        image_codec=None,
    ):
        self._config = dict(plugin_config)
        self._namespace = namespace
        self._executor = executor
        self._runtime_factory = runtime_factory or _default_runtime_factory
        self._ros_bindings_factory = ros_bindings or _default_ros_bindings
        self._codec = image_codec or _OpenCvCodec()

        base = f"/{namespace}" if namespace else ""
        self._configured_input_topic = _topic(
            self._config.get("input_topic"), f"{base}/camera/color", namespace)
        self._input_topic = self._configured_input_topic
        self._metadata_topic = _topic(
            self._config.get("metadata_topic"),
            f"{base}/food_segmentation/metadata", namespace)
        self._overlay_topic = _topic(
            self._config.get("overlay_topic"),
            f"{base}/food_segmentation/overlay", namespace)
        if len({self._input_topic, self._metadata_topic, self._overlay_topic}) != 3:
            raise ValueError("food segmentation input and output topics must be distinct")

        self._model_path = self._config.get("model_path", "")
        if not isinstance(self._model_path, str):
            raise ValueError("food_segmentation.model_path must be a string")
        self._confidence = _number(
            self._config, "confidence_threshold", 0.25, 0.0, 1.0)
        self._mask_threshold = _number(
            self._config, "mask_threshold", 0.5, 0.0, 1.0)
        self._max_rate_hz = _number(
            self._config, "max_rate_hz", DEFAULT_MAX_RATE_HZ, 0.1, 30.0)
        self._input_timeout_s = _number(
            self._config, "input_timeout_s", DEFAULT_INPUT_TIMEOUT_S, 0.1, 60.0)
        self._stop_timeout_s = _number(
            self._config, "stop_timeout_s", DEFAULT_STOP_TIMEOUT_S, 0.1, 120.0)
        quality = self._config.get("jpeg_quality", DEFAULT_JPEG_QUALITY)
        if isinstance(quality, bool) or not isinstance(quality, int) or not 1 <= quality <= 100:
            raise ValueError("food_segmentation.jpeg_quality must be an integer in [1, 100]")
        self._jpeg_quality = quality
        imgsz = self._config.get("imgsz", 640)
        if isinstance(imgsz, bool) or not isinstance(imgsz, int) or not 32 <= imgsz <= 4096:
            raise ValueError("food_segmentation.imgsz must be an integer in [32, 4096]")
        self._imgsz = imgsz
        device = self._config.get("device", "")
        if not isinstance(device, str):
            raise ValueError("food_segmentation.device must be a string")
        self._device = device.strip()
        self._max_compressed_bytes = _integer(
            self._config, "max_compressed_bytes",
            DEFAULT_MAX_COMPRESSED_BYTES, 1, 64 * 1024 * 1024)
        self._max_image_pixels = _integer(
            self._config, "max_image_pixels",
            DEFAULT_MAX_IMAGE_PIXELS, 1, 33_554_432)
        self._max_instances = _integer(
            self._config, "max_instances", DEFAULT_MAX_INSTANCES, 1, 1000)
        self._max_total_rle_runs = _integer(
            self._config, "max_total_rle_runs",
            DEFAULT_MAX_TOTAL_RLE_RUNS, 1, 10_000_000)
        self._max_metadata_bytes = _integer(
            self._config, "max_metadata_bytes",
            DEFAULT_MAX_METADATA_BYTES, 1024, 64 * 1024 * 1024)

        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._state = "idle"
        self._running = False
        self._generation = 0
        self._runtime = None
        self._worker: threading.Thread | None = None
        self._pending = None
        self._node = None
        self._subscription = None
        self._metadata_publisher = None
        self._overlay_publisher = None
        self._bindings = None
        self._last_error = None
        self._last_received_monotonic = None
        self._last_frame_timestamp = None
        self._last_processing_ms = None
        self._stats = {
            "received_frames": 0,
            "processed_frames": 0,
            "dropped_frames": 0,
            "decode_errors": 0,
            "inference_errors": 0,
            "publish_errors": 0,
            "limit_errors": 0,
        }

    def get_tool(self) -> dict:
        return {
            "name": "food_segmentation",
            "type": "processor",
            "multiInstance": False,
            "description": (
                "Segments COCO food instances from a ROS 2 JPEG stream. "
                "MCP controls lifecycle only; image data stays on DDS."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info"],
                        "description": "Processor lifecycle action",
                    },
                    "input_topic": {
                        "type": "string",
                        "description": (
                            "Canvas-resolved image/jpeg topic. If omitted, the "
                            "configured Bumi camera topic is used."
                        ),
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": ["input_topic"]},
                    "stop": {"params": []},
                    "info": {"params": []},
                },
            },
            "topic_in": [{"topic": self._input_topic, "format": "image/jpeg"}],
            "topic_out": [
                {"topic": self._metadata_topic, "format": "data/json"},
                {"topic": self._overlay_topic, "format": "image/jpeg"},
            ],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def _load_options(self) -> dict:
        options = {
            "imgsz": self._imgsz,
            "conf": self._confidence,
            "max_instances": self._max_instances,
        }
        if self._device:
            options["device"] = self._device
        return options

    def start(self, input_topic: str | None = None) -> bool:
        try:
            target_input_topic = _topic(
                input_topic, self._configured_input_topic, self._namespace)
        except (TypeError, ValueError) as exc:
            with self._lock:
                self._last_error = str(exc)
            return False
        with self._lock:
            if self._state == "running":
                if target_input_topic == self._input_topic:
                    return True
                self._last_error = (
                    f"already running from {self._input_topic}; stop before "
                    f"switching to {target_input_topic}"
                )
                return False
            if self._state in ("starting", "stopping"):
                return False
            self._state = "starting"
            self._last_error = None
            self._generation += 1
            generation = self._generation

            if not self._model_path.strip():
                self._state = "error"
                self._last_error = (
                    "food_segmentation.model_path must explicitly name a local "
                    "segmentation checkpoint"
                )
                print(f"[food_segmentation] start failed: {self._last_error}", flush=True)
                return False

        runtime = None
        node = None
        try:
            runtime = self._runtime_factory()
            runtime.load(self._model_path, **self._load_options())
            bindings = (self._ros_bindings_factory()
                        if callable(self._ros_bindings_factory)
                        else self._ros_bindings_factory)
            safe_namespace = re.sub(r"[^A-Za-z0-9_]", "_", self._namespace)
            node = bindings.Node(f"bumi_food_segmentation_{safe_namespace or 'default'}")
            qos = bindings.qos_profile()
            callback_group = bindings.callback_group()
            metadata_publisher = node.create_publisher(
                bindings.String, self._metadata_topic, qos)
            overlay_publisher = node.create_publisher(
                bindings.CompressedImage, self._overlay_topic, qos)
            subscription = node.create_subscription(
                bindings.CompressedImage,
                target_input_topic,
                self._on_image,
                qos,
                callback_group=callback_group,
            )
            self._executor.add_node(node)
        except Exception as exc:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            with self._lock:
                if self._state == "starting" and self._generation == generation:
                    self._state = "error"
                    self._last_error = str(exc)
            print(f"[food_segmentation] start failed: {exc}", flush=True)
            return False

        with self._lock:
            cancelled = (self._state != "starting"
                         or self._generation != generation)
        if cancelled:
            try:
                remove_node = getattr(self._executor, "remove_node", None)
                if callable(remove_node):
                    remove_node(node)
            finally:
                node.destroy_node()
            return False

        with self._lock:
            # The generation check above is repeated while committing state so
            # a stop between validation and this lock cannot be lost.
            if self._state != "starting" or self._generation != generation:
                cancelled = True
            else:
                cancelled = False
                self._input_topic = target_input_topic
                self._bindings = bindings
                self._runtime = runtime
                self._node = node
                self._subscription = subscription
                self._metadata_publisher = metadata_publisher
                self._overlay_publisher = overlay_publisher
                self._running = True
                self._state = "running"
                self._pending = None
                self._worker = threading.Thread(
                    target=self._worker_loop,
                    args=(generation,),
                    daemon=True,
                    name="bumi_food_segmentation_worker",
                )
                self._worker.start()
        if cancelled:
            try:
                remove_node = getattr(self._executor, "remove_node", None)
                if callable(remove_node):
                    remove_node(node)
            finally:
                node.destroy_node()
            return False
        return True

    def _cleanup_ros(self) -> None:
        with self._lock:
            node = self._node
            self._node = None
            self._subscription = None
            self._metadata_publisher = None
            self._overlay_publisher = None
            self._bindings = None
            self._runtime = None
        if node is None:
            return
        try:
            remove_node = getattr(self._executor, "remove_node", None)
            if callable(remove_node):
                remove_node(node)
        except Exception as exc:
            with self._lock:
                self._last_error = f"executor remove_node failed: {exc}"
            print(f"[food_segmentation] cleanup warning: {exc}", flush=True)
        try:
            node.destroy_node()
        except Exception as exc:
            with self._lock:
                self._last_error = f"ROS node destruction failed: {exc}"
            print(f"[food_segmentation] cleanup warning: {exc}", flush=True)

    def stop(self) -> bool:
        with self._condition:
            if self._state == "idle":
                return True
            if self._state == "error" and self._node is None:
                self._state = "idle"
                return True
            self._running = False
            self._generation += 1
            self._state = "stopping"
            self._pending = None
            worker = self._worker
            self._condition.notify_all()
        if worker is not None and worker is not threading.current_thread():
            worker.join(self._stop_timeout_s)
        if worker is not None and worker.is_alive():
            with self._lock:
                self._last_error = (
                    f"inference worker did not stop within {self._stop_timeout_s:g}s; "
                    "ROS resources are retained until it exits"
                )
            return False
        self._cleanup_ros()
        with self._lock:
            self._worker = None
            self._state = "idle"
        return True

    def _timestamp(self, message) -> dict[str, int]:
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        sec = getattr(stamp, "sec", 0)
        nanosec = getattr(stamp, "nanosec", 0)
        return {
            "sec": int(sec) if isinstance(sec, int) else 0,
            "nanosec": int(nanosec) if isinstance(nanosec, int) else 0,
        }

    def _on_image(self, message) -> None:
        try:
            fmt = str(getattr(message, "format", "") or "jpeg").lower()
            if "jpeg" not in fmt and "jpg" not in fmt:
                raise ValueError(f"unsupported CompressedImage format: {fmt!r}")
            raw_data = getattr(message, "data", b"")
            try:
                compressed_size = len(raw_data)
            except TypeError as exc:
                raise ValueError(
                    "CompressedImage data must be a sized byte sequence") from exc
            if compressed_size == 0:
                raise ValueError("CompressedImage JPEG payload is empty")
            if compressed_size > self._max_compressed_bytes:
                raise FrameLimitError(
                    "CompressedImage JPEG payload exceeds max_compressed_bytes "
                    f"({compressed_size} > {self._max_compressed_bytes})")
            data = bytes(raw_data)
            encoded_size = _jpeg_dimensions(data)
            if (encoded_size is not None
                    and encoded_size[0] * encoded_size[1] > self._max_image_pixels):
                raise FrameLimitError(
                    "JPEG header exceeds max_image_pixels "
                    f"({encoded_size[0] * encoded_size[1]} > "
                    f"{self._max_image_pixels})")
            timestamp = self._timestamp(message)
        except (FrameLimitError, SegmentationLimitError) as exc:
            with self._lock:
                self._stats["limit_errors"] += 1
                self._last_error = str(exc)
            return
        except Exception as exc:
            with self._lock:
                self._stats["decode_errors"] += 1
                self._last_error = str(exc)
            return

        with self._condition:
            if not self._running:
                return
            self._stats["received_frames"] += 1
            if self._pending is not None:
                self._stats["dropped_frames"] += 1
            self._pending = (message, data, timestamp, time.time_ns())
            self._last_received_monotonic = time.monotonic()
            self._last_frame_timestamp = timestamp
            self._condition.notify()

    def _worker_loop(self, generation: int) -> None:
        period = 1.0 / self._max_rate_hz
        next_allowed = 0.0
        try:
            while True:
                with self._condition:
                    while self._running and self._pending is None:
                        self._condition.wait(0.5)
                    if not self._running or generation != self._generation:
                        return
                    delay = next_allowed - time.monotonic()
                    if delay > 0:
                        self._condition.wait(delay)
                        continue
                    frame = self._pending
                    self._pending = None
                next_allowed = time.monotonic() + period
                self._process_frame(frame, generation)
        finally:
            with self._lock:
                stopped = not self._running
                if threading.current_thread() is self._worker:
                    self._worker = None
            if stopped:
                self._cleanup_ros()
                with self._lock:
                    if self._state == "stopping":
                        self._state = "idle"

    def _process_frame(self, frame, generation: int) -> None:
        message, data, timestamp, received_ns = frame
        started = time.perf_counter()
        try:
            image = np.asarray(self._codec.decode(data))
            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError("decoded JPEG must be an HxWx3 color image")
            if image.shape[0] <= 0 or image.shape[1] <= 0:
                raise ValueError("decoded JPEG dimensions must be positive")
            pixels = int(image.shape[0]) * int(image.shape[1])
            if pixels > self._max_image_pixels:
                raise FrameLimitError(
                    "decoded JPEG exceeds max_image_pixels "
                    f"({pixels} > {self._max_image_pixels})")
        except FrameLimitError as exc:
            with self._lock:
                self._stats["limit_errors"] += 1
                self._last_error = str(exc)
            return
        except Exception as exc:
            with self._lock:
                self._stats["decode_errors"] += 1
                self._last_error = str(exc)
            return

        try:
            height, width = image.shape[:2]
            with self._lock:
                runtime = self._runtime
            result = FoodSegmentationPipeline(
                runtime,
                [width, height],
                confidence_threshold=self._confidence,
                mask_threshold=self._mask_threshold,
                max_instances=self._max_instances,
                max_total_rle_runs=self._max_total_rle_runs,
            ).process(image)
            processing_ms = (time.perf_counter() - started) * 1000.0
            metadata = {
                "schema_version": SCHEMA_VERSION,
                "frame_timestamp": timestamp,
                "image_size": [width, height],
                "processed_instance_count": len(result["instances"]),
                "instances": result["instances"],
                "errors": result["errors"],
                "processing": {
                    "duration_ms": processing_ms,
                    "received_at_unix_ns": received_ns,
                    "completed_at_unix_ns": time.time_ns(),
                    "runtime": dict(getattr(runtime, "last_diagnostics", {}) or {}),
                },
            }
            metadata_json = _json_dumps_limited(
                metadata, self._max_metadata_bytes)
            overlay = self._render_overlay(image, result)
            overlay_jpeg = self._codec.encode(overlay, self._jpeg_quality)
        except (FrameLimitError, SegmentationLimitError) as exc:
            with self._lock:
                self._stats["limit_errors"] += 1
                self._last_error = str(exc)
            return
        except Exception as exc:
            with self._lock:
                self._stats["inference_errors"] += 1
                self._last_error = str(exc)
            return

        try:
            with self._lock:
                if not self._running or generation != self._generation:
                    return
                bindings = self._bindings
                metadata_message = bindings.String()
                metadata_message.data = metadata_json
                overlay_message = bindings.CompressedImage()
                if hasattr(message, "header"):
                    overlay_message.header = message.header
                overlay_message.format = "jpeg"
                overlay_message.data = overlay_jpeg
                self._metadata_publisher.publish(metadata_message)
                self._overlay_publisher.publish(overlay_message)
                self._stats["processed_frames"] += 1
                self._last_processing_ms = processing_ms
                self._last_error = None
        except Exception as exc:
            with self._lock:
                self._stats["publish_errors"] += 1
                self._last_error = str(exc)

    def _render_overlay(self, image: np.ndarray, result: dict) -> np.ndarray:
        overlay = image.copy()
        for index, instance in enumerate(result["instances"]):
            mask = np.asarray(decode_binary_rle(instance["mask"]), dtype=bool)
            if mask.shape != image.shape[:2]:
                raise ValueError("pipeline RLE mask size does not match decoded image")
            color = _COLORS[index % len(_COLORS)]
            color_array = np.asarray(color, dtype=np.float32)
            overlay[mask] = (
                overlay[mask].astype(np.float32) * 0.55 + color_array * 0.45
            ).astype(np.uint8)
            x1, y1, x2, y2 = instance["bbox_xyxy"]
            point1 = (max(0, int(math.floor(x1))), max(0, int(math.floor(y1))))
            point2 = (
                min(image.shape[1] - 1, int(math.ceil(x2)) - 1),
                min(image.shape[0] - 1, int(math.ceil(y2)) - 1),
            )
            self._codec.rectangle(overlay, point1, point2, color)
            label = f"{instance['class_name']} {instance['confidence']:.3f}"
            self._codec.label(
                overlay, label, (point1[0], max(15, point1[1] - 5)), color)
        return overlay

    def info(self) -> dict:
        with self._lock:
            if self._last_received_monotonic is None:
                input_state = "never_received"
                frame_age = None
            else:
                frame_age = max(0.0, time.monotonic() - self._last_received_monotonic)
                input_state = "fresh" if frame_age <= self._input_timeout_s else "stale"
            return {
                "state": self._state,
                "model_path": self._model_path,
                "runtime_loaded": self._runtime is not None,
                "topic_in": [{"topic": self._input_topic, "format": "image/jpeg"}],
                "topic_out": [
                    {"topic": self._metadata_topic, "format": "data/json"},
                    {"topic": self._overlay_topic, "format": "image/jpeg"},
                ],
                "max_rate_hz": self._max_rate_hz,
                "confidence_threshold": self._confidence,
                "mask_threshold": self._mask_threshold,
                "imgsz": self._imgsz,
                "device": self._device or None,
                "jpeg_quality": self._jpeg_quality,
                "max_compressed_bytes": self._max_compressed_bytes,
                "max_image_pixels": self._max_image_pixels,
                "max_instances": self._max_instances,
                "max_total_rle_runs": self._max_total_rle_runs,
                "max_metadata_bytes": self._max_metadata_bytes,
                "input_state": input_state,
                "last_frame_age_s": frame_age,
                "last_frame_timestamp": self._last_frame_timestamp,
                "last_processing_ms": self._last_processing_ms,
                "last_error": self._last_error,
                **self._stats,
            }

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            input_topic = args.get("input_topic")
            if not input_topic:
                input_topics = args.get("input_topics")
                if isinstance(input_topics, list) and input_topics:
                    input_topic = input_topics[0]
            ok = self.start(input_topic)
            result = self.info()
            result["ok"] = ok
            return result
        if action == "stop":
            ok = self.stop()
            result = self.info()
            result["ok"] = ok
            return result
        if action == "info":
            return self.info()
        return None
