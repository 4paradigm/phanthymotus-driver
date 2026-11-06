"""ROS2 transport for ImageQualityGuard; no camera acquisition or own executor.

Construction is inert. ROS imports and resources are acquired in start(), on
the bundle's initialized context. The subscription copies only the latest
compressed frame; a steady-clock timer performs real JPEG analysis. No image
is saved, transmitted to an external service, or retained as result data.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from types import SimpleNamespace


def _ros_bindings():
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    return SimpleNamespace(
        ok=rclpy.ok, Node=Node, Clock=Clock, ClockType=ClockType,
        TimerGroup=MutuallyExclusiveCallbackGroup, ReceiveGroup=ReentrantCallbackGroup,
        QoSProfile=QoSProfile, ReliabilityPolicy=ReliabilityPolicy,
        HistoryPolicy=HistoryPolicy, DurabilityPolicy=DurabilityPolicy,
        CompressedImage=CompressedImage, String=String,
    )


class ImageQualityRosRuntime:
    """One run's owned resources, with retryable partial-start cleanup.

    ros_bindings and clocks are internal test seams. Production always uses
    real rclpy and the shared executor supplied by BumiDeviceBundle.
    """

    def __init__(self, *, namespace, executor, ros_bindings=None,
                 monotonic=time.monotonic, wall_time=time.time):
        self._namespace, self._executor = namespace, executor
        self._ros = ros_bindings
        self._monotonic, self._wall_time = monotonic, wall_time
        self._lock = threading.RLock()
        self._tick_lock = threading.Lock()
        self._node = self._publisher = self._subscription = self._timer = None
        self._registered = self._active = False
        self._latest = None
        self._received = self._processed = self._overwritten = 0
        self._sequence = self._consumed = 0
        self._analysis_hz = 2.0
        self._run_id = self._input_topic = self._output_topic = ""
        self._gate = None
        self._started_at = self._next_due = 0.0
        self._last_timestamp_ms = None
        self._first_analysis_monotonic = self._last_analysis_monotonic = None
        self._last_error = None

    def start(self, *, input_topic, output_topic, run_id, analysis_hz, run_if_current):
        if self._node is not None:
            raise RuntimeError("runtime already owns resources; close before starting")
        if self._executor is None:
            raise RuntimeError("the bundle ROS2 executor is required")
        if (isinstance(analysis_hz, bool) or not isinstance(analysis_hz, (int, float))
                or not math.isfinite(analysis_hz) or not 0 < analysis_hz <= 30):
            raise ValueError("analysis_hz must be finite and in (0, 30]")
        if not callable(run_if_current) or not isinstance(run_id, str) or not run_id:
            raise ValueError("run identity and callback gate are required")
        ros = self._ros or _ros_bindings()
        self._ros = ros
        context = getattr(self._executor, "context", None)
        if not ros.ok(context=context):
            raise RuntimeError("ROS2 context is not initialized by the bundle")
        self._run_id, self._input_topic, self._output_topic = run_id, input_topic, output_topic
        self._analysis_hz, self._gate = float(analysis_hz), run_if_current
        self._started_at = self._next_due = self._monotonic()
        suffix = hashlib.sha256(run_id.encode()).hexdigest()[:16]
        # Store each resource immediately; the plugin calls close on failure.
        self._node = ros.Node("image_quality_guard_" + suffix,
                              namespace=self._namespace, context=context)
        input_qos = ros.QoSProfile(
            reliability=ros.ReliabilityPolicy.BEST_EFFORT,
            history=ros.HistoryPolicy.KEEP_LAST, depth=1,
            durability=ros.DurabilityPolicy.VOLATILE)
        output_qos = ros.QoSProfile(
            reliability=ros.ReliabilityPolicy.RELIABLE,
            history=ros.HistoryPolicy.KEEP_LAST, depth=1,
            durability=ros.DurabilityPolicy.VOLATILE)
        self._publisher = self._node.create_publisher(ros.String, output_topic, output_qos)
        self._subscription = self._node.create_subscription(
            ros.CompressedImage, input_topic, self._on_image, input_qos,
            callback_group=ros.ReceiveGroup())
        self._timer = self._node.create_timer(
            1.0 / self._analysis_hz, self._tick,
            clock=ros.Clock(clock_type=ros.ClockType.STEADY_TIME),
            callback_group=ros.TimerGroup())
        # Mark ownership before add_node: partial add failures are retryable.
        self._registered = True
        self._executor.add_node(self._node)
        with self._lock:
            self._active = True

    def _on_image(self, message):
        # No JPEG decode, waiting, algorithm or publication in this callback.
        try:
            payload = bytes(message.data)
        except (TypeError, ValueError, AttributeError):
            payload = None
        image_format = getattr(message, "format", None)
        with self._lock:
            if not self._active:
                return
            self._received += 1
            self._sequence += 1
            if self._latest is not None:
                self._overwritten += 1
            self._latest = (self._sequence, image_format, payload)

    def _tick(self):
        if not self._tick_lock.acquire(blocking=False):
            return
        try:
            now = self._monotonic()
            with self._lock:
                if not self._active or self._latest is None or now < self._next_due:
                    return
                sequence, image_format, payload = self._latest
                if sequence <= self._consumed:
                    return
                self._consumed = sequence
                self._latest = None
                self._next_due = now + 1.0 / self._analysis_hz

            def analyze_and_publish(analyzer):
                result = analyzer.analyze_jpeg(payload, image_format=image_format)
                timestamp = self._wall_time()
                timestamp_ms = int(timestamp * 1000) if math.isfinite(timestamp) else None
                result = {"schema": "motus.image_quality/1", "run_id": self._run_id,
                          "timestamp_ms": timestamp_ms, "timestamp_kind": "analysis_time",
                          "input_topic": self._input_topic, **result}
                encoded = json.dumps(result, ensure_ascii=False, allow_nan=False,
                                     separators=(",", ":"))
                with self._lock:
                    if not self._active or self._publisher is None:
                        return
                    self._publisher.publish(self._ros.String(data=encoded))
                    self._processed += 1
                    if self._first_analysis_monotonic is None:
                        self._first_analysis_monotonic = now
                    self._last_analysis_monotonic = now
                    self._last_timestamp_ms = timestamp_ms
                    self._last_error = None

            self._gate(analyze_and_publish)
        except Exception as error:
            # Keep the executor alive and make transport failures observable.
            with self._lock:
                self._last_error = {"code": "RUNTIME_ERROR", "message": str(error)[:400]}
        finally:
            self._tick_lock.release()

    def info(self):
        with self._lock:
            elapsed = ((self._last_analysis_monotonic - self._first_analysis_monotonic)
                       if self._processed > 1 else 0.0)
            return {"analysis_hz": self._analysis_hz, "frames_received": self._received,
                    "analyses_published": self._processed, "frames_overwritten": self._overwritten,
                    "effective_analysis_hz": (self._processed - 1) / elapsed if elapsed else 0.0,
                    "last_analysis_timestamp_ms": self._last_timestamp_ms,
                    "error": dict(self._last_error) if self._last_error else None}

    def close(self):
        """Invalidate first; attempt all cleanup and retain failures for retry."""
        with self._lock:
            self._active = False
            self._latest = None
        node = self._node
        if node is None:
            return
        errors = []
        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception as error:
                errors.append("cancel timer: " + str(error))
        for attribute, destroy in (("_timer", "destroy_timer"),
                                   ("_subscription", "destroy_subscription"),
                                   ("_publisher", "destroy_publisher")):
            resource = getattr(self, attribute)
            if resource is None:
                continue
            try:
                if getattr(node, destroy)(resource) is False:
                    raise RuntimeError("ROS2 refused resource destruction")
                setattr(self, attribute, None)
            except Exception as error:
                errors.append(destroy + ": " + str(error))
        if self._registered:
            try:
                self._executor.remove_node(node)
                self._registered = False
            except Exception as error:
                errors.append("remove node: " + str(error))
        if not errors:
            try:
                node.destroy_node()
                self._node = None
            except Exception as error:
                errors.append("destroy node: " + str(error))
        if errors:
            raise RuntimeError("; ".join(errors)[:1000])
