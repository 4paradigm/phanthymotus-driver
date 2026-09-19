"""Bounded RGB/depth/detection subscriptions and stationary observation snapshots."""

from collections import deque
import copy
import json
import math
import threading
import time
from uuid import uuid4
import zlib

import numpy as np

from .alignment import align_depth, validate_calibration


INPUT_FORMATS = ("image/jpeg", "image/depth-zlib", "data/json")
INPUT_NAMES = ("RGB 图像", "深度图像", "VOP 物品列表")
MAX_FRAME_AGE = 1.0
MAX_RGBD_SKEW = 0.2
STEADY_SECONDS = 0.6


def resolve_topics(args):
    topics = args.get("input_topics") or ([args["input_topic"]] if args.get("input_topic") else [])
    if not isinstance(topics, list) or len(topics) != 3 or any(not isinstance(t, str) for t in topics):
        raise ValueError("Connect ext_camera rgb, ext_camera depth and VOP objects to the three inputs")
    result = []
    for suffix in ("/rgb", "/depth", "/objects"):
        candidates = [topic.strip() for topic in topics if topic.strip().endswith(suffix)]
        if len(candidates) != 1 or not candidates[0].startswith("/"):
            raise ValueError("Expected one RGB, one depth and one objects input")
        result.append(candidates[0])
    if result[2] != result[0] + "/objects":
        raise ValueError("VOP must consume the same RGB topic connected to pick_place")
    return result


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class ObservationInputs:
    def __init__(self, ros2=None):
        self._ros2 = ros2
        self._condition = threading.Condition(threading.RLock())
        self._topics = []
        self._node = None
        self._generation = 0
        self._buffers = {name: deque(maxlen=90) for name in ("rgb", "depth", "metadata", "objects")}
        self._errors = {}

    def topics(self):
        with self._condition:
            return [
                {"format": fmt, "desc": desc, **({"topic": self._topics[i]} if self._topics else {})}
                for i, (fmt, desc) in enumerate(zip(INPUT_FORMATS, INPUT_NAMES))
            ]

    def start(self, args=None, cancel=None):
        if not args or (cancel is not None and cancel.is_set()):
            return self.info()
        topics = resolve_topics(args)
        with self._condition:
            if topics == self._topics and self._node is not None:
                return self.info()
        if self._ros2 is None:
            raise RuntimeError("ROS context is required for observation inputs")
        self.stop()
        with self._condition:
            if cancel is not None and cancel.is_set():
                return self.info()
            self._topics = topics
            generation = self._generation

        def cancelled():
            return generation != self._generation or (cancel is not None and cancel.is_set())

        node = None
        added = committed = False
        try:
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CompressedImage
            from std_msgs.msg import String

            node = Node("pick_place_inputs_" + uuid4().hex[:8], context=self._ros2.ctx_core)
            for name, topic, kind in (
                ("rgb", topics[0], CompressedImage),
                ("depth", topics[1], CompressedImage),
                ("objects", topics[2], String),
                ("metadata", topics[1] + "/metadata", String),
            ):
                with self._condition:
                    if cancelled():
                        return self.info()
                node.create_subscription(
                    kind,
                    topic,
                    lambda msg, name=name: self.receive(name, msg, generation),
                    qos_profile_sensor_data,
                )
            with self._condition:
                if cancelled():
                    return self.info()
            self._ros2.executor_core.add_node(node)
            added = True
            with self._condition:
                if not cancelled():
                    self._node = node
                    committed = True
            return self.info()
        finally:
            if not committed:
                with self._condition:
                    if generation == self._generation:
                        self._generation += 1
                        self._topics = []
                        self._errors.clear()
                        for buffer in self._buffers.values():
                            buffer.clear()
                        self._condition.notify_all()
                if node is not None:
                    try:
                        if added:
                            self._ros2.executor_core.remove_node(node)
                    finally:
                        node.destroy_node()

    def stop(self):
        with self._condition:
            node, self._node = self._node, None
            self._generation += 1
            self._topics = []
            self._errors.clear()
            for buffer in self._buffers.values():
                buffer.clear()
            self._condition.notify_all()
        if node is not None:
            try:
                self._ros2.executor_core.remove_node(node)
            finally:
                node.destroy_node()

    def receive(self, name, msg, generation=None):
        with self._condition:
            if generation is not None and generation != self._generation:
                return
            now = time.time()
            try:
                if name in ("rgb", "depth"):
                    stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                    if stamp <= 0 or not -0.1 <= now - stamp / 1e9 <= MAX_FRAME_AGE:
                        raise ValueError(f"{name} frame timestamp is stale or invalid")
                    payload = bytes(msg.data)
                    if not payload or len(payload) > 16 * 1024 * 1024:
                        raise ValueError(f"Invalid {name} payload size")
                    if name == "rgb" and (
                        "jpeg" not in msg.format.lower()
                        and "jpg" not in msg.format.lower()
                        or not payload.startswith(b"\xff\xd8")
                    ):
                        raise ValueError("RGB input must be JPEG")
                    if name == "depth" and msg.format != "16UC1; compressedDepth zlib":
                        raise ValueError("Depth input must be zlib-compressed uint16 millimetres")
                    value = {
                        "stamp_ns": stamp,
                        "received_at": now,
                        "data": payload,
                        "frame_id": msg.header.frame_id,
                    }
                else:
                    value = json.loads(msg.data)
                    if not isinstance(value, dict):
                        raise ValueError(f"Invalid {name} payload")
                    if name == "metadata":
                        validate_calibration(value)
                        for key in ("rgb_stamp_ns", "depth_stamp_ns",
                                    "rgb_header_stamp_ns", "depth_header_stamp_ns"):
                            if (
                                type(value.get(key)) is not int
                                or not -0.1 <= now - value[key] / 1e9 <= MAX_FRAME_AGE
                            ):
                                raise ValueError("RGB-D calibration timestamp is stale or invalid")
                        for key in ("rgb_frame_id", "depth_frame_id"):
                            if not isinstance(value.get(key), str) or not value[key]:
                                raise ValueError("RGB-D metadata lacks image frame identity")
                        if abs(value["rgb_stamp_ns"] - value["depth_stamp_ns"]) / 1e9 > MAX_RGBD_SKEW:
                            raise ValueError("RGB and depth acquisition times are too far apart")
                        if not self._topics or self._topics[0] not in value.get("rgb_topics", []):
                            raise ValueError("RGB and depth inputs must come from the same physical camera")
                    else:
                        stamp = value.get("timestamp")
                        latency = value.get("latency_ms")
                        objects = value.get("objects")
                        if not _finite(stamp) or not -0.1 <= now - stamp <= 5:
                            raise ValueError("VOP result timestamp is stale or invalid")
                        if not _finite(latency) or not 0 <= latency <= 5000:
                            raise ValueError("VOP result lacks a valid inference duration")
                        if (
                            not isinstance(objects, list)
                            or len(objects) > 1000
                            or value.get("count") != len(objects)
                        ):
                            raise ValueError("Invalid VOP object list")
                        for obj in objects:
                            position = obj.get("position") if isinstance(obj, dict) else None
                            if (
                                not isinstance(position, list)
                                or len(position) != 2
                                or any(not _finite(v) or not -1 <= v <= 1 for v in position)
                                or not isinstance(obj.get("name"), str)
                                or not _finite(obj.get("confidence"))
                                or not 0 <= obj["confidence"] <= 1
                            ):
                                raise ValueError("Invalid VOP object coordinates or confidence")
                    value["received_at"] = now
                self._buffers[name].append(value)
                self._errors.pop(name, None)
            except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as exc:
                self._errors[name] = str(exc)
            self._condition.notify_all()

    def info(self):
        with self._condition:
            now = time.time()
            missing = [
                name
                for name, values in self._buffers.items()
                if not values or now - values[-1]["received_at"] > (5 if name == "objects" else MAX_FRAME_AGE)
            ]
            error = next(iter(self._errors.values()), None)
            if not self._topics:
                error = "Connect RGB, depth and VOP inputs and start the card"
            return {
                "state": "error" if error else "starting" if missing else "running",
                "fresh": not error and not missing,
                "error": error,
                "missing": missing,
                "topic_in": self.topics(),
            }

    def identity(self):
        with self._condition:
            metadata = self._buffers["metadata"][-1] if self._buffers["metadata"] else {}
            return {
                "topics": list(self._topics),
                "serial_number": metadata.get("serial_number"),
                "session_id": metadata.get("session_id"),
            }

    @staticmethod
    def _thumbnail(payload):
        import cv2

        rgb = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if rgb is None:
            raise ValueError("RGB JPEG cannot be decoded")
        return cv2.resize(rgb, (160, 90), interpolation=cv2.INTER_AREA).astype(np.int16), rgb.shape

    def snapshot(self, after, cancel, check, timeout=12.0):
        deadline = time.monotonic() + timeout
        source = self.identity()
        first_detection = None
        baseline = None
        checked_stamp = 0
        restarts = 0
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise RuntimeError("Observation cancelled")
            if check() is False:
                # Pose/input readiness recovered or is still settling. Discard
                # the entire old window, including its in-flight detections.
                after = time.time() + 0.15
                first_detection = baseline = None
                restarts += 1
                cancel.wait(0.05)
                continue
            with self._condition:
                buffers = {key: list(values) for key, values in self._buffers.items()}
                identity = self.identity()
            if identity != source:
                raise RuntimeError("Observation input source changed")
            # Observe the entire post-settle window, not just its endpoints.
            acquisition = {m["rgb_header_stamp_ns"]: m["rgb_stamp_ns"]
                           for m in buffers["metadata"] if m["session_id"] == source["session_id"]}
            for frame in buffers["rgb"]:
                if frame["stamp_ns"] not in acquisition:
                    continue
                stamp = acquisition[frame["stamp_ns"]] / 1e9
                if stamp <= after or stamp <= checked_stamp:
                    continue
                thumb, shape = self._thumbnail(frame["data"])
                if baseline is None:
                    baseline = thumb
                elif np.mean(np.abs(thumb - baseline) > 20) > 0.005:
                    after, baseline, first_detection = stamp, thumb, None
                    restarts += 1
                checked_stamp = stamp
            detections = [d for d in buffers["objects"] if d["timestamp"] - d["latency_ms"] / 1000 > after]
            if detections and first_detection is None:
                # Drain an inference/queued frame that may predate settling.
                first_detection = detections[0]["timestamp"]
            restart = False
            for objects in reversed(detections):
                started = objects["timestamp"] - objects["latency_ms"] / 1000
                if first_detection is None or started <= first_detection or started < after + STEADY_SECONDS:
                    continue
                candidates = [
                    m
                    for m in buffers["metadata"]
                    if m["session_id"] == identity["session_id"]
                    and min(m["rgb_stamp_ns"], m["depth_stamp_ns"]) / 1e9 > after
                    and abs(m["rgb_stamp_ns"] / 1e9 - started) <= MAX_RGBD_SKEW
                ]
                for metadata in sorted(candidates, key=lambda m: abs(m["rgb_stamp_ns"] / 1e9 - started)):
                    rgb = next((v for v in buffers["rgb"] if v["stamp_ns"] == metadata["rgb_header_stamp_ns"]), None)
                    depth = next(
                        (v for v in buffers["depth"] if v["stamp_ns"] == metadata["depth_header_stamp_ns"]), None
                    )
                    if rgb is None or depth is None:
                        continue
                    if (
                        rgb["frame_id"] != metadata["rgb_frame_id"]
                        or depth["frame_id"] != metadata["depth_frame_id"]
                    ):
                        raise ValueError("RGB-D frame identity does not match calibration")
                    if time.time() - objects["timestamp"] > 1 or time.time() - metadata["rgb_stamp_ns"] / 1e9 > 2:
                        continue
                    intr = metadata["rgb_intrinsics"]
                    _, shape = self._thumbnail(rgb["data"])
                    if shape != (intr["height"], intr["width"]):
                        raise ValueError("RGB dimensions do not match calibration")
                    raw = align_depth(depth["data"], metadata)
                    if check() is False:
                        after = time.time() + 0.15
                        first_detection = baseline = None
                        restarts += 1
                        restart = True
                        break
                    if self.identity() != identity:
                        raise RuntimeError("Observation input source changed")
                    return {
                        "jpeg": rgb["data"],
                        "depth_zlib": zlib.compress(raw.tobytes()),
                        "captured_at": metadata["rgb_stamp_ns"] / 1e9,
                        "depth_captured_at": metadata["depth_stamp_ns"] / 1e9,
                        "width": intr["width"],
                        "height": intr["height"],
                        "intrinsics": copy.deepcopy(intr),
                        "serial_number": metadata["serial_number"],
                        "depth_scale_m": 0.001,
                        "input_identity": identity,
                        "source_calibration": {k: v for k, v in metadata.items() if k != "received_at"},
                        "objects": copy.deepcopy(objects["objects"]),
                        "objects_timestamp": objects["timestamp"],
                        "synchronization": {
                            "mode": "stationary_window",
                            "settled_after": after,
                            "window_restarts": restarts,
                            "rgb_depth_skew_ms": abs(metadata["rgb_stamp_ns"] - metadata["depth_stamp_ns"]) / 1e6,
                        },
                    }
                if restart:
                    break
            cancel.wait(0.05)
        raise RuntimeError("Fresh stationary RGB-D and VOP observation timed out")
