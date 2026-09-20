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

from .alignment import align_depth, decode_depth
from .calibration import read_calibration


INPUT_FORMATS = ("image/depth-zlib", "image/jpeg", "data/json")
INPUT_NAMES = ("深度图像", "RGB 图像", "VOP 物品列表")
MAX_FRAME_AGE = 1.0
MAX_RGBD_SKEW = 0.2
STEADY_SECONDS = 0.6
MAX_OBJECTS_PAYLOAD_BYTES = 256 * 1024
MAX_OBJECT_NAME_BYTES = 256


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
        raise ValueError("VOP must consume the same RGB topic connected to vision_pick_and_drop")
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
        self._buffers = {name: deque(maxlen=90) for name in ("rgb", "depth", "objects")}
        self._errors = {}
        self._calibration = None
        self._source_key = None
        self._session_id = uuid4().hex
        self._reader_stop = None

    def topics(self):
        with self._condition:
            return [
                {"format": fmt, "desc": desc, **({"topic": self._topics[(1, 0, 2)[i]]} if self._topics else {})}
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
        committed = False
        try:
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CompressedImage
            from std_msgs.msg import String

            node = Node("vision_pick_and_drop_inputs_" + uuid4().hex[:8], context=self._ros2.ctx_core)
            subscriptions = []
            for name, topic, kind in (
                ("rgb", topics[0], CompressedImage),
                ("depth", topics[1], CompressedImage),
                ("objects", topics[2], String),
            ):
                with self._condition:
                    if cancelled():
                        return self.info()
                sub = node.create_subscription(kind, topic, lambda msg: None, qos_profile_sensor_data)
                subscriptions.append((name, kind, sub))
            with self._condition:
                if not cancelled():
                    stopped = threading.Event()
                    worker = threading.Thread(target=self._read, args=(node, subscriptions, generation, stopped),
                                              name="pick-place-inputs", daemon=True)
                    worker.start()
                    self._node, self._reader_stop = node, stopped
                    committed = True
            return self.info()
        finally:
            if not committed:
                with self._condition:
                    if generation == self._generation:
                        self.stop()
                if node is not None:
                    node.destroy_node()

    def _reset_frames(self):
        self._session_id = uuid4().hex
        for buffer in self._buffers.values():
            buffer.clear()
        self._condition.notify_all()

    def stop(self):
        with self._condition:
            if self._reader_stop is not None:
                self._reader_stop.set()
            self._node = self._reader_stop = None
            self._generation += 1
            self._topics = []
            self._source_key = self._calibration = None
            self._errors.clear()
            self._reset_frames()
        # The reader owns destruction: never destroy a handle during take_message
        # or wait for a blocked SDK query while holding the lifecycle/action lock.

    def _refresh_source(self, node, generation):
        with self._condition:
            if generation != self._generation:
                return
            topics = list(self._topics)
        endpoints = [node.get_publishers_info_by_topic(t) for t in topics]
        key = None
        error = None
        if any(len(entries) > 1 for entries in endpoints):
            error = "Each observation input must have exactly one publisher"
        elif all(len(entries) == 1 for entries in endpoints):
            key = tuple((e[0].node_namespace, e[0].node_name, tuple(e[0].endpoint_gid)) for e in endpoints)
            if key[0][:2] != key[1][:2]:
                error = "RGB and depth inputs must come from the same physical camera publisher"
                key = None
        with self._condition:
            if generation != self._generation:
                return
            if key != self._source_key:
                self._source_key, self._calibration = key, None
                self._errors.clear()
                self._reset_frames()
            if error:
                self._errors["source"] = error
            else:
                self._errors.pop("source", None)

    def _initialize_calibration(self, generation):
        with self._condition:
            if (generation != self._generation or self._calibration is not None
                    or not self._source_key or "calibration" in self._errors
                    or not self._buffers["rgb"] or not self._buffers["depth"]):
                return
            source, session = self._source_key, self._session_id
            rgb, depth = self._buffers["rgb"][-1], self._buffers["depth"][-1]
        calibration, error = None, None
        try:
            _, shape = self._thumbnail(rgb["data"])
            calibration = read_calibration(source[0][1], shape, depth["data"], session)
        except Exception as exc:
            error = f"Camera calibration unavailable; restart the card after correcting inputs: {exc}"
        with self._condition:
            if generation == self._generation and source == self._source_key and session == self._session_id:
                self._calibration = calibration
                if error:
                    self._errors["calibration"] = error
                self._condition.notify_all()

    def _read(self, node, subscriptions, generation, stopped):
        # The installed executor discards MessageInfo. Take only our own three
        # subscriptions here, preserving DDS source timestamps without changing
        # the shared executor or registering this node with it.
        next_graph_check = 0
        try:
            while not stopped.is_set():
                if time.monotonic() >= next_graph_check:
                    self._refresh_source(node, generation)
                    next_graph_check = time.monotonic() + 0.2
                for name, kind, sub in subscriptions:
                    for _ in range(8):
                        if stopped.is_set():
                            return
                        with sub.handle:
                            pair = sub.handle.take_message(kind, False)
                        if pair is None:
                            break
                        msg, info = pair
                        self.receive(name, msg, generation, info)
                self._initialize_calibration(generation)
                stopped.wait(0.02)
        except Exception as exc:
            with self._condition:
                if generation == self._generation:
                    self._errors["receiver"] = str(exc)
                    self._reset_frames()
        finally:
            node.destroy_node()

    def receive(self, name, msg, generation=None, message_info=None):
        with self._condition:
            if generation is not None and generation != self._generation:
                return
            if not self._source_key:
                return
            now = time.time()
            self._expire_frames(now)
            try:
                if name in ("rgb", "depth"):
                    stamp = (message_info.get("source_timestamp") if isinstance(message_info, dict)
                             else getattr(message_info, "source_timestamp", None))
                    if type(stamp) is not int or stamp <= 0 or not -0.1 <= now - stamp / 1e9 <= MAX_FRAME_AGE:
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
                    prefix = self._source_key[0][1].rsplit("_realsense_rgbd_", 1)[0]
                    if msg.header.frame_id != f"{prefix}_{name}_optical":
                        raise ValueError("Image frame does not match the camera publisher")
                    if self._calibration is not None:
                        intr = self._calibration[name + "_intrinsics"]
                        try:
                            if name == "rgb":
                                _, shape = self._thumbnail(payload)
                                if shape != (intr["height"], intr["width"]):
                                    raise ValueError("RGB dimensions changed")
                            else:
                                decode_depth(payload, intr["width"], intr["height"])
                        except ValueError:
                            self._reset_frames()
                            self._errors["calibration"] = "Image dimensions changed or invalid depth; restart the card"
                            raise
                    previous = self._buffers[name]
                    if previous and stamp <= previous[-1]["stamp_ns"]:
                        return
                    value = {
                        "stamp_ns": stamp,
                        "header_stamp_ns": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                        "received_at": now,
                        "data": payload,
                        "frame_id": msg.header.frame_id,
                    }
                else:
                    payload = msg.data
                    # Bound character count before allocating its UTF-8 encoding.
                    if (not isinstance(payload, str) or len(payload) > MAX_OBJECTS_PAYLOAD_BYTES
                            or len(payload.encode("utf-8")) > MAX_OBJECTS_PAYLOAD_BYTES):
                        raise ValueError("Invalid VOP payload size; maximum is 262144 UTF-8 bytes")
                    value = json.loads(payload)
                    if not isinstance(value, dict):
                        raise ValueError(f"Invalid {name} payload")
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
                            or len(obj["name"]) > MAX_OBJECT_NAME_BYTES
                            or len(obj["name"].encode("utf-8")) > MAX_OBJECT_NAME_BYTES
                            or not _finite(obj.get("confidence"))
                            or not 0 <= obj["confidence"] <= 1
                        ):
                            raise ValueError("Invalid VOP object name (maximum 256 UTF-8 bytes), coordinates or confidence")
                    value = {"timestamp": stamp, "latency_ms": latency, "count": len(objects),
                             "objects": [{key: obj[key] for key in ("name", "position", "confidence")}
                                         for obj in objects], "received_at": now}
                self._buffers[name].append(value)
                self._errors.pop(name, None)
            except (ValueError, TypeError, KeyError, AttributeError, OverflowError, RecursionError, zlib.error) as exc:
                self._errors[name] = str(exc)
            self._condition.notify_all()

    def _expire_frames(self, now):
        if any(values and now - values[-1]["stamp_ns"] / 1e9 > MAX_FRAME_AGE
               for name, values in self._buffers.items() if name in ("rgb", "depth")):
            self._reset_frames()

    def info(self):
        with self._condition:
            now = time.time()
            self._expire_frames(now)
            missing = [
                name
                for name, values in self._buffers.items()
                if not values or now - values[-1]["received_at"] > (5 if name == "objects" else MAX_FRAME_AGE)
            ]
            if self._calibration is None:
                missing.append("calibration")
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
            self._expire_frames(time.time())
            return {
                "topics": list(self._topics),
                "serial_number": (self._calibration or {}).get("serial_number"),
                "session_id": self._session_id,
            }

    @staticmethod
    def _thumbnail(payload):
        import cv2

        rgb = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if rgb is None:
            raise ValueError("RGB JPEG cannot be decoded")
        return cv2.resize(rgb, (160, 90), interpolation=cv2.INTER_AREA).astype(np.int16), rgb.shape

    def snapshot(self, after, cancel, check):
        source = self.identity()
        first_detection = None
        baseline = None
        checked_stamp = 0
        restarts = 0
        while True:
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
                for name, values in self._buffers.items():
                    while values and (values[0]["timestamp"] if name == "objects"
                                      else values[0]["stamp_ns"] / 1e9) <= after:
                        values.popleft()
                buffers = {key: list(values) for key, values in self._buffers.items()}
                identity = self.identity()
                calibration = copy.deepcopy(self._calibration)
            if identity != source:
                raise RuntimeError("Observation input source changed")
            # Observe the entire post-settle window, not just its endpoints.
            for frame in buffers["rgb"]:
                stamp = frame["stamp_ns"] / 1e9
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
                if calibration is None:
                    continue
                candidates = [rgb for rgb in buffers["rgb"] if rgb["stamp_ns"] / 1e9 > after
                              and abs(rgb["stamp_ns"] / 1e9 - started) <= MAX_RGBD_SKEW]
                for rgb in sorted(candidates, key=lambda r: abs(r["stamp_ns"] / 1e9 - started)):
                    depths = [d for d in buffers["depth"] if d["stamp_ns"] / 1e9 > after
                              and abs(d["stamp_ns"] - rgb["stamp_ns"]) / 1e9 <= MAX_RGBD_SKEW]
                    if not depths:
                        continue
                    depth = min(depths, key=lambda d: abs(d["stamp_ns"] - rgb["stamp_ns"]))
                    if time.time() - objects["timestamp"] > 1 or time.time() - rgb["stamp_ns"] / 1e9 > MAX_FRAME_AGE:
                        continue
                    metadata = dict(calibration, session_id=identity["session_id"], rgb_topics=[source["topics"][0]],
                                    rgb_stamp_ns=rgb["stamp_ns"], depth_stamp_ns=depth["stamp_ns"],
                                    rgb_header_stamp_ns=rgb["header_stamp_ns"],
                                    depth_header_stamp_ns=depth["header_stamp_ns"],
                                    rgb_frame_id=rgb["frame_id"], depth_frame_id=depth["frame_id"])
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
                            "timestamp_source": "dds_source_timestamp",
                            "capture_time_approximate": True,
                            "settled_after": after,
                            "window_restarts": restarts,
                            "rgb_depth_skew_ms": abs(metadata["rgb_stamp_ns"] - metadata["depth_stamp_ns"]) / 1e6,
                        },
                    }
                if restart:
                    break
            cancel.wait(0.05)
