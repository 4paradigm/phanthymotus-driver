"""Helios multi-instance RGB card; shares one producer per selected camera."""

from __future__ import annotations

import hashlib
import re
import threading

from common.camera_info import build
from common.vendor_runtime import action_schema, tool
from .camera_capture import SOURCES, CameraCapture


class RosCameraStream:
    def __init__(self, ros2, namespace, source_id, source, fps):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import CompressedImage

        self.node = Node(f"helios_rgb_{source_id}", context=ros2.ctx_core)
        self._ros2 = ros2
        self._lock = threading.RLock()
        self._publishers = {}
        try:
            self._type = CompressedImage
            self._frame_id = f"helios_{source_id}_color_optical_frame"
            self._qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
            self.capture = CameraCapture(source, self._publish, fps)
            ros2.executor_core.add_node(self.node)
        except Exception:
            self.node.destroy_node()
            raise

    def _publish(self, jpeg, received_ns):
        msg = self._type()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(received_ns, 1_000_000_000)
        msg.header.frame_id = self._frame_id
        msg.format = "jpeg"
        msg.data = jpeg
        with self._lock:
            for publisher in self._publishers.values():
                publisher.publish(msg)

    def attach(self, instance, topic):
        with self._lock:
            if instance not in self._publishers:
                self._publishers[instance] = self.node.create_publisher(self._type, topic, self._qos)

    def detach(self, instance):
        with self._lock:
            publisher = self._publishers.pop(instance, None)
            if publisher is not None:
                self.node.destroy_publisher(publisher)

    def start(self):
        self.capture.start()

    def stop(self):
        self.capture.stop()
        self._ros2.executor_core.remove_node(self.node)
        self.node.destroy_node()

    def snapshot(self):
        return self.capture.snapshot()


class HeliosCamera:
    PREFIX = "camera"

    def __init__(self, config, namespace="rokae_helios", ros2=None, *, stream_factory=None):
        self._ros2 = ros2
        self._namespace = namespace
        self._fps = config.get("camera_publish_fps", 15)
        if isinstance(self._fps, bool) or not isinstance(self._fps, int) or not 1 <= self._fps <= 15:
            raise ValueError("camera_publish_fps must be an integer from 1 to 15")
        if not re.fullmatch(r"[a-zA-Z0-9_]+", namespace):
            raise ValueError("invalid camera namespace")
        self._factory = stream_factory or (
            lambda source_id: RosCameraStream(ros2, namespace, source_id, SOURCES[source_id], self._fps))
        self._lock = threading.RLock()
        self._configs = {}
        self._active = {}  # instance -> source; multiple instances may share a producer.
        self._streams = {}

    def get_tools(self):
        definition = tool("camera_rgb", "sensor",
                          "Helios RGB camera — JPEG with the standard Unitree camera view and FPS. "
                          "Choose one of five upper-body colour cameras: chest, head, head-front fisheye, head-rear-left fisheye, head-rear-right fisheye. "
                          "Capture starts on demand; no robot or obstacle-perception control.",
                          action_schema({"start": ([], "Start the selected RGB stream"),
                                         "stop": ([], "Release this card's RGB stream"),
                                         "info": ([], "Camera sources, live status and optical metadata")}, {}),
                          topic_out=[{"format": "image/jpeg", "desc": "RGB JPEG stream"}])
        definition["multiInstance"] = True
        definition["configSchema"] = {"type": "object", "properties": {
            "camera": {"type": "string", "description": "Camera", "scope": "instance",
                       "default": "chest_rgb", "oneOf": [
                           {"const": name, "title": source["label"]} for name, source in SOURCES.items()]},
        }}
        return [definition]

    def start(self):
        pass  # Driver startup must not open cameras not requested by a canvas card.

    def stop(self):
        with self._lock:
            for stream in list(self._streams.values()):
                stream.stop()
            self._streams.clear()
            self._active.clear()

    @staticmethod
    def _instance(args, *, required=False):
        value = args.get("instance_id", "")
        if not isinstance(value, str) or len(value) > 128 or (required and not value):
            raise ValueError("instance_id is required and must be at most 128 characters")
        return value

    def _source(self, instance, args):
        source_id = args.get("camera", self._configs.get(instance, "chest_rgb"))
        if not isinstance(source_id, str) or source_id not in SOURCES:
            raise ValueError("Unknown Helios camera")
        return source_id

    def _topic(self, instance):
        # Stable when the user selects another camera; no stale Core subscription.
        suffix = hashlib.sha256(instance.encode()).hexdigest()[:24] if instance else "preview"
        return f"/{self._namespace}/camera/card_{suffix}/rgb"

    def _release(self, instance):
        previous = self._active.pop(instance, None)
        if previous:
            self._drop(instance, previous)

    def _drop(self, instance, source_id):
        """Detach a card from a source; stop that producer once no card uses it."""
        self._streams[source_id].detach(instance)
        if source_id not in self._active.values():
            self._streams.pop(source_id).stop()

    def _acquire(self, instance, source_id):
        if self._active.get(instance) == source_id:
            return
        created = source_id not in self._streams
        if created:
            stream = self._factory(source_id)
            try:
                stream.start()  # Only succeeds after a real colour JPEG arrived.
            except Exception:
                stream.stop()
                raise
            self._streams[source_id] = stream
        try:
            self._streams[source_id].attach(instance, self._topic(instance))
        except Exception:
            if created:
                self._streams.pop(source_id).stop()
            raise
        # Switch: record the new source first, so the old one is no longer referenced
        # by this card when deciding whether its producer can be stopped.
        previous = self._active.get(instance)
        self._active[instance] = source_id
        if previous is not None:
            self._drop(instance, previous)

    def _info(self, instance, source_id):
        source = SOURCES[source_id]
        topic = self._topic(instance)
        stream = self._streams.get(source_id)
        snapshot = (stream.snapshot() if instance in self._active and stream else
                    {"state": "idle", "available": False})
        return {**snapshot, "camera": source_id, "label": source["label"],
                "position_mapping_confirmed": True, "target_fps": self._fps,
                "timestamp_source": "driver_receive_time",
                "topic_out": [{"topic": topic, "format": "image/jpeg"}],
                "camera_info": [build(id=f"rokae/helios/{source_id}", topic=topic,
                                      format="image/jpeg", width=snapshot.get("width"),
                                      height=snapshot.get("height"), source="unknown",
                                      pipeline=["v4l2", "videorate", "videoscale", "jpegenc"])],
                "sources": [{"id": key, "label": value["label"], "rgb_backend": "v4l2"}
                            for key, value in SOURCES.items()]}

    def dispatch(self, action, args):
        if args.get("_tool_name", "camera_rgb") != "camera_rgb":
            return None
        with self._lock:
            instance = self._instance(args, required=action in ("start", "stop", "config"))
            source_id = self._source(instance, args)
            if action == "info":
                # Report the source actually in use, including during a failed switch.
                return self._info(instance, self._active.get(instance, source_id))
            if action in ("config", "start"):
                if action == "start" or instance in self._active:
                    self._acquire(instance, source_id)
                self._configs[instance] = source_id
                return {**self._info(instance, source_id),
                        "state": "running" if instance in self._active else "configured"}
            if action == "stop":
                self._release(instance)
                return {**self._info(instance, self._configs.get(instance, source_id)), "state": "idle"}
            raise ValueError(f"Unsupported camera action: {action}")
