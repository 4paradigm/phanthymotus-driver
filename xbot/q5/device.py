"""Verified non-motion cards for the RobotEra Q5 bundle.

Direct base, arm, head, and hand cards live in ``direct_control.py``. This
module contains the verified state, battery, audio, D455 camera, and SLAM map
cards.
"""

from __future__ import annotations

import json
import math
import struct
import threading
import time

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import AudioPlay
from xbot_common_interfaces.srv import SetVolume

# main.py resolves all card classes through this module. Keep the direct
# control cards here as explicit exports while their implementation remains
# consolidated in direct_control.py.
from direct_control import (
    ArmControlPlugin,
    BaseDrivePlugin,
    HandControlPlugin,
    HandGesturePlugin,
    HeadControlPlugin,
)


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class _Q5MediaPlugin:
    """Base for read-only Domain-211 media subscriptions.

    The developer container owns the D455 and SLAM services. These cards only
    subscribe to their DDS output and send bounded, already-processed payloads
    to the existing Domain-42 bridge worker.
    """

    def __init__(self, plugin_config, namespace, executor, client):
        self._ns = namespace
        self._client = client
        self._executor = executor
        self._running = False
        self._last_sent = 0.0
        self._max_hz = max(0.1, float(plugin_config.get("max_hz", 10.0)))
        self._subscription = None
        self._node = Node(self._node_name)
        executor.add_node(self._node)

    def _send_media(self, payload):
        sender = getattr(self._client, "publish_media", None)
        if callable(sender):
            sender(payload)

    def stop(self):
        self._running = False

    def dispatch(self, action, args):
        del args
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "start":
            self.start()
        if action in ("start", "info"):
            return {
                "state": "running" if self._running else "idle",
                "source_topic": self._source_topic,
                "topic_out": [{"topic": self._topic, "format": self._format}],
            }
        return None


class CameraRgbPlugin(_Q5MediaPlugin):
    """D455 RGB to JPEG, throttled before crossing into Agent Core's DDS domain."""

    _node_name = "q5_camera_rgb"
    _format = "image/jpeg"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get("source_topic", "/camera/camera/color/image_raw"))
        self._topic = f"/{namespace}/q5/camera/rgb"
        self._jpeg_quality = max(20, min(95, int(plugin_config.get("jpeg_quality", 70))))
        self._latest = None
        self._lock = threading.Lock()
        self._encoder = None
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_rgb", "type": "sensor", "multiInstance": False,
            "description": "Q5 D455 RGB camera. The developer-container RealSense driver must already be running.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
        }

    def start(self):
        if self._running:
            return
        import cv2
        import numpy as np
        from sensor_msgs.msg import Image

        self._cv2, self._np = cv2, np
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_image, _LATEST_QOS)
        self._encoder = threading.Thread(target=self._encode_loop, daemon=True, name="q5_rgb_encoder")
        self._encoder.start()
        print(f"[CameraRgbPlugin] subscribed {self._source_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _on_image(self, msg):
        if self._running:
            with self._lock:
                self._latest = msg

    def _encode_loop(self):
        while self._running:
            with self._lock:
                msg, self._latest = self._latest, None
            if msg is None or time.monotonic() - self._last_sent < 1.0 / self._max_hz:
                time.sleep(0.005)
                continue
            try:
                channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
                if channels is None or msg.step < msg.width * channels:
                    continue
                raw = self._np.frombuffer(msg.data, dtype=self._np.uint8)
                image = raw[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * channels]
                image = image.reshape(msg.height, msg.width, channels)
                if msg.encoding == "rgb8":
                    image = self._cv2.cvtColor(image, self._cv2.COLOR_RGB2BGR)
                elif msg.encoding == "rgba8":
                    image = self._cv2.cvtColor(image, self._cv2.COLOR_RGBA2BGR)
                elif msg.encoding == "bgra8":
                    image = self._cv2.cvtColor(image, self._cv2.COLOR_BGRA2BGR)
                ok, jpeg = self._cv2.imencode(".jpg", image, [self._cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
                if ok:
                    self._send_media({"kind": "rgb", "data": jpeg.tobytes()})
                    self._last_sent = time.monotonic()
            except Exception as exc:
                self._node.get_logger().warn(f"RGB encode failed: {exc}")


class CameraDepthPlugin(_Q5MediaPlugin):
    """D455 aligned depth image, preserving the source Z16/16UC1 measurement unit."""

    _node_name = "q5_camera_depth"
    _format = "image/depth-z16"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get("source_topic", "/camera/camera/aligned_depth_to_color/image_raw"))
        self._topic = f"/{namespace}/q5/camera/depth"
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_depth", "type": "sensor", "multiInstance": False,
            "description": "Q5 D455 depth image, aligned to RGB. Values remain Z16/16UC1 millimetres.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
        }

    def start(self):
        if self._running:
            return
        from sensor_msgs.msg import Image
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_depth, _LATEST_QOS)
        print(f"[CameraDepthPlugin] subscribed {self._source_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _on_depth(self, msg):
        if (not self._running or msg.encoding not in ("16UC1", "mono16") or
                time.monotonic() - self._last_sent < 1.0 / self._max_hz):
            return
        needed = int(msg.height) * int(msg.step)
        if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * 2 or len(msg.data) < needed:
            return
        self._send_media({"kind": "depth", "height": msg.height, "width": msg.width,
                          "encoding": "16UC1", "is_bigendian": msg.is_bigendian,
                          "step": msg.step, "data": bytes(msg.data[:needed])})
        self._last_sent = time.monotonic()


class SlamPointCloudPlugin(_Q5MediaPlugin):
    """Q5 SLAM map cloud, separate from a real-time lidar/obstacle stream."""

    _node_name = "q5_slam_pointcloud"
    _format = "sensor/pointcloud"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get("source_topic", "/slam/map_cmap"))
        self._topic = f"/{namespace}/q5/slam/pointcloud"
        self._max_points = max(100, min(100000, int(plugin_config.get("max_points", 10000))))
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "slam_pointcloud", "type": "sensor", "multiInstance": False,
            "description": "Q5 SLAM map point cloud (not a real-time obstacle/lidar stream), limited for dashboard rendering.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
        }

    def start(self):
        if self._running:
            return
        from sensor_msgs.msg import PointCloud2
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                PointCloud2, self._source_topic, self._on_cloud, _LATEST_QOS)
        print(f"[SlamPointCloudPlugin] subscribed {self._source_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _on_cloud(self, msg):
        if not self._running or time.monotonic() - self._last_sent < 1.0 / self._max_hz:
            return
        point_step = int(msg.point_step)
        total = int(msg.width) * int(msg.height)
        raw = bytes(msg.data)
        if point_step < 12 or total <= 0 or len(raw) < point_step * total:
            return
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            return
        # sensor_msgs/PointField.FLOAT32. A map with another scalar format is
        # rejected rather than silently publishing corrupted coordinates.
        if any(int(fields[name].datatype) != 7 for name in ("x", "y", "z")):
            self._node.get_logger().warn("SLAM point cloud x/y/z are not float32")
            return
        endian = ">" if msg.is_bigendian else "<"
        stride = max(1, math.ceil(total / self._max_points))
        selected = bytearray()
        for index in range(0, total, stride):
            start = index * point_step
            x, y, z = (struct.unpack_from(endian + "f", raw, start + fields[name].offset)[0]
                       for name in ("x", "y", "z"))
            if all(math.isfinite(value) for value in (x, y, z)):
                # Agent Core expects tightly packed XYZ float32 records.
                selected.extend(struct.pack("<fff", x, y, z))
        count = len(selected) // 12
        if count:
            self._send_media({"kind": "pointcloud", "point_step": 12,
                              "count": count, "data": bytes(selected)})
            self._last_sent = time.monotonic()


def _wait_for_future(future, timeout_sec: float):
    """Wait for work completed by main.py's shared executor thread."""
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.result() if future.done() else None


class StatePlugin:
    """Read-only joint and Q5 FSM state card."""

    def __init__(self, plugin_config, namespace, executor, client):
        del plugin_config, executor
        self._ns = namespace
        self._client = client

    def get_tool(self):
        return {
            "name": "state", "type": "sensor", "multiInstance": False,
            "description": "Q5 joint feedback and READY/ACTIVE state.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            # q5_bridge_worker publishes these JSON topics in Agent Core's DDS domain.
            "topic_out": [
                {"topic": f"/{self._ns}/q5/joints_state", "format": "data/json"},
                {"topic": f"/{self._ns}/q5/robot_status", "format": "data/json"},
            ],
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        del args
        if action == "stop":
            return {"state": "idle"}
        if action not in ("start", "info"):
            return None
        joint = self._client.snapshot()
        status = self._client.sensor_snapshot("robot_status")
        if not status.get("available"):
            status = self._client.sensor_snapshot("query_state")
        return {
            "state": "running",
            "joint_state": {
                "available": joint.get("available", False),
                "fresh": joint.get("fresh", False),
                "age_ms": joint.get("age_ms"),
                "joint_count": joint.get("joint_count", 0),
                "position_unit": joint.get("position_unit", "rad"),
            },
            "robot_status": {
                "available": status.get("available", False),
                "fresh": status.get("fresh", False),
                "age_ms": status.get("age_ms"),
                "state": status.get("state"),
                "message": status.get("message", ""),
                "source": status.get("source_service", "/xbot_state"),
            },
            "motion_manager_lifecycle": self._client.get_lifecycle_state(),
        }


class BatteryPlugin:
    """Read-only battery state card, including verified board firmware."""

    def __init__(self, plugin_config, namespace, executor, client):
        del plugin_config, executor
        self._ns = namespace
        self._client = client

    def get_tool(self):
        return {
            "name": "battery", "type": "sensor", "multiInstance": False,
            "description": "Q5 battery level, electrical readings, and power-board firmware.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": f"/{self._ns}/battery_state", "format": "data/json"}],
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        del args
        if action == "stop":
            return {"state": "idle"}
        if action not in ("start", "info"):
            return None
        battery = self._client.sensor_snapshot("battery")
        firmware = self._client.sensor_snapshot("battery_version")
        return {
            "state": "running",
            "available": battery.get("available", False),
            "fresh": battery.get("fresh", False),
            "age_ms": battery.get("age_ms"),
            "percentage": battery.get("percentage"),
            "voltage_v": battery.get("voltage"),
            "current_a": battery.get("current"),
            "temperature_c": battery.get("temperature"),
            "power_supply_status": battery.get("power_supply_status"),
            "firmware": firmware.get("components", {}),
        }


class AudioPlugin:
    """Vendor audio playback via /audio_player/play and paired services."""

    def __init__(self, plugin_config, namespace, executor, client):
        del namespace, client
        self._node = Node("q5_audio")
        executor.add_node(self._node)
        self._action_client = ActionClient(self._node, AudioPlay, "/audio_player/play")
        self._srv_volume = self._node.create_client(SetVolume, "/audio_player/set_volume")
        self._srv_stop = self._node.create_client(Trigger, "/audio_player/stop_play")
        self._srv_is_play = self._node.create_client(Trigger, "/audio_player/is_play")
        self._device = plugin_config.get("device", "plughw:2,0")

    def get_tool(self):
        return {
            "name": "audio", "type": "actuator", "multiInstance": False,
            "description": "Q5 vendor audio playback, volume, stop, and status.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "play", "set_volume", "stop_audio", "is_play", "info"]},
                "mode": {"type": "integer", "enum": [0, 1, 2, 3], "description": "0=id, 1=path, 2=item JSON, 3=file name"},
                "id": {"type": "integer"}, "path": {"type": "string"}, "item": {"type": "string"},
                "file_name": {"type": "string"}, "force_play": {"type": "boolean"},
                "timeout": {"type": "integer", "minimum": 0},
                "channel": {"type": "string", "enum": ["default", "channel1", "channel2", "channel3"]},
                "version": {"type": "string", "enum": ["v1", "v2"]},
                "volume": {"type": "integer", "minimum": 0, "maximum": 100},
            }, "required": ["action"], "additionalProperties": False},
        }

    def start(self):
        pass

    def stop(self):
        self._stop_audio()

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready", "action_server": "/audio_player/play", "device": self._device}
        if action == "play":
            return self._play(args)
        if action == "set_volume":
            return self._set_volume(args.get("volume", 50))
        if action == "stop_audio":
            return self._stop_audio()
        if action == "is_play":
            return self._is_playing()
        if action == "stop":
            self._stop_audio()
            return {"state": "idle"}
        return None

    def _play(self, args):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": "/audio_player/play is unavailable"}
        goal = AudioPlay.Goal()
        goal.mode = int(args.get("mode", 1))
        goal.force_play = bool(args.get("force_play", False))
        goal.id = int(args.get("id", 0))
        goal.path = str(args.get("path", ""))
        goal.item = str(args.get("item", ""))
        goal.file_name = str(args.get("file_name", ""))
        goal.channel = str(args.get("channel", "default"))
        goal.timeout = int(args.get("timeout", 0))
        goal.version = str(args.get("version", "v1"))
        goal_handle = _wait_for_future(self._action_client.send_goal_async(goal), 5.0)
        if goal_handle is None:
            return {"state": "error", "message": "audio goal timed out"}
        if not goal_handle.accepted:
            return {"state": "error", "message": "audio goal rejected"}
        response = _wait_for_future(goal_handle.get_result_async(), max(10.0, goal.timeout + 2.0))
        if response is None:
            return {"state": "error", "message": "audio result timed out"}
        return {"state": "ok" if response.result.success else "error", "message": response.result.message}

    def _set_volume(self, value):
        if not self._srv_volume.service_is_ready():
            return {"state": "error", "message": "/audio_player/set_volume is unavailable"}
        req = SetVolume.Request()
        req.volume = max(0, min(100, int(value)))
        response = _wait_for_future(self._srv_volume.call_async(req), 2.0)
        if response is None:
            return {"state": "error", "message": "set-volume request timed out"}
        return {"state": "ok" if response.success else "error", "volume": req.volume, "message": response.message}

    def _stop_audio(self):
        if not self._srv_stop.service_is_ready():
            return {"state": "error", "message": "/audio_player/stop_play is unavailable"}
        response = _wait_for_future(self._srv_stop.call_async(Trigger.Request()), 2.0)
        if response is None:
            return {"state": "error", "message": "stop-audio request timed out"}
        return {"state": "ok" if response.success else "error", "message": response.message}

    def _is_playing(self):
        if not self._srv_is_play.service_is_ready():
            return {"state": "error", "message": "/audio_player/is_play is unavailable"}
        response = _wait_for_future(self._srv_is_play.call_async(Trigger.Request()), 2.0)
        if response is None:
            return {"state": "error", "message": "is-play request timed out"}
        return {"state": "ok", "is_playing": response.success, "message": response.message}
