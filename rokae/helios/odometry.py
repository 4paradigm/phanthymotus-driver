"""Vendor odometry pose and body-frame speed, in the loco_state layout."""

from __future__ import annotations

import json
import math
import os
import threading

from google.protobuf.message import DecodeError

from common.odom import build_interface, build_sample
from common.vendor_runtime import tool
from .chassis_pose_pb2 import UiDatagram
from .feedback import StreamFeedbackClient, positive_number, validate_url
from .sdk_odometry import SdkOdometryClient


SPEED_AXES = ("vx", "vy", "wz")


def parse_speed_frame(raw):
    """Decode current_state.speed and its enclosing chassis timestamp."""
    message = UiDatagram()
    try:
        message.ParseFromString(raw)
    except DecodeError as exc:
        raise ValueError("invalid chassis state Protobuf frame") from exc
    if not message.HasField("current_state"):
        return None
    state = message.current_state
    stamp = state.timeStamp if state.timeStamp > 0 else None
    speed = None
    if state.HasField("speed") and stamp is not None:
        # Proto3 omits zero-valued scalars. Presence of the speed message
        # distinguishes a real stationary observation from a missing speed.
        speed = [value if math.isfinite(value) else None
                 for value in (state.speed.vx, state.speed.vy, state.speed.w)]
    return {"source_stamp_ms": stamp, "speed": speed}


class SpeedFeedbackClient(StreamFeedbackClient):
    def __init__(self, url, **kwargs):
        self._last_stamp = None
        super().__init__(url, parser=self._parse, data_key="speed_feedback", binary=True,
                         max_msg_size=8 * 1024 * 1024, **kwargs)

    def _parse(self, raw):
        feedback = parse_speed_frame(raw)
        if feedback is None:
            return None
        stamp = feedback["source_stamp_ms"]
        if stamp is not None and self._last_stamp is not None and stamp <= self._last_stamp:
            # Replayed frames must not keep a frozen observation fresh.
            return None
        if stamp is not None:
            self._last_stamp = stamp
        return feedback

    def _on_connect(self):
        self._last_stamp = None


class HeliosOdometry:
    PREFIX = "odometry"

    def __init__(self, config, namespace="rokae_helios", ros2=None, client=None, native=None):
        self._config = config
        self._ros2 = ros2
        self._node = None
        self._publishers = []
        self._lock = threading.RLock()
        self._hz = positive_number(config.get("odometry_publish_hz", 10), "odometry_publish_hz")
        if self._hz > 30:
            raise ValueError("odometry_publish_hz must not exceed 30")
        self._frame = f"{namespace}/odom"
        self._child = f"{namespace}/chassis_planar"
        self._topics = [
            {"topic": f"/{namespace}/loco/state", "format": "data/json"},
            {"topic": f"/{namespace}/state/odom", "format": "state/odom"},
        ]
        url = (os.getenv("HELIOS_ODOMETRY_STATE_URL") or config.get("odometry_state_url")
               or config.get("battery_state_url"))
        self.client = client or self._new_client(url)
        self._native = native if native is not None else SdkOdometryClient(config, self.client.stale_timeout)

    def _new_client(self, url):
        return SpeedFeedbackClient(
            url, stale_timeout_s=self._config.get("odometry_stale_timeout_s", 1),
            connect_timeout_s=self._config.get("connect_timeout_s", 3),
            reconnect_delay_s=self._config.get("reconnect_delay_s", 1),
        )

    def _odom_interface(self):
        return build_interface(
            provides=SPEED_AXES, frame="body", rate_hz=self._hz,
            pose_drift="unbounded" if self._native.installed else "none",
            timing={"stamp_source": "received"},
        )

    def get_tools(self):
        sensor = tool(
            "odometry", "sensor",
            "Helios 里程计，持续显示原厂底盘坐标中的 x、y、yaw 和机体系速度。"
            "位姿由原厂 SDK 提供，单位为米和弧度。标准端口同时提供 Navi 速度反馈。",
            topic_out=self._topics,
        )
        sensor["configSchema"] = {"type": "object", "properties": {
            "odometry_state_url": {"type": "string", "scope": "shared", "x-sensitive": True,
                                   "description": "底盘只读状态 WebSocket 地址（留空使用部署配置）"},
        }}
        return [sensor]

    def start(self):
        with self._lock:
            if self._node is None and self._ros2 is not None:
                from rclpy.node import Node
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
                from std_msgs.msg import String
                node = Node("helios_odometry", context=self._ros2.ctx_core)
                try:
                    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
                    self._publishers = [node.create_publisher(String, t["topic"], qos) for t in self._topics]
                    self._message_type = String
                    node.create_timer(1 / self._hz, self._publish)
                    self._ros2.executor_core.add_node(node)
                    self._node = node
                except Exception:
                    self._publishers = []
                    node.destroy_node()
                    raise
            self.client.start()
            self._native.start()

    def stop(self):
        with self._lock:
            self.client.stop()
            self._native.stop()
            if self._node is not None:
                self._ros2.executor_core.remove_node(self._node)
                self._node.destroy_node()
                self._node = None
            self._publishers = []

    def payload(self):
        with self._lock:
            return self._snapshot()

    def _snapshot(self):
        snap = self.client.snapshot()
        raw = snap["speed_feedback"] if snap["available"] else None
        speed = raw.get("speed") if raw else None
        vx, vy, wz = speed if speed is not None else (None, None, None)
        native = self._native.snapshot()
        speed_available = all(value is not None for value in (vx, vy, wz))
        available = native is not None and speed_available
        return {
            "position": [native["x"], native["y"], None] if native else None,
            "rpy": [None, None, native["yaw"]] if native else None,
            "velocity": [vx, vy, None],
            "yaw_speed": wz,
            "frame_id": self._frame,
            "child_frame_id": self._child,
            "units": {"linear": "m/s", "angular": "rad/s", "length": "m", "angle": "rad"},
            "available": available,
            "pose_state": ("online" if native else "waiting" if self._native.installed else "sdk_unavailable"),
            "speed_state": "unavailable" if snap["available"] and not speed_available else snap["state"],
            "sample_timestamps_ms": {
                "pose": native["stamp_ms"] if native else None,
                "speed": snap["received_at_ms"] if speed is not None else None,
            },
            "pose_source_stamp_ns": native["source_stamp_ns"] if native else None,
            "speed_source_stamp_ms": raw["source_stamp_ms"] if speed is not None else None,
        }

    def odom_payload(self, value):
        pose = None
        stamps = value["sample_timestamps_ms"]
        if value["position"] is not None:
            x, y, _ = value["position"]
            angle = value["rpy"][2]
            pose = {
                "stamp_ms": stamps["pose"], "frame_id": self._frame, "child_frame_id": self._child,
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(angle / 2), "w": math.cos(angle / 2)},
            }
        # Preserve receipt time when republishing: an old observation must not
        # become fresh merely because a timer fired. Source time is separate.
        vx, vy, _ = value["velocity"]
        return build_sample(
            stamp_ms=stamps["speed"] or 0, frame="body",
            twist=[vx, vy, None, None, None, value["yaw_speed"]], pose=pose,
            vendor={"source_stamp_ms": value["speed_source_stamp_ms"], "stamp_source": "received",
                    "pose_source_stamp_ns": value["pose_source_stamp_ns"],
                    "pose_state": value["pose_state"], "speed_state": value["speed_state"],
                    "available": value["available"], "pose_source": "vmr_sdk_odometry",
                    "planar": True, "twist_source": "chassis_current_state_speed"},
        )

    def _publish(self):
        with self._lock:
            value = self._snapshot()
            for publisher, payload in zip(self._publishers, (value, self.odom_payload(value))):
                msg = self._message_type()
                msg.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
                publisher.publish(msg)

    def dispatch(self, action, args):
        if args.get("_tool_name", self.PREFIX) != self.PREFIX:
            return None
        with self._lock:
            if action == "info":
                return {**self.get_tools()[0], "odom_interface": self._odom_interface(),
                        "pose_source": "vmr_sdk_odometry", "planar": True,
                        "frame_id": self._frame, "child_frame_id": self._child,
                        "sdk_installed": self._native.installed}
            if action in (self.PREFIX, "read", "status"):
                return {"data": self.payload(), "topic_out": self._topics}
            if action in ("start", "config"):
                url = args.get("odometry_state_url")
                target_url = validate_url(url) if url else self.client.url
                if target_url != self.client.url:
                    replacement = self._new_client(target_url)
                    self.client.stop()
                    self.client = replacement
                self.start()
                return {"state": "running" if action == "start" else "configured", "topic_out": self._topics}
            if action == "stop":
                # Match loco_state: the canvas's stop leaves observation active.
                return {"state": "idle", "topic_out": self._topics}
            raise ValueError(f"unsupported odometry action: {action}")
