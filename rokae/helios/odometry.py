"""Read-only chassis localization and body-frame speed for Navi."""

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


SPEED_AXES = ("vx", "vy", "wz")


def parse_pose_frame(raw):
    """Decode the chassis state pose and its optional speed feedback."""
    message = UiDatagram()
    try:
        message.ParseFromString(raw)
    except DecodeError as exc:
        raise ValueError("invalid chassis pose Protobuf frame") from exc
    if not message.HasField("current_state"):
        return None
    state = message.current_state
    if not state.HasField("pose"):
        raise ValueError("current_state has no pose")
    values = [float(state.pose.x), float(state.pose.y), float(state.pose.theta)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("chassis pose contains non-finite values")
    stamp = state.timeStamp if state.timeStamp > 0 else None
    speed = None
    if state.HasField("speed") and stamp is not None:
        # Proto3 omits zero-valued scalars. Presence of the speed message
        # distinguishes a real stationary observation from a missing speed.
        speed = [value if math.isfinite(value) else None
                 for value in (state.speed.vx, state.speed.vy, state.speed.w)]
    return {"x": values[0], "y": values[1], "yaw": values[2],
            "source_stamp_ms": stamp, "speed": speed}


class PoseFeedbackClient(StreamFeedbackClient):
    def __init__(self, url, **kwargs):
        self._last_stamp = None
        super().__init__(url, parser=self._parse, data_key="pose", binary=True,
                         max_msg_size=8 * 1024 * 1024, **kwargs)

    def _parse(self, raw):
        pose = parse_pose_frame(raw)
        if pose is None:
            return None
        stamp = pose["source_stamp_ms"]
        if stamp is not None and self._last_stamp is not None and stamp <= self._last_stamp:
            # Replayed frames must not keep a frozen observation fresh.
            return None
        if stamp is not None:
            self._last_stamp = stamp
        return pose

    def _on_connect(self):
        self._last_stamp = None


class HeliosOdometry:
    PREFIX = "odometry"

    def __init__(self, config, namespace="rokae_helios", ros2=None, client=None):
        self._config = config
        self._ros2 = ros2
        self._node = None
        self._publishers = []
        self._lock = threading.RLock()
        self._origin = None
        self._hz = positive_number(config.get("odometry_publish_hz", 10), "odometry_publish_hz")
        if self._hz > 30:
            raise ValueError("odometry_publish_hz must not exceed 30")
        # A producer must not call an unverified map unit a metre. The standard
        # pose port stays null until the installation declares m or mm.
        self._unit = config.get("odometry_position_unit", "unverified")
        if self._unit not in ("unverified", "m", "mm"):
            raise ValueError("odometry_position_unit must be unverified, m or mm")
        self._scale = 0.001 if self._unit == "mm" else 1.0
        self._frame = f"{namespace}/relative_start"
        self._child = f"{namespace}/chassis_planar"
        self._topics = [
            {"topic": f"/{namespace}/state/relative_pose", "format": "data/json"},
            {"topic": f"/{namespace}/state/odom", "format": "state/odom"},
        ]
        url = (os.getenv("HELIOS_ODOMETRY_STATE_URL") or config.get("odometry_state_url")
               or config.get("battery_state_url"))
        self.client = client or self._new_client(url)

    def _new_client(self, url):
        return PoseFeedbackClient(
            url, stale_timeout_s=self._config.get("odometry_stale_timeout_s", 1),
            connect_timeout_s=self._config.get("connect_timeout_s", 3),
            reconnect_delay_s=self._config.get("reconnect_delay_s", 1),
        )

    def _odom_interface(self):
        return build_interface(
            provides=SPEED_AXES, frame="body", rate_hz=self._hz,
            pose_drift="none" if self._unit == "unverified" else "unbounded",
            timing={"stamp_source": "received"},
        )

    def get_tools(self):
        sensor = tool(
            "odometry", "sensor",
            "Helios 只读相对位姿：首次有效位姿为原点，输出 x、y、yaw；"
            "停止后再开始重新置零。来自底盘定位结果，不保证连续轮式里程计；"
            "标准端口另提供 Navi 所需的机体系线速度和角速度。位置单位未确认时仅显示原始尺度，"
            "标准 state/odom 的 pose 为 null。",
            topic_out=self._topics,
        )
        sensor["configSchema"] = {"type": "object", "properties": {
            "odometry_state_url": {"type": "string", "scope": "shared", "x-sensitive": True,
                                   "description": "底盘只读状态 WebSocket 地址（留空使用部署配置）"},
            "odometry_position_unit": {
                "type": "string", "scope": "shared", "enum": ["unverified", "m", "mm"],
                "default": "unverified",
                "description": "厂家位置单位：未确认保留原始尺度；确认米或毫米后才输出标准米制位姿",
            },
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

    def stop(self):
        with self._lock:
            self.client.stop()
            self._origin = None
            if self._node is not None:
                self._ros2.executor_core.remove_node(self._node)
                self._node.destroy_node()
                self._node = None
            self._publishers = []

    def payload(self, snap=None):
        with self._lock:
            if snap is None:
                snap = self.client.snapshot()
            pose = snap.get("pose") if snap["available"] else None
            relative = {"x": None, "y": None, "yaw": None}
            if pose:
                if self._origin is None:
                    self._origin = (pose["x"], pose["y"], pose["yaw"])
                ox, oy, heading = self._origin
                dx, dy = (pose["x"] - ox) * self._scale, (pose["y"] - oy) * self._scale
                c, s = math.cos(heading), math.sin(heading)
                angle = pose["yaw"] - heading
                relative = {"x": c * dx + s * dy, "y": -s * dx + c * dy,
                            "yaw": math.atan2(math.sin(angle), math.cos(angle))}
            return {**relative, "state": snap["state"], "available": snap["available"],
                    "position_unit": "source_unverified" if self._unit == "unverified" else "m",
                    "yaw_unit": "rad", "age_ms": snap["age_ms"],
                    "stamp_ms": snap["received_at_ms"],
                    "source_stamp_ms": pose["source_stamp_ms"] if pose else None}

    def odom_payload(self, value, speed=None):
        pose = None
        if value["available"] and self._unit != "unverified":
            angle = value["yaw"]
            pose = {
                "stamp_ms": value["stamp_ms"], "frame_id": self._frame, "child_frame_id": self._child,
                "position": {"x": value["x"], "y": value["y"], "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(angle / 2), "w": math.cos(angle / 2)},
            }
        # Preserve receipt time when republishing: an old observation must not
        # become fresh merely because a timer fired. Source time is separate.
        stamp = value["stamp_ms"] or 0
        vx, vy, wz = speed if value["available"] and speed is not None else (None, None, None)
        return build_sample(
            stamp_ms=stamp, frame="body", twist=[vx, vy, None, None, None, wz], pose=pose,
            vendor={"source_stamp_ms": value["source_stamp_ms"], "stamp_source": "received",
                    "state": value["state"], "available": value["available"],
                    "pose_source": "chassis_ui_localization", "planar": True,
                    "twist_source": "chassis_current_state_speed",
                    "position_unit_verified": self._unit != "unverified"},
        )

    def _publish(self):
        with self._lock:
            snap = self.client.snapshot()
            value = self.payload(snap)
            speed = snap["pose"].get("speed") if snap["available"] else None
            for publisher, payload in zip(self._publishers, (value, self.odom_payload(value, speed))):
                msg = self._message_type()
                msg.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
                publisher.publish(msg)

    def dispatch(self, action, args):
        if args.get("_tool_name", self.PREFIX) != self.PREFIX:
            return None
        with self._lock:
            if action == "info":
                return {**self.get_tools()[0], "odom_interface": self._odom_interface(),
                        "pose_source": "chassis_ui_localization", "planar": True,
                        "frame_id": self._frame, "child_frame_id": self._child,
                        "position_unit_verified": self._unit != "unverified"}
            if action in (self.PREFIX, "read", "status"):
                return {"data": self.payload(), "topic_out": self._topics}
            if action in ("start", "config"):
                unit = args.get("odometry_position_unit", self._unit)
                if unit not in ("unverified", "m", "mm"):
                    raise ValueError("odometry_position_unit must be unverified, m or mm")
                url = args.get("odometry_state_url")
                target_url = validate_url(url) if url else self.client.url
                if target_url != self.client.url or unit != self._unit:
                    replacement = self._new_client(target_url)
                    self.client.stop()
                    self.client = replacement
                    self._origin = None
                    self._unit = unit
                    self._scale = 0.001 if unit == "mm" else 1.0
                self.start()
                return {"state": "running" if action == "start" else "configured", "topic_out": self._topics}
            if action == "stop":
                self.client.stop()
                self._origin = None
                self._publish()
                return {"state": "idle", "topic_out": self._topics}
            raise ValueError(f"unsupported odometry action: {action}")
