#!/usr/bin/env python3
"""
dji/M300/device.py — DJI Matrice 300 RTK 无人机设备插件。

设计原则：
  - 一个设备 = 一个 tool，tool schema 含 type 字段（sensor / actuator）
  - sensor：只读声明，驱动启动时自动 start，数据通过 ROS2 topic 输出
  - actuator：单 tool + action 参数分发操作
  - start/stop 不暴露给 LLM，由驱动生命周期管理

插件：
  TelemetryPlugin        (sensor)    — 遥测数据订阅 (GPS, 姿态, 速度, 电池, 避障)
  CameraStreamPlugin     (sensor)    — 相机码流 H.264 → JPEG
  PerceptionPlugin       (sensor)    — 感知图像 (6方向×左右眼，共12路)
  HmsPlugin              (sensor)    — 健康管理系统告警
  FlightPlugin           (actuator)  — 飞行控制 (起飞/降落/返航/摇杆/刹车)
  TimeSyncPlugin         (actuator)  — 飞机信息查询与 GPS 对时
"""

from __future__ import annotations

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)



class _TelemetryNode(Node):
    def __init__(self, topic: str, bridge, publish_rate: int = 10):
        super().__init__("m300_telemetry")
        self._topic = topic
        self._bridge = bridge
        self._pub = self.create_publisher(String, topic, _LOW_LAT_QOS)
        self._timer = None
        self._rate = publish_rate
        self.state = "idle"

    def start(self):
        if self.state == "running":
            return
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.state = "running"
        self.get_logger().info(f"Telemetry started at {self._rate}Hz — {self._topic}")

    def stop(self):
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self.state = "idle"

    def _tick(self):
        try:
            resp = self._bridge.get_telemetry()
            if resp.get("ok"):
                msg = String()
                msg.data = json.dumps(resp["data"], separators=(",", ":"))
                self._pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Telemetry tick error: {e}")


class TelemetryPlugin:
    PREFIX = "telemetry"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._namespace = namespace
        self._topic = f"/{namespace}/telemetry/state"
        rate = plugin_config.get("publish_rate", 10)
        self._auto_start = plugin_config.get("auto_start", True)
        self._node = _TelemetryNode(self._topic, bridge, rate)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "telemetry",
            "type": "sensor",
            "description": "DJI Matrice 300 RTK 遥测数据：GPS位置、姿态、速度、电池、卫星、避障距离、飞行状态。",
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
            "autoStart": self._auto_start,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info"],
                    },
                },
                "required": ["action"],
            },
        }

    def start(self):
        self._node.start()

    def stop(self):
        self._node.stop()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self._node.start()
            return {"state": "running"}
        if action == "stop":
            self._node.stop()
            return {"state": "idle"}
        if action == "info":
            if self._auto_start and self._node.state != "running":
                self._node.start()
            return {
                "state": self._node.state,
                "auto_start": self._auto_start,
                "topic_out": [{"topic": self._topic, "format": "data/json"}],
            }
        return None


class _CameraStreamNode(Node):
    def __init__(self, topic: str, bridge, fps: int = 10, camera: str = "fpv"):
        super().__init__(f"m300_cam_{camera}")
        self._topic = topic
        self._bridge = bridge
        self._pub = self.create_publisher(CompressedImage, topic, _LOW_LAT_QOS)
        self._fps = fps
        self._camera = camera
        self._thread = None
        self.state = "idle"

    def start(self):
        if self.state == "running":
            return {"ok": True}
        response = self._bridge.start_liveview(camera=self._camera)
        if not response.get("ok"):
            self.get_logger().error(
                f"Camera stream start failed ({self._camera}): {response.get('error', response.get('data', {}))}")
            return response
        self.state = "running"
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"Camera stream started — {self._topic} ({self._camera})")
        return response

    def stop(self):
        self.state = "idle"
        self._bridge.stop_liveview(camera=self._camera)

    def _stream_loop(self):
        """Read JPEG frames from /dev/shm (FFmpeg decoded in C bridge)."""
        import os

        frame_path = f"/dev/shm/dji_frame_{self._camera}.jpg"
        last_mtime = 0
        pub_count = 0

        self.get_logger().info(f"stream_loop started, reading {frame_path}")

        while self.state == "running":
            time.sleep(0.033)  # ~30Hz check rate
            if self.state != "running":
                break
            try:
                if not os.path.exists(frame_path):
                    continue
                mtime = os.path.getmtime(frame_path)
                if mtime == last_mtime:
                    continue
                last_mtime = mtime
                with open(frame_path, "rb") as f:
                    jpeg_data = f.read()
                if jpeg_data and len(jpeg_data) > 100:
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.format = "jpeg"
                    msg.data = jpeg_data
                    self._pub.publish(msg)
                    pub_count += 1
                    if pub_count % 300 == 1:
                        self.get_logger().info(f"published #{pub_count} ({len(jpeg_data)} bytes)")
            except Exception:
                pass


class CameraStreamPlugin:
    PREFIX = "camera_stream"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._namespace = namespace
        self._bridge = bridge
        self._executor = executor
        self._fps = plugin_config.get("fps", 10)
        self._nodes: dict[str, _CameraStreamNode] = {}
        self._instance_configs: dict[str, dict] = {}

    def get_tool(self) -> dict:
        return {
            "name": "camera_stream",
            "type": "sensor",
            "multiInstance": True,
            "description": "Matrice 300 RTK 实时码流。当前飞机已验证仅支持 FPV；未挂载可用的 1/2/3 号载荷码流。",
            "topic_out": [{"format": "image/jpeg", "desc": "camera JPEG stream"}],
            "configSchema": {
                "type": "object",
                "properties": {
                    "camera_source": {
                        "type": "string",
                        "description": "Camera source",
                        "scope": "instance",
                        "oneOf": [
                            {"const": "fpv", "title": "Aircraft FPV (verified)"},
                        ],
                    },
                },
            },
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info"],
                    },
                },
                "required": ["action"],
            },
        }

    def start(self):
        pass  # multiInstance starts per-instance

    def stop(self):
        for node in self._nodes.values():
            node.stop()

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get("instance_id", "default")

        if action == "config":
            self._instance_configs[instance_id] = args
            camera = args.get("camera_source", "fpv")
            # If stream is running and camera changed, restart it
            if instance_id in self._nodes:
                node = self._nodes[instance_id]
                if node.state == "running" and node._camera != camera:
                    node.stop()
                    time.sleep(0.3)
                    node._camera = camera
                    node.start()
            return {"ok": True, "camera": camera}

        # Resolve camera from cached instance config
        camera = self._instance_configs.get(instance_id, {}).get("camera_source", "fpv")

        if action == "info":
            safe_id = instance_id.replace("-", "_")
            topic = f"/{self._namespace}/camera/{safe_id}/rgb"
            return {
                "state": self._nodes[instance_id].state if instance_id in self._nodes else "idle",
                "topic_out": [{"topic": topic, "format": "image/jpeg"}],
            }
        if action == "start":
            if instance_id in self._nodes:
                node = self._nodes[instance_id]
                if node._camera != camera:
                    node.stop()
                    time.sleep(0.3)
                    node._camera = camera
                result = node.start()
            else:
                safe_id = instance_id.replace("-", "_")
                topic = f"/{self._namespace}/camera/{safe_id}/rgb"
                node = _CameraStreamNode(topic, self._bridge, self._fps, camera)
                self._executor.add_node(node)
                self._nodes[instance_id] = node
                result = node.start()
            return {"state": node.state, "camera": camera, **result}
        if action == "stop":
            if instance_id in self._nodes:
                self._nodes[instance_id].stop()
            return {"state": "idle"}
        return None



class _PerceptionNode(Node):
    """Publish each physical perception camera on its own ROS topic.

    DJI's API subscribes by direction, while each subscription delivers both
    eyes in that direction.  The C bridge performs the dataType split and
    writes one JPEG per source; this node only polls and publishes those
    independent files.  Both eye instances from the same direction therefore
    share one DJI subscription and still get separate topics.
    """

    SOURCE_TO_DIRECTION = {
        "front_left": "front", "front_right": "front",
        "back_left": "back", "back_right": "back",
        "left_left": "left", "left_right": "left",
        "right_left": "right", "right_right": "right",
        "up_left": "up", "up_right": "up",
        "down_left": "down", "down_right": "down",
    }

    def __init__(self, namespace: str, bridge):
        super().__init__("m300_perception")
        self._namespace = namespace
        self._bridge = bridge
        self._lock = threading.RLock()
        self._pubs: dict[str, object] = {}
        self._active_sources: set[str] = set()
        self._last_mtime: dict[str, int] = {}
        self._timer = self.create_timer(1.0 / 30.0, self._publish_frames)
        self.state = "idle"

    def topic(self, source: str) -> str:
        return f"/{self._namespace}/perception/{source}"

    def start(self, source: str):
        with self._lock:
            if source not in self.SOURCE_TO_DIRECTION:
                return {"ok": False, "error": f"unsupported perception source: {source}"}
            if source not in self._pubs:
                self._pubs[source] = self.create_publisher(
                    CompressedImage, self.topic(source), _LOW_LAT_QOS)
            if source in self._active_sources:
                return {"ok": True, "source": source}

            response = self._bridge.start_perception(source=source)
            if not response.get("ok"):
                return response
            self._active_sources.add(source)
            self.state = "running"
            self.get_logger().info(
                f"Perception started — {source} ({self.topic(source)})")
            return response

    def stop(self, source: str = ""):
        with self._lock:
            if source:
                if source in self._active_sources:
                    self._bridge.stop_perception(source=source)
                    self._active_sources.discard(source)
            else:
                for active_source in list(self._active_sources):
                    self._bridge.stop_perception(source=active_source)
                self._active_sources.clear()
            if not self._active_sources:
                self.state = "idle"

    def is_active(self, source: str) -> bool:
        with self._lock:
            return source in self._active_sources

    def active_sources(self) -> set[str]:
        with self._lock:
            return set(self._active_sources)

    def _publish_frames(self):
        import os

        for source in self.active_sources():
            path = f"/dev/shm/dji_perception_{source}.jpg"
            try:
                # Use nanosecond mtime so two adjacent eye frames cannot be
                # mistaken for the same frame on filesystems with fine mtime.
                mtime = os.stat(path).st_mtime_ns
                if mtime == self._last_mtime.get(source):
                    continue
                with open(path, "rb") as f:
                    data = f.read()
                if len(data) <= 100:
                    continue
                self._last_mtime[source] = mtime
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg; grayscale"
                msg.data = data
                self._pubs[source].publish(msg)
            except OSError:
                continue


class PerceptionPlugin:
    PREFIX = "perception"
    SOURCES = [
        "front_left", "front_right",
        "back_left", "back_right",
        "left_left", "left_right",
        "right_left", "right_right",
        "up_left", "up_right",
        "down_left", "down_right",
    ]
    # Compatibility alias for code that used the old six-direction constant.
    DIRECTIONS = SOURCES
    SOURCE_LABELS = {
        "front_left": "Front — left eye",
        "front_right": "Front — right eye",
        "back_left": "Back — left eye",
        "back_right": "Back — right eye",
        "left_left": "Left — left eye",
        "left_right": "Left — right eye",
        "right_left": "Right — left eye",
        "right_right": "Right — right eye",
        "up_left": "Up — left eye",
        "up_right": "Up — right eye",
        "down_left": "Down — left eye",
        "down_right": "Down — right eye",
    }
    LEGACY_DIRECTION_TO_SOURCE = {
        "front": "front_left", "back": "back_left",
        "left": "left_left", "right": "right_left",
        "up": "up_left", "down": "down_left",
    }

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._namespace = namespace
        self._node = _PerceptionNode(namespace, bridge)
        self._lock = threading.RLock()
        self._instance_configs: dict[str, dict] = {}
        self._active_instances: dict[str, str] = {}
        executor.add_node(self._node)

    def _source_from_args(self, args: dict) -> str:
        # ``direction`` is the existing card field.  The aliases make direct
        # MCP callers able to use the clearer source/camera_source names too.
        source = (
            args.get("source")
            or args.get("camera_source")
            or args.get("direction")
            or "front_left"
        )
        return self.LEGACY_DIRECTION_TO_SOURCE.get(source, source)

    def get_tool(self) -> dict:
        return {
            "name": "perception",
            "type": "sensor",
            "multiInstance": True,
            "description": "Matrice 300 RTK 12路感知灰度相机：六个方向各自提供左眼和右眼；每个物理相机对应独立 topic/file，支持同时订阅多路输出。",
            "topic_out": [{"format": "image/jpeg", "desc": "perception grayscale stream"}],
            "configSchema": {
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "description": "Perception camera source (direction + eye)",
                        "scope": "instance",
                        "oneOf": [
                            {"const": source, "title": self.SOURCE_LABELS[source]}
                            for source in self.SOURCES
                        ],
                    },
                },
            },
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info"],
                    },
                    "direction": {
                        "type": "string",
                        "enum": self.SOURCES,
                        "description": "Perception camera source",
                    },
                },
                "required": ["action"],
            },
        }

    def start(self):
        pass  # Perception starts per-instance

    def stop(self):
        with self._lock:
            self._active_instances.clear()
            self._node.stop()

    @staticmethod
    def _instance_key(instance_id: str) -> str:
        # Canvas cards always provide an id.  Keep a single compatibility
        # slot for direct, unscoped MCP callers instead of letting every
        # unscoped call become a hidden independent instance.
        return instance_id or "__default__"

    def _release_instance(self, instance_key: str):
        source = self._active_instances.pop(instance_key, None)
        if source and source not in self._active_instances.values():
            self._node.stop(source)

    def _start_instance(self, instance_key: str, source: str) -> dict:
        old_source = self._active_instances.get(instance_key)
        if old_source and old_source != source:
            self._release_instance(instance_key)

        result = self._node.start(source)
        if result.get("ok"):
            self._active_instances[instance_key] = source
        return result

    def _instance_state(self, instance_key: str, source: str) -> str:
        return (
            "running"
            if self._active_instances.get(instance_key) == source
            and self._node.is_active(source)
            else "idle"
        )

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get("instance_id", "")
        instance_key = self._instance_key(instance_id)
        if action == "config":
            source = self._source_from_args(args)
            if source not in self.SOURCES:
                return {"ok": False, "error": f"unsupported perception source: {source}"}
            with self._lock:
                previous = self._instance_configs.get(instance_id, {}).get("source")
                self._instance_configs[instance_id] = {"source": source}
                # A live card may be reconfigured from one physical camera to
                # another. Release the old source before applying the new one
                # so the active source set stays in sync with the card.
                if previous and previous != source and instance_key in self._active_instances:
                    self._release_instance(instance_key)
            return {
                "ok": True,
                "source": source,
                "direction": source,
                "topic": self._node.topic(source),
            }

        source = self._instance_configs.get(instance_id, {}).get(
            "source", self._source_from_args(args)
        )
        if source not in self.SOURCES:
            return {"state": "error", "message": f"unsupported perception source: {source}"}
        topic = self._node.topic(source)
        if action == "start":
            with self._lock:
                result = self._start_instance(instance_key, source)
                response = {
                    "source": source,
                    "direction": source,
                    "topic_out": [{"topic": topic, "format": "image/jpeg"}],
                    **result,
                }
                # `state` is per card.  The node can remain running for a
                # different card; returning global node state made Agent Core
                # treat a failed card as successfully started.
                response["state"] = (
                    "running" if result.get("ok") else "error"
                )
                if not result.get("ok"):
                    # Agent Core renders this as the per-card startup error;
                    # the bridge keeps the low-level detail in `error`.
                    response["message"] = result.get(
                        "error", "perception start failed"
                    )
                return response
        if action == "stop":
            with self._lock:
                self._release_instance(instance_key)
                return {"state": "idle", "source": source}
        if action == "info":
            with self._lock:
                if instance_id not in self._instance_configs:
                    return {
                        "state": self._instance_state(instance_key, source),
                        "topic_out": [],
                    }
                return {
                    "state": self._instance_state(instance_key, source),
                    "source": source,
                    "direction": source,
                    "topic_out": [{"topic": topic, "format": "image/jpeg"}],
                }
        return None



class _HmsNode(Node):
    def __init__(self, topic: str, bridge):
        super().__init__("m300_hms")
        self._topic = topic
        self._bridge = bridge
        self._pub = self.create_publisher(String, topic, _LOW_LAT_QOS)
        self._timer = None
        self.state = "idle"

    def start(self):
        if self.state == "running":
            return
        self._timer = self.create_timer(5.0, self._tick)
        self.state = "running"

    def stop(self):
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self.state = "idle"

    def _tick(self):
        try:
            resp = self._bridge.get_hms_info()
            if resp.get("ok"):
                msg = String()
                msg.data = json.dumps(resp["data"], separators=(",", ":"))
                self._pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"HMS tick error: {e}")


class HmsPlugin:
    PREFIX = "hms"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._bridge = bridge
        self._topic = f"/{namespace}/hms/alerts"
        self._node = _HmsNode(self._topic, bridge)
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [
            {
                "name": "hms",
                "type": "sensor",
                "description": "Matrice 300 RTK 健康管理系统 (HMS) 告警。监控飞行器/负载健康状态，输出告警事件。",
                "topic_out": [{"topic": self._topic, "format": "data/json"}],
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["start", "stop", "info"],
                        },
                    },
                    "required": ["action"],
                },
            },
            {
                "name": "hms_inject",
                "type": "actuator",
                "hidden": True,
                "description": "Matrice 300 RTK HMS 手动注入告警（用于测试）。注入一个自定义错误码到健康管理系统。",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["inject"],
                        },
                        "error_code": {
                            "type": "integer",
                            "description": "错误码，十六进制格式，如 0x1E020001",
                        },
                        "error_level": {
                            "type": "integer",
                            "description": "告警级别，1=提示，2=警告，3=严重",
                            "minimum": 1,
                            "maximum": 3,
                        },
                    },
                    "required": ["action", "error_code"],
                },
            },
            {
                "name": "hms_eliminate",
                "type": "actuator",
                "hidden": True,
                "description": "Matrice 300 RTK HMS 手动消除告警（用于测试）。消除一个指定的自定义错误码告警。",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["eliminate"],
                        },
                        "error_code": {
                            "type": "integer",
                            "description": "错误码，十六进制格式，如 0x1E020001",
                        },
                    },
                    "required": ["action", "error_code"],
                },
            },
        ]

    def start(self):
        self._node.start()

    def stop(self):
        self._node.stop()

    def dispatch(self, action: str, args: dict) -> dict | None:
        tool_name = args.pop("_tool_name", None)
        if tool_name == "hms":
            if action == "start":
                self._node.start()
                return {"state": "running"}
            if action == "stop":
                self._node.stop()
                return {"state": "idle"}
            if action == "info":
                return {
                    "state": self._node.state,
                    "topic_out": [{"topic": self._topic, "format": "data/json"}],
                }
        elif tool_name == "hms_inject":
            if action == "inject":
                error_code = args.get("error_code", 0x1E020001)
                error_level = args.get("error_level", 1)
                return self._bridge.hms_inject_error(error_code, error_level)
        elif tool_name == "hms_eliminate":
            if action == "eliminate":
                error_code = args.get("error_code", 0x1E020001)
                return self._bridge.hms_eliminate_error(error_code)
        return None



class FlightPlugin:
    PREFIX = "flight"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._bridge = bridge
        self._has_authority = False

    def get_tools(self) -> list:
        return [
            {
                "name": "flight",
                "type": "actuator",
                "description": "Matrice 300 RTK 飞行控制。安全提示：SDK 控制期间遥控器摇杆无效，切换档位可立即夺回控制权。",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": [
                                "start", "stop",
                                "takeoff", "land", "confirm_landing", "go_home", "cancel_go_home",
                                "move", "stop_move",
                                "rotate_start", "rotate_stop",
                                "set_home", "set_obstacle_avoidance",
                            ],
                        },
                        "vx": {"type": "number", "description": "前进速度 (m/s)，正=前，范围 -15~15", "minimum": -15, "maximum": 15},
                        "vy": {"type": "number", "description": "侧移速度 (m/s)，正=右，范围 -15~15", "minimum": -15, "maximum": 15},
                        "vz": {"type": "number", "description": "升降速度 (m/s)，正=上，范围 -6~6", "minimum": -6, "maximum": 6},
                        "vyaw": {"type": "number", "description": "偏航角速度 (deg/s)，正=顺时针，范围 -75~75", "minimum": -75, "maximum": 75},
                        "duration": {"type": "number", "description": "持续时间(秒), -1=持续到stop_move", "default": 1},
                        "require_rc_confirm": {
                            "type": "boolean",
                            "description": "降落是否需要遥控器确认 (true=需确认, false=自动确认)",
                            "default": True,
                        },
                        "lat": {"type": "number", "description": "纬度 (返航点)"},
                        "lon": {"type": "number", "description": "经度 (返航点)"},
                        "enabled": {
                            "type": "string",
                            "description": "避障开关",
                            "enum": ["on", "off"],
                        },
                        "direction": {
                            "type": "string",
                            "description": "避障方向",
                            "enum": ["all", "front", "back", "left", "right", "up", "down"],
                        },
                    },
                    "required": ["action"],
                "x-action-params": {
                    "takeoff": {"params": [], "description": "起飞 (自动悬停在1.2m)"},
                    "land": {
                        "params": ["require_rc_confirm"],
                        "description": "降落 (require_rc_confirm: true=需遥控器确认, false=自动确认)",
                    },
                    "confirm_landing": {"params": [], "description": "确认降落 (飞机悬停等待确认时调用)"},
                    "go_home": {"params": [], "description": "返航 (飞回返航点)"},
                    "cancel_go_home": {"params": [], "description": "取消返航"},
                    "move": {
                        "params": ["vx", "vy", "vz", "vyaw", "duration"],
                        "description": "持续摇杆控制 — 设置速度向量 (duration秒后自动停止, -1=持续到stop_move)",
                    },
                    "stop_move": {"params": [], "description": "停止运动并悬停"},
                    "rotate_start": {"params": [], "description": "启动电机旋转桨叶 (全速)"},
                    "rotate_stop": {"params": [], "description": "停止电机 (仅地面可用)"},
                    "set_home": {
                        "params": ["lat", "lon"],
                        "description": "设置返航点 GPS 坐标",
                    },
                    "set_obstacle_avoidance": {
                        "params": ["enabled", "direction"],
                        "description": "设置避障开关 (方向可选 all/front/back/...)",
                    },
                },
            },
        }]

    def start(self):
        pass

    def stop(self):
        if self._has_authority:
            self._bridge.release_joystick_authority()
            self._has_authority = False

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop("_tool_name", None)

        # flight tool
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "takeoff":
            resp = self._bridge.takeoff()
            return {"ret": 0 if resp.get("ok") else -1, "action": "takeoff"}
        if action == "land":
            require_rc = args.get("require_rc_confirm", True)
            if isinstance(require_rc, str):
                require_rc = require_rc.lower() not in ("false", "0", "no")
            auto_confirm = not require_rc
            resp = self._bridge.land(auto_confirm=auto_confirm)
            if resp.get("ok"):
                msg = resp.get("data", {}).get("message", "Landing initiated")
                return {"ret": 0, "message": msg}
            return {"ret": -1, "data": resp.get("data", {})}
        if action == "confirm_landing":
            resp = self._bridge.confirm_landing()
            if resp.get("ok"):
                return {"ret": 0, "message": "Landing confirmed"}
            return {"ret": -1, "data": resp.get("data", {})}
        if action == "go_home":
            resp = self._bridge.go_home()
            return {"ret": 0 if resp.get("ok") else -1, "action": "go_home"}
        if action == "cancel_go_home":
            resp = self._bridge.cancel_go_home()
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "move":
            # Always obtain authority — C layer releases it after each move
            auth = self._bridge.obtain_joystick_authority()
            if not auth.get("ok"):
                return {"ret": -1, "error": "Failed to obtain joystick authority", "data": auth.get("data", {})}
            duration = args.get("duration", 1)
            try:
                duration = float(duration)
            except (TypeError, ValueError):
                duration = -1
            # Conservative limits pending a payload-and-firmware flight test.
            vx = max(-10, min(10, float(args.get("vx", 0))))
            vy = max(-10, min(10, float(args.get("vy", 0))))
            vz = max(-4, min(4, float(args.get("vz", 0))))
            vyaw = max(-60, min(60, float(args.get("vyaw", 0))))
            resp = self._bridge.joystick_move(
                vx=vx, vy=vy, vz=vz, vyaw=vyaw,
                duration=duration,
            )
            if resp.get("ok"):
                msg = resp.get("data", {}).get("message", "Moving")
                return {"ret": 0, "message": msg}
            return {"ret": -1, "data": resp.get("data", {})}
        if action == "stop_move":
            resp = self._bridge.stop_move()
            if resp.get("ok"):
                return {"ret": 0, "message": "Stopped, hovering"}
            return {"ret": -1, "data": resp.get("data", {})}
        if action == "rotate_start":
            resp = self._bridge.turn_on_motors()
            return {"ret": 0 if resp.get("ok") else -1, "action": "rotate_start"}
        if action == "rotate_stop":
            resp = self._bridge.turn_off_motors()
            return {"ret": 0 if resp.get("ok") else -1, "action": "rotate_stop"}
        if action == "set_home":
            resp = self._bridge.set_home_point(
                lat=args.get("lat", 0), lon=args.get("lon", 0),
            )
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "set_obstacle_avoidance":
            enabled_val = args.get("enabled", "on")
            resp = self._bridge.set_obstacle_avoidance(
                enabled=(enabled_val == "on" or enabled_val is True),
                direction=args.get("direction", "all"),
            )
            return {"ret": 0 if resp.get("ok") else -1}
        return None



class TimeSyncPlugin:
    PREFIX = "aircraft_info"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._bridge = bridge

    def get_tool(self) -> dict:
        return {
            "name": "aircraft_info",
            "type": "actuator",
            "description": "飞机信息查询：机型、固件版本、连接状态、GPS 对时。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["get_info", "sync_time"],
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "get_info": {"params": [], "description": "获取机型/固件/连接状态"},
                    "sync_time": {"params": [], "description": "从飞机 GPS 对时，返回 UTC 时间"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in ("start", "stop"):
            return {"state": "ready"}
        if action == "get_info":
            resp = self._bridge.get_aircraft_info()
            return {"ret": 0 if resp.get("ok") else -1, "data": resp.get("data", {})}
        if action == "sync_time":
            resp = self._bridge.sync_clock()
            return {"ret": 0 if resp.get("ok") else -1, "data": resp.get("data", {})}
        return None
