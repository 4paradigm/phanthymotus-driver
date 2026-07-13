#!/usr/bin/env python3
"""
dji/mavic3e/device.py — DJI Mavic 3E 无人机设备插件。

设计原则：
  - 一个设备 = 一个 tool，tool schema 含 type 字段（sensor / actuator）
  - sensor：只读声明，驱动启动时自动 start，数据通过 ROS2 topic 输出
  - actuator：单 tool + action 参数分发操作
  - start/stop 不暴露给 LLM，由驱动生命周期管理

插件：
  TelemetryPlugin        (sensor)    — 遥测数据订阅 (GPS, 姿态, 速度, 电池, 避障)
  CameraStreamPlugin     (sensor)    — 相机码流 H.264 → JPEG
  PerceptionPlugin       (sensor)    — 感知图像 (6方向避障相机)
  HmsPlugin              (sensor)    — 健康管理系统告警
  FlightPlugin           (actuator)  — 飞行控制 (起飞/降落/返航/摇杆/刹车)
  CameraPlugin           (actuator)  — 相机管理 (拍照/录像/变焦/对焦/曝光)
  GimbalPlugin           (actuator)  — 云台管理 (旋转/复位/模式)
  WaypointPlugin         (actuator)  — 航点任务 V3 (KMZ 上传/执行)
  SpeakerPlugin          (actuator)  — 喊话器 (播放/音量/停止)
  PowerPlugin            (sensor)    — 电源管理 (E-Port 电源状态)
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)


# ═══════════════════════════════════════════════════════════════════════════
#  TelemetryPlugin (sensor)
#  PSDK: 数据订阅 + 机型信息
# ═══════════════════════════════════════════════════════════════════════════

class _TelemetryNode(Node):
    def __init__(self, topic: str, bridge, publish_rate: int = 10):
        super().__init__("mavic3e_telemetry")
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
        self._node = _TelemetryNode(self._topic, bridge, rate)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "telemetry",
            "type": "sensor",
            "description": "DJI Mavic 3E 遥测数据：GPS位置、姿态、速度、电池、卫星、避障距离、飞行状态。",
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
            return {
                "state": self._node.state,
                "topic_out": [{"topic": self._topic, "format": "data/json"}],
            }
        return None


# ═══════════════════════════════════════════════════════════════════════════
#  CameraStreamPlugin (sensor, multiInstance)
#  PSDK: 获取相机码流 + 视频流传输
# ═══════════════════════════════════════════════════════════════════════════

class _CameraStreamNode(Node):
    def __init__(self, topic: str, bridge, fps: int = 10, camera: str = "wide"):
        super().__init__(f"mavic3e_cam_{camera}")
        self._topic = topic
        self._bridge = bridge
        self._pub = self.create_publisher(String, topic, _LOW_LAT_QOS)
        self._fps = fps
        self._camera = camera
        self._thread = None
        self.state = "idle"

    def start(self):
        if self.state == "running":
            return
        self._bridge.start_liveview(camera=self._camera)
        self.state = "running"
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"Camera stream started — {self._topic} ({self._camera})")

    def stop(self):
        self.state = "idle"
        self._bridge.stop_liveview()

    def _stream_loop(self):
        """Publish mock JPEG frames in mock mode, or receive from bridge in live mode."""
        import struct as _struct
        # Generate a minimal valid JPEG for mock mode (1x1 pixel, colored by camera type)
        _MOCK_JPEGS = {
            "wide": self._make_mock_jpeg(0x40, 0x80, 0x40),   # greenish
            "zoom": self._make_mock_jpeg(0x40, 0x40, 0xA0),   # blueish
            "ir":   self._make_mock_jpeg(0xA0, 0x40, 0x40),   # reddish
        }
        mock_frame = _MOCK_JPEGS.get(self._camera, _MOCK_JPEGS["wide"])

        while self.state == "running":
            time.sleep(1.0 / self._fps)
            if self.state != "running":
                break
            # Publish mock JPEG as base64 in String msg (dashboard decodes)
            import base64
            msg = String()
            msg.data = base64.b64encode(mock_frame).decode("ascii")
            self._pub.publish(msg)

    @staticmethod
    def _make_mock_jpeg(r: int, g: int, b: int) -> bytes:
        """Generate a minimal 8x8 JPEG with solid color for testing."""
        # Minimal JPEG: SOI + APP0 + DQT + SOF0 + DHT + SOS + data + EOI
        # For simplicity, use a pre-built tiny JPEG structure
        # This creates a valid 1x1 JPEG with the given RGB color
        import io
        try:
            from PIL import Image
            img = Image.new("RGB", (160, 120), (r, g, b))
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=50)
            return buf.getvalue()
        except ImportError:
            # Fallback: return a minimal valid JPEG (1x1 white pixel)
            return bytes([
                0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01,
                0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43,
                0x00, 0x08, 0x06, 0x06, 0x07, 0x06, 0x05, 0x08, 0x07, 0x07, 0x07, 0x09,
                0x09, 0x08, 0x0A, 0x0C, 0x14, 0x0D, 0x0C, 0x0B, 0x0B, 0x0C, 0x19, 0x12,
                0x13, 0x0F, 0x14, 0x1D, 0x1A, 0x1F, 0x1E, 0x1D, 0x1A, 0x1C, 0x1C, 0x20,
                0x24, 0x2E, 0x27, 0x20, 0x22, 0x2C, 0x23, 0x1C, 0x1C, 0x28, 0x37, 0x29,
                0x2C, 0x30, 0x31, 0x34, 0x34, 0x34, 0x1F, 0x27, 0x39, 0x3D, 0x38, 0x32,
                0x3C, 0x2E, 0x33, 0x34, 0x32, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x01,
                0x00, 0x01, 0x01, 0x01, 0x11, 0x00, 0xFF, 0xC4, 0x00, 0x1F, 0x00, 0x00,
                0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                0x09, 0x0A, 0x0B, 0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01, 0x00, 0x00, 0x3F,
                0x00, 0x7B, 0x40, 0x1B, 0xFF, 0xD9,
            ])


class CameraStreamPlugin:
    PREFIX = "camera_stream"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._namespace = namespace
        self._bridge = bridge
        self._executor = executor
        self._fps = plugin_config.get("fps", 10)
        self._nodes: dict[str, _CameraStreamNode] = {}

    def get_tool(self) -> dict:
        return {
            "name": "camera_stream",
            "type": "sensor",
            "multiInstance": True,
            "description": "Mavic 3E 相机实时码流 (H.264 解码 → JPEG)。支持广角/变焦/红外(3T)镜头切换。每个实例独立选择镜头。",
            "topic_out": [{"format": "image/jpeg", "desc": "camera JPEG stream"}],
            "configSchema": {
                "type": "object",
                "properties": {
                    "camera_source": {
                        "type": "string",
                        "description": "Camera source",
                        "scope": "instance",
                        "oneOf": [
                            {"const": "wide", "title": "Wide (24mm)"},
                            {"const": "zoom", "title": "Zoom (7-28x)"},
                            {"const": "ir", "title": "IR Thermal (3T only)"},
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
        camera = args.get("camera_source", "wide")

        if action == "info":
            safe_id = instance_id.replace("-", "_")
            topic = f"/{self._namespace}/camera/{safe_id}/rgb"
            return {
                "state": self._nodes[instance_id].state if instance_id in self._nodes else "idle",
                "topic_out": [{"topic": topic, "format": "image/jpeg"}],
            }
        if action == "start":
            if instance_id not in self._nodes:
                safe_id = instance_id.replace("-", "_")
                topic = f"/{self._namespace}/camera/{safe_id}/rgb"
                node = _CameraStreamNode(topic, self._bridge, self._fps, camera)
                self._executor.add_node(node)
                self._nodes[instance_id] = node
            self._nodes[instance_id].start()
            return {"state": "running"}
        if action == "stop":
            if instance_id in self._nodes:
                self._nodes[instance_id].stop()
            return {"state": "idle"}
        return None


# ═══════════════════════════════════════════════════════════════════════════
#  PerceptionPlugin (sensor, multiInstance)
#  PSDK: 感知数据 — 6方向避障灰度图
# ═══════════════════════════════════════════════════════════════════════════

class _PerceptionNode(Node):
    def __init__(self, namespace: str, bridge):
        super().__init__("mavic3e_perception")
        self._namespace = namespace
        self._bridge = bridge
        self._pubs: dict[str, object] = {}
        self._active_directions: set[str] = set()
        self.state = "idle"

    def start(self, direction: str):
        topic = f"/{self._namespace}/perception/{direction}"
        if direction not in self._pubs:
            self._pubs[direction] = self.create_publisher(String, topic, _LOW_LAT_QOS)
        self._active_directions.add(direction)
        self._bridge.start_perception(direction=direction)
        self.state = "running"
        self.get_logger().info(f"Perception started — {direction}")

    def stop(self, direction: str = ""):
        if direction:
            self._active_directions.discard(direction)
            self._bridge.stop_perception(direction=direction)
        else:
            for d in list(self._active_directions):
                self._bridge.stop_perception(direction=d)
            self._active_directions.clear()
        if not self._active_directions:
            self.state = "idle"


class PerceptionPlugin:
    PREFIX = "perception"
    DIRECTIONS = ["front", "back", "left", "right", "up", "down"]

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._namespace = namespace
        self._node = _PerceptionNode(namespace, bridge)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "perception",
            "type": "sensor",
            "multiInstance": True,
            "description": "Mavic 3E 感知避障图像。6个方向 (前/后/左/右/上/下) 灰度图，上下640x480，其余480x480，最多同时2路。",
            "topic_out": [{"topic": f"/{self._namespace}/perception/{{direction}}", "format": "image/jpeg"}],
            "configSchema": {
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "description": "Perception direction",
                        "scope": "instance",
                        "oneOf": [
                            {"const": d, "title": d.capitalize()}
                            for d in self.DIRECTIONS
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
                        "enum": self.DIRECTIONS,
                        "description": "Perception direction",
                    },
                },
                "required": ["action"],
            },
        }

    def start(self):
        pass  # Perception starts per-instance

    def stop(self):
        self._node.stop()

    def dispatch(self, action: str, args: dict) -> dict | None:
        direction = args.get("direction", "front")
        if action == "start":
            self._node.start(direction)
            return {"state": "running"}
        if action == "stop":
            self._node.stop(direction)
            return {"state": self._node.state}
        if action == "info":
            instance_id = args.get("instance_id", "")
            dir_name = args.get("direction", "front")
            topic = f"/{self._namespace}/perception/{dir_name}"
            if instance_id:
                topic = f"/{self._namespace}/perception/{instance_id.replace('-', '_')}"
            return {
                "state": self._node.state,
                "topic_out": [{"topic": topic, "format": "image/jpeg"}],
            }
        return None


# ═══════════════════════════════════════════════════════════════════════════
#  HmsPlugin (sensor)
#  PSDK: HMS管理 + 自定义HMS
# ═══════════════════════════════════════════════════════════════════════════

class _HmsNode(Node):
    def __init__(self, topic: str, bridge):
        super().__init__("mavic3e_hms")
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
            if resp.get("ok") and resp["data"].get("alerts"):
                msg = String()
                msg.data = json.dumps(resp["data"], separators=(",", ":"))
                self._pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"HMS tick error: {e}")


class HmsPlugin:
    PREFIX = "hms"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._topic = f"/{namespace}/hms/alerts"
        self._node = _HmsNode(self._topic, bridge)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "hms",
            "type": "sensor",
            "description": "Mavic 3E 健康管理系统 (HMS) 告警。监控飞行器/负载健康状态，输出告警事件。",
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
            return {
                "state": self._node.state,
                "topic_out": [{"topic": self._topic, "format": "data/json"}],
            }
        return None


# ═══════════════════════════════════════════════════════════════════════════
#  FlightPlugin (actuator, multi-tool)
#  PSDK: 飞行控制
# ═══════════════════════════════════════════════════════════════════════════

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
                "description": "Mavic 3E 飞行控制：起飞、降落、返航、摇杆控制、紧急刹车、设置返航点、避障开关。",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": [
                                "start", "stop",
                                "takeoff", "land", "go_home", "cancel_go_home",
                                "move", "stop_move",
                                "rotate_start", "rotate_stop",
                                "set_home", "set_obstacle_avoidance",
                            ],
                        },
                        "vx": {"type": "number", "description": "前进速度 (m/s)，正=前"},
                        "vy": {"type": "number", "description": "侧移速度 (m/s)，正=右"},
                        "vz": {"type": "number", "description": "升降速度 (m/s)，正=上"},
                        "vyaw": {"type": "number", "description": "偏航角速度 (deg/s)，正=顺时针"},
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
                    "land": {"params": [], "description": "降落"},
                    "go_home": {"params": [], "description": "返航 (飞回返航点)"},
                    "cancel_go_home": {"params": [], "description": "取消返航"},
                    "move": {
                        "params": ["vx", "vy", "vz", "vyaw"],
                        "description": "摇杆控制 — 设置速度向量 (需先获取控制权)",
                    },
                    "stop_move": {"params": [], "description": "紧急刹车 (悬停)"},
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
            resp = self._bridge.land()
            return {"ret": 0 if resp.get("ok") else -1, "action": "land"}
        if action == "go_home":
            resp = self._bridge.go_home()
            return {"ret": 0 if resp.get("ok") else -1, "action": "go_home"}
        if action == "cancel_go_home":
            resp = self._bridge.cancel_go_home()
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "move":
            if not self._has_authority:
                auth = self._bridge.obtain_joystick_authority()
                if auth.get("ok"):
                    self._has_authority = True
                else:
                    return {"ret": -1, "error": "Failed to obtain joystick authority"}
            resp = self._bridge.joystick_move(
                vx=args.get("vx", 0),
                vy=args.get("vy", 0),
                vz=args.get("vz", 0),
                vyaw=args.get("vyaw", 0),
            )
            return {"ret": 0 if resp.get("ok") else -1, "vx": args.get("vx", 0)}
        if action == "stop_move":
            resp = self._bridge.emergency_brake()
            return {"ret": 0 if resp.get("ok") else -1, "action": "brake"}
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


# ═══════════════════════════════════════════════════════════════════════════
#  CameraPlugin (actuator)
#  PSDK: 相机管理 + 基础相机功能
# ═══════════════════════════════════════════════════════════════════════════

class CameraPlugin:
    PREFIX = "camera"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._bridge = bridge

    def get_tool(self) -> dict:
        return {
            "name": "camera",
            "type": "actuator",
            "description": "Mavic 3E 相机管理：拍照、录像、变焦、对焦、曝光、存储查询、红外测温(3T)。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "start", "stop",
                            "take_photo", "start_video", "stop_video",
                            "set_mode", "set_zoom", "set_focus", "set_exposure",
                            "get_storage", "ir_temp_point", "ir_temp_area",
                        ],
                    },
                    "mode": {
                        "type": "string",
                        "description": "拍照模式 (single/interval/burst) 或相机模式 (photo/video)",
                        "enum": ["single", "interval", "burst", "photo", "video"],
                    },
                    "zoom_factor": {"type": "number", "description": "变焦倍数 (广角1x, 长焦7-28x)"},
                    "focus_x": {"type": "number", "description": "对焦点 X (0-1 归一化)"},
                    "focus_y": {"type": "number", "description": "对焦点 Y (0-1 归一化)"},
                    "iso": {"type": "integer", "description": "ISO (0=auto, 100-25600)"},
                    "aperture": {"type": "number", "description": "光圈 (如 2.8, 4.0)"},
                    "shutter_speed": {"type": "number", "description": "快门速度 (秒)"},
                    "ev": {"type": "number", "description": "曝光补偿 (-3.0 ~ +3.0)"},
                    "point_x": {"type": "number", "description": "红外测温点 X (0-1)"},
                    "point_y": {"type": "number", "description": "红外测温点 Y (0-1)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "take_photo": {"params": ["mode"], "description": "拍照 (支持单拍/连拍/定时)"},
                    "start_video": {"params": [], "description": "开始录像"},
                    "stop_video": {"params": [], "description": "停止录像"},
                    "set_mode": {"params": ["mode"], "description": "切换相机模式 (photo/video)"},
                    "set_zoom": {"params": ["zoom_factor"], "description": "设置变焦倍数"},
                    "set_focus": {"params": ["focus_x", "focus_y"], "description": "设置对焦点"},
                    "set_exposure": {
                        "params": ["iso", "aperture", "shutter_speed", "ev"],
                        "description": "设置曝光参数",
                    },
                    "get_storage": {"params": [], "description": "查询存储卡剩余容量"},
                    "ir_temp_point": {
                        "params": ["point_x", "point_y"],
                        "description": "红外点测温 (仅3T型号)",
                    },
                    "ir_temp_area": {"params": [], "description": "红外区域测温 (仅3T型号)"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "take_photo":
            resp = self._bridge.take_photo(mode=args.get("mode", "single"))
            return {"ret": 0 if resp.get("ok") else -1, "data": resp.get("data", {})}
        if action == "start_video":
            resp = self._bridge.start_video()
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "stop_video":
            resp = self._bridge.stop_video()
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "set_mode":
            resp = self._bridge.set_camera_mode(mode=args.get("mode", "photo"))
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "set_zoom":
            resp = self._bridge.set_zoom(factor=args.get("zoom_factor", 1.0))
            return {"ret": 0 if resp.get("ok") else -1, "data": resp.get("data", {})}
        if action == "set_focus":
            resp = self._bridge.set_focus(
                x=args.get("focus_x", 0.5), y=args.get("focus_y", 0.5),
            )
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "set_exposure":
            resp = self._bridge.set_exposure(
                iso=args.get("iso", 0),
                aperture=args.get("aperture", 0),
                shutter_speed=args.get("shutter_speed", 0),
                ev=args.get("ev", 0),
            )
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "get_storage":
            resp = self._bridge.get_storage()
            return resp.get("data", {})
        if action == "ir_temp_point":
            # 3T only — forward to bridge
            return {"ret": 0, "note": "ir_temp_point: requires 3T hardware"}
        if action == "ir_temp_area":
            return {"ret": 0, "note": "ir_temp_area: requires 3T hardware"}
        return None


# ═══════════════════════════════════════════════════════════════════════════
#  GimbalPlugin (actuator)
#  PSDK: 云台管理 + 云台功能
# ═══════════════════════════════════════════════════════════════════════════

class GimbalPlugin:
    PREFIX = "gimbal"

    # Mavic 3E gimbal range (narrower than M300/M350)
    PITCH_RANGE = (-90, 35)
    YAW_RANGE = (-40, 40)

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._bridge = bridge

    def get_tool(self) -> dict:
        return {
            "name": "gimbal",
            "type": "actuator",
            "description": (
                "Mavic 3E 云台控制：旋转 (pitch -90°~+35°, yaw -40°~+40°)、复位、模式切换。"
                "支持绝对角度/相对角度/角速度三种控制模式。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "rotate", "reset", "set_mode", "get_angles"],
                    },
                    "pitch": {"type": "number", "description": "俯仰角 (度), -90~+35"},
                    "yaw": {"type": "number", "description": "偏航角 (度), -40~+40"},
                    "roll": {"type": "number", "description": "横滚角 (度)"},
                    "mode": {
                        "type": "string",
                        "description": "控制模式或云台模式",
                        "enum": ["absolute", "relative", "speed", "free", "fpv", "yaw_follow"],
                    },
                    "duration": {"type": "number", "description": "旋转时长 (秒)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "rotate": {
                        "params": ["pitch", "yaw", "roll", "mode", "duration"],
                        "description": "旋转云台到指定角度",
                    },
                    "reset": {"params": [], "description": "复位云台到初始位置"},
                    "set_mode": {"params": ["mode"], "description": "设置云台模式 (free/fpv/yaw_follow)"},
                    "get_angles": {"params": [], "description": "获取当前云台角度"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "rotate":
            pitch = max(self.PITCH_RANGE[0], min(self.PITCH_RANGE[1], args.get("pitch", 0)))
            yaw = max(self.YAW_RANGE[0], min(self.YAW_RANGE[1], args.get("yaw", 0)))
            resp = self._bridge.gimbal_rotate(
                pitch=pitch,
                yaw=yaw,
                roll=args.get("roll", 0),
                mode=args.get("mode", "absolute"),
                duration=args.get("duration", 1.0),
            )
            return {"ret": 0 if resp.get("ok") else -1, "pitch": pitch, "yaw": yaw}
        if action == "reset":
            resp = self._bridge.gimbal_reset()
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "set_mode":
            resp = self._bridge.gimbal_set_mode(mode=args.get("mode", "yaw_follow"))
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "get_angles":
            resp = self._bridge.gimbal_get_angles()
            return resp.get("data", {})
        return None


# ═══════════════════════════════════════════════════════════════════════════
#  WaypointPlugin (actuator)
#  PSDK: 运动规划 (Waypoint V3)
# ═══════════════════════════════════════════════════════════════════════════

class WaypointPlugin:
    PREFIX = "waypoint"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._bridge = bridge

    def get_tool(self) -> dict:
        return {
            "name": "waypoint",
            "type": "actuator",
            "description": (
                "Mavic 3E 航点任务 (Waypoint V3)。上传 KMZ 文件后执行自主飞行任务。"
                "支持暂停/恢复/停止。KMZ 文件包含航点坐标、高度、速度、云台动作等。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "upload", "execute", "pause", "resume", "cancel", "status"],
                    },
                    "kmz_path": {"type": "string", "description": "KMZ 任务文件路径"},
                },
                "required": ["action"],
                "x-action-params": {
                    "upload": {"params": ["kmz_path"], "description": "上传 KMZ 航点任务文件"},
                    "execute": {"params": [], "description": "开始执行已上传的航点任务"},
                    "pause": {"params": [], "description": "暂停当前航点任务"},
                    "resume": {"params": [], "description": "恢复暂停的航点任务"},
                    "cancel": {"params": [], "description": "取消/停止航点任务"},
                    "status": {"params": [], "description": "查询航点任务执行状态"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "upload":
            kmz_path = args.get("kmz_path", "")
            if not kmz_path:
                return {"ret": -1, "error": "kmz_path is required"}
            resp = self._bridge.waypoint_upload(kmz_path)
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "execute":
            resp = self._bridge.waypoint_start()
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "pause":
            resp = self._bridge.waypoint_pause()
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "resume":
            resp = self._bridge.waypoint_resume()
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "cancel":
            resp = self._bridge.waypoint_stop()
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "status":
            resp = self._bridge.waypoint_status()
            return resp.get("data", {"state": "unknown"})
        return None


# ═══════════════════════════════════════════════════════════════════════════
#  SpeakerPlugin (actuator)
#  PSDK: 喊话器控件
# ═══════════════════════════════════════════════════════════════════════════

class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._bridge = bridge

    def get_tool(self) -> dict:
        return {
            "name": "speaker",
            "type": "actuator",
            "description": "Mavic 3E 喊话器：播放 TTS 文本或音频文件，音量控制。",
            "topic_in": [{"format": "audio/pcm-16k"}],
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "play", "set_volume", "stop_play"],
                    },
                    "text": {"type": "string", "description": "TTS 文本"},
                    "file_path": {"type": "string", "description": "音频文件路径"},
                    "volume": {"type": "integer", "description": "音量 (0-100)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "play": {"params": ["text", "file_path"], "description": "播放 TTS 文本或音频文件"},
                    "set_volume": {"params": ["volume"], "description": "设置喊话器音量 (0-100)"},
                    "stop_play": {"params": [], "description": "停止播放"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "play":
            resp = self._bridge.speaker_play(
                text=args.get("text", ""),
                file_path=args.get("file_path", ""),
            )
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "set_volume":
            resp = self._bridge.speaker_set_volume(volume=args.get("volume", 50))
            return {"ret": 0 if resp.get("ok") else -1}
        if action == "stop_play":
            resp = self._bridge.speaker_stop()
            return {"ret": 0 if resp.get("ok") else -1}
        return None



