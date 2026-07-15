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
from sensor_msgs.msg import CompressedImage

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
        self._pub = self.create_publisher(CompressedImage, topic, _LOW_LAT_QOS)
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
        """Read JPEG frames from /dev/shm (FFmpeg decoded in C bridge)."""
        import os

        frame_path = "/dev/shm/dji_frame.jpg"
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
                            {"const": "wide", "title": "Wide (广角 24mm)"},
                            {"const": "zoom", "title": "Zoom (变焦 162mm)"},
                            {"const": "ir", "title": "IR Thermal (红外，仅3T)"},
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
            camera = args.get("camera_source", "wide")
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
        cfg = self._instance_configs.get(instance_id, {})
        camera = args.get("camera_source") or cfg.get("camera_source", "wide")

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
                node.start()
            else:
                safe_id = instance_id.replace("-", "_")
                topic = f"/{self._namespace}/camera/{safe_id}/rgb"
                node = _CameraStreamNode(topic, self._bridge, self._fps, camera)
                self._executor.add_node(node)
                self._nodes[instance_id] = node
                node.start()
            return {"state": "running", "camera": camera}
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
                "description": "Mavic 3T 飞行控制。安全提示：SDK 控制期间遥控器摇杆无效，切换档位(T/P/S)可立即夺回控制权。",
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
            # Clamp velocities to Mavic 3T limits
            vx = max(-15, min(15, float(args.get("vx", 0))))
            vy = max(-15, min(15, float(args.get("vy", 0))))
            vz = max(-6, min(6, float(args.get("vz", 0))))
            vyaw = max(-75, min(75, float(args.get("vyaw", 0))))
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
                    "ir_temp_area": {
                        "params": ["ltx", "lty", "rbx", "rby"],
                        "description": "红外区域测温，坐标范围0-1 (左上角ltx,lty 右下角rbx,rby)",
                    },
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
            resp = self._bridge.ir_temp_point(
                x=args.get("point_x", 0.5), y=args.get("point_y", 0.5),
            )
            return {"ret": 0 if resp.get("ok") else -1, "data": resp.get("data", {})}
        if action == "ir_temp_area":
            resp = self._bridge.ir_temp_area(
                ltx=args.get("ltx", 0.25), lty=args.get("lty", 0.25),
                rbx=args.get("rbx", 0.75), rby=args.get("rby", 0.75),
            )
            return {"ret": 0 if resp.get("ok") else -1, "data": resp.get("data", {})}
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

    WAYPOINT_DIR = "/opt/phanthy-motus/data/waypoints"

    def __init__(self, plugin_config: dict, namespace: str, executor, bridge):
        self._bridge = bridge
        self._record_thread = None
        self._record_active = False
        self._record_points = []
        self._record_name = ""
        self._mark_points = []
        self._mark_name = ""
        self._mark_active = False
        import os
        os.makedirs(self.WAYPOINT_DIR, exist_ok=True)

    def get_tool(self) -> dict:
        return {
            "name": "waypoint",
            "type": "actuator",
            "description": (
                "Waypoint mission: record GPS track or mark key points → generate KMZ → auto-fly. "
                "Safety: switch RC mode to override at any time."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "start", "stop",
                            "record_start", "record_stop",
                            "mark_start", "mark_point", "mark_stop",
                            "list", "load", "execute",
                            "pause", "resume", "cancel", "status",
                        ],
                    },
                    "name": {"type": "string", "description": "Mission name (for record/mark/load)"},
                    "tag": {"type": "string", "description": "Optional tag for mission or point"},
                    "speed": {"type": "number", "description": "Flight speed (m/s), -1=use recorded speed", "default": 5},
                    "return_home": {"type": "boolean", "description": "Return to start point after mark_stop", "default": True},
                },
                "required": ["action"],
                "x-action-params": {
                    "record_start": {"params": ["name", "tag"], "description": "Start recording GPS track"},
                    "record_stop": {"params": [], "description": "Stop recording, save as KMZ"},
                    "mark_start": {"params": ["name", "tag"], "description": "Start marking key points"},
                    "mark_point": {"params": ["tag"], "description": "Mark current position as waypoint"},
                    "mark_stop": {"params": ["return_home"], "description": "Stop marking, save as KMZ"},
                    "list": {"params": [], "description": "List all saved waypoint missions"},
                    "load": {"params": ["name", "speed"], "description": "Load mission and upload to aircraft"},
                    "execute": {"params": [], "description": "Execute uploaded mission"},
                    "pause": {"params": [], "description": "Pause mission"},
                    "resume": {"params": [], "description": "Resume mission"},
                    "cancel": {"params": [], "description": "Cancel mission"},
                    "status": {"params": [], "description": "Query mission status"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        self._record_active = False

    # ── GPS helper ────────────────────────────────────────────────────

    def _get_current_gps(self) -> dict | None:
        """Get current GPS + velocity from bridge telemetry."""
        resp = self._bridge.get_telemetry()
        if not resp.get("ok"):
            return None
        data = resp.get("data", {})
        pos = data.get("position", {})
        vel = data.get("velocity", {})
        lat = pos.get("latitude")
        lon = pos.get("longitude")
        alt = pos.get("altitude")
        if lat is None or lon is None:
            return None
        # Compute horizontal speed
        import math
        vx = vel.get("vx", 0)
        vy = vel.get("vy", 0)
        speed = math.sqrt(vx * vx + vy * vy)
        return {"lat": lat, "lon": lon, "alt": alt or 0, "speed": round(speed, 1)}

    # ── KMZ generation ────────────────────────────────────────────────

    def _generate_kmz(self, waypoints: list, name: str, speed: float = 5.0,
                      finish_action: str = "goHome") -> str:
        """Generate KMZ file from waypoint list. Returns file path.
        speed=-1 means use per-point recorded speed."""
        import zipfile
        import os

        if not waypoints or len(waypoints) < 2:
            return ""

        # Determine effective speed
        if speed <= 0:
            # Use average recorded speed, fallback to 5
            speeds = [wp.get("speed", 0) for wp in waypoints if wp.get("speed", 0) > 0.5]
            eff_speed = sum(speeds) / len(speeds) if speeds else 5.0
        else:
            eff_speed = speed

        # Build waypoints XML
        wp_xml_parts = []
        for i, wp in enumerate(waypoints):
            wp_speed = wp.get("speed", eff_speed) if speed <= 0 else eff_speed
            wp_speed = max(1.0, wp_speed)  # minimum 1 m/s
        for i, wp in enumerate(waypoints):
            wp_xml_parts.append(f"""      <Placemark>
        <Point>
          <coordinates>{wp['lon']},{wp['lat']}</coordinates>
        </Point>
        <wpml:index>{i}</wpml:index>
        <wpml:executeHeight>{wp.get('alt', 0):.1f}</wpml:executeHeight>
        <wpml:waypointSpeed>{wp_speed:.1f}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
      </Placemark>""")

        waypoints_xml = "\n".join(wp_xml_parts)

        template_kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"
     xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
  <Document>
    <wpml:author>PhanthyMotus</wpml:author>
    <wpml:createTime>{int(time.time() * 1000)}</wpml:createTime>
    <wpml:updateTime>{int(time.time() * 1000)}</wpml:updateTime>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineCoordinateSysParam>
        <wpml:coordinateMode>WGS84</wpml:coordinateMode>
        <wpml:heightMode>relativeToStartPoint</wpml:heightMode>
      </wpml:waylineCoordinateSysParam>
      <wpml:autoFlightSpeed>{eff_speed:.1f}</wpml:autoFlightSpeed>
      <Placemark>
        <wpml:missionConfig>
          <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
          <wpml:finishAction>{finish_action}</wpml:finishAction>
          <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
          <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
          <wpml:globalTransitionalSpeed>{eff_speed:.1f}</wpml:globalTransitionalSpeed>
          <wpml:droneInfo>
            <wpml:droneEnumValue>89</wpml:droneEnumValue>
            <wpml:droneSubEnumValue>0</wpml:droneSubEnumValue>
          </wpml:droneInfo>
        </wpml:missionConfig>
      </Placemark>
    </Folder>
  </Document>
</kml>"""

        waylines_wpml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"
     xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
  <Document>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:autoFlightSpeed>{eff_speed:.1f}</wpml:autoFlightSpeed>
{waypoints_xml}
    </Folder>
  </Document>
</kml>"""

        timestamp = int(time.time())
        safe_name = name.replace(" ", "_").replace("'", "").replace('"', '')
        filename = f"{safe_name}_{timestamp}.kmz"
        filepath = os.path.join(self.WAYPOINT_DIR, filename)

        with zipfile.ZipFile(filepath, "w", zipfile.ZIP_DEFLATED) as zf:
            zf.writestr("wpmz/template.kml", template_kml)
            zf.writestr("wpmz/waylines.wpml", waylines_wpml)

        return filepath

    # ── Record mode ───────────────────────────────────────────────────

    def _record_loop(self):
        """Background thread: sample GPS every 1s, skip if < 1m from last."""
        import math
        last_lat, last_lon = None, None

        while self._record_active:
            gps = self._get_current_gps()
            if gps:
                if last_lat is not None:
                    dlat = (gps["lat"] - last_lat) * 111320
                    dlon = (gps["lon"] - last_lon) * 111320 * math.cos(math.radians(gps["lat"]))
                    dist = math.sqrt(dlat * dlat + dlon * dlon)
                    if dist < 1.0:
                        time.sleep(1)
                        continue
                last_lat, last_lon = gps["lat"], gps["lon"]
                self._record_points.append(gps)
                if len(self._record_points) % 30 == 1:
                    print(f"[waypoint] recording... {len(self._record_points)} points")
            time.sleep(1)

    # ── Dispatch ──────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict | None:
        import os
        import glob

        if action in ("start", "stop"):
            return {"state": "ready" if action == "start" else "idle"}

        # ── Record ──
        if action == "record_start":
            name = args.get("name", "track")
            tag = args.get("tag", "")
            if self._record_active:
                return {"ret": -1, "error": "Recording already in progress"}
            self._record_points = []
            self._record_name = f"{name}_{tag}" if tag else name
            self._record_active = True
            self._record_thread = threading.Thread(target=self._record_loop, daemon=True)
            self._record_thread.start()
            return {"ret": 0, "message": f"Recording started: {self._record_name}"}

        if action == "record_stop":
            if not self._record_active:
                return {"ret": -1, "error": "No recording in progress"}
            self._record_active = False
            if self._record_thread:
                self._record_thread.join(timeout=3)
            # Always capture final position
            gps = self._get_current_gps()
            if gps:
                self._record_points.append(gps)
            points = self._record_points
            if len(points) == 0:
                return {"ret": -1, "error": "No GPS data captured"}
            if len(points) == 1:
                # Didn't move — duplicate with slight offset so KMZ is valid
                p = dict(points[0])
                p["lat"] += 0.00001  # ~1m offset
                points.append(p)
            filepath = self._generate_kmz(points, self._record_name)
            return {"ret": 0, "message": f"Recorded {len(points)} points",
                    "file": filepath, "points": len(points)}

        # ── Mark ──
        if action == "mark_start":
            name = args.get("name", "route")
            tag = args.get("tag", "")
            if self._mark_active:
                return {"ret": -1, "error": "Marking already in progress"}
            self._mark_points = []
            self._mark_name = f"{name}_{tag}" if tag else name
            self._mark_active = True
            # Record start point
            gps = self._get_current_gps()
            if gps:
                gps["tag"] = "start"
                self._mark_points.append(gps)
            return {"ret": 0, "message": f"Marking started: {self._mark_name}",
                    "start_point": gps}

        if action == "mark_point":
            if not self._mark_active:
                return {"ret": -1, "error": "Marking not started"}
            gps = self._get_current_gps()
            if not gps:
                return {"ret": -1, "error": "GPS not available"}
            tag = args.get("tag", f"point_{len(self._mark_points)}")
            gps["tag"] = tag
            self._mark_points.append(gps)
            return {"ret": 0, "message": f"Point #{len(self._mark_points)} marked: {tag}",
                    "point": gps, "total": len(self._mark_points)}

        if action == "mark_stop":
            if not self._mark_active:
                return {"ret": -1, "error": "Marking not started"}
            self._mark_active = False
            return_home = args.get("return_home", True)
            if isinstance(return_home, str):
                return_home = return_home.lower() not in ("false", "0", "no")
            points = self._mark_points
            if return_home and len(points) >= 1:
                # Add start point as last waypoint
                home = dict(points[0])
                home["tag"] = "return_home"
                points.append(home)
            if len(points) < 2:
                return {"ret": -1, "error": f"Too few points ({len(points)}), need >= 2"}
            filepath = self._generate_kmz(points, self._mark_name)
            return {"ret": 0, "message": f"Saved {len(points)} waypoints",
                    "file": filepath, "points": len(points)}

        # ── Path management ──
        if action == "list":
            pattern = os.path.join(self.WAYPOINT_DIR, "*.kmz")
            files = sorted(glob.glob(pattern))
            missions = [os.path.basename(f) for f in files]
            return {"ret": 0, "missions": missions, "count": len(missions)}

        if action == "load":
            name = args.get("name", "")
            speed = float(args.get("speed", 5))
            if not name:
                return {"ret": -1, "error": "name is required"}
            # Find matching file
            pattern = os.path.join(self.WAYPOINT_DIR, f"{name}*")
            files = sorted(glob.glob(pattern))
            if not files:
                return {"ret": -1, "error": f"Mission not found: {name}"}
            kmz_path = files[-1]  # latest match
            resp = self._bridge.waypoint_upload(kmz_path)
            if resp.get("ok"):
                return {"ret": 0, "message": f"Loaded: {os.path.basename(kmz_path)}",
                        "file": kmz_path, "speed": speed}
            return {"ret": -1, "error": "Upload failed", "data": resp.get("data", {})}

        # ── Mission control ──
        if action == "execute":
            resp = self._bridge.waypoint_start()
            return {"ret": 0 if resp.get("ok") else -1,
                    "data": resp.get("data", {})}
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

# ═══════════════════════════════════════════════════════════════════════════
#  TimeSyncPlugin (actuator)
#  PSDK: 机型信息 + 时间同步
# ═══════════════════════════════════════════════════════════════════════════

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



