"""MAVLink transport and MCP-facing tools for Qianjiao 2.0 Pro."""
from __future__ import annotations

import json
import socket
import struct
import threading
import urllib.error
import urllib.parse
import urllib.request
import time
from typing import Any

try:
    from pymavlink import mavutil
except ImportError:  # importable in development without the optional SDK
    mavutil = None

UINT16_MAX = 65535
CHANNELS = ("heave", "pitch", "forward", "yaw", "lateral", "roll")
COMMAND_LONG = 76
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_MODE_FLAG_SAFETY_ARMED = 128


def _pwm(value: Any) -> int:
    value = float(value)
    if not -1.0 <= value <= 1.0:
        raise ValueError("axis values must be in [-1, 1]")
    return int(round(1500 + value * 400))


class MockLink:
    def __init__(self):
        self.armed = False
        self.last_rc = [1500] * 6
        self.last_command: dict[str, Any] | None = None
        self.last_heartbeat = time.monotonic()

    def heartbeat(self):
        self.last_heartbeat = time.monotonic()


class QianjiaoDevice:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.target_ip = str(cfg.get("target_ip", "192.168.1.101"))
        self.target_port = int(cfg.get("target_port", 14550))
        self.timeout = float(cfg.get("heartbeat_timeout", 3.0))
        self.rate = max(0.2, float(cfg.get("heartbeat_rate", 1.0)))
        self.mock = bool(cfg.get("mock", False))
        self.link: Any = MockLink() if self.mock else None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.RLock()
        self._last_heartbeat = 0.0
        self._armed = False
        self._last_error: str | None = None
        self.status_port = int(cfg.get("status_port", 8500))
        self.camera_ip = str(cfg.get("camera_ip", "192.168.1.88"))
        self.camera_http_base = f"http://{self.camera_ip}:{int(cfg.get('camera_http_port', 80))}"
        self.camera_rtsp = str(cfg.get("camera_rtsp", f"rtsp://admin:admin@{self.camera_ip}:8554/stream/0/0"))
        self.camera_light_path = str(cfg.get("camera_light_path", "/v1/light"))
        self._status_sock: socket.socket | None = None
        self._status_thread: threading.Thread | None = None
        self._rov_status: dict[str, Any] = {}
        self._rov_status_received_at = 0.0
        self._rov_status_source: str | None = None
        self._ros_node = None
        self._ros_pub = None

    def start_ros_status(self):
        """Publish vendor UDP status for Agent Core topic renderers."""
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            if not rclpy.ok():
                rclpy.init(args=None)
            self._ros_node = Node("qianjiao2_pro_status")
            self._ros_pub = self._ros_node.create_publisher(String, "/qianjiao2_pro/status", 10)
            threading.Thread(target=rclpy.spin, args=(self._ros_node,), daemon=True, name="qianjiao-status-ros").start()
        except Exception as exc:
            self._last_error = f"ROS status publisher: {exc}"

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        if not self.mock:
            if mavutil is None:
                raise RuntimeError("pymavlink is required for live mode")
            # udpin binds locally.  ``udp:<remote>`` would incorrectly try to
            # bind the board's socket to the remote ROV address (192.168.1.101).
            self.link = mavutil.mavlink_connection(
                f"udpin:0.0.0.0:{self.target_port}",
                source_system=int(self.cfg.get("source_system", 255)),
                source_component=int(self.cfg.get("source_component", 190)),
                mavlink20=False,
                dialect="ardupilotmega",
            )
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="qianjiao-mavlink")
        self._thread.start()
        self._status_thread = threading.Thread(target=self._status_loop, daemon=True, name="qianjiao-status")
        self._status_thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self.link is not None and hasattr(self.link, "close"):
            self.link.close()
        if self._status_sock:
            self._status_sock.close()
            self._status_sock = None
        if self._ros_node is not None:
            self._ros_node.destroy_node()
            self._ros_node = None

    def _loop(self):
        while not self._stop.is_set():
            try:
                if self.mock:
                    self.link.heartbeat()
                else:
                    # mavudp learns the peer address from the first packet.
                    # Do not attempt sendto(None) before that packet arrives.
                    if getattr(self.link, "address", None):
                        self.link.mav.heartbeat_send(11, 3, 0, 0, 4)
                    msg = self.link.recv_match(blocking=False)
                    while msg is not None:
                        if msg.get_type() == "HEARTBEAT":
                            self._last_heartbeat = time.monotonic()
                            self._armed = bool(int(getattr(msg, "base_mode", 0)) & MAV_MODE_FLAG_SAFETY_ARMED)
                        msg = self.link.recv_match(blocking=False)
            except Exception as exc:
                self._last_error = str(exc)
            self._stop.wait(1.0 / self.rate)

    def _connected(self) -> bool:
        last = self.link.last_heartbeat if self.mock else self._last_heartbeat
        return last > 0 and time.monotonic() - last <= self.timeout

    def _status_loop(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", self.status_port))
            sock.settimeout(1.0)
            self._status_sock = sock
            while not self._stop.is_set():
                try:
                    packet, peer = sock.recvfrom(65535)
                    parsed = self._parse_status_packet(packet)
                    if parsed is not None:
                        self._rov_status = parsed
                        self._rov_status_received_at = time.monotonic()
                        self._rov_status_source = f"{peer[0]}:{peer[1]}"
                        if self._ros_pub is not None:
                            from std_msgs.msg import String
                            msg = String()
                            msg.data = json.dumps(parsed, ensure_ascii=False, separators=(",", ":"))
                            self._ros_pub.publish(msg)
                except socket.timeout:
                    continue
        except Exception as exc:
            self._last_error = f"status UDP: {exc}"

    @staticmethod
    def _parse_status_packet(packet: bytes) -> dict[str, Any] | None:
        # Vendor sample is little-endian: id/version/reserved, uint32 length,
        # type/reserved, then UTF-8 JSON payload.
        if len(packet) < 12 or packet[0] != 0x03 or packet[8] != 0x01:
            return None
        payload_len = struct.unpack_from("<I", packet, 4)[0]
        if payload_len > len(packet) - 12:
            return None
        try:
            value = json.loads(packet[12:12 + payload_len].decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return None
        return value if isinstance(value, dict) else None

    def camera_request(self, method: str, path: str, body: Any = None) -> dict:
        url = self.camera_http_base + path
        data = None if body is None else json.dumps(body).encode("utf-8")
        req = urllib.request.Request(url, data=data, method=method, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as response:
                raw = response.read()
                try:
                    result = json.loads(raw.decode("utf-8")) if raw else {}
                except (UnicodeDecodeError, json.JSONDecodeError):
                    result = {"bytes": len(raw), "content_type": response.headers.get_content_type()}
                return {"http_status": response.status, "data": result}
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"camera HTTP {exc.code}: {exc.reason}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"camera unavailable: {exc.reason}") from exc

    def status(self) -> dict:
        armed = bool(self.link.armed) if self.mock else self._armed
        return {
            "connected": self._connected(),
            "armed": armed,
            "heartbeat_age": None if not self._connected() else round(time.monotonic() - (self.link.last_heartbeat if self.mock else self._last_heartbeat), 3),
            "transport": "mock" if self.mock else "mavlink-udp-v1",
            "endpoint": f"{self.target_ip}:{self.target_port}",
            "rov": self._rov_status,
            "status_port": self.status_port,
            "status_source": self._rov_status_source,
            "status_age": None if not self._rov_status_received_at else round(time.monotonic() - self._rov_status_received_at, 3),
            "camera": {"ip": self.camera_ip, "rtsp": self.camera_rtsp},
            "last_error": self._last_error,
        }

    def _require_connected(self):
        if not self._connected():
            raise RuntimeError("ROV heartbeat not received (connection timeout)")

    def arm(self, armed: bool) -> dict:
        self._require_connected()
        with self._lock:
            if self.mock:
                self.link.armed = armed
                self.link.last_command = {"command": MAV_CMD_COMPONENT_ARM_DISARM, "param1": int(armed)}
            else:
                self.link.mav.command_long_send(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, int(armed), 0, 0, 0, 0, 0, 0)
            return {"state": "armed" if armed else "disarmed", "command": MAV_CMD_COMPONENT_ARM_DISARM}

    def move(self, values: dict) -> dict:
        self._require_connected()
        if not self.mock and not self._is_armed_from_heartbeat():
            raise RuntimeError("ROV is not armed; call arm first")
        pwm = [_pwm(values.get(axis, 0.0)) for axis in CHANNELS]
        with self._lock:
            if self.mock:
                self.link.last_rc = pwm
            else:
                # MAVLink channels are 1-indexed; the vendor maps roll to
                # channel 7 and leaves channel 6 unused.
                channels = pwm[:5] + [UINT16_MAX, pwm[5], UINT16_MAX]
                args = [1, 1] + channels + [UINT16_MAX] * 10
                self.link.mav.rc_channels_override_send(*args)
        return {"state": "moving", "channels": dict(zip(CHANNELS, pwm))}

    def stop_motion(self) -> dict:
        return self.move({axis: 0 for axis in CHANNELS}) if self._connected() and (self.mock or self._is_armed_from_heartbeat()) else {"state": "stopped", "channels": {axis: 1500 for axis in CHANNELS}}

    def _is_armed_from_heartbeat(self) -> bool:
        return self._armed

    def get_tools(self):
        return [
            {"name": "rov_status", "type": "sensor", "description": "潜蛟实时状态：姿态、深度、位置、温度、电池和陀螺仪（UDP 8500，10Hz）", "topic_out": [{"topic": "/qianjiao2_pro/status", "format": "data/json"}], "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"]}}}},
            {"name": "rov_camera", "type": "sensor", "description": "潜蛟实时视频流（RTSP，需支持 RTSP 的播放器）", "topic_out": [{"topic": "rtsp://admin:admin@192.168.1.88:8554/stream/0/0", "format": "video/rtsp", "external": True}], "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["info"]}}}},
            {"name": "rov_camera_control", "type": "actuator", "description": "潜蛟相机控制：拍照、媒体列表、下载和补光灯", "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["capture", "medias", "download", "light"]}, "name": {"type": "string"}, "brightness": {"type": "integer", "minimum": 0, "maximum": 100}}, "required": ["action"]}},
            {"name": "rov_control", "type": "actuator", "description": "潜蛟 2.0 Pro 解锁及 6DOF 运动控制", "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["arm", "disarm", "move", "stop"]}, "heave": {"type": "number", "minimum": -1, "maximum": 1}, "pitch": {"type": "number", "minimum": -1, "maximum": 1}, "forward": {"type": "number", "minimum": -1, "maximum": 1}, "yaw": {"type": "number", "minimum": -1, "maximum": 1}, "lateral": {"type": "number", "minimum": -1, "maximum": 1}, "roll": {"type": "number", "minimum": -1, "maximum": 1}}, "required": ["action"]}},
        ]

    def dispatch(self, tool: str, args: dict) -> dict:
        action = args.get("action", "info")
        if tool == "rov_status":
            if action == "start": self.start()
            elif action == "stop": self.stop()
            return self.status()
        if tool == "rov_camera":
            return {"state": "available", "stream_url": self.camera_rtsp}
        if tool == "rov_camera_control":
            if action == "capture":
                return self.camera_request("POST", "/v1/capture")
            if action == "medias":
                return self.camera_request("GET", "/v1/medias")
            if action == "download":
                name = str(args.get("name", "")).strip()
                if not name or "/" in name or ".." in name:
                    raise ValueError("name must be a media filename")
                return {"download_url": f"{self.camera_http_base}/v1/medias/{urllib.parse.quote(name)}/download"}
            if action == "light":
                brightness = int(args.get("brightness", 0))
                return self.camera_request("POST", self.camera_light_path, {"brightness": brightness})
            raise ValueError(f"unsupported camera action: {action}")
        if action == "arm": return self.arm(True)
        if action == "disarm": return self.arm(False)
        if action == "move": return self.move(args)
        if action == "stop": return self.stop_motion()
        raise ValueError(f"unsupported action: {action}")
