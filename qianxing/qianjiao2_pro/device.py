"""MAVLink transport and MCP-facing tools for Qianjiao 2.0 Pro."""
from __future__ import annotations

import json
import subprocess
import socket
import struct
import threading
import urllib.error
import urllib.parse
import urllib.request
import ssl
import os
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
CONTROL_CARD = "control"


def _acp_notify(action_id: str | None, status: str, result: dict, tool: str = CONTROL_CARD) -> None:
    """Notify Agent Core when an asynchronous control action completes."""
    if not action_id:
        return
    url = __import__("os").environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                          "tool": tool, "ts": time.time()}).encode()
    request = urllib.request.Request(f"{url}/api/acp/complete", data=payload,
                                     headers={"Content-Type": "application/json"}, method="POST")
    try:
        urllib.request.urlopen(request, timeout=5,
                               context=ssl._create_unverified_context() if url.startswith("https://") else None).read()
    except Exception as exc:
        print(f"[Qianjiao ACP] callback failed for {action_id}: {exc}", flush=True)


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
        self._action_lock = threading.RLock()
        self._active_action: dict[str, Any] | None = None
        self._last_heartbeat = 0.0
        self._armed = False
        self.target_system = int(cfg.get("target_system", 1))
        self.target_component = int(cfg.get("target_component", 1))
        self._last_error: str | None = None
        self.status_port = int(cfg.get("status_port", 8500))
        self.camera_ip = str(cfg.get("camera_ip", "192.168.1.88"))
        self.camera_http_base = f"http://{self.camera_ip}:{int(cfg.get('camera_http_port', 80))}"
        camera_user = os.environ.get("QIANJIAO_CAMERA_USER", str(cfg.get("camera_user", "admin")))
        camera_password = os.environ.get("QIANJIAO_CAMERA_PASSWORD", str(cfg.get("camera_password", "")))
        credentials = f"{urllib.parse.quote(camera_user)}:{urllib.parse.quote(camera_password)}@" if camera_password else ""
        self.camera_rtsp = str(cfg.get("camera_rtsp", f"rtsp://{credentials}{self.camera_ip}:8554/stream/0/0"))
        self.camera_light_path = str(cfg.get("camera_light_path", "/v1/light"))
        self.video_url = str(cfg.get("video_url", "/video.mjpeg"))
        self.status_topic = str(cfg.get("status_topic", "/qianjiao2_pro/status"))
        self.loco_state_topic = str(cfg.get("loco_state_topic", "/qianjiao2_pro/loco_state"))
        self.battery_topic = str(cfg.get("battery_topic", "/qianjiao2_pro/battery"))
        self.imu_topic = str(cfg.get("imu_topic", "/qianjiao2_pro/imu"))
        self.camera_topic = str(cfg.get("camera_topic", "/qianjiao2_pro/camera/color"))
        self._status_sock: socket.socket | None = None
        self._status_thread: threading.Thread | None = None
        self._rov_status: dict[str, Any] = {}
        self._rov_status_received_at = 0.0
        self._rov_status_source: str | None = None
        self._ros_node = None
        self._ros_pub = None
        self._ros_loco_pub = None
        self._ros_battery_pub = None
        self._ros_imu_pub = None
        self._ros_camera_pub = None
        self._ros_thread: threading.Thread | None = None
        self._video_proc = None
        self._video_thread: threading.Thread | None = None
        self._video_cond = threading.Condition()
        self._video_frame = None
        self._video_ready = False
        self._video_error: str | None = None

    def _redacted_rtsp(self) -> str:
        return f"rtsp://{self.camera_ip}:8554/stream/0/0"

    def start_video_proxy(self):
        if self._video_thread and self._video_thread.is_alive():
            return
        self._video_thread = threading.Thread(target=self._video_loop, daemon=True, name="qianjiao-video-proxy")
        self._video_thread.start()

    def _video_loop(self):
        # Ubuntu 22.04 ships FFmpeg 4.4, which does not support the newer
        # ``-fps_mode`` option.  ``-vsync 0`` provides the same passthrough
        # behavior while keeping compatibility with that version.
        command = ["ffmpeg", "-loglevel", "error", "-rtsp_transport", "tcp", "-fflags", "+nobuffer", "-flags", "low_delay", "-analyzeduration", "1M", "-probesize", "1M", "-i", self.camera_rtsp, "-an", "-vf", "scale=1280:-2", "-vsync", "0", "-f", "mjpeg", "-q:v", "6", "pipe:1"]
        backoff = 1.0
        while not self._stop.is_set():
            buf = bytearray()
            try:
                self._video_proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                self._video_error = None
                stream = self._video_proc.stdout
                while not self._stop.is_set() and stream:
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    buf.extend(chunk)
                    while True:
                        start = buf.find(b"\xff\xd8")
                        if start < 0:
                            if len(buf) > 1:
                                del buf[:-1]
                            break
                        end = buf.find(b"\xff\xd9", start + 2)
                        if end < 0:
                            if start:
                                del buf[:start]
                            break
                        frame = bytes(buf[start:end + 2])
                        del buf[:end + 2]
                        with self._video_cond:
                            self._video_frame = frame
                            self._video_ready = True
                            self._video_cond.notify_all()
                        backoff = 1.0
                        if self._ros_camera_pub is not None and self._ros_node is not None:
                            from sensor_msgs.msg import CompressedImage
                            message = CompressedImage()
                            message.header.stamp = self._ros_node.get_clock().now().to_msg()
                            message.format = "jpeg"
                            message.data = frame
                            self._ros_camera_pub.publish(message)
                if self._video_proc.poll() is None:
                    self._video_proc.terminate()
                self._video_proc.wait(timeout=2)
            except Exception as exc:
                self._last_error = f"video proxy: {exc}"
                self._video_error = str(exc)
            finally:
                if self._video_proc is not None and self._video_proc.returncode not in (None, 0):
                    self._video_error = f"ffmpeg exited with code {self._video_proc.returncode}"
                self._video_proc = None
            if self._stop.is_set():
                break
            self._video_ready = False
            self._stop.wait(backoff)
            backoff = min(backoff * 2.0, 30.0)

    def get_video_frame(self, timeout=5.0):
        with self._video_cond:
            if self._video_frame is None: self._video_cond.wait(timeout)
            return self._video_frame

    def start_ros_status(self):
        """Publish vendor UDP status for Agent Core topic renderers."""
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CompressedImage
            from std_msgs.msg import String
            if not rclpy.ok():
                rclpy.init(args=None)
            self._ros_node = Node("qianjiao2_pro_status")
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.VOLATILE)
            self._ros_pub = self._ros_node.create_publisher(String, self.status_topic, qos)
            self._ros_loco_pub = self._ros_node.create_publisher(String, self.loco_state_topic, qos)
            self._ros_battery_pub = self._ros_node.create_publisher(String, self.battery_topic, qos)
            self._ros_imu_pub = self._ros_node.create_publisher(String, self.imu_topic, qos)
            self._ros_camera_pub = self._ros_node.create_publisher(CompressedImage, self.camera_topic, qos)
            self._ros_thread = threading.Thread(target=rclpy.spin, args=(self._ros_node,), daemon=True, name="qianjiao-status-ros")
            self._ros_thread.start()
        except Exception as exc:
            self._last_error = f"ROS status publisher: {exc}"

    def start(self):
        if self._thread and self._thread.is_alive():
            if self._ros_node is None:
                self.start_ros_status()
            self.start_video_proxy()
            return
        if not self.mock:
            if mavutil is None:
                raise RuntimeError("pymavlink is required for live mode")
            # The ROV learns the controller destination from incoming MAVLink
            # traffic when no mobile app is connected.  udpout initiates that
            # exchange; udpin would wait forever for a packet before it knows
            # where to send the heartbeat.
            self.link = mavutil.mavlink_connection(
                f"udpout:{self.target_ip}:{self.target_port}",
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
        if self._ros_node is None:
            self.start_ros_status()
        self.start_video_proxy()

    def stop(self):
        # Cancel an in-flight ACP action before closing transport resources.
        # This prevents a worker from sending a late propulsion command while
        # shutdown is tearing down the MAVLink link.
        with self._action_lock:
            active = self._active_action
            self._active_action = None
            if active:
                active["cancel_event"].set()
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self._status_thread:
            self._status_thread.join(timeout=2)
            self._status_thread = None
        if self.link is not None and hasattr(self.link, "close"):
            self.link.close()
        if self._status_sock:
            self._status_sock.close()
            self._status_sock = None
        if self._video_proc:
            self._video_proc.terminate()
            try:
                self._video_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._video_proc.kill()
        if self._video_thread:
            self._video_thread.join(timeout=3)
            self._video_thread = None
        if self._ros_node is not None:
            self._ros_node.destroy_node()
            self._ros_node = None
            self._ros_pub = None
            self._ros_loco_pub = None
            self._ros_battery_pub = None
            self._ros_imu_pub = None
            self._ros_camera_pub = None
        if self._ros_thread:
            self._ros_thread.join(timeout=2)
            self._ros_thread = None

    def _loop(self):
        while not self._stop.is_set():
            try:
                if self.mock:
                    self.link.heartbeat()
                else:
                    self.link.mav.heartbeat_send(11, 3, 0, 0, 4)
                    msg = self.link.recv_match(blocking=False)
                    while msg is not None:
                        if msg.get_type() == "HEARTBEAT":
                            self._last_heartbeat = time.monotonic()
                            src_system = msg.get_srcSystem()
                            src_component = msg.get_srcComponent()
                            if src_system is not None:
                                self.target_system = int(src_system)
                            if src_component is not None:
                                self.target_component = int(src_component)
                            self._armed = bool(int(getattr(msg, "base_mode", 0)) & MAV_MODE_FLAG_SAFETY_ARMED)
                        msg = self.link.recv_match(blocking=False)
            except Exception as exc:
                self._last_error = str(exc)
            self._stop.wait(1.0 / self.rate)

    def _connected(self) -> bool:
        last = self.link.last_heartbeat if self.mock else self._last_heartbeat
        return last > 0 and time.monotonic() - last <= self.timeout

    def _status_connected(self) -> bool:
        """Whether the vendor UDP status broadcast is fresh."""
        return bool(self._rov_status_received_at and
                    time.monotonic() - self._rov_status_received_at <= self.timeout)

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
                            msg.data = json.dumps(self._status_snapshot(parsed), ensure_ascii=False, separators=(",", ":"))
                            self._ros_pub.publish(msg)
                            if self._ros_loco_pub is not None:
                                msg.data = json.dumps(self._loco_snapshot(parsed), separators=(",", ":"))
                                self._ros_loco_pub.publish(msg)
                            if self._ros_battery_pub is not None:
                                msg.data = json.dumps(self._battery_snapshot(parsed), ensure_ascii=False, separators=(",", ":"))
                                self._ros_battery_pub.publish(msg)
                            if self._ros_imu_pub is not None:
                                msg.data = json.dumps(self._imu_snapshot(parsed), ensure_ascii=False, separators=(",", ":"))
                                self._ros_imu_pub.publish(msg)
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

    @staticmethod
    def _battery_snapshot(status: dict[str, Any]) -> dict[str, Any]:
        batteries = []
        for battery in status.get("batteries", []):
            if not isinstance(battery, dict):
                continue
            item = dict(battery)
            if isinstance(item.get("volt"), (int, float)):
                item["voltage_v"] = round(item["volt"] / 1000.0, 3)
            if isinstance(item.get("current"), (int, float)):
                item["current_a"] = round(item["current"] / 100.0, 2)
            if "remain" in item:
                item["remaining_percent"] = item["remain"]
            batteries.append(item)
        # The dashboard monitor renders top-level scalar fields.  Keep the
        # common (single-pack) ROV battery flat so it is readable instead of
        # displaying the nested ``batteries`` array as JSON text.
        if len(batteries) == 1:
            item = batteries[0]
            return {
                "battery_id": item.get("id"),
                "voltage_v": item.get("voltage_v"),
                "current_a": item.get("current_a"),
                "remaining_percent": item.get("remaining_percent"),
                "update_at": status.get("attUpdateAt"),
            }
        return {"battery_count": len(batteries), "batteries": batteries, "update_at": status.get("attUpdateAt")}

    @staticmethod
    def _imu_snapshot(status: dict[str, Any]) -> dict[str, Any]:
        raw = status.get("imu") if isinstance(status.get("imu"), dict) else {}
        angular_velocity = {
            axis: round(raw[axis] / 10.0, 1)
            for axis in ("gx", "gy", "gz") if isinstance(raw.get(axis), (int, float))
        }
        return {
            "angular_velocity_deg_s": angular_velocity,
            "raw_0_1_deg_s": raw,
            "attUpdateAt": status.get("attUpdateAt"),
        }

    @staticmethod
    def _loco_snapshot(status: dict[str, Any]) -> dict[str, Any]:
        return {key: status.get(key) for key in ("pitch", "roll", "yaw", "depth", "lat", "lon")}

    def _status_snapshot(self, status: dict[str, Any]) -> dict[str, Any]:
        age = time.monotonic() - self._rov_status_received_at if self._rov_status_received_at else None
        return {"connected": self._connected(), "status_connected": self._status_connected(), "temperature": status.get("temperature"),
                "status_age": round(age, 6) if age is not None else None,
                "status_age_ms": round(age * 1000, 1) if age is not None else None,
                "source": self._rov_status_source, "last_error": self._last_error,
                "video_ready": self._video_ready, "video_error": self._video_error,
                "mavlink_target": {"system": self.target_system, "component": self.target_component}}

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
            "status_connected": self._status_connected(),
            "armed": armed,
            "heartbeat_age": None if not self._connected() else round(time.monotonic() - (self.link.last_heartbeat if self.mock else self._last_heartbeat), 3),
            "transport": "mock" if self.mock else "mavlink-udp-v1",
            "endpoint": f"{self.target_ip}:{self.target_port}",
            "rov": self._rov_status,
            "status_port": self.status_port,
            "status_source": self._rov_status_source,
            "status_age": None if not self._rov_status_received_at else round(time.monotonic() - self._rov_status_received_at, 6),
            "camera": {"ip": self.camera_ip, "rtsp": self._redacted_rtsp()},
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
                self.link.last_command = {"command": MAV_CMD_COMPONENT_ARM_DISARM, "param1": 0 if armed else 1}
            else:
                # Vendor documentation: param1=0 unlocks, param1=1 locks.
                self.link.mav.command_long_send(self.target_system, self.target_component, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0 if armed else 1, 0, 0, 0, 0, 0, 0)
            return {"state": "armed" if armed else "disarmed", "command": MAV_CMD_COMPONENT_ARM_DISARM}

    def move(self, values: dict) -> dict:
        self._require_connected()
        if not self.mock and not self._is_armed_from_heartbeat():
            raise RuntimeError("ROV is locked; call unlock first")
        pwm = [_pwm(values.get(axis, 0.0)) for axis in CHANNELS]
        if "duration" not in values:
            raise ValueError("duration is required for move (-1 means continuous control)")
        duration = float(values["duration"])
        if duration != -1 and not 0.1 <= duration <= 60:
            raise ValueError("duration must be -1 (continuous) or between 0.1 and 60 seconds")

        def send(values_pwm):
            cancel_event = values.get("_cancel_event")
            action_id = values.get("_action_id")
            if cancel_event is not None:
                with self._action_lock:
                    owned = (not cancel_event.is_set() and self._active_action is not None
                             and self._active_action.get("action_id") == action_id)
                if not owned:
                    return False
            with self._lock:
                if self.mock:
                    self.link.last_rc = values_pwm
                else:
                    # MAVLink channels are 1-indexed; the vendor maps roll to
                    # channel 7 and leaves channel 6 unused.
                    channels = values_pwm[:5] + [UINT16_MAX, values_pwm[5], UINT16_MAX]
                    # MAVLink v1 RC_CHANNELS_OVERRIDE has exactly eight
                    # channel fields after target_system/component.
                    self.link.mav.rc_channels_override_send(self.target_system, self.target_component, *channels)
            return True

        send(pwm)
        cancel_event = values.get("_cancel_event")
        if duration == -1:
            while not self._stop.is_set() and not (cancel_event and cancel_event.is_set()):
                self._stop.wait(0.1)
                if not (cancel_event and cancel_event.is_set()) and not self._stop.is_set():
                    send(pwm)
            send([1500] * len(CHANNELS))
            return {"state": "stopped", "duration": -1, "channels": dict(zip(CHANNELS, pwm))}
        if duration != -1:
            deadline = time.monotonic() + duration
            while (time.monotonic() < deadline and not self._stop.is_set()
                   and not (cancel_event and cancel_event.is_set())):
                self._stop.wait(min(0.1, deadline - time.monotonic()))
                if (time.monotonic() < deadline and not (cancel_event and cancel_event.is_set())):
                    send(pwm)
            if not (cancel_event and cancel_event.is_set()):
                send([1500] * len(CHANNELS))
            return {"state": "stopped", "duration": duration, "channels": dict(zip(CHANNELS, pwm))}
        return {"state": "moving", "channels": dict(zip(CHANNELS, pwm))}

    def stop_motion(self) -> dict:
        with self._action_lock:
            active = self._active_action
            self._active_action = None
        if active:
            active["cancel_event"].set()
            _acp_notify(active["action_id"], "cancelled", {"reason": "stopped_by_user"})
        result = self.move({**{axis: 0 for axis in CHANNELS}, "duration": 0.1}) if self._connected() and (self.mock or self._is_armed_from_heartbeat()) else {"state": "stopped", "channels": {axis: 1500 for axis in CHANNELS}}
        return result

    def _is_armed_from_heartbeat(self) -> bool:
        return self._armed

    def _run_timed_move(self, args: dict, action_id: str) -> None:
        try:
            result = self.move(args)
            with self._action_lock:
                if self._active_action and self._active_action.get("action_id") == action_id:
                    self._active_action = None
                    _acp_notify(action_id, "completed", result)
        except Exception as exc:
            with self._action_lock:
                if self._active_action and self._active_action.get("action_id") == action_id:
                    self._active_action = None
            _acp_notify(action_id, "error", {"message": str(exc)})

    def _run_continuous_move(self, args: dict, action_id: str, cancel_event: threading.Event) -> None:
        try:
            self.move({**args, "_cancel_event": cancel_event})
            with self._action_lock:
                owned = bool(self._active_action and self._active_action.get("action_id") == action_id)
                if owned:
                    self._active_action = None
            if owned:
                _acp_notify(action_id, "cancelled", {"reason": "stopped_by_user"})
        except Exception as exc:
            with self._action_lock:
                if self._active_action and self._active_action.get("action_id") == action_id:
                    self._active_action = None
            _acp_notify(action_id, "error", {"message": str(exc)})

    def get_tools(self):
        sensor = lambda name, description, topic: {"name": name, "type": "sensor", "description": description, "topic_out": [{"topic": topic, "format": "data/json"}], "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["start", "stop", "info"], "description": "卡片生命周期或数据查询动作"}}, "required": ["action"]}}
        axis = lambda name, description: {"type": "number", "minimum": -1, "maximum": 1, "default": 0, "description": description}
        control_schema = {
            "type": "object",
            "properties": {"action": {"type": "string", "enum": ["start", "info", "unlock", "lock", "move", "stop", "cancel"], "description": "控制动作"}, "action_id": {"type": "string", "description": "ACP 异步动作标识，可选；不填写时由驱动生成"}, "heave": axis("heave", "升沉，范围 -1 到 1"), "pitch": axis("pitch", "俯仰，范围 -1 到 1"), "forward": axis("forward", "前后，范围 -1 到 1"), "yaw": axis("yaw", "偏航，范围 -1 到 1"), "lateral": axis("lateral", "横移，范围 -1 到 1"), "roll": axis("roll", "横滚，范围 -1 到 1"), "duration": {"type": "number", "minimum": -1, "maximum": 60, "description": "move 持续时间（秒）；-1 表示持续控制，0 无效，0.1-60 到时自动停止"}},
            "required": ["action"],
            "x-completion": {"actions": ["move"], "timeout": 61},
            "x-action-params": {
                "unlock": {"params": [], "description": "解锁推进器，允许运动控制"},
                "lock": {"params": [], "description": "锁定推进器，禁止运动控制"},
                "move": {"params": ["heave", "pitch", "forward", "yaw", "lateral", "roll", "duration"], "required": ["duration"], "description": "发送 6 自由度控制量；duration=-1 持续控制，0.1-60 秒后自动停止，未提供的轴默认为 0"},
                "stop": {"params": [], "description": "停止运动并将各轴归中"},
                "cancel": {"params": [], "description": "取消当前 ACP 运动任务并立即停止"},
                "start": {"params": [], "description": "初始化控制卡"},
                "info": {"params": [], "description": "读取控制链路状态"},
            },
        }
        return [
            sensor("loco_state", "潜蛟运动状态：姿态角、深度和定位信息。", self.loco_state_topic),
            sensor("status", "潜蛟系统状态：连接、温度和健康信息。", self.status_topic),
            sensor("battery", "潜蛟电池状态：电压、电流和剩余电量。", self.battery_topic),
            sensor("imu", "潜蛟 IMU 角速度数据。", self.imu_topic),
            {"name": "camera", "type": "sensor", "description": "潜蛟实时相机图像（RTSP 转 JPEG）。", "topic_out": [{"topic": self.camera_topic, "format": "image/jpeg"}], "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["start", "stop", "info"], "description": "相机流生命周期或信息查询"}}, "required": ["action"]}},
            {"name": "control", "type": "actuator", "description": "潜蛟 2.0 Pro 运动控制：解锁、停止或发送 6 自由度控制量。", "inputSchema": control_schema},
        ]

    def dispatch(self, tool: str, args: dict) -> dict:
        action = args.get("action", "info")
        sensor_tools = {
            "status": self.status_topic, "battery": self.battery_topic,
            "imu": self.imu_topic, "loco_state": self.loco_state_topic,
        }
        if tool in sensor_tools and action in ("start", "stop"):
            return {"state": "running" if action == "start" else "idle",
                    "topic_out": [{"topic": sensor_tools[tool], "format": "data/json"}]}
        if tool in ("camera", "rov_camera") and action in ("start", "stop"):
            return {"state": "running" if action == "start" else "idle",
                    "topic_out": [{"topic": self.camera_topic, "format": "image/jpeg"}]}
        if tool in ("status", "rov_status"):
            return {**self._status_snapshot(self._rov_status), "topic_out": [{"topic": self.status_topic, "format": "data/json"}]}
        if tool in ("camera", "rov_camera"):
            return {"state": "available" if self._video_ready else "unavailable",
                    "topic_out": [{"topic": self.camera_topic, "format": "image/jpeg"}],
                    "stream_url": self.video_url,
                    "source_rtsp": f"rtsp://{self.camera_ip}:8554/stream/0/0",
                    "error": self._video_error}
        if tool == "battery":
            return {**self._battery_snapshot(self._rov_status), "topic_out": [{"topic": self.battery_topic, "format": "data/json"}]}
        if tool == "imu":
            return {**self._imu_snapshot(self._rov_status), "topic_out": [{"topic": self.imu_topic, "format": "data/json"}]}
        if tool == "loco_state":
            return {**self._loco_snapshot(self._rov_status), "topic_out": [{"topic": self.loco_state_topic, "format": "data/json"}]}
        if tool == "control" and action in ("start", "info"): return {"state": "ready", "mavlink_connected": self._connected()}
        if tool == "control" and action == "stop": return self.stop_motion()
        if tool == "control" and action == "cancel": return self.stop_motion()
        if tool == "control" and action == "unlock": return self.arm(True)
        if tool == "control" and action == "lock":
            self.stop_motion()
            return self.arm(False)
        if tool == "control" and action == "move":
            action_id = str(args.get("action_id") or f"qianjiao_move_{int(time.time() * 1000)}")
            duration = float(args["duration"])
            if duration == -1:
                cancel_event = threading.Event()
                with self._action_lock:
                    previous = self._active_action
                    self._active_action = {"action_id": action_id, "cancel_event": cancel_event}
                if previous:
                    previous["cancel_event"].set()
                    _acp_notify(previous["action_id"], "cancelled", {"reason": "replaced_by_new_command"})
                move_args = {**args, "_cancel_event": cancel_event, "_action_id": action_id}
                threading.Thread(target=self._run_continuous_move, args=(move_args, action_id, cancel_event), daemon=True, name="qianjiao-move-acp").start()
                return {"state": "running", "action_id": action_id, "stops_automatically": False}
            with self._action_lock:
                previous = self._active_action
                cancel_event = threading.Event()
                self._active_action = {"action_id": action_id, "cancel_event": cancel_event}
            if previous:
                previous["cancel_event"].set()
                _acp_notify(previous["action_id"], "cancelled", {"reason": "replaced_by_new_command"})
            args = {**args, "_cancel_event": cancel_event, "_action_id": action_id}
            threading.Thread(target=self._run_timed_move, args=(args, action_id), daemon=True, name="qianjiao-move-acp").start()
            return {"state": "queued", "action_id": action_id, "stops_automatically": True, "duration": duration}
        raise ValueError(f"unsupported action: {action}")
